// Commander X16 Emulator
// GDB Remote Serial Protocol stub
// License: 2-clause BSD

#include "gdbstub.h"
#include "glue.h"
#include "memory.h"
#include "video.h"
#include "cpu/fake6502.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
typedef SOCKET socket_t;
#define SOCKET_INVALID INVALID_SOCKET
#define CLOSE_SOCKET closesocket
#define SOCK_ERRNO WSAGetLastError()
#define WOULD_BLOCK (SOCK_ERRNO == WSAEWOULDBLOCK)
static bool wsa_initialized = false;
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
typedef int socket_t;
#define SOCKET_INVALID (-1)
#define CLOSE_SOCKET close
#define SOCK_ERRNO errno
#define WOULD_BLOCK (SOCK_ERRNO == EAGAIN || SOCK_ERRNO == EWOULDBLOCK)
#endif

// -------------------------------------------------------------------
// Globals
// -------------------------------------------------------------------

bool gdb_enabled = false;
bool gdb_connected = false;

// -------------------------------------------------------------------
// Constants
// -------------------------------------------------------------------

#define GDB_MAX_BREAKPOINTS 32
#define GDB_PKT_BUF_SIZE   4096
#define GDB_RECV_BUF_SIZE  4096

// Extended address map for memory commands (m/M):
//   $00000000-$00FFFFFF  Banked: bank=bits[23:16], cpu_addr=bits[15:0]
//   $01000000-$0101FFFF  VERA VRAM (128KB)
#define GDB_VRAM_BASE   0x01000000UL
#define GDB_VRAM_END    0x01020000UL

// -------------------------------------------------------------------
// State machine
// -------------------------------------------------------------------

typedef enum {
	GDB_STATE_LISTENING,  // waiting for connection
	GDB_STATE_STOPPED,    // connected, CPU halted
	GDB_STATE_RUNNING,    // connected, CPU running
	GDB_STATE_DETACHED,   // was connected, now disconnected
} gdb_state_t;

static gdb_state_t gdb_state = GDB_STATE_LISTENING;

// -------------------------------------------------------------------
// Sockets
// -------------------------------------------------------------------

static socket_t listen_sock = SOCKET_INVALID;
static socket_t client_sock = SOCKET_INVALID;

// -------------------------------------------------------------------
// Receive buffer
// -------------------------------------------------------------------

static uint8_t recv_buf[GDB_RECV_BUF_SIZE];
static int recv_len = 0;

// -------------------------------------------------------------------
// Breakpoints
// -------------------------------------------------------------------

struct gdb_breakpoint {
	uint16_t addr;
	uint8_t  bank;      // bank number from GDB 24-bit address bits[23:16]
	bool     banked;    // true if addr >= $A000 (bank-sensitive region)
	bool     active;
};

static struct gdb_breakpoint breakpoints[GDB_MAX_BREAKPOINTS];
static int num_breakpoints = 0;

// -------------------------------------------------------------------
// Range stepping (vCont;r)
// -------------------------------------------------------------------

static bool range_stepping = false;
static uint16_t range_step_start = 0;
static uint16_t range_step_end = 0;

// -------------------------------------------------------------------
// Watchpoints (Z2/Z3/Z4)
// -------------------------------------------------------------------

#define GDB_MAX_WATCHPOINTS 16

typedef enum { WP_WRITE = 2, WP_READ = 3, WP_ACCESS = 4 } wp_type_t;

struct gdb_watchpoint {
	uint32_t  addr;     // 24-bit (bank:cpuaddr), same encoding as m/M packets
	uint32_t  len;      // byte count of watched region
	wp_type_t type;
	bool      active;
};

static struct gdb_watchpoint watchpoints[GDB_MAX_WATCHPOINTS];
static int num_watchpoints = 0;
bool gdb_watchpoints_active = false;
static bool wp_hit = false;
static uint32_t wp_hit_addr = 0;
static wp_type_t wp_hit_type = WP_WRITE;

// -------------------------------------------------------------------
// Bank helpers
// -------------------------------------------------------------------

static uint8_t
get_current_bank_for_addr(uint16_t addr)
{
	if (addr >= 0xC000) return memory_get_rom_bank();
	if (addr >= 0xA000) return memory_get_ram_bank();
	return 0;
}

// -------------------------------------------------------------------
// Helpers: hex encoding
// -------------------------------------------------------------------

static const char hex_chars[] = "0123456789abcdef";

static int
hex_digit(char c)
{
	if (c >= '0' && c <= '9') return c - '0';
	if (c >= 'a' && c <= 'f') return c - 'a' + 10;
	if (c >= 'A' && c <= 'F') return c - 'A' + 10;
	return -1;
}

static uint32_t
hex_to_u32(const char *p, int max_chars, const char **endp)
{
	uint32_t val = 0;
	int count = 0;
	while (count < max_chars) {
		int d = hex_digit(*p);
		if (d < 0) break;
		val = (val << 4) | (uint32_t)d;
		p++;
		count++;
	}
	if (endp) *endp = p;
	return val;
}

static void
byte_to_hex(uint8_t b, char *out)
{
	out[0] = hex_chars[(b >> 4) & 0xf];
	out[1] = hex_chars[b & 0xf];
}

// -------------------------------------------------------------------
// Platform helpers
// -------------------------------------------------------------------

static void
set_nonblocking(socket_t s)
{
#ifdef _WIN32
	u_long mode = 1;
	ioctlsocket(s, FIONBIO, &mode);
#else
	int flags = fcntl(s, F_GETFL, 0);
	fcntl(s, F_SETFL, flags | O_NONBLOCK);
#endif
}

// -------------------------------------------------------------------
// Packet send/receive
// -------------------------------------------------------------------

static void
gdb_send_raw(const void *data, int len)
{
	if (client_sock == SOCKET_INVALID) return;
	const uint8_t *p = (const uint8_t *)data;
	int sent = 0;
	while (sent < len) {
		int n = send(client_sock, (const char *)(p + sent), len - sent, 0);
		if (n <= 0) break;
		sent += n;
	}
}

static void
gdb_send_ack(void)
{
	gdb_send_raw("+", 1);
}

static void
gdb_send_packet(const char *data)
{
	int len = (int)strlen(data);
	uint8_t checksum = 0;
	for (int i = 0; i < len; i++) {
		checksum += (uint8_t)data[i];
	}

	char buf[GDB_PKT_BUF_SIZE + 8];
	buf[0] = '$';
	memcpy(buf + 1, data, len);
	buf[1 + len] = '#';
	byte_to_hex(checksum, buf + 2 + len);
	buf[4 + len] = '\0';

	gdb_send_raw(buf, 4 + len);
}

static void
gdb_send_ok(void)
{
	gdb_send_packet("OK");
}

static void
gdb_send_empty(void)
{
	gdb_send_packet("");
}

static void
gdb_send_error(uint8_t err)
{
	char buf[8];
	buf[0] = 'E';
	byte_to_hex(err, buf + 1);
	buf[3] = '\0';
	gdb_send_packet(buf);
}

// -------------------------------------------------------------------
// Register access
//
// Register map (65C02):
//   0: A   (8-bit)
//   1: X   (8-bit)
//   2: Y   (8-bit)
//   3: SP  (8-bit, low byte of regs.sp)
//   4: PC  (16-bit LE)
//   5: P   (8-bit, status/flags)
//
// 'g' response: A(2) X(2) Y(2) SP(2) PC(4) P(2) = 14 hex chars
// -------------------------------------------------------------------

static uint8_t
reg_read(int idx)
{
	switch (idx) {
		case 0: return regs.a;
		case 1: return regs.xl;
		case 2: return regs.yl;
		case 3: return (uint8_t)(regs.sp & 0xff);
		case 5: return regs.status;
		default: return 0;
	}
}

static void
reg_write(int idx, uint8_t val)
{
	switch (idx) {
		case 0: regs.a = val; break;
		case 1: regs.xl = val; break;
		case 2: regs.yl = val; break;
		case 3: regs.sp = 0x0100 | val; break;
		case 5: regs.status = val; break;
		default: break;
	}
}

static void
handle_read_registers(void)
{
	char buf[32];
	int pos = 0;

	// A, X, Y, SP (8-bit each)
	for (int i = 0; i < 4; i++) {
		byte_to_hex(reg_read(i), buf + pos);
		pos += 2;
	}
	// PC (16-bit LE)
	byte_to_hex((uint8_t)(regs.pc & 0xff), buf + pos);
	pos += 2;
	byte_to_hex((uint8_t)((regs.pc >> 8) & 0xff), buf + pos);
	pos += 2;
	// P (8-bit)
	byte_to_hex(regs.status, buf + pos);
	pos += 2;

	buf[pos] = '\0';
	gdb_send_packet(buf);
}

static void
handle_write_registers(const char *data)
{
	// Expect 14 hex chars: A(2) X(2) Y(2) SP(2) PC(4) P(2)
	int len = (int)strlen(data);
	if (len < 14) {
		gdb_send_error(1);
		return;
	}

	const char *p = data;
	// A
	reg_write(0, (uint8_t)hex_to_u32(p, 2, &p));
	// X
	reg_write(1, (uint8_t)hex_to_u32(p, 2, &p));
	// Y
	reg_write(2, (uint8_t)hex_to_u32(p, 2, &p));
	// SP
	reg_write(3, (uint8_t)hex_to_u32(p, 2, &p));
	// PC (16-bit LE: low byte first)
	uint8_t pcl = (uint8_t)hex_to_u32(p, 2, &p);
	uint8_t pch = (uint8_t)hex_to_u32(p, 2, &p);
	regs.pc = (uint16_t)pcl | ((uint16_t)pch << 8);
	// P
	reg_write(5, (uint8_t)hex_to_u32(p, 2, &p));

	gdb_send_ok();
}

static void
handle_read_register(const char *data)
{
	const char *p = data;
	uint32_t idx = hex_to_u32(p, 8, &p);

	char buf[8];
	if (idx == 4) {
		// PC is 16-bit LE
		byte_to_hex((uint8_t)(regs.pc & 0xff), buf);
		byte_to_hex((uint8_t)((regs.pc >> 8) & 0xff), buf + 2);
		buf[4] = '\0';
	} else if (idx <= 5) {
		byte_to_hex(reg_read((int)idx), buf);
		buf[2] = '\0';
	} else {
		gdb_send_error(1);
		return;
	}
	gdb_send_packet(buf);
}

static void
handle_write_register(const char *data)
{
	const char *p = data;
	uint32_t idx = hex_to_u32(p, 8, &p);
	if (*p != '=') {
		gdb_send_error(1);
		return;
	}
	p++;

	if (idx == 4) {
		// PC: 16-bit LE
		uint8_t pcl = (uint8_t)hex_to_u32(p, 2, &p);
		uint8_t pch = (uint8_t)hex_to_u32(p, 2, &p);
		regs.pc = (uint16_t)pcl | ((uint16_t)pch << 8);
	} else if (idx <= 5) {
		uint8_t val = (uint8_t)hex_to_u32(p, 2, &p);
		reg_write((int)idx, val);
	} else {
		gdb_send_error(1);
		return;
	}
	gdb_send_ok();
}

// -------------------------------------------------------------------
// Memory access
// -------------------------------------------------------------------

static uint8_t
ext_read_byte(uint32_t addr)
{
	if (addr < GDB_VRAM_BASE) {
		// Banked: bank=bits[23:16], cpu_addr=bits[15:0]
		int16_t bank = (int16_t)((addr >> 16) & 0xFF);
		uint16_t cpu_addr = (uint16_t)(addr & 0xFFFF);
		return debug_read6502(cpu_addr, 0, bank);
	} else if (addr < GDB_VRAM_END) {
		// VERA VRAM
		return video_space_read(addr - GDB_VRAM_BASE);
	}
	return 0xFF; // out of range
}

static void
handle_read_memory(const char *data)
{
	const char *p = data;
	uint32_t addr = hex_to_u32(p, 8, &p);
	if (*p != ',') {
		gdb_send_error(1);
		return;
	}
	p++;
	uint32_t len = hex_to_u32(p, 8, &p);

	if (len > (GDB_PKT_BUF_SIZE - 16) / 2) {
		len = (GDB_PKT_BUF_SIZE - 16) / 2;
	}

	// Reject reads starting entirely beyond the address map
	if (addr >= GDB_VRAM_END) {
		gdb_send_error(2);
		return;
	}

	char buf[GDB_PKT_BUF_SIZE];
	int pos = 0;
	for (uint32_t i = 0; i < len; i++) {
		uint8_t val = ext_read_byte(addr + i);
		byte_to_hex(val, buf + pos);
		pos += 2;
	}
	buf[pos] = '\0';
	gdb_send_packet(buf);
}

static bool
ext_write_byte(uint32_t addr, uint8_t val)
{
	if (addr < GDB_VRAM_BASE) {
		// Banked: bank=bits[23:16], cpu_addr=bits[15:0]
		uint8_t bank = (uint8_t)((addr >> 16) & 0xFF);
		uint16_t cpu_addr = (uint16_t)(addr & 0xFFFF);
		if (cpu_addr >= 0xA000 && cpu_addr < 0xC000) {
			// Banked RAM — write directly
			if (bank >= num_ram_banks) return false;
			BRAM[((uint32_t)bank << 13) + cpu_addr - 0xA000] = val;
			return true;
		} else if (cpu_addr >= 0xC000) {
			// ROM — read-only
			return false;
		} else {
			// $0000-$9FFF: bank doesn't apply, plain write
			write6502(cpu_addr, 0, val);
			return true;
		}
	} else if (addr < GDB_VRAM_END) {
		// VERA VRAM
		video_space_write(addr - GDB_VRAM_BASE, val);
		return true;
	}
	return false; // out of range
}

static void
handle_write_memory(const char *data)
{
	const char *p = data;
	uint32_t addr = hex_to_u32(p, 8, &p);
	if (*p != ',') {
		gdb_send_error(1);
		return;
	}
	p++;
	uint32_t len = hex_to_u32(p, 8, &p);
	if (*p != ':') {
		gdb_send_error(1);
		return;
	}
	p++;

	for (uint32_t i = 0; i < len; i++) {
		uint8_t val = (uint8_t)hex_to_u32(p, 2, &p);
		if (!ext_write_byte(addr + i, val)) {
			gdb_send_error(2);
			return;
		}
	}
	gdb_send_ok();
}

// -------------------------------------------------------------------
// Breakpoints
// -------------------------------------------------------------------

static void handle_set_watchpoint(const char *data);
static void handle_remove_watchpoint(const char *data);

static void
handle_set_breakpoint(const char *data)
{
	// Z<type>,addr,kind
	const char *p = data;
	uint32_t type = hex_to_u32(p, 2, &p);
	if (type >= 2) {
		// Watchpoint types 2/3/4
		handle_set_watchpoint(data);
		return;
	}
	if (type != 0 && type != 1) {
		gdb_send_empty();
		return;
	}
	if (*p != ',') {
		gdb_send_error(1);
		return;
	}
	p++;
	uint32_t addr = hex_to_u32(p, 8, &p);
	// kind field ignored for 65C02

	uint16_t cpu_addr = (uint16_t)(addr & 0xFFFF);
	uint8_t bank = (uint8_t)((addr >> 16) & 0xFF);
	bool banked = (cpu_addr >= 0xA000);

	// Check if already set
	for (int i = 0; i < num_breakpoints; i++) {
		if (breakpoints[i].active && breakpoints[i].addr == cpu_addr) {
			if (!banked || breakpoints[i].bank == bank) {
				gdb_send_ok();
				return;
			}
		}
	}

	// Find a free slot
	for (int i = 0; i < GDB_MAX_BREAKPOINTS; i++) {
		if (!breakpoints[i].active) {
			breakpoints[i].addr = cpu_addr;
			breakpoints[i].bank = bank;
			breakpoints[i].banked = banked;
			breakpoints[i].active = true;
			if (i >= num_breakpoints) num_breakpoints = i + 1;
			gdb_send_ok();
			return;
		}
	}
	gdb_send_error(2);
}

static void
handle_remove_breakpoint(const char *data)
{
	// z<type>,addr,kind
	const char *p = data;
	uint32_t type = hex_to_u32(p, 2, &p);
	if (type >= 2) {
		// Watchpoint types 2/3/4
		handle_remove_watchpoint(data);
		return;
	}
	if (type != 0 && type != 1) {
		gdb_send_empty();
		return;
	}
	if (*p != ',') {
		gdb_send_error(1);
		return;
	}
	p++;
	uint32_t addr = hex_to_u32(p, 8, &p);

	uint16_t cpu_addr = (uint16_t)(addr & 0xFFFF);
	uint8_t bank = (uint8_t)((addr >> 16) & 0xFF);
	bool banked = (cpu_addr >= 0xA000);

	for (int i = 0; i < num_breakpoints; i++) {
		if (breakpoints[i].active && breakpoints[i].addr == cpu_addr) {
			if (!banked || breakpoints[i].bank == bank) {
				breakpoints[i].active = false;
				gdb_send_ok();
				return;
			}
		}
	}
	gdb_send_ok(); // not found is still OK per GDB spec
}

static bool
check_breakpoints(void)
{
	for (int i = 0; i < num_breakpoints; i++) {
		if (breakpoints[i].active && breakpoints[i].addr == regs.pc) {
			if (!breakpoints[i].banked) {
				return true;
			}
			// Bank-sensitive: compare current bank
			if (breakpoints[i].bank == get_current_bank_for_addr(regs.pc)) {
				return true;
			}
		}
	}
	return false;
}

// -------------------------------------------------------------------
// Watchpoint helpers
// -------------------------------------------------------------------

static void
update_watchpoints_active(void)
{
	gdb_watchpoints_active = false;
	for (int i = 0; i < num_watchpoints; i++) {
		if (watchpoints[i].active) {
			gdb_watchpoints_active = true;
			return;
		}
	}
}

static void
handle_set_watchpoint(const char *data)
{
	// Z<type>,addr,len
	const char *p = data;
	uint32_t type = hex_to_u32(p, 2, &p);
	if (type < 2 || type > 4) {
		gdb_send_empty();
		return;
	}
	if (*p != ',') {
		gdb_send_error(1);
		return;
	}
	p++;
	uint32_t addr = hex_to_u32(p, 8, &p);
	if (*p != ',') {
		gdb_send_error(1);
		return;
	}
	p++;
	uint32_t len = hex_to_u32(p, 8, &p);
	if (len == 0) len = 1;

	// Check if already set
	for (int i = 0; i < num_watchpoints; i++) {
		if (watchpoints[i].active &&
		    watchpoints[i].addr == addr &&
		    watchpoints[i].len == len &&
		    watchpoints[i].type == (wp_type_t)type) {
			gdb_send_ok();
			return;
		}
	}

	// Find a free slot
	for (int i = 0; i < GDB_MAX_WATCHPOINTS; i++) {
		if (!watchpoints[i].active) {
			watchpoints[i].addr = addr;
			watchpoints[i].len = len;
			watchpoints[i].type = (wp_type_t)type;
			watchpoints[i].active = true;
			if (i >= num_watchpoints) num_watchpoints = i + 1;
			update_watchpoints_active();
			gdb_send_ok();
			return;
		}
	}
	gdb_send_error(2);
}

static void
handle_remove_watchpoint(const char *data)
{
	// z<type>,addr,len
	const char *p = data;
	uint32_t type = hex_to_u32(p, 2, &p);
	if (type < 2 || type > 4) {
		gdb_send_empty();
		return;
	}
	if (*p != ',') {
		gdb_send_error(1);
		return;
	}
	p++;
	uint32_t addr = hex_to_u32(p, 8, &p);
	if (*p != ',') {
		gdb_send_error(1);
		return;
	}
	p++;
	uint32_t len = hex_to_u32(p, 8, &p);
	if (len == 0) len = 1;

	for (int i = 0; i < num_watchpoints; i++) {
		if (watchpoints[i].active &&
		    watchpoints[i].addr == addr &&
		    watchpoints[i].len == len &&
		    watchpoints[i].type == (wp_type_t)type) {
			watchpoints[i].active = false;
			update_watchpoints_active();
			gdb_send_ok();
			return;
		}
	}
	gdb_send_ok(); // not found is still OK per GDB spec
}

void
gdbstub_check_write(uint16_t addr, uint8_t value)
{
	(void)value;
	uint32_t full_addr;
	if (addr >= 0xA000) {
		full_addr = ((uint32_t)get_current_bank_for_addr(addr) << 16) | addr;
	} else {
		full_addr = addr;
	}

	for (int i = 0; i < num_watchpoints; i++) {
		if (!watchpoints[i].active) continue;
		if (watchpoints[i].type != WP_WRITE && watchpoints[i].type != WP_ACCESS) continue;
		if (full_addr >= watchpoints[i].addr &&
		    full_addr < watchpoints[i].addr + watchpoints[i].len) {
			wp_hit = true;
			wp_hit_addr = full_addr;
			wp_hit_type = watchpoints[i].type;
			return;
		}
	}
}

void
gdbstub_check_read(uint16_t addr)
{
	uint32_t full_addr;
	if (addr >= 0xA000) {
		full_addr = ((uint32_t)get_current_bank_for_addr(addr) << 16) | addr;
	} else {
		full_addr = addr;
	}

	for (int i = 0; i < num_watchpoints; i++) {
		if (!watchpoints[i].active) continue;
		if (watchpoints[i].type != WP_READ && watchpoints[i].type != WP_ACCESS) continue;
		if (full_addr >= watchpoints[i].addr &&
		    full_addr < watchpoints[i].addr + watchpoints[i].len) {
			wp_hit = true;
			wp_hit_addr = full_addr;
			wp_hit_type = watchpoints[i].type;
			return;
		}
	}
}

// -------------------------------------------------------------------
// vCont support
// -------------------------------------------------------------------

static void
handle_v_command(const char *data, int len)
{
	if (len >= 5 && strncmp(data, "Cont?", 5) == 0) {
		gdb_send_packet("vCont;c;s;t;r");
		return;
	}

	if (len >= 5 && strncmp(data, "Cont;", 5) == 0) {
		const char *action = data + 5;
		switch (action[0]) {
			case 'c':
				range_stepping = false;
				gdb_state = GDB_STATE_RUNNING;
				return;
			case 's':
				range_stepping = false;
				step6502();
				gdb_state = GDB_STATE_STOPPED;
				gdb_send_packet("S05");
				return;
			case 't':
				range_stepping = false;
				gdb_state = GDB_STATE_STOPPED;
				gdb_send_packet("S02");
				return;
			case 'r': {
				// vCont;r start,end — range stepping
				const char *p = action + 1;
				range_step_start = (uint16_t)hex_to_u32(p, 8, &p);
				if (*p == ',') p++;
				range_step_end = (uint16_t)hex_to_u32(p, 8, &p);
				range_stepping = true;
				gdb_state = GDB_STATE_RUNNING;
				return;
			}
			default:
				break;
		}
	}

	// Unrecognized v command
	gdb_send_empty();
}

// -------------------------------------------------------------------
// Command dispatch
// -------------------------------------------------------------------

// -------------------------------------------------------------------
// Monitor command handler (qRcmd)
// Decodes hex-encoded command string and dispatches trace commands.
// -------------------------------------------------------------------

#ifdef TRACE
extern bool trace_mode;
extern bool trace_compact;
extern uint16_t trace_address;
#endif

static void
gdb_send_monitor_output(const char *msg)
{
	// qRcmd response: 'O' followed by hex-encoded ASCII output
	int len = (int)strlen(msg);
	char buf[GDB_PKT_BUF_SIZE];
	buf[0] = 'O';
	for (int i = 0; i < len && (2 * i + 3) < GDB_PKT_BUF_SIZE; i++) {
		byte_to_hex((uint8_t)msg[i], buf + 1 + 2 * i);
	}
	buf[1 + 2 * len] = '\0';
	gdb_send_packet(buf);
}

static void
handle_monitor_command(const char *hex_cmd)
{
	// Decode hex-encoded ASCII command
	int hex_len = (int)strlen(hex_cmd);
	char cmd[256];
	int cmd_len = 0;
	for (int i = 0; i + 1 < hex_len && cmd_len < (int)sizeof(cmd) - 1; i += 2) {
		int hi = hex_digit(hex_cmd[i]);
		int lo = hex_digit(hex_cmd[i + 1]);
		if (hi < 0 || lo < 0) break;
		cmd[cmd_len++] = (char)((hi << 4) | lo);
	}
	cmd[cmd_len] = '\0';

#ifdef TRACE
	if (strcmp(cmd, "trace on") == 0 || strcmp(cmd, "trace on full") == 0) {
		trace_mode = true;
		trace_compact = false;
		trace_address = 0;
		gdb_send_monitor_output("Trace enabled (full)\n");
		gdb_send_ok();
	} else if (strcmp(cmd, "trace on compact") == 0) {
		trace_mode = true;
		trace_compact = true;
		trace_address = 0;
		gdb_send_monitor_output("Trace enabled (compact)\n");
		gdb_send_ok();
	} else if (strcmp(cmd, "trace off") == 0) {
		trace_mode = false;
		trace_address = 0;
		fflush(stderr);
		gdb_send_monitor_output("Trace disabled\n");
		gdb_send_ok();
	} else if (strncmp(cmd, "trace ", 6) == 0) {
		// "trace XXXX [compact]" — set trace address trigger
		char *endptr;
		uint16_t addr = (uint16_t)strtol(cmd + 6, &endptr, 16);
		trace_address = addr;
		trace_mode = false;
		trace_compact = (endptr && *endptr == ' ' && strcmp(endptr + 1, "compact") == 0);
		char msg[80];
		snprintf(msg, sizeof(msg), "Trace will start at $%04X (%s)\n", addr, trace_compact ? "compact" : "full");
		gdb_send_monitor_output(msg);
		gdb_send_ok();
	} else {
		gdb_send_monitor_output("Unknown command. Available: trace on [compact|full]|off|<addr> [compact]\n");
		gdb_send_ok();
	}
#else
	(void)cmd;
	gdb_send_monitor_output("Trace support not compiled in (rebuild with -DENABLE_TRACE=ON)\n");
	gdb_send_ok();
#endif
}

static void
handle_command(const char *pkt, int pkt_len)
{
	if (pkt_len == 0) return;

	switch (pkt[0]) {
		case '?':
			// Halt reason
			gdbstub_report_stop(5);
			break;

		case 'g':
			handle_read_registers();
			break;

		case 'G':
			handle_write_registers(pkt + 1);
			break;

		case 'p':
			handle_read_register(pkt + 1);
			break;

		case 'P':
			handle_write_register(pkt + 1);
			break;

		case 'm':
			handle_read_memory(pkt + 1);
			break;

		case 'M':
			handle_write_memory(pkt + 1);
			break;

		case 'c':
			// Continue
			gdb_state = GDB_STATE_RUNNING;
			break;

		case 's':
			// Single step: execute one instruction, then stop
			step6502();
			gdb_state = GDB_STATE_STOPPED;
			gdb_send_packet("S05");
			break;

		case 'Z':
			handle_set_breakpoint(pkt + 1);
			break;

		case 'z':
			handle_remove_breakpoint(pkt + 1);
			break;

		case 'k':
			// Kill - disconnect and quit emulator
			printf("[GDB] Kill request received.\n");
			if (client_sock != SOCKET_INVALID) {
				CLOSE_SOCKET(client_sock);
				client_sock = SOCKET_INVALID;
			}
			gdb_connected = false;
			gdb_state = GDB_STATE_DETACHED;
			break;

		case 'D':
			// Detach - disconnect, resume execution
			gdb_send_ok();
			if (client_sock != SOCKET_INVALID) {
				CLOSE_SOCKET(client_sock);
				client_sock = SOCKET_INVALID;
			}
			gdb_connected = false;
			gdb_state = GDB_STATE_DETACHED;
			printf("[GDB] Client detached.\n");
			break;

		case 'q':
			// Query commands
			if (strncmp(pkt, "qSupported", 10) == 0) {
				gdb_send_packet("PacketSize=4096;x16.addrsize=25;hwbreak+;vContSupported+");
			} else if (strncmp(pkt, "qAttached", 9) == 0) {
				gdb_send_packet("1");
			} else if (strncmp(pkt, "qC", 2) == 0 && pkt[2] == '\0') {
				gdb_send_packet("QC0");
			} else if (strncmp(pkt, "qfThreadInfo", 12) == 0) {
				gdb_send_packet("m0");
			} else if (strncmp(pkt, "qsThreadInfo", 12) == 0) {
				gdb_send_packet("l");
			} else if (strncmp(pkt, "qTStatus", 8) == 0) {
				gdb_send_packet("T0");
			} else if (strncmp(pkt, "qOffsets", 8) == 0) {
				gdb_send_packet("Text=0;Data=0;Bss=0");
			} else if (strncmp(pkt, "qSymbol::", 9) == 0) {
				gdb_send_ok();
			} else if (strncmp(pkt, "qRcmd,", 6) == 0) {
				handle_monitor_command(pkt + 6);
			} else {
				gdb_send_empty();
			}
			break;

		case 'v':
			handle_v_command(pkt + 1, pkt_len - 1);
			break;

		default:
			// Unsupported command
			gdb_send_empty();
			break;
	}
}

// -------------------------------------------------------------------
// Packet parser: extract $data#checksum from recv buffer
// Returns number of bytes consumed, or 0 if no complete packet yet.
// -------------------------------------------------------------------

static int
parse_packet(char *out_data, int out_max, int *out_len)
{
	// Look for Ctrl-C (0x03) byte first
	for (int i = 0; i < recv_len; i++) {
		if (recv_buf[i] == 0x03) {
			// Ctrl-C interrupt: consume bytes up to and including it
			*out_len = 0;
			// Signal interrupt by setting out_data[0] = 0x03
			if (out_max > 0) out_data[0] = 0x03;
			*out_len = -1; // special signal for Ctrl-C
			return i + 1;
		}
	}

	// Find '$'
	int start = -1;
	for (int i = 0; i < recv_len; i++) {
		if (recv_buf[i] == '$') {
			start = i;
			break;
		}
	}
	if (start < 0) {
		// No packet start; discard everything before any potential '$'
		recv_len = 0;
		return 0;
	}

	// Find '#'
	int hash = -1;
	for (int i = start + 1; i < recv_len; i++) {
		if (recv_buf[i] == '#') {
			hash = i;
			break;
		}
	}
	if (hash < 0) return 0; // incomplete packet

	// Need 2 more bytes for checksum
	if (hash + 2 >= recv_len) return 0;

	int data_len = hash - start - 1;
	if (data_len >= out_max) data_len = out_max - 1;

	memcpy(out_data, recv_buf + start + 1, data_len);
	out_data[data_len] = '\0';
	*out_len = data_len;

	// Verify checksum
	uint8_t expected = 0;
	for (int i = start + 1; i < hash; i++) {
		expected += recv_buf[i];
	}
	int d0 = hex_digit((char)recv_buf[hash + 1]);
	int d1 = hex_digit((char)recv_buf[hash + 2]);
	if (d0 >= 0 && d1 >= 0) {
		uint8_t received = (uint8_t)((d0 << 4) | d1);
		if (received == expected) {
			gdb_send_ack();
		}
		// We process the command regardless of checksum
	}

	return hash + 3; // consumed up to and including checksum
}

// -------------------------------------------------------------------
// Connection management
// -------------------------------------------------------------------

static void
disconnect_client(void)
{
	if (client_sock != SOCKET_INVALID) {
		CLOSE_SOCKET(client_sock);
		client_sock = SOCKET_INVALID;
	}
	gdb_connected = false;
	recv_len = 0;
	printf("[GDB] Client disconnected.\n");
}

static void
accept_connection(void)
{
	struct sockaddr_in client_addr;
	socklen_t addr_len = sizeof(client_addr);
	socket_t s = accept(listen_sock, (struct sockaddr *)&client_addr, &addr_len);
	if (s == SOCKET_INVALID) return;

	// Only one client at a time
	if (client_sock != SOCKET_INVALID) {
		CLOSE_SOCKET(s);
		return;
	}

	client_sock = s;
	set_nonblocking(client_sock);

	// Disable Nagle's algorithm for low latency
	int flag = 1;
	setsockopt(client_sock, IPPROTO_TCP, TCP_NODELAY, (const char *)&flag, sizeof(flag));

	gdb_connected = true;
	gdb_state = GDB_STATE_STOPPED;
	recv_len = 0;

	printf("[GDB] Client connected from %s:%d (PC=$%04X)\n",
	       inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port), regs.pc); fflush(stdout);

	// Clear breakpoints and watchpoints on new connection
	for (int i = 0; i < GDB_MAX_BREAKPOINTS; i++) {
		breakpoints[i].active = false;
	}
	num_breakpoints = 0;

	for (int i = 0; i < GDB_MAX_WATCHPOINTS; i++) {
		watchpoints[i].active = false;
	}
	num_watchpoints = 0;
	gdb_watchpoints_active = false;
	wp_hit = false;
	range_stepping = false;
}

// -------------------------------------------------------------------
// Receive data from client
// Returns true if data was received, false otherwise
// -------------------------------------------------------------------

static bool
recv_data(void)
{
	if (client_sock == SOCKET_INVALID) return false;

	int space = GDB_RECV_BUF_SIZE - recv_len;
	if (space <= 0) return false;

	int n = recv(client_sock, (char *)(recv_buf + recv_len), space, 0);
	if (n > 0) {
		recv_len += n;
		return true;
	} else if (n == 0) {
		// Connection closed
		disconnect_client();
		gdb_state = GDB_STATE_LISTENING;
		return false;
	} else {
		if (WOULD_BLOCK) return false;
		// Error
		disconnect_client();
		gdb_state = GDB_STATE_LISTENING;
		return false;
	}
}

// -------------------------------------------------------------------
// Process all complete packets in the receive buffer
// -------------------------------------------------------------------

static void
process_packets(void)
{
	char pkt_data[GDB_PKT_BUF_SIZE];
	int pkt_len;

	for (;;) {
		int consumed = parse_packet(pkt_data, sizeof(pkt_data), &pkt_len);
		if (consumed == 0) break;

		// Remove consumed bytes from recv_buf
		if (consumed < recv_len) {
			memmove(recv_buf, recv_buf + consumed, recv_len - consumed);
		}
		recv_len -= consumed;

		if (pkt_len == -1) {
			// Ctrl-C interrupt
			if (gdb_state == GDB_STATE_RUNNING) {
				gdb_state = GDB_STATE_STOPPED;
				gdb_send_packet("S02"); // SIGINT
			}
		} else {
			handle_command(pkt_data, pkt_len);
		}

		// If we transitioned to running, stop processing packets
		if (gdb_state == GDB_STATE_RUNNING) break;
	}
}

// -------------------------------------------------------------------
// Public API
// -------------------------------------------------------------------

void
gdbstub_init(uint16_t port)
{
#ifdef _WIN32
	if (!wsa_initialized) {
		WSADATA wsa;
		if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
			fprintf(stderr, "[GDB] WSAStartup failed.\n");
			return;
		}
		wsa_initialized = true;
	}
#endif

	listen_sock = socket(AF_INET, SOCK_STREAM, 0);
	if (listen_sock == SOCKET_INVALID) {
		fprintf(stderr, "[GDB] Failed to create socket.\n");
		return;
	}

	int opt = 1;
	setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, (const char *)&opt, sizeof(opt));

	struct sockaddr_in addr;
	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = htonl(INADDR_ANY);
	addr.sin_port = htons(port);

	if (bind(listen_sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		fprintf(stderr, "[GDB] Failed to bind to port %d: %s\n", port, strerror(errno));
		CLOSE_SOCKET(listen_sock);
		listen_sock = SOCKET_INVALID;
		return;
	}

	if (listen(listen_sock, 1) < 0) {
		fprintf(stderr, "[GDB] Failed to listen: %s\n", strerror(errno));
		CLOSE_SOCKET(listen_sock);
		listen_sock = SOCKET_INVALID;
		return;
	}

	set_nonblocking(listen_sock);

	gdb_state = GDB_STATE_LISTENING;
	gdb_enabled = true;

	printf("[GDB] Listening on port %d...\n", port);
}

void
gdbstub_shutdown(void)
{
	if (client_sock != SOCKET_INVALID) {
		CLOSE_SOCKET(client_sock);
		client_sock = SOCKET_INVALID;
	}
	if (listen_sock != SOCKET_INVALID) {
		CLOSE_SOCKET(listen_sock);
		listen_sock = SOCKET_INVALID;
	}
	gdb_connected = false;
	gdb_enabled = false;

#ifdef _WIN32
	if (wsa_initialized) {
		WSACleanup();
		wsa_initialized = false;
	}
#endif

	printf("[GDB] Shut down.\n");
}

int
gdbstub_poll(void)
{
	switch (gdb_state) {
		case GDB_STATE_LISTENING:
			accept_connection();
			if (gdb_state == GDB_STATE_STOPPED) {
				// Just connected - halt CPU, wait for GDB commands
				return 1;
			}
			return 0; // No connection yet, keep running

		case GDB_STATE_STOPPED:
			if (client_sock == SOCKET_INVALID) {
				// No client connected — try to accept one while halted
				accept_connection();
				// accept_connection sets gdb_state to STOPPED if connected
				if (client_sock != SOCKET_INVALID) {
					// New client just connected; send stop notification
					gdbstub_report_stop(5); // SIGTRAP
				}
				return 1; // Stay halted, waiting for client
			}
			// CPU is halted; process GDB commands
			recv_data();
			process_packets();

			if (gdb_state == GDB_STATE_RUNNING) {
				return 0; // Resume execution
			}
			if (gdb_state == GDB_STATE_DETACHED || gdb_state == GDB_STATE_LISTENING) {
				return 0; // Disconnected, keep running
			}
			return 1; // Still stopped

		case GDB_STATE_RUNNING:
			// CPU is running; check for incoming data (Ctrl-C)
			recv_data();
			if (recv_len > 0) {
				process_packets();
			}

			if (gdb_state == GDB_STATE_STOPPED) {
				printf("[GDB] STOP: Ctrl-C at PC=$%04X\n", regs.pc); fflush(stdout);
				return 1; // Ctrl-C received, halt
			}

			// Check breakpoints after each instruction
			if (check_breakpoints()) {
				gdb_state = GDB_STATE_STOPPED;
				range_stepping = false;
				wp_hit = false;
				printf("[GDB] STOP: breakpoint at PC=$%04X\n", regs.pc); fflush(stdout);
				gdbstub_report_stop(5); // SIGTRAP
				return 1;
			}

			// Check watchpoints
			if (wp_hit) {
				gdb_state = GDB_STATE_STOPPED;
				range_stepping = false;
				// Send stop reply with watchpoint info
				char wp_reply[64];
				const char *wp_kind;
				switch (wp_hit_type) {
					case WP_READ:   wp_kind = "rwatch"; break;
					case WP_ACCESS: wp_kind = "awatch"; break;
					default:        wp_kind = "watch";  break;
				}
				snprintf(wp_reply, sizeof(wp_reply), "T05%s:%x;", wp_kind, wp_hit_addr);
				gdb_send_packet(wp_reply);
				wp_hit = false;
				return 1;
			}

			// Range stepping: stop when PC leaves the range
			if (range_stepping &&
			    (regs.pc < range_step_start || regs.pc >= range_step_end)) {
				gdb_state = GDB_STATE_STOPPED;
				range_stepping = false;
				gdbstub_report_stop(5); // SIGTRAP
				return 1;
			}
			return 0; // Keep running

		case GDB_STATE_DETACHED:
			// Check for new connections
			accept_connection();
			if (gdb_state == GDB_STATE_STOPPED) {
				return 1;
			}
			return 0;
	}
	return 0;
}

void
gdbstub_break(uint8_t signal)
{
	// Always halt the CPU when GDB is enabled, regardless of connection state.
	// If no client is connected, we still transition to STOPPED so the CPU
	// halts and waits for a client to reconnect.
	printf("[GDB] STOP: gdbstub_break(sig=%d) at PC=$%04X (state=%d, connected=%d, rambank=%d, rombank=%d)\n",
	       signal, regs.pc, gdb_state, gdb_connected,
	       memory_get_ram_bank(), memory_get_rom_bank()); fflush(stdout);
	gdb_state = GDB_STATE_STOPPED;
	range_stepping = false;
	wp_hit = false;
	gdbstub_report_stop(signal); // safe if no client — report_stop checks client_sock
}

void
gdbstub_report_stop(uint8_t signal)
{
	if (client_sock == SOCKET_INVALID) return;

	// Use T format with PC register value to reduce follow-up queries
	// T SS 04:pclo,pchi;
	char buf[32];
	int pos = 0;
	buf[pos++] = 'T';
	byte_to_hex(signal, buf + pos);
	pos += 2;
	// Register 4 = PC (16-bit LE)
	buf[pos++] = '0';
	buf[pos++] = '4';
	buf[pos++] = ':';
	byte_to_hex((uint8_t)(regs.pc & 0xff), buf + pos);
	pos += 2;
	byte_to_hex((uint8_t)((regs.pc >> 8) & 0xff), buf + pos);
	pos += 2;
	buf[pos++] = ';';
	buf[pos] = '\0';
	gdb_send_packet(buf);
}
