# GDB Remote Serial Protocol

The x16 emulator includes a GDB remote serial protocol stub that allows
external tools to halt the CPU, inspect and modify registers and memory,
set breakpoints and watchpoints, single-step, and control instruction
tracing.

## Starting the GDB stub

```
x16emu -gdb [<port>]
```

The default TCP port is **2159**.  The emulator listens for one client at
a time.  When a client connects the CPU halts immediately and waits for
commands.  If the client disconnects, the CPU resumes and the emulator
goes back to listening for a new connection.

## Connecting

Any tool that speaks the GDB remote serial protocol can connect.  A
minimal session with Python looks like:

```python
import socket

sock = socket.create_connection(("localhost", 2159))

def send_pkt(data: str) -> str:
    chk = sum(data.encode()) & 0xFF
    sock.sendall(f"${data}#{chk:02x}".encode())
    return sock.recv(4096).decode()
```

To halt the CPU at any time, send a raw **0x03** byte (Ctrl-C).  The
emulator replies with a stop packet (`S02`, SIGINT).

## Supported packets

### Query / handshake

| Packet | Response | Description |
|--------|----------|-------------|
| `qSupported` | feature list | Reports `PacketSize=4096`, `hwbreak+`, `vContSupported+` |
| `qAttached` | `1` | Always attached to the emulated CPU |
| `qC` | `QC0` | Current thread (single-threaded) |
| `qfThreadInfo` | `m0` | First thread |
| `qsThreadInfo` | `l` | End of thread list |
| `qTStatus` | `T0` | Tracepoint status (none active) |
| `qOffsets` | `Text=0;Data=0;Bss=0` | Section offsets |
| `qRcmd,<hex>` | see [Monitor commands](#monitor-commands) | Hex-encoded monitor command |

### Halt reason

| Packet | Description |
|--------|-------------|
| `?` | Returns `T05` (SIGTRAP) with the current PC |

### Registers

The register map for 65C02 mode:

| Index | Register | Width |
|-------|----------|-------|
| 0 | A | 8-bit |
| 1 | X | 8-bit |
| 2 | Y | 8-bit |
| 3 | SP | 8-bit (low byte) |
| 4 | PC | 16-bit little-endian |
| 5 | P (status) | 8-bit |

| Packet | Description |
|--------|-------------|
| `g` | Read all registers.  Returns 14 hex chars: `AA XX YY SS PClo PChi PP` |
| `G<hex>` | Write all registers (same 14-char format) |
| `p<n>` | Read register *n* |
| `P<n>=<hex>` | Write register *n* |

### Memory

Addresses use an extended 25-bit scheme:

| GDB address range | Maps to |
|-------------------|---------|
| `$00000000`–`$0000FFFF` | CPU address space using **current** bank settings |
| `$00XX0000`–`$00XXFFFF` | CPU address space with explicit bank `$XX` (bits 23–16).  For addresses `$A000`–`$BFFF` this selects the RAM bank; for `$C000`–`$FFFF` the ROM bank. |
| `$01000000`–`$0101FFFF` | VERA VRAM (128 KB) |

| Packet | Description |
|--------|-------------|
| `m<addr>,<len>` | Read *len* bytes starting at *addr* (hex).  Returns hex-encoded data. |
| `M<addr>,<len>:<data>` | Write *len* bytes of hex *data* starting at *addr*.  ROM addresses (`$C000`–`$FFFF`) are read-only and will error. |

### Execution control

| Packet | Description |
|--------|-------------|
| `c` | Continue (resume execution) |
| `s` | Single-step one instruction |
| `vCont;c` | Continue |
| `vCont;s` | Single-step |
| `vCont;t` | Stop (halt CPU) |
| `vCont;r<start>,<end>` | Range-step: run until PC leaves `[start, end)` |
| `vCont?` | Query supported vCont actions.  Returns `vCont;c;s;t;r` |
| **0x03** (raw byte) | Interrupt — halt the CPU immediately (like Ctrl-C) |

### Breakpoints (up to 32)

Breakpoints use the same extended address encoding as memory commands.
For banked addresses (`$A000`+), the bank byte determines which bank the
breakpoint applies to.  For non-banked addresses the bank byte is
ignored.

| Packet | Description |
|--------|-------------|
| `Z0,<addr>,<kind>` | Set software breakpoint (*kind* is ignored) |
| `z0,<addr>,<kind>` | Remove software breakpoint |
| `Z1,<addr>,<kind>` | Set hardware breakpoint (treated same as Z0) |
| `z1,<addr>,<kind>` | Remove hardware breakpoint |

### Watchpoints (up to 16)

Watchpoints monitor memory accesses during CPU execution.  They use the
same 24-bit address encoding as breakpoints.

| Packet | Description |
|--------|-------------|
| `Z2,<addr>,<len>` | Write watchpoint — stop when the CPU writes to `[addr, addr+len)` |
| `z2,<addr>,<len>` | Remove write watchpoint |
| `Z3,<addr>,<len>` | Read watchpoint — stop when the CPU reads from the range |
| `z3,<addr>,<len>` | Remove read watchpoint |
| `Z4,<addr>,<len>` | Access watchpoint — stop on either read or write |
| `z4,<addr>,<len>` | Remove access watchpoint |

When a watchpoint fires the stop reply includes the type and address,
e.g. `T05watch:1234;` or `T05rwatch:1234;` or `T05awatch:1234;`.

### Disconnect

| Packet | Description |
|--------|-------------|
| `D` | Detach — resume execution, close connection |
| `k` | Kill — close connection (emulator keeps running) |

## Monitor commands

Monitor commands are sent via the `qRcmd` packet.  The command string is
hex-encoded by the client.  For example, in GDB:

```
(gdb) monitor trace on compact
```

Or with a raw packet (hex-encoding `trace on`):

```
$qRcmd,747261636520 on#xx
```

### Trace commands

Requires the emulator to be built with trace support (`-DENABLE_TRACE=ON`
or auto-detected when `rom_lst.h` and `rom_labels.h` are present).
Trace output is written to **stderr**.

| Command | Description |
|---------|-------------|
| `trace on` | Enable tracing (full format) |
| `trace on full` | Same as `trace on` |
| `trace on compact` | Enable tracing (compact format — see below) |
| `trace off` | Disable tracing and flush output |
| `trace <addr>` | Set address trigger: tracing starts when PC reaches `$addr` (full format) |
| `trace <addr> compact` | Set address trigger with compact format |

#### Full trace format

Each instruction produces a line like:

```
[  counter] label<pad>  BK:.,ADDR XX XX XX  disasm          A=$XX X=$XX Y=$XX S=$XX P=czidb-vn EA=$BK:ADDR  listing
```

| Field | Width | Description |
|-------|-------|-------------|
| Listing preamble | varies | Source listing context lines (if `rom_lst.h` data available) |
| `[counter]` | 11 | Instruction sequence number |
| Label | 20 | Symbol name from `rom_labels.h` (padded to 20 chars) |
| Bank | 3 | ROM/RAM bank number or `--` for non-banked addresses |
| Address | 8 | `:.,%04x` format PC value |
| Raw bytes | 9 | Hex bytes of the instruction, padded to 9 chars |
| Disassembly | 15 | Mnemonic + operands, padded to 15 chars |
| Registers | 38 | `A=$XX X=$XX Y=$XX S=$XX P=` + 8 flag chars |
| Effective addr | 13 | `EA=$BK:ADDR` or `VRAM=$XXXXX`, padded to 13 chars |
| Listing suffix | varies | Source listing for current instruction |

#### Compact trace format

Drops fields that are empty (with stub ROM headers) or redundant:

```
[  counter]  BK:.,ADDR disasm          A=$XX X=$XX Y=$XX S=$XX P=czidb-vn EA=BK:ADDR
```

Compared to full format, compact mode:

- **Removes** the label column (20 chars of padding when `rom_labels.h` is stubs)
- **Removes** raw instruction bytes (redundant with disassembly)
- **Removes** listing preamble and suffix lookups
- **Removes** empty effective-address padding (EA only printed when present)
- Saves roughly **45%** of output volume per line

## STP instruction

When the emulator is started with `-gdb`, the `STP` instruction (opcode
`$DB`) triggers `gdbstub_break()` which halts the CPU and notifies the
connected GDB client with a SIGTRAP stop.  This is useful for setting
programmatic breakpoints in 6502 code.

## Limits

| Resource | Limit |
|----------|-------|
| Breakpoints | 32 |
| Watchpoints | 16 |
| Packet buffer | 4096 bytes |
| Connections | 1 at a time |
