// Commander X16 Emulator
// GDB Remote Serial Protocol stub
// License: 2-clause BSD

#ifndef _GDBSTUB_H_
#define _GDBSTUB_H_

#include <stdint.h>
#include <stdbool.h>

// Initialize GDB stub: create listen socket on given port
void gdbstub_init(uint16_t port);

// Shut down GDB stub: close all sockets
void gdbstub_shutdown(void);

// Poll for GDB activity. Called each iteration of emulator_loop.
// Returns: 0 = keep running, 1 = halted/waiting for GDB, -1 = quit
int gdbstub_poll(void);

// Notify GDB that the CPU stopped (e.g. breakpoint hit, step complete)
void gdbstub_report_stop(uint8_t signal);

extern bool gdb_enabled;
extern bool gdb_connected;

#endif
