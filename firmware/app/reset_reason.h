/*
 * Why the chip last reset, from the RP2350's POWMAN CHIP_RESET register
 * (plans/web-config-server.md, "Diagnostics page"), shown in /status.json
 * and on the USB status. Core 0, read once at boot.
 */
#ifndef RESET_REASON_H
#define RESET_REASON_H

#include <stdint.h>

/* Reads and keeps the reason; call early in main(). */
void reset_reason_init(void);

/* A short phrase ("power on", "watchdog reboot", "debugger reset", ...). */
const char *reset_reason(void);

/* The raw CHIP_RESET value read at boot. */
uint32_t reset_reason_raw(void);

#endif /* RESET_REASON_H */
