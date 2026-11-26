#include "reset_reason.h"

#include "hardware/structs/powman.h"
#include "hardware/watchdog.h"
#include "pico/platform/sections.h"

/* POWMAN CHIP_RESET records chip-level resets only, and its HAD_POR bit
 * stays set through the smaller ones: after a watchdog_reboot() it still
 * reads 0x00010000 (checked on the bench 2026-11-25), and a core reset from
 * the debugger (SYSRESETREQ) leaves no trace in it at all. So the order is:
 * the watchdog's own reason register (cleared by a chip reset), then the
 * chip-level bits other than power-on, then a marker in RAM that the boot
 * code leaves alone: still there means the RAM was kept, so it was a core
 * reset, not a power-on. A chip-level bit other than power-on also stays
 * set until the next chip-level reset, so a core reset after, say, a RUN pin
 * reset still reads "RUN pin". Flashing over SWD may read "power on":
 * OpenOCD's flash loader uses the SRAM and can overwrite the marker (one
 * flash did, the next didn't). Bench 2026-11-25: web reboot -> watchdog
 * reboot, OpenOCD "reset run" -> core reset. */
#define WARM_MAGIC 0x57A2B007u

static uint32_t __uninitialized_ram(s_warm);
static uint32_t s_raw;
static const char *s_reason = "unknown";

void reset_reason_init(void)
{
    s_raw = powman_hw->chip_reset;
    const bool warm = s_warm == WARM_MAGIC;
    s_warm = WARM_MAGIC;
    if (watchdog_caused_reboot()) {
        /* A timeout only once watchdog_enable() is in use; the reboot action
         * (web page, USB command) uses watchdog_reboot(). */
        s_reason = watchdog_enable_caused_reboot() ? "watchdog timeout" : "watchdog reboot";
        return;
    }
    static const struct {
        uint32_t bits;
        const char *name;
    } k_map[] = {
        { POWMAN_CHIP_RESET_HAD_GLITCH_DETECT_BITS, "glitch detector" },
        { POWMAN_CHIP_RESET_HAD_BOR_BITS, "brown-out" },
        { POWMAN_CHIP_RESET_HAD_RUN_LOW_BITS, "RUN pin" },
        { POWMAN_CHIP_RESET_HAD_RESCUE_BITS, "rescue" },
        { POWMAN_CHIP_RESET_HAD_DP_RESET_REQ_BITS, "debugger reset" },
        { POWMAN_CHIP_RESET_HAD_HZD_SYS_RESET_REQ_BITS, "debugger system reset" },
        { POWMAN_CHIP_RESET_HAD_SWCORE_PD_BITS, "core power-down" },
    };
    for (unsigned i = 0; i < sizeof k_map / sizeof k_map[0]; i++) {
        if (s_raw & k_map[i].bits) {
            s_reason = k_map[i].name;
            return;
        }
    }
    s_reason = warm ? "core reset (debugger or software)" : "power on";
}

const char *reset_reason(void) { return s_reason; }

uint32_t reset_reason_raw(void) { return s_raw; }
