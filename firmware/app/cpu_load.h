/* CPU load of a polling main loop, for the status report.
 *
 * Both cores run a loop that never sleeps, so there is no idle task to time.
 * Instead, the meter times every pass of the loop, and the loop says for
 * each pass whether it did work (a sample handled, a report printed). A pass
 * that did no work is an idle pass. Most idle passes take about the same
 * time. The cost of one idle pass is the mean of the typical ones (those no
 * longer than twice the median; the rest were hit by an interrupt or did
 * hidden work), and load is the share of the window not explained by that
 * many idle passes:
 *
 *     load = 1 - idle_passes * typical_idle_pass / window
 *
 * So every work pass counts as busy in full, and so does whatever makes an
 * idle pass longer than typical: interrupt handlers that land in it
 * (UART DMA, USB, CYW43 background work), and hidden work in a call the loop
 * doesn't flag. Polling and finding nothing counts as idle, because a loop
 * that slept there would lose nothing.
 *
 * Not the minimum: the shortest pass is an outlier (a lucky run through the
 * flash cache), and a change that only shortens it moved a minimum-based
 * figure by several points with no change in the work done. Not a bin
 * centre either: the histogram also keeps each bin's exact cycles, because
 * rounding a 150-cycle pass to an 8-cycle bin, times 900 000 passes a second,
 * moves the result by a few percent.
 *
 * Idle passes take a few microseconds, so they are timed with the core's own
 * SysTick at the processor clock: the RP2350's Cortex-M33 has no DWT cycle
 * counter (DWT_CTRL.NOCYCCNT is set). Each core has its own SysTick, so
 * cpu_load_init() and cpu_load_pass() must run on the core being measured.
 * The SysTick wraps every 2^24 cycles (63 ms at 266 MHz); a pass that
 * time_us_64() shows long goes to the histogram's top bin instead, so a wrap
 * can't fake a short one. If FreeRTOS later takes the SysTick, use its idle
 * hook instead.
 *
 * Header only, so the functions inline into nav_core1.c and stay in SRAM.
 */
#ifndef CPU_LOAD_H
#define CPU_LOAD_H

#include <stdbool.h>
#include <stdint.h>

#include "hardware/structs/systick.h"
#include "pico/time.h"

#define CPU_LOAD_BIN_SHIFT 3u   /* histogram bins of 8 cycles (30 ns at 266 MHz) */
#define CPU_LOAD_BINS      512u /* up to 4096 cycles (27 us); longer goes to the top bin */

typedef struct {
    float    cyc_per_us;    /* processor clock / 1e6                   */
    uint32_t last_cvr;      /* SysTick at the start of the last pass   */
    uint64_t last_us;
    uint64_t win_start_us;
    uint32_t idle;          /* idle passes in this window              */
    uint32_t work;          /* work passes in this window              */
    uint32_t min_cyc;       /* shortest idle pass                      */
    uint32_t max_us;        /* longest pass of either kind             */
    uint32_t hist[CPU_LOAD_BINS]; /* idle pass durations                */
    uint32_t frac[CPU_LOAD_BINS]; /* sum of the cycles below each bin's start */
} cpu_load_t;

typedef struct {
    uint16_t load_permille;  /* 0..1000                                  */
    uint32_t max_pass_us;    /* longest loop pass in the window          */
    uint32_t idle_pass_ns;   /* typical idle pass (the cost used)        */
    uint32_t idle_min_ns;    /* shortest idle pass                       */
    uint32_t idle_per_s;     /* idle passes per second                   */
    uint32_t work_per_s;     /* work passes per second                   */
} cpu_load_result_t;

static inline void cpu_load_clear(cpu_load_t *c)
{
    c->idle = 0;
    c->work = 0;
    c->min_cyc = UINT32_MAX;
    c->max_us = 0;
    for (uint32_t i = 0; i < CPU_LOAD_BINS; i++) {
        c->hist[i] = 0;
        c->frac[i] = 0;
    }
}

/* Starts this core's SysTick (free running at the processor clock) and the
 * first window. clk_hz is clock_get_hz(clk_sys), read by the caller because
 * core 1 keeps clock code out of its SRAM path. */
static inline void cpu_load_init(cpu_load_t *c, uint32_t clk_hz)
{
    systick_hw->csr = 0;
    systick_hw->rvr = 0x00FFFFFFu;
    systick_hw->cvr = 0; /* any write clears it; it reloads on the next tick */
    systick_hw->csr = M33_SYST_CSR_CLKSOURCE_BITS | M33_SYST_CSR_ENABLE_BITS;

    c->cyc_per_us = (float)clk_hz * 1e-6f;
    c->last_cvr = systick_hw->cvr;
    c->last_us = time_us_64();
    c->win_start_us = c->last_us;
    cpu_load_clear(c);
}

/* Call once at the top of every loop pass. worked says whether the pass
 * that just ended did work. */
static inline void cpu_load_pass(cpu_load_t *c, bool worked)
{
    uint32_t cvr = systick_hw->cvr;
    uint64_t us = time_us_64();
    uint32_t dc = (c->last_cvr - cvr) & 0x00FFFFFFu; /* counts down */
    uint32_t du = (uint32_t)(us - c->last_us);
    c->last_cvr = cvr;
    c->last_us = us;

    if (du > c->max_us) {
        c->max_us = du;
    }
    if (worked) {
        c->work++;
        return;
    }
    c->idle++;
    uint32_t bin = du < 50u ? dc >> CPU_LOAD_BIN_SHIFT : CPU_LOAD_BINS - 1u;
    if (bin >= CPU_LOAD_BINS) {
        bin = CPU_LOAD_BINS - 1u;
    }
    c->hist[bin]++;
    if (bin < CPU_LOAD_BINS - 1u) {
        c->frac[bin] += dc & ((1u << CPU_LOAD_BIN_SHIFT) - 1u);
    }
    if (du < 50u && dc < c->min_cyc) {
        c->min_cyc = dc;
    }
}

/* Closes the window that started at the last take (or init), writes its
 * figures and starts the next one. */
static inline void cpu_load_take(cpu_load_t *c, uint64_t now_us, cpu_load_result_t *r)
{
    uint32_t win_us = (uint32_t)(now_us - c->win_start_us);
    float win_cyc = (float)win_us * c->cyc_per_us;

    /* The median's bin, then the exact mean of the passes up to twice it. */
    float typical_cyc = 0.0f;
    if (c->idle > 0) {
        uint32_t half = (c->idle + 1u) / 2u, seen = 0, mb = 0;
        for (uint32_t i = 0; i < CPU_LOAD_BINS; i++) {
            seen += c->hist[i];
            if (seen >= half) {
                mb = i;
                break;
            }
        }
        uint32_t lim = 2u * mb + 1u;
        if (lim > CPU_LOAD_BINS - 2u) {
            lim = CPU_LOAD_BINS - 2u;
        }
        float n = 0.0f, cyc = 0.0f;
        for (uint32_t i = 0; i <= lim; i++) {
            n += (float)c->hist[i];
            cyc += (float)c->hist[i] * (float)(i << CPU_LOAD_BIN_SHIFT) + (float)c->frac[i];
        }
        if (n > 0.0f) {
            typical_cyc = cyc / n;
        }
    }

    float load = 1.0f;
    if (win_cyc > 0.0f) {
        load = 1.0f - (float)c->idle * typical_cyc / win_cyc;
    }
    if (load < 0.0f) {
        load = 0.0f;
    }
    if (load > 1.0f) {
        load = 1.0f;
    }

    const float ns_per_cyc = 1000.0f / c->cyc_per_us;
    const float per_s = win_us ? 1e6f / (float)win_us : 0.0f;
    r->load_permille = (uint16_t)(load * 1000.0f + 0.5f);
    r->max_pass_us = c->max_us;
    r->idle_pass_ns = (uint32_t)(typical_cyc * ns_per_cyc + 0.5f);
    r->idle_min_ns = c->min_cyc == UINT32_MAX ? 0u : (uint32_t)((float)c->min_cyc * ns_per_cyc + 0.5f);
    r->idle_per_s = (uint32_t)((float)c->idle * per_s + 0.5f);
    r->work_per_s = (uint32_t)((float)c->work * per_s + 0.5f);

    c->win_start_us = now_us;
    cpu_load_clear(c);
}

#endif
