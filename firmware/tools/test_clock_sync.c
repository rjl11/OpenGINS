/*
 * Host test for app/clock_sync.c: a simulated unit clock with an offset and
 * a drift, network delays with a floor and a long tail, and a unit reboot.
 *
 *   gcc -O2 -Wall -Wextra -I firmware/app firmware/tools/test_clock_sync.c firmware/app/clock_sync.c -lm -o temp/test_clock_sync && temp/test_clock_sync
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "clock_sync.h"

static int s_fail;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); s_fail++; } } while (0)

/* The unit's clock at Pico time t: started at boot_t with drift ppm. */
static double g_drift = 40e-6;
static uint64_t g_unit0 = 5000000000ull; /* unit clock at Pico 0 */
static uint64_t unit_at(uint64_t t) { return g_unit0 + (uint64_t)llround((double)t * (1.0 + g_drift)); }

static uint32_t rnd(uint32_t n) { return (uint32_t)(rand() % (int)n); }

/* One request/reply exchange at Pico time t. Delays: 150 us floor each way,
 * plus a tail: usually a little, sometimes several ms (a slow poll). */
static void exchange(clock_sync_t *c, uint64_t t, bool asym_spike)
{
    uint32_t seq = clock_sync_request(c, t);
    uint32_t up = 150 + rnd(40) + (rnd(10) == 0 ? rnd(5000) : 0);
    uint32_t turn = 30 + rnd(200);
    uint32_t down = 150 + rnd(40) + (rnd(5) == 0 ? rnd(1000) : 0);
    if (asym_spike) up += 3000;
    uint64_t t_unit_rx = t + up;
    clock_sync_reply(c, seq, turn, unit_at(t_unit_rx), t_unit_rx + turn + down);
}

int main(void)
{
    srand(1);
    clock_sync_t c;
    clock_sync_init(&c);
    CHECK(!clock_sync_valid(&c, 0));

    /* 1. 60 s at 1 Hz. */
    uint64_t t = 1000000;
    for (int i = 0; i < 60; i++, t += 1000000) {
        exchange(&c, t, i % 7 == 3);
    }
    CHECK(clock_sync_valid(&c, t));
    /* A frame stamped by the unit at Pico time tf maps back to tf. */
    double worst = 0;
    for (int k = 0; k < 50; k++) {
        uint64_t tf = t - 20000000 + (uint64_t)k * 400000;
        uint64_t p;
        CHECK(clock_sync_to_pico(&c, (uint32_t)unit_at(tf), t, &p));
        double e = fabs((double)(int64_t)(p - tf));
        if (e > worst) worst = e;
    }
    printf("  error over the last 20 s: %.0f us (drift fit %.1f ppm, true %.1f; rtt min %u, %u used, rms %.0f us)\n",
           worst, clock_sync_drift_ppm(&c), -g_drift * 1e6, c.rtt_min, c.used, (double)c.resid_us);
    CHECK(worst < 100.0);
    /* The offset is Pico - unit: a unit clock running fast makes it fall. */
    CHECK(fabs(clock_sync_drift_ppm(&c) + g_drift * 1e6) < 5.0);

    /* 2. Wrap of the low 32 bits: put the unit clock just before a wrap. */
    g_unit0 = 0xFFFFFFFFull * 3 - 30000000ull - t; /* wraps 30 s from now */
    clock_sync_t w;
    clock_sync_init(&w);
    for (int i = 0; i < 50; i++, t += 1000000) exchange(&w, t, false);
    CHECK(w.resets == 0);
    uint64_t p;
    uint64_t tf = t - 5000000; /* after the wrap */
    CHECK(clock_sync_to_pico(&w, (uint32_t)unit_at(tf), t, &p));
    CHECK(fabs((double)(int64_t)(p - tf)) < 100.0);
    tf = t - 40000000; /* before the wrap */
    CHECK(clock_sync_to_pico(&w, (uint32_t)unit_at(tf), t, &p));
    CHECK(fabs((double)(int64_t)(p - tf)) < 100.0);

    /* 3. The unit reboots: its clock starts near 0. The sync starts over
     * and is right again a few replies later. */
    g_unit0 = 2000000ull - (uint64_t)((double)t * (1.0 + g_drift));
    uint32_t r0 = w.resets;
    for (int i = 0; i < 10; i++, t += 1000000) exchange(&w, t, false);
    CHECK(w.resets == r0 + 1);
    tf = t - 2000000;
    CHECK(clock_sync_to_pico(&w, (uint32_t)unit_at(tf), t, &p));
    CHECK(fabs((double)(int64_t)(p - tf)) < 200.0);

    /* 4. Stale after 10 s without replies; lost request times out; a late
     * reply is not taken. */
    CHECK(!clock_sync_valid(&w, t + CLOCK_SYNC_STALE_US + 1));
    uint32_t s = clock_sync_request(&w, t);
    CHECK(clock_sync_pending(&w, t + 1000));
    CHECK(!clock_sync_pending(&w, t + CLOCK_SYNC_TIMEOUT_US + 1));
    CHECK(w.timeouts == 1);
    CHECK(!clock_sync_reply(&w, s, 50, unit_at(t), t + CLOCK_SYNC_TIMEOUT_US + 10));
    CHECK(w.stray == 1);

    if (s_fail == 0) {
        printf("test_clock_sync: all passed\n");
        return 0;
    }
    printf("test_clock_sync: %d failed\n", s_fail);
    return 1;
}
