/* Host test for firmware/app/front_imu.c: timing with and without the clock
 * sync, the mounting rotation, lost batches, the chassis tilt filter.
 *
 * gcc -O2 -Wall -Wextra -I firmware/app -I firmware/drivers/flow_io -I common \
 *     firmware/app/front_imu.c firmware/tools/test_front_imu.c -lm -o temp/test_front_imu
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "front_imu.h"

static int fails;
#define CHECK(c)                                                         \
    do {                                                                 \
        if (!(c)) {                                                      \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c);          \
            fails++;                                                     \
        }                                                                \
    } while (0)
#define CHECK_NEAR(a, b, tol)                                                                          \
    do {                                                                                               \
        double a_ = (a), b_ = (b);                                                                     \
        if (!(fabs(a_ - b_) <= (tol))) {                                                               \
            printf("FAIL %s:%d: %s = %.6f, want %.6f +- %.3g\n", __FILE__, __LINE__, #a, a_, b_, (double)(tol)); \
            fails++;                                                                                   \
        }                                                                                              \
    } while (0)

#define D2R (3.14159265358979323846 / 180.0)
#define G   9.80665

static front_imu_config_t cfg(void)
{
    front_imu_config_t c = { .mount_rpy_rad = { 0, 0, 0 }, .tilt_tau_s = 0.5f, .acc_gate = 0.05f, .free_latency_us = 1500u };
    return c;
}

/* Specific force at rest for this roll and pitch (FRD). */
static void rest_acc(double roll, double pitch, float a[3])
{
    a[0] = (float)(G * sin(pitch));
    a[1] = (float)(-G * sin(roll) * cos(pitch));
    a[2] = (float)(-G * cos(roll) * cos(pitch));
}

/* A stream of batches of 8 at 800 Hz, the unit clock starting just before
 * a wrap; arrivals late by 1.5 .. 4.5 ms, the least every 5th batch. */
static void test_free_running(void)
{
    front_imu_t f;
    front_imu_config_t c = cfg();
    front_imu_init(&f, &c);
    const uint64_t pico_off = 7000000u;
    uint32_t       unit = 0xFFFF0000u;
    uint64_t       unit_abs = 0;
    front_imu_sample_t out[FLOW_IMU_BATCH_MAX];
    double         worst = 0;
    uint64_t       last = 0;
    for (int b = 0; b < 400; b++) {
        flow_msg_imu_batch_t m;
        memset(&m, 0, sizeof m);
        m.t0_us = unit;
        m.seq = (uint16_t)b;
        m.n = 8;
        float a[3];
        rest_acc(0, 0, a);
        for (int i = 0; i < 8; i++) {
            m.s[i].dt_us = (uint16_t)(i * 1250);
            memcpy(m.s[i].acc, a, sizeof a);
        }
        const uint64_t last_abs = unit_abs + 7u * 1250u;
        const uint64_t late = 1500u + (b % 5 == 0 ? 0u : (uint64_t)(b * 37 % 3000));
        const unsigned n = front_imu_batch(&f, &m, pico_off + last_abs + late, out);
        CHECK(n == 8);
        for (unsigned i = 0; i < n; i++) {
            CHECK(out[i].t_us > last);
            last = out[i].t_us;
            if (b > 5) {
                const double err = (double)out[i].t_us - (double)(pico_off + unit_abs + i * 1250u);
                if (fabs(err) > worst) worst = fabs(err);
            }
        }
        unit += 10000u;
        unit_abs += 10000u;
    }
    /* The leak creeps the offset up by 200 ppm between the least-late
     * batches (every 50 ms): about 10 us at most. */
    CHECK(worst <= 12.0);
    CHECK(f.st.batches == 400 && f.st.samples == 3200 && f.st.lost == 0 && f.st.resets == 0 && f.st.synced == 0);
    CHECK(front_imu_fresh(&f, last + 1000u, 100000u));
    CHECK(!front_imu_fresh(&f, last + 200000u, 100000u));
    printf("free-running: worst time error %.1f us\n", worst);

    /* Two batches lost. */
    flow_msg_imu_batch_t m;
    memset(&m, 0, sizeof m);
    m.t0_us = unit + 20000u;
    m.seq = 402;
    m.n = 1;
    front_imu_batch(&f, &m, pico_off + unit_abs + 20000u + 1500u, out);
    CHECK(f.st.lost == 2);
}

static bool fake_clock(void *ctx, uint32_t unit_us, uint64_t now, uint64_t *pico)
{
    (void)now;
    *pico = *(const uint64_t *)ctx + unit_us;
    return true;
}

static void test_synced_and_mount(void)
{
    front_imu_t f;
    front_imu_config_t c = cfg();
    c.mount_rpy_rad[2] = (float)(90 * D2R); /* TM171 x points to the right */
    front_imu_init(&f, &c);
    uint64_t base = 3000000u;
    front_imu_set_clock(&f, fake_clock, &base);
    flow_msg_imu_batch_t m;
    memset(&m, 0, sizeof m);
    m.t0_us = 1000000u;
    m.n = 2;
    m.s[1].dt_us = 1250;
    m.s[0].gyr[0] = m.s[1].gyr[0] = 0.5f;
    m.s[0].acc[0] = m.s[1].acc[0] = 2.0f;
    front_imu_sample_t out[FLOW_IMU_BATCH_MAX];
    CHECK(front_imu_batch(&f, &m, 4005000u, out) == 2);
    CHECK(out[0].t_us == 4000000u && out[1].t_us == 4001250u);
    CHECK_NEAR(out[1].dt_s, 0.00125, 1e-9);
    CHECK_NEAR(out[0].gyr[0], 0, 1e-6);
    CHECK_NEAR(out[0].gyr[1], 0.5, 1e-6);
    CHECK_NEAR(out[0].acc[1], 2.0, 1e-6);
    CHECK(f.st.synced == 1);
}

static void test_tilt(void)
{
    front_imu_t f;
    front_imu_config_t c = cfg();
    front_imu_init(&f, &c);
    uint64_t base = 0;
    front_imu_set_clock(&f, fake_clock, &base);
    float a[3];
    front_imu_sample_t out[FLOW_IMU_BATCH_MAX];
    flow_msg_imu_batch_t m;
    float r, p;
    CHECK(!front_imu_tilt(&f, 0, 500000u, &r, &p));
    /* Start level, then the chassis stands at roll 3, pitch -2 deg: the
     * filter follows within a few time constants. */
    uint32_t t = 1000u;
    for (int b = 0; b < 400; b++) {
        memset(&m, 0, sizeof m);
        m.t0_us = t;
        m.seq = (uint16_t)b;
        m.n = 8;
        rest_acc(b < 10 ? 0 : 3 * D2R, b < 10 ? 0 : -2 * D2R, a);
        for (int i = 0; i < 8; i++) {
            m.s[i].dt_us = (uint16_t)(i * 1250);
            memcpy(m.s[i].acc, a, sizeof a);
        }
        front_imu_batch(&f, &m, t + 10000u, out);
        if (b == 9) {
            CHECK(front_imu_tilt(&f, t + 9000u, 500000u, &r, &p));
            CHECK_NEAR(r, 0, 1e-6);
        }
        t += 10000u;
    }
    CHECK(front_imu_tilt(&f, t, 500000u, &r, &p));
    CHECK_NEAR(r, 3 * D2R, 1e-4);
    CHECK_NEAR(p, -2 * D2R, 1e-4);

    /* A roll rate with the accelerometer off the 1 g gate (a hard turn):
     * the gyro alone carries the roll. 0.1 rad/s for 0.5 s = +0.05 rad. */
    const float r0 = r;
    for (int b = 0; b < 50; b++) {
        memset(&m, 0, sizeof m);
        m.t0_us = t;
        m.seq = (uint16_t)(400 + b);
        m.n = 8;
        for (int i = 0; i < 8; i++) {
            m.s[i].dt_us = (uint16_t)(i * 1250);
            m.s[i].gyr[0] = 0.1f;
            m.s[i].acc[1] = 4.0f; /* |a| 8 % above 1 g: outside the gate */
            m.s[i].acc[2] = (float)-G;
        }
        front_imu_batch(&f, &m, t + 10000u, out);
        t += 10000u;
    }
    CHECK(front_imu_tilt(&f, t, 500000u, &r, &p));
    CHECK_NEAR(r - r0, 0.05, 0.002);
    CHECK(!front_imu_tilt(&f, t + 600000u, 500000u, &r, &p)); /* stale */
}

int main(void)
{
    test_free_running();
    test_synced_and_mount();
    test_tilt();
    printf(fails ? "test_front_imu: %d FAILED\n" : "test_front_imu: OK\n", fails);
    return fails ? EXIT_FAILURE : EXIT_SUCCESS;
}
