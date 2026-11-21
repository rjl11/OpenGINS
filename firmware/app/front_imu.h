/*
 * front_imu: the front camera unit's TM171 on the Pico (core 0), from its
 * FLOW_MSG_IMU_BATCH messages (common/flow_link.h). Two jobs
 * (plans/front-camera-unit.md, "The TM171's three jobs"):
 *
 *  - the backup IMU for the roof TM210: each sample on the Pico clock, in
 *    body axes (the TM171 mounting from the web settings), which main.c
 *    hands to core 1 when it switches the source;
 *  - the chassis roll and pitch for the cab-roll ground point: a
 *    complementary filter on the same samples (gyro, pulled towards the
 *    accelerometer's level with time constant tilt_tau_s, and only while the
 *    specific force is within acc_gate of 1 g).
 *
 * Time: the unit's clock is mapped onto the Pico's with the clock sync when
 * it is valid (the callback set with front_imu_set_clock(), eth.c);
 * otherwise (the bench path over USB, or before the sync settles) by the
 * smallest arrival minus unit time, creeping up by FRONT_IMU_LEAK_PPM, as
 * flow_nav.c does for free-running frames, with free_latency_us taken off.
 *
 * Plain C, no SDK calls: host test firmware/tools/test_front_imu.c.
 */
#ifndef FRONT_IMU_H
#define FRONT_IMU_H

#include <stdbool.h>
#include <stdint.h>

#include "flow_link.h"
#include "flow_nav.h" /* flow_nav_clock_fn */

#define FRONT_IMU_LEAK_PPM   200u
#define FRONT_IMU_MAX_LAG_US 200000u

/* One sample on the Pico clock, body FRD, SI. */
typedef struct {
    uint64_t t_us;
    float    dt_s; /* since the sample before */
    float    gyr[3];
    float    acc[3];
} front_imu_sample_t;

typedef struct {
    float    mount_rpy_rad[3]; /* TM171 -> body, Rz Ry Rx */
    float    tilt_tau_s;       /* complementary filter time constant */
    float    acc_gate;         /* accelerometer used within this fraction of 1 g */
    uint32_t free_latency_us;  /* without the clock sync: sample to arrival, least */
} front_imu_config_t;

typedef struct {
    uint32_t batches, samples;
    uint32_t lost;       /* batches missing by their sequence numbers */
    uint32_t unit_gaps;  /* batches the unit flagged FLOW_IMU_F_GAP   */
    uint32_t clamped;    /* batches with a clamped value              */
    uint32_t synced;     /* batches timed by the clock sync           */
    uint32_t resets;     /* free-running offset started over          */
    uint32_t bad;        /* not decodable, or time running backwards  */
} front_imu_stats_t;

typedef struct {
    front_imu_config_t cfg;
    float              r_bm[9]; /* TM171 -> body, row-major */
    flow_nav_clock_fn  clock;
    void              *clock_ctx;

    bool     have_seq;
    uint16_t last_seq;
    uint64_t last_t_us; /* Pico time of the last sample handed out */

    bool     free_have;
    uint32_t free_off;
    uint64_t free_leak_t;

    bool     tilt_have;
    float    roll, pitch; /* chassis, body FRD convention [rad] */
    uint64_t tilt_t_us;

    front_imu_stats_t st;
} front_imu_t;

void front_imu_init(front_imu_t *f, const front_imu_config_t *cfg);
void front_imu_set_clock(front_imu_t *f, flow_nav_clock_fn fn, void *ctx);
/* New mounting (web settings); the tilt starts over. */
void front_imu_set_mount(front_imu_t *f, const float rpy_rad[3]);

/* One batch, arrived at t_arrival_us. Writes up to FLOW_IMU_BATCH_MAX
 * samples (body FRD, Pico clock) and returns how many. */
unsigned front_imu_batch(front_imu_t *f, const flow_msg_imu_batch_t *b, uint64_t t_arrival_us,
                         front_imu_sample_t *out);

/* The last sample is younger than max_age_us. */
bool front_imu_fresh(const front_imu_t *f, uint64_t now, uint64_t max_age_us);

/* The chassis roll and pitch [rad], if the filter runs and is fresh. */
bool front_imu_tilt(const front_imu_t *f, uint64_t now, uint64_t max_age_us, float *roll, float *pitch);

#endif /* FRONT_IMU_H */
