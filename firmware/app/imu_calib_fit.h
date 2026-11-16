/*
 * Gyro bias vs temperature fit (plans/imu-auto-calibration.md, Phase 1).
 *
 * Model, per axis: b(T) = b0 + k * (T - T_ref). Each standstill chunk gives
 * one measured bias per axis at one temperature. The fit is a Bayesian
 * linear least squares with a zero-mean prior on b0 and k. The design
 * [1, T - T_ref] and the weights are the same for all three axes, so the
 * sufficient statistics are one 2x2 information matrix and three 2-vector
 * right-hand sides, and the posterior sigmas are the same for every axis.
 *
 * Plain C with no Pico SDK headers, so it can be built and tested on a host.
 */
#ifndef IMU_CALIB_FIT_H
#define IMU_CALIB_FIT_H

#include <stdbool.h>
#include <stdint.h>

/* Sufficient statistics of the chunks so far. Stored in flash as is. */
typedef struct {
    double info[3];        /* sum w*[1, d; d, d^2], d = T - T_ref: [00, 01, 11] */
    double rhs[3][2];      /* per axis: sum w*b*[1, d]                     */
    float  t_min_c;        /* temperature range seen                       */
    float  t_max_c;
    float  still_s;        /* total standstill time used [s]              */
    uint32_t n_chunks;
} gcal_stats_t;

typedef struct {
    float t_ref_c;         /* reference temperature of b0 [degC]           */
    float sd_b0_rps;       /* prior sigma of b0 [rad/s]                    */
    float sd_k_rps_c;      /* prior sigma of k [rad/s/degC]                */
    /* k is fitted only once the chunks span at least this much; below it
     * k = 0 and b0 alone is fitted. Over a small span, bias drift in time
     * would otherwise be read as a temperature slope. */
    float k_min_span_c;
} gcal_prior_t;

typedef struct {
    float b0[3];           /* bias at t_ref [rad/s]                        */
    float k[3];            /* slope [rad/s/degC]                           */
    float sd_b0;           /* posterior sigma, same for every axis [rad/s] */
    float sd_k;            /* [rad/s/degC]; the prior's while k is not fitted */
    bool  k_fitted;        /* the temperature span allowed fitting k       */
} gcal_result_t;

typedef struct {
    float max_b0_rps;
    float max_k_rps_c;
} gcal_limits_t;

void gcal_stats_clear(gcal_stats_t *s);

/* One chunk: bias[3] measured at temp_c with variance var (rad/s)^2 per
 * axis. Returns false (nothing added) for a non-finite input or var <= 0. */
bool gcal_stats_add(gcal_stats_t *s, const gcal_prior_t *p, const float bias[3], float temp_c,
                    float var);

/* Scales the information by factor (0..1], so old chunks count less. */
void gcal_stats_forget(gcal_stats_t *s, float factor);

/* Posterior mean and sigmas. With no chunks the result is the prior. */
void gcal_solve(const gcal_stats_t *s, const gcal_prior_t *p, gcal_result_t *r);

/* NULL if the result is finite and within the limits, else a message. */
const char *gcal_validate(const gcal_result_t *r, const gcal_limits_t *lim);

/* NULL if the stored statistics are usable (finite, positive diagonal). */
const char *gcal_stats_validate(const gcal_stats_t *s);

/* Earth rotation rate in the body frame [rad/s], from roll, pitch, yaw
 * (NED, ZYX Euler) and the geodetic latitude. */
void gcal_earth_rate_body(const float rpy_rad[3], double lat_rad, float out[3]);

#endif /* IMU_CALIB_FIT_H */
