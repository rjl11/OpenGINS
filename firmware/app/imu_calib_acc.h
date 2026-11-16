/*
 * Accelerometer calibration from static poses (plans/imu-auto-calibration.md,
 * Phase 2). imu_tk's method (Tedaldi, Pretto, Menegatti, ICRA 2014, as in
 * external/INSLIB/tools/inslib_imu_calib.py): in every still pose the
 * accelerometer sees only gravity, so the calibration must make the
 * corrected magnitude equal the local g. Only the magnitude enters, so the
 * poses can point anywhere.
 *
 * Model (INSLIB's REQ-NAV-037 form): a = M * (raw - b), with M upper
 * triangular (a rotation can't be seen from magnitudes, so it is fixed to
 * none), 9 parameters:
 *
 *     M = [ sx  mxy mxz ]      p = [sx sy sz mxy mxz myz bx by bz]
 *         [ 0   sy  myz ]
 *         [ 0   0   sz  ]
 *
 * The fit is Gauss-Newton with a prior on every parameter (identity, zero
 * bias), so a direction the poses don't reach keeps the prior: a unit that
 * only ever stands level learns roughly one combination of z scale and z
 * bias, and nothing else moves.
 *
 * Plain C with no Pico SDK headers, so it can be built and tested on a host.
 */
#ifndef IMU_CALIB_ACC_H
#define IMU_CALIB_ACC_H

#include <stdbool.h>
#include <stdint.h>

#include "imu_calib_lsq.h" /* calib_dir_coverage() */

#define ACAL_NPAR 9

/* Parameter order in p[] and sd[]. */
enum { ACAL_SX = 0, ACAL_SY, ACAL_SZ, ACAL_MXY, ACAL_MXZ, ACAL_MYZ, ACAL_BX, ACAL_BY, ACAL_BZ };

/* One static pose: the mean raw specific force and its sigma per axis. */
typedef struct {
    float acc[3];          /* m/s^2, raw (before any calibration)          */
    float sd;              /* sigma of the magnitude residual [m/s^2]      */
} acal_pose_t;

typedef struct {
    float sd_scale;        /* prior sigma of sx, sy, sz (around 1)         */
    float sd_mis;          /* prior sigma of mxy, mxz, myz [rad]           */
    float sd_bias;         /* prior sigma of bx, by, bz [m/s^2]            */
} acal_prior_t;

typedef struct {
    float p[ACAL_NPAR];    /* posterior mean                               */
    float sd[ACAL_NPAR];   /* posterior sigma                              */
    float rms_before;      /* rms of |raw| - g over the poses [m/s^2]      */
    float rms_after;       /* rms of |M(raw - b)| - g                      */
    int   iterations;
    bool  ok;              /* solved (false: no poses, or not converged)   */
} acal_result_t;

typedef struct {
    float max_scale_dev;   /* |s - 1| limit                                */
    float max_mis_rad;
    float max_bias;        /* m/s^2                                        */
} acal_limits_t;

/* Normal gravity at geodetic latitude and ellipsoid height (WGS84
 * Somigliana with the free-air term) [m/s^2]. */
float acal_gravity(double lat_rad, float h_m);

/* Solves for the 9 parameters from n poses (n may be 0: the prior). */
void acal_solve(const acal_pose_t *poses, int n, float g, const acal_prior_t *prior,
                acal_result_t *r);

/* NULL if the result is finite and within the limits, else a message. */
const char *acal_validate(const acal_result_t *r, const acal_limits_t *lim);

/* INSLIB's column-major 3x3 misalignment and the fixed bias from p. */
void acal_to_inslib(const float p[ACAL_NPAR], float m_colmajor[9], float bias[3]);

/* Coverage of the pose directions: the smallest eigenvalue of the mean of
 * u*u^T over the unit gravity directions u. 1/3 = spread over the whole
 * sphere, 0 = all in one plane (or one direction). */
float acal_coverage(const acal_pose_t *poses, int n);

#endif /* IMU_CALIB_ACC_H */
