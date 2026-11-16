/*
 * Magnetometer calibration (plans/imu-auto-calibration.md, Phase 3), into
 * INSLIB's model: m = M * (raw - b), b the hard iron, M soft iron, scale and
 * (from the reference fit only) the rotation onto the IMU axes.
 *
 * 1. Ellipsoid fit: turned through many attitudes, a calibrated sensor
 *    traces a sphere of the WMM field strength F. Fits a symmetric M (no
 *    rotation: a sphere can't show one, so the symmetric square root is the
 *    one that rotates nothing, as in inslib_mag_calib.py) and the centre b,
 *    9 parameters, by least squares with a prior. Where a sample comes with
 *    the down direction (from the accelerometer), a second residual asks
 *    for the vertical component F sin(I), I the WMM inclination (the dip
 *    constraint, as in inslib_frame_align.align_by_dip). Without it, samples
 *    from turns on level ground (a ring) lie on a whole family of spheres
 *    and the heading comes out wrong; with it, the ring is enough.
 * 2. Reference fit: when the attitude is known without the magnetometer
 *    (INS yaw from a GNSS heading), the WMM vector rotated into the body
 *    frame is the true field, and  m_ref = M * raw + d  (d = -M b) is
 *    linear in 12 unknowns. Row i of M with d_i has the same regressor
 *    x = [raw, 1] for all three rows, so the sufficient statistics are one
 *    4x4 matrix and three right-hand sides. The ellipsoid result is the
 *    prior, so with no reference data the result is the ellipsoid's.
 *
 * Plain C with no Pico SDK headers, so it can be built and tested on a host.
 */
#ifndef IMU_CALIB_MAG_H
#define IMU_CALIB_MAG_H

#include <stdbool.h>
#include <stdint.h>

#define MCAL_NPAR 9

/* Ellipsoid parameters: the symmetric M and the centre. */
enum { MCAL_A00 = 0, MCAL_A11, MCAL_A22, MCAL_A01, MCAL_A02, MCAL_A12, MCAL_CX, MCAL_CY, MCAL_CZ };

/* One sample: the raw field and, if known, the body-frame down direction. */
typedef struct {
    float m[3];            /* raw [uT]                                      */
    float down[3];         /* unit vector, body frame                       */
    bool  has_down;
} mcal_sample_t;

typedef struct {
    float sd_diag_rel;     /* prior sigma of the M diagonal, relative to its prior mean */
    float sd_offdiag;      /* prior sigma of the M off-diagonal (relative)  */
    float sd_hard_ut;      /* prior sigma of the centre [uT]                */
    float sample_sd_ut;    /* sigma of one sample's magnitude residual [uT] */
    float dip_sd_ut;       /* sigma of one sample's vertical-component residual */
} mcal_prior_t;

typedef struct {
    float p[MCAL_NPAR];
    float sd[MCAL_NPAR];
    float s0;              /* prior mean of the M diagonal (1: the raw unit) */
    float field_ut;        /* F the fit was scaled to                       */
    float rms_before;      /* rms of |raw| - F [uT]                         */
    float rms_after;       /* rms of |M(raw - c)| - F                       */
    float rms_dip_ut;      /* rms of the vertical-component residual, -1 = none */
    int   n_down;          /* samples with a down direction                 */
    int   n;
    int   iterations;
    bool  ok;
} mcal_ell_t;

/* Sufficient statistics of the reference fit. Stored in flash as is. */
typedef struct {
    double s[10];          /* sum w*x*x^T, x = [raw, 1], upper triangle row by row */
    double r[3][4];        /* per row i: sum w*ref_i*x                      */
    double yy;             /* sum w*|ref|^2, for the residual               */
    double w;              /* sum w                                         */
    uint32_t n;
} mcal_ref_stats_t;

/* A calibration as INSLIB takes it. */
typedef struct {
    float m[9];            /* row-major                                     */
    float b[3];            /* hard iron [uT]                                */
    float sd_m;            /* largest posterior sigma of an M element       */
    float sd_b_ut;         /* largest posterior sigma of the hard iron      */
    float rms_ref_ut;      /* rms residual over the reference data, -1 = none */
    uint32_t n_ref;        /* reference samples used                        */
    bool  ok;
} mcal_cal_t;

typedef struct {
    float min_diag, max_diag;  /* M diagonal                                */
    float max_offdiag;
    float max_hard_ut;
    float max_rms_rel;         /* ellipsoid rms after, relative to F         */
} mcal_limits_t;

/* incl_rad: WMM inclination (positive down). */
void mcal_ellipsoid(const mcal_sample_t *s, int n, float field_ut, float incl_rad,
                    const mcal_prior_t *prior, mcal_ell_t *out);

void mcal_ref_clear(mcal_ref_stats_t *s);
/* raw: the sample, ref: the true field in the body frame [uT], sd its sigma
 * per axis. Returns false for a non-finite input. */
bool mcal_ref_add(mcal_ref_stats_t *s, const float raw[3], const float ref[3], float sd);
void mcal_ref_forget(mcal_ref_stats_t *s, float factor);
const char *mcal_ref_validate(const mcal_ref_stats_t *s);

/* The ellipsoid result refined by the reference data. sd_m: prior sigma of
 * every M element around the ellipsoid's (allows a rotation), sd_b_ut the
 * hard iron's. */
void mcal_ref_solve(const mcal_ref_stats_t *s, const mcal_ell_t *ell, float sd_m, float sd_b_ut,
                    mcal_cal_t *out);

/* NULL if usable, else a message. ell may be NULL (no rms check). */
const char *mcal_validate(const mcal_cal_t *c, const mcal_ell_t *ell, const mcal_limits_t *lim);

/* INSLIB's column-major misalignment and fixed bias. */
void mcal_to_inslib(const mcal_cal_t *c, float m_colmajor[9], float bias[3]);

#endif /* IMU_CALIB_MAG_H */
