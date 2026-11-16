#include "imu_calib.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "hardware/flash.h"
#include "pico/time.h"

#include "app_config.h"
#include "flash_rec.h"
#include "magnetic_model.h"
#include "nav_core1.h"
#include "nav_suite.h"

#define REC_MAGIC        0x4C435347u /* "GSCL" */
/* 2: pose bins added (Phase 2). 3: magnetometer bins, reference statistics
 * and the full GNSS position (Phase 3). 4: gyro matrix and its statistics
 * (Phase 4). 5: hold flags and the stored sets (Phase 5). An older record
 * is ignored. */
#define REC_VERSION      5
/* The two sectors below the settings (settings.c uses the last two). */
#define REC_SECTOR       (PICO_FLASH_SIZE_BYTES - 4u * FLASH_SECTOR_SIZE)

#define WINDOW_US        200000u
#define WINDOW_MIN_N     8u
#define DEG2RAD          0.017453292519943295
#define RAD2DEG_F        57.29577951f
#define G_NOMINAL        9.80665f

/* Pose bins: 6 cube faces (dominant axis and sign of the gravity direction)
 * x 5 (a centre patch, then 4 quadrants around it). */
#define POSE_BINS        (6 * 5)
#define POSE_CENTRE_SIN  0.342f /* sin 20 deg */

/* Magnetometer bins: 6 cube faces x 3 x 3 cells, by raw direction. */
#define MAG_BINS         (6 * 9)

typedef struct {
    float    m[3];         /* raw [uT], mean of a group                   */
    float    down[3];      /* body-frame down, if has_down                */
    uint16_t boot;
    uint8_t  valid;
    uint8_t  has_down;
} mag_bin_t;

typedef struct {
    float    acc[3];       /* mean raw specific force [m/s^2]             */
    float    dur_s;
    float    temp_c;
    uint16_t boot;         /* payload_t.boots when it was taken           */
    uint8_t  valid;
    uint8_t  pad;
} pose_bin_t;

/* Flash payload. */
typedef struct {
    gcal_stats_t stats;
    double   lat_rad;      /* last GNSS latitude                          */
    float    h_m;          /* last GNSS ellipsoid height                  */
    uint16_t node_id;      /* TM210 the statistics belong to              */
    uint8_t  id_valid;     /* node_id is set                              */
    uint8_t  lat_valid;
    uint32_t boots;        /* boots that loaded this record               */
    pose_bin_t bins[POSE_BINS];
    /* Phase 3. lat_valid also says lon_rad and year are set. */
    double   lon_rad;
    float    year;         /* decimal year of the last fix (WMM epoch)    */
    mag_bin_t mag[MAG_BINS];
    mcal_ref_stats_t ref;
    /* Phase 4: the gyro matrix in use (row-major; all zero = identity, none
     * learned yet) and the statistics linearised at it. */
    float    gyr_m0[9];
    rot_stats_t rot;
    /* Phase 5: per group (IMU_CALIB_*) the hold flag, and the set in use for
     * the gyro bias, accelerometer and magnetometer (the gyro matrix is
     * gyr_m0): what a held group, or one without usable data, gets at boot. */
    uint8_t  hold[IMU_CALIB_GROUPS];
    uint8_t  keep_valid[3];
    uint8_t  pad5;
    gcal_result_t keep_gyr;
    acal_result_t keep_acc;
    mcal_cal_t    keep_mag;
} payload_t;

_Static_assert(sizeof(payload_t) <= FLASH_REC_MAX_PAYLOAD, "calibration must fit in one flash record");

static flash_rec_t s_rec;
static payload_t s_p;               /* statistics being learned            */
static gcal_prior_t s_prior;
static gcal_limits_t s_lim;
static acal_prior_t s_aprior;
static acal_limits_t s_alim;
static imu_calib_status_t s_st;
static float s_gyr_psd, s_acc_psd;
static uint64_t s_last_save_us;
static bool s_learn;                /* identity known: chunks are used     */

/* 0.2 s window (the same statistic as INSLIB's stillness gate). */
static struct {
    uint32_t n;
    uint64_t t0;
    float mean[6], m2[6];
    float dt_s;
} s_win;

/* A window inside the run: added to the chunk and the pose only once the
 * next window also passes, so the last window before motion is dropped. */
static struct {
    bool valid;
    float mean[6];                  /* gyro, accelerometer                 */
    uint32_t n;
    float dt_s;
    float temp_c;
} s_pend;

static bool s_run;                  /* inside a run of still windows       */
static struct {
    double sum[3];                  /* sum of n * window mean (gyro)       */
    uint32_t n;
    float dt_s;
    float t_min, t_max;
    double t_sum;                   /* sum of n * temperature              */
} s_chunk;
static struct {
    double sum[3];                  /* sum of n * window mean (accel)      */
    uint32_t n;
    float dt_s;
    double t_sum;
    bool done;                      /* committed for this run              */
} s_pose;

/* Magnetometer (Phase 3). */
static mcal_prior_t s_mprior;
static mcal_limits_t s_mlim;
static unsigned s_platform;
static struct {
    double m[3], a[3];
    uint32_t n;
    bool down_ok;                   /* every sample in the group was quasi-static */
} s_mgrp;
static float s_last_mag[3];         /* latest raw sample and when           */
static uint64_t s_last_mag_us;
static float s_last_rate_dps;       /* |gyro| of the latest IMU sample      */
static float s_last_gz_dps;
static bool s_mag_dirty;            /* bins changed: solve again            */
static uint64_t s_mag_solved_us;
static uint64_t s_ref_last_us, s_full_since_us, s_yaw_seen_us, s_nav_t_us;
static uint32_t s_n_yaw_prev;
static bool s_fix_this_boot;
static float s_mm_use[9];           /* this boot's magnetometer correction (column-major) */

/* Gyro matrix (Phase 4). */
static float s_gm_use[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 }; /* row-major, in use */
static rot_preint_t s_rot;
typedef enum { ROT_IDLE = 0, ROT_ARMED, ROT_FROZEN } rot_state_t;
static rot_state_t s_rot_state;
static float s_rot_g1[3];
static struct {
    double sum[3];                  /* accel, sum of n * window mean over the run */
    uint32_t n;
    float dt_s;
} s_runacc;
static struct {
    bool active;
    float psi0, sd0, pred;
    float h[ROT_NPAR];
    uint64_t t0, t_last;
} s_turn;
static float s_nav_rpy[3];
static bool s_nav_rpy_ok;
static float s_mb_use[3];

static float s_k_use[3];            /* this boot's gyro correction         */
static float s_b0_use[3];
static float s_am_use[9];           /* this boot's accelerometer correction */
static float s_ab_use[3];

/* ---- Fits ---------------------------------------------------------------- */

static float local_g(void)
{
    double lat = s_p.lat_valid ? s_p.lat_rad : APP_CALIB_DEFAULT_LAT_DEG * DEG2RAD;
    return acal_gravity(lat, s_p.lat_valid ? s_p.h_m : 0.0f);
}

static int collect_poses(acal_pose_t *out)
{
    int n = 0;
    const float floor2 = APP_CALIB_ACC_FLOOR_MPS2 * APP_CALIB_ACC_FLOOR_MPS2;
    for (int b = 0; b < POSE_BINS; b++) {
        const pose_bin_t *pb = &s_p.bins[b];
        if (!pb->valid) {
            continue;
        }
        for (int i = 0; i < 3; i++) {
            out[n].acc[i] = pb->acc[i];
        }
        out[n].sd = sqrtf(s_acc_psd / pb->dur_s + floor2);
        n++;
    }
    return n;
}

static void solve_acc(acal_result_t *r, int *n_out, float *cov_out)
{
    acal_pose_t poses[POSE_BINS];
    int n = collect_poses(poses);
    acal_solve(poses, n, local_g(), &s_aprior, r);
    if (n_out) *n_out = n;
    if (cov_out) *cov_out = acal_coverage(poses, n);
}

/* WMM at the stored position: total field, inclination, NED vector. */
static bool wmm(float *f, float *incl, float b_ned[3])
{
    if (!s_p.lat_valid) {
        return false;
    }
    const float lat = (float)(s_p.lat_rad / DEG2RAD), lon = (float)(s_p.lon_rad / DEG2RAD);
    *f = magnetic_field_strength_uT(lat, lon);
    *incl = (float)(magnetic_inclination_deg(lat, lon) * DEG2RAD);
    if (b_ned) {
        magnetic_field_ned_uT(lat, lon, s_p.year, b_ned);
    }
    return *f > 1.0f && isfinite(*f) && isfinite(*incl);
}

static int collect_mag(mcal_sample_t *out)
{
    int n = 0;
    for (int b = 0; b < MAG_BINS; b++) {
        const mag_bin_t *mb = &s_p.mag[b];
        if (!mb->valid) {
            continue;
        }
        for (int i = 0; i < 3; i++) {
            out[n].m[i] = mb->m[i];
            out[n].down[i] = mb->down[i];
        }
        out[n].has_down = mb->has_down != 0;
        n++;
    }
    return n;
}

/* Ellipsoid plus reference. Returns false (nothing solved) without a
 * position for the WMM or with too few samples. */
static bool solve_mag(mcal_ell_t *ell, mcal_cal_t *cal, int *n_out, float *cov_out)
{
    mcal_sample_t smp[MAG_BINS];
    int n = collect_mag(smp);
    if (n_out) *n_out = n;
    float f, incl;
    if (!wmm(&f, &incl, NULL) || (n < APP_CALIB_MAG_MIN_SAMPLES && s_p.ref.n == 0)) {
        return false;
    }
    mcal_ellipsoid(smp, n, f, incl, &s_mprior, ell);
    mcal_ref_solve(&s_p.ref, ell, APP_CALIB_MAG_REF_SD_M, APP_CALIB_MAG_REF_SD_B_UT, cal);
    if (cov_out) {
        *cov_out = n > 0 ? calib_dir_coverage(smp[0].m, (int)(sizeof(mcal_sample_t) / sizeof(float)), n,
                                              cal->b)
                         : 0.0f;
    }
    return true;
}

static void solve_mag_learned(void)
{
    int n = 0;
    float cov = 0.0f;
    s_st.mag_solved = solve_mag(&s_st.mag_ell, &s_st.mag_learned, &n, &cov);
    s_st.mag_bins = (uint32_t)n;
    s_st.mag_coverage = cov;
    s_st.mag_n_ref = s_p.ref.n;
    s_st.mag_learned_bad = s_st.mag_solved ? mcal_validate(&s_st.mag_learned, &s_st.mag_ell, &s_mlim) : NULL;
    s_st.mag_wait_fix = !s_p.lat_valid;

    /* What the fit is doing now (mag_boot_note only says what happened at boot). */
    static char state[80];
    if (!s_p.lat_valid) {
        snprintf(state, sizeof state, "waiting for a GNSS fix (no WMM field)");
    } else if (!s_st.mag_solved) {
        snprintf(state, sizeof state, "collecting: %d of %d sample directions needed", n,
                 APP_CALIB_MAG_MIN_SAMPLES);
    } else if (s_st.mag_learned_bad) {
        snprintf(state, sizeof state, "fit rejected: %s", s_st.mag_learned_bad);
    } else {
        snprintf(state, sizeof state, "fitted; %s",
                 s_p.hold[IMU_CALIB_MAG] ? "held: used only when accepted" : "applied at the next boot");
    }
    s_st.mag_state = state;
    s_mag_dirty = false;
}

static bool m_is_zero(const float m[9])
{
    for (int i = 0; i < 9; i++) {
        if (m[i] != 0.0f) {
            return false;
        }
    }
    return true;
}

static void solve_gyr_learned(void)
{
    rot_solve(&s_p.rot, APP_CALIB_GYR_SD_SCALE, (float)(APP_CALIB_GYR_SD_MIS_DEG * DEG2RAD),
              &s_st.gyr_learned);
    s_st.gyr_taken = rot_accept(&s_st.gyr_learned, s_gm_use, APP_CALIB_GYR_ACCEPT_SD_SCALE,
                                (float)(APP_CALIB_GYR_ACCEPT_SD_MIS_DEG * DEG2RAD), s_st.gyr_next);
    s_st.gyr_next_bad = rot_validate(s_st.gyr_next, APP_CALIB_GYR_MAX_SCALE_DEV,
                                     (float)(APP_CALIB_GYR_MAX_MIS_DEG * DEG2RAD));
    s_st.n_rot = s_p.rot.n_rot;
    s_st.n_turn = s_p.rot.n_turn;
    memcpy(s_st.gyr_m, s_gm_use, sizeof s_gm_use);
}

static void solve_learned(void)
{
    gcal_solve(&s_p.stats, &s_prior, &s_st.learned);
    s_st.learned_bad = gcal_validate(&s_st.learned, &s_lim);
    s_st.stats = s_p.stats;
    int n;
    solve_acc(&s_st.acc_learned, &n, &s_st.acc_coverage);
    s_st.acc_poses = (uint32_t)n;
    s_st.acc_learned_bad = n > 0 ? acal_validate(&s_st.acc_learned, &s_alim) : NULL;
}

static void drop_correction(void)
{
    s_st.applied = false;
    memset(&s_st.use, 0, sizeof s_st.use);
    memset(s_k_use, 0, sizeof s_k_use);
    memset(s_b0_use, 0, sizeof s_b0_use);
    s_st.acc_applied = false;
    memset(&s_st.acc_use, 0, sizeof s_st.acc_use);
    memset(s_am_use, 0, sizeof s_am_use); /* all-zero = identity in INSLIB */
    memset(s_ab_use, 0, sizeof s_ab_use);
    s_st.mag_applied = false;
    memset(&s_st.mag_use, 0, sizeof s_st.mag_use);
    memset(s_mm_use, 0, sizeof s_mm_use);
    memset(s_mb_use, 0, sizeof s_mb_use);
}

/* Make r the correction in use and the stored set of its group. */
static void use_gyr(const gcal_result_t *r)
{
    s_st.applied = true;
    s_st.use = *r;
    for (int i = 0; i < 3; i++) {
        s_b0_use[i] = r->b0[i];
        s_k_use[i] = r->k[i];
    }
    s_p.keep_gyr = *r;
    s_p.keep_valid[IMU_CALIB_GYR] = 1;
}

static void use_acc(const acal_result_t *r)
{
    s_st.acc_applied = true;
    s_st.acc_use = *r;
    acal_to_inslib(r->p, s_am_use, s_ab_use);
    s_p.keep_acc = *r;
    s_p.keep_valid[IMU_CALIB_ACC] = 1;
}

static void use_mag(const mcal_cal_t *c)
{
    s_st.mag_applied = true;
    s_st.mag_use = *c;
    mcal_to_inslib(c, s_mm_use, s_mb_use);
    s_p.keep_mag = *c;
    s_p.keep_valid[IMU_CALIB_MAG] = 1;
}

#define NOTE_HELD       "held set applied"
#define NOTE_HELD_NONE  "held: no set stored, no correction"
#define NOTE_KEPT       "no usable data: the stored set is applied"

void imu_calib_init(float gyr_psd, float acc_psd, unsigned platform)
{
    s_gyr_psd = gyr_psd;
    s_acc_psd = acc_psd;
    s_platform = platform;
    s_mprior.sd_diag_rel = APP_CALIB_MAG_SD_DIAG_REL;
    s_mprior.sd_offdiag = APP_CALIB_MAG_SD_OFFDIAG;
    s_mprior.sd_hard_ut = APP_CALIB_MAG_SD_HARD_UT;
    s_mprior.sample_sd_ut = APP_CALIB_MAG_SD_UT;
    s_mprior.dip_sd_ut = APP_CALIB_MAG_DIP_SD_UT;
    s_mlim.min_diag = APP_CALIB_MAG_MIN_DIAG;
    s_mlim.max_diag = APP_CALIB_MAG_MAX_DIAG;
    s_mlim.max_offdiag = APP_CALIB_MAG_MAX_OFFDIAG;
    s_mlim.max_hard_ut = APP_CALIB_MAG_MAX_HARD_UT;
    s_mlim.max_rms_rel = APP_CALIB_MAG_MAX_RMS_REL;
    s_prior.t_ref_c = APP_CALIB_T_REF_C;
    s_prior.sd_b0_rps = (float)(APP_CALIB_B0_PRIOR_DPS * DEG2RAD);
    s_prior.sd_k_rps_c = (float)(APP_CALIB_K_PRIOR_DPS_C * DEG2RAD);
    s_prior.k_min_span_c = APP_CALIB_K_MIN_SPAN_C;
    s_lim.max_b0_rps = (float)(APP_CALIB_MAX_B0_DPS * DEG2RAD);
    s_lim.max_k_rps_c = (float)(APP_CALIB_MAX_K_DPS_C * DEG2RAD);
    s_aprior.sd_scale = APP_CALIB_ACC_SD_SCALE;
    s_aprior.sd_mis = (float)(APP_CALIB_ACC_SD_MIS_DEG * DEG2RAD);
    s_aprior.sd_bias = APP_CALIB_ACC_SD_BIAS_MPS2;
    s_alim.max_scale_dev = APP_CALIB_ACC_MAX_SCALE_DEV;
    s_alim.max_mis_rad = (float)(APP_CALIB_ACC_MAX_MIS_DEG * DEG2RAD);
    s_alim.max_bias = APP_CALIB_ACC_MAX_BIAS_MPS2;

    memset(&s_st, 0, sizeof s_st);
    s_st.last_save_rc = 1;
    drop_correction();
    flash_rec_init(&s_rec, REC_SECTOR, REC_MAGIC, REC_VERSION, (uint16_t)sizeof(payload_t));

    memset(&s_p, 0, sizeof s_p);
    if (!flash_rec_load(&s_rec, &s_p)) {
        s_st.boot_note = "no record";
        s_st.acc_boot_note = "no record";
        s_st.mag_boot_note = "no record";
    } else {
        s_p.boots++;
        /* Per group: mode hold applies the stored set; mode auto the fit of
         * the data when it passes, else the stored set (no usable data),
         * else nothing. A stored set out of range is dropped. */
        for (int g = 0; g < IMU_CALIB_GROUPS; g++) {
            s_p.hold[g] = s_p.hold[g] ? 1 : 0;
        }
        if (s_p.keep_valid[IMU_CALIB_GYR] && gcal_validate(&s_p.keep_gyr, &s_lim) != NULL) {
            s_p.keep_valid[IMU_CALIB_GYR] = 0;
        }
        if (s_p.keep_valid[IMU_CALIB_ACC] && acal_validate(&s_p.keep_acc, &s_alim) != NULL) {
            s_p.keep_valid[IMU_CALIB_ACC] = 0;
        }
        if (s_p.keep_valid[IMU_CALIB_MAG] && mcal_validate(&s_p.keep_mag, NULL, &s_mlim) != NULL) {
            s_p.keep_valid[IMU_CALIB_MAG] = 0;
        }

        /* Gyro. */
        gcal_result_t r;
        const char *fit = NULL; /* why the fit is not used, NULL = it is */
        if (gcal_stats_validate(&s_p.stats) != NULL) {
            fit = "record unusable, starting anew";
            gcal_stats_clear(&s_p.stats);
        } else if (s_p.stats.n_chunks == 0) {
            fit = "no chunks"; /* the record holds other data only */
        } else {
            gcal_solve(&s_p.stats, &s_prior, &r);
            if (gcal_validate(&r, &s_lim) != NULL) {
                fit = "record out of range, starting anew";
                gcal_stats_clear(&s_p.stats);
            } else {
                gcal_stats_forget(&s_p.stats, APP_CALIB_FORGET_PER_BOOT);
            }
        }
        if (s_p.hold[IMU_CALIB_GYR]) {
            if (s_p.keep_valid[IMU_CALIB_GYR]) {
                use_gyr(&s_p.keep_gyr);
            }
            s_st.boot_note = s_p.keep_valid[IMU_CALIB_GYR] ? NOTE_HELD : NOTE_HELD_NONE;
        } else if (fit == NULL) {
            use_gyr(&r);
            s_st.boot_note = "applied";
        } else if (s_p.keep_valid[IMU_CALIB_GYR]) {
            use_gyr(&s_p.keep_gyr);
            s_st.boot_note = NOTE_KEPT;
        } else {
            s_st.boot_note = fit;
        }
        /* Accelerometer. */
        acal_result_t ar;
        int n;
        solve_acc(&ar, &n, NULL);
        fit = NULL;
        if (n == 0) {
            fit = "no poses";
        } else if (acal_validate(&ar, &s_alim) != NULL) {
            fit = "poses out of range, starting anew";
            memset(s_p.bins, 0, sizeof s_p.bins);
        }
        if (s_p.hold[IMU_CALIB_ACC]) {
            if (s_p.keep_valid[IMU_CALIB_ACC]) {
                use_acc(&s_p.keep_acc);
            }
            s_st.acc_boot_note = s_p.keep_valid[IMU_CALIB_ACC] ? NOTE_HELD : NOTE_HELD_NONE;
        } else if (fit == NULL) {
            use_acc(&ar);
            s_st.acc_boot_note = "applied";
        } else if (s_p.keep_valid[IMU_CALIB_ACC]) {
            use_acc(&s_p.keep_acc);
            s_st.acc_boot_note = NOTE_KEPT;
        } else {
            s_st.acc_boot_note = fit;
        }
        /* Magnetometer. */
        if (mcal_ref_validate(&s_p.ref) != NULL) {
            mcal_ref_clear(&s_p.ref);
        }
        mcal_ell_t me;
        mcal_cal_t mc;
        int mn = 0;
        fit = NULL;
        if (!solve_mag(&me, &mc, &mn, NULL)) {
            fit = s_p.lat_valid ? "too few samples" : "no GNSS fix yet (no WMM field)";
        } else if (mcal_validate(&mc, &me, &s_mlim) != NULL) {
            fit = "out of range, starting anew";
            memset(s_p.mag, 0, sizeof s_p.mag);
            mcal_ref_clear(&s_p.ref);
        } else {
            mcal_ref_forget(&s_p.ref, APP_CALIB_FORGET_PER_BOOT);
        }
        if (s_p.hold[IMU_CALIB_MAG]) {
            if (s_p.keep_valid[IMU_CALIB_MAG]) {
                use_mag(&s_p.keep_mag);
            }
            s_st.mag_boot_note = s_p.keep_valid[IMU_CALIB_MAG] ? NOTE_HELD : NOTE_HELD_NONE;
        } else if (fit == NULL) {
            use_mag(&mc);
            s_st.mag_boot_note = "applied";
        } else if (s_p.keep_valid[IMU_CALIB_MAG]) {
            use_mag(&s_p.keep_mag);
            s_st.mag_boot_note = NOTE_KEPT;
        } else {
            s_st.mag_boot_note = fit;
        }
        /* Gyro matrix: the stored one, plus every element the statistics now
         * pin down. A change re-linearises, so the statistics start anew. */
        float m0[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
        if (!m_is_zero(s_p.gyr_m0)) {
            memcpy(m0, s_p.gyr_m0, sizeof m0);
        }
        const float mis_lim = (float)(APP_CALIB_GYR_MAX_MIS_DEG * DEG2RAD);
        if (rot_validate(m0, APP_CALIB_GYR_MAX_SCALE_DEV, mis_lim) != NULL ||
            rot_stats_validate(&s_p.rot) != NULL) {
            s_st.gyr_m_boot_note = "record out of range, starting anew";
            const float id[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
            memcpy(m0, id, sizeof m0);
            rot_stats_clear(&s_p.rot);
        } else if (s_p.hold[IMU_CALIB_GYR_M]) {
            s_st.gyr_m_boot_note = "held";
        } else {
            rot_result_t rr;
            float mn[9];
            rot_solve(&s_p.rot, APP_CALIB_GYR_SD_SCALE, (float)(APP_CALIB_GYR_SD_MIS_DEG * DEG2RAD), &rr);
            int taken = rot_accept(&rr, m0, APP_CALIB_GYR_ACCEPT_SD_SCALE,
                                   (float)(APP_CALIB_GYR_ACCEPT_SD_MIS_DEG * DEG2RAD), mn);
            if (taken > 0 && rot_validate(mn, APP_CALIB_GYR_MAX_SCALE_DEV, mis_lim) == NULL) {
                memcpy(m0, mn, sizeof m0);
                rot_stats_clear(&s_p.rot);
                s_st.unsaved++; /* the new matrix must reach flash */
                s_st.gyr_m_boot_note = "updated from the statistics";
            } else {
                s_st.gyr_m_boot_note = taken > 0 ? "update out of range, kept the matrix" : "kept";
            }
        }
        memcpy(s_gm_use, m0, sizeof m0);
        memcpy(s_p.gyr_m0, m0, sizeof m0);
        const float id[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
        s_st.gyr_m_applied = memcmp(m0, id, sizeof id) != 0;
    }
    if (!s_st.gyr_m_boot_note) {
        s_st.gyr_m_boot_note = "no record";
    }
    s_st.rec_seq = flash_rec_seq(&s_rec);
    solve_learned();
    solve_mag_learned();
    solve_gyr_learned();
    rot_preint_reset(&s_rot);
}

void imu_calib_gyr_misalignment(float m[9])
{
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            m[c * 3 + r] = s_gm_use[r * 3 + c]; /* column-major */
        }
    }
}

void imu_calib_set_position(double lat_rad, double lon_rad, float h_m, float year)
{
    if (!isfinite(lat_rad) || !isfinite(lon_rad) || !isfinite(h_m) || !isfinite(year)) {
        return;
    }
    const bool first = !s_p.lat_valid;
    s_p.lat_rad = lat_rad;
    s_p.lon_rad = lon_rad;
    s_p.h_m = h_m;
    s_p.year = year;
    s_p.lat_valid = 1;
    if (!s_fix_this_boot) {
        s_fix_this_boot = true;
        s_mag_dirty = true; /* the WMM field is known now, or newer */
        if (first) {
            printf("[calib] first GNSS fix: WMM field available for the magnetometer fit\n");
        }
    }
}

void imu_calib_mag_calibration(float m_colmajor[9], float bias[3])
{
    memcpy(m_colmajor, s_mm_use, sizeof s_mm_use);
    memcpy(bias, s_mb_use, sizeof s_mb_use);
}

void imu_calib_set_psd(float gyr_psd, float acc_psd)
{
    s_gyr_psd = gyr_psd;
    s_acc_psd = acc_psd;
}

void imu_calib_gyr_fixed_bias(float out[3])
{
    for (int i = 0; i < 3; i++) {
        out[i] = s_b0_use[i];
    }
}

void imu_calib_acc_calibration(float m_colmajor[9], float bias[3])
{
    memcpy(m_colmajor, s_am_use, sizeof s_am_use);
    memcpy(bias, s_ab_use, sizeof s_ab_use);
}

void imu_calib_correct(float gyr[3], const tm210_stats_t *st)
{
    if (!s_st.applied || !st->status_valid) {
        return;
    }
    const float d = st->temperature_c - APP_CALIB_T_REF_C;
    for (int i = 0; i < 3; i++) {
        gyr[i] -= s_k_use[i] * d;
    }
}

/* ---- Chunks (gyro) --------------------------------------------------------- */

static void chunk_clear(void)
{
    memset(&s_chunk, 0, sizeof s_chunk);
}

/* Keeps the last GNSS position for the Earth rate and the local g. */
static void take_position(const nav_output_t *o)
{
    /* The full position (with longitude and epoch for the WMM) comes from
     * imu_calib_set_position(); this only covers a fix the main loop has
     * not handed over yet. */
    if (o->pos_valid && !s_p.lat_valid) {
        s_p.lat_rad = o->llh[0];
        s_p.lon_rad = o->llh[1];
        s_p.h_m = (float)o->llh[2];
    }
}

/* The chunk is complete: one bias point at one temperature. */
static void chunk_close(void)
{
    if (s_chunk.n == 0 || s_chunk.dt_s < APP_CALIB_CHUNK_MIN_S) {
        chunk_clear();
        return;
    }
    if (!s_learn) {
        chunk_clear();
        return;
    }
    if (s_chunk.t_max - s_chunk.t_min > APP_CALIB_CHUNK_MAX_DT_C) {
        s_st.skip_temp++;
        chunk_clear();
        return;
    }

    /* Earth rate in the body frame needs the yaw, so INS or AHRS only. */
    nav_output_t o;
    nav_get_output(&o);
    if (o.t_us == 0 || !o.rpy_valid || (o.att_source != NAV_ATT_INS && o.att_source != NAV_ATT_AHRS)) {
        s_st.skip_att++;
        chunk_clear();
        return;
    }
    take_position(&o);
    const double lat = s_p.lat_valid ? s_p.lat_rad : APP_CALIB_DEFAULT_LAT_DEG * DEG2RAD;
    float we[3];
    gcal_earth_rate_body(o.rpy_rad, lat, we);

    float bias[3];
    for (int i = 0; i < 3; i++) {
        bias[i] = (float)(s_chunk.sum[i] / (double)s_chunk.n) - we[i];
    }
    const float temp = (float)(s_chunk.t_sum / (double)s_chunk.n);
    const float floor_rps = (float)(APP_CALIB_FLOOR_DPS * DEG2RAD);
    const float var = s_gyr_psd / s_chunk.dt_s + floor_rps * floor_rps;
    if (gcal_stats_add(&s_p.stats, &s_prior, bias, temp, var)) {
        s_p.stats.still_s += s_chunk.dt_s;
        s_st.chunks_boot++;
        s_st.unsaved++;
        solve_learned();
        printf("[calib] chunk %.0f s at %.2f C: bias %.4f %.4f %.4f deg/s (earth rate removed)\n",
               (double)s_chunk.dt_s, (double)temp, (double)(bias[0] * RAD2DEG_F),
               (double)(bias[1] * RAD2DEG_F), (double)(bias[2] * RAD2DEG_F));
    }
    chunk_clear();
}

/* ---- Poses (accelerometer) ------------------------------------------------- */

static int pose_bin(const float a[3])
{
    const float len = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    const float u[3] = { a[0] / len, a[1] / len, a[2] / len };
    int ax = 0;
    for (int i = 1; i < 3; i++) {
        if (fabsf(u[i]) > fabsf(u[ax])) {
            ax = i;
        }
    }
    const int face = 2 * ax + (u[ax] < 0.0f ? 1 : 0);
    const float o1 = u[(ax + 1) % 3], o2 = u[(ax + 2) % 3];
    int sub = 0;
    if (fabsf(o1) >= POSE_CENTRE_SIN || fabsf(o2) >= POSE_CENTRE_SIN) {
        float ang = atan2f(o2, o1) + (float)M_PI; /* 0 .. 2 pi */
        int q = (int)(ang / (float)(M_PI / 2.0));
        sub = 1 + (q < 0 ? 0 : q > 3 ? 3 : q);
    }
    return face * 5 + sub;
}

static void pose_clear(void)
{
    memset(&s_pose, 0, sizeof s_pose);
}

/* The pose is complete: keep it if its bin has nothing better. */
static void pose_commit(void)
{
    if (s_pose.done || !s_learn || s_pose.n == 0 || s_pose.dt_s < APP_CALIB_POSE_MIN_S) {
        return;
    }
    s_pose.done = true;
    float a[3];
    for (int i = 0; i < 3; i++) {
        a[i] = (float)(s_pose.sum[i] / (double)s_pose.n);
    }
    const int b = pose_bin(a);
    pose_bin_t *pb = &s_p.bins[b];
    /* Longer is better up to the cap; at the cap the newest wins, so old
     * poses are replaced as new data comes in. */
    const float score = fminf(s_pose.dt_s, APP_CALIB_POSE_MAX_S);
    if (pb->valid && score < fminf(pb->dur_s, APP_CALIB_POSE_MAX_S)) {
        s_st.poses_kept_old++;
        return;
    }
    nav_output_t o;
    nav_get_output(&o);
    take_position(&o);
    for (int i = 0; i < 3; i++) {
        pb->acc[i] = a[i];
    }
    pb->dur_s = s_pose.dt_s;
    pb->temp_c = (float)(s_pose.t_sum / (double)s_pose.n);
    pb->boot = (uint16_t)s_p.boots;
    pb->valid = 1;
    s_st.poses_boot++;
    s_st.unsaved++;
    solve_learned();
    printf("[calib] pose %.0f s in bin %d: acc %.4f %.4f %.4f m/s2, |a| %.4f (g %.4f), "
           "%lu poses, coverage %.3f\n",
           (double)s_pose.dt_s, b, (double)a[0], (double)a[1], (double)a[2],
           (double)sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]), (double)local_g(),
           (unsigned long)s_st.acc_poses, (double)s_st.acc_coverage);
}

/* ---- Still runs ------------------------------------------------------------ */

/* ---- Rotations between still runs (gyro matrix) --------------------------- */

static bool runacc_mean(float a[3])
{
    if (s_runacc.n == 0 || s_runacc.dt_s < APP_CALIB_POSE_MIN_S) {
        return false;
    }
    for (int i = 0; i < 3; i++) {
        a[i] = (float)(s_runacc.sum[i] / (double)s_runacc.n);
    }
    return true;
}

/* The rotation since the last still run is complete: g2 is this run's
 * gravity direction. */
static void rot_finish(const float g2[3])
{
    s_rot_state = ROT_IDLE;
    if (!s_learn) {
        return;
    }
    const float ang = rot_preint_angle(&s_rot);
    if (ang < (float)(APP_CALIB_ROT_MIN_ANGLE_DEG * DEG2RAD)) {
        s_st.rot_small++;
        return;
    }
    if (rot_add_rotation(&s_p.rot, &s_rot, s_rot_g1, g2, APP_CALIB_ROT_SD_RAD)) {
        s_st.rot_boot++;
        s_st.unsaved++;
        solve_gyr_learned();
        printf("[calib] rotation %.0f deg in %.1f s: %lu rotations, %lu turns\n", (double)(ang * RAD2DEG_F),
               (double)s_rot.t_s, (unsigned long)s_p.rot.n_rot, (unsigned long)s_p.rot.n_turn);
    }
}

static void run_end(void)
{
    chunk_close();     /* the pending window (the last one) is not in them */
    pose_commit();
    pose_clear();
    if (s_run) {
        float a[3];
        const bool have = runacc_mean(a);
        if (s_rot_state == ROT_FROZEN) {
            if (have) {
                rot_finish(a);
            } else {
                s_rot_state = ROT_IDLE;
            }
        }
        /* Arm the next rotation from this run. The preintegration already
         * holds the samples since the last still window. */
        if (have) {
            for (int i = 0; i < 3; i++) {
                s_rot_g1[i] = a[i];
            }
            s_rot_state = ROT_ARMED;
        } else {
            s_rot_state = ROT_IDLE;
        }
    }
    s_run = false;
    s_pend.valid = false;
}

static void window_done(bool pass, const float mean[6], uint32_t n, float dt_s, float temp_c)
{
    if (!pass || !isfinite(temp_c)) {
        if (!isfinite(temp_c) && pass && s_chunk.n > 0) {
            s_st.skip_temp++;
            chunk_clear();
        }
        run_end();
        return;
    }
    bool first = false;
    if (!s_run) {
        first = true;
        s_run = true;  /* the first window of a run is dropped */
        s_pend.valid = false;
        chunk_clear();
        pose_clear();
        memset(&s_runacc, 0, sizeof s_runacc);
        if (s_rot_state == ROT_ARMED) {
            s_rot_state = ROT_FROZEN; /* the rotation is over; wait for this pose */
        } else {
            rot_preint_reset(&s_rot);
        }
    } else if (s_rot_state != ROT_FROZEN) {
        rot_preint_reset(&s_rot); /* still: the next rotation starts here */
    }
    for (int i = 0; i < 3; i++) {
        s_runacc.sum[i] += (double)mean[3 + i] * (double)n;
    }
    s_runacc.n += n;
    s_runacc.dt_s += dt_s;
    if (s_rot_state == ROT_FROZEN && s_runacc.dt_s >= APP_CALIB_POSE_MAX_S) {
        float a[3];
        if (runacc_mean(a)) {
            rot_finish(a); /* a long stop: no need to wait for its end */
        }
    }
    if (first) {
        return; /* the first window of a run goes into no chunk or pose */
    }
    if (s_pend.valid) {
        for (int i = 0; i < 3; i++) {
            s_chunk.sum[i] += (double)s_pend.mean[i] * (double)s_pend.n;
        }
        if (s_chunk.n == 0 || s_pend.temp_c < s_chunk.t_min) s_chunk.t_min = s_pend.temp_c;
        if (s_chunk.n == 0 || s_pend.temp_c > s_chunk.t_max) s_chunk.t_max = s_pend.temp_c;
        s_chunk.n += s_pend.n;
        s_chunk.dt_s += s_pend.dt_s;
        s_chunk.t_sum += (double)s_pend.temp_c * (double)s_pend.n;
        if (s_chunk.dt_s >= APP_CALIB_CHUNK_MAX_S) {
            chunk_close();
        }
        if (!s_pose.done) {
            for (int i = 0; i < 3; i++) {
                s_pose.sum[i] += (double)s_pend.mean[3 + i] * (double)s_pend.n;
            }
            s_pose.n += s_pend.n;
            s_pose.dt_s += s_pend.dt_s;
            s_pose.t_sum += (double)s_pend.temp_c * (double)s_pend.n;
            if (s_pose.dt_s >= APP_CALIB_POSE_MAX_S) {
                pose_commit();
            }
        }
    }
    s_pend.valid = true;
    for (int i = 0; i < 6; i++) {
        s_pend.mean[i] = mean[i];
    }
    s_pend.n = n;
    s_pend.dt_s = dt_s;
    s_pend.temp_c = temp_c;
}

static int mag_bin(const float m[3])
{
    int ax = 0;
    for (int i = 1; i < 3; i++) {
        if (fabsf(m[i]) > fabsf(m[ax])) {
            ax = i;
        }
    }
    const float big = fabsf(m[ax]);
    if (big < 1e-3f) {
        return -1;
    }
    const int face = 2 * ax + (m[ax] < 0.0f ? 1 : 0);
    int cell = 0;
    for (int k = 0; k < 2; k++) {
        float a = m[(ax + 1 + k) % 3] / big; /* -1 .. 1 */
        int q = (int)((a + 1.0f) * 1.5f);
        cell = cell * 3 + (q < 0 ? 0 : q > 2 ? 2 : q);
    }
    return face * 9 + cell;
}

/* A group of APP_CALIB_MAG_GROUP magnetometer samples, taken while turning
 * slowly, is one binned sample. Newest wins in its bin. */
static void mag_group_done(void)
{
    float m[3], down[3];
    for (int i = 0; i < 3; i++) {
        m[i] = (float)(s_mgrp.m[i] / s_mgrp.n);
    }
    bool has_down = s_mgrp.down_ok;
    if (has_down) {
        double a[3] = { s_mgrp.a[0] / s_mgrp.n, s_mgrp.a[1] / s_mgrp.n, s_mgrp.a[2] / s_mgrp.n };
        double len = sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
        has_down = len > 1.0;
        for (int i = 0; i < 3; i++) {
            down[i] = has_down ? (float)(-a[i] / len) : 0.0f; /* specific force points up */
        }
    } else {
        down[0] = down[1] = down[2] = 0.0f;
    }
    /* A sample far off the current fit (steel nearby, a running motor) is
     * not stored. */
    if (s_st.mag_solved && s_st.mag_bins >= APP_CALIB_MAG_MIN_SAMPLES && s_st.mag_learned_bad == NULL) {
        const mcal_cal_t *cl = &s_st.mag_learned;
        float v[3], n2 = 0.0f;
        for (int i = 0; i < 3; i++) {
            v[i] = cl->m[i * 3 + 0] * (m[0] - cl->b[0]) + cl->m[i * 3 + 1] * (m[1] - cl->b[1]) +
                   cl->m[i * 3 + 2] * (m[2] - cl->b[2]);
            n2 += v[i] * v[i];
        }
        if (fabsf(sqrtf(n2) - s_st.mag_ell.field_ut) > APP_CALIB_MAG_OUTLIER_REL * s_st.mag_ell.field_ut) {
            s_st.mag_outliers++;
            return;
        }
    }
    int b = mag_bin(m);
    if (b < 0) {
        return;
    }
    mag_bin_t *mb = &s_p.mag[b];
    for (int i = 0; i < 3; i++) {
        mb->m[i] = m[i];
        mb->down[i] = down[i];
    }
    mb->has_down = has_down;
    mb->boot = (uint16_t)s_p.boots;
    mb->valid = 1;
    s_st.mag_groups_boot++;
    if (s_st.mag_groups_boot % APP_CALIB_MAG_GROUPS_PER_UNSAVED == 0) {
        s_st.unsaved++;
    }
    s_mag_dirty = true;
}

static void mag_push(const tm210_sample_t *s)
{
    const float rate = sqrtf(s->gyr[0] * s->gyr[0] + s->gyr[1] * s->gyr[1] + s->gyr[2] * s->gyr[2]) *
                       RAD2DEG_F;
    s_last_rate_dps = rate;
    s_last_gz_dps = s->gyr[2] * RAD2DEG_F;
    if (!s->mag_new) {
        return;
    }
    for (int i = 0; i < 3; i++) {
        s_last_mag[i] = s->mag[i];
    }
    s_last_mag_us = s->t_us;
    if (!s_learn || s->gap || rate > APP_CALIB_MAG_MAX_RATE_DPS) {
        s_mgrp.n = 0;
        return;
    }
    const float amag = sqrtf(s->acc[0] * s->acc[0] + s->acc[1] * s->acc[1] + s->acc[2] * s->acc[2]);
    const bool quiet = fabsf(amag - G_NOMINAL) <= APP_CALIB_MAG_DOWN_MAX_DG_MPS2;
    if (s_mgrp.n == 0) {
        memset(&s_mgrp, 0, sizeof s_mgrp);
        s_mgrp.down_ok = true;
    }
    for (int i = 0; i < 3; i++) {
        s_mgrp.m[i] += s->mag[i];
        s_mgrp.a[i] += s->acc[i];
    }
    s_mgrp.down_ok = s_mgrp.down_ok && quiet;
    if (++s_mgrp.n >= APP_CALIB_MAG_GROUP) {
        mag_group_done();
        s_mgrp.n = 0;
    }
}

/* Gyro sample for the rotation and turn integration: raw minus the bias in
 * use, through the matrix in use. */
static void gyr_push(const tm210_sample_t *s, const tm210_stats_t *st)
{
    if (s->gap) {
        rot_preint_reset(&s_rot);
        s_rot_state = ROT_IDLE;
        s_turn.active = false;
        return;
    }
    float u[3];
    const float d = (s_st.applied && st->status_valid) ? st->temperature_c - APP_CALIB_T_REF_C : 0.0f;
    for (int i = 0; i < 3; i++) {
        u[i] = s->gyr[i] - s_b0_use[i] - s_k_use[i] * d;
    }
    if (s_rot_state != ROT_FROZEN) {
        rot_preint_step(&s_rot, u, s_gm_use, s->dt_s);
        if (s_rot_state == ROT_ARMED && s_rot.t_s > APP_CALIB_ROT_MAX_S) {
            s_rot_state = ROT_IDLE; /* too long between two still runs */
        }
    }
    if (s_turn.active && s_nav_rpy_ok) {
        /* Yaw rate from the body rates: (sin(roll) w_y + cos(roll) w_z) / cos(pitch),
         * plus the Earth's vertical rate the gyro also sees. */
        const float cr = cosf(s_nav_rpy[0]), sr = sinf(s_nav_rpy[0]), cp = cosf(s_nav_rpy[1]);
        const float c[3] = { 0.0f, sr / cp, cr / cp };
        float w[3];
        for (int i = 0; i < 3; i++) {
            w[i] = s_gm_use[i * 3 + 0] * u[0] + s_gm_use[i * 3 + 1] * u[1] + s_gm_use[i * 3 + 2] * u[2];
        }
        const double lat = s_p.lat_valid ? s_p.lat_rad : APP_CALIB_DEFAULT_LAT_DEG * DEG2RAD;
        const float earth = (float)(7.2921151467e-5 * sin(lat));
        s_turn.pred += (c[1] * w[1] + c[2] * w[2] + earth) * s->dt_s;
        for (int i = 1; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                s_turn.h[3 * i + j] += c[i] * u[j] * s->dt_s;
            }
        }
    }
}

void imu_calib_gnss_heading(uint64_t t_us, float heading, float sd, bool is_course)
{
#if APP_CALIB_ENABLE
    if (!s_learn || !isfinite(heading) || !(sd > 0.0f)) {
        return;
    }
    if (is_course) {
        if (s_platform != APP_PLATFORM_TRACTOR && s_platform != APP_PLATFORM_FIXED_WING) {
            return;
        }
        /* Forward only: reversing turns the course by 180 deg. */
        if (!s_nav_rpy_ok || fabsf(remainderf(heading - s_nav_rpy[2], 2.0f * (float)M_PI)) > 0.5f) {
            s_turn.active = false;
            return;
        }
    }
    const bool straight = fabsf(s_last_gz_dps) < APP_CALIB_TURN_STRAIGHT_DPS;
    if (s_turn.active && (t_us - s_turn.t_last > 2000000u ||
                          t_us - s_turn.t0 > (uint64_t)(APP_CALIB_TURN_MAX_S * 1e6f))) {
        s_turn.active = false; /* heading lost, or too long */
    }
    if (s_turn.active) {
        s_turn.t_last = t_us;
        if (!straight) {
            return;
        }
        const float meas = s_turn.pred + remainderf(heading - s_turn.psi0 - s_turn.pred, 2.0f * (float)M_PI);
        if (fabsf(s_turn.pred) >= (float)(APP_CALIB_TURN_MIN_ANGLE_DEG * DEG2RAD)) {
            const float f = APP_CALIB_TURN_SD_FLOOR_RAD;
            const float tsd = sqrtf(s_turn.sd0 * s_turn.sd0 + sd * sd + f * f);
            if (rot_add_turn(&s_p.rot, s_turn.pred, meas, s_turn.h, tsd)) {
                s_st.turn_boot++;
                s_st.unsaved++;
                solve_gyr_learned();
                printf("[calib] turn %.0f deg (GNSS %.1f deg off): %lu rotations, %lu turns\n",
                       (double)(s_turn.pred * RAD2DEG_F), (double)((s_turn.pred - meas) * RAD2DEG_F),
                       (unsigned long)s_p.rot.n_rot, (unsigned long)s_p.rot.n_turn);
            }
        } else if (fabsf(s_turn.pred) >= (float)(5.0 * DEG2RAD)) {
            return; /* a small heading change on the straight: keep going */
        }
        s_turn.active = false; /* start the next segment here */
    }
    if (straight) {
        memset(&s_turn, 0, sizeof s_turn);
        s_turn.active = true;
        s_turn.psi0 = heading;
        s_turn.sd0 = sd;
        s_turn.t0 = s_turn.t_last = t_us;
    }
#else
    (void)t_us;
    (void)heading;
    (void)sd;
    (void)is_course;
#endif
}

void imu_calib_push(const tm210_sample_t *s, const tm210_stats_t *st)
{
#if APP_CALIB_ENABLE
    gyr_push(s, st);
    mag_push(s);
    const float x[6] = { s->gyr[0], s->gyr[1], s->gyr[2], s->acc[0], s->acc[1], s->acc[2] };
    if (s->gap) {
        s_win.n = 0;
        run_end();
    }
    if (s_win.n == 0) {
        for (int i = 0; i < 6; i++) {
            s_win.mean[i] = x[i];
            s_win.m2[i] = 0.0f;
        }
        s_win.t0 = s->t_us;
        s_win.dt_s = s->dt_s;
        s_win.n = 1;
        return;
    }
    s_win.n++;
    s_win.dt_s += s->dt_s;
    for (int i = 0; i < 6; i++) {
        float d = x[i] - s_win.mean[i];
        s_win.mean[i] += d / (float)s_win.n;
        s_win.m2[i] += d * (x[i] - s_win.mean[i]);
    }
    if (s_win.n < WINDOW_MIN_N || s->t_us - s_win.t0 < WINDOW_US) {
        return;
    }
    const float *m = s_win.mean;
    const float inv = 1.0f / (3.0f * (float)(s_win.n - 1u));
    const float g_dps = sqrtf((s_win.m2[0] + s_win.m2[1] + s_win.m2[2]) * inv) * RAD2DEG_F;
    const float a = sqrtf((s_win.m2[3] + s_win.m2[4] + s_win.m2[5]) * inv);
    /* The variance test alone passes a steady turn (constant rate, constant
     * centripetal acceleration), so the mean rate and the magnitude must
     * look like rest too. */
    const float rate_dps = sqrtf(m[0] * m[0] + m[1] * m[1] + m[2] * m[2]) * RAD2DEG_F;
    const float amag = sqrtf(m[3] * m[3] + m[4] * m[4] + m[5] * m[5]);
    bool pass = g_dps <= APP_STILL_GYR_STDDEV_DPS && a <= APP_STILL_ACC_STDDEV_MPS2;
    if (pass && (rate_dps > APP_CALIB_STILL_MAX_RATE_DPS ||
                 fabsf(amag - G_NOMINAL) > APP_CALIB_STILL_MAX_DG_MPS2)) {
        pass = false;
        s_st.windows_not_rest++;
    }
    window_done(pass, s_win.mean, s_win.n, s_win.dt_s,
                st->status_valid ? st->temperature_c : NAN);
    s_win.n = 0;
#else
    (void)s;
    (void)st;
#endif
}

/* ---- Identity and saving --------------------------------------------------- */

static bool check_identity(const tm210_t *imu)
{
    if (s_st.id_state != IMU_CALIB_ID_WAIT || tm210_state(imu) != TM210_ST_RUN) {
        return false;
    }
    const bool was = s_st.applied || s_st.acc_applied || s_st.mag_applied || s_st.gyr_m_applied;
    if (!imu->got_setting) {
        /* Running on the raw stream without a Setting reply (TM210_E_DEAF):
         * the IMU can't be told apart, so nothing is learned or applied. */
        s_st.id_state = IMU_CALIB_ID_UNKNOWN;
        printf("[calib] TM210 node ID unknown: calibration %s, not learning\n",
               was ? "dropped" : "off");
        drop_correction();
        return was;
    }
    s_st.node_id = imu->node_id;
    s_st.id_state = IMU_CALIB_ID_OK;
    s_learn = true;
    if (!s_p.id_valid) {
        s_p.node_id = imu->node_id;
        s_p.id_valid = 1;
        return false;
    }
    if (s_p.node_id == imu->node_id) {
        return false;
    }
    printf("[calib] record is for TM210 node %u, this is %u: starting anew\n", s_p.node_id,
           imu->node_id);
    drop_correction();
    memset(&s_p, 0, sizeof s_p);
    s_p.node_id = imu->node_id;
    s_p.id_valid = 1;
    const float id[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
    memcpy(s_gm_use, id, sizeof id);
    memcpy(s_p.gyr_m0, id, sizeof id);
    s_st.gyr_m_applied = false;
    solve_learned();
    solve_mag_learned();
    solve_gyr_learned();
    return was;
}

/* Is the INS yaw from a GNSS heading (not the magnetometer) right now? */
static bool heading_from_gnss(const nav_output_t *o, uint64_t now_us, const char **why)
{
    if (o->n_yaw != s_n_yaw_prev) {
        s_n_yaw_prev = o->n_yaw;
        s_yaw_seen_us = now_us;
    }
    if (o->dual && s_yaw_seen_us != 0 && now_us - s_yaw_seen_us < 2000000u) {
        *why = "dual-antenna heading";
        return true;
    }
    const bool course_ok = s_platform == APP_PLATFORM_TRACTOR || s_platform == APP_PLATFORM_FIXED_WING;
    if (!course_ok || !o->vel_valid) {
        return false;
    }
    const float vn = o->vel_ned[0], ve = o->vel_ned[1];
    const float speed = sqrtf(vn * vn + ve * ve);
    if (speed < APP_CALIB_MAG_REF_MIN_SPEED_MPS) {
        return false;
    }
    if (s_platform == APP_PLATFORM_TRACTOR) {
        /* Driving forward: the course is the heading only then. */
        const float fwd = vn * cosf(o->rpy_rad[2]) + ve * sinf(o->rpy_rad[2]);
        if (fwd < 0.9f * speed) {
            return false;
        }
    } else if (fabsf(s_last_gz_dps) > APP_CALIB_MAG_REF_FW_MAX_YAW_DPS) {
        return false; /* fixed wing: straight flight only */
    }
    *why = "GNSS course";
    return true;
}

static void ref_sample(uint64_t now_us)
{
    /* Runs on every main-loop pass, but core 1 publishes only 10 times a
     * second: skip the locked copy until it has. */
    static uint32_t s_seq_seen;
    const uint32_t seq = nav_output_seq();
    if (seq == s_seq_seen) {
        return;
    }
    s_seq_seen = seq;

    nav_output_t o;
    nav_get_output(&o);
    if (o.t_us == 0 || o.t_us == s_nav_t_us) {
        return;
    }
    s_nav_t_us = o.t_us;
    s_nav_rpy_ok = o.rpy_valid && o.att_source == NAV_ATT_INS;
    if (s_nav_rpy_ok) {
        for (int i = 0; i < 3; i++) {
            s_nav_rpy[i] = o.rpy_rad[i];
        }
    }
    const bool full = o.mode == NAV_SUITE_MODE_FULL && o.att_source == NAV_ATT_INS && o.rpy_valid;
    if (!full) {
        s_full_since_us = 0;
        return;
    }
    if (s_full_since_us == 0) {
        s_full_since_us = now_us;
    }
    const char *why = NULL;
    const bool gnss_yaw = heading_from_gnss(&o, now_us, &why);
    s_st.mag_ref_source = gnss_yaw ? why : NULL;
    if (!s_learn || !gnss_yaw || now_us - s_full_since_us < (uint64_t)APP_CALIB_MAG_REF_FULL_S * 1000000u ||
        now_us - s_ref_last_us < APP_CALIB_MAG_REF_PERIOD_US || s_last_rate_dps > APP_CALIB_MAG_REF_MAX_RATE_DPS ||
        s_last_mag_us == 0 || now_us - s_last_mag_us > 50000u) {
        return;
    }
    float f, incl, b_ned[3];
    if (!wmm(&f, &incl, b_ned)) {
        return;
    }
    /* True field in the body frame: C_nb^T * b_ned (ZYX Euler). */
    const float cr = cosf(o.rpy_rad[0]), sr = sinf(o.rpy_rad[0]);
    const float cp = cosf(o.rpy_rad[1]), sp = sinf(o.rpy_rad[1]);
    const float cy = cosf(o.rpy_rad[2]), sy = sinf(o.rpy_rad[2]);
    const float cnb[3][3] = {
        { cp * cy, sr * sp * cy - cr * sy, cr * sp * cy + sr * sy },
        { cp * sy, sr * sp * sy + cr * cy, cr * sp * sy - sr * cy },
        { -sp, sr * cp, cr * cp },
    };
    float ref[3];
    for (int j = 0; j < 3; j++) {
        ref[j] = cnb[0][j] * b_ned[0] + cnb[1][j] * b_ned[1] + cnb[2][j] * b_ned[2];
    }
    if (mcal_ref_add(&s_p.ref, s_last_mag, ref, APP_CALIB_MAG_REF_SD_UT)) {
        s_ref_last_us = now_us;
        s_st.mag_ref_boot++;
        if (s_st.mag_ref_boot % APP_CALIB_MAG_REFS_PER_UNSAVED == 0) {
            s_st.unsaved++;
        }
        s_mag_dirty = true;
    }
}

bool imu_calib_poll(uint64_t now_us, const tm210_t *imu)
{
#if APP_CALIB_ENABLE
    bool restart = check_identity(imu);
    ref_sample(now_us);
    if (s_mag_dirty && now_us - s_mag_solved_us >= APP_CALIB_MAG_SOLVE_PERIOD_US) {
        s_mag_solved_us = now_us;
        solve_mag_learned();
    }
    if (tm210_state(imu) != TM210_ST_RUN && s_run) {
        s_win.n = 0;
        run_end();
    }
    s_st.still = s_run;
    s_st.chunk_s = s_chunk.dt_s;
    s_st.rot_state = s_rot_state == ROT_ARMED ? "armed (turning)" : s_rot_state == ROT_FROZEN ? "waiting for the pose" : "idle";
    s_st.turn_active = s_turn.active;

    /* Save at standstill only (the flash write pauses core 1 briefly). */
    if (s_learn && s_run && s_st.unsaved >= APP_CALIB_SAVE_MIN_CHUNKS &&
        (s_last_save_us == 0 || now_us - s_last_save_us >= (uint64_t)APP_CALIB_SAVE_MIN_S * 1000000u)) {
        uint64_t t0 = time_us_64();
        int rc = flash_rec_save(&s_rec, &s_p);
        s_last_save_us = now_us ? now_us : 1;
        s_st.last_save_rc = rc;
        if (rc == 0) {
            s_st.unsaved = 0;
            s_st.rec_seq = flash_rec_seq(&s_rec);
        }
        printf("[calib] save %s (record %lu, %lu ms)\n", rc == 0 ? "done" : "failed",
               (unsigned long)s_st.rec_seq, (unsigned long)((time_us_64() - t0) / 1000u));
    }
    return restart;
#else
    (void)now_us;
    (void)imu;
    return false;
#endif
}

/* ---- Web page actions (Phase 5) ------------------------------------------ */

static const char *const k_gkey[IMU_CALIB_GROUPS + 1] = { "gyr", "acc", "mag", "gyr_m", "all" };
static const char *const k_gname[IMU_CALIB_GROUPS + 1] = { "gyro bias", "accelerometer", "magnetometer",
                                                           "gyro matrix", "IMU calibration" };
static const char *const k_okey[IMU_CALIB_OPS] = { "accept", "hold", "auto", "clear", "factory" };
static const float k_ident[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

int imu_calib_group_from_name(const char *s)
{
    for (int g = 0; g <= IMU_CALIB_GROUPS; g++) {
        if (strcmp(s, k_gkey[g]) == 0) {
            return g;
        }
    }
    return -1;
}

int imu_calib_op_from_name(const char *s)
{
    for (int op = 0; op < IMU_CALIB_OPS; op++) {
        if (strcmp(s, k_okey[op]) == 0) {
            return op;
        }
    }
    return -1;
}

const char *imu_calib_group_key(int g)
{
    return g >= 0 && g <= IMU_CALIB_GROUPS ? k_gkey[g] : "?";
}

/* NULL if the learned set of g can be accepted now (stillness aside). */
static const char *accept_check(int g)
{
    switch (g) {
    case IMU_CALIB_GYR:
        return s_p.stats.n_chunks == 0 ? "no chunks yet" : s_st.learned_bad;
    case IMU_CALIB_ACC:
        return s_st.acc_poses == 0 ? "no poses yet" : s_st.acc_learned_bad;
    case IMU_CALIB_MAG:
        if (!s_st.mag_solved) {
            return s_st.mag_state ? s_st.mag_state : "not fitted yet";
        }
        return s_st.mag_learned_bad;
    case IMU_CALIB_GYR_M:
        return s_st.gyr_taken == 0 ? "no element pinned down yet" : s_st.gyr_next_bad;
    default:
        return "unknown group";
    }
}

static bool group_stored(int g)
{
    return g == IMU_CALIB_GYR_M ? s_st.gyr_m_applied : s_p.keep_valid[g] != 0;
}

static const char *next_boot(int g)
{
    const bool kept = group_stored(g);
    if (s_p.hold[g]) {
        return kept ? "the held set" : "no correction (held, nothing stored)";
    }
    if (g == IMU_CALIB_GYR_M) {
        if (s_st.gyr_taken > 0 && !s_st.gyr_next_bad) {
            static char b[64];
            snprintf(b, sizeof b, "the matrix with %d of 9 elements updated", s_st.gyr_taken);
            return b;
        }
        return kept ? "the matrix in use" : "identity (no correction)";
    }
    if (accept_check(g) == NULL) {
        return "the learned set";
    }
    return kept ? "the stored set (no usable data)" : "no correction";
}

/* The rotation and turn trackers integrate with the bias and matrix in use:
 * start them again when either changes. */
static void reset_motion(void)
{
    rot_preint_reset(&s_rot);
    s_rot_state = ROT_IDLE;
    s_turn.active = false;
}

static void clear_data(int g)
{
    switch (g) {
    case IMU_CALIB_GYR:
        gcal_stats_clear(&s_p.stats);
        break;
    case IMU_CALIB_ACC:
        memset(s_p.bins, 0, sizeof s_p.bins);
        break;
    case IMU_CALIB_MAG:
        memset(s_p.mag, 0, sizeof s_p.mag);
        mcal_ref_clear(&s_p.ref);
        break;
    case IMU_CALIB_GYR_M:
        rot_stats_clear(&s_p.rot);
        reset_motion();
        break;
    default:
        break;
    }
}

static void factory(int g)
{
    static const char k_note[] = "reset to factory (web page)";
    if (g < 0 || g >= IMU_CALIB_GROUPS) {
        return;
    }
    clear_data(g);
    switch (g) {
    case IMU_CALIB_GYR:
        s_p.hold[IMU_CALIB_GYR] = 0;
        s_p.keep_valid[IMU_CALIB_GYR] = 0;
        s_st.applied = false;
        memset(&s_st.use, 0, sizeof s_st.use);
        memset(s_b0_use, 0, sizeof s_b0_use);
        memset(s_k_use, 0, sizeof s_k_use);
        reset_motion();
        s_st.boot_note = k_note;
        break;
    case IMU_CALIB_ACC:
        s_p.hold[IMU_CALIB_ACC] = 0;
        s_p.keep_valid[IMU_CALIB_ACC] = 0;
        s_st.acc_applied = false;
        memset(&s_st.acc_use, 0, sizeof s_st.acc_use);
        memset(s_am_use, 0, sizeof s_am_use);
        memset(s_ab_use, 0, sizeof s_ab_use);
        s_st.acc_boot_note = k_note;
        break;
    case IMU_CALIB_MAG:
        s_p.hold[IMU_CALIB_MAG] = 0;
        s_p.keep_valid[IMU_CALIB_MAG] = 0;
        s_st.mag_applied = false;
        memset(&s_st.mag_use, 0, sizeof s_st.mag_use);
        memset(s_mm_use, 0, sizeof s_mm_use);
        memset(s_mb_use, 0, sizeof s_mb_use);
        s_st.mag_boot_note = k_note;
        break;
    case IMU_CALIB_GYR_M:
        s_p.hold[IMU_CALIB_GYR_M] = 0;
        memcpy(s_gm_use, k_ident, sizeof k_ident);
        memcpy(s_p.gyr_m0, k_ident, sizeof k_ident);
        s_st.gyr_m_applied = false;
        reset_motion();
        s_st.gyr_m_boot_note = k_note;
        break;
    default:
        break;
    }
}

static void accept(int g)
{
    static const char k_note[] = "accepted (web page)";
    switch (g) {
    case IMU_CALIB_GYR: {
        const gcal_result_t r = s_st.learned;
        use_gyr(&r);
        reset_motion();
        s_st.boot_note = k_note;
        break;
    }
    case IMU_CALIB_ACC: {
        const acal_result_t r = s_st.acc_learned;
        use_acc(&r);
        s_st.acc_boot_note = k_note;
        break;
    }
    case IMU_CALIB_MAG: {
        const mcal_cal_t c = s_st.mag_learned;
        use_mag(&c);
        s_st.mag_boot_note = k_note;
        break;
    }
    case IMU_CALIB_GYR_M:
        memcpy(s_gm_use, s_st.gyr_next, sizeof s_gm_use);
        memcpy(s_p.gyr_m0, s_st.gyr_next, sizeof s_p.gyr_m0);
        rot_stats_clear(&s_p.rot); /* linearised at the old matrix */
        reset_motion();
        s_st.gyr_m_applied = memcmp(s_gm_use, k_ident, sizeof k_ident) != 0;
        s_st.gyr_m_boot_note = k_note;
        break;
    default:
        break;
    }
}

int imu_calib_action(int g, int op, char *msg, size_t cap)
{
    if (g < 0 || g > IMU_CALIB_GROUPS || op < 0 || op >= IMU_CALIB_OPS ||
        (g == IMU_CALIB_GROUPS && op != IMU_CALIB_OP_FACTORY)) {
        snprintf(msg, cap, "refused: unknown calibration group or action");
        return -1;
    }
    const char *gn = k_gname[g];
    if (!s_learn) {
        snprintf(msg, cap, "refused: %s: the TM210 is %s", gn,
                 s_st.id_state == IMU_CALIB_ID_WAIT ? "not identified yet" : "running without a node ID");
        return -1;
    }
    const bool live = op == IMU_CALIB_OP_ACCEPT || op == IMU_CALIB_OP_FACTORY;
    if (live && !s_run) {
        snprintf(msg, cap, "refused: %s: hold the unit still first (the filter restarts)", gn);
        return -1;
    }
    switch (op) {
    case IMU_CALIB_OP_ACCEPT: {
        solve_learned();
        solve_mag_learned();
        solve_gyr_learned();
        const char *why = accept_check(g);
        if (why != NULL) {
            snprintf(msg, cap, "refused: %s: nothing to accept (%s)", gn, why);
            return -1;
        }
        accept(g);
        break;
    }
    case IMU_CALIB_OP_HOLD:
        s_p.hold[g] = 1;
        break;
    case IMU_CALIB_OP_AUTO:
        s_p.hold[g] = 0;
        break;
    case IMU_CALIB_OP_CLEAR:
        clear_data(g);
        break;
    case IMU_CALIB_OP_FACTORY:
        if (g == IMU_CALIB_GROUPS) {
            for (int i = 0; i < IMU_CALIB_GROUPS; i++) {
                factory(i);
            }
            s_st.chunks_boot = 0;
            s_st.poses_boot = 0;
            s_st.mag_groups_boot = 0;
            s_st.mag_ref_boot = 0;
            s_st.rot_boot = 0;
            s_st.turn_boot = 0;
        } else {
            factory(g);
        }
        break;
    default:
        break;
    }
    solve_learned();
    solve_mag_learned();
    solve_gyr_learned();

    const uint64_t t0 = time_us_64();
    const int rc = flash_rec_save(&s_rec, &s_p);
    s_st.last_save_rc = rc;
    if (rc == 0) {
        s_st.unsaved = 0;
        s_st.rec_seq = flash_rec_seq(&s_rec);
        s_last_save_us = t0 ? t0 : 1;
    }
    static const char *const k_done[IMU_CALIB_OPS] = {
        "accepted, in use now (filter restarted)",
        "held: boot keeps the stored set until you accept",
        "auto: boot applies the learned set",
        "collected data erased; the set in use stays",
        "reset to factory: no correction from now (filter restarted)",
    };
    if (rc == 0) {
        snprintf(msg, cap, "%s: %s; record %lu", gn, k_done[op], (unsigned long)s_st.rec_seq);
    } else {
        snprintf(msg, cap, "%s: %s, but the flash save failed (error %d): lost at reboot", gn, k_done[op], rc);
    }
    return live ? 1 : 0;
}

void imu_calib_status(imu_calib_status_t *out)
{
    for (int g = 0; g < IMU_CALIB_GROUPS; g++) {
        s_st.hold[g] = s_p.hold[g] != 0;
        s_st.stored[g] = group_stored(g);
        s_st.accept_why[g] = accept_check(g);
        s_st.next_boot[g] = next_boot(g);
    }
    *out = s_st;
}
