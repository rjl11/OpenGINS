/* Survey-pole aiding from natural sway (plans/pole-tilt-aiding.md).
 *
 * Three helpers for the INS on the pole platform, none of which needs the
 * user to do anything special:
 *
 *  1. Planted-tip detector. The specific force moved to the pole tip,
 *     f_T = f + w x (w x l) + dw/dt x l (Ai, Chen, Niu, IEEE TIM 2026, eq. 14,
 *     docs/survey-pole/), stays at g while the tip rests on the ground, however
 *     the top sways. Plus the INS's own tip velocity, so a pole carried at a
 *     steady speed doesn't pass.
 *  2. While planted, a latched tip position for INSLIB's local-position input
 *     (lever arm = IMU to tip): the pole-tip constraint of the same paper. The
 *     GNSS antenna then moves on a sphere around a fixed point, which makes
 *     roll, pitch, the gyro bias and, with RTK, the yaw observable.
 *  3. A yaw fix per planted episode: the antenna fixes against the antenna
 *     offsets from the tip that the INS attitude predicts, p_i = t + Rz(dpsi)
 *     a_i, solved linearly for (t, cos dpsi, sin dpsi). Works from any yaw
 *     error, unlike the filter's small-angle update (track matching after
 *     Chen et al., GPS Solutions 2020).
 *  4. A magnetometer trust gate: field strength and dip against the WMM while
 *     the attitude varies, and the heading against the INS once its yaw is
 *     known. The magnetometer reaches INSLIB only while trusted.
 *  5. Per planted episode, the sums for learning the pole axis in the IMU
 *     frame (pole_mount_fit.h, on core 0): the same RTK fixes against an
 *     attitude integrated from the gyro alone since the episode began, so
 *     the tip constraint (fed with the axis in use) can't bend it.
 *
 * Plain C for core 1 (placed in SRAM by firmware/link/default_text_excludes.incl);
 * host test firmware/tools/test_pole_aid.c. Rotation matrices are body to NED,
 * column-major (INSLIB's ins_get_rotmat_b_to_n): R[i + 3 * j] is row i, column j.
 */
#ifndef POLE_AID_H
#define POLE_AID_H

#include <stdbool.h>
#include <stdint.h>

#define POLE_WIN_BLOCKS  5    /* detector window: 5 blocks of 50 ms        */
#define POLE_DIFF_N      8    /* gyro derivative over 8 samples (10 ms)     */
#define POLE_ATT_N       64   /* attitude ring, 100 Hz: 0.64 s              */
#define POLE_MAG_BLOCKS  10   /* magnetometer window: 10 blocks of 1 s      */

/* One planted episode's sums for the pole-axis fit (pole_mount_fit.h). Per
 * RTK fix i: p_i the antenna (north, east) from the episode's first fix [m],
 * G_i the top two rows of the gyro-integrated attitude R_i (body to NED).
 * Kept so that the yaw correction can be applied afterwards: the model is
 * p_i = t + Rz(dpsi) G_i d, d = antenna - tip in the body frame. */
typedef struct {
    uint16_t n;            /* fixes                                         */
    uint16_t pad;
    float    dpsi;         /* the episode's yaw fit (true = INS + dpsi) [rad] */
    float    twist_deg;    /* rotation about the pole axis, max - min       */
    float    sp[2];        /* sum p_i                                       */
    float    sg[2][3];     /* sum G_i                                       */
    float    sgg[3][3];    /* sum G_i^T G_i                                 */
    float    sgp[3][2][2]; /* [c][j][k] = sum G_i[j][c] p_i[k]              */
    float    spp;          /* sum |p_i|^2                                   */
} pole_episode_t;

/* Adds one fix to e (also used by the host test). R column-major. */
void pole_episode_add(pole_episode_t *e, const float R[9], float pN, float pE);

/* Magnetometer trust gate (pole_aid_t cfg defaults, and the reason text on
 * the web pages, main.c mag_reason()). */
#define POLE_MAG_NORM_TOL        0.10f  /* fraction: |B| against the model */
#define POLE_MAG_DIP_TOL_DEG     5.0f
#define POLE_MAG_TILT_RANGE_DEG  5.0f
#define POLE_MAG_YAW_RANGE_DEG   20.0f
#define POLE_MAG_YAW_SD_MAX_DEG  5.0f
#define POLE_MAG_RES_MEAN_MAX_DEG 5.0f
#define POLE_MAG_RES_SD_MAX_DEG  3.0f

typedef struct {
    bool     active;       /* pole platform with an antenna height          */
    bool     planted;      /* tip resting on the ground now                 */
    bool     latched;      /* tip constraint being fed                      */
    bool     mag_trusted;
    uint32_t plants;       /* planted episodes                              */
    uint32_t tip_fusions;  /* local-position constraints handed to INSLIB   */
    uint32_t yaw_fits;     /* accepted yaw fixes                            */
    uint32_t yaw_rejects;  /* fits tried and rejected                       */
    uint32_t episodes;     /* episode records offered for the axis fit      */
    float    twist_deg;    /* current or last episode: turn about the pole  */
    float    tip_err_ms2;  /* last window: mean ||f_T| - g|                 */
    float    tip_speed_ms; /* last window: max INS tip speed, -1 = no INS   */
    /* The last fit tried. */
    float    fit_dpsi_deg, fit_sd_deg, fit_spread_m, fit_rms_m, fit_scale;
    uint16_t fit_n;
    bool     fit_ok;
    bool     fit_skipped;  /* too few fixes or too little sway: not solved,
                              not a rejection; only n and spread are set */
    /* The last full magnetometer window. */
    float    mag_norm_err_pct;  /* worst |B|/|B_wmm| - 1 */
    float    mag_dip_err_deg;   /* worst dip error */
    float    mag_tilt_range_deg, mag_yaw_range_deg;
    float    mag_res_mean_deg, mag_res_sd_deg; /* heading vs INS, NAN = not checked */
    int      mag_blocks;        /* filled 1 s blocks in the last window; 0 = no field model */
} pole_aid_status_t;

typedef struct {
    float g;               /* gravity used by the detector                  */
    float err_mean_max;    /* window mean ||f_T| - g| [m/s^2]               */
    float err_peak_max;    /* any 50 ms block's peak [m/s^2]                */
    float tip_speed_max;   /* INS tip speed [m/s]                           */
    float settle_s;        /* planted this long before latching             */
    float feed_period_s;   /* tip constraint rate                           */
    float episode_max_s;   /* fit and relatch at least this often           */
    float fit_sd_max_m;    /* GNSS horizontal sigma to count as RTK         */
    uint16_t fit_min_n;
    float fit_min_spread_m;
    float fit_max_rms_m;
    float fit_max_scale_err;
    float fit_min_sd_deg;
    float mag_norm_tol;    /* fraction                                      */
    float mag_dip_tol_deg;
    float mag_tilt_range_deg, mag_yaw_range_deg;
    float mag_yaw_sd_max_deg;   /* INS yaw known enough to check headings */
    float mag_res_mean_max_deg, mag_res_sd_max_deg;
} pole_aid_cfg_t;

typedef struct {
    pole_aid_cfg_t cfg;
    float tip_b[3];        /* IMU to pole tip, body FRD                     */
    float ant_b[3];        /* IMU to GNSS antenna, body FRD                 */
    float axis_b[3];       /* unit, antenna to tip                          */

    /* Detector. */
    float    w_lp[3];
    float    w_hist[POLE_DIFF_N][3];
    float    t_hist[POLE_DIFF_N];
    uint8_t  hist_i, hist_n;
    float    t_s;               /* seconds since reset (detector clock)    */
    float    blk_sum, blk_peak, blk_vmax;
    bool     blk_vknown;
    uint16_t blk_cnt;
    float    blk_t0;
    float    win_mean[POLE_WIN_BLOCKS], win_peak[POLE_WIN_BLOCKS], win_v[POLE_WIN_BLOCKS];
    bool     win_vknown[POLE_WIN_BLOCKS];
    uint8_t  win_i, win_n;
    float    t_planted;
    float    t_next_feed;
    float    t_episode;
    float    tip_n[3];          /* latched tip, INSLIB local NED            */
    bool     latch_wanted;

    /* Attitude ring for the fit (time of validity of the GNSS fixes). */
    uint64_t att_t[POLE_ATT_N];
    float    att_R[POLE_ATT_N][9];
    uint8_t  att_i, att_n;
    uint64_t att_next_us;

    /* Yaw fit of the current episode. */
    bool     fit_origin;
    double   lat0, lon0;
    float    k_n, k_e;          /* metres per radian                        */
    double   S[4][4], b[4], pp;
    double   a_sum[2], a_sq;
    uint16_t fit_n;
    bool     yaw_ready;
    float    yaw_dpsi, yaw_sd;

    /* Pole-axis sums (item 5): attitude from the gyro alone while planted,
     * re-synced to the INS between episodes. */
    float    qg[4];             /* body to NED, w x y z                     */
    bool     qg_ok;
    bool     qg_resync;         /* take the INS attitude at the next pole_aid_att() */
    float    att_q[POLE_ATT_N][4];
    float    twist, twist_min, twist_max;
    pole_episode_t ep;
    pole_episode_t ep_out;
    bool     ep_ready;

    /* Magnetometer trust. */
    struct {
        uint32_t n, n_res;
        float    norm_err, dip_err;
        float    res_sum, res_sq;
        float    tilt_min, tilt_max;
        float    yaw_c, yaw_s;
    } mag_blk[POLE_MAG_BLOCKS], mag_cur;
    uint8_t  mag_i, mag_n;
    uint64_t mag_t0_us;

    pole_aid_status_t st;
} pole_aid_t;

void pole_aid_default_cfg(pole_aid_cfg_t *c);

/* Starts over (filter restart). active false: everything stays idle. */
void pole_aid_reset(pole_aid_t *p, const pole_aid_cfg_t *cfg, bool active, const float tip_b[3],
                    const float ant_b[3]);

/* New geometry (antenna height changed) without losing the rest. */
void pole_aid_set_geometry(pole_aid_t *p, bool active, const float tip_b[3], const float ant_b[3]);

/* One IMU sample: specific force and rate (body FRD; the rate with the
 * calibration and the INS bias applied when known: it is also integrated
 * for item 5), dt, and the INS's predicted tip velocity (NED) or NULL while
 * the INS isn't ready. Returns whether the tip counts as planted. */
bool pole_aid_imu(pole_aid_t *p, float dt, const float f[3], const float w[3], const float *v_tip_n);

/* The INS attitude at t (kept at 100 Hz for the fit). Also the start of the
 * gyro-only attitude of item 5 while not planted. */
void pole_aid_att(pole_aid_t *p, uint64_t t_us, const float R[9]);

/* True once per latch: the caller latches the tip (its INS position + R l)
 * with pole_aid_latch(). */
bool pole_aid_want_latch(const pole_aid_t *p);
void pole_aid_latch(pole_aid_t *p, const float tip_n[3]);

/* True when a tip constraint is due now; tip_n is the latched tip. */
bool pole_aid_feed_due(pole_aid_t *p, float tip_n[3]);

/* A GNSS fix (antenna), horizontal 1-sigma. */
void pole_aid_gnss(pole_aid_t *p, uint64_t t_valid_us, double lat_rad, double lon_rad,
                   float sd_h_m);

/* True once per accepted fit: dpsi is the yaw correction (true yaw = INS
 * yaw + dpsi), sd its 1-sigma [rad]. */
bool pole_aid_take_yaw(pole_aid_t *p, float *dpsi, float *sd);

/* True once per episode whose yaw fit was solved with enough sway and a tip
 * that stayed put (the RMS gate is left to the axis fit: a mounting error
 * shows as a residual once the pole twists). */
bool pole_aid_take_episode(pole_aid_t *p, pole_episode_t *out);

/* Quaternion helpers (also used by the host test): body to NED, w x y z;
 * R column-major. */
void pole_quat_from_R(const float R[9], float q[4]);
void pole_quat_to_R(const float q[4], float R[9]);
void pole_quat_step(float q[4], const float w[3], float dt);

/* One calibrated magnetometer sample (body, uT) with the attitude (INS, or
 * the gyro-only filter with a free yaw: yaw_abs false), the INS yaw 1-sigma
 * (or a large value) and the WMM field (NED, uT; zero = unknown). Returns
 * whether the magnetometer is trusted now. */
bool pole_aid_mag(pole_aid_t *p, uint64_t t_us, const float b[3], const float R[9], bool yaw_abs,
                  float yaw_sd_rad, const float m_n[3]);

const pole_aid_status_t *pole_aid_status(const pole_aid_t *p);

#endif
