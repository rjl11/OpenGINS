#include "nav_core1.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "hardware/clocks.h"
#include "pico/critical_section.h"
#include "pico/flash.h"
#include "pico/multicore.h"
#include "pico/time.h"
#include "pico/util/queue.h"

#include "log.h"
#include "nav_suite.h"

#include "app_config.h"

#define IMU_QUEUE_LEN       128 /* 160 ms at 800 Hz */
#define GNSS_QUEUE_LEN      8
#define BARO_QUEUE_LEN      32
#define SPEED_QUEUE_LEN     16
#define LOG_QUEUE_LEN       8
#define POLE_QUEUE_LEN      4   /* pole_episode_t, one per planted episode */
#define LOG_MSG_LEN         200 /* the longest INSLIB line, its 3D gate breakdown, is about 170 */
#define PUBLISH_PERIOD_US   100000u
/* No IMU sample for this long: core 1 steps the filter without one, every
 * STALL_EPOCH_US, so INSLIB sees the loss (it checks the time since the last
 * IMU sample on every epoch, REQ-NAV-089 / REQ-SUITE-027, default 0.2 s) and
 * stops instead of leaving its last solution standing. The TM210 runs at
 * 800 Hz, so 50 ms is 40 samples missing. */
#define IMU_STALL_US        50000u
#define STALL_EPOCH_US      50000u
#define STACK_FILL          0xA5A5A5A5u

static queue_t s_imu_q, s_gnss_q, s_baro_q, s_speed_q, s_log_q, s_pole_q;
static critical_section_t s_out_lock;
static nav_output_t s_out;                 /* guarded by s_out_lock */
static volatile uint32_t s_out_seq;        /* bumped by core 1 after each publish */
static volatile nav_antenna_t s_mode;
static nav_drops_t s_drops;                /* core 0 only */
static nav_params_t s_params_new;          /* guarded by s_out_lock */
static volatile uint32_t s_params_seq;     /* bumped by core 0 on a change */

/* AAPCS wants the stack pointer on an 8-byte boundary. */
static uint32_t s_stack[APP_CORE1_STACK_BYTES / sizeof(uint32_t)] __attribute__((aligned(8)));

/* Core 1 only. */
static nav_params_t s_p;                   /* settings the filter runs with */
static nav_suite_t s_suite;
static nav_output_t s_work;
static uint64_t s_update_us_sum;
static uint32_t s_update_count;
static bool s_mag_model_set;
static uint64_t s_run_t0_us;               /* first IMU sample of this run, 0 = none, MAX = settled */
static cpu_load_t s_load;
static uint32_t s_clk_hz;                  /* set by core 0 before the launch */
static uint64_t s_last_imu_us;             /* last IMU sample, 0 = none yet */
static uint64_t s_next_stall_epoch_us;
static pole_aid_t s_pole;                  /* survey-pole aiding (pole_aid.h) */

/* ---- Core 1 ------------------------------------------------------------ */

/* INSLIB's messages are formatted here and printed by core 0, so core 1
 * never waits for USB. */
static void log_sink(int level, const char *file, int line, const char *fmt, va_list args)
{
    static const char *const k_level[] = { "", "FATAL", "ERROR", "WARN", "INFO" };
    char msg[LOG_MSG_LEN];
    /* File name only: the build's absolute path would fill the message. */
    for (const char *p = file; *p != '\0'; p++) {
        if (*p == '/' || *p == '\\') {
            file = p + 1;
        }
    }
    int n = snprintf(msg, sizeof msg, "%s %s:%d: ", k_level[(level >= 1 && level <= 4) ? level : 0],
                     file, line);
    if (n > 0 && (size_t)n < sizeof msg) {
        vsnprintf(msg + n, sizeof msg - (size_t)n, fmt, args);
    }
    queue_try_add(&s_log_q, msg);
}

static void filter_init(nav_antenna_t mode)
{
    ins_init_t init;
    ins_options_t opt;

    memset(&s_suite, 0, sizeof s_suite);
    memset(&init, 0, sizeof init);
    memset(&opt, 0, sizeof opt);

    /* Position stays zero: auto-init replaces it with the first good fix. */
    init.time = (ins_time_us_t)time_us_64();
    init.gyr_bias_pred_stddev_rps_sqrts = s_p.gyr_bias_rw;
    init.acc_bias_pred_stddev_mps2_sqrts = s_p.acc_bias_rw;

    opt.auto_init = true;
    /* Course yaw only with one antenna. In dual mode it adds nothing and
     * pulls the wrong way when reversing (plans/gnss-um98x-config.md). And
     * only on a platform whose course is its heading: on a survey pole or a
     * multirotor INS yaw comes from the GNSS velocity while the body moves
     * and accelerates (and the magnetometer, if fitted). */
    opt.automotive_mode = mode != NAV_ANT_DUAL && s_p.course_yaw;
    opt.automotive_min_speed_mps = s_p.automotive_min_speed;
    opt.max_deadreckoning_sec = s_p.max_deadreckoning_s;
    opt.gnss_pos_decimation = s_p.gnss_pos_decim;
    /* Keep the GNSS height row (nav_params_t.baro_height_off, web setting
     * height_src). */
    opt.baro_height_disable = s_p.baro_height_off;
    opt.estimate_mag_bias = s_p.use_mag && APP_ESTIMATE_MAG_BIAS;
    /* RTK-grade floors (0 would mean INSLIB's single-frequency defaults);
     * main.c raises the position sigma itself for the weaker fix types. */
    opt.gnss_pos_stddev_floor_hor_m = APP_GNSS_FLOOR_POS_HOR_M;
    opt.gnss_pos_stddev_floor_ver_m = APP_GNSS_FLOOR_POS_VER_M;
    opt.gnss_vel_stddev_floor_hor_mps = APP_GNSS_FLOOR_VEL_HOR_MPS;
    opt.gnss_vel_stddev_floor_ver_mps = APP_GNSS_FLOOR_VEL_VER_MPS;
    /* nav_suite_init() copies it into the ARS/AHRS stillness detectors. */
    opt.auto_zupt_static_gyr_stddev_rps = APP_STILL_GYR_STDDEV_DPS * 0.017453293f;
    /* nav_suite applies it to every sub-filter (ins_apply_calibration). The
     * misalignment matrices stay all-zero, which INSLIB reads as identity. */
    for (int i = 0; i < 3; i++) {
        opt.imu_gyr_fixed_bias[i] = s_p.gyr_fixed_bias[i];
        opt.imu_acc_fixed_bias[i] = s_p.acc_fixed_bias[i];
    }
    for (int i = 0; i < 9; i++) {
        opt.imu_acc_misalignment[i] = s_p.acc_misalignment[i];
        opt.mag_misalignment[i] = s_p.mag_misalignment[i];
        opt.imu_gyr_misalignment[i] = s_p.gyr_misalignment[i];
    }
    for (int i = 0; i < 3; i++) {
        opt.mag_fixed_bias[i] = s_p.mag_fixed_bias[i];
    }

    nav_suite_init(&s_suite, &init, &opt);
    /* Offset filter tuning, with either height source (web settings,
     * defaults APP_HEIGHT_OFFSET_*; 0 = INSLIB's default). With the
     * barometric INS height its sigma also sets how much of the weather
     * drift stays in the published ellipsoid height (app_config.h).
     * nav_suite_init() zeroes the template, and the filter is built from it
     * at its first baro/GNSS pair, so setting it here is in time. */
    s_suite.local_gnss_cfg.min_update_interval_sec = s_p.offset_interval_s;
    s_suite.local_gnss_cfg.stddev_inflation_factor = s_p.offset_inflation;
    s_suite.local_gnss_cfg.rw_stddev_mps = s_p.offset_rw_mps;
    /* The barometer sigma only with the barometric INS height, where the
     * offset's sigma stays in the published height. With the GNSS height
     * INSLIB's 2 m averages more pairs: 2 cm rms handover error against
     * 4 cm with 0.1 m in the overnight log (app_config.h). */
    s_suite.local_gnss_cfg.local_stddev_m = s_p.baro_height_off ? 0.0f : s_p.offset_baro_sd_m;
    s_mag_model_set = false;
    pole_aid_cfg_t pc;
    pole_aid_default_cfg(&pc);
    pc.err_mean_max = APP_POLE_TIP_ERR_MAX;
    pc.tip_speed_max = APP_POLE_TIP_SPEED_MAX;
    pc.fit_sd_max_m = APP_POLE_FIT_GNSS_SD_MAX_M;
    pole_aid_reset(&s_pole, &pc, s_p.pole_aid, s_p.tip_leverarm_m, s_p.leverarm_m);
    s_work.dual = mode == NAV_ANT_DUAL;
    s_work.n_reinit++;
    s_run_t0_us = 0;
}

/* Startup counts (samples that piled up before the filter ran) are cleared
 * NAV_SETTLE_US after the first IMU sample of each filter (re)start. */
static void settle_check(uint64_t imu_t_us)
{
    if (s_run_t0_us == 0) {
        s_run_t0_us = imu_t_us ? imu_t_us : 1;
        return;
    }
    if (s_run_t0_us != UINT64_MAX && imu_t_us - s_run_t0_us >= NAV_SETTLE_US) {
        s_work.n_gnss_late = 0;
        s_work.n_baro_late = 0;
        s_work.n_speed_late = 0;
        s_work.t_settled_us = imu_t_us;
        s_run_t0_us = UINT64_MAX; /* done for this run */
    }
}

static void set_diag(float *dst, float a, float b, float c)
{
    /* 3x3 column-major, diagonal only */
    dst[0] = a;
    dst[4] = b;
    dst[8] = c;
}

static void attach_gnss(ins_measurements_t *m, const nav_gnss_t *g)
{
    int64_t age_us = (int64_t)m->timestamp - (int64_t)g->t_valid_us;
    if (age_us < 0) {
        age_us = 0;
    }
    if (age_us >= (int64_t)INS_MAX_DELAY_MS * 1000) {
        s_work.n_gnss_late++;
        return;
    }
    if (g->yaw_valid) {
        m->yaw.yaw_rad = g->yaw_rad;
        m->yaw.stddev_rad = g->yaw_sd_rad;
        m->yaw.is_valid = true;
        m->yaw_delay_ms = (int)(age_us / 1000);
        m->yaw_delay_us = (int32_t)(age_us % 1000);
        s_work.n_yaw++;
    }
    if (!g->pos_valid && !g->vel_valid) {
        return;
    }

    if (g->pos_valid) {
        m->gnss_pos.llh[0] = g->lat_rad;
        m->gnss_pos.llh[1] = g->lon_rad;
        m->gnss_pos.llh[2] = g->h_ell_m;
        set_diag(m->gnss_pos.Qll_ned, g->pos_sd_ned[0] * g->pos_sd_ned[0],
                 g->pos_sd_ned[1] * g->pos_sd_ned[1], g->pos_sd_ned[2] * g->pos_sd_ned[2]);
        m->gnss_pos.is_valid = true;
    }
    if (g->vel_valid) {
        memcpy(m->gnss_vel.vel_ned, g->vel_ned, sizeof m->gnss_vel.vel_ned);
        set_diag(m->gnss_vel.Qll_ned, g->vel_sd_ned[0] * g->vel_sd_ned[0],
                 g->vel_sd_ned[1] * g->vel_sd_ned[1], g->vel_sd_ned[2] * g->vel_sd_ned[2]);
        m->gnss_vel.is_valid = true;
    }
    if (s_p.pole_aid && g->pos_valid) {
        pole_aid_gnss(&s_pole, g->t_valid_us, g->lat_rad, g->lon_rad,
                      sqrtf(g->pos_sd_ned[0] * g->pos_sd_ned[0] + g->pos_sd_ned[1] * g->pos_sd_ned[1]));
    }
    m->gnss_leverarm_b[0] = s_p.leverarm_m[0];
    m->gnss_leverarm_b[1] = s_p.leverarm_m[1];
    m->gnss_leverarm_b[2] = s_p.leverarm_m[2];
    m->gps_week = g->week;
    m->gps_itow_ms = (int)g->tow_ms;
    m->gnss_delay_ms = (int)(age_us / 1000);
    m->gnss_delay_us = (int32_t)(age_us % 1000);
    s_work.n_gnss++;

    if (g->pos_valid && !s_mag_model_set) {
        /* GPS week 0 began on 1980-01-06. */
        float year = 1980.0f + (5.0f + (float)g->week * 7.0f) / 365.25f;
        nav_suite_set_magnetic_model_position(&s_suite, g->lat_rad, g->lon_rad, year);
        s_mag_model_set = true;
    }
}

/* Ground speed (REQ-NAV-068) or body velocity (REQ-NAV-096), history-
 * anchored like GNSS. INSLIB's own
 * speed_scale / speed_stddev_rel stay at their defaults: each source puts
 * its scale and its proportional error into the sample. */
static void attach_speed(ins_measurements_t *m, const nav_speed_t *sp)
{
    int64_t age_us = (int64_t)m->timestamp - (int64_t)sp->t_valid_us;
    if (age_us < 0) {
        age_us = 0;
    }
    if (age_us >= (int64_t)INS_MAX_DELAY_MS * 1000) {
        s_work.n_speed_late++;
        return;
    }
    if (sp->has_vel_b) {
        /* Body velocity (REQ-NAV-096): x / y measured, z left out by its
         * zero variance; the lever arm is applied already (flow_nav.c). */
        m->body_vel.vel_b[0] = sp->vel_b[0];
        m->body_vel.vel_b[1] = sp->vel_b[1];
        m->body_vel.vel_b[2] = 0.0f;
        memset(m->body_vel.Qll_b, 0, sizeof m->body_vel.Qll_b);
        m->body_vel.Qll_b[0] = sp->Qll_b[0];
        m->body_vel.Qll_b[1] = sp->Qll_b[1];
        m->body_vel.Qll_b[3] = sp->Qll_b[1];
        m->body_vel.Qll_b[4] = sp->Qll_b[2];
        m->body_vel.is_valid = true;
        memset(m->body_vel_leverarm_b, 0, sizeof m->body_vel_leverarm_b);
        m->body_vel_delay_ms = (int)(age_us / 1000);
        m->body_vel_delay_us = (int32_t)(age_us % 1000);
    } else {
        m->speed.speed_mps = sp->speed_mps;
        m->speed.stddev_mps = sp->sd_mps;
        m->speed.is_valid = true;
        m->speed_delay_ms = (int)(age_us / 1000);
        m->speed_delay_us = (int32_t)(age_us % 1000);
    }
    if (sp->source < NAV_SPEED_SOURCES) {
        s_work.n_speed[sp->source]++;
    }
}

static void publish(uint64_t now)
{
    nav_output_t *o = &s_work;

    o->t_us = now;
    o->mode = (int)nav_suite_get_mode(&s_suite);
    o->rpy_valid = nav_suite_get_rpy(&s_suite, &o->rpy_rad[0], &o->rpy_rad[1], &o->rpy_rad[2]);
    o->pos_valid = ins_is_ready(&s_suite.ins) && ins_get_latlonh(&s_suite.ins, o->llh);
    /* With the barometer, INSLIB's vertical datum is fixed once at bootstrap
       and never re-tied to GNSS, so ins's own height carries the barometer's
       weather drift (about 24 m in the 10 h log of 2026-11-15). The suite's
       ellipsoid height takes the drift its offset filter has seen back off. */
    float h_ell;
    if (o->pos_valid && nav_suite_get_height_ellipsoid(&s_suite, &h_ell)) o->llh[2] = h_ell;
    o->vel_valid = ins_is_ready(&s_suite.ins) && ins_get_velocity_ned(&s_suite.ins, o->vel_ned);
    o->height_valid = nav_suite_get_height(&s_suite, &o->height_m);
    o->bias_valid = ins_get_bias_gyr(&s_suite.ins, o->gyr_bias) &&
                    ins_get_bias_acc(&s_suite.ins, o->acc_bias);
    o->zaru_active = nav_suite_get_zaru_active(&s_suite);
    if (s_last_imu_us == 0) {
        o->imu_age_ms = -1;
    } else {
        uint64_t age = now > s_last_imu_us ? (now - s_last_imu_us) / 1000u : 0;
        o->imu_age_ms = age > INT32_MAX ? INT32_MAX : (int32_t)age;
    }
    o->n_imu_loss = s_suite.n_imu_loss;
    const ins_diag_t *d = ins_get_diag(&s_suite.ins);
    o->n_speed_used = d->n_speed_used + d->n_body_vel_used;
    o->n_speed_skipped = d->n_speed_skipped + d->n_body_vel_skipped;

    /* Same order as nav_suite_get_rpy(). */
    float r, p, y;
    if (!o->rpy_valid) {
        o->att_source = NAV_ATT_NONE;
    } else if (ins_is_ready(&s_suite.ins) && nav_suite_get_rpy_ins(&s_suite, &r, &p, &y)) {
        o->att_source = NAV_ATT_INS;
    } else if (nav_suite_get_rpy_ahrs(&s_suite, &r, &p, &y)) {
        o->att_source = NAV_ATT_AHRS;
    } else {
        o->att_source = NAV_ATT_ARS;
    }
    o->pole = *pole_aid_status(&s_pole);
    o->rpy_sd_valid = o->att_source == NAV_ATT_INS &&
                      ins_get_rpy_stddev(&s_suite.ins, &o->rpy_sd_rad[0], &o->rpy_sd_rad[1], &o->rpy_sd_rad[2]);

    /* INSLIB has no getters for these; they are plain ahrs_t fields. */
    const ahrs_t *a = &s_suite.ahrs;
    o->ahrs_valid = a->is_initialized;
    if (a->t_last_mag_fusion == 0) {
        o->mag_fusion_age_ms = -1;
    } else {
        int64_t age = (int64_t)now - (int64_t)a->t_last_mag_fusion;
        o->mag_fusion_age_ms = age < 0 ? 0 : (age > INT32_MAX / 2 ? INT32_MAX / 2 : (int32_t)(age / 1000));
    }
    o->mag_declination = a->declination_applied;
    o->mag_on_magnetic_north = a->yaw_on_magnetic_north;
    o->update_us_mean = s_update_count ? (uint32_t)(s_update_us_sum / s_update_count) : 0;
    if (now - s_load.win_start_us >= 1000000u) {
        cpu_load_take(&s_load, now, &o->load);
    }

    critical_section_enter_blocking(&s_out_lock);
    s_out = *o;
    critical_section_exit(&s_out_lock);
    s_out_seq++; /* after the copy, so a reader that sees it gets the new output */

    o->update_us_max = 0;
    s_update_us_sum = 0;
    s_update_count = 0;
}

/* Samples that arrived since the last epoch ride on this one. is_valid is
 * set only for a new sample. drain: take every queued barometer sample and
 * keep the newest (stall epochs come at 20 Hz, the barometer at 100 Hz). */
static void attach_queued(ins_measurements_t *m, bool drain)
{
    nav_gnss_t gnss;
    nav_baro_t baro;
    nav_speed_t speed;

    if (queue_try_remove(&s_gnss_q, &gnss)) {
        attach_gnss(m, &gnss);
    }
    bool have_baro = queue_try_remove(&s_baro_q, &baro);
    while (drain && have_baro && queue_try_remove(&s_baro_q, &baro)) {
    }
    if (have_baro && s_p.use_baro) {
        /* The barometer has no delay compensation in INSLIB, so a sample
         * that waited in the queue is not fused as current. With the
         * barometer switched off, samples still queued from before the
         * switch are dropped. */
        if ((int64_t)m->timestamp - (int64_t)baro.t_us > APP_BARO_MAX_AGE_US) {
            s_work.n_baro_late++;
        } else {
            m->baro.pressure_pa = baro.pressure_pa;
            m->baro.is_valid = true;
            s_work.n_baro++;
        }
    }
    if (queue_try_remove(&s_speed_q, &speed)) {
        attach_speed(m, &speed);
    }
}

static void run_epoch(const ins_measurements_t *m)
{
    uint64_t t0 = time_us_64();
    nav_suite_update(&s_suite, m);
    uint32_t took = (uint32_t)(time_us_64() - t0);

    s_work.n_updates++;
    s_update_us_sum += took;
    s_update_count++;
    if (took > s_work.update_us_max) {
        s_work.update_us_max = took;
    }
}

/* R body to NED, column-major (INSLIB's layout), from ZYX roll/pitch/yaw. */
static void rpy_to_rotmat(float r, float p, float y, float R[9])
{
    float cr = cosf(r), sr = sinf(r), cp = cosf(p), sp = sinf(p), cy = cosf(y), sy = sinf(y);
    R[0] = cp * cy;
    R[1] = cp * sy;
    R[2] = -sp;
    R[3] = sr * sp * cy - cr * sy;
    R[4] = sr * sp * sy + cr * cy;
    R[5] = sr * cp;
    R[6] = cr * sp * cy + sr * sy;
    R[7] = cr * sp * sy - sr * cy;
    R[8] = cr * cp;
}

static void rot_b2n(const float R[9], const float v[3], float o[3])
{
    for (int i = 0; i < 3; i++) o[i] = R[i] * v[0] + R[i + 3] * v[1] + R[i + 6] * v[2];
}

/* Survey pole (pole_aid.h): detector, tip constraint, yaw fix and the
 * magnetometer gate, on this epoch's measurements. */
static void pole_step(ins_measurements_t *m, const nav_imu_t *imu)
{
    const bool ready = ins_is_ready(&s_suite.ins);
    float R[9], v[3], vt[3];
    bool have_v = false;
    if (ready && ins_get_rotmat_b_to_n(&s_suite.ins, R) && ins_get_velocity_ned(&s_suite.ins, v)) {
        const float *l = s_p.tip_leverarm_m;
        float wl[3] = { imu->gyr[1] * l[2] - imu->gyr[2] * l[1], imu->gyr[2] * l[0] - imu->gyr[0] * l[2],
                        imu->gyr[0] * l[1] - imu->gyr[1] * l[0] };
        rot_b2n(R, wl, vt);
        for (int i = 0; i < 3; i++) vt[i] += v[i];
        have_v = true;
        pole_aid_att(&s_pole, imu->t_us, R);
    }
    /* The rate as INSLIB uses it (ins_imu_calibrate: M (raw - fixed bias),
     * all-zero M = identity) less its bias estimate: pole_aid integrates it
     * over an episode for the pole-axis fit. */
    float wc[3], w0[3], wb[3];
    for (int i = 0; i < 3; i++) w0[i] = imu->gyr[i] - s_p.gyr_fixed_bias[i];
    bool gm_set = false;
    for (int i = 0; i < 9; i++) gm_set = gm_set || s_p.gyr_misalignment[i] != 0.0f;
    if (gm_set) {
        rot_b2n(s_p.gyr_misalignment, w0, wc);
    } else {
        memcpy(wc, w0, sizeof wc);
    }
    if (ready && ins_get_bias_gyr(&s_suite.ins, wb)) {
        for (int i = 0; i < 3; i++) wc[i] -= wb[i];
    }
    pole_aid_imu(&s_pole, imu->dt_s, imu->acc, wc, have_v ? vt : NULL);
    pole_episode_t ep;
    if (pole_aid_take_episode(&s_pole, &ep)) {
        queue_try_add(&s_pole_q, &ep);
    }

    if (have_v) {
        float pos[3], tip[3];
        if (pole_aid_want_latch(&s_pole) && ins_get_position_local(&s_suite.ins, pos)) {
            rot_b2n(R, s_p.tip_leverarm_m, tip);
            for (int i = 0; i < 3; i++) tip[i] += pos[i];
            pole_aid_latch(&s_pole, tip);
        }
        if (pole_aid_feed_due(&s_pole, tip)) {
            /* The pole-tip constraint at position level: the tip stays
             * where it was latched (INSLIB local NED, lever arm IMU to tip). */
            memcpy(m->local_pos.pos_ned, tip, sizeof m->local_pos.pos_ned);
            float var = APP_POLE_TIP_SD_M * APP_POLE_TIP_SD_M;
            memset(m->local_pos.Qll_ned, 0, sizeof m->local_pos.Qll_ned);
            m->local_pos.Qll_ned[0] = var;
            m->local_pos.Qll_ned[4] = var;
            m->local_pos.Qll_ned[8] = var;
            m->local_pos.is_valid = true;
            memcpy(m->local_pos_leverarm_b, s_p.tip_leverarm_m, sizeof m->local_pos_leverarm_b);
        }
        float dpsi, sd, r, p, y;
        if (pole_aid_take_yaw(&s_pole, &dpsi, &sd) && ins_get_rpy(&s_suite.ins, &r, &p, &y)) {
            float yaw = y + dpsi;
            if (yaw > 3.14159265f) yaw -= 6.2831853f;
            if (yaw < -3.14159265f) yaw += 6.2831853f;
            m->yaw.yaw_rad = yaw;
            m->yaw.stddev_rad = sd;
            m->yaw.is_valid = true;
            s_work.n_yaw++;
        }
    }

    if (m->mag.is_valid) {
        /* The calibrated field as INSLIB will see it (ins_imu_calibrate:
         * M (raw - bias), all-zero M = identity), less its hard-iron estimate. */
        float c[3], b[3], mb[3];
        for (int i = 0; i < 3; i++) c[i] = imu->mag[i] - s_p.mag_fixed_bias[i];
        bool m_set = false;
        for (int i = 0; i < 9; i++) m_set = m_set || s_p.mag_misalignment[i] != 0.0f;
        if (m_set) {
            rot_b2n(s_p.mag_misalignment, c, b);
        } else {
            memcpy(b, c, sizeof b);
        }
        if (ready && s_suite.ins.opt.estimate_mag_bias && ins_get_bias_mag(&s_suite.ins, mb)) {
            for (int i = 0; i < 3; i++) b[i] -= mb[i];
        }
        float Ra[9], r, p, y, sr, sp, sy = 10.0f;
        bool yaw_abs = false, att = false;
        if (have_v) {
            memcpy(Ra, R, sizeof Ra);
            yaw_abs = true;
            att = true;
            if (!ins_get_rpy_stddev(&s_suite.ins, &sr, &sp, &sy)) sy = 10.0f;
        } else if (nav_suite_get_rpy_ars(&s_suite, &r, &p, &y)) {
            rpy_to_rotmat(r, p, y, Ra);
            att = true;
        }
        static const float k_none[3] = { 0.0f, 0.0f, 0.0f };
        bool trusted = att && pole_aid_mag(&s_pole, imu->t_us, b, Ra, yaw_abs, sy,
                                           s_mag_model_set ? s_suite.ins.magnetic_n : k_none);
        if (!trusted) {
            m->mag.is_valid = false;
            s_work.n_mag_withheld++;
        }
    }
}

static void core1_main(void)
{
    nav_imu_t imu;
    ins_measurements_t m;

    log_set_sink(log_sink);
    /* Lets core 0 pause this core while it writes the settings to flash. */
    flash_safe_execute_core_init();

    /* The antenna mode is a setting known at boot, so the filter starts at
     * once. It restarts if the receiver can't do that mode, or when the
     * settings change. */
    uint32_t params_seq = s_params_seq;
    critical_section_enter_blocking(&s_out_lock);
    s_p = s_params_new;
    critical_section_exit(&s_out_lock);
    nav_antenna_t mode = s_mode;
    filter_init(mode);
    uint64_t next_publish = time_us_64() + PUBLISH_PERIOD_US;
    cpu_load_init(&s_load, s_clk_hz);
    bool worked = false; /* this pass did work (cpu_load.h) */

    for (;;) {
        cpu_load_pass(&s_load, worked);
        worked = false;
        nav_antenna_t new_mode = s_mode;
        uint32_t new_seq = s_params_seq;
        if (new_mode != mode || new_seq != params_seq) {
            bool restart = new_mode != mode;
            if (new_seq != params_seq) {
                params_seq = new_seq;
                nav_params_t old = s_p;
                critical_section_enter_blocking(&s_out_lock);
                s_p = s_params_new;
                critical_section_exit(&s_out_lock);
                /* The coasting window alone: INSLIB reads it from its options
                 * at every check (ins_max_dr_sec), so the running filter takes
                 * it without a restart. */
                old.max_deadreckoning_s = s_p.max_deadreckoning_s;
                /* The position decimation too: INSLIB reads it at every fix,
                 * and its phase counter wraps at the new value. */
                old.gnss_pos_decim = s_p.gnss_pos_decim;
                /* So is the pole aiding (the antenna height): pole_aid takes
                 * the new tip without a restart. */
                old.pole_aid = s_p.pole_aid;
                memcpy(old.tip_leverarm_m, s_p.tip_leverarm_m, sizeof old.tip_leverarm_m);
                if (memcmp(&old, &s_p, sizeof old) != 0) {
                    restart = true;
                } else {
                    s_suite.ins.opt.max_deadreckoning_sec = s_p.max_deadreckoning_s;
                    /* 0 = INSLIB's default, which ins_init resolves only
                     * at a start: resolve it here the same way. */
                    s_suite.ins.opt.gnss_pos_decimation = s_p.gnss_pos_decim > 0 ? s_p.gnss_pos_decim : 1;
                    pole_aid_set_geometry(&s_pole, s_p.pole_aid, s_p.tip_leverarm_m, s_p.leverarm_m);
                }
            }
            mode = new_mode;
            if (restart) filter_init(mode);
            worked = true;
        }

        if (queue_try_remove(&s_imu_q, &imu)) {
            settle_check(imu.t_us);
            memset(&m, 0, sizeof m);
            m.timestamp = (ins_time_us_t)imu.t_us;
            m.strapdown_dt_sec = imu.dt_s;
            memcpy(m.acc.data, imu.acc, sizeof m.acc.data);
            memcpy(m.gyr.data, imu.gyr, sizeof m.gyr.data);
            for (int i = 0; i < 3; i++) {
                m.acc.Qll_diag[i] = s_p.acc_psd;
                m.gyr.Qll_diag[i] = s_p.gyr_psd;
            }
            m.acc.is_valid = true;
            m.gyr.is_valid = true;
            if (s_p.use_mag && imu.mag_new) {
                memcpy(m.mag.data, imu.mag, sizeof m.mag.data);
                m.mag.is_valid = true;
            }
            if (s_p.pole_aid) {
                pole_step(&m, &imu);
            }
            if (m.mag.is_valid) {
                s_work.n_mag++;
            }
            attach_queued(&m, false);
            run_epoch(&m);
            s_last_imu_us = imu.t_us;
            s_next_stall_epoch_us = 0;
            worked = true;
        } else if (s_last_imu_us != 0) {
            /* IMU silent: step the filter without it, so INSLIB notices the
             * loss and stops (mode NONE) instead of its last solution being
             * published as current. Queued samples ride along; INSLIB keeps
             * nothing of them while it has no IMU. */
            uint64_t t = time_us_64();
            if (t - s_last_imu_us >= IMU_STALL_US && t >= s_next_stall_epoch_us) {
                memset(&m, 0, sizeof m);
                m.timestamp = (ins_time_us_t)t;
                attach_queued(&m, true);
                run_epoch(&m);
                s_work.n_stall_epochs++;
                s_next_stall_epoch_us = t + STALL_EPOCH_US;
                worked = true;
            }
        }

        uint64_t now = time_us_64();
        if (now >= next_publish) {
            publish(now);
            worked = true;
            next_publish += PUBLISH_PERIOD_US;
            if (next_publish < now) {
                next_publish = now + PUBLISH_PERIOD_US;
            }
        }
    }
}

/* ---- Core 0 ------------------------------------------------------------ */

void nav_set_params(const nav_params_t *params)
{
    critical_section_enter_blocking(&s_out_lock);
    s_params_new = *params;
    critical_section_exit(&s_out_lock);
    s_params_seq++;
}

void nav_start(nav_antenna_t mode, const nav_params_t *params)
{
    s_mode = mode;
    s_params_new = *params;
    queue_init(&s_imu_q, sizeof(nav_imu_t), IMU_QUEUE_LEN);
    queue_init(&s_gnss_q, sizeof(nav_gnss_t), GNSS_QUEUE_LEN);
    queue_init(&s_baro_q, sizeof(nav_baro_t), BARO_QUEUE_LEN);
    queue_init(&s_speed_q, sizeof(nav_speed_t), SPEED_QUEUE_LEN);
    queue_init(&s_log_q, LOG_MSG_LEN, LOG_QUEUE_LEN);
    queue_init(&s_pole_q, sizeof(pole_episode_t), POLE_QUEUE_LEN);
    critical_section_init(&s_out_lock);
    s_clk_hz = clock_get_hz(clk_sys);

    for (size_t i = 0; i < sizeof s_stack / sizeof s_stack[0]; i++) {
        s_stack[i] = STACK_FILL;
    }
    multicore_launch_core1_with_stack(core1_main, s_stack, sizeof s_stack);
}

static bool push(queue_t *q, const void *item, uint32_t *drops)
{
    if (queue_try_add(q, item)) {
        return true;
    }
    (*drops)++;
    return false;
}

/* A dropped sample's dt is added to the next one, so the filter integrates
 * over the whole interval and not only over the last sample period. */
bool nav_push_imu(const nav_imu_t *s)
{
    static float s_lost_dt_s; /* core 0 only */
    nav_imu_t n = *s;

    n.dt_s += s_lost_dt_s;
    if (push(&s_imu_q, &n, &s_drops.imu)) {
        s_lost_dt_s = 0.0f;
        return true;
    }
    s_lost_dt_s = n.dt_s;
    return false;
}

bool nav_push_gnss(const nav_gnss_t *s) { return push(&s_gnss_q, s, &s_drops.gnss); }
bool nav_push_baro(const nav_baro_t *s) { return push(&s_baro_q, s, &s_drops.baro); }
bool nav_push_speed(const nav_speed_t *s) { return push(&s_speed_q, s, &s_drops.speed); }

void nav_set_antenna_mode(nav_antenna_t mode) { s_mode = mode; }

uint32_t nav_output_seq(void)
{
    return s_out_seq;
}

void nav_get_output(nav_output_t *out)
{
    critical_section_enter_blocking(&s_out_lock);
    *out = s_out;
    critical_section_exit(&s_out_lock);
}

void nav_queue_drops(nav_drops_t *out) { *out = s_drops; }

void nav_queue_drops_settle(void)
{
    static uint64_t s_done_for;
    nav_output_t o;
    nav_get_output(&o);
    if (o.t_settled_us != 0 && o.t_settled_us != s_done_for) {
        memset(&s_drops, 0, sizeof s_drops);
        s_done_for = o.t_settled_us;
    }
}

bool nav_pop_pole_episode(pole_episode_t *out) { return queue_try_remove(&s_pole_q, out); }

bool nav_pop_log(char *buf, size_t len)
{
    char msg[LOG_MSG_LEN];
    if (!queue_try_remove(&s_log_q, msg)) {
        return false;
    }
    msg[LOG_MSG_LEN - 1] = '\0';
    snprintf(buf, len, "%s", msg);
    return true;
}

uint32_t nav_stack_used(void)
{
    /* The stack grows down from the end of the array. */
    size_t n = sizeof s_stack / sizeof s_stack[0];
    size_t untouched = 0;
    while (untouched < n && s_stack[untouched] == STACK_FILL) {
        untouched++;
    }
    return (uint32_t)((n - untouched) * sizeof(uint32_t));
}

uint32_t nav_stack_size(void) { return (uint32_t)sizeof s_stack; }

const char *nav_mode_name(int mode)
{
    switch (mode) {
    case NAV_SUITE_MODE_NONE:          return "NONE";
    case NAV_SUITE_MODE_ATTITUDE_ONLY: return "ATTITUDE_ONLY";
    case NAV_SUITE_MODE_COASTING:      return "COASTING";
    case NAV_SUITE_MODE_FULL:          return "FULL";
    default:                           return "?";
    }
}
