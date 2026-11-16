/*
 * OpenGINS demo: sensor bring-up and INSLIB fused output.
 *
 * Core 0 (this file): UART DMA, the TM210 and UM980 drivers with their boot
 * provisioning, the BMP581, and a status report on USB once a second.
 * Core 1 (nav_core1.c): the INSLIB nav_suite filter.
 *
 * Wi-Fi and the web server (app/web/) run when built with WEB_MODE=always:
 * station or open AP, status page with /status.json, config page with
 * /config.json and POST /config (plans/web-config-server.md, Phases 2-4).
 * Settings come from flash or the build defaults (settings.h).
 *
 * No Bluetooth in this build. Pins are in board_pins.h, other settings in
 * app_config.h.
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "hardware/clocks.h"
#include "pico/stdlib.h"

#include "app_config.h"
#include "board_pins.h"
#include "bmp581.h"
#include "die_temp.h"
#include "field_io.h"
#include "geoid.h"
#include "ground_point.h"
#include "imu_calib.h"
#include "nav_core1.h"
#include "nav_suite.h"
#include "paogi.h"
#include "pole_mount.h"
#include "settings.h"
#include "tm210.h"
#include "uart_dma.h"
#include "um980.h"
#include "usb_log.h"
#if APP_WEB_BUILT
#include "hardware/watchdog.h"
#include "web/config_web.h"
#include "web/jsonw.h"
#include "web/web.h"
#endif

#define IMU_RX_BITS         13 /* 8 KB, about 200 ms at 39 kB/s */
#define GNSS_RX_BITS        12 /* 4 KB */
#define USB_WAIT_MS         3000
#define STATUS_PERIOD_US    1000000u
#define BARO_RETRY_US       5000000u
#define DEG                 (180.0 / M_PI)

static uint8_t s_imu_rx[1u << IMU_RX_BITS] __attribute__((aligned(1u << IMU_RX_BITS)));
static uint8_t s_gnss_rx[1u << GNSS_RX_BITS] __attribute__((aligned(1u << GNSS_RX_BITS)));

static uart_dma_t s_imu_port, s_gnss_port;
static tm210_t s_imu;
static um980_t s_gnss;
static bmp581_t s_baro;
static bool s_baro_ok;

static tm210_sample_t s_last_imu;
static um980_nav_t s_last_nav;
static bmp581_sample_t s_last_baro;
static bool s_have_nav;

/* Geoid undulation (ellipsoid minus geoid height) from the receiver's last
 * usable fix: the fallback outside the geoid grid (geoid.h). It changes by
 * centimetres over kilometres, so an old value is still good. */
static float s_undulation_m;
static bool s_have_undulation;

/* Geoid height N at a point: the grid in flash (SWEN17, so h_ell - N is
 * RH 2000), else the receiver's own model. False before any usable fix
 * outside the grid. */
static bool undulation_at(double lat_deg, double lon_deg, float *n_m, const char **model)
{
    if (geoid_height_m(&geoid_grid, lat_deg, lon_deg, n_m)) {
        *model = geoid_grid.name;
        return true;
    }
    if (s_have_undulation) {
        *n_m = s_undulation_m;
        *model = "receiver";
        return true;
    }
    return false;
}

/* Core 0 main-loop load (cpu_load.h); core 1's comes in nav_output_t. */
static cpu_load_t s_load0;
static cpu_load_result_t s_load0_res;
static float s_die_c; /* RP2350 die temperature, read with the status report */

/* IMU noise at rest, the statistic behind INSLIB's stillness variance gate
 * (ahrs_static_variance_update / REQ-NAV-013): the per-axis sample stddev
 * over a 0.2 s window, RMS over the three axes, for gyro and accel. Each
 * status period reports the min / mean / max over its windows and how many
 * windows were under the limits core 1 gives INSLIB. Diagnostic only. */
#define NOISE_WINDOW_US     200000u
#define NOISE_MIN_SAMPLES   8u
#define NOISE_GATE_GYR_DPS  APP_STILL_GYR_STDDEV_DPS
#define NOISE_GATE_ACC      APP_STILL_ACC_STDDEV_MPS2
typedef struct {
    float min, max, sum;
} noise_acc_t;
typedef struct {
    uint32_t n;            /* samples in the current window */
    uint64_t t0;
    float mean[6], m2[6];
    uint32_t windows, pass;
    noise_acc_t gyr_dps, acc;
} noise_t;
static noise_t s_noise;
typedef struct {
    uint32_t windows, pass;
    float gyr_dps[3], acc[3]; /* min, mean, max */
} noise_report_t;
static noise_report_t s_noise_rep;

static void noise_acc_add(noise_acc_t *a, float x, bool first)
{
    if (first || x < a->min) a->min = x;
    if (first || x > a->max) a->max = x;
    a->sum = first ? x : a->sum + x;
}

static void noise_push(const tm210_sample_t *s)
{
    noise_t *z = &s_noise;
    const float x[6] = { s->gyr[0], s->gyr[1], s->gyr[2], s->acc[0], s->acc[1], s->acc[2] };
    if (s->gap) {
        z->n = 0; /* a lost sample would hide in the window */
    }
    if (z->n == 0) {
        for (int i = 0; i < 6; i++) {
            z->mean[i] = x[i];
            z->m2[i] = 0.0f;
        }
        z->t0 = s->t_us;
        z->n = 1;
        return;
    }
    z->n++;
    for (int i = 0; i < 6; i++) {
        float d = x[i] - z->mean[i];
        z->mean[i] += d / (float)z->n;
        z->m2[i] += d * (x[i] - z->mean[i]);
    }
    if (z->n < NOISE_MIN_SAMPLES || s->t_us - z->t0 < NOISE_WINDOW_US) {
        return;
    }
    float inv = 1.0f / (3.0f * (float)(z->n - 1u));
    float g = sqrtf((z->m2[0] + z->m2[1] + z->m2[2]) * inv) * (float)DEG;
    float a = sqrtf((z->m2[3] + z->m2[4] + z->m2[5]) * inv);
    bool first = z->windows == 0;
    noise_acc_add(&z->gyr_dps, g, first);
    noise_acc_add(&z->acc, a, first);
    z->windows++;
    if (g <= NOISE_GATE_GYR_DPS && a <= NOISE_GATE_ACC) {
        z->pass++;
    }
    z->n = 0;
}

/* Close the status period: keep its figures for the report, start anew. */
static void noise_roll(void)
{
    noise_t *z = &s_noise;
    noise_report_t *r = &s_noise_rep;
    r->windows = z->windows;
    r->pass = z->pass;
    if (z->windows > 0) {
        r->gyr_dps[0] = z->gyr_dps.min;
        r->gyr_dps[1] = z->gyr_dps.sum / (float)z->windows;
        r->gyr_dps[2] = z->gyr_dps.max;
        r->acc[0] = z->acc.min;
        r->acc[1] = z->acc.sum / (float)z->windows;
        r->acc[2] = z->acc.max;
    }
    z->windows = 0;
    z->pass = 0;
}

/* Magnetometer samples core 1 handed to INSLIB in the last status period.
 * INSLIB fuses only some of them (see mag_fusion_age_ms). */
static uint32_t s_mag_hz;

static void mag_roll(void)
{
    static uint32_t s_prev;
    nav_output_t o;
    nav_get_output(&o);
    s_mag_hz = o.n_mag - s_prev;
    s_prev = o.n_mag;
}

static const char *att_source_name(int s)
{
    switch (s) {
    case NAV_ATT_INS:  return "INS";
    case NAV_ATT_AHRS: return "AHRS (magnetometer)";
    case NAV_ATT_ARS:  return "ARS (gyro, relative yaw)";
    default:           return "none";
    }
}

/* Which north the AHRS yaw points at. */
static const char *mag_north_name(const nav_output_t *o)
{
    if (!o->mag_declination) return "magnetic (no position, no declination)";
    if (o->mag_on_magnetic_north) return "magnetic (declination waiting to re-frame)";
    return "true";
}

/* Why the pole's magnetometer trust gate withholds the magnetometer, in
 * words for the web pages. Empty when trusted. Core 0 only (snprintf). */
static void mag_reason(const pole_aid_status_t *p, char *out, size_t n)
{
    size_t k = 0;
    out[0] = '\0';
    if (p->mag_trusted) return;
    if (p->mag_blocks == 0) {
        snprintf(out, n, "waiting for a field model (GNSS position)");
        return;
    }
#define WHY(...) do { if (k) k += (size_t)snprintf(out + k, n - k, "; "); \
                      if (k < n) k += (size_t)snprintf(out + k, n - k, __VA_ARGS__); } while (0)
    if (p->mag_norm_err_pct > POLE_MAG_NORM_TOL * 100.0f) {
        WHY("field strength off %.0f%% (max %.0f%%)", (double)p->mag_norm_err_pct,
            (double)(POLE_MAG_NORM_TOL * 100.0f));
    }
    if (p->mag_dip_err_deg > POLE_MAG_DIP_TOL_DEG) {
        WHY("dip error %.1f deg (max %.0f deg)", (double)p->mag_dip_err_deg, (double)POLE_MAG_DIP_TOL_DEG);
    }
    if (p->mag_tilt_range_deg < POLE_MAG_TILT_RANGE_DEG && p->mag_yaw_range_deg < POLE_MAG_YAW_RANGE_DEG) {
        WHY("pole not tilted or turned enough (tilt %.1f deg, yaw %.0f deg; needs %.0f or %.0f deg)",
            (double)p->mag_tilt_range_deg, (double)p->mag_yaw_range_deg, (double)POLE_MAG_TILT_RANGE_DEG,
            (double)POLE_MAG_YAW_RANGE_DEG);
    }
    if (!isnan(p->mag_res_mean_deg) && (fabsf(p->mag_res_mean_deg) > POLE_MAG_RES_MEAN_MAX_DEG ||
                                        p->mag_res_sd_deg > POLE_MAG_RES_SD_MAX_DEG)) {
        WHY("heading vs INS off by %.1f +- %.1f deg", (double)p->mag_res_mean_deg, (double)p->mag_res_sd_deg);
    }
    if (k == 0) snprintf(out, n, "collecting samples");
#undef WHY
}

static float mag_field_ut(void)
{
    const float *m = s_last_imu.mag;
    return sqrtf(m[0] * m[0] + m[1] * m[1] + m[2] * m[2]);
}

/* Dual-antenna heading: baseline-length gate (plans/gnss-um98x-config.md,
 * "VARIABLELENGTH and the mount"). The median is over the last fixed epochs. */
#define BASELINE_HIST       31
#define BASELINE_MIN_HIST   5
static float s_base_hist[BASELINE_HIST];
static unsigned s_base_n, s_base_pos;
static um980_heading_t s_last_hdg;
static bool s_have_hdg;
static float s_base_median_m;
static uint32_t s_hdg_used, s_hdg_gated;

/* Settings in use (settings.h): loaded at boot, live fields replaced by a
 * save from the web page. s_boot_settings keeps what this boot started with,
 * for the fields that only apply after a reboot. */
static settings_t s_settings, s_boot_settings;
static bool s_settings_from_flash;

#if APP_WEB_BUILT
/* Wi-Fi and the web server run this boot: always in WEB_MODE=always, in
 * WEB_MODE=switch only when GPIO22 was pulled to GND at boot. */
static bool s_web_on;
#endif

static void nav_params_from_settings(nav_params_t *p, const settings_t *s)
{
    /* Zeroed first: apply_live() compares two of these with memcmp, and the
     * bools leave padding. */
    memset(p, 0, sizeof *p);
    p->use_mag = s->sensor_mag != 0;
    p->use_baro = s->sensor_baro != 0;
    for (int i = 0; i < 3; i++) {
        p->leverarm_m[i] = s->leverarm_m[i];
    }
    p->acc_psd = s->acc_psd;
    p->gyr_psd = s->gyr_psd;
    p->gyr_bias_rw = s->gyr_bias_rw;
    p->acc_bias_rw = s->acc_bias_rw;
    p->automotive_min_speed = s->automotive_min_speed;
    p->max_deadreckoning_s = s->max_deadreckoning_s;
    /* The platform in effect since boot, not the saved one: it changes at a
     * reboot, so a save never restarts the filter for it. */
    unsigned plat = settings_platform(&s_boot_settings);
    p->course_yaw = plat == APP_PLATFORM_TRACTOR || plat == APP_PLATFORM_FIXED_WING;
    /* Pole aiding: the tip is the antenna height below the antenna, along
     * the pole axis (body down, or the learned axis, pole_mount.h). */
    float h_ant = settings_ant_height(s, plat);
    p->pole_aid = plat == APP_PLATFORM_POLE && h_ant > 0.0f;
    float u[3];
    pole_mount_axis(u);
    for (int i = 0; i < 3; i++) p->tip_leverarm_m[i] = s->leverarm_m[i] + h_ant * u[i];
    imu_calib_gyr_fixed_bias(p->gyr_fixed_bias);
    imu_calib_acc_calibration(p->acc_misalignment, p->acc_fixed_bias);
    imu_calib_mag_calibration(p->mag_misalignment, p->mag_fixed_bias);
    imu_calib_gyr_misalignment(p->gyr_misalignment);
}

static bool baro_start(void)
{
    bmp581_config_t cfg;
    bmp581_default_config(&cfg);
    cfg.i2c = PIN_BMP581_I2C;
    cfg.sda_pin = PIN_BMP581_SDA;
    cfg.scl_pin = PIN_BMP581_SCL;
    cfg.baud_hz = BMP581_I2C_HZ;
    cfg.int_pin = PIN_BMP581_INT;

    int rc = bmp581_init(&s_baro, &cfg);
    if (rc != BMP581_OK) {
        printf("[bmp581] init failed: %s (%d), retrying\n", bmp581_strerror(rc), rc);
        return false;
    }
    printf("[bmp581] running at 0x%02x\n", s_baro.cfg.addr);
    return true;
}

static void forward_gnss(const um980_nav_t *n)
{
    nav_gnss_t g = { 0 };
    double track = n->track_deg / DEG;

    g.t_valid_us = n->t_valid_us;
    g.pos_valid = n->pos_usable;
    g.vel_valid = n->vel_usable;
    g.lat_rad = n->lat_deg / DEG;
    g.lon_rad = n->lon_deg / DEG;
    g.h_ell_m = n->hgt_msl_m + (double)n->undulation_m; /* INSLIB wants ellipsoid height */
    g.pos_sd_ned[0] = n->sd_lat_m;
    g.pos_sd_ned[1] = n->sd_lon_m;
    g.pos_sd_ned[2] = n->sd_hgt_m;
    g.vel_ned[0] = (float)(n->hor_speed_mps * cos(track));
    g.vel_ned[1] = (float)(n->hor_speed_mps * sin(track));
    g.vel_ned[2] = (float)-n->vert_speed_mps;
    g.vel_sd_ned[0] = n->sd_hor_speed_mps;
    g.vel_sd_ned[1] = n->sd_hor_speed_mps;
    g.vel_sd_ned[2] = n->sd_vert_speed_mps;
    g.week = n->week;
    g.tow_ms = n->tow_ms;

    if (g.pos_valid || g.vel_valid) {
        nav_push_gnss(&g);
    }
    if (g.vel_valid && n->hor_speed_mps >= APP_CALIB_MAG_REF_MIN_SPEED_MPS) {
        /* The course over ground for the gyro turn fit (imu_calib decides by
         * the platform whether it is a heading). */
        float csd = (float)(n->sd_hor_speed_mps / n->hor_speed_mps);
        imu_calib_gnss_heading(n->t_valid_us, (float)remainder(track, 2.0 * M_PI), csd > 0.005f ? csd : 0.005f,
                               true);
    }
    if (g.pos_valid) {
        /* GPS week 0 began on 1980-01-06. */
        float year = 1980.0f + (5.0f + (float)n->week * 7.0f + (float)n->tow_ms / 86400000.0f) / 365.25f;
        imu_calib_set_position(g.lat_rad, g.lon_rad, (float)g.h_ell_m, year);
    }
}

static float median(const float *v, unsigned n)
{
    float s[BASELINE_HIST];
    for (unsigned i = 0; i < n; i++) {
        s[i] = v[i];
    }
    for (unsigned i = 1; i < n; i++) { /* insertion sort, n <= 31 at 20 Hz */
        float x = s[i];
        unsigned j = i;
        while (j > 0 && s[j - 1] > x) {
            s[j] = s[j - 1];
            j--;
        }
        s[j] = x;
    }
    return s[n / 2];
}

/* A fixed heading whose baseline length agrees with the running median goes
 * to core 1 as the body yaw. The first few fixed epochs only fill the
 * median. */
static void forward_heading(const um980_heading_t *h)
{
    if (!h->fixed) {
        return;
    }
    bool gate_ok = false;
    if (s_base_n >= BASELINE_MIN_HIST) {
        s_base_median_m = median(s_base_hist, s_base_n);
        gate_ok = fabsf(h->length_m - s_base_median_m) <= s_settings.baseline_gate_m;
    }
    s_base_hist[s_base_pos] = h->length_m;
    s_base_pos = (s_base_pos + 1u) % BASELINE_HIST;
    if (s_base_n < BASELINE_HIST) {
        s_base_n++;
    }
    if (!gate_ok) {
        if (s_base_n > BASELINE_MIN_HIST) {
            s_hdg_gated++;
        }
        return;
    }

    /* Heading is clockwise from true north, as NED yaw. */
    double yaw = (double)(h->heading_deg - s_settings.baseline_yaw_deg) / DEG;
    yaw = remainder(yaw, 2.0 * M_PI);
    float sd = h->sd_heading_deg;
    float floor_deg = APP_HEADING_SD_FLOOR_DEG_1M / h->length_m;
    if (sd < floor_deg) {
        sd = floor_deg;
    }

    nav_gnss_t g = { 0 };
    g.t_valid_us = h->t_valid_us;
    g.yaw_valid = true;
    g.yaw_rad = (float)yaw;
    g.yaw_sd_rad = (float)(sd / DEG);
    g.week = h->week;
    g.tow_ms = h->tow_ms;
    if (nav_push_gnss(&g)) {
        s_hdg_used++;
    }
    imu_calib_gnss_heading(h->t_valid_us, (float)yaw, (float)(sd / DEG), false);
}

static const char *fix_name(uint32_t pos_type)
{
    switch (pos_type) {
    case UM980_TYPE_NONE:           return "none";
    case UM980_TYPE_SINGLE:         return "single";
    case UM980_TYPE_PSRDIFF:        return "dgps";
    case UM980_TYPE_SBAS:           return "sbas";
    case UM980_TYPE_L1_FLOAT:
    case UM980_TYPE_IONOFREE_FLOAT:
    case UM980_TYPE_NARROW_FLOAT:   return "rtk-float";
    case UM980_TYPE_L1_INT:
    case UM980_TYPE_WIDE_INT:
    case UM980_TYPE_NARROW_INT:     return "rtk-fixed";
    default:                        return pos_type >= UM980_TYPE_PPP_CONVERGING ? "ppp" : "other";
    }
}

static const char *model_name(void)
{
    switch (um980_model(&s_gnss)) {
    case 0:                    return NULL;
    case UM980_PRODUCT_UM980:  return "UM980";
    case UM980_PRODUCT_UM982:  return "UM982";
    default:                   return "other";
    }
}

static const char *calib_id_name(imu_calib_id_t s)
{
    switch (s) {
    case IMU_CALIB_ID_OK:      return "ok";
    case IMU_CALIB_ID_UNKNOWN: return "unknown (not learning)";
    default:                   return "waiting for the TM210";
    }
}

static void print_mag_cal(const char *lbl, const mcal_cal_t *m)
{
    printf("      %s  M %.3f %.3f %.3f / %.3f %.3f %.3f / %.3f %.3f %.3f  hard iron %.1f %.1f %.1f uT"
           "  (sd M %.3f, b %.1f)",
           lbl, (double)m->m[0], (double)m->m[1], (double)m->m[2], (double)m->m[3], (double)m->m[4],
           (double)m->m[5], (double)m->m[6], (double)m->m[7], (double)m->m[8], (double)m->b[0],
           (double)m->b[1], (double)m->b[2], (double)m->sd_m, (double)m->sd_b_ut);
}

/* Learned gyro scale / misalignment (imu_calib.h, Phase 4). */
static void print_calib_gyr_m(const imu_calib_status_t *c)
{
    printf("CALIB gyrM  %s  rotations %lu (this boot %lu, too small %lu, %s)  turns %lu (this boot %lu%s)\n",
           c->gyr_m_applied ? "applied" : "identity", (unsigned long)c->n_rot, (unsigned long)c->rot_boot,
           (unsigned long)c->rot_small, c->rot_state ? c->rot_state : "", (unsigned long)c->n_turn,
           (unsigned long)c->turn_boot, c->turn_active ? ", in a turn segment" : "");
    if (c->n_rot + c->n_turn > 0) {
        const float *d = c->gyr_learned.dm, *e = c->gyr_learned.sd;
        printf("      learned dM  scale %.5f %.5f %.5f (sd %.5f %.5f %.5f)  mis xy %.3f yx %.3f deg"
               "  -> next boot changes %d of 9%s%s\n",
               (double)d[0], (double)d[4], (double)d[8], (double)e[0], (double)e[4], (double)e[8],
               (double)d[1] * DEG, (double)d[3] * DEG, c->gyr_taken, c->gyr_next_bad ? "  REJECTED: " : "",
               c->gyr_next_bad ? c->gyr_next_bad : "");
    }
}

/* Learned magnetometer calibration (imu_calib.h, Phase 3). */
static void print_calib_mag(const imu_calib_status_t *c)
{
    printf("CALIB mag   %s  %s\n", c->mag_applied ? "applied" : "not applied",
           c->mag_state ? c->mag_state : "");
    printf("      samples %lu/54 (this boot %lu, outliers %lu)  coverage %.3f"
           "  reference %lu (this boot %lu, yaw from %s)\n",
           (unsigned long)c->mag_bins, (unsigned long)c->mag_groups_boot,
           (unsigned long)c->mag_outliers, (double)c->mag_coverage, (unsigned long)c->mag_n_ref,
           (unsigned long)c->mag_ref_boot, c->mag_ref_source ? c->mag_ref_source : "no GNSS heading");
    if (c->mag_applied) {
        print_mag_cal("in use ", &c->mag_use);
        printf("\n");
    }
    if (c->mag_solved) {
        print_mag_cal("learned", &c->mag_learned);
        printf("  F %.1f uT  rms %.2f -> %.2f uT, dip rms %.2f (%d)%s%s\n", (double)c->mag_ell.field_ut,
               (double)c->mag_ell.rms_before, (double)c->mag_ell.rms_after,
               (double)c->mag_ell.rms_dip_ut, c->mag_ell.n_down,
               c->mag_learned_bad ? "  REJECTED: " : "", c->mag_learned_bad ? c->mag_learned_bad : "");
    }
}

/* Learned pole axis (pole_mount.h); nothing off the pole. */
static void print_calib_pole(void)
{
    pole_mount_status_t p;
    pole_mount_status(&p);
    if (!p.active) return;
    printf("CALIB pole  %s, %s  record %lu  episodes %lu (this boot %lu, dropped %lu)  twist %.0f deg\n",
           p.applied ? "learned axis" : "body down axis", p.hold ? "hold" : "auto", (unsigned long)p.rec_seq,
           (unsigned long)p.stats.n_ep, (unsigned long)p.ep_boot, (unsigned long)p.ep_rejected,
           (double)p.stats.twist_deg);
    if (p.applied) {
        printf("      in use   tilt %.2f deg toward %.0f deg\n", (double)p.use_tilt_deg, (double)p.use_azim_deg);
    }
    if (p.stats.n_ep > 0 && p.learned.ok) {
        const pm_result_t *r = &p.learned;
        printf("      learned  tilt %.2f deg (sd %.2f) toward %.0f deg  length %.3f +- %.3f m vs set %.3f m%s"
               "  rms %.3f m  next boot: %s%s%s\n",
               (double)r->tilt_deg, (double)r->tilt_sd_deg, (double)r->azim_deg, (double)r->len_m,
               (double)r->len_sd_m, (double)p.ant_height_m, p.len_differs ? " (CHECK the antenna height)" : "",
               (double)r->rms_m, p.next_boot, p.accept_why ? "; not yet: " : "", p.accept_why ? p.accept_why : "");
    }
    if (p.last_rms_m >= 0.0f) {
        printf("      last episode %s: rms %.3f m  twist %.0f deg\n", p.last_added ? "added" : "dropped",
               (double)p.last_rms_m, (double)p.last_twist_deg);
    }
}

/* Learned gyro bias vs temperature (imu_calib.h). */
static void print_calib(void)
{
    imu_calib_status_t c;
    imu_calib_status(&c);
    printf("CALIB gyro  %s  record %lu  id %s  chunks %lu (this boot %lu, unsaved %lu, skipped "
           "att %lu temp %lu)%s\n",
           c.applied ? "applied" : "not applied", (unsigned long)c.rec_seq, calib_id_name(c.id_state),
           (unsigned long)c.stats.n_chunks, (unsigned long)c.chunks_boot, (unsigned long)c.unsaved,
           (unsigned long)c.skip_att, (unsigned long)c.skip_temp,
           c.still ? "  (still)" : "");
    if (c.applied) {
        printf("      in use   b0 %.4f %.4f %.4f deg/s  k %.5f %.5f %.5f deg/s/C\n",
               (double)c.use.b0[0] * DEG, (double)c.use.b0[1] * DEG, (double)c.use.b0[2] * DEG,
               (double)c.use.k[0] * DEG, (double)c.use.k[1] * DEG, (double)c.use.k[2] * DEG);
    }
    if (c.stats.n_chunks > 0) {
        printf("      learned  b0 %.4f %.4f %.4f deg/s (sd %.4f)  k %.5f %.5f %.5f deg/s/C (sd %.5f)%s"
               "  T %.1f..%.1f C  %.0f s still%s%s\n",
               (double)c.learned.b0[0] * DEG, (double)c.learned.b0[1] * DEG,
               (double)c.learned.b0[2] * DEG, (double)c.learned.sd_b0 * DEG,
               (double)c.learned.k[0] * DEG, (double)c.learned.k[1] * DEG,
               (double)c.learned.k[2] * DEG, (double)c.learned.sd_k * DEG,
               c.learned.k_fitted ? "" : " (not fitted: span too small)",
               (double)c.stats.t_min_c, (double)c.stats.t_max_c, (double)c.stats.still_s,
               c.learned_bad ? "  REJECTED: " : "", c.learned_bad ? c.learned_bad : "");
    }
    printf("CALIB acc   %s  poses %lu/30 (this boot %lu, kept old %lu)  coverage %.3f of 0.333"
           "  not-rest windows %lu\n",
           c.acc_applied ? "applied" : "not applied", (unsigned long)c.acc_poses,
           (unsigned long)c.poses_boot, (unsigned long)c.poses_kept_old, (double)c.acc_coverage,
           (unsigned long)c.windows_not_rest);
    print_calib_mag(&c);
    print_calib_gyr_m(&c);
    print_calib_pole();
    const acal_result_t *ar[2] = { c.acc_applied ? &c.acc_use : NULL,
                                   c.acc_poses > 0 ? &c.acc_learned : NULL };
    const char *lbl[2] = { "in use ", "learned" };
    for (int k = 0; k < 2; k++) {
        if (!ar[k]) {
            continue;
        }
        const float *q = ar[k]->p;
        const float *e = ar[k]->sd;
        printf("      %s  scale %.4f %.4f %.4f (sd z %.4f)  mis %.3f %.3f %.3f deg (sd xy %.3f)"
               "  bias %.4f %.4f %.4f m/s2 (sd z %.4f)  rms %.4f -> %.4f%s%s\n",
               lbl[k], (double)q[ACAL_SX], (double)q[ACAL_SY], (double)q[ACAL_SZ],
               (double)e[ACAL_SZ], (double)q[ACAL_MXY] * DEG, (double)q[ACAL_MXZ] * DEG,
               (double)q[ACAL_MYZ] * DEG, (double)e[ACAL_MXY] * DEG, (double)q[ACAL_BX],
               (double)q[ACAL_BY], (double)q[ACAL_BZ], (double)e[ACAL_BZ],
               (double)ar[k]->rms_before, (double)ar[k]->rms_after,
               k == 1 && c.acc_learned_bad ? "  REJECTED: " : "",
               k == 1 && c.acc_learned_bad ? c.acc_learned_bad : "");
    }
}

/* The position the outputs use (the status report and /status.json now;
 * SW Maps and $PAOGI once they exist): the filter's while it runs on a live
 * IMU, else the receiver's own fix. INSLIB has no GNSS-only mode: after an
 * IMU loss it stops (mode NONE) until the IMU is back. */
#define POS_IMU_FRESH_MS   200   /* INSLIB's IMU-loss timeout */
#define POS_OUT_FRESH_US   1000000u
#define POS_GNSS_FRESH_US  2000000u
typedef struct {
    const char *source;    /* "ins", "gnss" or "none" */
    const char *fix;       /* the receiver's fix type when source is gnss */
    double      lat_deg, lon_deg, h_ell_m;
    /* Tractor / pole with ant_height_m above 0: lat/lon/h above are the
     * ground point below the antenna. The antenna itself is in ant_*
     * whenever source is not none, with or without a ground point. */
    bool        ground;
    float       ant_height_m;
    bool        tilted;    /* the ground point used the attitude (gate passed) */
    float       tilt_deg;  /* body down axis from the vertical, if an attitude */
    float       axis_sd_deg; /* its 1-sigma from the INS, -1 = no INS attitude */
    double      ant_lat_deg, ant_lon_deg, ant_h_ell_m;
} best_pos_t;

/* now - t, 0 if t is later: the caller's now may be from before the
 * output or the epoch was taken. */
static uint64_t age_us(uint64_t now, uint64_t t) { return t > now ? 0 : now - t; }

static void best_position(const nav_output_t *o, uint64_t now, best_pos_t *b)
{
    memset(b, 0, sizeof *b);
    b->source = "none";
    bool imu_live = o->imu_age_ms >= 0 && o->imu_age_ms < POS_IMU_FRESH_MS;
    if (o->t_us != 0 && age_us(now, o->t_us) < POS_OUT_FRESH_US && o->pos_valid && imu_live) {
        b->source = "ins";
        b->lat_deg = o->llh[0] * DEG;
        b->lon_deg = o->llh[1] * DEG;
        b->h_ell_m = o->llh[2];
    } else if (s_last_nav.pos_usable && age_us(now, s_last_nav.t_valid_us) < POS_GNSS_FRESH_US) {
        b->source = "gnss";
        b->fix = fix_name(s_last_nav.pos_type);
        b->lat_deg = s_last_nav.lat_deg;
        b->lon_deg = s_last_nav.lon_deg;
        b->h_ell_m = s_last_nav.hgt_msl_m + (double)s_last_nav.undulation_m;
    }

    /* Ground point: the filter's position is the IMU point, so its offset
     * is the lever arm plus the antenna height; the receiver's is the
     * antenna, so only the height. Along the body axes while the INS
     * attitude is good enough (APP_GROUND_TILT_MAX_SD_DEG), else straight
     * down, as for a levelled pole: the AHRS / ARS attitudes have no
     * uncertainty to check, and a wrong yaw would throw a leaning pole's
     * tip the wrong way. */
    float h_ant = settings_ant_height(&s_settings, settings_platform(&s_boot_settings));
    if (b->source[0] == 'n') return;
    bool ins = b->source[0] == 'i';
    if (!(h_ant > 0.0f)) {
        /* No ground point: lat/lon/h stay the output, and ant_* is the
         * antenna ($PAOGI sends the antenna): the filter's IMU point moved
         * by the lever arm, or the receiver's fix. */
        double in[3] = { b->lat_deg / DEG, b->lon_deg / DEG, b->h_ell_m }, ant[3];
        if (ins) {
            double rpy[3] = { o->rpy_rad[0], o->rpy_rad[1], o->rpy_rad[2] };
            ground_point(in, o->rpy_valid ? rpy : NULL, s_settings.leverarm_m, ant);
            b->ant_lat_deg = ant[0] * DEG;
            b->ant_lon_deg = ant[1] * DEG;
            b->ant_h_ell_m = ant[2];
        } else {
            b->ant_lat_deg = b->lat_deg;
            b->ant_lon_deg = b->lon_deg;
            b->ant_h_ell_m = b->h_ell_m;
        }
        return;
    }
    /* The pole axis in the body frame (pole_mount.h): the body down axis
     * unless a learned one is in use. */
    float u[3];
    pole_mount_axis(u);
    double rpy[3] = { o->rpy_rad[0], o->rpy_rad[1], o->rpy_rad[2] };
    const double *att = NULL;
    b->axis_sd_deg = -1.0f;
    if (o->rpy_valid) {
        /* The pole's tilt: the axis turned to NED, against the vertical. */
        double cr = cos(rpy[0]), sr = sin(rpy[0]), cp = cos(rpy[1]), sp = sin(rpy[1]);
        double cz = -sp * u[0] + sr * cp * u[1] + cr * cp * u[2];
        b->tilt_deg = (float)(acos(cz > 1.0 ? 1.0 : (cz < -1.0 ? -1.0 : cz)) * DEG);
        if (o->rpy_sd_valid) {
            b->axis_sd_deg = (float)(ground_axis_sd(rpy, o->rpy_sd_rad) * DEG);
            if (b->axis_sd_deg <= APP_GROUND_TILT_MAX_SD_DEG) att = rpy;
        }
    }
    /* Along the axis with an attitude; straight down without one: a
     * levelled pole's axis is the true vertical, not the IMU's. */
    float off[3] = { 0.0f, 0.0f, h_ant };
    if (att) {
        for (int i = 0; i < 3; i++) off[i] = h_ant * u[i];
    }
    if (ins) {
        for (int i = 0; i < 3; i++) off[i] += s_settings.leverarm_m[i];
    }
    double in[3] = { b->lat_deg / DEG, b->lon_deg / DEG, b->h_ell_m }, gp[3], ant[3];
    ground_point(in, att, off, gp);
    if (ins) {
        /* The antenna from the IMU point, for the status. */
        ground_point(in, att, s_settings.leverarm_m, ant);
    } else {
        ant[0] = in[0];
        ant[1] = in[1];
        ant[2] = in[2];
    }
    b->ground = true;
    b->ant_height_m = h_ant;
    b->tilted = att != NULL;
    b->ant_lat_deg = ant[0] * DEG;
    b->ant_lon_deg = ant[1] * DEG;
    b->ant_h_ell_m = ant[2];
    b->lat_deg = gp[0] * DEG;
    b->lon_deg = gp[1] * DEG;
    b->h_ell_m = gp[2];
}

/* $PAOGI on the USB port at 10 Hz on the tractor platform (paogi.h,
 * plans/ethernet-w5500-agopenweb.md for the field rules). Sent when core 1
 * publishes, or on its own 100 ms timer while core 1 doesn't, so the
 * receiver-only fallback keeps the rate. */
#define PAOGI_PERIOD_US    100000u
#define PAOGI_FALLBACK_US  150000u  /* no publish for this long: the timer sends */
#define PAOGI_DR_AFTER_US  1000000u /* INS with no fresh GNSS epoch: quality 6 */
#define DAY_MS             86400000LL

static bool     s_paogi_on;     /* tractor platform in effect since boot */
static double   s_paogi_gyr[3]; /* corrected gyro summed since the last sentence */
static uint32_t s_paogi_gyr_n;

static void paogi_gyro(const float g[3])
{
    if (!s_paogi_on) return;
    for (int i = 0; i < 3; i++) s_paogi_gyr[i] += g[i];
    s_paogi_gyr_n++;
}

/* The receiver's position type on the GGA quality scale. PPP counts as a
 * single fix: its accuracy while converging is not known from the type. */
static int gga_quality(uint32_t pos_type)
{
    switch (pos_type) {
    case UM980_TYPE_NONE:           return 0;
    case UM980_TYPE_PSRDIFF:
    case UM980_TYPE_SBAS:           return 2;
    case UM980_TYPE_L1_FLOAT:
    case UM980_TYPE_IONOFREE_FLOAT:
    case UM980_TYPE_NARROW_FLOAT:   return 5;
    case UM980_TYPE_L1_INT:
    case UM980_TYPE_WIDE_INT:
    case UM980_TYPE_NARROW_INT:     return 4;
    default:                        return 1;
    }
}

static void paogi_send(const nav_output_t *o, uint64_t now)
{
    best_pos_t bp;
    best_position(o, now, &bp);
    bool ins = bp.source[0] == 'i';
    bool gnss_fresh = s_have_nav && s_last_nav.pos_usable && age_us(now, s_last_nav.t_valid_us) < PAOGI_DR_AFTER_US;

    paogi_fix_t f;
    memset(&f, 0, sizeof f);
    f.sats = -1;
    f.hdop = -1.0f;
    f.age_s = -1.0f;

    /* UTC from the last epoch's GPS time, moved on to the filter output's
     * time when that is the source. Integer milliseconds: a float time of
     * week is too coarse late in the week. */
    if (s_have_nav) {
        int64_t ms = (int64_t)s_last_nav.tow_ms - (int64_t)s_last_nav.leap_s * 1000;
        if (ins) ms += ((int64_t)o->t_us - (int64_t)s_last_nav.t_valid_us) / 1000;
        f.time_valid = true;
        f.utc_ms = (uint32_t)(((ms % DAY_MS) + DAY_MS) % DAY_MS);
        f.sats = s_last_nav.n_used;
    }
    const um980_stats_t *gs = um980_stats(&s_gnss);
    if (gs->dop_valid) f.hdop = gs->hdop;
    if (gnss_fresh && s_last_nav.diff_age_s > 0.0f) f.age_s = s_last_nav.diff_age_s;

    /* The antenna, not the ground point: AgOpenGPS corrects for the
     * antenna height itself. */
    if (bp.source[0] != 'n') {
        f.pos_valid = true;
        f.lat_deg = bp.ant_lat_deg;
        f.lon_deg = bp.ant_lon_deg;
        f.alt_msl_m = bp.ant_h_ell_m - (s_have_undulation ? (double)s_undulation_m : 0.0);
        if (!ins) {
            f.quality = gga_quality(s_last_nav.pos_type);
        } else {
            f.quality = gnss_fresh ? gga_quality(s_last_nav.pos_type) : 6;
        }
    }

    if (ins && o->vel_valid) {
        f.speed_valid = true;
        f.speed_mps = hypotf(o->vel_ned[0], o->vel_ned[1]);
    } else if (!ins && s_have_nav && s_last_nav.vel_usable && age_us(now, s_last_nav.t_valid_us) < POS_GNSS_FRESH_US) {
        f.speed_valid = true;
        f.speed_mps = (float)s_last_nav.hor_speed_mps;
    }

    /* Roll and pitch from the filter's best attitude source while it runs
     * on a live IMU; the heading only from the INS while it navigates (a
     * magnetometer yaw is no steering heading). */
    bool out_live = o->t_us != 0 && age_us(now, o->t_us) < POS_OUT_FRESH_US && o->imu_age_ms >= 0 &&
                    o->imu_age_ms < POS_IMU_FRESH_MS;
    double roll = 0.0, pitch = 0.0;
    if (out_live && o->rpy_valid) {
        roll = o->rpy_rad[0];
        pitch = o->rpy_rad[1];
        f.att_valid = true;
        f.roll_deg = (float)(roll * DEG);
        f.pitch_deg = (float)(pitch * DEG);
        if (ins && o->att_source == NAV_ATT_INS &&
            (o->mode == NAV_SUITE_MODE_FULL || o->mode == NAV_SUITE_MODE_COASTING)) {
            f.heading_valid = true;
            f.heading_deg = (float)(o->rpy_rad[2] * DEG);
        }
    }

    /* Yaw rate: the mean corrected gyro since the last sentence, less the
     * filter's bias, turned into the rate of the NED yaw (clockwise). */
    if (s_paogi_gyr_n > 0) {
        double q = s_paogi_gyr[1] / s_paogi_gyr_n, r = s_paogi_gyr[2] / s_paogi_gyr_n;
        if (o->bias_valid) {
            q -= o->gyr_bias[1];
            r -= o->gyr_bias[2];
        }
        double cp = cos(pitch);
        if (fabs(cp) > 0.1) f.yaw_rate_dps = (float)((q * sin(roll) + r * cos(roll)) / cp * DEG);
    }
    s_paogi_gyr[0] = s_paogi_gyr[1] = s_paogi_gyr[2] = 0.0;
    s_paogi_gyr_n = 0;

    char line[PAOGI_MAX];
    if (paogi_format(&f, line, sizeof line) > 0) printf("%s", line);
}

/* Called every pass: decides whether a sentence is due. */
static bool paogi_poll(uint64_t now)
{
    static bool     s_init;
    static uint32_t s_seq;
    static uint64_t s_last_pub_us, s_next_us;
    uint32_t seq = nav_output_seq();
    bool published = seq != s_seq;
    if (!s_init) {
        s_init = true;
        s_seq = seq;
        s_last_pub_us = now;
        s_next_us = now + PAOGI_PERIOD_US;
        return false;
    }
    if (published) {
        s_seq = seq;
        s_last_pub_us = now;
    }
    bool due;
    if (now - s_last_pub_us < PAOGI_FALLBACK_US) {
        due = published;
    } else {
        due = now >= s_next_us;
    }
    if (!due) return false;
    s_next_us = now + PAOGI_PERIOD_US;
    if (usb_log_on() || !paogi_enabled()) return false;
    nav_output_t o;
    nav_get_output(&o);
    paogi_send(&o, now);
    return true;
}

/* Survey-pole aiding (pole_aid.h): one or two lines, nothing when off. */
static void print_pole(const pole_aid_status_t *p)
{
    if (!p->active) return;
    printf("POLE  %s  tip err %.3f m/s2  tip speed ", p->planted ? (p->latched ? "planted, tip held" : "planted")
                                                                  : "moving",
           (double)p->tip_err_ms2);
    if (p->tip_speed_ms < 0.0f) {
        printf("- (no INS)");
    } else {
        printf("%.2f m/s", (double)p->tip_speed_ms);
    }
    printf("  plants %lu  tip fusions %lu  yaw fits %lu (rejected %lu)\n", (unsigned long)p->plants,
           (unsigned long)p->tip_fusions, (unsigned long)p->yaw_fits, (unsigned long)p->yaw_rejects);
    if (p->fit_skipped) {
        printf("      last fit skipped, too little sway: n %u  spread %.3f m\n", p->fit_n,
               (double)p->fit_spread_m);
    } else if (p->fit_n > 0) {
        printf("      last fit %s: dpsi %.1f deg  sd %.1f  n %u  spread %.3f m  rms %.3f m  scale %.3f\n",
               p->fit_ok ? "used" : "rejected", (double)p->fit_dpsi_deg, (double)p->fit_sd_deg, p->fit_n,
               (double)p->fit_spread_m, (double)p->fit_rms_m, (double)p->fit_scale);
    }
    printf("      mag %s  |B| err %.1f %%  dip err %.1f deg  tilt range %.1f  yaw range %.0f deg",
           p->mag_trusted ? "trusted" : "withheld", (double)p->mag_norm_err_pct, (double)p->mag_dip_err_deg,
           (double)p->mag_tilt_range_deg, (double)p->mag_yaw_range_deg);
    if (!isnan(p->mag_res_mean_deg)) {
        printf("  vs INS %.1f +- %.1f deg", (double)p->mag_res_mean_deg, (double)p->mag_res_sd_deg);
    }
    printf("\n");
}

static void print_status(uint64_t now, uint32_t baro_hz)
{
    const tm210_stats_t *is = tm210_stats(&s_imu);
    const um980_stats_t *gs = um980_stats(&s_gnss);
    const bmp581_stats_t *bs = bmp581_stats(&s_baro);
    nav_output_t o;

    nav_get_output(&o);

    printf("\n--- t=%.1f s ---\n", (double)now * 1e-6);

#if APP_WEB_BUILT
    web_info_t wi;
    web_get_info(&wi);
    const uint8_t *ip = (const uint8_t *)&wi.ip;
    if (!s_web_on) {
        printf("WIFI  off             (web-mode switch open at boot)\n");
    } else {
    printf("WIFI  %-15s %s", web_state_name(wi.state), web_ssid());
    if (wi.state == WEB_WIFI_UP) {
        printf("  http://%u.%u.%u.%u/  %ld dBm", ip[0], ip[1], ip[2], ip[3], (long)wi.rssi_dbm);
    } else if (wi.state == WEB_WIFI_AP) {
        printf("  http://%u.%u.%u.%u/  clients %lu", ip[0], ip[1], ip[2], ip[3],
               (unsigned long)wi.ap_clients);
    } else if (wi.state == WEB_WIFI_WAIT_RETRY) {
        printf("  retry in %lu s", (unsigned long)wi.retry_in_s);
    }
    printf("  joins %lu  drops %lu\n", (unsigned long)wi.joins, (unsigned long)wi.drops);
    }
#endif

    printf("IMU   %-15s %4lu Hz  crc %lu  gaps %lu  ", tm210_state_name(tm210_state(&s_imu)),
           (unsigned long)is->rate_hz, (unsigned long)is->crc_errors, (unsigned long)is->gaps);
    if (is->status_valid) {
        printf("%.1f C  qos %u  status %lu Hz", (double)is->temperature_c, is->qos,
               (unsigned long)is->status_rate_hz);
    }
    if (tm210_last_error(&s_imu) != TM210_OK) {
        printf("  [%s]", tm210_strerror(tm210_last_error(&s_imu)));
    }
    printf("\n");
    if (tm210_state(&s_imu) == TM210_ST_RUN) {
        printf("      acc %7.3f %7.3f %7.3f m/s2   gyr %7.4f %7.4f %7.4f rad/s\n",
               (double)s_last_imu.acc[0], (double)s_last_imu.acc[1], (double)s_last_imu.acc[2],
               (double)s_last_imu.gyr[0], (double)s_last_imu.gyr[1], (double)s_last_imu.gyr[2]);
        const noise_report_t *r = &s_noise_rep;
        if (r->windows > 0) {
            printf("      noise 0.2 s  gyr %.3f/%.3f/%.3f deg/s  acc %.3f/%.3f/%.3f m/s2"
                   "  (min/mean/max)  still %lu/%lu\n",
                   (double)r->gyr_dps[0], (double)r->gyr_dps[1], (double)r->gyr_dps[2],
                   (double)r->acc[0], (double)r->acc[1], (double)r->acc[2],
                   (unsigned long)r->pass, (unsigned long)r->windows);
        }
    }
    print_calib();

    printf("GNSS  %-15s %4lu Hz  crc %lu  pps %lu", um980_state_name(um980_state(&s_gnss)),
           (unsigned long)gs->nav_rate_hz, (unsigned long)gs->crc_errors,
           (unsigned long)gs->pps_edges);
    if (model_name() != NULL) {
        printf("  %s build %lu", model_name(), (unsigned long)s_gnss.build);
        printf("  %s antenna", um980_dual(&s_gnss) ? "dual" : "single");
    }
    if (um980_last_error(&s_gnss) != UM980_OK) {
        printf("  [%s]", um980_strerror(um980_last_error(&s_gnss)));
    }
    printf("\n");
    if (um980_dual_refused(&s_gnss)) {
        printf("      warning: dual antenna is set, but a UM980 is fitted: running single\n");
    }
    if (um980_dual(&s_gnss) && s_have_hdg) {
        printf("      heading %.2f deg  sd %.2f  pitch %.2f  %s  baseline %.3f m (median %.3f)"
               "  used %lu  gated %lu\n",
               (double)s_last_hdg.heading_deg, (double)s_last_hdg.sd_heading_deg,
               (double)s_last_hdg.pitch_deg, fix_name(s_last_hdg.pos_type),
               (double)s_last_hdg.length_m, (double)s_base_median_m, (unsigned long)s_hdg_used,
               (unsigned long)s_hdg_gated);
    }
    if (s_have_nav) {
        printf("      %s  sv %u/%u  hdop %.1f  age %.1f s  time %s\n", fix_name(s_last_nav.pos_type),
               s_last_nav.n_used, s_last_nav.n_tracked, gs->dop_valid ? (double)gs->hdop : 0.0,
               (double)s_last_nav.diff_age_s, s_last_nav.t_from_pps ? "pps" : "arrival");
        if (s_last_nav.pos_usable) {
            printf("      lat %.8f  lon %.8f  h %.3f m  sd %.3f %.3f %.3f m\n", s_last_nav.lat_deg,
                   s_last_nav.lon_deg, s_last_nav.hgt_msl_m + (double)s_last_nav.undulation_m,
                   (double)s_last_nav.sd_lat_m, (double)s_last_nav.sd_lon_m,
                   (double)s_last_nav.sd_hgt_m);
        }
    }

    if (!s_settings.sensor_baro) {
        printf("BARO  off (settings)\n");
    } else if (s_baro_ok) {
        printf("BARO  run             %4lu Hz  i2c errors %lu  reinits %lu  %.1f Pa  %.1f C\n",
               (unsigned long)baro_hz, (unsigned long)bs->i2c_errors, (unsigned long)bs->reinits,
               (double)s_last_baro.pressure_pa, (double)s_last_baro.temperature_c);
    } else {
        printf("BARO  not found\n");
    }

    field_io_print_status();

    const cpu_load_result_t *c0 = &s_load0_res, *c1 = &o.load;
    for (int k = 0; k < 2; k++) {
        const cpu_load_result_t *c = k ? c1 : c0;
        printf("%s core%d %5.1f %%  loop max %lu us  idle pass %lu ns (min %lu)  passes/s idle %lu work %lu\n",
               k ? "     " : "CPU  ", k, (double)c->load_permille / 10.0, (unsigned long)c->max_pass_us,
               (unsigned long)c->idle_pass_ns, (unsigned long)c->idle_min_ns,
               (unsigned long)c->idle_per_s, (unsigned long)c->work_per_s);
    }
    printf("      die %.1f C (on-chip sensor, uncalibrated)\n", (double)s_die_c);

    if (o.t_us == 0) {
        printf("NAV   starting\n");
    } else {
        nav_drops_t d;
        nav_queue_drops(&d);
        printf("NAV   %-15s %s  update mean %lu us, max %lu us  stack %lu/%lu B\n",
               nav_mode_name(o.mode), o.dual ? "dual" : "single", (unsigned long)o.update_us_mean,
               (unsigned long)o.update_us_max, (unsigned long)nav_stack_used(),
               (unsigned long)nav_stack_size());
        if (o.imu_age_ms < 0 || o.imu_age_ms >= POS_IMU_FRESH_MS) {
            printf("      IMU: NO DATA %s, filter stopped (IMU losses %lu, epochs without IMU %lu)\n",
                   o.imu_age_ms < 0 ? "yet" : "for a while", (unsigned long)o.n_imu_loss,
                   (unsigned long)o.n_stall_epochs);
            if (o.imu_age_ms >= 0) printf("      last IMU sample %ld ms ago\n", (long)o.imu_age_ms);
        } else if (o.n_imu_loss) {
            printf("      IMU losses so far %lu (filter stopped each time and re-initialised)\n",
                   (unsigned long)o.n_imu_loss);
        }
        printf("      queue drops: imu %lu  gnss %lu  baro %lu  speed %lu\n", (unsigned long)d.imu,
               (unsigned long)d.gnss, (unsigned long)d.baro, (unsigned long)d.speed);
        if (o.rpy_valid) {
            printf("      roll %7.2f  pitch %7.2f  yaw %7.2f deg%s\n", (double)o.rpy_rad[0] * DEG,
                   (double)o.rpy_rad[1] * DEG, (double)o.rpy_rad[2] * DEG,
                   o.zaru_active ? "  (standing still)" : "");
        }
        printf("      attitude from %s\n", att_source_name(o.att_source));
        printf("MAG   AHRS %-3s  samples %3lu Hz  |B| %.1f uT  ", o.ahrs_valid ? "run" : "off",
               (unsigned long)s_mag_hz, (double)mag_field_ut());
        if (o.mag_fusion_age_ms < 0) {
            printf("no fusion yet");
        } else {
            printf("last fusion %ld ms ago", (long)o.mag_fusion_age_ms);
        }
        printf("  north %s\n", mag_north_name(&o));
        print_pole(&o.pole);
        if (o.pos_valid) {
            printf("      lat %.8f  lon %.8f  h %.3f m\n", o.llh[0] * DEG, o.llh[1] * DEG, o.llh[2]);
        }
        if (o.vel_valid) {
            printf("      vel N %.3f  E %.3f  D %.3f m/s\n", (double)o.vel_ned[0],
                   (double)o.vel_ned[1], (double)o.vel_ned[2]);
        }
        if (o.height_valid) {
            printf("      height above start %.2f m\n", (double)o.height_m);
        }
        if (o.bias_valid) {
            printf("      gyro bias %.5f %.5f %.5f rad/s   acc bias %.4f %.4f %.4f m/s2\n",
                   (double)o.gyr_bias[0], (double)o.gyr_bias[1], (double)o.gyr_bias[2],
                   (double)o.acc_bias[0], (double)o.acc_bias[1], (double)o.acc_bias[2]);
        }
        printf("      epochs: imu %lu  gnss %lu (late %lu)  yaw %lu  baro %lu (late %lu)  speed flow %lu wheel %lu"
               " (late %lu, fused %lu)\n",
               (unsigned long)o.n_updates, (unsigned long)o.n_gnss, (unsigned long)o.n_gnss_late,
               (unsigned long)o.n_yaw, (unsigned long)o.n_baro, (unsigned long)o.n_baro_late,
               (unsigned long)o.n_speed[NAV_SPEED_FLOW], (unsigned long)o.n_speed[NAV_SPEED_WHEEL],
               (unsigned long)o.n_speed_late, (unsigned long)o.n_speed_used);
        printf("      rx ring overflows: imu %lu  gnss %lu\n",
               (unsigned long)s_imu_port.rx_overflows, (unsigned long)s_gnss_port.rx_overflows);
    }

    best_pos_t bp;
    best_position(&o, now, &bp);
    if (bp.source[0] == 'n') {
        printf("POS   none (no filter solution, no recent receiver fix)\n");
    } else {
        printf("POS   from %s%s%s%s  lat %.8f  lon %.8f  h %.3f m (ellipsoid)\n",
               bp.source[0] == 'i' ? "the filter" : "the receiver", bp.fix ? " (" : "", bp.fix ? bp.fix : "",
               bp.fix ? ", filter down)" : "", bp.lat_deg, bp.lon_deg, bp.h_ell_m);
        float n;
        const char *model;
        if (undulation_at(bp.lat_deg, bp.lon_deg, &n, &model)) {
            printf("      h %.3f m above the geoid (%s)\n", bp.h_ell_m - (double)n, model);
        }
        if (bp.ground) {
            printf("      ground point %.3f m below the antenna, %s", (double)bp.ant_height_m,
                   bp.tilted ? "along the body down axis" : "straight down");
            if (bp.axis_sd_deg >= 0.0f) {
                printf(" (tilt %.1f deg, axis sd %.2f deg, gate %.1f)", (double)bp.tilt_deg,
                       (double)bp.axis_sd_deg, (double)APP_GROUND_TILT_MAX_SD_DEG);
            } else {
                printf(" (no INS attitude)");
            }
            printf("\n      antenna lat %.8f  lon %.8f  h %.3f m\n", bp.ant_lat_deg, bp.ant_lon_deg, bp.ant_h_ell_m);
        }
    }
}

#if APP_WEB_BUILT
static void jw_vec3(jw_t *w, const char *key, const float *v, int decimals)
{
    jw_arr(w, key);
    for (int i = 0; i < 3; i++) {
        jw_num(w, NULL, (double)v[i], decimals);
    }
    jw_end(w, ']');
}

/* h_geoid_m and geoid_model beside an ellipsoid height; nothing while no
 * geoid height is known there. */
static void jw_geoid(jw_t *w, double lat_deg, double lon_deg, double h_ell_m)
{
    float n;
    const char *model;
    if (!undulation_at(lat_deg, lon_deg, &n, &model)) return;
    jw_num(w, "h_geoid_m", h_ell_m - (double)n, 3);
    jw_str(w, "geoid_model", model);
}

/* "pole" in /status.json; nothing when the aiding is off. Short keys: the
 * document has to stay under WEB_DOC_MAX. */
static void jw_pole(jw_t *w, const pole_aid_status_t *p)
{
    if (!p->active) return;
    jw_obj(w, "pole");
    jw_str(w, "state", p->planted ? (p->latched ? "held" : "planted") : "moving");
    jw_num(w, "tip_err", (double)p->tip_err_ms2, 3);
    jw_num(w, "tip_speed", (double)p->tip_speed_ms, 2);
    jw_int(w, "plants", p->plants);
    jw_int(w, "tip_fusions", p->tip_fusions);
    jw_int(w, "yaw_fits", p->yaw_fits);
    jw_int(w, "yaw_rejects", p->yaw_rejects);
    if (p->fit_n > 0) {
        jw_obj(w, "fit");
        jw_str(w, "result", p->fit_skipped ? "skipped" : (p->fit_ok ? "used" : "rejected"));
        jw_int(w, "n", p->fit_n);
        jw_num(w, "spread", (double)p->fit_spread_m, 3);
        if (!p->fit_skipped) {
            jw_num(w, "dpsi", (double)p->fit_dpsi_deg, 1);
            jw_num(w, "sd", (double)p->fit_sd_deg, 1);
            jw_num(w, "rms", (double)p->fit_rms_m, 3);
        }
        jw_end(w, '}');
    }
    jw_obj(w, "mag");
    jw_bool(w, "trusted", p->mag_trusted);
    jw_num(w, "norm_err_pct", (double)p->mag_norm_err_pct, 1);
    jw_num(w, "dip_err", (double)p->mag_dip_err_deg, 1);
    jw_num(w, "tilt_range", (double)p->mag_tilt_range_deg, 1);
    if (!isnan(p->mag_res_mean_deg)) {
        jw_num(w, "res", (double)p->mag_res_mean_deg, 1);
        jw_num(w, "res_sd", (double)p->mag_res_sd_deg, 1);
    }
    jw_end(w, '}');
    jw_end(w, '}');
}

static void jw_load(jw_t *w, const char *key, const cpu_load_result_t *r)
{
    jw_obj(w, key);
    jw_num(w, "load_pct", (double)r->load_permille / 10.0, 1);
    jw_int(w, "loop_max_us", r->max_pass_us);
    jw_int(w, "idle_pass_ns", r->idle_pass_ns);
    jw_int(w, "idle_min_ns", r->idle_min_ns);
    jw_int(w, "idle_passes_per_s", r->idle_per_s);
    jw_int(w, "work_passes_per_s", r->work_per_s);
    jw_end(w, '}');
}

/* The same data as print_status(), as JSON for /status.json. */
static void publish_status_json(uint64_t now, uint32_t baro_hz)
{
    static char buf[WEB_DOC_MAX];
    const tm210_stats_t *is = tm210_stats(&s_imu);
    const um980_stats_t *gs = um980_stats(&s_gnss);
    const bmp581_stats_t *bs = bmp581_stats(&s_baro);
    nav_output_t o;
    web_info_t wi;
    jw_t w;

    nav_get_output(&o);
    web_get_info(&wi);
    jw_init(&w, buf, sizeof buf);
    jw_obj(&w, NULL);
    jw_str(&w, "fw", "OpenGINS demo");
    jw_int(&w, "uptime_s", (long long)(now / 1000000u));

    char ip[16];
    const uint8_t *b = (const uint8_t *)&wi.ip;
    snprintf(ip, sizeof ip, "%u.%u.%u.%u", b[0], b[1], b[2], b[3]);
    jw_obj(&w, "wifi");
    jw_str(&w, "state", web_state_name(wi.state));
    jw_str(&w, "ssid", web_ssid());
    jw_str(&w, "ip", ip);
    if (wi.state == WEB_WIFI_AP) {
        jw_int(&w, "ap_clients", wi.ap_clients);
    } else if (wi.state == WEB_WIFI_UP) {
        jw_int(&w, "rssi_dbm", wi.rssi_dbm);
    } else {
        jw_null(&w, "rssi_dbm");
    }
    jw_int(&w, "joins", wi.joins);
    jw_int(&w, "drops", wi.drops);
    jw_int(&w, "http_busy", wi.http_busy);
    jw_end(&w, '}');

    jw_obj(&w, "imu");
    jw_str(&w, "state", tm210_state_name(tm210_state(&s_imu)));
    jw_str(&w, "error", tm210_last_error(&s_imu) == TM210_OK ? "ok" :
                        tm210_strerror(tm210_last_error(&s_imu)));
    jw_int(&w, "rate_hz", is->rate_hz);
    jw_int(&w, "crc_errors", is->crc_errors);
    jw_int(&w, "gaps", is->gaps);
    if (is->status_valid) {
        jw_num(&w, "temp_c", (double)is->temperature_c, 1);
        jw_int(&w, "qos", is->qos);
    } else {
        jw_null(&w, "temp_c");
        jw_null(&w, "qos");
    }
    if (tm210_state(&s_imu) == TM210_ST_RUN) {
        jw_vec3(&w, "acc_mps2", s_last_imu.acc, 3);
        jw_vec3(&w, "gyr_rps", s_last_imu.gyr, 4);
        if (s_noise_rep.windows > 0) {
            jw_vec3(&w, "noise_gyr_dps", s_noise_rep.gyr_dps, 3);
            jw_vec3(&w, "noise_acc_mps2", s_noise_rep.acc, 3);
            jw_int(&w, "noise_still_windows", s_noise_rep.pass);
            jw_int(&w, "noise_windows", s_noise_rep.windows);
        }
    }
    jw_end(&w, '}');

    jw_obj(&w, "gnss");
    jw_str(&w, "state", um980_state_name(um980_state(&s_gnss)));
    jw_str(&w, "error", um980_last_error(&s_gnss) == UM980_OK ? "ok" :
                        um980_strerror(um980_last_error(&s_gnss)));
    jw_int(&w, "rate_hz", gs->nav_rate_hz);
    jw_int(&w, "crc_errors", gs->crc_errors);
    jw_int(&w, "pps_edges", gs->pps_edges);
    if (model_name() != NULL) {
        jw_str(&w, "model", model_name());
        jw_int(&w, "build", (long long)s_gnss.build);
        jw_str(&w, "antenna", um980_dual(&s_gnss) ? "dual" : "single");
    } else {
        jw_null(&w, "model");
    }
    jw_bool(&w, "dual_refused", um980_dual_refused(&s_gnss));
    if (s_have_nav) {
        jw_str(&w, "fix", fix_name(s_last_nav.pos_type));
        jw_int(&w, "sv_used", s_last_nav.n_used);
        jw_int(&w, "sv_tracked", s_last_nav.n_tracked);
        if (gs->dop_valid) {
            jw_num(&w, "hdop", (double)gs->hdop, 1);
        } else {
            jw_null(&w, "hdop");
        }
        jw_num(&w, "age_s", (double)s_last_nav.diff_age_s, 1);
        jw_str(&w, "time_from", s_last_nav.t_from_pps ? "pps" : "arrival");
        if (s_last_nav.pos_usable) {
            jw_num(&w, "lat_deg", s_last_nav.lat_deg, 8);
            jw_num(&w, "lon_deg", s_last_nav.lon_deg, 8);
            double h_ell = s_last_nav.hgt_msl_m + (double)s_last_nav.undulation_m;
            jw_num(&w, "h_ell_m", h_ell, 3);
            jw_geoid(&w, s_last_nav.lat_deg, s_last_nav.lon_deg, h_ell);
            jw_num(&w, "h_msl_receiver_m", s_last_nav.hgt_msl_m, 3);
            jw_num(&w, "undulation_receiver_m", (double)s_last_nav.undulation_m, 3);
            float sd[3] = { s_last_nav.sd_lat_m, s_last_nav.sd_lon_m, s_last_nav.sd_hgt_m };
            jw_vec3(&w, "sd_m", sd, 3);
        }
    }
    jw_end(&w, '}');

    if (um980_dual(&s_gnss) && s_have_hdg) {
        jw_obj(&w, "heading");
        jw_num(&w, "heading_deg", (double)s_last_hdg.heading_deg, 2);
        jw_num(&w, "sd_deg", (double)s_last_hdg.sd_heading_deg, 2);
        jw_num(&w, "pitch_deg", (double)s_last_hdg.pitch_deg, 2);
        jw_str(&w, "fix", fix_name(s_last_hdg.pos_type));
        jw_num(&w, "baseline_m", (double)s_last_hdg.length_m, 3);
        jw_num(&w, "median_m", (double)s_base_median_m, 3);
        jw_int(&w, "used", s_hdg_used);
        jw_int(&w, "gated", s_hdg_gated);
        jw_end(&w, '}');
    }

    jw_obj(&w, "baro");
    jw_bool(&w, "enabled", s_settings.sensor_baro != 0);
    jw_bool(&w, "found", s_baro_ok);
    if (s_baro_ok && s_settings.sensor_baro) {
        jw_int(&w, "rate_hz", baro_hz);
        jw_int(&w, "i2c_errors", bs->i2c_errors);
        jw_int(&w, "reinits", bs->reinits);
        jw_num(&w, "pressure_pa", (double)s_last_baro.pressure_pa, 1);
        jw_num(&w, "temp_c", (double)s_last_baro.temperature_c, 1);
    }
    jw_end(&w, '}');

    field_io_json(&w);

    jw_obj(&w, "cpu");
    jw_load(&w, "core0", &s_load0_res);
    jw_load(&w, "core1", &o.load);
    jw_num(&w, "die_temp_c", (double)s_die_c, 1);
    jw_end(&w, '}');

    jw_obj(&w, "nav");
    if (o.t_us == 0) {
        jw_str(&w, "mode", "starting");
    } else {
        jw_str(&w, "mode", nav_mode_name(o.mode));
        jw_str(&w, "attitude_from", att_source_name(o.att_source));
        jw_str(&w, "antenna", o.dual ? "dual" : "single");
        jw_int(&w, "update_mean_us", o.update_us_mean);
        jw_int(&w, "update_max_us", o.update_us_max);
        jw_int(&w, "stack_used_b", nav_stack_used());
        jw_int(&w, "stack_size_b", nav_stack_size());
        nav_drops_t d;
        nav_queue_drops(&d);
        jw_obj(&w, "queue_drops");
        jw_int(&w, "imu", d.imu);
        jw_int(&w, "gnss", d.gnss);
        jw_int(&w, "baro", d.baro);
        jw_int(&w, "speed", d.speed);
        jw_end(&w, '}');
        jw_int(&w, "reinits", o.n_reinit);
        jw_int(&w, "imu_age_ms", o.imu_age_ms);
        jw_bool(&w, "imu_lost", o.imu_age_ms < 0 || o.imu_age_ms >= POS_IMU_FRESH_MS);
        jw_int(&w, "imu_losses", o.n_imu_loss);
        jw_int(&w, "epochs_without_imu", o.n_stall_epochs);
        if (o.rpy_valid) {
            float rpy[3] = { (float)(o.rpy_rad[0] * DEG), (float)(o.rpy_rad[1] * DEG),
                             (float)(o.rpy_rad[2] * DEG) };
            jw_vec3(&w, "rpy_deg", rpy, 2);
            jw_bool(&w, "standing_still", o.zaru_active);
        }
        if (o.pos_valid) {
            jw_num(&w, "lat_deg", o.llh[0] * DEG, 8);
            jw_num(&w, "lon_deg", o.llh[1] * DEG, 8);
            jw_num(&w, "h_ell_m", o.llh[2], 3);
            jw_geoid(&w, o.llh[0] * DEG, o.llh[1] * DEG, o.llh[2]);
        }
        if (o.vel_valid) {
            jw_vec3(&w, "vel_ned_mps", o.vel_ned, 3);
        }
        if (o.height_valid) {
            jw_num(&w, "height_above_start_m", (double)o.height_m, 2);
        }
        if (o.bias_valid) {
            jw_vec3(&w, "gyr_bias_rps", o.gyr_bias, 5);
            jw_vec3(&w, "acc_bias_mps2", o.acc_bias, 4);
        }
        jw_obj(&w, "epochs");
        jw_int(&w, "imu", o.n_updates);
        jw_int(&w, "gnss", o.n_gnss);
        jw_int(&w, "gnss_late", o.n_gnss_late);
        jw_int(&w, "yaw", o.n_yaw);
        jw_int(&w, "baro", o.n_baro);
        jw_int(&w, "baro_late", o.n_baro_late);
        jw_int(&w, "speed_flow", o.n_speed[NAV_SPEED_FLOW]);
        jw_int(&w, "speed_wheel", o.n_speed[NAV_SPEED_WHEEL]);
        jw_int(&w, "speed_late", o.n_speed_late);
        jw_int(&w, "speed_used", o.n_speed_used);
        jw_int(&w, "speed_skipped", o.n_speed_skipped);
        jw_end(&w, '}');
    }
    jw_end(&w, '}');

    /* What the outputs use: the filter, else the receiver's own fix. */
    best_pos_t bp;
    best_position(&o, now, &bp);
    jw_obj(&w, "position");
    jw_str(&w, "source", bp.source);
    if (bp.fix) jw_str(&w, "fix", bp.fix);
    if (bp.source[0] != 'n') {
        jw_num(&w, "lat_deg", bp.lat_deg, 8);
        jw_num(&w, "lon_deg", bp.lon_deg, 8);
        jw_num(&w, "h_ell_m", bp.h_ell_m, 3);
        jw_geoid(&w, bp.lat_deg, bp.lon_deg, bp.h_ell_m);
        jw_str(&w, "point", bp.ground ? "ground" : "antenna");
        if (bp.ground) {
            jw_num(&w, "ant_height_m", (double)bp.ant_height_m, 3);
            jw_bool(&w, "tilted", bp.tilted);
            if (bp.axis_sd_deg >= 0.0f) {
                jw_num(&w, "tilt_deg", (double)bp.tilt_deg, 1);
                jw_num(&w, "axis_sd_deg", (double)bp.axis_sd_deg, 2);
            } else {
                jw_null(&w, "axis_sd_deg");
            }
            jw_num(&w, "axis_sd_max_deg", (double)APP_GROUND_TILT_MAX_SD_DEG, 1);
            jw_obj(&w, "antenna");
            jw_num(&w, "lat_deg", bp.ant_lat_deg, 8);
            jw_num(&w, "lon_deg", bp.ant_lon_deg, 8);
            jw_num(&w, "h_ell_m", bp.ant_h_ell_m, 3);
            jw_end(&w, '}');
        }
    }
    jw_end(&w, '}');

    if (o.t_us != 0) {
        /* "withheld": the pole's trust gate keeps the magnetometer from the
         * filer, so the AHRS can be off for that reason alone. */
        const bool gated = o.pole.active && !o.pole.mag_trusted;
        char why[160];
        mag_reason(&o.pole, why, sizeof why);
        jw_obj(&w, "mag");
        jw_str(&w, "state", o.ahrs_valid ? "run" : (gated ? "withheld" : "off"));
        jw_int(&w, "samples_hz", s_mag_hz);
        jw_num(&w, "field_ut", (double)mag_field_ut(), 1);
        if (o.mag_fusion_age_ms < 0) {
            jw_null(&w, "last_fusion_ms");
        } else {
            jw_int(&w, "last_fusion_ms", o.mag_fusion_age_ms);
        }
        jw_str(&w, "north", mag_north_name(&o));
        jw_str(&w, "reason", gated ? why : "");
        jw_end(&w, '}');
        jw_pole(&w, &o.pole);
    }

    {
        /* Summary; the details are /calib.json (publish_calib_json()). */
        imu_calib_status_t c;
        imu_calib_status(&c);
        jw_obj(&w, "calib");
        jw_int(&w, "record", c.rec_seq);
        jw_str(&w, "imu_id", calib_id_name(c.id_state));
        jw_bool(&w, "still", c.still);
        const bool ap[IMU_CALIB_GROUPS] = { c.applied, c.acc_applied, c.mag_applied, c.gyr_m_applied };
        static const char *const k_lbl[IMU_CALIB_GROUPS] = { "gyro_bias", "accelerometer", "magnetometer",
                                                             "gyro_matrix" };
        for (int g = 0; g < IMU_CALIB_GROUPS; g++) {
            char t[32];
            snprintf(t, sizeof t, "%s, %s", ap[g] ? "in use" : "none", c.hold[g] ? "hold" : "auto");
            jw_str(&w, k_lbl[g], t);
        }
        jw_str(&w, "details", "/calib.html");
        jw_end(&w, '}');
    }

    jw_obj(&w, "rx_overflows");
    jw_int(&w, "imu", s_imu_port.rx_overflows);
    jw_int(&w, "gnss", s_gnss_port.rx_overflows);
    jw_end(&w, '}');

    jw_end(&w, '}');
    if (!jw_ok(&w)) {
        static const char k_err[] = "{\"error\":\"status JSON too long\"}";
        web_set_status_json(k_err, sizeof k_err - 1);
        return;
    }
    web_set_status_json(buf, w.len);
}

static void jw_floats(jw_t *w, const char *key, const float *v, int n, int dec)
{
    jw_arr(w, key);
    for (int i = 0; i < n; i++) {
        jw_num(w, NULL, (double)v[i], dec);
    }
    jw_end(w, ']');
}

static void jw_scaled3(jw_t *w, const char *key, const float *v, double scale, int dec)
{
    float t[3];
    for (int i = 0; i < 3; i++) {
        t[i] = (float)(v[i] * scale);
    }
    jw_vec3(w, key, t, dec);
}

/* The fields every calibration group has (calib.html). */
static void calib_group_head(jw_t *w, const imu_calib_status_t *c, int g, bool applied, const char *note)
{
    jw_obj(w, imu_calib_group_key(g));
    jw_str(w, "mode", c->hold[g] ? "hold" : "auto");
    jw_bool(w, "applied", applied);
    jw_bool(w, "stored", c->stored[g]);
    jw_str(w, "last", note ? note : "");
    jw_str(w, "next_boot", c->next_boot[g] ? c->next_boot[g] : "");
    if (c->accept_why[g]) {
        jw_str(w, "accept_refused", c->accept_why[g]);
    } else {
        jw_null(w, "accept_refused");
    }
}

/* /calib.json: every calibration group in detail (imu_calib.h). */
static void publish_calib_json(void)
{
    static char buf[WEB_DOC_MAX];
    imu_calib_status_t c;
    imu_calib_status(&c);
    jw_t w;
    jw_init(&w, buf, sizeof buf);
    jw_obj(&w, NULL);
    jw_int(&w, "record", c.rec_seq);
    jw_str(&w, "imu_id", calib_id_name(c.id_state));
    jw_bool(&w, "still", c.still);
    jw_int(&w, "unsaved", c.unsaved);

    /* Gyro bias vs temperature. */
    calib_group_head(&w, &c, IMU_CALIB_GYR, c.applied, c.boot_note);
    jw_num(&w, "t_ref_c", (double)APP_CALIB_T_REF_C, 1);
    if (c.applied) {
        jw_obj(&w, "use");
        jw_scaled3(&w, "b0_dps", c.use.b0, DEG, 4);
        jw_scaled3(&w, "k_dps_c", c.use.k, DEG, 5);
        jw_end(&w, '}');
    }
    if (c.stats.n_chunks > 0) {
        jw_obj(&w, "learned");
        jw_scaled3(&w, "b0_dps", c.learned.b0, DEG, 4);
        jw_num(&w, "b0_sd_dps", (double)c.learned.sd_b0 * DEG, 4);
        jw_scaled3(&w, "k_dps_c", c.learned.k, DEG, 5);
        jw_num(&w, "k_sd_dps_c", (double)c.learned.sd_k * DEG, 5);
        jw_bool(&w, "k_fitted", c.learned.k_fitted);
        jw_end(&w, '}');
    }
    jw_obj(&w, "data");
    jw_int(&w, "chunks", c.stats.n_chunks);
    jw_int(&w, "chunks_boot", c.chunks_boot);
    if (c.stats.n_chunks > 0) {
        jw_num(&w, "t_min_c", (double)c.stats.t_min_c, 1);
        jw_num(&w, "t_max_c", (double)c.stats.t_max_c, 1);
    }
    jw_num(&w, "still_s", (double)c.stats.still_s, 0);
    jw_int(&w, "skip_att", c.skip_att);
    jw_int(&w, "skip_temp", c.skip_temp);
    jw_int(&w, "windows_not_rest", c.windows_not_rest);
    jw_end(&w, '}');
    jw_end(&w, '}');

    /* Accelerometer. */
    calib_group_head(&w, &c, IMU_CALIB_ACC, c.acc_applied, c.acc_boot_note);
    const acal_result_t *ar[2] = { c.acc_applied ? &c.acc_use : NULL, c.acc_poses > 0 ? &c.acc_learned : NULL };
    static const char *const k_key[2] = { "use", "learned" };
    for (int k = 0; k < 2; k++) {
        if (!ar[k]) {
            continue;
        }
        const float *q = ar[k]->p, *e = ar[k]->sd;
        jw_obj(&w, k_key[k]);
        jw_vec3(&w, "scale", &q[ACAL_SX], 4);
        jw_scaled3(&w, "mis_deg", &q[ACAL_MXY], DEG, 3);
        jw_vec3(&w, "bias_mps2", &q[ACAL_BX], 4);
        if (k == 1) {
            jw_vec3(&w, "scale_sd", &e[ACAL_SX], 4);
            jw_scaled3(&w, "mis_sd_deg", &e[ACAL_MXY], DEG, 3);
            jw_vec3(&w, "bias_sd_mps2", &e[ACAL_BX], 4);
        }
        jw_num(&w, "rms_before", (double)ar[k]->rms_before, 4);
        jw_num(&w, "rms_after", (double)ar[k]->rms_after, 4);
        jw_end(&w, '}');
    }
    jw_obj(&w, "data");
    jw_int(&w, "poses", c.acc_poses);
    jw_int(&w, "poses_boot", c.poses_boot);
    jw_int(&w, "kept_old", c.poses_kept_old);
    jw_num(&w, "coverage", (double)c.acc_coverage, 3);
    jw_end(&w, '}');
    jw_end(&w, '}');

    /* Magnetometer. */
    calib_group_head(&w, &c, IMU_CALIB_MAG, c.mag_applied, c.mag_boot_note);
    jw_str(&w, "state", c.mag_state ? c.mag_state : "");
    const mcal_cal_t *mc[2] = { c.mag_applied ? &c.mag_use : NULL, c.mag_solved ? &c.mag_learned : NULL };
    for (int k = 0; k < 2; k++) {
        if (!mc[k]) {
            continue;
        }
        jw_obj(&w, k_key[k]);
        jw_floats(&w, "m", mc[k]->m, 9, 4);
        jw_vec3(&w, "hard_iron_ut", mc[k]->b, 2);
        if (k == 1) {
            jw_num(&w, "m_sd", (double)mc[k]->sd_m, 4);
            jw_num(&w, "hard_iron_sd_ut", (double)mc[k]->sd_b_ut, 2);
        }
        jw_end(&w, '}');
    }
    jw_obj(&w, "data");
    jw_int(&w, "samples", c.mag_bins);
    jw_int(&w, "samples_boot", c.mag_groups_boot);
    jw_int(&w, "outliers", c.mag_outliers);
    jw_num(&w, "coverage", (double)c.mag_coverage, 3);
    jw_int(&w, "reference", c.mag_n_ref);
    jw_int(&w, "reference_boot", c.mag_ref_boot);
    jw_str(&w, "yaw_from", c.mag_ref_source ? c.mag_ref_source : "no GNSS heading");
    if (c.mag_solved) {
        jw_num(&w, "field_ut", (double)c.mag_ell.field_ut, 1);
        jw_num(&w, "rms_before_ut", (double)c.mag_ell.rms_before, 2);
        jw_num(&w, "rms_after_ut", (double)c.mag_ell.rms_after, 2);
    }
    jw_end(&w, '}');
    jw_end(&w, '}');

    /* Gyro scale and misalignment. */
    calib_group_head(&w, &c, IMU_CALIB_GYR_M, c.gyr_m_applied, c.gyr_m_boot_note);
    jw_obj(&w, "use");
    jw_floats(&w, "m", c.gyr_m, 9, 5);
    jw_end(&w, '}');
    if (c.n_rot + c.n_turn > 0) {
        jw_obj(&w, "learned");
        jw_floats(&w, "dm", c.gyr_learned.dm, 9, 5);
        jw_floats(&w, "dm_sd", c.gyr_learned.sd, 9, 5);
        if (c.gyr_taken > 0) {
            jw_floats(&w, "next_m", c.gyr_next, 9, 5);
        }
        jw_int(&w, "elements_taken", c.gyr_taken);
        jw_end(&w, '}');
    }
    jw_obj(&w, "data");
    jw_int(&w, "rotations", c.n_rot);
    jw_int(&w, "rotations_boot", c.rot_boot);
    jw_int(&w, "too_small", c.rot_small);
    jw_int(&w, "turns", c.n_turn);
    jw_int(&w, "turns_boot", c.turn_boot);
    jw_str(&w, "rotation_state", c.rot_state ? c.rot_state : "");
    jw_bool(&w, "in_turn", c.turn_active);
    jw_end(&w, '}');
    jw_end(&w, '}');

    /* Pole axis (pole_mount.h), on the pole only. The same head fields as
     * the IMU groups, so calib.html shows it the same way. */
    pole_mount_status_t pm;
    pole_mount_status(&pm);
    if (pm.active) {
        jw_obj(&w, "pole");
        jw_str(&w, "mode", pm.hold ? "hold" : "auto");
        jw_bool(&w, "applied", pm.applied);
        jw_bool(&w, "stored", pm.stored);
        jw_str(&w, "last", pm.note ? pm.note : "");
        jw_str(&w, "next_boot", pm.next_boot ? pm.next_boot : "");
        if (pm.accept_why) {
            jw_str(&w, "accept_refused", pm.accept_why);
        } else {
            jw_null(&w, "accept_refused");
        }
        jw_obj(&w, "use");
        jw_num(&w, "tilt_deg", (double)pm.use_tilt_deg, 2);
        jw_num(&w, "toward_deg", (double)pm.use_azim_deg, 0);
        jw_end(&w, '}');
        if (pm.stats.n_ep > 0 && pm.learned.ok) {
            jw_obj(&w, "learned");
            jw_num(&w, "tilt_deg", (double)pm.learned.tilt_deg, 2);
            jw_num(&w, "tilt_sd_deg", (double)pm.learned.tilt_sd_deg, 2);
            jw_num(&w, "toward_deg", (double)pm.learned.azim_deg, 0);
            jw_num(&w, "length_m", (double)pm.learned.len_m, 3);
            jw_num(&w, "length_sd_m", (double)pm.learned.len_sd_m, 3);
            jw_num(&w, "ant_height_m", (double)pm.ant_height_m, 3);
            jw_bool(&w, "check_ant_height", pm.len_differs);
            jw_num(&w, "rms_m", (double)pm.learned.rms_m, 3);
            jw_end(&w, '}');
        }
        jw_obj(&w, "data");
        jw_int(&w, "episodes", pm.stats.n_ep);
        jw_int(&w, "episodes_boot", pm.ep_boot);
        jw_int(&w, "dropped", pm.ep_rejected);
        jw_num(&w, "twist_deg", (double)pm.stats.twist_deg, 0);
        if (pm.last_rms_m >= 0.0f) {
            jw_num(&w, "last_rms_m", (double)pm.last_rms_m, 3);
            jw_num(&w, "last_twist_deg", (double)pm.last_twist_deg, 0);
        }
        jw_end(&w, '}');
        jw_end(&w, '}');
    }

    jw_end(&w, '}');
    if (!jw_ok(&w)) {
        static const char k_err[] = "{\"error\":\"calib JSON too long\"}";
        web_set_doc(WEB_DOC_CALIB, k_err, sizeof k_err - 1);
        return;
    }
    web_set_doc(WEB_DOC_CALIB, buf, w.len);
}
#endif

#if APP_WEB_BUILT
static uint64_t s_reboot_at;

/* Live fields: the filter settings go to core 1, which re-initialises; the
 * heading gate and baseline direction are read from s_settings on core 0. */
static void apply_live(const settings_t *s)
{
    nav_params_t old_p, new_p;
    nav_params_from_settings(&old_p, &s_settings);
    nav_params_from_settings(&new_p, s);
    if (memcmp(&old_p, &new_p, sizeof old_p) != 0) {
        nav_set_params(&new_p);
    }
    imu_calib_set_psd(s->gyr_psd, s->acc_psd);
    pole_mount_set_height(s->ant_height_m);
    field_io_apply_settings(s);
    s_settings = *s;
}

/* A command from the web config page (config_web.h). */
static void handle_config_cmd(uint64_t now)
{
    config_cmd_t cmd;
    if (!config_web_take(&cmd)) {
        return;
    }
    char res[160];
    switch (cmd.kind) {
    case CFG_CMD_SAVE: {
        uint64_t t0 = time_us_64();
        int rc = settings_save(&cmd.settings);
        unsigned long ms = (unsigned long)((time_us_64() - t0) / 1000u);
        if (rc != 0) {
            snprintf(res, sizeof res, "save failed: flash error %d; nothing changed", rc);
            break;
        }
        apply_live(&cmd.settings);
        s_settings_from_flash = true;
        snprintf(res, sizeof res, "saved as record %lu in %lu ms%s", (unsigned long)settings_seq(), ms,
                 settings_reboot_fields_differ(&s_settings, &s_boot_settings)
                     ? "; reboot to apply the antenna / platform / Wi-Fi changes" : "");
        break;
    }
    case CFG_CMD_DEFAULTS: {
        int rc = settings_erase();
        if (rc != 0) {
            snprintf(res, sizeof res, "erase failed: flash error %d; nothing changed", rc);
            break;
        }
        settings_t d;
        settings_defaults(&d);
        apply_live(&d);
        s_settings_from_flash = false;
        snprintf(res, sizeof res, "build defaults restored%s",
                 settings_reboot_fields_differ(&s_settings, &s_boot_settings)
                     ? "; reboot to apply the antenna / platform / Wi-Fi changes" : "");
        break;
    }
    case CFG_CMD_REBOOT:
        s_reboot_at = now + 500000u; /* lets the reply and the next poll go out */
        snprintf(res, sizeof res, "rebooting");
        break;
    case CFG_CMD_RECONFIGURE_GNSS:
        um980_reconfigure(&s_gnss);
        snprintf(res, sizeof res, "GNSS receiver: full re-provision started");
        break;
    case CFG_CMD_RECONFIGURE_IMU:
        tm210_reconfigure(&s_imu);
        snprintf(res, sizeof res, "IMU: provisioning sequence started");
        break;
    case CFG_CMD_CALIB_RESET:
    case CFG_CMD_CALIB: {
        /* calib_reset is the factory reset of every group. */
        const bool all = cmd.kind == CFG_CMD_CALIB_RESET;
        int rc;
        if (!all && cmd.calib_group == POLE_MOUNT_GROUP) {
            /* The pole axis: its own record (pole_mount.h). */
            rc = pole_mount_action(cmd.calib_op, res, sizeof res);
        } else {
            rc = imu_calib_action(all ? IMU_CALIB_GROUPS : cmd.calib_group,
                                  all ? IMU_CALIB_OP_FACTORY : cmd.calib_op, res, sizeof res);
        }
        if (rc > 0) {
            nav_params_t p;
            nav_params_from_settings(&p, &s_settings);
            nav_set_params(&p);
        }
        publish_calib_json();
        break;
    }
    default:
        return;
    }
    printf("[config] %s\n", res);
    config_web_publish(&s_settings, &s_boot_settings, s_settings_from_flash, res);
}
#endif

int main(void)
{
    stdio_init_all();

    /* Give a terminal a moment to attach, so the provisioning log is seen. */
    absolute_time_t usb_until = make_timeout_time_ms(USB_WAIT_MS);
    while (!stdio_usb_connected() && !time_reached(usb_until)) {
        sleep_ms(10);
    }
    printf("\nOpenGINS demo: TM210 + UM980 + BMP581 -> INSLIB nav_suite\n");

    /* Settings from flash, or the build defaults. A record that fails the
     * range check (e.g. written by an older build) is not used. */
    s_settings_from_flash = settings_load(&s_settings);
    const char *bad = settings_validate(&s_settings);
    if (bad != NULL) {
        printf("[settings] saved record rejected (%s), using defaults\n", bad);
        settings_defaults(&s_settings);
        s_settings_from_flash = false;
    }
    s_boot_settings = s_settings;
    printf("[settings] %s", s_settings_from_flash ? "loaded from flash" : "build defaults");
    if (s_settings_from_flash && settings_loaded_version() != SETTINGS_VERSION) {
        printf(" (version %u record, newer fields at their defaults until the next save)", settings_loaded_version());
    }
    printf("; flow camera %.2f m, image top %.0f deg, lever %.2f %.2f %.2f m\n", (double)s_settings.flow_height_m,
           (double)s_settings.flow_cam_yaw_deg, (double)s_settings.flow_leverarm_m[0],
           (double)s_settings.flow_leverarm_m[1], (double)s_settings.flow_leverarm_m[2]);
    printf("[settings] antenna height over ground %.3f m%s\n", (double)s_settings.ant_height_m,
           settings_ant_height(&s_settings, settings_platform(&s_boot_settings)) > 0.0f ? " (output is the ground point)"
           : (s_settings.ant_height_m > 0.0f ? " (not used on this platform)" : " (output is the antenna)"));
    printf("[settings] sensors: baro %s  mag %s  flow %s  wheel %s\n", s_settings.sensor_baro ? "on" : "off",
           s_settings.sensor_mag ? "on" : "off", s_settings.sensor_flow ? "on" : "off",
           s_settings.sensor_wheel ? "on" : "off");

    /* Learned gyro calibration: this boot's correction, before nav_start(). */
    imu_calib_init(s_settings.gyr_psd, s_settings.acc_psd, settings_platform(&s_settings));
    {
        imu_calib_status_t cs;
        imu_calib_status(&cs);
        printf("[calib] %s (record %lu)", cs.boot_note, (unsigned long)cs.rec_seq);
        if (cs.applied) {
            printf(": b0 %.4f %.4f %.4f deg/s at %.0f C, k %.5f %.5f %.5f deg/s/C",
                   (double)cs.use.b0[0] * DEG, (double)cs.use.b0[1] * DEG,
                   (double)cs.use.b0[2] * DEG, (double)APP_CALIB_T_REF_C,
                   (double)cs.use.k[0] * DEG, (double)cs.use.k[1] * DEG,
                   (double)cs.use.k[2] * DEG);
        }
        printf("\n");
        printf("[calib] accelerometer: %s", cs.acc_boot_note);
        if (cs.acc_applied) {
            const float *q = cs.acc_use.p;
            printf(" (%lu poses): scale %.4f %.4f %.4f  mis %.3f %.3f %.3f deg  bias %.4f %.4f %.4f m/s2",
                   (unsigned long)cs.acc_poses, (double)q[ACAL_SX], (double)q[ACAL_SY],
                   (double)q[ACAL_SZ], (double)q[ACAL_MXY] * DEG, (double)q[ACAL_MXZ] * DEG,
                   (double)q[ACAL_MYZ] * DEG, (double)q[ACAL_BX], (double)q[ACAL_BY],
                   (double)q[ACAL_BZ]);
        }
        printf("\n");
        printf("[calib] magnetometer: %s\n", cs.mag_boot_note);
        printf("[calib] gyro matrix: %s", cs.gyr_m_boot_note);
        if (cs.gyr_m_applied) {
            const float *g = cs.gyr_m;
            printf(": %.5f %.5f %.5f / %.5f %.5f %.5f / %.5f %.5f %.5f", (double)g[0], (double)g[1],
                   (double)g[2], (double)g[3], (double)g[4], (double)g[5], (double)g[6], (double)g[7],
                   (double)g[8]);
        }
        printf("\n");
    }

    /* Pole axis (pole_mount.h): this boot's axis, before nav_start(). */
    {
        unsigned plat = settings_platform(&s_boot_settings);
        float h = settings_ant_height(&s_settings, plat);
        pole_mount_init(plat == APP_PLATFORM_POLE && h > 0.0f, s_settings.ant_height_m);
        pole_mount_status_t ps;
        pole_mount_status(&ps);
        printf("[calib] pole axis: %s (record %lu)", ps.note, (unsigned long)ps.rec_seq);
        if (ps.applied) {
            printf(": tilt %.2f deg toward %.0f deg in the IMU x/y", (double)ps.use_tilt_deg,
                   (double)ps.use_azim_deg);
        }
        printf("\n");
    }

#if APP_WEB_BUILT
#if APP_WEB_MODE == APP_WEB_SWITCH
    /* Production: the web-mode switch is read once, here. Pulled to GND =
     * Wi-Fi on for this boot; open (internal pull-up) = off, so the radio,
     * its current draw and its interrupts stay away from the filter. */
    gpio_init(PIN_WEB_MODE_SWITCH);
    gpio_set_dir(PIN_WEB_MODE_SWITCH, GPIO_IN);
    gpio_pull_up(PIN_WEB_MODE_SWITCH);
    sleep_ms(2); /* let the pull-up charge the switch wiring */
    s_web_on = !gpio_get(PIN_WEB_MODE_SWITCH);
    printf("[web] web-mode switch (GP%d) %s: Wi-Fi %s\n", PIN_WEB_MODE_SWITCH,
           s_web_on ? "to GND" : "open", s_web_on ? "on" : "off");
#else
    s_web_on = true;
#endif
    if (s_web_on) {
        /* Before the sensor UARTs start: loading the CYW43439 firmware blocks for
         * a few hundred ms, longer than the IMU ring holds. The join itself runs
         * in the background from web_poll(). */
        const web_config_t web_cfg = {
            .mode = s_settings.wifi_mode == APP_WIFI_AP  ? WEB_NET_AP :
                    s_settings.wifi_mode == APP_WIFI_STA ? WEB_NET_STA : WEB_NET_AUTO,
            .ssid = s_settings.sta_ssid, .password = s_settings.sta_password,
            .ap_ssid = APP_WIFI_AP_SSID, .ap_fallback_s = APP_WIFI_AP_FALLBACK_S,
            .hostname = APP_WEB_HOSTNAME,
        };
        uint64_t t_web = time_us_64();
        bool web_ok = web_init(&web_cfg);
        config_web_init();
        config_web_publish(&s_settings, &s_boot_settings, s_settings_from_flash, NULL);
        printf("[web] init %s in %lu ms\n", web_ok ? "done" : "failed",
               (unsigned long)((time_us_64() - t_web) / 1000u));
    }
#endif

    if (!uart_dma_init(&s_imu_port, PIN_TM210_UART, PIN_TM210_TX, PIN_TM210_RX, TM210_BAUD,
                       s_imu_rx, IMU_RX_BITS) ||
        !uart_dma_init(&s_gnss_port, PIN_UM980_UART, PIN_UM980_TX, PIN_UM980_RX, UM980_BAUD,
                       s_gnss_rx, GNSS_RX_BITS)) {
        panic("no free DMA channel");
    }

    const tm210_config_t imu_cfg = { .acc_is_gravity = APP_TM210_ACC_IS_GRAVITY, .verbose = true };
    /* The antenna-mode setting: build default, or the web page's saved value. */
    const bool dual_set = s_settings.antenna_dual != 0;
    printf("GNSS antenna mode setting: %s\n", dual_set ? "dual" : "single");
    printf("platform setting: %s (GNSS course as heading: %s)\n",
           settings_platform_name(settings_platform(&s_settings)),
           settings_platform(&s_settings) == APP_PLATFORM_TRACTOR ||
                   settings_platform(&s_settings) == APP_PLATFORM_FIXED_WING
               ? (dual_set ? "no, dual-antenna heading" : "yes, single antenna")
               : "no, not on this platform");
    /* The platform in effect since boot, as for the other platform choices. */
    s_paogi_on = settings_platform(&s_boot_settings) == APP_PLATFORM_TRACTOR;
    if (s_paogi_on) {
        printf("$PAOGI on USB at 10 Hz (USB command 'paogi off' stops it)\n");
    }

    const um980_config_t gnss_cfg = {
        .pps_pin = PIN_UM980_PPS, .dual_antenna = dual_set, .verbose = true,
    };
    tm210_init(&s_imu, &s_imu_port, &imu_cfg);
    um980_init(&s_gnss, &s_gnss_port, &gnss_cfg);
    /* Off in the settings: not started; switching it on later starts it
     * through the retry below. */
    s_baro_ok = s_settings.sensor_baro ? baro_start() : false;
    field_io_init();
    field_io_apply_settings(&s_settings);
    usb_log_init();

    nav_antenna_t ant_sent = dual_set ? NAV_ANT_DUAL : NAV_ANT_SINGLE;
    nav_params_t np;
    nav_params_from_settings(&np, &s_settings);
    nav_start(ant_sent, &np);

    uint64_t next_status = time_us_64() + STATUS_PERIOD_US;
    uint64_t next_baro_try = time_us_64() + BARO_RETRY_US;
    cpu_load_init(&s_load0, clock_get_hz(clk_sys));
    die_temp_init();
    s_die_c = die_temp_read();
    bool worked = false; /* this pass did work (cpu_load.h) */

    for (;;) {
        cpu_load_pass(&s_load0, worked);
        worked = false;

        /* IMU: every sample goes to core 1. */
        tm210_sample_t is;
        while (tm210_poll(&s_imu, &is)) {
            nav_imu_t n;
            n.t_us = is.t_us;
            n.dt_s = is.dt_s;
            for (int i = 0; i < 3; i++) {
                n.acc[i] = is.acc[i];
                n.gyr[i] = is.gyr[i];
                n.mag[i] = is.mag[i];
            }
            n.mag_new = is.mag_new;
            imu_calib_push(&is, tm210_stats(&s_imu)); /* raw, before the correction */
            imu_calib_correct(n.gyr, tm210_stats(&s_imu));
            nav_push_imu(&n);
            field_io_imu(&n);
            usb_log_imu(&n, is.gap);
            paogi_gyro(n.gyr);
            noise_push(&is);
            s_last_imu = is;
            worked = true;
        }

        /* GNSS: epochs flow once the receiver is configured. */
        um980_nav_t gn;
        um980_heading_t gh;
        for (;;) {
            bool have_nav = um980_poll(&s_gnss, &gn);
            if (um980_take_heading(&s_gnss, &gh)) {
                forward_heading(&gh);
                usb_log_heading(&gh);
                s_last_hdg = gh;
                s_have_hdg = true;
                worked = true;
            }
            if (!have_nav) {
                break;
            }
            forward_gnss(&gn);
            usb_log_gnss(&gn);
            field_io_gnss(&gn);
            s_last_nav = gn;
            s_have_nav = true;
            if (gn.pos_usable) {
                s_undulation_m = gn.undulation_m;
                s_have_undulation = true;
            }
            worked = true;
        }
        /* Once the receiver is identified, the filter follows the effective
         * mode. That differs from the setting only when dual is set and a
         * UM980 is fitted. */
        if (um980_model(&s_gnss) != 0) {
            nav_antenna_t ant = um980_dual(&s_gnss) ? NAV_ANT_DUAL : NAV_ANT_SINGLE;
            if (ant != ant_sent) {
                ant_sent = ant;
                nav_set_antenna_mode(ant);
            }
        }

        /* Barometer. */
        uint64_t now = time_us_64();

        /* Gyro calibration: identity check and saving. Another TM210 than
         * the record's: restart the filter without the correction. */
        if (imu_calib_poll(now, &s_imu)) {
            nav_params_t p;
            nav_params_from_settings(&p, &s_settings);
            nav_set_params(&p);
        }

        if (!s_settings.sensor_baro) {
            /* Switched off: not polled (a running BMP581 keeps sampling on
             * its own, unread). */
        } else if (s_baro_ok) {
            bmp581_sample_t bs;
            if (bmp581_poll(&s_baro, &bs)) {
                nav_baro_t b = { .t_us = bs.t_us, .pressure_pa = bs.pressure_pa };
                nav_push_baro(&b);
                usb_log_baro(bs.t_us, bs.pressure_pa, bs.temperature_c);
                s_last_baro = bs;
                worked = true;
            } else if (bmp581_stuck(&s_baro, now) && now >= next_baro_try) {
                /* The first attempt right away, then one per BARO_RETRY_US. */
                int rc = bmp581_reinit(&s_baro);
                printf("[bmp581] reads failing or no data: re-init %s\n",
                       rc == BMP581_OK ? "ok" : bmp581_strerror(rc));
                next_baro_try = time_us_64() + BARO_RETRY_US;
            }
        } else if (now >= next_baro_try) {
            s_baro_ok = baro_start();
            next_baro_try = now + BARO_RETRY_US;
        }

        /* Wheel-speed edges and the flow link (field_io.h), then the USB
         * log: the filter output when core 1 published, commands, sending. */
        worked |= field_io_poll(now);
        if (usb_log_on()) {
            static uint32_t s_nav_seq;
            uint32_t seq = nav_output_seq();
            if (seq != s_nav_seq) {
                s_nav_seq = seq;
                nav_output_t o;
                nav_get_output(&o);
                usb_log_nav(&o);
            }
        }
        worked |= usb_log_poll(now);
        if (s_paogi_on) {
            worked |= paogi_poll(now);
        }

        /* Survey pole: episode records for the pole-axis fit. */
        {
            pole_episode_t ep;
            while (nav_pop_pole_episode(&ep)) {
                pole_mount_add(&ep);
                worked = true;
            }
            pole_mount_poll(now);
        }

        /* INSLIB messages from core 1. */
        char msg[200]; /* LOG_MSG_LEN in nav_core1.c */
        while (nav_pop_log(msg, sizeof msg)) {
            printf("[inslib] %s\n", msg);
            worked = true;
        }

#if APP_WEB_BUILT
        if (s_web_on) {
            web_poll(now);
            handle_config_cmd(now);
        }
        if (s_reboot_at != 0 && now >= s_reboot_at) {
            printf("[config] reboot\n");
            watchdog_reboot(0, 0, 0);
            for (;;) {
                tight_loop_contents();
            }
        }
#endif

        if (now >= next_status) {
            static uint32_t s_baro_prev;
            uint32_t baro_hz = bmp581_stats(&s_baro)->samples - s_baro_prev;
            s_baro_prev = bmp581_stats(&s_baro)->samples;

            cpu_load_take(&s_load0, now, &s_load0_res);
            s_die_c = die_temp_read();
            worked = true;
            nav_queue_drops_settle();
            noise_roll();
            mag_roll();
            field_io_roll(now);
            print_status(now, baro_hz);
#if APP_WEB_BUILT
            if (s_web_on) {
                publish_status_json(now, baro_hz);
                publish_calib_json();
            }
#endif
            next_status += STATUS_PERIOD_US;
        }
    }
}
