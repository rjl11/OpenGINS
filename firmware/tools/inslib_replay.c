/* Replay a field log (field_log.py record) through INSLIB nav_suite: the
 * logged IMU, magnetometer and barometer, with simulated GNSS: RTK fixed for
 * the whole run at a constant truth position plus white noise, on every
 * logged GNSS epoch (20 Hz) at its validity and arrival times. The truth is
 * the median of the logged RTK fixed epochs (or --truth).
 *
 * Mirrors firmware/app/nav_core1.c (filter_init, attach_gnss,
 * attach_queued) with the firmware defaults (app_config.h, settings.c).
 * Not modelled: the pole aiding (pole_aid), the IMU calibration from flash
 * (the logged gyro is already the corrected one), the 50 ms stall epochs.
 *
 * Built with a host gcc from the repo root, the command in BUILDING.md
 * ("Log tools"); the INSLIB sources are the firmware's
 * (firmware/drivers/inslib/CMakeLists.txt).
 *
 * usage: inslib_replay LOG.bin OUT.csv [options]
 *   --height gnss|baro   web setting height_src (default gnss, the pole
 *                        default: INS on the GNSS height; baro: INS on the
 *                        barometric height)
 *   --sd-hor M --sd-ver M   simulated fix noise (default 0.025, 0.05)
 *   --truth LAT LON H_ELL   truth position [deg, deg, m]
 *   --seed N
 *   --offset-interval S --offset-inflation F --offset-rw M
 *   --offset-local-sd M  local_gnss offset filter: fusion interval,
 *                        sigma inflation, random walk, barometer sigma per
 *                        pair (default: the firmware's APP_HEIGHT_OFFSET_*;
 *                        0 = INSLIB default)
 * Output: 10 Hz rows; gps_s from the logged GNSS epochs. Plot with
 * inslib_replay_plot.py. About 6 min for 7 h of log on a 6-core ARM
 * board. */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "log.h"
#include "nav_suite.h"

#define DEG (180.0 / M_PI)

/* The firmware defaults (settings.c takes these as the web settings'
   defaults), so a change there carries over to the replay. */
#include "app_config.h"
static float s_off_interval = -1, s_off_inflation = -1, s_off_rw = -1, s_off_local_sd = -1; /* -1 = mode default */

/* simulated RTK fixed */
static double s_sd_hor = 0.025, s_sd_ver = 0.05;
#define SIM_VEL_SD 0.01

enum { R_START = 1, R_IMU = 3, R_GNSS = 4, R_BARO = 6 };

static uint16_t crc16(const uint8_t *d, size_t n)
{
    uint16_t c = 0xFFFF;
    for (size_t i = 0; i < n; i++) {
        c ^= (uint16_t)d[i] << 8;
        for (int k = 0; k < 8; k++) c = (c & 0x8000) ? (uint16_t)((c << 1) ^ 0x1021) : (uint16_t)(c << 1);
    }
    return c;
}

/* Calls fn(type, payload, len) for each good record. */
typedef void (*rec_fn)(int, const uint8_t *, int);
static long walk(const char *path, rec_fn fn)
{
    FILE *f = fopen(path, "rb");
    if (!f) { perror(path); exit(1); }
    static uint8_t buf[1 << 22];
    size_t have = 0, i;
    long crc_err = 0;
    for (;;) {
        size_t got = fread(buf + have, 1, sizeof buf - have, f);
        have += got;
        i = 0;
        while (i + 4 <= have) {
            if (buf[i] != 0xA7 || buf[i + 1] != 0x4C) { i++; continue; }
            int n = buf[i + 3];
            if (i + 6 + (size_t)n > have) break;
            uint16_t c = (uint16_t)(buf[i + 4 + n] | (buf[i + 5 + n] << 8));
            if (crc16(buf + i + 2, (size_t)n + 2) != c) { crc_err++; i++; continue; }
            fn(buf[i + 2], buf + i + 4, n);
            i += 6 + (size_t)n;
        }
        memmove(buf, buf + i, have - i);
        have -= i;
        if (got == 0) break;
    }
    fclose(f);
    return crc_err;
}

#define RD(T, p, off) ({ T _v; memcpy(&_v, (p) + (off), sizeof _v); _v; })

/* ---- pass 1: truth from the logged fixed epochs ---- */
static double *s_lat, *s_lon, *s_h, *s_und;
static size_t s_nfix, s_capfix;
static void pass1(int type, const uint8_t *p, int n)
{
    if (type != R_GNSS || n < 92 || p[22] != 50) return;
    if (s_nfix == s_capfix) {
        s_capfix = s_capfix ? s_capfix * 2 : 65536;
        s_lat = realloc(s_lat, s_capfix * sizeof *s_lat);
        s_lon = realloc(s_lon, s_capfix * sizeof *s_lon);
        s_h = realloc(s_h, s_capfix * sizeof *s_h);
        s_und = realloc(s_und, s_capfix * sizeof *s_und);
    }
    s_lat[s_nfix] = RD(double, p, 26);
    s_lon[s_nfix] = RD(double, p, 34);
    s_h[s_nfix] = RD(double, p, 42);
    s_und[s_nfix] = RD(float, p, 50);
    s_nfix++;
}
static int cmpd(const void *a, const void *b)
{
    double x = *(const double *)a, y = *(const double *)b;
    return (x > y) - (x < y);
}
static double median(double *v, size_t n) { qsort(v, n, sizeof *v, cmpd); return v[n / 2]; }

/* ---- pass 2: replay ---- */
typedef struct { uint64_t t_valid, t_arrival; uint16_t week; uint32_t tow; } gnss_ev_t;
typedef struct { uint64_t t; float p, tc; } baro_ev_t;
#define QN 64
static gnss_ev_t s_gq[QN]; static int s_gh, s_gt;
static baro_ev_t s_bq[QN]; static int s_bh, s_bt;

static nav_suite_t s_suite;
static int s_baro_mode;
static double s_truth_lat, s_truth_lon, s_truth_h_ell;
static FILE *s_out;
static uint64_t s_next_out;
static float s_last_p = NAN, s_last_tc = NAN;
static uint64_t s_last_imu_t;
static bool s_mag_model_set;
static long s_n_imu, s_n_gnss, s_n_baro, s_n_baro_late;
static unsigned s_rng = 12345u;
static uint64_t s_gps_t_us; static double s_gps_s; /* last GNSS epoch: Pico time -> GPS time */

static double urand(void)
{
    s_rng = s_rng * 1664525u + 1013904223u;
    return ((s_rng >> 8) + 0.5) / 16777216.0;
}
static double gauss(void)
{
    static int have; static double spare;
    if (have) { have = 0; return spare; }
    double u = urand(), v = urand(), r = sqrt(-2.0 * log(u));
    spare = r * sin(2 * M_PI * v); have = 1;
    return r * cos(2 * M_PI * v);
}

static void log_sink(int level, const char *file, int line, const char *fmt, va_list args)
{
    (void)file; (void)line;
    if (level > LOG_LEVEL_WARN) return;
    double t = s_last_imu_t * 1e-6;
    fprintf(stderr, "[%9.3f] ", t);
    vfprintf(stderr, fmt, args);
    fputc('\n', stderr);
}

static void filter_init(uint64_t t0)
{
    ins_init_t init; ins_options_t opt;
    memset(&s_suite, 0, sizeof s_suite);
    memset(&init, 0, sizeof init);
    memset(&opt, 0, sizeof opt);
    init.time = (ins_time_us_t)t0;
    init.gyr_bias_pred_stddev_rps_sqrts = APP_GYR_BIAS_RW;
    init.acc_bias_pred_stddev_mps2_sqrts = APP_ACC_BIAS_RW;
    opt.auto_init = true;
    opt.automotive_mode = false;      /* pole: no course yaw */
    opt.automotive_min_speed_mps = 0.0f;
    opt.max_deadreckoning_sec = APP_MAX_DEADRECKONING_SEC;
    opt.gnss_pos_decimation = APP_GNSS_POS_DECIMATION;
    opt.baro_height_disable = !s_baro_mode;
    opt.estimate_mag_bias = APP_ESTIMATE_MAG_BIAS;
    opt.gnss_pos_stddev_floor_hor_m = APP_GNSS_FLOOR_POS_HOR_M;
    opt.gnss_pos_stddev_floor_ver_m = APP_GNSS_FLOOR_POS_VER_M;
    opt.gnss_vel_stddev_floor_hor_mps = APP_GNSS_FLOOR_VEL_HOR_MPS;
    opt.gnss_vel_stddev_floor_ver_mps = APP_GNSS_FLOOR_VEL_VER_MPS;
    opt.auto_zupt_static_gyr_stddev_rps = APP_STILL_GYR_STDDEV_DPS * 0.017453293f;
    nav_suite_init(&s_suite, &init, &opt);
    /* the firmware's APP_HEIGHT_OFFSET_* defaults, with either height source */
    float iv = APP_HEIGHT_OFFSET_INTERVAL_S;
    float inf = APP_HEIGHT_OFFSET_INFLATION;
    float rw = APP_HEIGHT_OFFSET_RW_MPS;
    float lsd = s_baro_mode ? APP_HEIGHT_OFFSET_BARO_SD_M : 0.0f; /* as nav_core1.c */
    if (s_off_interval >= 0) iv = s_off_interval;
    if (s_off_inflation >= 0) inf = s_off_inflation;
    if (s_off_rw >= 0) rw = s_off_rw;
    s_suite.local_gnss_cfg.min_update_interval_sec = iv;
    s_suite.local_gnss_cfg.stddev_inflation_factor = inf;
    s_suite.local_gnss_cfg.rw_stddev_mps = rw;
    if (s_off_local_sd >= 0) lsd = s_off_local_sd;
    s_suite.local_gnss_cfg.local_stddev_m = lsd;
}

static void set_diag(float *d, float a, float b, float c) { d[0] = a; d[4] = b; d[8] = c; }

static void attach_gnss(ins_measurements_t *m, const gnss_ev_t *g)
{
    int64_t age = (int64_t)m->timestamp - (int64_t)g->t_valid;
    if (age < 0) age = 0;
    if (age >= (int64_t)INS_MAX_DELAY_MS * 1000) return;
    /* simulated fix: truth + noise, NED offsets turned into lat/lon */
    const double a = 6378137.0, e2 = 6.69437999014e-3;
    double sl = sin(s_truth_lat), w = sqrt(1 - e2 * sl * sl);
    double rn = a * (1 - e2) / (w * w * w), re = a / w;
    double dn = s_sd_hor * gauss(), de = s_sd_hor * gauss(), dh = s_sd_ver * gauss();
    m->gnss_pos.llh[0] = s_truth_lat + dn / (rn + s_truth_h_ell);
    m->gnss_pos.llh[1] = s_truth_lon + de / ((re + s_truth_h_ell) * cos(s_truth_lat));
    m->gnss_pos.llh[2] = s_truth_h_ell + dh;
    set_diag(m->gnss_pos.Qll_ned, (float)(s_sd_hor * s_sd_hor), (float)(s_sd_hor * s_sd_hor),
             (float)(s_sd_ver * s_sd_ver));
    m->gnss_pos.is_valid = true;
    for (int i = 0; i < 3; i++) m->gnss_vel.vel_ned[i] = (float)(SIM_VEL_SD * gauss());
    set_diag(m->gnss_vel.Qll_ned, (float)(SIM_VEL_SD * SIM_VEL_SD), (float)(SIM_VEL_SD * SIM_VEL_SD),
             (float)(SIM_VEL_SD * SIM_VEL_SD));
    m->gnss_vel.is_valid = true;
    m->gps_week = g->week;
    m->gps_itow_ms = (int)g->tow;
    m->gnss_delay_ms = (int)(age / 1000);
    m->gnss_delay_us = (int32_t)(age % 1000);
    s_n_gnss++;
    if (!s_mag_model_set) {
        float year = 1980.0f + (5.0f + (float)g->week * 7.0f) / 365.25f;
        nav_suite_set_magnetic_model_position(&s_suite, s_truth_lat, s_truth_lon, year);
        s_mag_model_set = true;
    }
}

static void output(uint64_t t)
{
    float r = NAN, p = NAN, y = NAN, h_ell = NAN, h_loc = NAN, hb = NAN, vb = NAN, isa = NAN, off = NAN, off_sd = NAN;
    double llh[3] = { NAN, NAN, NAN };
    float ab[3] = { NAN, NAN, NAN };
    nav_suite_get_rpy(&s_suite, &r, &p, &y);
    bool ready = ins_is_ready(&s_suite.ins);
    if (ready) { ins_get_latlonh(&s_suite.ins, llh); ins_get_bias_acc(&s_suite.ins, ab); }
    if (!nav_suite_get_height_ellipsoid(&s_suite, &h_ell)) h_ell = NAN;
    if (!nav_suite_get_height(&s_suite, &h_loc)) h_loc = NAN;
    if (!nav_suite_get_baro_alt(&s_suite, &hb, &vb)) { hb = NAN; vb = NAN; }
    if (!baro_alt_get_isa_altitude(&s_suite.baro_alt, &isa)) isa = NAN;
    if (!s_suite.local_gnss.is_initialized || !local_gnss_alt_get(&s_suite.local_gnss, &off, &off_sd)) {
        off = NAN; off_sd = NAN;
    }
    double gps = s_gps_t_us ? s_gps_s + ((double)t - (double)s_gps_t_us) * 1e-6 : NAN;
    fprintf(s_out, "%.3f,%.3f,%d,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.3f,%.4f,%.3f,%.3f,%.3f,%.5f,%.4f,%.3f\n",
            t * 1e-6, gps, (int)nav_suite_get_mode(&s_suite), ready, s_truth_h_ell, h_ell, llh[2], h_loc, hb, vb, isa,
            off, off_sd, s_suite.datum_offset_valid ? s_suite.datum_offset_m : NAN, s_last_p, s_last_tc,
            r * DEG, p * DEG, ab[2], s_suite.baro_alt.is_initialized ? 1.0f : 0.0f, y * DEG);
}

static void pass2(int type, const uint8_t *p, int n)
{
    if (type == R_START) {
        if (s_last_imu_t) fprintf(stderr, "second START record: Pico rebooted, filter restarted\n");
        s_gh = s_gt = s_bh = s_bt = 0;
        s_last_imu_t = 0;
        s_next_out = 0;
        s_gps_t_us = 0;
        return;
    }
    if (type == R_GNSS && n >= 92) {
        gnss_ev_t g = { RD(uint64_t, p, 0), RD(uint64_t, p, 8), RD(uint16_t, p, 16), RD(uint32_t, p, 18) };
        s_gps_t_us = g.t_valid;
        s_gps_s = g.week * 604800.0 + g.tow * 1e-3;
        if ((s_gt + 1) % QN != s_gh) { s_gq[s_gt] = g; s_gt = (s_gt + 1) % QN; }
        return;
    }
    if (type == R_BARO && n >= 16) {
        baro_ev_t b = { RD(uint64_t, p, 0), RD(float, p, 8), RD(float, p, 12) };
        if ((s_bt + 1) % QN != s_bh) { s_bq[s_bt] = b; s_bt = (s_bt + 1) % QN; }
        return;
    }
    if (type != R_IMU || n < 49) return;

    uint64_t t = RD(uint64_t, p, 0);
    if (s_last_imu_t == 0) {
        filter_init(t);
        s_mag_model_set = false;
        s_next_out = t;
    }
    ins_measurements_t m;
    memset(&m, 0, sizeof m);
    m.timestamp = (ins_time_us_t)t;
    m.strapdown_dt_sec = RD(float, p, 8);
    for (int i = 0; i < 3; i++) {
        m.gyr.data[i] = RD(float, p, 12 + 4 * i);
        m.acc.data[i] = RD(float, p, 24 + 4 * i);
        m.acc.Qll_diag[i] = APP_ACC_PSD;
        m.gyr.Qll_diag[i] = APP_GYR_PSD;
    }
    m.acc.is_valid = m.gyr.is_valid = true;
    if (p[48] & 1) {
        for (int i = 0; i < 3; i++) m.mag.data[i] = RD(float, p, 36 + 4 * i);
        m.mag.is_valid = true;
    }
    /* GNSS reaches core 1 when it has arrived (t_arrival); one per epoch, as
       the firmware's queue */
    if (s_gh != s_gt && s_gq[s_gh].t_arrival <= t) {
        attach_gnss(&m, &s_gq[s_gh]);
        s_gh = (s_gh + 1) % QN;
    }
    if (s_bh != s_bt && s_bq[s_bh].t <= t) {
        baro_ev_t b = s_bq[s_bh];
        s_bh = (s_bh + 1) % QN;
        s_last_p = b.p; s_last_tc = b.tc;
        if ((int64_t)t - (int64_t)b.t > APP_BARO_MAX_AGE_US) {
            s_n_baro_late++;
        } else {
            m.baro.pressure_pa = b.p;
            m.baro.is_valid = true;
            s_n_baro++;
        }
    }
    nav_suite_update(&s_suite, &m);
    s_last_imu_t = t;
    s_n_imu++;
    if (t >= s_next_out) {
        output(t);
        s_next_out += 100000;
        if (s_next_out < t) s_next_out = t + 100000;
    }
}

static void usage(const char *a0)
{
    fprintf(stderr, "usage: %s LOG.bin OUT.csv [--height gnss|baro] [--sd-hor M] [--sd-ver M] [--seed N]\n"
                    "       [--truth LAT LON H_ELL] [--offset-interval S] [--offset-inflation F] [--offset-rw M]\n"
                    "       [--offset-local-sd M]\n",
            a0);
    exit(2);
}

int main(int argc, char **argv)
{
    double tr_lat = NAN, tr_lon = NAN, tr_h = NAN;
    if (argc < 3) usage(argv[0]);
    for (int i = 3; i < argc; i++) {
        const char *o = argv[i];
#define ARG() (i + 1 < argc ? argv[++i] : (usage(argv[0]), ""))
        if (!strcmp(o, "--height")) { const char *v = ARG(); s_baro_mode = !strcmp(v, "baro"); if (!s_baro_mode && strcmp(v, "gnss")) usage(argv[0]); }
        else if (!strcmp(o, "--sd-hor")) s_sd_hor = atof(ARG());
        else if (!strcmp(o, "--sd-ver")) s_sd_ver = atof(ARG());
        else if (!strcmp(o, "--seed")) s_rng = (unsigned)strtoul(ARG(), NULL, 0);
        else if (!strcmp(o, "--truth")) { tr_lat = atof(ARG()); tr_lon = atof(ARG()); tr_h = atof(ARG()); }
        else if (!strcmp(o, "--offset-interval")) s_off_interval = (float)atof(ARG());
        else if (!strcmp(o, "--offset-inflation")) s_off_inflation = (float)atof(ARG());
        else if (!strcmp(o, "--offset-rw")) s_off_rw = (float)atof(ARG());
        else if (!strcmp(o, "--offset-local-sd")) s_off_local_sd = (float)atof(ARG());
        else usage(argv[0]);
    }
    log_set_sink(log_sink);

    if (isnan(tr_h)) {
        long ce = walk(argv[1], pass1);
        if (s_nfix == 0) { fprintf(stderr, "no RTK fixed epochs for the truth: give --truth\n"); return 1; }
        tr_lat = median(s_lat, s_nfix);
        tr_lon = median(s_lon, s_nfix);
        double h_msl = median(s_h, s_nfix), und = median(s_und, s_nfix);
        tr_h = h_msl + und;
        fprintf(stderr, "truth from %zu fixed epochs: h_msl %.3f und %.3f (crc errors %ld)\n", s_nfix, h_msl, und, ce);
    }
    s_truth_lat = tr_lat / DEG;
    s_truth_lon = tr_lon / DEG;
    s_truth_h_ell = tr_h;
    fprintf(stderr, "truth lat %.8f lon %.8f h_ell %.3f; height %s; sim sd %.3f/%.3f m\n", tr_lat, tr_lon, tr_h,
            s_baro_mode ? "baro" : "gnss", s_sd_hor, s_sd_ver);

    s_out = fopen(argv[2], "w");
    if (!s_out) { perror(argv[2]); return 1; }
    fprintf(s_out, "t_s,gps_s,mode,ins_ready,truth_h_ell,h_ell,ins_h,height_m,baro_alt_h,baro_alt_v,isa_alt,"
                   "offset_m,offset_sd,datum_offset,p_pa,temp_c,roll_deg,pitch_deg,acc_bias_z,baro_init,yaw_deg\n");
    walk(argv[1], pass2);
    fclose(s_out);
    fprintf(stderr, "imu %ld, gnss %ld, baro %ld (late %ld), n_imu_loss %u\n", s_n_imu, s_n_gnss, s_n_baro,
            s_n_baro_late, s_suite.n_imu_loss);
    return 0;
}
