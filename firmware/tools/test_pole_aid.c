/* Host test for firmware/app/pole_aid.c. Build and run from the repo root:
 *
 *   gcc -O2 -Wall -Wextra -I firmware/app firmware/tools/test_pole_aid.c \
 *       firmware/app/pole_aid.c -lm -o temp/test_pole_aid
 *   temp/test_pole_aid
 *
 * The IMU samples come from an exact rigid-body motion: the attitude R(t) and
 * the tip position are given, the IMU sits at tip - R l, and the specific
 * force and the rate are differentiated numerically from that.
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "pole_aid.h"

static int s_fail;

#define CHECK(c, ...) do { if (!(c)) { printf("FAIL %s:%d: ", __FILE__, __LINE__); \
                                       printf(__VA_ARGS__); printf("\n"); s_fail++; } } while (0)

#define D2R (3.14159265358979323846 / 180.0)
#define G 9.81
#define DT (1.0 / 800.0)

typedef struct {
    double yaw0;        /* true yaw */
    double roll_amp, pitch_amp, f1, f2;
    double tip_v[3];    /* tip velocity (0 = planted) */
    double bounce_amp, bounce_f; /* vertical bounce of the whole pole */
} motion_t;

static const float k_tip_b[3] = { 0.0f, 0.0f, 1.9f };  /* IMU to tip, body FRD */
static const float k_ant_b[3] = { 0.0f, 0.0f, -0.1f }; /* IMU to antenna */

/* R body to NED, row-major in d[3][3], ZYX. */
static void attitude(const motion_t *m, double t, double R[3][3])
{
    double r = m->roll_amp * sin(2 * M_PI * m->f1 * t);
    double p = m->pitch_amp * sin(2 * M_PI * m->f2 * t + 0.7);
    double y = m->yaw0 + 0.05 * sin(2 * M_PI * 0.13 * t);
    double cr = cos(r), sr = sin(r), cp = cos(p), sp = sin(p), cy = cos(y), sy = sin(y);
    R[0][0] = cp * cy; R[0][1] = sr * sp * cy - cr * sy; R[0][2] = cr * sp * cy + sr * sy;
    R[1][0] = cp * sy; R[1][1] = sr * sp * sy + cr * cy; R[1][2] = cr * sp * sy - sr * cy;
    R[2][0] = -sp;     R[2][1] = sr * cp;                R[2][2] = cr * cp;
}

static void tip_pos(const motion_t *m, double t, double p[3])
{
    for (int i = 0; i < 3; i++) p[i] = m->tip_v[i] * t;
    p[2] += m->bounce_amp * sin(2 * M_PI * m->bounce_f * t);
}

/* Position of a body point (offset o_b from the IMU) at t. */
static void point_pos(const motion_t *m, double t, const float o_b[3], double out[3])
{
    double R[3][3], tip[3];
    attitude(m, t, R);
    tip_pos(m, t, tip);
    for (int i = 0; i < 3; i++) {
        double imu = tip[i];
        for (int j = 0; j < 3; j++) imu -= R[i][j] * k_tip_b[j];
        out[i] = imu;
        for (int j = 0; j < 3; j++) out[i] += R[i][j] * o_b[j];
    }
}

static const float k_zero[3] = { 0, 0, 0 };

/* Exact IMU sample at t: specific force and rate (body), IMU velocity (NED),
 * R column-major. */
static void sample(const motion_t *m, double t, float f[3], float w[3], double v[3], float Rc[9])
{
    const double h = 1e-4;
    double pm[3], p0[3], pp[3], acc[3];
    point_pos(m, t - h, k_zero, pm);
    point_pos(m, t, k_zero, p0);
    point_pos(m, t + h, k_zero, pp);
    for (int i = 0; i < 3; i++) {
        acc[i] = (pp[i] - 2 * p0[i] + pm[i]) / (h * h);
        v[i] = (pp[i] - pm[i]) / (2 * h);
    }
    double R[3][3], Rm[3][3], Rp[3][3];
    attitude(m, t, R);
    attitude(m, t - h, Rm);
    attitude(m, t + h, Rp);
    double fn[3] = { acc[0], acc[1], acc[2] - G };
    for (int i = 0; i < 3; i++) {
        f[i] = 0.0f;
        for (int j = 0; j < 3; j++) f[i] += (float)(R[j][i] * fn[j]);
    }
    /* [w]x = R' dR/dt */
    double W[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            W[i][j] = 0.0;
            for (int k = 0; k < 3; k++) W[i][j] += R[k][i] * (Rp[k][j] - Rm[k][j]) / (2 * h);
        }
    w[0] = (float)W[2][1];
    w[1] = (float)W[0][2];
    w[2] = (float)W[1][0];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) Rc[i + 3 * j] = (float)R[i][j];
}

static void tip_vel(const float Rc[9], const float w[3], const double v[3], float vt[3])
{
    float wl[3] = { w[1] * k_tip_b[2] - w[2] * k_tip_b[1], w[2] * k_tip_b[0] - w[0] * k_tip_b[2],
                    w[0] * k_tip_b[1] - w[1] * k_tip_b[0] };
    for (int i = 0; i < 3; i++)
        vt[i] = (float)v[i] + Rc[i] * wl[0] + Rc[i + 3] * wl[1] + Rc[i + 6] * wl[2];
}

/* Runs the detector over [t0, t1); returns the fraction of samples planted
 * in the last half. */
static double run_detector(pole_aid_t *p, const motion_t *m, double t0, double t1, bool give_v)
{
    long n = 0, planted = 0;
    for (double t = t0; t < t1; t += DT) {
        float f[3], w[3], Rc[9], vt[3];
        double v[3];
        sample(m, t, f, w, v, Rc);
        tip_vel(Rc, w, v, vt);
        bool pl = pole_aid_imu(p, (float)DT, f, w, give_v ? vt : NULL);
        if (t > (t0 + t1) / 2) {
            n++;
            planted += pl;
        }
    }
    return n ? (double)planted / n : 0.0;
}

static void test_detector(void)
{
    pole_aid_cfg_t cfg;
    pole_aid_default_cfg(&cfg);
    pole_aid_t p;

    motion_t sway = { .yaw0 = 0.3, .roll_amp = 4 * D2R, .pitch_amp = 3 * D2R, .f1 = 0.6, .f2 = 0.45 };
    pole_aid_reset(&p, &cfg, true, k_tip_b, k_ant_b);
    double fr = run_detector(&p, &sway, 0.0, 3.0, true);
    CHECK(fr > 0.99, "sway around a planted tip: planted %.3f", fr);
    CHECK(p.st.tip_err_ms2 < 0.05f, "tip force error %.4f", (double)p.st.tip_err_ms2);

    motion_t big = sway;
    big.roll_amp = 15 * D2R;
    big.f1 = 1.0;
    pole_aid_reset(&p, &cfg, true, k_tip_b, k_ant_b);
    fr = run_detector(&p, &big, 0.0, 3.0, true);
    CHECK(fr > 0.99, "deliberate swing 15 deg 1 Hz: planted %.3f", fr);

    motion_t carry = sway;
    carry.tip_v[0] = 1.0;
    pole_aid_reset(&p, &cfg, true, k_tip_b, k_ant_b);
    fr = run_detector(&p, &carry, 0.0, 3.0, true);
    CHECK(fr == 0.0, "carried at 1 m/s: planted %.3f", fr);

    motion_t walk = sway;
    walk.bounce_amp = 0.03;
    walk.bounce_f = 2.0;
    pole_aid_reset(&p, &cfg, true, k_tip_b, k_ant_b);
    /* No INS velocity either: the force test alone must reject it, and
     * without a velocity nothing is planted anyway. */
    fr = run_detector(&p, &walk, 0.0, 3.0, true);
    CHECK(fr == 0.0, "walking bounce: planted %.3f", fr);
    pole_aid_reset(&p, &cfg, true, k_tip_b, k_ant_b);
    fr = run_detector(&p, &sway, 0.0, 3.0, false);
    CHECK(fr == 0.0, "no INS velocity: planted %.3f", fr);

    pole_aid_reset(&p, &cfg, false, k_tip_b, k_ant_b);
    fr = run_detector(&p, &sway, 0.0, 3.0, true);
    CHECK(fr == 0.0, "inactive: planted %.3f", fr);
}

/* Small deterministic noise, roughly uniform in [-a, a]. */
static double noise(unsigned *s, double a)
{
    *s = *s * 1103515245u + 12345u;
    return ((double)((*s >> 8) & 0xFFFF) / 32768.0 - 1.0) * a;
}

/* Planted sway for dur s with the INS yaw off by err; GNSS at 20 Hz with
 * noise; returns whether a fit was offered, and dpsi / sd. */
static bool run_fit(double err, double tip_drift, double dur, float *dpsi, float *sd, pole_aid_t *p)
{
    pole_aid_cfg_t cfg;
    pole_aid_default_cfg(&cfg);
    pole_aid_reset(p, &cfg, true, k_tip_b, k_ant_b);
    motion_t m = { .yaw0 = 1.0, .roll_amp = 4 * D2R, .pitch_amp = 3 * D2R, .f1 = 0.6, .f2 = 0.45 };
    const double lat0 = 58.09 * D2R, lon0 = 12.92 * D2R, Re = 6371000.0;
    unsigned seed = 12345;
    double next_gnss = 0.0;
    double ce = cos(-err), se = sin(-err);
    for (double t = 0.0; t < dur; t += DT) {
        float f[3], w[3], Rc[9], vt[3], Ri[9];
        double v[3];
        sample(&m, t, f, w, v, Rc);
        tip_vel(Rc, w, v, vt);
        pole_aid_imu(p, (float)DT, f, w, vt);
        /* INS attitude: the true one turned by -err about down. */
        for (int j = 0; j < 3; j++) {
            Ri[0 + 3 * j] = (float)(ce * Rc[0 + 3 * j] - se * Rc[1 + 3 * j]);
            Ri[1 + 3 * j] = (float)(se * Rc[0 + 3 * j] + ce * Rc[1 + 3 * j]);
            Ri[2 + 3 * j] = Rc[2 + 3 * j];
        }
        uint64_t tu = (uint64_t)(t * 1e6) + 1000000u;
        pole_aid_att(p, tu, Ri);
        if (pole_aid_want_latch(p)) {
            float tip[3] = { 0, 0, 0 };
            pole_aid_latch(p, tip);
        }
        float tipn[3];
        pole_aid_feed_due(p, tipn);
        if (t >= next_gnss) {
            next_gnss += 0.05;
            double a[3];
            point_pos(&m, t, k_ant_b, a);
            a[0] += tip_drift * t + noise(&seed, 0.017);
            a[1] += noise(&seed, 0.017);
            pole_aid_gnss(p, tu, lat0 + a[0] / Re, lon0 + a[1] / (Re * cos(lat0)), 0.015f);
        }
    }
    /* Lift the pole: the episode ends and is fitted. */
    float junk[3] = { 3.0f, 0.0f, -9.81f }, w0[3] = { 0, 0, 0 }, vt[3] = { 0, 0, 0 };
    for (int k = 0; k < 400; k++) pole_aid_imu(p, (float)DT, junk, w0, vt);
    return pole_aid_take_yaw(p, dpsi, sd);
}

static void test_fit(void)
{
    pole_aid_t p;
    float dpsi, sd;
    const double errs[] = { 0.0, 90.0, 170.0, -45.0 };
    for (unsigned i = 0; i < sizeof errs / sizeof errs[0]; i++) {
        bool ok = run_fit(errs[i] * D2R, 0.0, 4.5, &dpsi, &sd, &p);
        CHECK(ok, "INS yaw off by %.0f deg: no fit (n %u spread %.3f rms %.4f scale %.3f)", errs[i],
              p.st.fit_n, (double)p.st.fit_spread_m, (double)p.st.fit_rms_m, (double)p.st.fit_scale);
        if (ok) {
            double e = remainder(dpsi / D2R - errs[i], 360.0);
            CHECK(fabs(e) < 3.0 && sd / D2R < 6.0, "INS yaw off by %.0f deg: dpsi %.2f sd %.2f", errs[i],
                  dpsi / D2R, sd / D2R);
            printf("  fit, INS yaw off by %4.0f deg: dpsi %7.2f deg, sd %.2f deg, n %u, spread %.3f m, rms %.4f m\n",
                   errs[i], dpsi / D2R, sd / D2R, p.st.fit_n, (double)p.st.fit_spread_m, (double)p.st.fit_rms_m);
        }
    }
    bool ok = run_fit(30 * D2R, 0.05, 4.5, &dpsi, &sd, &p);
    CHECK(!ok, "tip drifting 5 cm/s: fit accepted (dpsi %.2f)", dpsi / D2R);
    CHECK(!p.st.fit_skipped && p.st.yaw_rejects == 1, "tip drifting: skipped %d rejects %u", p.st.fit_skipped,
          p.st.yaw_rejects);

    /* Latched for about 0.45 s: a few fixes, too few to fit. Skipped, not
     * rejected. */
    ok = run_fit(0.0, 0.0, 1.0, &dpsi, &sd, &p);
    CHECK(!ok && p.st.fit_skipped && p.st.yaw_rejects == 0 && p.st.fit_scale == 0.0f,
          "short episode: ok %d skipped %d rejects %u", ok, p.st.fit_skipped, p.st.yaw_rejects);
}

static void test_mag(void)
{
    pole_aid_cfg_t cfg;
    pole_aid_default_cfg(&cfg);
    pole_aid_t p;
    const float m_n[3] = { 15.5f, 1.2f, 50.1f };
    motion_t m = { .yaw0 = 0.5, .roll_amp = 5 * D2R, .pitch_amp = 4 * D2R, .f1 = 0.3, .f2 = 0.2 };

    /* scale: the magnetometer reads this many times the field; yaw_err:
     * the INS yaw is off by it. */
    for (int c = 0; c < 4; c++) {
        double scale = c == 1 ? 1.2 : 1.0, yaw_err = c == 2 ? 10 * D2R : 0.0;
        bool abs_yaw = c >= 2;
        pole_aid_reset(&p, &cfg, true, k_tip_b, k_ant_b);
        bool trusted = false;
        for (double t = 0.0; t < 15.0; t += 1.0 / 100.0) {
            double R[3][3];
            attitude(&m, t, R);
            float b[3], Rc[9];
            for (int i = 0; i < 3; i++) {
                b[i] = 0.0f;
                for (int j = 0; j < 3; j++) b[i] += (float)(R[j][i] * m_n[j] * scale);
            }
            double ce = cos(-yaw_err), se = sin(-yaw_err);
            for (int j = 0; j < 3; j++) {
                Rc[0 + 3 * j] = (float)(ce * R[0][j] - se * R[1][j]);
                Rc[1 + 3 * j] = (float)(se * R[0][j] + ce * R[1][j]);
                Rc[2 + 3 * j] = (float)R[2][j];
            }
            trusted = pole_aid_mag(&p, (uint64_t)(t * 1e6) + 1000000u, b, Rc, abs_yaw, 0.02f, m_n);
        }
        bool want = c == 0 || c == 3;
        CHECK(trusted == want, "mag case %d: trusted %d (norm %.1f %% dip %.2f tilt range %.1f res %.2f/%.2f)", c,
              trusted, (double)p.st.mag_norm_err_pct, (double)p.st.mag_dip_err_deg,
              (double)p.st.mag_tilt_range_deg, (double)p.st.mag_res_mean_deg, (double)p.st.mag_res_sd_deg);
    }
    /* No WMM yet: never trusted. */
    pole_aid_reset(&p, &cfg, true, k_tip_b, k_ant_b);
    const float none[3] = { 0, 0, 0 };
    float b[3] = { 10, 0, 40 }, I[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
    bool t = true;
    for (int k = 0; k < 2000; k++) t = pole_aid_mag(&p, 1000000u + k * 10000u, b, I, false, 1.0f, none);
    CHECK(!t, "no WMM: trusted");
}

int main(void)
{
    test_detector();
    test_fit();
    test_mag();
    printf(s_fail ? "%d FAILED\n" : "all passed\n", s_fail);
    return s_fail != 0;
}
