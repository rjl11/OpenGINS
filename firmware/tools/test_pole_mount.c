/* Host test for the pole-axis fit: firmware/app/pole_aid.c (the episode sums)
 * and firmware/app/pole_mount_fit.c. Build and run from the repo root:
 *
 *   gcc -O2 -Wall -Wextra -I firmware/app firmware/tools/test_pole_mount.c \
 *       firmware/app/pole_aid.c firmware/app/pole_mount_fit.c \
 *       firmware/app/imu_calib_lsq.c -lm -o temp/test_pole_mount
 *   temp/test_pole_mount
 *
 * An exact rigid-body motion as in test_pole_aid.c, but the IMU sits tilted
 * on the pole: the real axis (antenna to tip) is u_true in the body frame,
 * while pole_aid assumes the body down axis. The tip stays at the origin;
 * the pole sways, and twists about the vertical by a set amount.
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "pole_aid.h"
#include "pole_mount_fit.h"

static int s_fail;

#define CHECK(c, ...) do { if (!(c)) { printf("FAIL %s:%d: ", __FILE__, __LINE__); \
                                       printf(__VA_ARGS__); printf("\n"); s_fail++; } } while (0)

#define D2R (3.14159265358979323846 / 180.0)
#define G 9.81
#define DT (1.0 / 800.0)
#define L_POLE 2.0

static const float k_ant_b[3] = { 0.0f, 0.0f, -0.1f }; /* IMU to antenna */
static float s_tip_true[3];                             /* IMU to tip, real */

typedef struct {
    double yaw0, twist_amp, twist_f; /* yaw = yaw0 + amp sin(2 pi f t)       */
    double turn_rate;                /* plus a steady turn [rad/s]           */
    double roll_amp, pitch_amp, f1, f2;
    double drift;                    /* tip moving north [m/s]               */
} motion_t;

static void attitude(const motion_t *m, double t, double R[3][3])
{
    double r = m->roll_amp * sin(2 * M_PI * m->f1 * t);
    double p = m->pitch_amp * sin(2 * M_PI * m->f2 * t + 0.7);
    double y = m->yaw0 + m->twist_amp * sin(2 * M_PI * m->twist_f * t) + m->turn_rate * t;
    double cr = cos(r), sr = sin(r), cp = cos(p), sp = sin(p), cy = cos(y), sy = sin(y);
    R[0][0] = cp * cy; R[0][1] = sr * sp * cy - cr * sy; R[0][2] = cr * sp * cy + sr * sy;
    R[1][0] = cp * sy; R[1][1] = sr * sp * sy + cr * cy; R[1][2] = cr * sp * sy - sr * cy;
    R[2][0] = -sp;     R[2][1] = sr * cp;                R[2][2] = cr * cp;
}

static void point_pos(const motion_t *m, double t, const float o_b[3], double out[3])
{
    double R[3][3];
    attitude(m, t, R);
    for (int i = 0; i < 3; i++) {
        out[i] = (i == 0 ? m->drift * t : 0.0);
        for (int j = 0; j < 3; j++) out[i] += R[i][j] * (o_b[j] - s_tip_true[j]);
    }
}

static const float k_zero[3] = { 0, 0, 0 };

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

/* Roughly Gaussian noise (sum of four uniforms), sd about a. */
static double noise(unsigned *s, double a)
{
    double x = 0.0;
    for (int k = 0; k < 4; k++) {
        *s = *s * 1103515245u + 12345u;
        x += (double)((*s >> 8) & 0xFFFF) / 65536.0 - 0.5;
    }
    return x * a * 1.732;
}

/* One planted episode of dur s. yaw_err: the INS yaw is off by it; bend:
 * an INS roll error that follows the twist (what a tip constraint with the
 * wrong axis does), which the gyro-only attitude must not see. Returns
 * whether an episode record came out. */
static bool episode(const motion_t *m, double dur, double yaw_err, double bend, unsigned seed, pole_episode_t *out,
                    pole_aid_t *p)
{
    pole_aid_cfg_t cfg;
    pole_aid_default_cfg(&cfg);
    float tip_assumed[3] = { k_ant_b[0], k_ant_b[1], k_ant_b[2] + (float)L_POLE };
    pole_aid_reset(p, &cfg, true, tip_assumed, k_ant_b);
    const double lat0 = 58.09 * D2R, lon0 = 12.92 * D2R, Re = 6371000.0;
    double next_gnss = 0.0;
    for (double t = 0.0; t < dur; t += DT) {
        float f[3], w[3], Rc[9], Ri[9];
        double v[3];
        sample(m, t, f, w, v, Rc);
        /* INS attitude: yawed by -yaw_err, rolled by bend * sin(yaw). */
        double R[3][3], y = atan2(Rc[1], Rc[0]);
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++) R[i][j] = Rc[i + 3 * j];
        double ce = cos(-yaw_err), se = sin(-yaw_err), b = bend * sin(y);
        double cb = cos(b), sb = sin(b);
        for (int j = 0; j < 3; j++) {
            /* Rx(b) in NED (about north) after Rz(-yaw_err). */
            double n0 = ce * R[0][j] - se * R[1][j], e0 = se * R[0][j] + ce * R[1][j], d0 = R[2][j];
            Ri[0 + 3 * j] = (float)n0;
            Ri[1 + 3 * j] = (float)(cb * e0 - sb * d0);
            Ri[2 + 3 * j] = (float)(sb * e0 + cb * d0);
        }
        /* The INS tip velocity: the tip doesn't move (or drifts). */
        float vt[3] = { (float)m->drift, 0.0f, 0.0f };
        uint64_t tu = (uint64_t)(t * 1e6) + 1000000u;
        pole_aid_att(p, tu, Ri);
        pole_aid_imu(p, (float)DT, f, w, vt);
        if (pole_aid_want_latch(p)) {
            float tip[3] = { 0, 0, 0 };
            pole_aid_latch(p, tip);
        }
        float tipn[3];
        pole_aid_feed_due(p, tipn);
        if (t >= next_gnss) {
            next_gnss += 0.05;
            double a[3];
            point_pos(m, t, k_ant_b, a);
            a[0] += noise(&seed, 0.012);
            a[1] += noise(&seed, 0.012);
            pole_aid_gnss(p, tu, lat0 + a[0] / Re, lon0 + a[1] / (Re * cos(lat0)), 0.015f);
        }
    }
    float junk[3] = { 3.0f, 0.0f, -9.81f }, w0[3] = { 0, 0, 0 }, vt[3] = { 0, 0, 0 };
    for (int k = 0; k < 400; k++) pole_aid_imu(p, (float)DT, junk, w0, vt);
    return pole_aid_take_episode(p, out);
}

static void set_mount(double tilt_deg, double azim_deg)
{
    double t = tilt_deg * D2R, a = azim_deg * D2R;
    double u[3] = { sin(t) * cos(a), sin(t) * sin(a), cos(t) };
    for (int i = 0; i < 3; i++) s_tip_true[i] = k_ant_b[i] + (float)(L_POLE * u[i]);
}

static void prior(pm_prior_t *pr) { pm_prior_init(pr, (float)L_POLE, 3.0f, 0.05f, 0.015f, 10.0f, 0.02f, 1.5f); }

/* n episodes with the given twist; returns the result. */
/* n episodes into s (cleared first unless keep). */
static void run(int n, double twist_deg, double turn_dps, double bend_deg, double drift, pm_stats_t *s,
                pm_result_t *r, int *offered, int *added, bool keep)
{
    pm_prior_t pr;
    prior(&pr);
    if (!keep) memset(s, 0, sizeof *s);
    *offered = *added = 0;
    pole_aid_t p;
    for (int k = 0; k < n; k++) {
        motion_t m = { .yaw0 = 0.7 * k, .twist_amp = twist_deg * D2R / 2, .twist_f = 0.2,
                       .turn_rate = turn_dps * D2R, .roll_amp = 4 * D2R, .pitch_amp = 3 * D2R,
                       .f1 = 0.6, .f2 = 0.45, .drift = drift };
        pole_episode_t e;
        double yaw_err = (k % 4) * 50.0 * D2R - 60.0 * D2R;
        if (episode(&m, 4.5, yaw_err, bend_deg * D2R, 1000u + 77u * k, &e, &p)) {
            (*offered)++;
            float rms;
            if (pm_add_episode(s, &e, &pr, &rms)) (*added)++;
        }
    }
    pm_solve(s, &pr, r);
}

static double ang_err(double a, double b) { return remainder(a - b, 360.0); }

static void report(const char *name, const pm_stats_t *s, const pm_result_t *r, int offered, int added)
{
    printf("  %-28s episodes %d/%d  tilt %.3f deg az %6.1f  sd %.3f  L %.4f +- %.4f m  rms %.4f m  twist %.0f deg\n",
           name, added, offered, (double)r->tilt_deg, (double)r->azim_deg, (double)r->tilt_sd_deg,
           (double)r->len_m, (double)r->len_sd_m, (double)r->rms_m, (double)s->twist_deg);
}

static void test_quat(void)
{
    /* Integrating the exact body rate reproduces the attitude over 5 s. */
    motion_t m = { .yaw0 = 1.0, .twist_amp = 0.4, .twist_f = 0.2, .turn_rate = 0.5, .roll_amp = 6 * D2R,
                   .pitch_amp = 5 * D2R, .f1 = 0.6, .f2 = 0.45 };
    set_mount(0.0, 0.0);
    float f[3], w[3], Rc[9], q[4], Rq[9];
    double v[3];
    sample(&m, 0.0, f, w, v, Rc);
    pole_quat_from_R(Rc, q);
    pole_quat_to_R(q, Rq);
    double e0 = 0.0;
    for (int i = 0; i < 9; i++) e0 = fmax(e0, fabs(Rq[i] - Rc[i]));
    CHECK(e0 < 1e-6, "quaternion round trip %.2e", e0);
    double t = 0.0;
    for (int k = 0; k < 4000; k++) {
        sample(&m, t + 0.5 * DT, f, w, v, Rc);
        pole_quat_step(q, w, (float)DT);
        t += DT;
    }
    sample(&m, t, f, w, v, Rc);
    pole_quat_to_R(q, Rq);
    double e = 0.0;
    for (int i = 0; i < 9; i++) e = fmax(e, fabs(Rq[i] - Rc[i]));
    CHECK(e < 3e-4, "gyro attitude after 5 s off by %.2e (about %.3f deg)", e, e / D2R);
}

static void test_fits(void)
{
    pm_stats_t s;
    pm_result_t r;
    int off, add;

    /* Natural twist, +-20 deg. */
    set_mount(1.5, 60.0);
    run(20, 40.0, 0.0, 0.0, 0.0, &s, &r, &off, &add, false);
    report("1.5 deg, twist +-20", &s, &r, off, add);
    CHECK(add >= 18, "natural twist: %d of %d episodes added", add, off);
    CHECK(fabs(r.tilt_deg - 1.5) < 0.2 && fabs(ang_err(r.azim_deg, 60.0)) < 15.0,
          "natural twist: tilt %.3f az %.1f", (double)r.tilt_deg, (double)r.azim_deg);
    CHECK(fabs(r.len_m - L_POLE) < 0.02, "natural twist: L %.4f", (double)r.len_m);

    /* The INS attitude bent by the tip constraint: no effect, the fit runs
     * on the gyro attitude. */
    run(20, 40.0, 0.0, 0.5, 0.0, &s, &r, &off, &add, false);
    report("same, INS bent 0.5 deg", &s, &r, off, add);
    CHECK(fabs(r.tilt_deg - 1.5) < 0.2 && fabs(ang_err(r.azim_deg, 60.0)) < 15.0,
          "INS bent: tilt %.3f az %.1f", (double)r.tilt_deg, (double)r.azim_deg);

    /* One slow full turn on the point (80 deg/s for 4.5 s). */
    set_mount(2.0, -120.0);
    run(1, 0.0, 80.0, 0.0, 0.0, &s, &r, &off, &add, false);
    report("2 deg, one full turn", &s, &r, off, add);
    CHECK(add == 1, "full turn: %d of %d added", add, off);
    CHECK(fabs(r.tilt_deg - 2.0) < 0.3 && fabs(ang_err(r.azim_deg, -120.0)) < 15.0 && r.tilt_sd_deg <= 0.2f,
          "full turn: tilt %.3f az %.1f sd %.3f", (double)r.tilt_deg, (double)r.azim_deg, (double)r.tilt_sd_deg);

    /* No twist at all: the tilt stays at its prior. */
    set_mount(1.5, 60.0);
    run(20, 0.0, 0.0, 0.0, 0.0, &s, &r, &off, &add, false);
    report("1.5 deg, no twist", &s, &r, off, add);
    CHECK(r.tilt_sd_deg > 0.2f, "no twist: tilt sd %.3f claims an answer", (double)r.tilt_sd_deg);
    CHECK(fabs(r.len_m - L_POLE) < 0.02, "no twist: L %.4f (sway alone shows it)", (double)r.len_m);

    /* Mounted straight: nothing to find. */
    set_mount(0.0, 0.0);
    run(20, 40.0, 0.0, 0.0, 0.0, &s, &r, &off, &add, false);
    report("straight, twist +-20", &s, &r, off, add);
    CHECK(r.tilt_deg < 0.2, "straight: tilt %.3f", (double)r.tilt_deg);

    /* A tip slipping 5 cm/s during normal use: dropped (by pole_aid's scale
     * gate or here), and the answer stays. */
    set_mount(1.5, 60.0);
    run(20, 40.0, 0.0, 0.0, 0.0, &s, &r, &off, &add, false);
    run(6, 40.0, 0.0, 0.0, 0.05, &s, &r, &off, &add, true);
    report("then tip slipping 5 cm/s", &s, &r, off, add);
    CHECK(add == 0, "slipping tip: %d of %d added", add, off);
    CHECK(fabs(r.tilt_deg - 1.5) < 0.2, "after the slips: tilt %.3f", (double)r.tilt_deg);
}

int main(void)
{
    test_quat();
    test_fits();
    printf(s_fail ? "%d FAILED\n" : "all passed\n", s_fail);
    return s_fail != 0;
}
