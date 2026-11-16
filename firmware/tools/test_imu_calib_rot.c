/*
 * Host test of firmware/app/imu_calib_rot.c (plans/imu-auto-calibration.md,
 * Phase 4). Build and run on any host with a C compiler:
 *
 *   gcc -O2 -Wall -Wextra -I firmware/app firmware/tools/test_imu_calib_rot.c \
 *       firmware/app/imu_calib_rot.c firmware/app/imu_calib_lsq.c -lm \
 *       -o test_imu_calib_rot && ./test_imu_calib_rot
 *
 * Exit code 0 = all checks passed.
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "imu_calib_rot.h"

static int s_fail;

#define CHECK(cond, ...)                                    \
    do {                                                    \
        if (!(cond)) {                                      \
            printf("FAIL %s:%d: ", __FILE__, __LINE__);     \
            printf(__VA_ARGS__);                            \
            printf("\n");                                   \
            s_fail++;                                       \
        }                                                   \
    } while (0)

static unsigned long long s_seed = 3;
static double urand(void)
{
    s_seed = s_seed * 6364136223846793005ull + 1442695040888963407ull;
    return ((double)(s_seed >> 11) + 0.5) / 9007199254740992.0;
}
static double grand(void) { return sqrt(-2.0 * log(urand())) * cos(2.0 * M_PI * urand()); }

#define DT (1.0 / 800.0)
#define D2R (M_PI / 180.0)

/* True gyro calibration: w_true = M_t * u, u = raw - b. */
static const double k_mt[9] = { 1.010, 0.004, -0.003, -0.002, 0.992, 0.005, 0.003, -0.004, 1.006 };
static double k_mti[9];

static void inv3(const double a[9], double o[9])
{
    double det = a[0] * (a[4] * a[8] - a[5] * a[7]) - a[1] * (a[3] * a[8] - a[5] * a[6]) +
                 a[2] * (a[3] * a[7] - a[4] * a[6]);
    o[0] = (a[4] * a[8] - a[5] * a[7]) / det;
    o[1] = (a[2] * a[7] - a[1] * a[8]) / det;
    o[2] = (a[1] * a[5] - a[2] * a[4]) / det;
    o[3] = (a[5] * a[6] - a[3] * a[8]) / det;
    o[4] = (a[0] * a[8] - a[2] * a[6]) / det;
    o[5] = (a[2] * a[3] - a[0] * a[5]) / det;
    o[6] = (a[3] * a[7] - a[4] * a[6]) / det;
    o[7] = (a[1] * a[6] - a[0] * a[7]) / det;
    o[8] = (a[0] * a[4] - a[1] * a[3]) / det;
}

static void mul3(const double a[9], const double b[9], double o[9])
{
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            o[r * 3 + c] = a[r * 3 + 0] * b[0 * 3 + c] + a[r * 3 + 1] * b[1 * 3 + c] + a[r * 3 + 2] * b[2 * 3 + c];
        }
    }
}

static void exp3(const double w[3], double r[9])
{
    double th = sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
    if (th < 1e-12) {
        memset(r, 0, 9 * sizeof r[0]);
        r[0] = r[4] = r[8] = 1.0;
        return;
    }
    double k[3] = { w[0] / th, w[1] / th, w[2] / th }, s = sin(th), c = 1.0 - cos(th);
    r[0] = 1 - c * (k[1] * k[1] + k[2] * k[2]);
    r[1] = -s * k[2] + c * k[0] * k[1];
    r[2] = s * k[1] + c * k[0] * k[2];
    r[3] = s * k[2] + c * k[0] * k[1];
    r[4] = 1 - c * (k[0] * k[0] + k[2] * k[2]);
    r[5] = -s * k[0] + c * k[1] * k[2];
    r[6] = -s * k[1] + c * k[0] * k[2];
    r[7] = s * k[0] + c * k[1] * k[2];
    r[8] = 1 - c * (k[0] * k[0] + k[1] * k[1]);
}

/* Gravity direction in the body frame for body-to-nav R (specific force up). */
static void grav_body(const double r[9], float g[3])
{
    const double up[3] = { 0, 0, -1 };
    for (int i = 0; i < 3; i++) {
        g[i] = (float)(r[0 * 3 + i] * up[0] + r[1 * 3 + i] * up[1] + r[2 * 3 + i] * up[2]);
    }
}

/* One smooth rotation of total angle `ang` about the body axis `ax` in
 * `dur` s: truth R advanced, raw gyro u fed to the preintegration. */
static void rotate(double r[9], const double ax[3], double ang, double dur, rot_preint_t *p,
                   double noise)
{
    int n = (int)(dur / DT);
    for (int k = 0; k < n; k++) {
        double t = (k + 0.5) * DT;
        double rate = ang / dur * (1.0 - cos(2.0 * M_PI * t / dur)); /* integrates to ang */
        double w[3] = { ax[0] * rate, ax[1] * rate, ax[2] * rate };
        double u[3];
        for (int i = 0; i < 3; i++) {
            u[i] = k_mti[i * 3 + 0] * w[0] + k_mti[i * 3 + 1] * w[1] + k_mti[i * 3 + 2] * w[2];
        }
        float uf[3] = { (float)(u[0] + noise * grand()), (float)(u[1] + noise * grand()),
                        (float)(u[2] + noise * grand()) };
        rot_preint_step(p, uf, NULL, (float)DT);
        double e[9], rn[9], wdt[3] = { w[0] * DT, w[1] * DT, w[2] * DT };
        exp3(wdt, e);
        mul3(r, e, rn);
        memcpy(r, rn, sizeof rn);
    }
}

static void random_axis(double ax[3])
{
    double z = 2.0 * urand() - 1.0, ph = 2.0 * M_PI * urand(), s = sqrt(1.0 - z * z);
    ax[0] = s * cos(ph);
    ax[1] = s * sin(ph);
    ax[2] = z;
}

/* The preintegrated rotation equals the integrated truth at M = M_t. */
static void test_preint_identity(void)
{
    double r[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
    rot_preint_t p;
    rot_preint_reset(&p);
    const double ax[3] = { 0.0, 0.6, 0.8 };
    rotate(r, ax, 120.0 * D2R, 2.0, &p, 0.0);
    float ang = rot_preint_angle(&p);
    /* With M0 = I the gyro reads M_t^-1 w, so the angle is about 1 % off. */
    CHECK(fabsf(ang - (float)(120.0 * D2R)) < 0.02f && fabsf(ang - (float)(120.0 * D2R)) > 1e-4f,
          "angle %g deg", ang / D2R);
    CHECK(fabsf(p.t_s - 2.0f) < 0.01f, "time %g", p.t_s);
}

/* Poses all over the sphere (a pole): every element of M is found. */
static void test_rotations(void)
{
    rot_stats_t st;
    rot_stats_clear(&st);
    double r[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
    for (int k = 0; k < 40; k++) {
        float g1[3], g2[3];
        grav_body(r, g1);
        rot_preint_t p;
        rot_preint_reset(&p);
        double ax[3];
        random_axis(ax);
        rotate(r, ax, (60.0 + 100.0 * urand()) * D2R, 1.5 + urand(), &p, 0.003);
        grav_body(r, g2);
        for (int i = 0; i < 3; i++) {
            g1[i] += (float)(0.001 * grand());
            g2[i] += (float)(0.001 * grand());
        }
        CHECK(rot_add_rotation(&st, &p, g1, g2, 0.002f), "add rotation %d", k);
    }
    CHECK(st.n_rot == 40 && rot_stats_validate(&st) == NULL, "stats");
    rot_result_t res;
    rot_solve(&st, 0.02f, (float)(0.5 * D2R), &res);
    CHECK(res.ok, "solve");
    for (int i = 0; i < 9; i++) {
        double truth = k_mt[i] - (i % 4 == 0 ? 1.0 : 0.0);
        CHECK(fabs(res.dm[i] - truth) < 4.0 * res.sd[i] + 2e-4, "dM[%d] %g vs %g (sd %g)", i, res.dm[i],
              truth, res.sd[i]);
        CHECK(res.sd[i] < 0.002f, "sd[%d] %g did not shrink", i, res.sd[i]);
    }
    float m[9];
    int taken = rot_accept(&res, NULL, 0.002f, 0.002f, m);
    CHECK(taken == 9, "accepted %d of 9", taken);
    CHECK(rot_validate(m, 0.03f, (float)(2.0 * D2R)) == NULL, "valid M rejected");
}

/* A tractor: level turns about z with a GNSS heading before and after. Only
 * the z row is seen; the rest stays at the prior and is not accepted. */
static void test_turns(void)
{
    rot_stats_t st;
    rot_stats_clear(&st);
    for (int k = 0; k < 20; k++) {
        double ang = (k & 1 ? -1.0 : 1.0) * (150.0 + 60.0 * urand()) * D2R, dur = 8.0;
        int n = (int)(dur / DT);
        double pred = 0.0, meas = 0.0, h[9] = { 0 };
        for (int s = 0; s < n; s++) {
            double t = (s + 0.5) * DT;
            double rate = ang / dur * (1.0 - cos(2.0 * M_PI * t / dur));
            double w[3] = { 0.0, 0.0, rate }, u[3];
            for (int i = 0; i < 3; i++) {
                u[i] = k_mti[i * 3 + 0] * w[0] + k_mti[i * 3 + 1] * w[1] + k_mti[i * 3 + 2] * w[2] +
                       0.003 * grand();
            }
            /* level: yaw rate = body z rate; with M0 = I it is u_z */
            pred += u[2] * DT;
            for (int j = 0; j < 3; j++) {
                h[6 + j] += u[j] * DT;
            }
            meas += rate * DT;
        }
        float hf[9];
        for (int i = 0; i < 9; i++) {
            hf[i] = (float)h[i];
        }
        meas += 0.002 * grand(); /* about 0.1 deg of heading noise */
        CHECK(rot_add_turn(&st, (float)pred, (float)meas, hf, 0.003f), "add turn %d", k);
    }
    rot_result_t res;
    rot_solve(&st, 0.02f, (float)(0.5 * D2R), &res);
    CHECK(res.ok, "turns: solve");
    CHECK(fabs(res.dm[8] - (k_mt[8] - 1.0)) < 4.0 * res.sd[8], "turns: dMzz %g vs %g (sd %g)", res.dm[8],
          k_mt[8] - 1.0, res.sd[8]);
    CHECK(res.sd[8] < 0.0005f, "turns: sd zz %g", res.sd[8]);
    CHECK(res.sd[0] > 0.9f * 0.02f && res.sd[4] > 0.9f * 0.02f, "turns: x/y scale sd %g %g should stay",
          res.sd[0], res.sd[4]);
    float m[9];
    int taken = rot_accept(&res, NULL, 0.002f, (float)(0.05 * D2R), m);
    CHECK(taken == 1 && fabsf(m[8] - (float)k_mt[8]) < 0.001f && m[0] == 1.0f && m[1] == 0.0f,
          "turns: accepted %d, zz %g", taken, m[8]);
}

static void test_reject(void)
{
    float m[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
    CHECK(rot_validate(m, 0.03f, 0.035f) == NULL, "identity rejected");
    m[4] = 1.05f;
    CHECK(rot_validate(m, 0.03f, 0.035f) != NULL, "scale 1.05 accepted");
    m[4] = 1.0f;
    m[5] = 0.05f;
    CHECK(rot_validate(m, 0.03f, 0.035f) != NULL, "misalignment 0.05 accepted");
    rot_stats_t st;
    rot_stats_clear(&st);
    rot_preint_t p;
    rot_preint_reset(&p);
    float z[3] = { 0, 0, 0 }, g[3] = { 0, 0, -1 }, hn[9] = { NAN };
    CHECK(!rot_add_rotation(&st, &p, z, g, 0.002f), "zero gravity accepted");
    CHECK(!rot_add_turn(&st, 1.0f, 1.0f, hn, 0.01f), "NaN gradient accepted");
    rot_result_t r;
    rot_solve(&st, 0.02f, 0.01f, &r);
    CHECK(r.ok && r.dm[0] == 0.0f && fabsf(r.sd[0] - 0.02f) < 1e-6f, "empty: prior expected");
    st.h[3] = NAN;
    CHECK(rot_stats_validate(&st) != NULL, "NaN statistics accepted");
}

int main(void)
{
    inv3(k_mt, k_mti);
    test_preint_identity();
    test_rotations();
    test_turns();
    test_reject();
    if (s_fail) {
        printf("%d check(s) failed\n", s_fail);
        return 1;
    }
    printf("all checks passed\n");
    return 0;
}
