/*
 * Host test of firmware/app/imu_calib_mag.c (plans/imu-auto-calibration.md,
 * Phase 3). Build and run on any host with a C compiler:
 *
 *   gcc -O2 -Wall -Wextra -I firmware/app firmware/tools/test_imu_calib_mag.c \
 *       firmware/app/imu_calib_mag.c firmware/app/imu_calib_lsq.c -lm \
 *       -o test_imu_calib_mag && ./test_imu_calib_mag
 *
 * Exit code 0 = all checks passed.
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "imu_calib_mag.h"

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

static unsigned long long s_seed = 7;
static double urand(void)
{
    s_seed = s_seed * 6364136223846793005ull + 1442695040888963407ull;
    return ((double)(s_seed >> 11) + 0.5) / 9007199254740992.0;
}
static double grand(void) { return sqrt(-2.0 * log(urand())) * cos(2.0 * M_PI * urand()); }

static const mcal_prior_t k_prior = { .sd_diag_rel = 0.3f, .sd_offdiag = 0.2f, .sd_hard_ut = 100.0f,
                                      .sample_sd_ut = 0.3f, .dip_sd_ut = 0.6f };
static const mcal_limits_t k_lim = { .min_diag = 0.5f, .max_diag = 2.0f, .max_offdiag = 0.3f,
                                     .max_hard_ut = 200.0f, .max_rms_rel = 0.05f };

/* True calibration: m_true = M * (raw - b). M = R * S with S symmetric soft
 * iron and scale (the TM210 reads about 0.8 of the field) and R a 2 deg
 * rotation about z (the mounting). */
static double k_m[3][3], k_minv[3][3];
static const double k_b[3] = { 18.0, -7.5, 31.0 };
static const double k_f = 51.2;
static const double k_incl = 72.0 * M_PI / 180.0; /* dip, positive down */

static void inv3d(double a[3][3], double o[3][3])
{
    double det = a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) -
                 a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) +
                 a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
    o[0][0] = (a[1][1] * a[2][2] - a[1][2] * a[2][1]) / det;
    o[0][1] = (a[0][2] * a[2][1] - a[0][1] * a[2][2]) / det;
    o[0][2] = (a[0][1] * a[1][2] - a[0][2] * a[1][1]) / det;
    o[1][0] = (a[1][2] * a[2][0] - a[1][0] * a[2][2]) / det;
    o[1][1] = (a[0][0] * a[2][2] - a[0][2] * a[2][0]) / det;
    o[1][2] = (a[0][2] * a[1][0] - a[0][0] * a[1][2]) / det;
    o[2][0] = (a[1][0] * a[2][1] - a[1][1] * a[2][0]) / det;
    o[2][1] = (a[0][1] * a[2][0] - a[0][0] * a[2][1]) / det;
    o[2][2] = (a[0][0] * a[1][1] - a[0][1] * a[1][0]) / det;
}

static void setup_truth(void)
{
    const double s[3][3] = { { 1.26, 0.03, -0.02 }, { 0.03, 1.22, 0.015 }, { -0.02, 0.015, 1.30 } };
    const double a = 2.0 * M_PI / 180.0;
    const double r[3][3] = { { cos(a), -sin(a), 0 }, { sin(a), cos(a), 0 }, { 0, 0, 1 } };
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            k_m[i][j] = 0;
            for (int k = 0; k < 3; k++) {
                k_m[i][j] += r[i][k] * s[k][j];
            }
        }
    }
    inv3d(k_m, k_minv);
}

/* raw for a true field vector, with noise. */
static void make_raw(const double t[3], float raw[3], double noise)
{
    for (int i = 0; i < 3; i++) {
        raw[i] = (float)(k_minv[i][0] * t[0] + k_minv[i][1] * t[1] + k_minv[i][2] * t[2] + k_b[i] +
                         noise * grand());
    }
}

static void random_dir(double f, double t[3])
{
    double z = 2.0 * urand() - 1.0, ph = 2.0 * M_PI * urand(), r = sqrt(1.0 - z * z);
    t[0] = f * r * cos(ph);
    t[1] = f * r * sin(ph);
    t[2] = f * z;
}

/* A sample at attitude roll/pitch/yaw (NED, ZYX): the true field and the
 * down direction in the body frame (body = C_nb^T * ned). */
static void sample_at(double roll, double pitch, double yaw, mcal_sample_t *out, double t[3])
{
    const double cr = cos(roll), sr = sin(roll), cp = cos(pitch), sp = sin(pitch);
    const double cy = cos(yaw), sy = sin(yaw);
    const double c[3][3] = {
        { cp * cy, sr * sp * cy - cr * sy, cr * sp * cy + sr * sy },
        { cp * sy, sr * sp * sy + cr * cy, cr * sp * sy - sr * cy },
        { -sp, sr * cp, cr * cp },
    };
    const double b_ned[3] = { k_f * cos(k_incl), 0.0, k_f * sin(k_incl) };
    for (int j = 0; j < 3; j++) {
        t[j] = c[0][j] * b_ned[0] + c[1][j] * b_ned[1] + c[2][j] * b_ned[2];
        out->down[j] = (float)c[2][j];
    }
    out->has_down = true;
    make_raw(t, out->m, 0.2);
}

/* Orientations all over the sphere (a pole turned by hand). */
static void test_ellipsoid_and_reference(void)
{
    mcal_sample_t smp[54];
    for (int j = 0; j < 54; j++) {
        double t[3];
        sample_at(M_PI * (2.0 * urand() - 1.0), 0.5 * M_PI * (2.0 * urand() - 1.0),
                  M_PI * (2.0 * urand() - 1.0), &smp[j], t);
    }
    mcal_ell_t e;
    mcal_ellipsoid(smp, 54, (float)k_f, (float)k_incl, &k_prior, &e);
    CHECK(e.ok, "ellipsoid not ok (%d iterations)", e.iterations);
    CHECK(e.rms_after < 0.5f && e.rms_before > 5.0f, "ellipsoid rms %g -> %g", e.rms_before,
          e.rms_after);
    CHECK(e.n_down == 54 && e.rms_dip_ut >= 0.0f && e.rms_dip_ut < 1.5f, "dip rms %g (n %d)",
          e.rms_dip_ut, e.n_down);
    for (int k = 0; k < 3; k++) {
        CHECK(fabsf(e.p[MCAL_CX + k] - (float)k_b[k]) < 0.5f, "centre[%d] %g vs %g", k,
              e.p[MCAL_CX + k], k_b[k]);
    }

    /* Without reference data the calibration is the ellipsoid's: symmetric,
     * so the 2 deg rotation is still in it. */
    mcal_ref_stats_t st;
    mcal_ref_clear(&st);
    mcal_cal_t c0;
    mcal_ref_solve(&st, &e, 0.05f, 5.0f, &c0);
    CHECK(c0.ok && c0.n_ref == 0 && c0.rms_ref_ut < 0.0f, "no-reference solve");
    CHECK(fabsf(c0.m[1] - c0.m[3]) < 1e-5f, "no reference: M not symmetric");
    CHECK(fabsf(c0.b[0] - e.p[MCAL_CX]) < 1e-3f, "no reference: b %g vs centre %g", c0.b[0],
          e.p[MCAL_CX]);
    CHECK(mcal_validate(&c0, &e, &k_lim) == NULL, "ellipsoid result rejected: %s",
          mcal_validate(&c0, &e, &k_lim));

    /* With the true field as reference, the full M (rotation included). */
    for (int j = 0; j < 300; j++) {
        double t[3];
        float r[3], ref[3];
        random_dir(k_f, t);
        make_raw(t, r, 0.2);
        for (int i = 0; i < 3; i++) {
            ref[i] = (float)(t[i] + 0.5 * grand()); /* attitude error */
        }
        mcal_ref_add(&st, r, ref, 0.6f);
    }
    mcal_cal_t c;
    mcal_ref_solve(&st, &e, 0.05f, 5.0f, &c);
    CHECK(c.ok && c.n_ref == 300, "reference solve");
    for (int i = 0; i < 3; i++) {
        for (int k = 0; k < 3; k++) {
            CHECK(fabsf(c.m[i * 3 + k] - (float)k_m[i][k]) < 0.01f, "M[%d][%d] %g vs %g", i, k,
                  c.m[i * 3 + k], k_m[i][k]);
        }
        CHECK(fabsf(c.b[i] - (float)k_b[i]) < 0.5f, "b[%d] %g vs %g", i, c.b[i], k_b[i]);
    }
    CHECK(c.rms_ref_ut > 0.2f && c.rms_ref_ut < 1.0f, "reference rms %g", c.rms_ref_ut);
    CHECK(mcal_validate(&c, &e, &k_lim) == NULL, "reference result rejected");

    /* The rotation shows as the antisymmetric part: about 2 deg about z. */
    float rot = 0.5f * (c.m[3] - c.m[1]) / c.m[0];
    CHECK(fabsf(rot - (float)(2.0 * M_PI / 180.0)) < 0.003f, "rotation %g rad", rot);

    float m[9], b[3];
    mcal_to_inslib(&c, m, b);
    CHECK(m[1] == c.m[3] && m[3] == c.m[1] && b[2] == c.b[2], "column-major layout");

    mcal_ref_forget(&st, 0.5f);
    CHECK(mcal_ref_validate(&st) == NULL && st.n == 300, "forget");
}

/* Heading error over the level ring (yaw j * 10 deg) with calibration c, on
 * noise-free raw samples: the worst change relative to the first sample
 * (the constant offset and the wrap at +-180 deg drop out), and optionally
 * the offset itself and the worst raw error the same way. */
static double level_heading_error(const mcal_cal_t *c, float raw[36][3], double *raw_worst,
                                  double *offset)
{
    double err0 = 0.0, rerr0 = 0.0, worst = 0.0;
    for (int j = 0; j < 36; j++) {
        double yaw = 2.0 * M_PI * j / 36.0, v[3];
        for (int i = 0; i < 3; i++) {
            v[i] = c->m[i * 3 + 0] * (raw[j][0] - c->b[0]) + c->m[i * 3 + 1] * (raw[j][1] - c->b[1]) +
                   c->m[i * 3 + 2] * (raw[j][2] - c->b[2]);
        }
        double err = atan2(-v[1], v[0]) - yaw; /* the field seen from the body is at heading +yaw */
        double rerr = atan2(-raw[j][1], raw[j][0]) - yaw;
        if (j == 0) {
            err0 = err;
            rerr0 = rerr;
            if (offset) *offset = remainder(err, 2.0 * M_PI);
        }
        double d = fabs(remainder(err - err0, 2.0 * M_PI));
        if (d > worst) worst = d;
        if (raw_worst) {
            double dr = fabs(remainder(rerr - rerr0, 2.0 * M_PI));
            if (dr > *raw_worst) *raw_worst = dr;
        }
    }
    return worst;
}

/* A tractor on level ground: turns about z only, dip 72 deg. The ellipsoid
 * is a horizontal circle: the fit stays near its prior where it can't see. */
static void test_level_turns(void)
{
    mcal_sample_t smp[36];
    float raw[36][3]; /* noise-free, to judge the calibration */
    for (int j = 0; j < 36; j++) {
        double t[3];
        sample_at(0.0, 0.0, 2.0 * M_PI * j / 36.0, &smp[j], t);
        make_raw(t, raw[j], 0.0);
    }
    mcal_ell_t e;
    mcal_ellipsoid(smp, 36, (float)k_f, (float)k_incl, &k_prior, &e);
    CHECK(e.ok, "level: not ok");
    CHECK(e.rms_after < 0.5f, "level: rms after %g", e.rms_after);
    /* The 3D centre can't be found from a ring: with z nearly constant, an
     * xz / yz soft-iron term acts like a horizontal offset. What level
     * driving needs is the heading, and that is observable: the corrected
     * field must turn with the true yaw (up to a constant, which only the
     * reference fit can resolve). Uncorrected, hard iron of this size bends
     * the heading by tens of degrees. */
    mcal_ref_stats_t st;
    mcal_ref_clear(&st);
    mcal_cal_t c;
    mcal_ref_solve(&st, &e, 0.05f, 5.0f, &c);
    double raw_worst = 0.0;
    double worst = level_heading_error(&c, raw, &raw_worst, NULL);
    /* About 1 deg is left: on a ring the xz / yz soft-iron terms couple with
     * the unseen z centre and scale, and a symmetric M can't match all of
     * it. Without the dip residual this case was off by up to 180 deg. */
    CHECK(worst < 1.5 * M_PI / 180.0, "level: heading error %.3f deg after the ellipsoid fit",
          worst * 180.0 / M_PI);
    CHECK(raw_worst > 10.0 * M_PI / 180.0, "level: raw heading error only %.1f deg?",
          raw_worst * 180.0 / M_PI);

    /* A tractor with a GNSS heading: the reference fit on the same ring
     * removes the rest, including the constant offset (the mounting). */
    for (int rep = 0; rep < 5; rep++) {
        for (int j = 0; j < 36; j++) {
            double t[3];
            mcal_sample_t s1;
            sample_at(0.0, 0.0, 2.0 * M_PI * (j + 0.2 * rep) / 36.0, &s1, t);
            float ref[3] = { (float)t[0], (float)t[1], (float)t[2] };
            mcal_ref_add(&st, s1.m, ref, 0.6f);
        }
    }
    mcal_ref_solve(&st, &e, 0.05f, 5.0f, &c);
    double offset = 0.0;
    worst = level_heading_error(&c, raw, NULL, &offset);
    CHECK(worst < 0.3 * M_PI / 180.0, "level + reference: heading error %.3f deg", worst * 180.0 / M_PI);
    CHECK(fabs(offset) < 0.3 * M_PI / 180.0, "level + reference: heading offset %.3f deg",
          offset * 180.0 / M_PI);
}

static void test_reject_and_empty(void)
{
    mcal_ell_t e;
    mcal_ellipsoid(NULL, 0, 50.0f, 1.2f, &k_prior, &e);
    CHECK(e.ok && e.p[MCAL_A00] == 1.0f && e.p[MCAL_CX] == 0.0f, "empty: prior expected");
    mcal_cal_t c;
    memset(&c, 0, sizeof c);
    c.ok = true;
    c.m[0] = c.m[4] = c.m[8] = 1.2f;
    CHECK(mcal_validate(&c, NULL, &k_lim) == NULL, "identity-ish rejected");
    c.m[4] = 2.5f;
    CHECK(mcal_validate(&c, NULL, &k_lim) != NULL, "scale 2.5 accepted");
    c.m[4] = 1.2f;
    c.m[5] = 0.5f;
    CHECK(mcal_validate(&c, NULL, &k_lim) != NULL, "off-diagonal 0.5 accepted");
    c.m[5] = 0.0f;
    c.b[1] = 250.0f;
    CHECK(mcal_validate(&c, NULL, &k_lim) != NULL, "hard iron 250 uT accepted");
    c.b[1] = 0.0f;
    mcal_ell_t bad;
    memset(&bad, 0, sizeof bad);
    bad.n = 20;
    bad.field_ut = 50.0f;
    bad.rms_after = 4.0f;
    CHECK(mcal_validate(&c, &bad, &k_lim) != NULL, "8 %% residual accepted");

    mcal_ref_stats_t st;
    mcal_ref_clear(&st);
    float nan3[3] = { NAN, 0, 0 }, ok3[3] = { 1, 2, 3 };
    CHECK(!mcal_ref_add(&st, nan3, ok3, 1.0f) && st.n == 0, "NaN reference added");
    st.s[0] = NAN;
    CHECK(mcal_ref_validate(&st) != NULL, "NaN statistics accepted");
}

int main(void)
{
    setup_truth();
    test_ellipsoid_and_reference();
    test_level_turns();
    test_reject_and_empty();
    if (s_fail) {
        printf("%d check(s) failed\n", s_fail);
        return 1;
    }
    printf("all checks passed\n");
    return 0;
}
