/*
 * Host test of firmware/app/imu_calib_acc.c (plans/imu-auto-calibration.md,
 * Phase 2). Build and run on any host with a C compiler:
 *
 *   gcc -O2 -Wall -Wextra -I firmware/app firmware/tools/test_imu_calib_acc.c \
 *       firmware/app/imu_calib_acc.c firmware/app/imu_calib_lsq.c -lm -o test_imu_calib_acc && ./test_imu_calib_acc
 *
 * Exit code 0 = all checks passed.
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "imu_calib_acc.h"

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

static unsigned long long s_seed = 99;
static double urand(void)
{
    s_seed = s_seed * 6364136223846793005ull + 1442695040888963407ull;
    return ((double)(s_seed >> 11) + 0.5) / 9007199254740992.0;
}
static double grand(void) { return sqrt(-2.0 * log(urand())) * cos(2.0 * M_PI * urand()); }

static const acal_prior_t k_prior = { .sd_scale = 0.02f, .sd_mis = 0.01f, .sd_bias = 0.1f };
static const acal_limits_t k_lim = { .max_scale_dev = 0.05f, .max_mis_rad = 0.035f, .max_bias = 0.5f };

/* The true calibration: raw = M^-1 * a + b, for a true specific force a. */
static const float k_true[ACAL_NPAR] = { 1.012f, 0.993f, 1.005f, 0.004f, -0.003f, 0.002f,
                                         0.06f, -0.04f, 0.09f };

static void make_raw(const float t[ACAL_NPAR], const double a[3], float raw[3])
{
    /* Solve M u = a for the upper-triangular M, then raw = u + b. */
    double u2 = a[2] / t[ACAL_SZ];
    double u1 = (a[1] - t[ACAL_MYZ] * u2) / t[ACAL_SY];
    double u0 = (a[0] - t[ACAL_MXY] * u1 - t[ACAL_MXZ] * u2) / t[ACAL_SX];
    raw[0] = (float)(u0 + t[ACAL_BX]);
    raw[1] = (float)(u1 + t[ACAL_BY]);
    raw[2] = (float)(u2 + t[ACAL_BZ]);
}

/* Poses spread over the sphere recover every parameter. */
static void test_sphere(void)
{
    const float g = 9.8187f;
    const float sd = 0.002f;
    acal_pose_t poses[24];
    for (int j = 0; j < 24; j++) {
        double z = 2.0 * urand() - 1.0, ph = 2.0 * M_PI * urand(), rxy = sqrt(1.0 - z * z);
        double a[3] = { g * rxy * cos(ph), g * rxy * sin(ph), g * z };
        make_raw(k_true, a, poses[j].acc);
        for (int i = 0; i < 3; i++) {
            poses[j].acc[i] += (float)(sd * grand());
        }
        poses[j].sd = sd;
    }
    acal_result_t r;
    acal_solve(poses, 24, g, &k_prior, &r);
    CHECK(r.ok, "not ok after %d iterations", r.iterations);
    for (int i = 0; i < ACAL_NPAR; i++) {
        CHECK(fabsf(r.p[i] - k_true[i]) < 4.0f * r.sd[i] + 1e-6f, "p[%d] %g vs %g (sd %g)", i, r.p[i],
              k_true[i], r.sd[i]);
        CHECK(r.sd[i] < 0.2f * (i <= ACAL_SZ ? k_prior.sd_scale : i <= ACAL_MYZ ? k_prior.sd_mis
                                                                                 : k_prior.sd_bias),
              "sd[%d] %g did not shrink", i, r.sd[i]);
    }
    CHECK(r.rms_after < 2.0f * sd && r.rms_before > 0.02f, "rms %g -> %g", r.rms_before, r.rms_after);
    CHECK(acal_validate(&r, &k_lim) == NULL, "valid result rejected");
    float cov = acal_coverage(poses, 24);
    CHECK(cov > 0.15f && cov <= 1.0f / 3.0f + 1e-4f, "sphere coverage %g", cov);
}

/* Level only (a tractor): the magnitude at that pose is fixed, every other
 * parameter stays near its prior. */
static void test_level_only(void)
{
    const float g = 9.8187f;
    acal_pose_t poses[5];
    for (int j = 0; j < 5; j++) {
        double tilt = 0.02 * (j - 2); /* +-2.3 deg */
        double a[3] = { g * sin(tilt), 0.0, -g * cos(tilt) };
        make_raw(k_true, a, poses[j].acc);
        poses[j].sd = 0.002f;
    }
    acal_result_t r;
    acal_solve(poses, 5, g, &k_prior, &r);
    CHECK(r.ok, "level: not ok");
    CHECK(r.rms_after < 0.002f, "level: residual %g", r.rms_after);
    CHECK(fabsf(r.p[ACAL_SX] - 1.0f) < 0.5f * k_prior.sd_scale, "level: sx moved to %g", r.p[ACAL_SX]);
    CHECK(fabsf(r.p[ACAL_SY] - 1.0f) < 1e-4f, "level: sy moved to %g", r.p[ACAL_SY]);
    CHECK(fabsf(r.p[ACAL_BY]) < 1e-4f, "level: by moved to %g", r.p[ACAL_BY]);
    CHECK(r.sd[ACAL_SY] > 0.9f * k_prior.sd_scale, "level: sd sy %g should stay at the prior",
          r.sd[ACAL_SY]);
    CHECK(acal_coverage(poses, 5) < 0.01f, "level coverage %g", acal_coverage(poses, 5));
}

static void test_prior_reject_misc(void)
{
    acal_result_t r;
    acal_solve(NULL, 0, 9.81f, &k_prior, &r);
    CHECK(r.ok && r.p[ACAL_SX] == 1.0f && r.p[ACAL_BX] == 0.0f, "no poses must give the prior");
    CHECK(fabsf(r.sd[ACAL_BZ] - k_prior.sd_bias) < 1e-5f, "no poses: sd %g", r.sd[ACAL_BZ]);
    CHECK(acal_validate(&r, &k_lim) == NULL, "prior rejected");
    r.p[ACAL_SY] = 1.08f;
    CHECK(acal_validate(&r, &k_lim) != NULL, "scale 1.08 accepted");
    r.p[ACAL_SY] = 1.0f;
    r.p[ACAL_MXZ] = 0.05f;
    CHECK(acal_validate(&r, &k_lim) != NULL, "misalignment 0.05 rad accepted");
    r.p[ACAL_MXZ] = 0.0f;
    r.p[ACAL_BY] = NAN;
    CHECK(acal_validate(&r, &k_lim) != NULL, "NaN accepted");

    float m[9], b[3];
    acal_to_inslib(k_true, m, b);
    CHECK(m[0] == k_true[ACAL_SX] && m[3] == k_true[ACAL_MXY] && m[7] == k_true[ACAL_MYZ] &&
              m[1] == 0.0f && m[2] == 0.0f && m[5] == 0.0f && b[2] == k_true[ACAL_BZ],
          "column-major layout");

    /* WGS84 normal gravity: 9.7803 at the equator, 9.8322 at the pole,
     * 9.8192 at 60 deg; 1000 m lowers it by 3.1 mm/s^2. */
    CHECK(fabsf(acal_gravity(0.0, 0.0f) - 9.78033f) < 1e-4f, "g equator %g", acal_gravity(0.0, 0.0f));
    CHECK(fabsf(acal_gravity(M_PI / 2, 0.0f) - 9.83218f) < 1e-4f, "g pole %g",
          acal_gravity(M_PI / 2, 0.0f));
    CHECK(fabsf(acal_gravity(M_PI / 3, 0.0f) - 9.81921f) < 2e-4f, "g 60 %g", acal_gravity(M_PI / 3, 0.0f));
    CHECK(fabsf(acal_gravity(0.0, 0.0f) - acal_gravity(0.0, 1000.0f) - 0.003086f) < 1e-5f, "free air");

    /* Coverage: six faces of a cube give 1/3; one direction gives 0. */
    acal_pose_t six[6];
    memset(six, 0, sizeof six);
    for (int j = 0; j < 6; j++) {
        six[j].acc[j / 2] = (j & 1) ? -9.8f : 9.8f;
    }
    CHECK(fabsf(acal_coverage(six, 6) - 1.0f / 3.0f) < 1e-5f, "cube coverage %g", acal_coverage(six, 6));
    CHECK(acal_coverage(six, 1) < 1e-6f, "single pose coverage %g", acal_coverage(six, 1));
    /* Two directions (a plane at most): still 0. */
    CHECK(acal_coverage(six, 2) < 1e-6f, "+-x coverage %g", acal_coverage(six, 2));
}

int main(void)
{
    test_sphere();
    test_level_only();
    test_prior_reject_misc();
    if (s_fail) {
        printf("%d check(s) failed\n", s_fail);
        return 1;
    }
    printf("all checks passed\n");
    return 0;
}
