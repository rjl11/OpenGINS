/*
 * Host test of firmware/app/imu_calib_fit.c (plans/imu-auto-calibration.md,
 * Phase 1). Build and run on any host with a C compiler:
 *
 *   gcc -O2 -Wall -Wextra -I firmware/app firmware/tools/test_imu_calib_fit.c \
 *       firmware/app/imu_calib_fit.c -lm -o test_imu_calib_fit && ./test_imu_calib_fit
 *
 * Exit code 0 = all checks passed.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "imu_calib_fit.h"

#define D2R 0.017453292519943295

static int s_fail;

#define CHECK(cond, ...)                         \
    do {                                         \
        if (!(cond)) {                           \
            printf("FAIL %s:%d: ", __FILE__, __LINE__); \
            printf(__VA_ARGS__);                 \
            printf("\n");                        \
            s_fail++;                            \
        }                                        \
    } while (0)

/* Deterministic Gaussian noise (Box-Muller on a LCG). */
static unsigned long s_seed = 12345;
static double urand(void)
{
    s_seed = s_seed * 6364136223846793005ull + 1442695040888963407ull;
    return ((double)(s_seed >> 11) + 0.5) / 9007199254740992.0;
}
static double grand(void)
{
    return sqrt(-2.0 * log(urand())) * cos(2.0 * M_PI * urand());
}

static const gcal_prior_t k_prior = {
    .t_ref_c = 25.0f,
    .sd_b0_rps = (float)(1.0 * D2R),
    .sd_k_rps_c = (float)(0.004 * D2R),
    .k_min_span_c = 3.0f,
};

static void test_recovery(void)
{
    const float b0[3] = { (float)(0.20 * D2R), (float)(-0.10 * D2R), (float)(0.05 * D2R) };
    const float k[3] = { (float)(0.003 * D2R), (float)(-0.002 * D2R), (float)(0.0010 * D2R) };
    const double sd = 0.002 * D2R; /* per chunk */
    gcal_stats_t s;
    gcal_stats_clear(&s);
    for (int n = 0; n < 200; n++) {
        float t = 5.0f + 30.0f * (float)n / 199.0f; /* 5 .. 35 degC */
        float b[3];
        for (int i = 0; i < 3; i++) {
            b[i] = b0[i] + k[i] * (t - k_prior.t_ref_c) + (float)(sd * grand());
        }
        CHECK(gcal_stats_add(&s, &k_prior, b, t, (float)(sd * sd)), "add failed");
    }
    gcal_result_t r;
    gcal_solve(&s, &k_prior, &r);
    for (int i = 0; i < 3; i++) {
        CHECK(fabsf(r.b0[i] - b0[i]) < 4.0f * r.sd_b0, "b0[%d] %g vs %g (sd %g)", i, r.b0[i], b0[i],
              r.sd_b0);
        CHECK(fabsf(r.k[i] - k[i]) < 4.0f * r.sd_k, "k[%d] %g vs %g (sd %g)", i, r.k[i], k[i], r.sd_k);
    }
    CHECK(r.k_fitted, "k not fitted over 30 C");
    CHECK(r.sd_k < 0.05f * k_prior.sd_k_rps_c, "sd_k %g did not shrink", r.sd_k);
    CHECK(s.n_chunks == 200 && s.t_min_c == 5.0f && fabsf(s.t_max_c - 35.0f) < 1e-4f,
          "range %g..%g n %u", s.t_min_c, s.t_max_c, s.n_chunks);
    const gcal_limits_t lim = { (float)(2.0 * D2R), (float)(0.02 * D2R) };
    CHECK(gcal_validate(&r, &lim) == NULL, "valid result rejected");
    CHECK(gcal_stats_validate(&s) == NULL, "valid stats rejected");
}

/* One temperature only: b0 is learned at that temperature, k stays at its
 * prior (zero mean, prior sigma). */
static void test_no_span(void)
{
    gcal_stats_t s;
    gcal_stats_clear(&s);
    const float b[3] = { (float)(0.1 * D2R), 0.0f, 0.0f };
    const double sd = 0.002 * D2R;
    for (int n = 0; n < 50; n++) {
        gcal_stats_add(&s, &k_prior, b, 25.0f, (float)(sd * sd));
    }
    gcal_result_t r;
    gcal_solve(&s, &k_prior, &r);
    CHECK(fabsf(r.b0[0] - b[0]) < 1e-3f * b[0] + 1e-9f, "b0 %g vs %g", r.b0[0], b[0]);
    CHECK(fabsf(r.k[0]) < 1e-9f, "k %g should stay 0", r.k[0]);
    CHECK(fabsf(r.sd_k - k_prior.sd_k_rps_c) < 1e-3f * k_prior.sd_k_rps_c, "sd_k %g vs prior %g",
          r.sd_k, k_prior.sd_k_rps_c);
}

/* The bench case of 2026-11-15: bias drifting in time over a 0.16 degC
 * span. k must stay 0, and b0 is the mean bias. */
static void test_drift_small_span(void)
{
    gcal_stats_t s;
    gcal_stats_clear(&s);
    const float t[3] = { 19.24f, 19.16f, 19.08f };
    const float y[3] = { 0.0019f, 0.0052f, 0.0123f }; /* deg/s */
    const double sd = 0.005 * D2R;
    for (int n = 0; n < 3; n++) {
        float b[3] = { 0.0f, (float)(y[n] * D2R), 0.0f };
        gcal_stats_add(&s, &k_prior, b, t[n], (float)(sd * sd));
    }
    gcal_result_t r;
    gcal_solve(&s, &k_prior, &r);
    CHECK(!r.k_fitted, "k fitted over a 0.16 C span");
    CHECK(r.k[1] == 0.0f, "k %g should be 0", r.k[1]);
    const float mean = (float)((0.0019 + 0.0052 + 0.0123) / 3.0 * D2R);
    CHECK(fabsf(r.b0[1] - mean) < 1e-3f * mean, "b0 %g vs mean %g", r.b0[1], mean);
}

static void test_prior_and_forget(void)
{
    gcal_stats_t s;
    gcal_stats_clear(&s);
    gcal_result_t r;
    gcal_solve(&s, &k_prior, &r);
    CHECK(r.b0[0] == 0.0f && r.k[0] == 0.0f, "empty stats must give the prior mean");
    CHECK(fabsf(r.sd_b0 - k_prior.sd_b0_rps) < 1e-6f * k_prior.sd_b0_rps, "empty sd_b0 %g", r.sd_b0);

    const float b[3] = { (float)(0.1 * D2R), 0.0f, 0.0f };
    for (int n = 0; n < 10; n++) {
        gcal_stats_add(&s, &k_prior, b, 20.0f + (float)n, (float)(1e-5 * 1e-5));
    }
    double info0 = s.info[0];
    gcal_stats_forget(&s, 0.5f);
    CHECK(fabs(s.info[0] - 0.5 * info0) < 1e-9 * info0, "forget did not halve the information");
    gcal_result_t r2;
    gcal_solve(&s, &k_prior, &r2);
    CHECK(fabsf(r2.b0[0] - b[0]) < 1e-3f * b[0], "forget moved the mean: %g", r2.b0[0]);
    gcal_stats_forget(&s, 0.0f); /* ignored */
    CHECK(fabs(s.info[0] - 0.5 * info0) < 1e-9 * info0, "factor 0 must be ignored");
}

static void test_reject(void)
{
    const gcal_limits_t lim = { (float)(2.0 * D2R), (float)(0.02 * D2R) };
    gcal_result_t r = { { 0 }, { 0 }, 1e-5f, 1e-6f, false };
    CHECK(gcal_validate(&r, &lim) == NULL, "zero result rejected");
    r.b0[1] = (float)(3.0 * D2R);
    CHECK(gcal_validate(&r, &lim) != NULL, "b0 3 deg/s accepted");
    r.b0[1] = 0.0f;
    r.k[2] = (float)(-0.05 * D2R);
    CHECK(gcal_validate(&r, &lim) != NULL, "k -0.05 deg/s/C accepted");
    r.k[2] = NAN;
    CHECK(gcal_validate(&r, &lim) != NULL, "NaN accepted");

    gcal_stats_t s;
    gcal_stats_clear(&s);
    float bad[3] = { NAN, 0.0f, 0.0f };
    CHECK(!gcal_stats_add(&s, &k_prior, bad, 25.0f, 1e-8f), "NaN bias added");
    float ok[3] = { 0.0f, 0.0f, 0.0f };
    CHECK(!gcal_stats_add(&s, &k_prior, ok, 25.0f, 0.0f), "zero variance added");
    CHECK(s.n_chunks == 0, "rejected chunks counted");
    s.info[0] = -1.0;
    CHECK(gcal_stats_validate(&s) != NULL, "negative information accepted");
}

/* Level, facing north: the Earth rate is (W cos lat, 0, -W sin lat) in FRD.
 * Facing east, the north component appears on -y. */
static void test_earth_rate(void)
{
    const double w = 7.2921151467e-5, lat = 60.0 * D2R;
    float out[3];
    const float north[3] = { 0.0f, 0.0f, 0.0f };
    gcal_earth_rate_body(north, lat, out);
    CHECK(fabs(out[0] - w * cos(lat)) < 1e-9 && fabs(out[1]) < 1e-9 && fabs(out[2] + w * sin(lat)) < 1e-9,
          "north: %g %g %g", out[0], out[1], out[2]);
    const float east[3] = { 0.0f, 0.0f, (float)(90.0 * D2R) };
    gcal_earth_rate_body(east, lat, out);
    CHECK(fabs(out[0]) < 1e-9 && fabs(out[1] + w * cos(lat)) < 1e-9 && fabs(out[2] + w * sin(lat)) < 1e-9,
          "east: %g %g %g", out[0], out[1], out[2]);
    /* Upside down (roll 180): the vertical component flips. */
    const float inv[3] = { (float)M_PI, 0.0f, 0.0f };
    gcal_earth_rate_body(inv, lat, out);
    CHECK(fabs(out[2] - w * sin(lat)) < 1e-9, "inverted z: %g", out[2]);
    /* Magnitude is always W. */
    const float any[3] = { 0.3f, -0.7f, 2.1f };
    gcal_earth_rate_body(any, lat, out);
    double m = sqrt((double)out[0] * out[0] + (double)out[1] * out[1] + (double)out[2] * out[2]);
    CHECK(fabs(m - w) < 1e-9, "magnitude %g", m);
}

int main(void)
{
    test_recovery();
    test_no_span();
    test_drift_small_span();
    test_prior_and_forget();
    test_reject();
    test_earth_rate();
    if (s_fail) {
        printf("%d check(s) failed\n", s_fail);
        return 1;
    }
    printf("all checks passed\n");
    return 0;
}
