/* Host test for firmware/app/speed_ppm.c (edge-timed raw pulses per metre).
 * Build and run from the repo root:
 *   gcc -O2 -Wall -Wextra -I firmware/app firmware/tools/test_speed_ppm.c firmware/app/speed_ppm.c -lm -o temp/test_speed_ppm && temp/test_speed_ppm
 */
#include <math.h>
#include <stdio.h>

#include "speed_ppm.h"

static int s_fail;
static int s_overlap; /* results whose run covered the slow second */

static void check(int ok, const char *what)
{
    printf("%s  %s\n", ok ? "ok  " : "FAIL", what);
    if (!ok) s_fail = 1;
}

static const speed_ppm_config_t k_cfg = { .min_mps = 1.0f, .min_span_s = 5.0f, .min_edges = 10 };

/* Drives at speed v(t) for dur_s with a pickup at ppm pulses per metre (edge
 * positions offset by phase_m), GNSS epochs at 20 Hz, edges given before the
 * epoch that follows them. Optional: GNSS speed drops below the gate from
 * t_slow_s for 1 s. Returns the number of results; the last one in *last,
 * the largest |ppm error| in *worst. */
typedef double (*speed_fn)(double t);

static int drive(speed_fn v, double dur_s, double ppm, double phase_m, double t_slow_s, speed_ppm_result_t *last,
                 double *worst)
{
    speed_ppm_t s;
    speed_ppm_init(&s, &k_cfg);
    const double dt = 1e-4; /* integrate the truth at 10 kHz */
    double x = 0.0;
    double next_edge = (1.0 - phase_m * ppm) / ppm;
    int n = 0;
    *worst = 0.0;
    uint64_t next_epoch_us = 50000;
    for (double t = 0.0; t < dur_s; t += dt) {
        x += v(t) * dt;
        while (x >= next_edge) {
            speed_ppm_edge(&s, (uint64_t)llround(t * 1e6));
            next_edge += 1.0 / ppm;
        }
        const uint64_t now = (uint64_t)llround(t * 1e6);
        if (now >= next_epoch_us) {
            const double te = (double)next_epoch_us * 1e-6;
            double gv = v(te);
            if (t_slow_s >= 0.0 && te >= t_slow_s && te < t_slow_s + 1.0) gv = 0.5;
            speed_ppm_result_t r;
            if (speed_ppm_gnss(&s, next_epoch_us, gv, true, &r)) {
                n++;
                *last = r;
                const double t_end = (double)r.t_end_us * 1e-6, t_start = t_end - (double)r.span_s;
                if (t_slow_s >= 0.0 && t_start < t_slow_s + 1.0 && t_end > t_slow_s) s_overlap++;
                double e = fabs(r.ppm - ppm) / ppm;
                if (e > *worst) *worst = e;
            }
            next_epoch_us += 50000;
        }
    }
    return n;
}

static double v_const(double t) { (void)t; return 2.0; }
static double v_varying(double t) { return 2.0 + 0.5 * sin(t * 0.4); }
static double v_fast(double t) { (void)t; return 4.0; }

int main(void)
{
    speed_ppm_result_t r = { 0 };
    double worst;
    int n;

    /* Coarse Hall pickup: 1.9 pulses/m at 2 m/s, 3.8 edges/s. */
    n = drive(v_const, 60.0, 1.9, 0.13, -1.0, &r, &worst);
    printf("     coarse steady: %d results, last %.4f pulses/m over %.2f s, %u pulses, worst err %.2e\n", n,
           (double)r.ppm, (double)r.span_s, (unsigned)r.pulses, worst);
    check(n >= 9 && n <= 12, "coarse steady: one result per ~5 s run");
    check(worst < 1e-3, "coarse steady: within 0.1 %");
    check(r.span_s >= 5.0f && r.pulses >= 10, "coarse steady: run gates held");

    /* Speed changing along the run. */
    n = drive(v_varying, 120.0, 1.9, 0.37, -1.0, &r, &worst);
    printf("     coarse varying: %d results, worst err %.2e\n", n, worst);
    check(n >= 15, "coarse varying: results");
    check(worst < 2e-3, "coarse varying: within 0.2 %");

    /* ISO 11786: 130 pulses/m at 4 m/s, 520 edges/s, more than the ring per
     * epoch. */
    n = drive(v_fast, 30.0, 130.0, 0.0, -1.0, &r, &worst);
    printf("     ISO 11786: %d results, last %.3f pulses/m, worst err %.2e\n", n, (double)r.ppm, worst);
    check(n >= 4, "ISO 11786: results");
    check(worst < 1e-3, "ISO 11786: within 0.1 %");

    /* A slow second in the middle starts the run over. */
    n = drive(v_const, 30.0, 1.9, 0.0, 12.0, &r, &worst);
    printf("     slow gap: %d results, worst err %.2e\n", n, worst);
    check(n >= 3 && s_overlap == 0, "slow gap: no run spans the slow second");
    check(worst < 1e-3, "slow gap: within 0.1 %");

    /* Unusable or slow GNSS gives nothing. */
    {
        speed_ppm_t s;
        speed_ppm_init(&s, &k_cfg);
        int got = 0;
        for (uint64_t t = 50000; t < 30000000; t += 50000) {
            speed_ppm_edge(&s, t - 1000);
            speed_ppm_result_t rr;
            got += speed_ppm_gnss(&s, t, 2.0, false, &rr);
        }
        check(got == 0, "unusable GNSS: no result");
    }

    printf(s_fail ? "FAILED\n" : "all passed\n");
    return s_fail;
}
