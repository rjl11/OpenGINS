/* speed_ppm: edge-timed raw pulses per metre (speed_ppm.h). */
#include "speed_ppm.h"

#include <string.h>

#define SPEED_PPM_MAX_EPOCH_GAP_US 500000u

void speed_ppm_init(speed_ppm_t *s, const speed_ppm_config_t *cfg)
{
    memset(s, 0, sizeof *s);
    s->cfg = *cfg;
}

void speed_ppm_reset(speed_ppm_t *s)
{
    s->run = false;
    s->have_epoch = false;
}

void speed_ppm_edge(speed_ppm_t *s, uint64_t t_us)
{
    s->edge_t[s->n_edges % SPEED_PPM_RING] = t_us;
    s->n_edges++;
}

/* Newest edge in (t_lo, t_hi], as its index; false if none is in the ring. */
static bool newest_edge(const speed_ppm_t *s, uint64_t t_lo, uint64_t t_hi, uint32_t *idx)
{
    uint32_t n = s->n_edges < SPEED_PPM_RING ? s->n_edges : SPEED_PPM_RING;
    for (uint32_t k = 1; k <= n; k++) {
        uint32_t i = s->n_edges - k;
        uint64_t t = s->edge_t[i % SPEED_PPM_RING];
        if (t <= t_lo) {
            return false;
        }
        if (t <= t_hi) {
            *idx = i;
            return true;
        }
    }
    return false;
}

bool speed_ppm_gnss(speed_ppm_t *s, uint64_t t_us, double speed_mps, bool usable, speed_ppm_result_t *out)
{
    if (!usable || speed_mps < (double)s->cfg.min_mps) {
        speed_ppm_reset(s);
        return false;
    }
    if (!s->have_epoch || t_us <= s->t_prev || t_us - s->t_prev > SPEED_PPM_MAX_EPOCH_GAP_US) {
        /* First good epoch, or a gap: distance starts here, and so does the
         * run (no edge before this epoch can be placed). */
        s->run = false;
        s->have_epoch = true;
        s->t_prev = t_us;
        s->d_prev = 0.0;
        s->v_prev = speed_mps;
        return false;
    }

    const double dt = (double)(t_us - s->t_prev) * 1e-6;
    const double d = s->d_prev + 0.5 * (s->v_prev + speed_mps) * dt;
    bool done = false;

    uint32_t j;
    if (newest_edge(s, s->t_prev, t_us, &j)) {
        const uint64_t te = s->edge_t[j % SPEED_PPM_RING];
        const double de = s->d_prev + (d - s->d_prev) * ((double)(te - s->t_prev) / (double)(t_us - s->t_prev));
        bool start = !s->run;
        if (s->run) {
            const uint32_t pulses = j - s->idx0;
            const float span = (float)((double)(te - s->t0) * 1e-6);
            const double dist = de - s->d0;
            if (span >= s->cfg.min_span_s && pulses >= s->cfg.min_edges && dist > 0.0) {
                out->ppm = (float)((double)pulses / dist);
                out->mps = (float)(dist / (double)span);
                out->span_s = span;
                out->pulses = pulses;
                out->t_end_us = te;
                done = true;
                start = true; /* the next run starts at this edge */
            }
        }
        if (start) {
            s->run = true;
            s->idx0 = j;
            s->t0 = te;
            s->d0 = de;
        }
    }
    s->t_prev = t_us;
    s->d_prev = d;
    s->v_prev = speed_mps;
    return done;
}
