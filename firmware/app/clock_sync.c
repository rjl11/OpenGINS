#include "clock_sync.h"

#include <math.h>
#include <string.h>

void clock_sync_init(clock_sync_t *c) { memset(c, 0, sizeof *c); }

uint32_t clock_sync_request(clock_sync_t *c, uint64_t now)
{
    c->seq++;
    c->pending = true;
    c->t_req = now;
    c->requests++;
    return c->seq;
}

bool clock_sync_pending(clock_sync_t *c, uint64_t now)
{
    /* Signed: now may be a loop-pass time a little before t_req. */
    if (c->pending && (int64_t)(now - c->t_req) > (int64_t)CLOCK_SYNC_TIMEOUT_US) {
        c->pending = false;
        c->timeouts++;
    }
    return c->pending;
}

static void restart(clock_sync_t *c)
{
    c->n = 0;
    c->head = 0;
    c->have = false;
    c->resets++;
}

/* The line through the replies with the shortest round trips. */
static void fit(clock_sync_t *c)
{
    uint32_t rmin = UINT32_MAX;
    for (uint32_t i = 0; i < c->n; i++) {
        if (c->s[i].rtt < rmin) rmin = c->s[i].rtt;
    }
    const uint32_t lim = rmin + CLOCK_SYNC_RTT_SLACK_US;
    /* Centre on the newest sample: small numbers in the doubles. */
    const clock_sync_sample_t *newest = &c->s[(c->head + CLOCK_SYNC_N - 1u) % CLOCK_SYNC_N];
    const uint64_t t0 = newest->t;
    const int64_t  o0 = newest->off;
    double sw = 0, st = 0, so = 0, stt = 0, sto = 0;
    uint64_t tmin = UINT64_MAX, tmax = 0;
    uint32_t m = 0;
    for (uint32_t i = 0; i < c->n; i++) {
        const clock_sync_sample_t *x = &c->s[i];
        if (x->rtt > lim) continue;
        const double t = (double)(int64_t)(x->t - t0);
        const double o = (double)(x->off - o0);
        sw += 1; st += t; so += o; stt += t * t; sto += t * o;
        if (x->t < tmin) tmin = x->t;
        if (x->t > tmax) tmax = x->t;
        m++;
    }
    double b = 0.0, a;
    const double den = sw * stt - st * st;
    if (m >= 2 && tmax - tmin >= CLOCK_SYNC_MIN_SPAN_US && den > 0.0) {
        b = (sw * sto - st * so) / den;
        if (b > CLOCK_SYNC_MAX_DRIFT) b = CLOCK_SYNC_MAX_DRIFT;
        if (b < -CLOCK_SYNC_MAX_DRIFT) b = -CLOCK_SYNC_MAX_DRIFT;
    }
    a = (so - b * st) / sw; /* the line at t0 */
    double r2 = 0;
    for (uint32_t i = 0; i < c->n; i++) {
        const clock_sync_sample_t *x = &c->s[i];
        if (x->rtt > lim) continue;
        const double e = (double)(x->off - o0) - (a + b * (double)(int64_t)(x->t - t0));
        r2 += e * e;
    }
    c->have = true;
    c->a = (double)o0 + a;
    c->b = b;
    c->t_ref = t0;
    c->rtt_min = rmin;
    c->used = m;
    c->resid_us = (float)sqrt(r2 / (double)m);
}

double clock_sync_offset_us(const clock_sync_t *c, uint64_t now)
{
    return c->a + c->b * (double)(int64_t)(now - c->t_ref);
}

double clock_sync_drift_ppm(const clock_sync_t *c) { return c->b * 1e6; }

bool clock_sync_reply(clock_sync_t *c, uint32_t seq, uint32_t turnaround_us, uint64_t unit_rx_us, uint64_t now)
{
    if (!c->pending || seq != c->seq || now < c->t_req) {
        c->stray++;
        return false;
    }
    c->pending = false;
    c->replies++;
    const uint64_t round = now - c->t_req;
    if (turnaround_us > round || round - turnaround_us > CLOCK_SYNC_MAX_RTT_US) {
        c->too_slow++;
        return false;
    }
    const uint32_t rtt = (uint32_t)(round - turnaround_us);
    const uint64_t t = c->t_req + rtt / 2u;
    const int64_t  off = (int64_t)(t - unit_rx_us);

    if (c->have) {
        const double e = (double)off - clock_sync_offset_us(c, t);
        if (fabs(e) > CLOCK_SYNC_RESET_US || unit_rx_us < c->unit_last) {
            restart(c);
        }
    }
    c->s[c->head] = (clock_sync_sample_t){ .t = t, .off = off, .rtt = rtt };
    c->head = (c->head + 1u) % CLOCK_SYNC_N;
    if (c->n < CLOCK_SYNC_N) c->n++; /* restart() sets head 0: [0, n) are valid */
    c->t_last = now;
    c->unit_last = unit_rx_us;
    fit(c);
    return true;
}

bool clock_sync_valid(const clock_sync_t *c, uint64_t now)
{
    /* Signed: the last reply may be stamped a little after the caller's now. */
    return c->have && (int64_t)(now - c->t_last) < (int64_t)CLOCK_SYNC_STALE_US;
}

bool clock_sync_to_pico(const clock_sync_t *c, uint32_t unit_us, uint64_t now, uint64_t *pico_us)
{
    if (!clock_sync_valid(c, now)) {
        return false;
    }
    const uint64_t full = c->unit_last + (uint64_t)(int64_t)(int32_t)(unit_us - (uint32_t)c->unit_last);
    /* The offset at the frame's own (Pico) time; one step is enough at
     * these drifts. */
    const uint64_t approx = full + (uint64_t)(int64_t)llround(c->a);
    *pico_us = full + (uint64_t)(int64_t)llround(clock_sync_offset_us(c, approx));
    return true;
}
