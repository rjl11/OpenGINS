#include "pole_mount_fit.h"

#include <math.h>
#include <string.h>

#include "imu_calib_lsq.h"

#define R2D 57.29577951308232
#define D2R 0.017453292519943295

void pm_axis_default(float u[3])
{
    u[0] = 0.0f;
    u[1] = 0.0f;
    u[2] = 1.0f;
}

void pm_prior_init(pm_prior_t *p, float h_m, float tilt_sd_deg, float len_sd_m, float fix_sd_m, float corr,
                   float ep_rms_max_m, float ep_rms_rel)
{
    memset(p, 0, sizeof *p);
    double h = h_m > 0.1f ? h_m : 0.1;
    p->d0[2] = -h;
    p->sd[0] = p->sd[1] = h * tan(tilt_sd_deg * D2R);
    p->sd[2] = len_sd_m;
    p->fix_sd_m = fix_sd_m;
    p->corr = corr > 1.0f ? corr : 1.0;
    p->ep_rms_max_m = ep_rms_max_m;
    p->ep_rms_rel = ep_rms_rel;
}

static double rss_at(const double N[3][3], const double b[3], double c, const double d[3])
{
    double q = c;
    for (int i = 0; i < 3; i++) {
        q -= 2.0 * b[i] * d[i];
        for (int j = 0; j < 3; j++) q += d[i] * N[i][j] * d[j];
    }
    return q > 0.0 ? q : 0.0;
}

void pm_solve(const pm_stats_t *s, const pm_prior_t *p, pm_result_t *r)
{
    memset(r, 0, sizeof *r);
    double w = 1.0 / (p->fix_sd_m * p->fix_sd_m * p->corr);
    double a[9], x[3], inv[3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) a[i * 3 + j] = w * s->N[i][j];
        a[i * 3 + i] += 1.0 / (p->sd[i] * p->sd[i]);
        x[i] = w * s->b[i] + p->d0[i] / (p->sd[i] * p->sd[i]);
    }
    if (!lsq_spd_solve(3, a, x, inv)) {
        for (int i = 0; i < 3; i++) r->d[i] = p->d0[i];
        pm_axis_default(r->u);
        return;
    }
    double L = sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]);
    if (!(L > 0.05)) {
        for (int i = 0; i < 3; i++) r->d[i] = p->d0[i];
        pm_axis_default(r->u);
        return;
    }
    r->ok = true;
    for (int i = 0; i < 3; i++) {
        r->d[i] = x[i];
        r->sd[i] = sqrt(inv[i]);
        r->u[i] = (float)(-x[i] / L);
    }
    r->len_m = (float)L;
    r->len_sd_m = (float)r->sd[2];
    r->tilt_deg = (float)(atan2(hypot(r->u[0], r->u[1]), r->u[2]) * R2D);
    r->azim_deg = (float)(atan2(r->u[1], r->u[0]) * R2D);
    double sh = r->sd[0] > r->sd[1] ? r->sd[0] : r->sd[1];
    r->tilt_sd_deg = (float)(atan2(sh, L) * R2D);
    r->rms_m = s->n_fix > 0 ? (float)sqrt(rss_at(s->N, s->b, s->c, x) / (2.0 * s->n_fix)) : 0.0f;
}

float pm_episode_psi(const pole_episode_t *e, const double d[3])
{
    if (e->n == 0) return 0.0f;
    double n = e->n, su[2], M[2][2];
    for (int j = 0; j < 2; j++) {
        su[j] = 0.0;
        for (int c = 0; c < 3; c++) su[j] += e->sg[j][c] * d[c];
    }
    for (int j = 0; j < 2; j++)
        for (int k = 0; k < 2; k++) {
            double m = 0.0;
            for (int c = 0; c < 3; c++) m += d[c] * e->sgp[c][j][k];
            M[j][k] = m - su[j] * e->sp[k] / n;
        }
    /* p ~ t + Q u, Q = [c -s; s c]: maximise sum p.Q u. */
    return (float)atan2(M[0][1] - M[1][0], M[0][0] + M[1][1]);
}

void pm_episode_reduce(const pole_episode_t *e, float psi, double N[3][3], double b[3], double *c)
{
    memset(N, 0, sizeof(double) * 9);
    memset(b, 0, sizeof(double) * 3);
    *c = 0.0;
    if (e->n == 0) return;
    double n = e->n, cs = cos(psi), sn = sin(psi);
    /* q_i = Q^T p_i, Q^T = [c s; -s c]. */
    const double Qt[2][2] = { { cs, sn }, { -sn, cs } };
    double sq[2];
    for (int j = 0; j < 2; j++) sq[j] = Qt[j][0] * e->sp[0] + Qt[j][1] * e->sp[1];
    for (int a = 0; a < 3; a++) {
        double gq = 0.0;
        for (int j = 0; j < 2; j++)
            for (int k = 0; k < 2; k++) gq += Qt[j][k] * e->sgp[a][j][k];
        b[a] = gq - (e->sg[0][a] * sq[0] + e->sg[1][a] * sq[1]) / n;
        for (int k = 0; k < 3; k++) N[a][k] = e->sgg[a][k] - (e->sg[0][a] * e->sg[0][k] + e->sg[1][a] * e->sg[1][k]) / n;
    }
    *c = e->spp - (e->sp[0] * e->sp[0] + e->sp[1] * e->sp[1]) / n;
}

bool pm_add_episode(pm_stats_t *s, const pole_episode_t *e, const pm_prior_t *p, float *rms_out)
{
    if (rms_out) *rms_out = -1.0f;
    if (e->n < 3) return false;
    pm_result_t r;
    pm_solve(s, p, &r);
    double d[3] = { r.d[0], r.d[1], r.d[2] };
    /* With a baseline, an episode must also fit about as well as the rest:
     * a slipping tip partly hides in the axis when it is alone. */
    float gate = p->ep_rms_max_m;
    if (s->n_ep >= PM_REL_MIN_EP && p->ep_rms_rel > 0.0f) {
        float rel = p->ep_rms_rel * r.rms_m;
        if (rel < PM_REL_FLOOR_M) rel = PM_REL_FLOOR_M;
        if (rel < gate) gate = rel;
    }
    double Nk[3][3], bk[3], ck = 0.0;
    pm_stats_t t = *s;
    /* Twice: the yaw against the current axis, then against the axis with
     * this episode in. */
    for (int it = 0; it < 2; it++) {
        float psi = pm_episode_psi(e, d);
        pm_episode_reduce(e, psi, Nk, bk, &ck);
        t = *s;
        for (int i = 0; i < 3; i++) {
            t.b[i] += bk[i];
            for (int j = 0; j < 3; j++) t.N[i][j] += Nk[i][j];
        }
        t.c += ck;
        t.n_fix += e->n;
        pm_solve(&t, p, &r);
        for (int i = 0; i < 3; i++) d[i] = r.d[i];
    }
    float rms = (float)sqrt(rss_at(Nk, bk, ck, d) / (2.0 * e->n));
    if (rms_out) *rms_out = rms;
    if (!(rms <= gate)) return false;
    t.n_ep = s->n_ep + 1;
    t.twist_deg = s->twist_deg + e->twist_deg;
    *s = t;
    return true;
}
