#include "imu_calib_rot.h"

#include <math.h>
#include <string.h>

#include "imu_calib_lsq.h"

static int uidx(int r, int c)
{
    /* Upper triangle of a 9x9, row by row. */
    if (r > c) {
        int t = r;
        r = c;
        c = t;
    }
    return r * ROT_NPAR - r * (r - 1) / 2 + (c - r);
}

static void exp_d(const double phi[3], double r[9])
{
    const double th = sqrt(phi[0] * phi[0] + phi[1] * phi[1] + phi[2] * phi[2]);
    double a, b;
    if (th < 1e-8) {
        a = 1.0 - th * th / 6.0;
        b = 0.5 - th * th / 24.0;
    } else {
        a = sin(th) / th;
        b = (1.0 - cos(th)) / (th * th);
    }
    const double x = phi[0], y = phi[1], z = phi[2];
    /* R = I + a [phi]x + b [phi]x^2 */
    r[0] = 1.0 - b * (y * y + z * z);
    r[1] = -a * z + b * x * y;
    r[2] = a * y + b * x * z;
    r[3] = a * z + b * x * y;
    r[4] = 1.0 - b * (x * x + z * z);
    r[5] = -a * x + b * y * z;
    r[6] = -a * y + b * x * z;
    r[7] = a * x + b * y * z;
    r[8] = 1.0 - b * (x * x + y * y);
}

void rot_exp(const float phi[3], float r[9])
{
    const double p[3] = { phi[0], phi[1], phi[2] };
    double rd[9];
    exp_d(p, rd);
    for (int i = 0; i < 9; i++) {
        r[i] = (float)rd[i];
    }
}

void rot_preint_reset(rot_preint_t *p)
{
    memset(p, 0, sizeof *p);
    p->dr[0] = p->dr[4] = p->dr[8] = 1.0;
}

float rot_preint_angle(const rot_preint_t *p)
{
    double c = 0.5 * (p->dr[0] + p->dr[4] + p->dr[8] - 1.0);
    c = c > 1.0 ? 1.0 : c < -1.0 ? -1.0 : c;
    return (float)acos(c);
}

void rot_preint_step(rot_preint_t *p, const float u[3], const float *m0, float dt)
{
    double w[3];
    for (int i = 0; i < 3; i++) {
        w[i] = m0 ? (double)m0[i * 3 + 0] * u[0] + (double)m0[i * 3 + 1] * u[1] +
                        (double)m0[i * 3 + 2] * u[2]
                  : (double)u[i];
    }
    const double phi[3] = { w[0] * dt, w[1] * dt, w[2] * dt };
    double e[9];
    exp_d(phi, e);

    /* J <- E^T J + A dt, A[i][3i+k] = u_k (dw_i = sum_k dM_ik u_k). The
     * right Jacobian of the step is taken as I: steps are about 1e-3 rad. */
    double jn[3 * ROT_NPAR];
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < ROT_NPAR; c++) {
            jn[r * ROT_NPAR + c] = e[0 * 3 + r] * p->j[0 * ROT_NPAR + c] +
                                   e[1 * 3 + r] * p->j[1 * ROT_NPAR + c] +
                                   e[2 * 3 + r] * p->j[2 * ROT_NPAR + c];
        }
    }
    for (int i = 0; i < 3; i++) {
        for (int k = 0; k < 3; k++) {
            jn[i * ROT_NPAR + 3 * i + k] += (double)u[k] * dt;
        }
    }
    memcpy(p->j, jn, sizeof jn);

    /* dR <- dR E */
    double rn[9];
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            rn[r * 3 + c] = p->dr[r * 3 + 0] * e[0 * 3 + c] + p->dr[r * 3 + 1] * e[1 * 3 + c] +
                            p->dr[r * 3 + 2] * e[2 * 3 + c];
        }
    }
    memcpy(p->dr, rn, sizeof rn);
    p->t_s += dt;
}

void rot_stats_clear(rot_stats_t *s)
{
    memset(s, 0, sizeof *s);
}

const char *rot_stats_validate(const rot_stats_t *s)
{
    for (int k = 0; k < 45; k++) {
        if (!isfinite(s->h[k])) {
            return "gyro rotation statistics not finite";
        }
    }
    for (int k = 0; k < ROT_NPAR; k++) {
        if (!isfinite(s->g[k]) || s->h[uidx(k, k)] < 0.0) {
            return "gyro rotation statistics not finite";
        }
    }
    if (!isfinite(s->w) || !isfinite(s->rr) || s->w < 0.0) {
        return "gyro rotation statistics not finite";
    }
    return NULL;
}

/* One linear observation row: residual r0, gradient hrow, weight w. */
static void add_row(rot_stats_t *s, double r0, const double hrow[ROT_NPAR], double w)
{
    for (int a = 0; a < ROT_NPAR; a++) {
        s->g[a] += w * hrow[a] * r0;
        for (int b = a; b < ROT_NPAR; b++) {
            s->h[uidx(a, b)] += w * hrow[a] * hrow[b];
        }
    }
    s->rr += w * r0 * r0;
    s->w += w;
}

bool rot_add_rotation(rot_stats_t *s, const rot_preint_t *p, const float g1[3], const float g2[3],
                      float sd)
{
    double n1 = sqrt((double)g1[0] * g1[0] + (double)g1[1] * g1[1] + (double)g1[2] * g1[2]);
    double n2 = sqrt((double)g2[0] * g2[0] + (double)g2[1] * g2[1] + (double)g2[2] * g2[2]);
    if (!(n1 > 1e-6) || !(n2 > 1e-6) || !isfinite(n1) || !isfinite(n2) || !(sd > 0.0f)) {
        return false;
    }
    const double u1[3] = { g1[0] / n1, g1[1] / n1, g1[2] / n1 };
    const double u2[3] = { g2[0] / n2, g2[1] / n2, g2[2] / n2 };
    /* v = dR^T u1, the first gravity direction seen in the second pose. */
    double v[3];
    for (int i = 0; i < 3; i++) {
        v[i] = p->dr[0 * 3 + i] * u1[0] + p->dr[1 * 3 + i] * u1[1] + p->dr[2 * 3 + i] * u1[2];
    }
    /* r(dM) = v - u2 + [v]x J dM */
    const double vx[3][3] = { { 0, -v[2], v[1] }, { v[2], 0, -v[0] }, { -v[1], v[0], 0 } };
    const double w = 1.0 / ((double)sd * (double)sd);
    for (int i = 0; i < 3; i++) {
        double hrow[ROT_NPAR];
        for (int c = 0; c < ROT_NPAR; c++) {
            hrow[c] = vx[i][0] * p->j[0 * ROT_NPAR + c] + vx[i][1] * p->j[1 * ROT_NPAR + c] +
                      vx[i][2] * p->j[2 * ROT_NPAR + c];
        }
        add_row(s, v[i] - u2[i], hrow, w);
    }
    s->n_rot++;
    return true;
}

bool rot_add_turn(rot_stats_t *s, float pred, float meas, const float h[ROT_NPAR], float sd)
{
    if (!isfinite(pred) || !isfinite(meas) || !(sd > 0.0f)) {
        return false;
    }
    double hrow[ROT_NPAR];
    for (int c = 0; c < ROT_NPAR; c++) {
        if (!isfinite(h[c])) {
            return false;
        }
        hrow[c] = h[c];
    }
    add_row(s, (double)pred - (double)meas, hrow, 1.0 / ((double)sd * (double)sd));
    s->n_turn++;
    return true;
}

void rot_solve(const rot_stats_t *s, float sd_scale, float sd_mis, rot_result_t *r)
{
    double a[ROT_NPAR * ROT_NPAR], b[ROT_NPAR], inv[ROT_NPAR];
    for (int i = 0; i < ROT_NPAR; i++) {
        for (int k = 0; k < ROT_NPAR; k++) {
            a[i * ROT_NPAR + k] = s->h[uidx(i, k)];
        }
        const double sd = (i % 4 == 0) ? sd_scale : sd_mis; /* 0, 4, 8: diagonal */
        a[i * ROT_NPAR + i] += 1.0 / (sd * sd);
        b[i] = -s->g[i];
    }
    r->ok = lsq_spd_solve(ROT_NPAR, a, b, inv);
    for (int i = 0; i < ROT_NPAR; i++) {
        r->dm[i] = r->ok ? (float)b[i] : 0.0f;
        r->sd[i] = r->ok ? (float)sqrt(inv[i]) : NAN;
    }
}

int rot_accept(const rot_result_t *r, const float *m0, float max_sd_scale, float max_sd_mis,
               float m_out[9])
{
    int taken = 0;
    for (int i = 0; i < 9; i++) {
        const float base = m0 ? m0[i] : (i % 4 == 0 ? 1.0f : 0.0f);
        const float lim = (i % 4 == 0) ? max_sd_scale : max_sd_mis;
        if (r->ok && r->sd[i] <= lim) {
            m_out[i] = base + r->dm[i];
            taken++;
        } else {
            m_out[i] = base;
        }
    }
    return taken;
}

const char *rot_validate(const float m[9], float max_scale_dev, float max_mis_rad)
{
    for (int i = 0; i < 9; i++) {
        if (!isfinite(m[i])) {
            return "not finite";
        }
        if (i % 4 == 0) {
            if (fabsf(m[i] - 1.0f) > max_scale_dev) {
                return "gyro scale out of range";
            }
        } else if (fabsf(m[i]) > max_mis_rad) {
            return "gyro misalignment out of range";
        }
    }
    return NULL;
}
