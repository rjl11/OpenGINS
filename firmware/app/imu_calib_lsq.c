#include "imu_calib_lsq.h"

#include <math.h>
#include <string.h>

#define MAX_ITER     25
#define MAX_HALVINGS 8

/* In-place Cholesky of the n x n SPD matrix a (row-major, lower triangle). */
static bool lsq_cholesky(int n, double *a)
{
    for (int j = 0; j < n; j++) {
        double d = a[j * n + j];
        for (int k = 0; k < j; k++) {
            d -= a[j * n + k] * a[j * n + k];
        }
        if (!(d > 0.0)) {
            return false;
        }
        a[j * n + j] = sqrt(d);
        for (int i = j + 1; i < n; i++) {
            double s = a[i * n + j];
            for (int k = 0; k < j; k++) {
                s -= a[i * n + k] * a[j * n + k];
            }
            a[i * n + j] = s / a[j * n + j];
        }
    }
    return true;
}

/* Solves L L^T x = b with the factor from lsq_cholesky (b overwritten). */
static void lsq_chol_subst(int n, const double *l, double *b)
{
    for (int i = 0; i < n; i++) {
        double s = b[i];
        for (int k = 0; k < i; k++) {
            s -= l[i * n + k] * b[k];
        }
        b[i] = s / l[i * n + i];
    }
    for (int i = n - 1; i >= 0; i--) {
        double s = b[i];
        for (int k = i + 1; k < n; k++) {
            s -= l[k * n + i] * b[k];
        }
        b[i] = s / l[i * n + i];
    }
}

bool lsq_spd_solve(int n, double *a, double *b, double *inv_diag)
{
    if (!lsq_cholesky(n, a)) {
        return false;
    }
    lsq_chol_subst(n, a, b);
    if (inv_diag) {
        for (int i = 0; i < n; i++) {
            double e[LSQ_MAX_PAR] = { 0 };
            e[i] = 1.0;
            lsq_chol_subst(n, a, e);
            inv_diag[i] = e[i];
        }
    }
    return true;
}

static double cost(int np, const double *p, const double *p0, const double *sd, int nobs,
                   lsq_resid_fn f, void *ctx)
{
    double c = 0.0;
    for (int j = 0; j < nobs; j++) {
        double r = f(ctx, j, p, NULL);
        c += r * r;
    }
    for (int i = 0; i < np; i++) {
        double d = (p[i] - p0[i]) / sd[i];
        c += d * d;
    }
    return c;
}

/* H = J^T J + diag(1/sd^2), g = -(J^T r + (p - p0)/sd^2). */
static void normal_eq(int np, const double *p, const double *p0, const double *sd, int nobs,
                      lsq_resid_fn f, void *ctx, double *h, double *g)
{
    memset(h, 0, sizeof(double) * (size_t)(np * np));
    for (int i = 0; i < np; i++) {
        h[i * np + i] = 1.0 / (sd[i] * sd[i]);
        g[i] = -(p[i] - p0[i]) / (sd[i] * sd[i]);
    }
    for (int j = 0; j < nobs; j++) {
        double jac[LSQ_MAX_PAR];
        double r = f(ctx, j, p, jac);
        for (int a = 0; a < np; a++) {
            g[a] -= jac[a] * r;
            for (int b = 0; b <= a; b++) {
                h[a * np + b] += jac[a] * jac[b];
            }
        }
    }
    for (int a = 0; a < np; a++) {
        for (int b = a + 1; b < np; b++) {
            h[a * np + b] = h[b * np + a];
        }
    }
}

void lsq_solve(int np, const double *p0, const double *sd, const double *start, int nobs,
               lsq_resid_fn f, void *ctx, double *p_out, double *sd_out, lsq_info_t *info)
{
    double p[LSQ_MAX_PAR], h[LSQ_MAX_PAR * LSQ_MAX_PAR], g[LSQ_MAX_PAR];
    memcpy(p, start ? start : p0, sizeof(double) * (size_t)np);

    double c = cost(np, p, p0, sd, nobs, f, ctx);
    bool converged = false;
    int it = 0;
    for (; it < MAX_ITER && !converged; it++) {
        normal_eq(np, p, p0, sd, nobs, f, ctx, h, g);
        if (!lsq_cholesky(np, h)) {
            break;
        }
        lsq_chol_subst(np, h, g);
        /* Step halving keeps the cost falling. */
        double step = 1.0, pn[LSQ_MAX_PAR], cn = c;
        int k = 0;
        for (; k < MAX_HALVINGS; k++, step *= 0.5) {
            for (int i = 0; i < np; i++) {
                pn[i] = p[i] + step * g[i];
            }
            cn = cost(np, pn, p0, sd, nobs, f, ctx);
            if (cn <= c) {
                break;
            }
        }
        if (k == MAX_HALVINGS) {
            converged = true; /* no step lowers the cost: at the minimum */
            break;
        }
        double maxrel = 0.0;
        for (int i = 0; i < np; i++) {
            double rel = fabs(pn[i] - p[i]) / sd[i];
            if (rel > maxrel) {
                maxrel = rel;
            }
        }
        memcpy(p, pn, sizeof(double) * (size_t)np);
        c = cn;
        if (maxrel < 1e-6) {
            converged = true;
        }
    }

    /* Posterior covariance: H^-1 at the solution. */
    normal_eq(np, p, p0, sd, nobs, f, ctx, h, g);
    double inv[LSQ_MAX_PAR];
    for (int i = 0; i < np; i++) {
        g[i] = 0.0;
    }
    bool pd = lsq_spd_solve(np, h, g, inv);
    for (int i = 0; i < np; i++) {
        p_out[i] = p[i];
        sd_out[i] = pd ? sqrt(inv[i]) : NAN;
    }
    if (info) {
        info->iterations = it;
        info->ok = converged && pd;
    }
}

float calib_dir_coverage(const float *xyz, int stride, int n, const float centre[3])
{
    if (n == 0) {
        return 0.0f;
    }
    double s[3][3] = { { 0 } };
    int used = 0;
    for (int j = 0; j < n; j++) {
        const float *a = xyz + (size_t)j * (size_t)stride;
        double d[3] = { a[0] - centre[0], a[1] - centre[1], a[2] - centre[2] };
        double len = sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
        if (len < 1e-6) {
            continue;
        }
        used++;
        for (int r = 0; r < 3; r++) {
            for (int c = 0; c < 3; c++) {
                s[r][c] += d[r] / len * d[c] / len;
            }
        }
    }
    if (used == 0) {
        return 0.0f;
    }
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            s[r][c] /= used;
        }
    }
    /* Smallest eigenvalue of a symmetric 3x3 (trigonometric method). */
    double p1 = s[0][1] * s[0][1] + s[0][2] * s[0][2] + s[1][2] * s[1][2];
    double q = (s[0][0] + s[1][1] + s[2][2]) / 3.0;
    if (p1 < 1e-18) {
        double m = s[0][0];
        if (s[1][1] < m) m = s[1][1];
        if (s[2][2] < m) m = s[2][2];
        return (float)m;
    }
    double p2 = (s[0][0] - q) * (s[0][0] - q) + (s[1][1] - q) * (s[1][1] - q) +
                (s[2][2] - q) * (s[2][2] - q) + 2.0 * p1;
    double pp = sqrt(p2 / 6.0);
    double b[3][3];
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            b[r][c] = (s[r][c] - (r == c ? q : 0.0)) / pp;
        }
    }
    double detb = b[0][0] * (b[1][1] * b[2][2] - b[1][2] * b[2][1]) -
                  b[0][1] * (b[1][0] * b[2][2] - b[1][2] * b[2][0]) +
                  b[0][2] * (b[1][0] * b[2][1] - b[1][1] * b[2][0]);
    double rr = detb / 2.0;
    double phi = rr <= -1.0 ? M_PI / 3.0 : rr >= 1.0 ? 0.0 : acos(rr) / 3.0;
    double e_min = q + 2.0 * pp * cos(phi + 2.0 * M_PI / 3.0);
    return (float)(e_min < 0.0 ? 0.0 : e_min);
}
