#include "imu_calib_mag.h"

#include <math.h>
#include <string.h>

#include "imu_calib_lsq.h"

/* ---- Ellipsoid fit ---------------------------------------------------------- */

static void sym_from_p(const double *p, double m[3][3])
{
    m[0][0] = p[MCAL_A00];
    m[1][1] = p[MCAL_A11];
    m[2][2] = p[MCAL_A22];
    m[0][1] = m[1][0] = p[MCAL_A01];
    m[0][2] = m[2][0] = p[MCAL_A02];
    m[1][2] = m[2][1] = p[MCAL_A12];
}

/* |M(raw - c)| - F, unweighted, and its gradient. */
static double ell_residual(const double *p, const float raw[3], double f, double *jac)
{
    double m[3][3];
    sym_from_p(p, m);
    const double u[3] = { raw[0] - p[MCAL_CX], raw[1] - p[MCAL_CY], raw[2] - p[MCAL_CZ] };
    double v[3];
    for (int i = 0; i < 3; i++) {
        v[i] = m[i][0] * u[0] + m[i][1] * u[1] + m[i][2] * u[2];
    }
    double n = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (n < 1e-9) {
        n = 1e-9;
    }
    if (jac) {
        jac[MCAL_A00] = v[0] * u[0] / n;
        jac[MCAL_A11] = v[1] * u[1] / n;
        jac[MCAL_A22] = v[2] * u[2] / n;
        jac[MCAL_A01] = (v[0] * u[1] + v[1] * u[0]) / n;
        jac[MCAL_A02] = (v[0] * u[2] + v[2] * u[0]) / n;
        jac[MCAL_A12] = (v[1] * u[2] + v[2] * u[1]) / n;
        /* d|v|/dc = -(M^T v) / |v|, M symmetric */
        for (int k = 0; k < 3; k++) {
            jac[MCAL_CX + k] = -(m[0][k] * v[0] + m[1][k] * v[1] + m[2][k] * v[2]) / n;
        }
    }
    return n - f;
}

/* down . M(raw - c) - F sin(I), unweighted, and its gradient. */
static double dip_residual(const double *p, const float raw[3], const float d[3], double fz,
                           double *jac)
{
    double m[3][3];
    sym_from_p(p, m);
    const double u[3] = { raw[0] - p[MCAL_CX], raw[1] - p[MCAL_CY], raw[2] - p[MCAL_CZ] };
    double vz = 0.0;
    for (int i = 0; i < 3; i++) {
        vz += d[i] * (m[i][0] * u[0] + m[i][1] * u[1] + m[i][2] * u[2]);
    }
    if (jac) {
        jac[MCAL_A00] = d[0] * u[0];
        jac[MCAL_A11] = d[1] * u[1];
        jac[MCAL_A22] = d[2] * u[2];
        jac[MCAL_A01] = d[0] * u[1] + d[1] * u[0];
        jac[MCAL_A02] = d[0] * u[2] + d[2] * u[0];
        jac[MCAL_A12] = d[1] * u[2] + d[2] * u[1];
        for (int k = 0; k < 3; k++) {
            jac[MCAL_CX + k] = -(m[k][0] * d[0] + m[k][1] * d[1] + m[k][2] * d[2]);
        }
    }
    return vz - fz;
}

typedef struct {
    const mcal_sample_t *s;
    int n;
    const int *down_idx;   /* samples with a down direction */
    double f, fz, w, wd;
} ell_ctx_t;

/* Observations 0..n-1: magnitudes; n..: vertical components. */
static double ell_weighted(void *vctx, int j, const double *p, double *jac)
{
    const ell_ctx_t *c = vctx;
    double r, w;
    if (j < c->n) {
        r = ell_residual(p, c->s[j].m, c->f, jac);
        w = c->w;
    } else {
        const mcal_sample_t *s = &c->s[c->down_idx[j - c->n]];
        r = dip_residual(p, s->m, s->down, c->fz, jac);
        w = c->wd;
    }
    if (jac) {
        for (int i = 0; i < MCAL_NPAR; i++) {
            jac[i] *= w;
        }
    }
    return r * w;
}

/* Algebraic sphere fit |raw|^2 = 2 raw.c + k, for the first guess. */
static bool sphere_fit(const mcal_sample_t *s, int n, double c[3], double *radius)
{
    if (n < 8) {
        return false;
    }
    double a[16] = { 0 }, b[4] = { 0 };
    for (int j = 0; j < n; j++) {
        const float *raw = s[j].m;
        const double x[4] = { 2.0 * raw[0], 2.0 * raw[1], 2.0 * raw[2], 1.0 };
        const double y = (double)raw[0] * raw[0] + (double)raw[1] * raw[1] + (double)raw[2] * raw[2];
        for (int r = 0; r < 4; r++) {
            b[r] += x[r] * y;
            for (int q = 0; q < 4; q++) {
                a[r * 4 + q] += x[r] * x[q];
            }
        }
    }
    if (!lsq_spd_solve(4, a, b, NULL)) {
        return false;
    }
    const double r2 = b[3] + b[0] * b[0] + b[1] * b[1] + b[2] * b[2];
    if (!(r2 > 0.0) || !isfinite(r2)) {
        return false;
    }
    c[0] = b[0];
    c[1] = b[1];
    c[2] = b[2];
    *radius = sqrt(r2);
    return true;
}

static double ell_rms(const double *p, const mcal_sample_t *s, int n, double f)
{
    if (n == 0) {
        return 0.0;
    }
    double q = 0.0;
    for (int j = 0; j < n; j++) {
        double r = ell_residual(p, s[j].m, f, NULL);
        q += r * r;
    }
    return sqrt(q / n);
}

#define MCAL_MAX_SAMPLES 128

void mcal_ellipsoid(const mcal_sample_t *s, int n, float field_ut, float incl_rad,
                    const mcal_prior_t *prior, mcal_ell_t *out)
{
    if (n > MCAL_MAX_SAMPLES) {
        n = MCAL_MAX_SAMPLES;
    }
    memset(out, 0, sizeof *out);
    out->field_ut = field_ut;
    out->n = n;

    /* First guess for the centre: the algebraic sphere fit, but only when
     * the samples surround its centre. For a ring of samples (a vehicle
     * turning on level ground) it returns a small sphere through the ring,
     * far off in the unseen direction; the sample mean is the safer start. */
    double c0[3] = { 0.0, 0.0, 0.0 }, radius = 0.0;
    bool use_sphere = sphere_fit(s, n, c0, &radius);
    if (use_sphere) {
        const float cf[3] = { (float)c0[0], (float)c0[1], (float)c0[2] };
        use_sphere = calib_dir_coverage(s[0].m, (int)(sizeof(mcal_sample_t) / sizeof(float)), n, cf) >= 0.1f;
    }
    if (!use_sphere) {
        c0[0] = c0[1] = c0[2] = 0.0;
        for (int j = 0; j < n; j++) {
            for (int k = 0; k < 3; k++) {
                c0[k] += s[j].m[k] / (double)n;
            }
        }
    }
    /* Prior scale: the raw unit as converted (plans/imu-tm210.md: about one
     * field strength per TM210 unit), not a data-derived value, so a
     * direction the samples don't reach keeps a sane scale. */
    const double s0 = 1.0;
    out->s0 = (float)s0;

    double p0[MCAL_NPAR] = { s0, s0, s0, 0, 0, 0, 0, 0, 0 };
    double sd[MCAL_NPAR];
    for (int i = 0; i < MCAL_NPAR; i++) {
        sd[i] = i <= MCAL_A22 ? prior->sd_diag_rel * s0 : i <= MCAL_A12 ? prior->sd_offdiag * s0
                                                                         : prior->sd_hard_ut;
    }
    double start[MCAL_NPAR];
    memcpy(start, p0, sizeof start);
    start[MCAL_CX] = c0[0];
    start[MCAL_CY] = c0[1];
    start[MCAL_CZ] = c0[2];

    const double unit[MCAL_NPAR] = { s0, s0, s0, 0, 0, 0, 0, 0, 0 };
    out->rms_before = (float)ell_rms(unit, s, n, field_ut);
    (void)radius;

    int down_idx[MCAL_MAX_SAMPLES], nd = 0;
    for (int j = 0; j < n; j++) {
        if (s[j].has_down) {
            down_idx[nd++] = j;
        }
    }
    out->n_down = nd;
    const double fz = (double)field_ut * sin((double)incl_rad);
    ell_ctx_t ctx = { s, n, down_idx, field_ut, fz, 1.0 / prior->sample_sd_ut, 1.0 / prior->dip_sd_ut };
    double p[MCAL_NPAR], psd[MCAL_NPAR];
    lsq_info_t info;
    lsq_solve(MCAL_NPAR, p0, sd, start, n + nd, ell_weighted, &ctx, p, psd, &info);
    for (int i = 0; i < MCAL_NPAR; i++) {
        out->p[i] = (float)p[i];
        out->sd[i] = (float)psd[i];
    }
    out->rms_after = (float)ell_rms(p, s, n, field_ut);
    out->rms_dip_ut = -1.0f;
    if (nd > 0) {
        double q = 0.0;
        for (int k = 0; k < nd; k++) {
            double r = dip_residual(p, s[down_idx[k]].m, s[down_idx[k]].down, fz, NULL);
            q += r * r;
        }
        out->rms_dip_ut = (float)sqrt(q / nd);
    }
    out->iterations = info.iterations;
    out->ok = info.ok;
}

/* ---- Reference fit ----------------------------------------------------------- */

static int sidx(int r, int c)
{
    /* Upper triangle of a 4x4, row by row. */
    if (r > c) {
        int t = r;
        r = c;
        c = t;
    }
    static const int k_row0[4] = { 0, 4, 7, 9 };
    return k_row0[r] + (c - r);
}

void mcal_ref_clear(mcal_ref_stats_t *s)
{
    memset(s, 0, sizeof *s);
}

bool mcal_ref_add(mcal_ref_stats_t *s, const float raw[3], const float ref[3], float sd)
{
    if (!isfinite(sd) || sd <= 0.0f) {
        return false;
    }
    for (int i = 0; i < 3; i++) {
        if (!isfinite(raw[i]) || !isfinite(ref[i])) {
            return false;
        }
    }
    const double w = 1.0 / ((double)sd * (double)sd);
    const double x[4] = { raw[0], raw[1], raw[2], 1.0 };
    for (int r = 0; r < 4; r++) {
        for (int c = r; c < 4; c++) {
            s->s[sidx(r, c)] += w * x[r] * x[c];
        }
    }
    for (int i = 0; i < 3; i++) {
        for (int k = 0; k < 4; k++) {
            s->r[i][k] += w * (double)ref[i] * x[k];
        }
        s->yy += w * (double)ref[i] * (double)ref[i];
    }
    s->w += w;
    s->n++;
    return true;
}

void mcal_ref_forget(mcal_ref_stats_t *s, float factor)
{
    if (!(factor > 0.0f) || factor > 1.0f) {
        return;
    }
    for (int k = 0; k < 10; k++) {
        s->s[k] *= factor;
    }
    for (int i = 0; i < 3; i++) {
        for (int k = 0; k < 4; k++) {
            s->r[i][k] *= factor;
        }
    }
    s->yy *= factor;
    s->w *= factor;
}

const char *mcal_ref_validate(const mcal_ref_stats_t *s)
{
    for (int k = 0; k < 10; k++) {
        if (!isfinite(s->s[k])) {
            return "reference statistics not finite";
        }
    }
    for (int i = 0; i < 3; i++) {
        for (int k = 0; k < 4; k++) {
            if (!isfinite(s->r[i][k])) {
                return "reference statistics not finite";
            }
        }
    }
    if (!isfinite(s->yy) || !isfinite(s->w) || s->w < 0.0 || s->s[sidx(3, 3)] < 0.0) {
        return "reference statistics not finite";
    }
    return NULL;
}

static bool inv3(const float a[9], float out[9])
{
    const double det = (double)a[0] * ((double)a[4] * a[8] - (double)a[5] * a[7]) -
                       (double)a[1] * ((double)a[3] * a[8] - (double)a[5] * a[6]) +
                       (double)a[2] * ((double)a[3] * a[7] - (double)a[4] * a[6]);
    if (!(fabs(det) > 1e-12)) {
        return false;
    }
    const double id = 1.0 / det;
    out[0] = (float)(((double)a[4] * a[8] - (double)a[5] * a[7]) * id);
    out[1] = (float)(((double)a[2] * a[7] - (double)a[1] * a[8]) * id);
    out[2] = (float)(((double)a[1] * a[5] - (double)a[2] * a[4]) * id);
    out[3] = (float)(((double)a[5] * a[6] - (double)a[3] * a[8]) * id);
    out[4] = (float)(((double)a[0] * a[8] - (double)a[2] * a[6]) * id);
    out[5] = (float)(((double)a[2] * a[3] - (double)a[0] * a[5]) * id);
    out[6] = (float)(((double)a[3] * a[7] - (double)a[4] * a[6]) * id);
    out[7] = (float)(((double)a[1] * a[6] - (double)a[0] * a[7]) * id);
    out[8] = (float)(((double)a[0] * a[4] - (double)a[1] * a[3]) * id);
    return true;
}

void mcal_ref_solve(const mcal_ref_stats_t *s, const mcal_ell_t *ell, float sd_m, float sd_b_ut,
                    mcal_cal_t *out)
{
    memset(out, 0, sizeof *out);
    double m0[3][3];
    double pe[MCAL_NPAR];
    for (int i = 0; i < MCAL_NPAR; i++) {
        pe[i] = ell->p[i];
    }
    sym_from_p(pe, m0);
    const double c0[3] = { pe[MCAL_CX], pe[MCAL_CY], pe[MCAL_CZ] };

    /* Prior per row: [M_i0 M_i1 M_i2 d_i], d = -M c. sigma of d ~ |M| sigma of b. */
    const double s0 = ell->s0 > 0.0f ? ell->s0 : 1.0;
    const double pm = 1.0 / ((double)sd_m * s0 * (double)sd_m * s0);
    const double pd = 1.0 / ((double)sd_b_ut * s0 * (double)sd_b_ut * s0);
    const double pdiag[4] = { pm, pm, pm, pd };

    double worst_m = 0.0, worst_d = 0.0;
    bool ok = true;
    double x[3][4];
    for (int i = 0; i < 3; i++) {
        double a[16], b[4], inv[4];
        const double x0[4] = { m0[i][0], m0[i][1], m0[i][2],
                               -(m0[i][0] * c0[0] + m0[i][1] * c0[1] + m0[i][2] * c0[2]) };
        for (int r = 0; r < 4; r++) {
            for (int c = 0; c < 4; c++) {
                a[r * 4 + c] = s->s[sidx(r, c)] + (r == c ? pdiag[r] : 0.0);
            }
            b[r] = s->r[i][r] + pdiag[r] * x0[r];
        }
        if (!lsq_spd_solve(4, a, b, inv)) {
            ok = false;
            break;
        }
        for (int r = 0; r < 4; r++) {
            x[i][r] = b[r];
        }
        for (int r = 0; r < 3; r++) {
            if (sqrt(inv[r]) > worst_m) worst_m = sqrt(inv[r]);
        }
        if (sqrt(inv[3]) > worst_d) worst_d = sqrt(inv[3]);
    }
    if (!ok) {
        return;
    }
    for (int i = 0; i < 3; i++) {
        for (int c = 0; c < 3; c++) {
            out->m[i * 3 + c] = (float)x[i][c];
        }
    }
    float mi[9];
    if (!inv3(out->m, mi)) {
        return;
    }
    for (int i = 0; i < 3; i++) {
        /* b = -M^-1 d */
        out->b[i] = -(float)((double)mi[i * 3 + 0] * x[0][3] + (double)mi[i * 3 + 1] * x[1][3] +
                             (double)mi[i * 3 + 2] * x[2][3]);
    }
    out->sd_m = (float)worst_m;
    out->sd_b_ut = (float)(worst_d / s0);
    out->n_ref = s->n;
    if (s->n > 0 && s->w > 0.0) {
        /* SSR = sum w |y - X x|^2 = yy - 2 sum_i x_i.r_i + sum_i x_i^T S x_i */
        double ssr = s->yy;
        for (int i = 0; i < 3; i++) {
            for (int r = 0; r < 4; r++) {
                ssr -= 2.0 * x[i][r] * s->r[i][r];
                for (int c = 0; c < 4; c++) {
                    ssr += x[i][r] * s->s[sidx(r, c)] * x[i][c];
                }
            }
        }
        out->rms_ref_ut = (float)sqrt(fmax(ssr, 0.0) / (3.0 * s->w));
    } else {
        out->rms_ref_ut = -1.0f;
    }
    out->ok = ell->ok;
}

const char *mcal_validate(const mcal_cal_t *c, const mcal_ell_t *ell, const mcal_limits_t *lim)
{
    if (!c->ok) {
        return "fit did not converge";
    }
    double diag = 0.0;
    for (int i = 0; i < 9; i++) {
        if (!isfinite(c->m[i])) {
            return "not finite";
        }
    }
    for (int i = 0; i < 3; i++) {
        if (!isfinite(c->b[i])) {
            return "not finite";
        }
        if (c->m[i * 4] < lim->min_diag || c->m[i * 4] > lim->max_diag) {
            return "magnetometer scale out of range";
        }
        if (fabsf(c->b[i]) > lim->max_hard_ut) {
            return "magnetometer hard iron out of range";
        }
        diag += c->m[i * 4] / 3.0;
    }
    for (int r = 0; r < 3; r++) {
        for (int k = 0; k < 3; k++) {
            if (r != k && fabs(c->m[r * 3 + k]) > lim->max_offdiag * diag) {
                return "magnetometer soft iron / rotation out of range";
            }
        }
    }
    if (ell && ell->n > 0 && ell->rms_after > lim->max_rms_rel * ell->field_ut) {
        return "magnetometer residual too large (disturbed samples?)";
    }
    return NULL;
}

void mcal_to_inslib(const mcal_cal_t *c, float m[9], float bias[3])
{
    for (int r = 0; r < 3; r++) {
        for (int k = 0; k < 3; k++) {
            m[k * 3 + r] = c->m[r * 3 + k]; /* column-major */
        }
        bias[r] = c->b[r];
    }
}
