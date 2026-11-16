#include "imu_calib_acc.h"

#include <math.h>
#include <string.h>

#include "imu_calib_lsq.h"

static const double k_p0[ACAL_NPAR] = { 1, 1, 1, 0, 0, 0, 0, 0, 0 };

float acal_gravity(double lat_rad, float h_m)
{
    const double s2 = sin(lat_rad) * sin(lat_rad);
    const double g0 = 9.7803253359 * (1.0 + 0.00193185265241 * s2) / sqrt(1.0 - 0.00669437999013 * s2);
    return (float)(g0 - 3.086e-6 * (double)h_m);
}

/* |M(a - b)| - g, unweighted, and its gradient with respect to p. */
static double residual(const double p[ACAL_NPAR], const float a[3], double g, double jac[ACAL_NPAR])
{
    const double u[3] = { a[0] - p[ACAL_BX], a[1] - p[ACAL_BY], a[2] - p[ACAL_BZ] };
    const double v[3] = {
        p[ACAL_SX] * u[0] + p[ACAL_MXY] * u[1] + p[ACAL_MXZ] * u[2],
        p[ACAL_SY] * u[1] + p[ACAL_MYZ] * u[2],
        p[ACAL_SZ] * u[2],
    };
    double n = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (n < 1e-9) {
        n = 1e-9;
    }
    if (jac) {
        jac[ACAL_SX] = v[0] * u[0] / n;
        jac[ACAL_MXY] = v[0] * u[1] / n;
        jac[ACAL_MXZ] = v[0] * u[2] / n;
        jac[ACAL_SY] = v[1] * u[1] / n;
        jac[ACAL_MYZ] = v[1] * u[2] / n;
        jac[ACAL_SZ] = v[2] * u[2] / n;
        /* d|v|/db = -(M^T v) / |v| */
        jac[ACAL_BX] = -(p[ACAL_SX] * v[0]) / n;
        jac[ACAL_BY] = -(p[ACAL_MXY] * v[0] + p[ACAL_SY] * v[1]) / n;
        jac[ACAL_BZ] = -(p[ACAL_MXZ] * v[0] + p[ACAL_MYZ] * v[1] + p[ACAL_SZ] * v[2]) / n;
    }
    return n - g;
}

typedef struct {
    const acal_pose_t *poses;
    double g;
} ctx_t;

static double weighted(void *vctx, int j, const double *p, double *jac)
{
    const ctx_t *c = vctx;
    const double w = 1.0 / (double)c->poses[j].sd;
    double r = residual(p, c->poses[j].acc, c->g, jac);
    if (jac) {
        for (int i = 0; i < ACAL_NPAR; i++) {
            jac[i] *= w;
        }
    }
    return r * w;
}

static double rms(const double p[ACAL_NPAR], const acal_pose_t *poses, int n, double g)
{
    if (n == 0) {
        return 0.0;
    }
    double s = 0.0;
    for (int j = 0; j < n; j++) {
        double r = residual(p, poses[j].acc, g, NULL);
        s += r * r;
    }
    return sqrt(s / n);
}

void acal_solve(const acal_pose_t *poses, int n, float g, const acal_prior_t *prior,
                acal_result_t *r)
{
    double sd[ACAL_NPAR], p[ACAL_NPAR], psd[ACAL_NPAR];
    for (int i = 0; i < ACAL_NPAR; i++) {
        sd[i] = i <= ACAL_SZ ? prior->sd_scale : i <= ACAL_MYZ ? prior->sd_mis : prior->sd_bias;
    }
    memset(r, 0, sizeof *r);
    r->rms_before = (float)rms(k_p0, poses, n, g);

    ctx_t c = { poses, g };
    lsq_info_t info;
    lsq_solve(ACAL_NPAR, k_p0, sd, NULL, n, weighted, &c, p, psd, &info);
    for (int i = 0; i < ACAL_NPAR; i++) {
        r->p[i] = (float)p[i];
        r->sd[i] = (float)psd[i];
    }
    r->rms_after = (float)rms(p, poses, n, g);
    r->iterations = info.iterations;
    r->ok = info.ok;
}

const char *acal_validate(const acal_result_t *r, const acal_limits_t *lim)
{
    if (!r->ok) {
        return "fit did not converge";
    }
    for (int i = 0; i < ACAL_NPAR; i++) {
        if (!isfinite(r->p[i]) || !isfinite(r->sd[i])) {
            return "not finite";
        }
    }
    for (int i = ACAL_SX; i <= ACAL_SZ; i++) {
        if (fabsf(r->p[i] - 1.0f) > lim->max_scale_dev) {
            return "accelerometer scale out of range";
        }
    }
    for (int i = ACAL_MXY; i <= ACAL_MYZ; i++) {
        if (fabsf(r->p[i]) > lim->max_mis_rad) {
            return "accelerometer misalignment out of range";
        }
    }
    for (int i = ACAL_BX; i <= ACAL_BZ; i++) {
        if (fabsf(r->p[i]) > lim->max_bias) {
            return "accelerometer bias out of range";
        }
    }
    return NULL;
}

void acal_to_inslib(const float p[ACAL_NPAR], float m[9], float bias[3])
{
    /* Column-major: m[col * 3 + row]. */
    memset(m, 0, 9 * sizeof m[0]);
    m[0] = p[ACAL_SX];
    m[3] = p[ACAL_MXY];
    m[4] = p[ACAL_SY];
    m[6] = p[ACAL_MXZ];
    m[7] = p[ACAL_MYZ];
    m[8] = p[ACAL_SZ];
    bias[0] = p[ACAL_BX];
    bias[1] = p[ACAL_BY];
    bias[2] = p[ACAL_BZ];
}

float acal_coverage(const acal_pose_t *poses, int n)
{
    static const float zero[3] = { 0.0f, 0.0f, 0.0f };
    return n == 0 ? 0.0f
                  : calib_dir_coverage(poses[0].acc, (int)(sizeof(acal_pose_t) / sizeof(float)), n, zero);
}
