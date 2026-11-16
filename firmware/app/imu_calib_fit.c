#include "imu_calib_fit.h"

#include <math.h>
#include <string.h>

#define EARTH_RATE_RPS 7.2921151467e-5 /* WGS84 */

void gcal_stats_clear(gcal_stats_t *s)
{
    memset(s, 0, sizeof *s);
}

bool gcal_stats_add(gcal_stats_t *s, const gcal_prior_t *p, const float bias[3], float temp_c,
                    float var)
{
    if (!isfinite(temp_c) || !isfinite(var) || var <= 0.0f) {
        return false;
    }
    for (int i = 0; i < 3; i++) {
        if (!isfinite(bias[i])) {
            return false;
        }
    }
    const double w = 1.0 / (double)var;
    const double d = (double)temp_c - (double)p->t_ref_c;
    s->info[0] += w;
    s->info[1] += w * d;
    s->info[2] += w * d * d;
    for (int i = 0; i < 3; i++) {
        s->rhs[i][0] += w * (double)bias[i];
        s->rhs[i][1] += w * (double)bias[i] * d;
    }
    if (s->n_chunks == 0 || temp_c < s->t_min_c) {
        s->t_min_c = temp_c;
    }
    if (s->n_chunks == 0 || temp_c > s->t_max_c) {
        s->t_max_c = temp_c;
    }
    s->n_chunks++;
    return true;
}

void gcal_stats_forget(gcal_stats_t *s, float factor)
{
    if (!(factor > 0.0f) || factor > 1.0f) {
        return;
    }
    for (int j = 0; j < 3; j++) {
        s->info[j] *= factor;
    }
    for (int i = 0; i < 3; i++) {
        s->rhs[i][0] *= factor;
        s->rhs[i][1] *= factor;
    }
}

void gcal_solve(const gcal_stats_t *s, const gcal_prior_t *p, gcal_result_t *r)
{
    const double pb = 1.0 / ((double)p->sd_b0_rps * (double)p->sd_b0_rps);
    r->k_fitted = s->n_chunks > 0 && s->t_max_c - s->t_min_c >= p->k_min_span_c;
    if (!r->k_fitted) {
        /* b0 alone, k = 0: the weighted mean bias, shrunk by the prior. */
        const double a = s->info[0] + pb;
        for (int i = 0; i < 3; i++) {
            r->b0[i] = (float)(s->rhs[i][0] / a);
            r->k[i] = 0.0f;
        }
        r->sd_b0 = (float)sqrt(1.0 / a);
        r->sd_k = p->sd_k_rps_c;
        return;
    }
    /* A = info + prior information; x = A^-1 * rhs (prior mean is zero). */
    const double pk = 1.0 / ((double)p->sd_k_rps_c * (double)p->sd_k_rps_c);
    const double a00 = s->info[0] + pb;
    const double a01 = s->info[1];
    const double a11 = s->info[2] + pk;
    const double det = a00 * a11 - a01 * a01;
    /* Both diagonal terms carry the prior, so det > 0 for any valid stats. */
    const double i00 = a11 / det;
    const double i01 = -a01 / det;
    const double i11 = a00 / det;
    for (int i = 0; i < 3; i++) {
        r->b0[i] = (float)(i00 * s->rhs[i][0] + i01 * s->rhs[i][1]);
        r->k[i] = (float)(i01 * s->rhs[i][0] + i11 * s->rhs[i][1]);
    }
    r->sd_b0 = (float)sqrt(i00);
    r->sd_k = (float)sqrt(i11);
}

const char *gcal_validate(const gcal_result_t *r, const gcal_limits_t *lim)
{
    for (int i = 0; i < 3; i++) {
        if (!isfinite(r->b0[i]) || !isfinite(r->k[i])) {
            return "not finite";
        }
        if (fabsf(r->b0[i]) > lim->max_b0_rps) {
            return "gyro bias at T_ref out of range";
        }
        if (fabsf(r->k[i]) > lim->max_k_rps_c) {
            return "gyro bias temperature slope out of range";
        }
    }
    if (!isfinite(r->sd_b0) || !isfinite(r->sd_k)) {
        return "sigma not finite";
    }
    return NULL;
}

const char *gcal_stats_validate(const gcal_stats_t *s)
{
    for (int j = 0; j < 3; j++) {
        if (!isfinite(s->info[j])) {
            return "information not finite";
        }
    }
    if (s->info[0] < 0.0 || s->info[2] < 0.0) {
        return "information negative";
    }
    for (int i = 0; i < 3; i++) {
        if (!isfinite(s->rhs[i][0]) || !isfinite(s->rhs[i][1])) {
            return "right-hand side not finite";
        }
    }
    if (!isfinite(s->t_min_c) || !isfinite(s->t_max_c) || !isfinite(s->still_s)) {
        return "temperature range not finite";
    }
    return NULL;
}

void gcal_earth_rate_body(const float rpy_rad[3], double lat_rad, float out[3])
{
    const double w_n[3] = { EARTH_RATE_RPS * cos(lat_rad), 0.0, -EARTH_RATE_RPS * sin(lat_rad) };
    const double cr = cos(rpy_rad[0]), sr = sin(rpy_rad[0]);
    const double cp = cos(rpy_rad[1]), sp = sin(rpy_rad[1]);
    const double cy = cos(rpy_rad[2]), sy = sin(rpy_rad[2]);
    /* C_nb (body to NED), ZYX; body = C_nb^T * n. */
    const double c[3][3] = {
        { cp * cy, sr * sp * cy - cr * sy, cr * sp * cy + sr * sy },
        { cp * sy, sr * sp * sy + cr * cy, cr * sp * sy - sr * cy },
        { -sp,     sr * cp,                cr * cp },
    };
    for (int j = 0; j < 3; j++) {
        out[j] = (float)(c[0][j] * w_n[0] + c[1][j] * w_n[1] + c[2][j] * w_n[2]);
    }
}
