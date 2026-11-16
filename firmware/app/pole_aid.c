#include "pole_aid.h"

#include <math.h>
#include <string.h>

#define D2R 0.017453293f
#define R2D 57.29578f
#define EARTH_R 6371000.0f /* the fit spans centimetres: a sphere is plenty */

void pole_aid_default_cfg(pole_aid_cfg_t *c)
{
    memset(c, 0, sizeof *c);
    c->g = 9.81f;
    c->err_mean_max = 0.25f;
    c->err_peak_max = 1.0f;
    c->tip_speed_max = 0.2f;
    c->settle_s = 0.3f;
    c->feed_period_s = 0.05f;
    c->episode_max_s = 5.0f;
    c->fit_sd_max_m = 0.03f;
    c->fit_min_n = 10;
    c->fit_min_spread_m = 0.08f;
    c->fit_max_rms_m = 0.02f;
    c->fit_max_scale_err = 0.1f;
    c->fit_min_sd_deg = 2.0f;
    c->mag_norm_tol = POLE_MAG_NORM_TOL;
    c->mag_dip_tol_deg = POLE_MAG_DIP_TOL_DEG;
    c->mag_tilt_range_deg = POLE_MAG_TILT_RANGE_DEG;
    c->mag_yaw_range_deg = POLE_MAG_YAW_RANGE_DEG;
    c->mag_yaw_sd_max_deg = POLE_MAG_YAW_SD_MAX_DEG;
    c->mag_res_mean_max_deg = POLE_MAG_RES_MEAN_MAX_DEG;
    c->mag_res_sd_max_deg = POLE_MAG_RES_SD_MAX_DEG;
}

static void cross(const float a[3], const float b[3], float o[3])
{
    o[0] = a[1] * b[2] - a[2] * b[1];
    o[1] = a[2] * b[0] - a[0] * b[2];
    o[2] = a[0] * b[1] - a[1] * b[0];
}

/* R column-major, body to NED. */
static void rot(const float R[9], const float v[3], float o[3])
{
    for (int i = 0; i < 3; i++) o[i] = R[i] * v[0] + R[i + 3] * v[1] + R[i + 6] * v[2];
}

static float wrap_pi(float a)
{
    while (a > 3.14159265f) a -= 6.2831853f;
    while (a < -3.14159265f) a += 6.2831853f;
    return a;
}

void pole_quat_from_R(const float R[9], float q[4])
{
    /* R[i + 3 * j] is row i, column j. */
    float m00 = R[0], m11 = R[4], m22 = R[8];
    float m10 = R[1], m01 = R[3], m20 = R[2], m02 = R[6], m21 = R[5], m12 = R[7];
    float tr = m00 + m11 + m22;
    if (tr > 0.0f) {
        float s = sqrtf(tr + 1.0f) * 2.0f;
        q[0] = 0.25f * s;
        q[1] = (m21 - m12) / s;
        q[2] = (m02 - m20) / s;
        q[3] = (m10 - m01) / s;
    } else if (m00 > m11 && m00 > m22) {
        float s = sqrtf(1.0f + m00 - m11 - m22) * 2.0f;
        q[0] = (m21 - m12) / s;
        q[1] = 0.25f * s;
        q[2] = (m01 + m10) / s;
        q[3] = (m02 + m20) / s;
    } else if (m11 > m22) {
        float s = sqrtf(1.0f + m11 - m00 - m22) * 2.0f;
        q[0] = (m02 - m20) / s;
        q[1] = (m01 + m10) / s;
        q[2] = 0.25f * s;
        q[3] = (m12 + m21) / s;
    } else {
        float s = sqrtf(1.0f + m22 - m00 - m11) * 2.0f;
        q[0] = (m10 - m01) / s;
        q[1] = (m02 + m20) / s;
        q[2] = (m12 + m21) / s;
        q[3] = 0.25f * s;
    }
}

void pole_quat_to_R(const float q[4], float R[9])
{
    float w = q[0], x = q[1], y = q[2], z = q[3];
    R[0] = 1.0f - 2.0f * (y * y + z * z);
    R[1] = 2.0f * (x * y + w * z);
    R[2] = 2.0f * (x * z - w * y);
    R[3] = 2.0f * (x * y - w * z);
    R[4] = 1.0f - 2.0f * (x * x + z * z);
    R[5] = 2.0f * (y * z + w * x);
    R[6] = 2.0f * (x * z + w * y);
    R[7] = 2.0f * (y * z - w * x);
    R[8] = 1.0f - 2.0f * (x * x + y * y);
}

/* q <- q (x) [1, w dt / 2], normalised: first order is plenty at 800 Hz. */
void pole_quat_step(float q[4], const float w[3], float dt)
{
    float h = 0.5f * dt;
    float dx = w[0] * h, dy = w[1] * h, dz = w[2] * h;
    float a = q[0], b = q[1], c = q[2], d = q[3];
    float o[4] = { a - b * dx - c * dy - d * dz, a * dx + b + c * dz - d * dy, a * dy - b * dz + c + d * dx,
                   a * dz + b * dy - c * dx + d };
    float n = sqrtf(o[0] * o[0] + o[1] * o[1] + o[2] * o[2] + o[3] * o[3]);
    if (!(n > 0.0f)) return;
    for (int i = 0; i < 4; i++) q[i] = o[i] / n;
}

void pole_episode_add(pole_episode_t *e, const float R[9], float pN, float pE)
{
    /* G = top two rows of R: G[j][c] = R[j + 3 c]. */
    float G[2][3];
    for (int j = 0; j < 2; j++)
        for (int c = 0; c < 3; c++) G[j][c] = R[j + 3 * c];
    const float pv[2] = { pN, pE };
    for (int j = 0; j < 2; j++) {
        e->sp[j] += pv[j];
        for (int c = 0; c < 3; c++) e->sg[j][c] += G[j][c];
    }
    for (int a = 0; a < 3; a++)
        for (int b = 0; b < 3; b++) e->sgg[a][b] += G[0][a] * G[0][b] + G[1][a] * G[1][b];
    for (int c = 0; c < 3; c++)
        for (int j = 0; j < 2; j++)
            for (int k = 0; k < 2; k++) e->sgp[c][j][k] += G[j][c] * pv[k];
    e->spp += pN * pN + pE * pE;
    if (e->n < 0xFFFFu) e->n++;
}

static void fit_clear(pole_aid_t *p)
{
    memset(&p->ep, 0, sizeof p->ep);
    p->twist = p->twist_min = p->twist_max = 0.0f;
    p->fit_origin = false;
    memset(p->S, 0, sizeof p->S);
    memset(p->b, 0, sizeof p->b);
    p->pp = 0.0;
    p->a_sum[0] = p->a_sum[1] = 0.0;
    p->a_sq = 0.0;
    p->fit_n = 0;
}

void pole_aid_set_geometry(pole_aid_t *p, bool active, const float tip_b[3], const float ant_b[3])
{
    for (int i = 0; i < 3; i++) {
        p->tip_b[i] = tip_b[i];
        p->ant_b[i] = ant_b[i];
    }
    /* Pole axis, antenna to tip, for the twist figure. */
    float ax[3], n = 0.0f;
    for (int i = 0; i < 3; i++) {
        ax[i] = tip_b[i] - ant_b[i];
        n += ax[i] * ax[i];
    }
    n = sqrtf(n);
    for (int i = 0; i < 3; i++) p->axis_b[i] = n > 0.0f ? ax[i] / n : (i == 2 ? 1.0f : 0.0f);
    if (p->st.active != active) {
        p->st.planted = p->st.latched = false;
        p->st.mag_trusted = false;
        p->latch_wanted = false;
        fit_clear(p);
    }
    p->st.active = active;
}

void pole_aid_reset(pole_aid_t *p, const pole_aid_cfg_t *cfg, bool active, const float tip_b[3],
                    const float ant_b[3])
{
    memset(p, 0, sizeof *p);
    p->cfg = *cfg;
    p->st.tip_speed_ms = -1.0f;
    p->st.mag_res_mean_deg = NAN;
    p->st.mag_res_sd_deg = NAN;
    pole_aid_set_geometry(p, active, tip_b, ant_b);
}

/* 4x4 inverse by Gauss-Jordan with partial pivoting. False if singular. */
static bool inv4(double A[4][4], double I[4][4])
{
    for (int r = 0; r < 4; r++)
        for (int c = 0; c < 4; c++) I[r][c] = r == c ? 1.0 : 0.0;
    for (int c = 0; c < 4; c++) {
        int piv = c;
        for (int r = c + 1; r < 4; r++)
            if (fabs(A[r][c]) > fabs(A[piv][c])) piv = r;
        if (fabs(A[piv][c]) < 1e-12) return false;
        if (piv != c) {
            for (int k = 0; k < 4; k++) {
                double t = A[c][k]; A[c][k] = A[piv][k]; A[piv][k] = t;
                t = I[c][k]; I[c][k] = I[piv][k]; I[piv][k] = t;
            }
        }
        double d = 1.0 / A[c][c];
        for (int k = 0; k < 4; k++) {
            A[c][k] *= d;
            I[c][k] *= d;
        }
        for (int r = 0; r < 4; r++) {
            if (r == c) continue;
            double f = A[r][c];
            if (f == 0.0) continue;
            for (int k = 0; k < 4; k++) {
                A[r][k] -= f * A[c][k];
                I[r][k] -= f * I[c][k];
            }
        }
    }
    return true;
}

/* Ends the episode's fit: solve, judge, and offer the yaw if good. */
static void fit_finish(pole_aid_t *p)
{
    const pole_aid_cfg_t *c = &p->cfg;
    pole_aid_status_t *st = &p->st;
    uint16_t n = p->fit_n;
    if (n == 0) {
        fit_clear(p);
        return;
    }
    double mN = p->a_sum[0] / n, mE = p->a_sum[1] / n;
    double var = p->a_sq / n - (mN * mN + mE * mE);
    float spread = var > 0.0 ? (float)sqrt(var) : 0.0f;
    st->fit_n = n;
    st->fit_spread_m = spread;
    st->fit_ok = false;
    st->fit_skipped = n < c->fit_min_n || spread < c->fit_min_spread_m;
    if (st->fit_skipped) {
        /* Not enough sway to say anything: not counted as a rejection, and
         * the figures of the last solved fit no longer apply. */
        st->fit_dpsi_deg = st->fit_sd_deg = st->fit_rms_m = st->fit_scale = 0.0f;
        fit_clear(p);
        return;
    }
    double A[4][4], I[4][4];
    memcpy(A, p->S, sizeof A);
    if (!inv4(A, I)) {
        st->yaw_rejects++;
        fit_clear(p);
        return;
    }
    double x[4];
    for (int r = 0; r < 4; r++) {
        x[r] = 0.0;
        for (int k = 0; k < 4; k++) x[r] += I[r][k] * p->b[k];
    }
    double rss = p->pp;
    for (int r = 0; r < 4; r++) rss -= x[r] * p->b[r];
    if (rss < 0.0) rss = 0.0;
    float rms = (float)sqrt(rss / (2.0 * n));
    float scale = (float)sqrt(x[2] * x[2] + x[3] * x[3]);
    float dpsi = (float)atan2(x[3], x[2]);
    double s2 = rss / (double)(2 * n - 4);
    float sd = scale > 0.0f ? (float)sqrt(s2 * (I[2][2] + I[3][3]) * 0.5) / scale : 1.0f;
    float sd_min = c->fit_min_sd_deg * D2R;
    if (sd < sd_min) sd = sd_min;
    st->fit_dpsi_deg = dpsi * R2D;
    st->fit_sd_deg = sd * R2D;
    st->fit_rms_m = rms;
    st->fit_scale = scale;
    st->twist_deg = (p->twist_max - p->twist_min) * R2D;
    if (fabsf(scale - 1.0f) <= c->fit_max_scale_err && p->ep.n == n) {
        /* Item 5: offered whatever the RMS, which a mounting error raises
         * once the pole twists; the axis fit judges the residual itself. */
        p->ep_out = p->ep;
        p->ep_out.dpsi = dpsi;
        p->ep_out.twist_deg = st->twist_deg;
        p->ep_ready = true;
        st->episodes++;
    }
    if (rms <= c->fit_max_rms_m && fabsf(scale - 1.0f) <= c->fit_max_scale_err) {
        st->fit_ok = true;
        st->yaw_fits++;
        p->yaw_ready = true;
        p->yaw_dpsi = dpsi;
        p->yaw_sd = sd;
    } else {
        st->yaw_rejects++;
    }
    fit_clear(p);
}

static void release(pole_aid_t *p)
{
    if (p->st.planted) fit_finish(p);
    p->st.planted = false;
    p->st.latched = false;
    p->latch_wanted = false;
}

bool pole_aid_imu(pole_aid_t *p, float dt, const float f[3], const float w[3], const float *v_tip_n)
{
    if (!p->st.active) return false;
    if (!(dt > 0.0f) || dt > 0.1f) dt = 0.00125f;
    p->t_s += dt;
    if (p->qg_ok) pole_quat_step(p->qg, w, dt);
    if (p->st.planted) {
        p->twist += (w[0] * p->axis_b[0] + w[1] * p->axis_b[1] + w[2] * p->axis_b[2]) * dt;
        if (p->twist < p->twist_min) p->twist_min = p->twist;
        if (p->twist > p->twist_max) p->twist_max = p->twist;
    }

    /* Low-passed rate (20 Hz) and its derivative over POLE_DIFF_N samples. */
    float a = dt / (0.008f + dt);
    for (int i = 0; i < 3; i++) p->w_lp[i] += a * (w[i] - p->w_lp[i]);
    uint8_t j = p->hist_i;
    float wd[3] = { 0.0f, 0.0f, 0.0f };
    if (p->hist_n == POLE_DIFF_N) {
        float span = p->t_s - p->t_hist[j];
        if (span > 0.0f)
            for (int i = 0; i < 3; i++) wd[i] = (p->w_lp[i] - p->w_hist[j][i]) / span;
    } else {
        p->hist_n++;
    }
    for (int i = 0; i < 3; i++) p->w_hist[j][i] = p->w_lp[i];
    p->t_hist[j] = p->t_s;
    p->hist_i = (uint8_t)((j + 1) % POLE_DIFF_N);

    /* Specific force at the tip. */
    float wl[3], wwl[3], wdl[3], ft[3];
    cross(w, p->tip_b, wl);
    cross(w, wl, wwl);
    cross(wd, p->tip_b, wdl);
    for (int i = 0; i < 3; i++) ft[i] = f[i] + wwl[i] + wdl[i];
    float e = fabsf(sqrtf(ft[0] * ft[0] + ft[1] * ft[1] + ft[2] * ft[2]) - p->cfg.g);

    if (p->blk_cnt == 0) {
        p->blk_sum = 0.0f;
        p->blk_peak = 0.0f;
        p->blk_vmax = 0.0f;
        p->blk_vknown = true;
        p->blk_t0 = p->t_s;
    }
    p->blk_cnt++;
    p->blk_sum += e;
    if (e > p->blk_peak) p->blk_peak = e;
    if (v_tip_n != NULL) {
        float v = sqrtf(v_tip_n[0] * v_tip_n[0] + v_tip_n[1] * v_tip_n[1] + v_tip_n[2] * v_tip_n[2]);
        if (v > p->blk_vmax) p->blk_vmax = v;
    } else {
        p->blk_vknown = false;
    }
    if (p->t_s - p->blk_t0 < 0.05f) return p->st.planted;

    /* A 50 ms block is complete: judge the window. */
    p->win_mean[p->win_i] = p->blk_sum / p->blk_cnt;
    p->win_peak[p->win_i] = p->blk_peak;
    p->win_v[p->win_i] = p->blk_vmax;
    p->win_vknown[p->win_i] = p->blk_vknown;
    p->win_i = (uint8_t)((p->win_i + 1) % POLE_WIN_BLOCKS);
    if (p->win_n < POLE_WIN_BLOCKS) p->win_n++;
    p->blk_cnt = 0;

    float mean = 0.0f, peak = 0.0f, vmax = 0.0f;
    bool vknown = true;
    for (int k = 0; k < p->win_n; k++) {
        mean += p->win_mean[k];
        if (p->win_peak[k] > peak) peak = p->win_peak[k];
        if (p->win_v[k] > vmax) vmax = p->win_v[k];
        vknown = vknown && p->win_vknown[k];
    }
    mean /= p->win_n;
    p->st.tip_err_ms2 = mean;
    p->st.tip_speed_ms = vknown ? vmax : -1.0f;

    bool ok = p->win_n == POLE_WIN_BLOCKS && mean <= p->cfg.err_mean_max && peak <= p->cfg.err_peak_max &&
              vknown && vmax <= p->cfg.tip_speed_max;
    if (!ok) {
        release(p);
        return false;
    }
    if (!p->st.planted) {
        p->st.planted = true;
        p->st.plants++;
        p->t_planted = p->t_s;
        p->t_episode = p->t_s;
        fit_clear(p);
    }
    if (!p->st.latched && !p->latch_wanted && p->t_s - p->t_planted >= p->cfg.settle_s) {
        p->latch_wanted = true;
    }
    if (p->st.latched && p->t_s - p->t_episode >= p->cfg.episode_max_s) {
        /* Long occupation: fit what we have and latch afresh, so neither
         * the fit nor the latch carries a stale yaw or position for long. */
        fit_finish(p);
        p->t_episode = p->t_s;
        p->st.latched = false;
        p->latch_wanted = true;
        p->qg_resync = true;
    }
    return true;
}

bool pole_aid_want_latch(const pole_aid_t *p) { return p->st.active && p->latch_wanted; }

void pole_aid_latch(pole_aid_t *p, const float tip_n[3])
{
    for (int i = 0; i < 3; i++) p->tip_n[i] = tip_n[i];
    p->latch_wanted = false;
    p->st.latched = true;
    p->t_next_feed = p->t_s;
}

bool pole_aid_feed_due(pole_aid_t *p, float tip_n[3])
{
    if (!p->st.active || !p->st.latched || p->t_s < p->t_next_feed) return false;
    p->t_next_feed = p->t_s + p->cfg.feed_period_s;
    for (int i = 0; i < 3; i++) tip_n[i] = p->tip_n[i];
    p->st.tip_fusions++;
    return true;
}

void pole_aid_att(pole_aid_t *p, uint64_t t_us, const float R[9])
{
    if (!p->st.active || t_us < p->att_next_us) return;
    p->att_next_us = t_us + 10000u;
    if (!p->qg_ok || !p->st.planted || p->qg_resync) {
        /* Between episodes the gyro attitude follows the INS; while planted
         * it runs on the gyro alone. */
        pole_quat_from_R(R, p->qg);
        p->qg_ok = true;
        p->qg_resync = false;
    }
    p->att_t[p->att_i] = t_us;
    memcpy(p->att_R[p->att_i], R, sizeof p->att_R[0]);
    memcpy(p->att_q[p->att_i], p->qg, sizeof p->att_q[0]);
    p->att_i = (uint8_t)((p->att_i + 1) % POLE_ATT_N);
    if (p->att_n < POLE_ATT_N) p->att_n++;
}

void pole_aid_gnss(pole_aid_t *p, uint64_t t_valid_us, double lat_rad, double lon_rad, float sd_h_m)
{
    if (!p->st.active || !p->st.latched || !(sd_h_m <= p->cfg.fit_sd_max_m)) return;
    /* The attitude nearest the time of validity, within 20 ms. */
    int best = -1;
    uint64_t best_d = 20001u;
    for (int k = 0; k < p->att_n; k++) {
        uint64_t d = p->att_t[k] > t_valid_us ? p->att_t[k] - t_valid_us : t_valid_us - p->att_t[k];
        if (d < best_d) {
            best_d = d;
            best = k;
        }
    }
    if (best < 0) return;
    float l[3], a[3];
    for (int i = 0; i < 3; i++) l[i] = p->ant_b[i] - p->tip_b[i];
    rot(p->att_R[best], l, a);

    if (!p->fit_origin) {
        p->fit_origin = true;
        p->lat0 = lat_rad;
        p->lon0 = lon_rad;
        p->k_n = EARTH_R;
        p->k_e = EARTH_R * cosf((float)lat_rad);
    }
    double pN = (lat_rad - p->lat0) * p->k_n;
    double pE = (lon_rad - p->lon0) * p->k_e;
    float Rg[9];
    pole_quat_to_R(p->att_q[best], Rg);
    pole_episode_add(&p->ep, Rg, (float)pN, (float)pE);
    double aN = a[0], aE = a[1];
    /* Rows [1 0 aN -aE] (north) and [0 1 aE aN] (east), unknowns
     * (tN, tE, cos dpsi, sin dpsi). */
    p->S[0][0] += 1.0;
    p->S[1][1] += 1.0;
    p->S[0][2] += aN;
    p->S[0][3] -= aE;
    p->S[1][2] += aE;
    p->S[1][3] += aN;
    p->S[2][2] += aN * aN + aE * aE;
    p->S[3][3] += aN * aN + aE * aE;
    p->S[2][0] = p->S[0][2];
    p->S[3][0] = p->S[0][3];
    p->S[2][1] = p->S[1][2];
    p->S[3][1] = p->S[1][3];
    p->b[0] += pN;
    p->b[1] += pE;
    p->b[2] += aN * pN + aE * pE;
    p->b[3] += -aE * pN + aN * pE;
    p->pp += pN * pN + pE * pE;
    p->a_sum[0] += aN;
    p->a_sum[1] += aE;
    p->a_sq += aN * aN + aE * aE;
    if (p->fit_n < 0xFFFFu) p->fit_n++;
}

bool pole_aid_take_yaw(pole_aid_t *p, float *dpsi, float *sd)
{
    if (!p->yaw_ready) return false;
    p->yaw_ready = false;
    *dpsi = p->yaw_dpsi;
    *sd = p->yaw_sd;
    return true;
}

/* Magnetometer: 1 s blocks, judged over the last POLE_MAG_BLOCKS. */
static void mag_judge(pole_aid_t *p)
{
    const pole_aid_cfg_t *c = &p->cfg;
    pole_aid_status_t *st = &p->st;
    int filled = 0, res_blocks = 0;
    float nerr = 0.0f, derr = 0.0f, tmin = 1e9f, tmax = -1e9f;
    float rs = 0.0f, rq = 0.0f;
    uint32_t rn = 0;
    for (int k = 0; k < p->mag_n; k++) {
        if (p->mag_blk[k].n == 0) continue;
        filled++;
        if (p->mag_blk[k].norm_err > nerr) nerr = p->mag_blk[k].norm_err;
        if (p->mag_blk[k].dip_err > derr) derr = p->mag_blk[k].dip_err;
        if (p->mag_blk[k].tilt_min < tmin) tmin = p->mag_blk[k].tilt_min;
        if (p->mag_blk[k].tilt_max > tmax) tmax = p->mag_blk[k].tilt_max;
        if (p->mag_blk[k].n_res > 0) {
            res_blocks++;
            rs += p->mag_blk[k].res_sum;
            rq += p->mag_blk[k].res_sq;
            rn += p->mag_blk[k].n_res;
        }
    }
    /* Widest yaw difference between block means. */
    float yrange = 0.0f;
    for (int a = 0; a < p->mag_n; a++) {
        if (p->mag_blk[a].n == 0) continue;
        float ya = atan2f(p->mag_blk[a].yaw_s, p->mag_blk[a].yaw_c);
        for (int b = a + 1; b < p->mag_n; b++) {
            if (p->mag_blk[b].n == 0) continue;
            float d = fabsf(wrap_pi(atan2f(p->mag_blk[b].yaw_s, p->mag_blk[b].yaw_c) - ya));
            if (d > yrange) yrange = d;
        }
    }
    st->mag_blocks = filled;
    st->mag_norm_err_pct = nerr * 100.0f;
    st->mag_dip_err_deg = derr * R2D;
    st->mag_tilt_range_deg = filled ? (tmax - tmin) * R2D : 0.0f;
    st->mag_yaw_range_deg = yrange * R2D;
    bool res_checked = res_blocks >= POLE_MAG_BLOCKS - 2 && rn > 1;
    float rmean = 0.0f, rsd = 0.0f;
    if (res_checked) {
        rmean = rs / rn;
        float v = rq / rn - rmean * rmean;
        rsd = v > 0.0f ? sqrtf(v) : 0.0f;
        st->mag_res_mean_deg = rmean * R2D;
        st->mag_res_sd_deg = rsd * R2D;
    } else {
        st->mag_res_mean_deg = NAN;
        st->mag_res_sd_deg = NAN;
    }

    /* The latest block alone decides whether trust is kept. */
    int last = (p->mag_i + POLE_MAG_BLOCKS - 1) % POLE_MAG_BLOCKS;
    bool last_ok = p->mag_blk[last].n > 0 && p->mag_blk[last].norm_err <= c->mag_norm_tol &&
                   p->mag_blk[last].dip_err <= c->mag_dip_tol_deg * D2R;
    bool res_ok = !res_checked || (fabsf(rmean) <= c->mag_res_mean_max_deg * D2R &&
                                   rsd <= c->mag_res_sd_max_deg * D2R);
    if (st->mag_trusted) {
        st->mag_trusted = last_ok && res_ok;
        return;
    }
    bool varied = st->mag_tilt_range_deg >= c->mag_tilt_range_deg || st->mag_yaw_range_deg >= c->mag_yaw_range_deg;
    st->mag_trusted = filled >= POLE_MAG_BLOCKS - 2 && nerr <= c->mag_norm_tol &&
                      derr <= c->mag_dip_tol_deg * D2R && varied && res_ok;
}

bool pole_aid_mag(pole_aid_t *p, uint64_t t_us, const float b[3], const float R[9], bool yaw_abs,
                  float yaw_sd_rad, const float m_n[3])
{
    if (!p->st.active) return true; /* not this module's call */
    float mref = sqrtf(m_n[0] * m_n[0] + m_n[1] * m_n[1] + m_n[2] * m_n[2]);
    if (mref < 1.0f) {
        /* No WMM yet (no position): nothing to compare with. */
        p->st.mag_trusted = false;
        p->st.mag_blocks = 0;
        p->mag_n = 0;
        p->mag_i = 0;
        memset(&p->mag_cur, 0, sizeof p->mag_cur);
        p->mag_t0_us = t_us;
        return false;
    }
    if (p->mag_cur.n == 0) {
        p->mag_t0_us = t_us;
        p->mag_cur.tilt_min = 1e9f;
        p->mag_cur.tilt_max = -1e9f;
    }
    float B[3];
    rot(R, b, B);
    float bn = sqrtf(B[0] * B[0] + B[1] * B[1] + B[2] * B[2]);
    if (bn > 1.0f) {
        float ne = fabsf(bn / mref - 1.0f);
        float dip = asinf(B[2] / bn), dref = asinf(m_n[2] / mref);
        float de = fabsf(dip - dref);
        float tilt = acosf(fmaxf(-1.0f, fminf(1.0f, R[8])));
        float yaw = atan2f(R[1], R[0]);
        if (ne > p->mag_cur.norm_err) p->mag_cur.norm_err = ne;
        if (de > p->mag_cur.dip_err) p->mag_cur.dip_err = de;
        if (tilt < p->mag_cur.tilt_min) p->mag_cur.tilt_min = tilt;
        if (tilt > p->mag_cur.tilt_max) p->mag_cur.tilt_max = tilt;
        p->mag_cur.yaw_c += cosf(yaw);
        p->mag_cur.yaw_s += sinf(yaw);
        if (yaw_abs && yaw_sd_rad <= p->cfg.mag_yaw_sd_max_deg * D2R) {
            /* Angle from the WMM's horizontal field to the measured one: the
             * INS yaw error as the magnetometer sees it. */
            float res = atan2f(m_n[0] * B[1] - m_n[1] * B[0], m_n[0] * B[0] + m_n[1] * B[1]);
            p->mag_cur.res_sum += res;
            p->mag_cur.res_sq += res * res;
            p->mag_cur.n_res++;
        }
        p->mag_cur.n++;
    }
    if (t_us - p->mag_t0_us >= 1000000u) {
        p->mag_blk[p->mag_i] = p->mag_cur;
        p->mag_i = (uint8_t)((p->mag_i + 1) % POLE_MAG_BLOCKS);
        if (p->mag_n < POLE_MAG_BLOCKS) p->mag_n++;
        memset(&p->mag_cur, 0, sizeof p->mag_cur);
        mag_judge(p);
    }
    return p->st.mag_trusted;
}

bool pole_aid_take_episode(pole_aid_t *p, pole_episode_t *out)
{
    if (!p->ep_ready) return false;
    p->ep_ready = false;
    *out = p->ep_out;
    return true;
}

const pole_aid_status_t *pole_aid_status(const pole_aid_t *p) { return &p->st; }
