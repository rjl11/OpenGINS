#include "flow_nav.h"

#include <math.h>
#include <string.h>

uint32_t flow_nav_period_of(uint32_t ordinal)
{
    /* Each run of FLOW_LINK_GAP_PERIOD periods holds GAP_PERIOD - 1 pulses. */
    return ordinal + (ordinal - 1u) / (FLOW_LINK_GAP_PERIOD - 1u);
}

uint32_t flow_nav_ordinal_of(uint32_t period)
{
    return period - period / FLOW_LINK_GAP_PERIOD;
}

/* Camera -> body at cam_yaw 0: image x (right) = body y, image y (down in
 * the image) = body -x (image top looks forward), optical axis = body z
 * (down). Then turned by cam_yaw about body z. */
static void set_rotation(flow_nav_t *s)
{
    const float c = cosf(s->cfg.cam_yaw_rad), sn = sinf(s->cfg.cam_yaw_rad);
    /* Rz(yaw) * M0, M0 = [[0,-1,0],[1,0,0],[0,0,1]] (columns: x_c, y_c, z_c in body) */
    const float m0[9] = { 0, -1, 0, 1, 0, 0, 0, 0, 1 };
    const float rz[9] = { c, -sn, 0, sn, c, 0, 0, 0, 1 };
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            float a = 0.0f;
            for (int k = 0; k < 3; k++) a += rz[i * 3 + k] * m0[k * 3 + j];
            s->r_cb[i * 3 + j] = a;
        }
    }
}

void flow_nav_init(flow_nav_t *s, const flow_nav_config_t *cfg)
{
    memset(s, 0, sizeof *s);
    s->cfg = *cfg;
    set_rotation(s);
    s->cos_tilt = 1.0f;
    s->tilt_ok = true;
}

void flow_nav_set_geometry(flow_nav_t *s, float height_m, float cam_yaw_rad, const float leverarm_m[3])
{
    s->cfg.height_m = height_m;
    s->cfg.cam_yaw_rad = cam_yaw_rad;
    for (int i = 0; i < 3; i++) s->cfg.leverarm_m[i] = leverarm_m[i];
    set_rotation(s);
    s->win_n = 0;
    s->win_vx = s->win_vy = s->win_var = s->win_t = 0.0;
}

void flow_nav_gyro(flow_nav_t *s, uint64_t t_us, const float gyr_b[3])
{
    flow_nav_gyro_t *g = &s->gyro[s->gyro_n % FLOW_NAV_GYRO_RING];
    g->t = t_us;
    g->g[0] = gyr_b[0];
    g->g[1] = gyr_b[1];
    g->g[2] = gyr_b[2];
    s->gyro_n++;
}

/* Rotation over (ta, tb] [rad, body], each gyro sample held over the
 * interval since the one before. False if the ring doesn't cover it. */
static bool gyro_integral(const flow_nav_t *s, uint64_t ta, uint64_t tb, float out[3])
{
    out[0] = out[1] = out[2] = 0.0f;
    uint32_t n = s->gyro_n;
    if (n < 2u || tb <= ta) return false;
    uint32_t oldest = n > FLOW_NAV_GYRO_RING ? n - FLOW_NAV_GYRO_RING : 0u;
    if (s->gyro[(n - 1u) % FLOW_NAV_GYRO_RING].t < tb) return false; /* not yet covered */
    if (s->gyro[oldest % FLOW_NAV_GYRO_RING].t > ta) return false;   /* already gone */
    double acc[3] = { 0, 0, 0 };
    for (uint32_t i = n - 1u; i > oldest; i--) {
        const flow_nav_gyro_t *cur = &s->gyro[i % FLOW_NAV_GYRO_RING];
        const flow_nav_gyro_t *prev = &s->gyro[(i - 1u) % FLOW_NAV_GYRO_RING];
        if (cur->t <= ta) break;
        uint64_t a = prev->t > ta ? prev->t : ta;
        uint64_t b = cur->t < tb ? cur->t : tb;
        if (b > a) {
            double dt = (double)(b - a) * 1e-6;
            for (int k = 0; k < 3; k++) acc[k] += cur->g[k] * dt;
        }
    }
    for (int k = 0; k < 3; k++) out[k] = (float)acc[k];
    return true;
}

void flow_nav_event(flow_nav_t *s, const flow_event_t *e)
{
    if (e->kind == FLOW_EV_TRIGGER) {
        flow_nav_exp_t *x = &s->exp[e->seq % FLOW_NAV_EXP_RING];
        x->period = e->seq;
        x->t_trig = e->t_us;
        x->t_fall = x->t_rise = 0;
        s->last_trig_period = e->seq;
    } else if (e->kind == FLOW_EV_GAP) {
        s->have_gap = true;
        s->gap_period = e->seq;
        s->gap_t = e->t_us;
    } else if (e->kind == FLOW_EV_STROBE && s->last_trig_period != 0u) {
        flow_nav_exp_t *x = &s->exp[s->last_trig_period % FLOW_NAV_EXP_RING];
        if (x->period != s->last_trig_period || e->t_us < x->t_trig) return;
        if (e->level == 0) { /* falling: exposure starts (active low) */
            if (x->t_fall == 0 && e->t_us - x->t_trig <= s->cfg.strobe_window_us) x->t_fall = e->t_us;
        } else if (x->t_fall != 0 && x->t_rise == 0) {
            x->t_rise = e->t_us;
        }
    }
}

void flow_nav_attitude(flow_nav_t *s, bool valid, float roll_rad, float pitch_rad)
{
    if (!valid) {
        s->cos_tilt = 1.0f;
        s->tilt_ok = true;
        return;
    }
    s->tilt_ok = fabsf(roll_rad) <= s->cfg.max_tilt_rad && fabsf(pitch_rad) <= s->cfg.max_tilt_rad;
    s->cos_tilt = cosf(roll_rad) * cosf(pitch_rad);
}

/* Exposure middle of the frame at this pulse ordinal; 0 if unknown. */
static uint64_t exposure_mid(flow_nav_t *s, uint32_t ordinal, uint32_t exposure_us)
{
    uint32_t p = flow_nav_period_of(ordinal);
    const flow_nav_exp_t *x = &s->exp[p % FLOW_NAV_EXP_RING];
    if (x->period != p || x->t_trig == 0) return 0;
    if (x->t_fall != 0 && x->t_rise > x->t_fall) {
        s->st.strobe_timed++;
        return x->t_fall + (x->t_rise - x->t_fall) / 2u;
    }
    return x->t_trig + s->cfg.exposure_delay_us + exposure_us / 2u;
}

bool flow_nav_message(flow_nav_t *s, const flow_msg_flow_t *m, uint64_t t_arrival_us, flow_nav_speed_t *out)
{
    s->st.msgs++;

    /* Anchoring first, whatever else is wrong with this message. */
    if (m->flags & FLOW_F_DROPPED) {
        if (s->anchored) s->st.breaks++;
        s->anchored = false;
        /* Frames from before the break don't mix with later ones. */
        s->win_n = 0;
        s->win_vx = s->win_vy = s->win_var = s->win_t = 0.0;
    }
    if ((m->flags & FLOW_F_AFTER_GAP) && s->have_gap && s->gap_t < t_arrival_us) {
        uint32_t ord = flow_nav_ordinal_of(s->gap_period + 1u);
        if (s->anchored && s->anchor_ord + (m->frame_seq - s->anchor_seq) != ord) s->st.resyncs++;
        s->anchored = true;
        s->anchor_seq = m->frame_seq;
        s->anchor_ord = ord;
        s->st.anchors++;
    }

    if (!(m->flags & FLOW_F_TRIGGERED)) {
        s->st.untriggered++;
        return false;
    }
    if (!s->anchored) {
        s->st.not_anchored++;
        return false;
    }
    if (!(m->flags & FLOW_F_VALID) || m->n_tracked < s->cfg.min_features) {
        s->st.invalid++;
        return false;
    }
    if (!s->tilt_ok) {
        s->st.too_tilted++;
        return false;
    }

    const uint32_t ord = s->anchor_ord + (m->frame_seq - s->anchor_seq);
    const uint32_t exp_us = (uint32_t)m->exposure_100us * 100u;
    const uint64_t t1 = exposure_mid(s, ord, exp_us);
    const uint64_t t0 = ord > 1u ? exposure_mid(s, ord - 1u, exp_us) : 0;
    if (t0 == 0 || t1 == 0) {
        s->st.no_exposure++;
        return false;
    }
    if (t1 <= t0 || t1 - t0 > 100000u) {
        s->st.bad_dt++;
        return false;
    }
    float dth_b[3];
    if (!gyro_integral(s, t0, t1, dth_b)) {
        s->st.no_gyro++;
        return false;
    }
    const float dt = (float)(t1 - t0) * 1e-6f;
    const float *r = s->r_cb;

    /* Rotation into camera axes: dth_c = R_cb^T dth_b. */
    float dth_c[3];
    for (int i = 0; i < 3; i++) dth_c[i] = r[0 * 3 + i] * dth_b[0] + r[1 * 3 + i] * dth_b[1] + r[2 * 3 + i] * dth_b[2];

    const float z = s->cfg.height_m / (s->cos_tilt > 0.2f ? s->cos_tilt : 0.2f);
    const float vcx = -z * (m->disp_x_rad + dth_c[1]) / dt;
    const float vcy = -z * (m->disp_y_rad - dth_c[0]) / dt;

    /* Camera velocity in body axes (the optical-axis component is not
     * measured; a ground vehicle has none), then to the IMU point:
     * v_imu = v_cam - w x r. */
    float v[3];
    for (int i = 0; i < 3; i++) v[i] = r[i * 3 + 0] * vcx + r[i * 3 + 1] * vcy;
    const float w[3] = { dth_b[0] / dt, dth_b[1] / dt, dth_b[2] / dt };
    const float *la = s->cfg.leverarm_m;
    v[0] -= w[1] * la[2] - w[2] * la[1];
    v[1] -= w[2] * la[0] - w[0] * la[2];

    const float spd = sqrtf(v[0] * v[0] + v[1] * v[1]);
    if (!(spd <= s->cfg.max_mps)) {
        s->st.too_fast++;
        return false;
    }
    const float sd = z * m->stddev_rad / dt;

    /* A window left half full (frames rejected since) is stale: start over. */
    if (s->win_n != 0 && (float)(t1 - s->win_t0) * 1e-6f > 2.0f * s->cfg.fuse_period_s) {
        s->win_n = 0;
        s->win_vx = s->win_vy = s->win_var = s->win_t = 0.0;
    }
    if (s->win_n == 0) s->win_t0 = t1;
    s->win_n++;
    s->win_vx += v[0];
    s->win_vy += v[1];
    s->win_var += (double)sd * sd;
    s->win_t += (double)(t1 - s->win_t0);
    s->st.used++;

    if ((float)(t1 - s->win_t0) * 1e-6f + dt < s->cfg.fuse_period_s) return false;

    const double n = (double)s->win_n;
    flow_nav_speed_t o;
    o.v_b[0] = (float)(s->win_vx / n);
    o.v_b[1] = (float)(s->win_vy / n);
    o.speed_mps = sqrtf(o.v_b[0] * o.v_b[0] + o.v_b[1] * o.v_b[1]);
    o.t_valid_us = s->win_t0 + (uint64_t)(s->win_t / n);
    /* Frame noise averages down; the height error does not. */
    const double noise = s->win_var / (n * n);
    const double rel = (double)s->cfg.height_rel_sd * o.speed_mps;
    float sdo = (float)sqrt(noise + rel * rel);
    o.sd_mps = sdo > s->cfg.min_sd_mps ? sdo : s->cfg.min_sd_mps;
    o.n_frames = (uint16_t)s->win_n;
    s->win_n = 0;
    s->win_vx = s->win_vy = s->win_var = s->win_t = 0.0;

    s->last = o;
    s->have_last = true;
    s->st.samples++;
    *out = o;
    return true;
}
