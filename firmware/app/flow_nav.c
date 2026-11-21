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
    memset(s, 0, sizeof *s); /* no clock sync until flow_nav_set_clock() */
    s->cfg = *cfg;
    set_rotation(s);
    s->cos_tilt = 1.0f;
    s->tilt_ok = true;
}

void flow_nav_set_clock(flow_nav_t *s, flow_nav_clock_fn fn, void *ctx)
{
    s->clock = fn;
    s->clock_ctx = ctx;
}

void flow_nav_set_geometry(flow_nav_t *s, float height_m, float cam_yaw_rad, const float leverarm_m[3])
{
    s->cfg.height_m = height_m;
    s->cfg.cam_yaw_rad = cam_yaw_rad;
    for (int i = 0; i < 3; i++) s->cfg.leverarm_m[i] = leverarm_m[i];
    set_rotation(s);
    s->win_n = 0;
    s->win_vx = s->win_vy = s->win_var = s->win_rel2 = s->win_t = 0.0;
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

/* Free-running: tracks the clock offset with every message (flow_nav.h). */
static void free_track(flow_nav_t *s, uint32_t pi_time_us, uint64_t t_arr)
{
    const uint32_t off = (uint32_t)t_arr - pi_time_us;
    if (!s->free_have) {
        s->free_have = true;
        s->free_off = off;
        s->free_leak_t = t_arr;
        return;
    }
    const uint64_t div = 1000000u / FLOW_NAV_FREE_LEAK_PPM; /* us per 1 us of creep */
    if (t_arr > s->free_leak_t + div) {
        const uint64_t k = (t_arr - s->free_leak_t) / div;
        s->free_off += (uint32_t)k;
        s->free_leak_t += k * div;
    }
    const int32_t lag = (int32_t)(off - s->free_off);
    if (lag < 0) {
        s->free_off = off;
    } else if ((uint32_t)lag > FLOW_NAV_FREE_MAX_LAG_US) {
        s->free_off = off;
        s->free_leak_t = t_arr;
        s->st.free_resets++;
    }
}

static void win_clear(flow_nav_t *s)
{
    s->win_n = 0;
    s->win_vx = s->win_vy = s->win_var = s->win_rel2 = s->win_t = 0.0;
}

/* The fields both message types share. */
typedef struct {
    uint32_t frame_seq, pi_time_us, pi_dt_us;
    uint16_t n_tracked;
    uint8_t  exposure_100us, flags;
} frame_hdr_t;

/* Anchoring, the free-running clock and the gates every message passes,
 * then the two frames' exposure times. False (counted) if the message
 * can't be used. */
static bool frame_times(flow_nav_t *s, const frame_hdr_t *m, uint64_t t_arrival_us, uint64_t *t0_out,
                        uint64_t *t1_out)
{
    s->st.msgs++;

    /* Anchoring first, whatever else is wrong with this message. */
    if (m->flags & FLOW_F_DROPPED) {
        if (s->anchored) s->st.breaks++;
        s->anchored = false;
        /* Frames from before the break don't mix with later ones. */
        win_clear(s);
    }
    if ((m->flags & FLOW_F_AFTER_GAP) && s->have_gap && s->gap_t < t_arrival_us) {
        uint32_t ord = flow_nav_ordinal_of(s->gap_period + 1u);
        if (s->anchored && s->anchor_ord + (m->frame_seq - s->anchor_seq) != ord) s->st.resyncs++;
        s->anchored = true;
        s->anchor_seq = m->frame_seq;
        s->anchor_ord = ord;
        s->st.anchors++;
    }

    const bool free_run = !(m->flags & FLOW_F_TRIGGERED);
    if (free_run) {
        s->st.untriggered++;
        free_track(s, m->pi_time_us, t_arrival_us);
    } else if (!s->anchored) {
        s->st.not_anchored++;
        return false;
    }
    /* A phase-correlation frame has no feature count (n_tracked is its
     * response); the Pi has already gated it on the response. */
    const bool phase = (m->flags & FLOW_F_PHASE) != 0;
    if (!(m->flags & FLOW_F_VALID) || (!phase && m->n_tracked < s->cfg.min_features)) {
        s->st.invalid++;
        return false;
    }
    if (!s->tilt_ok) {
        s->st.too_tilted++;
        return false;
    }

    uint64_t t0, t1;
    uint64_t t_sync;
    if (free_run && s->clock != NULL && s->clock(s->clock_ctx, m->pi_time_us, t_arrival_us, &t_sync)) {
        t1 = (uint64_t)((int64_t)t_sync + s->cfg.sync_ts_to_mid_us);
        if (m->pi_dt_us == 0u || t1 <= m->pi_dt_us || t1 > t_arrival_us) {
            s->st.bad_dt++;
            return false;
        }
        t0 = t1 - m->pi_dt_us;
        s->st.synced++;
    } else if (free_run) {
        const uint64_t back = (uint64_t)(uint32_t)(((uint32_t)t_arrival_us - m->pi_time_us) - s->free_off) +
                              s->cfg.free_latency_us;
        if (m->pi_dt_us == 0u || t_arrival_us <= back + m->pi_dt_us) {
            s->st.bad_dt++;
            return false;
        }
        t1 = t_arrival_us - back;
        t0 = t1 - m->pi_dt_us;
    } else {
        const uint32_t ord = s->anchor_ord + (m->frame_seq - s->anchor_seq);
        const uint32_t exp_us = (uint32_t)m->exposure_100us * 100u;
        t1 = exposure_mid(s, ord, exp_us);
        t0 = ord > 1u ? exposure_mid(s, ord - 1u, exp_us) : 0;
        if (t0 == 0 || t1 == 0) {
            s->st.no_exposure++;
            return false;
        }
    }
    if (t1 <= t0 || t1 - t0 > 100000u) {
        s->st.bad_dt++;
        return false;
    }
    *t0_out = t0;
    *t1_out = t1;
    return true;
}

/* Camera velocity (camera axes x / y) to body axes at the IMU point:
 * v_imu = v_cam - w x r. The optical-axis component is not measured; a
 * ground vehicle has none. False (counted) if faster than max_mps. */
static bool to_imu_point(flow_nav_t *s, float vcx, float vcy, const float dth_b[3], float dt, float v[3])
{
    const float *r = s->r_cb;
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
    return true;
}

/* One frame's velocity into the averaging window: sd is its image-noise
 * 1-sigma per axis, rel the relative 1-sigma of the height it was scaled
 * by. True with *out set when the window is complete. */
static bool accumulate(flow_nav_t *s, uint64_t t1, float dt, const float v[3], float sd, float rel, bool phase,
                       flow_nav_speed_t *out)
{
    /* A window left half full (frames rejected since) is stale: start over. */
    if (s->win_n != 0 && (float)(t1 - s->win_t0) * 1e-6f > 2.0f * s->cfg.fuse_period_s) win_clear(s);
    if (s->win_n == 0) s->win_t0 = t1;
    s->win_n++;
    s->win_vx += v[0];
    s->win_vy += v[1];
    s->win_var += (double)sd * sd;
    s->win_rel2 += (double)rel * rel;
    s->win_t += (double)(t1 - s->win_t0);
    s->st.used++;
    if (phase) s->st.phase++;

    if ((float)(t1 - s->win_t0) * 1e-6f + dt < s->cfg.fuse_period_s) return false;

    const double n = (double)s->win_n;
    flow_nav_speed_t o;
    o.v_b[0] = (float)(s->win_vx / n);
    o.v_b[1] = (float)(s->win_vy / n);
    o.speed_mps = sqrtf(o.v_b[0] * o.v_b[0] + o.v_b[1] * o.v_b[1]);
    o.t_valid_us = s->win_t0 + (uint64_t)(s->win_t / n);
    /* Frame noise averages down; the height error does not (its mean
     * relative variance over the window). */
    const double noise = s->win_var / (n * n);
    const double r2 = s->win_rel2 / n;
    float sdo = (float)sqrt(noise + r2 * o.speed_mps * o.speed_mps);
    o.sd_mps = sdo > s->cfg.min_sd_mps ? sdo : s->cfg.min_sd_mps;
    /* The same two parts as a 2x2 covariance: the frame noise is the same
     * on both axes, the height error scales the whole vector (v v^T times
     * the relative variance). The floor goes on the noise. */
    const double floor2 = (double)s->cfg.min_sd_mps * s->cfg.min_sd_mps;
    const double nv = noise > floor2 ? noise : floor2;
    o.Qll_b[0] = (float)(nv + r2 * o.v_b[0] * o.v_b[0]);
    o.Qll_b[1] = (float)(r2 * o.v_b[0] * o.v_b[1]);
    o.Qll_b[2] = (float)(nv + r2 * o.v_b[1] * o.v_b[1]);
    o.n_frames = (uint16_t)s->win_n;
    win_clear(s);

    s->last = o;
    s->have_last = true;
    s->st.samples++;
    *out = o;
    return true;
}

bool flow_nav_message(flow_nav_t *s, const flow_msg_flow_t *m, uint64_t t_arrival_us, flow_nav_speed_t *out)
{
    const frame_hdr_t h = { m->frame_seq, m->pi_time_us, m->pi_dt_us, m->n_tracked, m->exposure_100us, m->flags };
    uint64_t t0, t1;
    if (!frame_times(s, &h, t_arrival_us, &t0, &t1)) return false;
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

    float v[3];
    if (!to_imu_point(s, vcx, vcy, dth_b, dt, v)) return false;
    const float sd = z * m->stddev_rad / dt;
    return accumulate(s, t1, dt, v, sd, s->cfg.height_rel_sd, (m->flags & FLOW_F_PHASE) != 0, out);
}

bool flow_nav_cam_vel(flow_nav_t *s, const flow_msg_cam_vel_t *m, uint64_t t_arrival_us, flow_nav_speed_t *out)
{
    s->st.cam_vel++;
    const frame_hdr_t h = { m->frame_seq, m->pi_time_us, m->pi_dt_us, m->n_tracked, m->exposure_100us, m->flags };
    uint64_t t0, t1;
    if (!frame_times(s, &h, t_arrival_us, &t0, &t1)) return false;
    if (!(m->height_m > 0.0f) || !isfinite(m->vel_x_mps) || !isfinite(m->vel_y_mps) ||
        !(m->stddev_mps >= 0.0f) || !(m->height_sd_m >= 0.0f)) {
        s->st.invalid++;
        return false;
    }
    /* The rotation is out already; the roof gyro is still what moves the
     * velocity to the IMU point (yaw is shared by cab and chassis). */
    float dth_b[3];
    if (!gyro_integral(s, t0, t1, dth_b)) {
        s->st.no_gyro++;
        return false;
    }
    const float dt = (float)(t1 - t0) * 1e-6f;
    float v[3];
    if (!to_imu_point(s, m->vel_x_mps, m->vel_y_mps, dth_b, dt, v)) return false;

    s->last_height_m = m->height_m;
    s->last_height_range = (m->flags & FLOW_F_RANGE) != 0;
    if (s->last_height_range) s->st.ranged++;
    const float rel = m->height_sd_m > 0.0f ? m->height_sd_m / m->height_m : s->cfg.height_rel_sd;
    return accumulate(s, t1, dt, v, m->stddev_mps, rel, (m->flags & FLOW_F_PHASE) != 0, out);
}
