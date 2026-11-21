#include "front_imu.h"

#include <math.h>
#include <string.h>

#define G_NOMINAL 9.80665f

static void mount_matrix(const float rpy[3], float r[9])
{
    const float cr = cosf(rpy[0]), sr = sinf(rpy[0]);
    const float cp = cosf(rpy[1]), sp = sinf(rpy[1]);
    const float cy = cosf(rpy[2]), sy = sinf(rpy[2]);
    const float m[9] = { cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
                         sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
                         -sp,     cp * sr,                cp * cr };
    memcpy(r, m, sizeof m);
}

static void mul3(const float r[9], const float v[3], float o[3])
{
    for (int i = 0; i < 3; i++) o[i] = r[i * 3] * v[0] + r[i * 3 + 1] * v[1] + r[i * 3 + 2] * v[2];
}

void front_imu_init(front_imu_t *f, const front_imu_config_t *cfg)
{
    memset(f, 0, sizeof *f);
    f->cfg = *cfg;
    mount_matrix(cfg->mount_rpy_rad, f->r_bm);
}

void front_imu_set_clock(front_imu_t *f, flow_nav_clock_fn fn, void *ctx)
{
    f->clock = fn;
    f->clock_ctx = ctx;
}

void front_imu_set_mount(front_imu_t *f, const float rpy[3])
{
    if (memcmp(rpy, f->cfg.mount_rpy_rad, sizeof f->cfg.mount_rpy_rad) == 0) return;
    memcpy(f->cfg.mount_rpy_rad, rpy, sizeof f->cfg.mount_rpy_rad);
    mount_matrix(rpy, f->r_bm);
    f->tilt_have = false;
}

/* Without the clock sync: the smallest arrival - unit time (flow_nav.c). */
static void free_track(front_imu_t *f, uint32_t unit_us, uint64_t t_arr)
{
    const uint32_t off = (uint32_t)t_arr - unit_us;
    if (!f->free_have) {
        f->free_have = true;
        f->free_off = off;
        f->free_leak_t = t_arr;
        return;
    }
    const uint64_t div = 1000000u / FRONT_IMU_LEAK_PPM;
    if (t_arr > f->free_leak_t + div) {
        const uint64_t k = (t_arr - f->free_leak_t) / div;
        f->free_off += (uint32_t)k;
        f->free_leak_t += k * div;
    }
    const int32_t lag = (int32_t)(off - f->free_off);
    if (lag < 0) {
        f->free_off = off;
    } else if ((uint32_t)lag > FRONT_IMU_MAX_LAG_US) {
        f->free_off = off;
        f->free_leak_t = t_arr;
        f->st.resets++;
    }
}

/* Gyro propagation of roll and pitch (Euler rates), then a pull towards
 * the accelerometer's level while it reads about 1 g. */
static void tilt_step(front_imu_t *f, const float w[3], const float a[3], float dt, uint64_t t)
{
    const float an = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    const bool  acc_ok = fabsf(an - G_NOMINAL) < f->cfg.acc_gate * G_NOMINAL;
    /* FRD at rest: a = (g sin(pitch), -g sin(roll) cos(pitch), -g cos(roll) cos(pitch)). */
    const float roll_a = atan2f(-a[1], -a[2]);
    const float pitch_a = atan2f(a[0], sqrtf(a[1] * a[1] + a[2] * a[2]));
    if (!f->tilt_have) {
        if (!acc_ok) return;
        f->roll = roll_a;
        f->pitch = pitch_a;
        f->tilt_have = true;
        f->tilt_t_us = t;
        return;
    }
    const float sr = sinf(f->roll), cr = cosf(f->roll);
    const float tp = tanf(f->pitch);
    f->roll += (w[0] + (w[1] * sr + w[2] * cr) * tp) * dt;
    f->pitch += (w[1] * cr - w[2] * sr) * dt;
    if (acc_ok && f->cfg.tilt_tau_s > 0.0f) {
        const float k = dt / (f->cfg.tilt_tau_s + dt);
        float dr = roll_a - f->roll;
        if (dr > 3.14159265f) dr -= 6.2831853f;
        if (dr < -3.14159265f) dr += 6.2831853f;
        f->roll += k * dr;
        f->pitch += k * (pitch_a - f->pitch);
    }
    f->tilt_t_us = t;
}

unsigned front_imu_batch(front_imu_t *f, const flow_msg_imu_batch_t *b, uint64_t t_arr, front_imu_sample_t *out)
{
    if (b->n == 0 || b->n > FLOW_IMU_BATCH_MAX) {
        f->st.bad++;
        return 0;
    }
    f->st.batches++;
    if (f->have_seq && (uint16_t)(b->seq - f->last_seq) != 1u) f->st.lost += (uint16_t)(b->seq - f->last_seq - 1u);
    f->have_seq = true;
    f->last_seq = b->seq;
    if (b->flags & FLOW_IMU_F_GAP) f->st.unit_gaps++;
    if (b->flags & FLOW_IMU_F_CLAMPED) f->st.clamped++;

    /* The last sample's unit time is what arrived last. */
    const uint32_t last_unit = b->t0_us + b->s[b->n - 1u].dt_us;
    uint64_t       t0;
    uint64_t       t_sync;
    if (f->clock != NULL && f->clock(f->clock_ctx, b->t0_us, t_arr, &t_sync)) {
        t0 = t_sync;
        f->st.synced++;
    } else {
        free_track(f, last_unit, t_arr);
        const uint64_t back = (uint64_t)(uint32_t)(((uint32_t)t_arr - last_unit) - f->free_off) + f->cfg.free_latency_us +
                              b->s[b->n - 1u].dt_us;
        if (t_arr <= back) {
            f->st.bad++;
            return 0;
        }
        t0 = t_arr - back;
    }

    unsigned k = 0;
    for (unsigned i = 0; i < b->n; i++) {
        const uint64_t t = t0 + b->s[i].dt_us;
        if (t <= f->last_t_us) continue; /* overlaps what was handed out */
        front_imu_sample_t *o = &out[k];
        memset(o, 0, sizeof *o);
        o->t_us = t;
        o->dt_s = f->last_t_us != 0 && t - f->last_t_us < 100000u ? (float)(t - f->last_t_us) * 1e-6f : 0.00125f;
        mul3(f->r_bm, b->s[i].gyr, o->gyr);
        mul3(f->r_bm, b->s[i].acc, o->acc);
        tilt_step(f, o->gyr, o->acc, o->dt_s, t);
        f->last_t_us = t;
        k++;
    }
    if (k < b->n) f->st.bad++;
    f->st.samples += k;
    return k;
}

bool front_imu_fresh(const front_imu_t *f, uint64_t now, uint64_t max_age_us)
{
    return f->last_t_us != 0 && (now < f->last_t_us || now - f->last_t_us < max_age_us);
}

bool front_imu_tilt(const front_imu_t *f, uint64_t now, uint64_t max_age_us, float *roll, float *pitch)
{
    if (!f->tilt_have || !(now < f->tilt_t_us || now - f->tilt_t_us < max_age_us)) return false;
    *roll = f->roll;
    *pitch = f->pitch;
    return true;
}
