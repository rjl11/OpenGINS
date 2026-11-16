#include "pole_mount.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "hardware/flash.h"
#include "pico/time.h"

#include "app_config.h"
#include "flash_rec.h"
#include "imu_calib.h"

#define REC_MAGIC   0x4D505347u /* "GSPM" */
#define REC_VERSION 1
/* The two sectors below the IMU calibration's (imu_calib.c uses -4, settings -2). */
#define REC_SECTOR  (PICO_FLASH_SIZE_BYTES - 6u * FLASH_SECTOR_SIZE)
#define R2D         57.29578f

typedef struct {
    pm_stats_t stats;
    float      use_u[3];   /* stored set: unit axis, antenna to tip         */
    uint8_t    use_valid;
    uint8_t    hold;
    uint8_t    pad[2];
} payload_t;

_Static_assert(sizeof(payload_t) <= FLASH_REC_MAX_PAYLOAD, "pole axis must fit in one flash record");

static flash_rec_t s_rec;
static payload_t s_p;
static pm_prior_t s_prior;
static pole_mount_status_t s_st;
static float s_u[3] = { 0.0f, 0.0f, 1.0f }; /* axis in use */
static bool s_dirty;
static uint64_t s_last_save_us;

static void prior_set(float h)
{
    pm_prior_init(&s_prior, h, APP_POLE_MOUNT_TILT_PRIOR_DEG, APP_POLE_MOUNT_LEN_PRIOR_M, APP_POLE_MOUNT_FIX_SD_M,
                  APP_POLE_MOUNT_CORR, APP_POLE_MOUNT_EP_RMS_MAX_M, APP_POLE_MOUNT_EP_RMS_REL);
    s_st.ant_height_m = h;
}

static void solve(void)
{
    pm_solve(&s_p.stats, &s_prior, &s_st.learned);
    s_st.stats = s_p.stats;
    const pm_result_t *r = &s_st.learned;
    float h = s_st.ant_height_m;
    s_st.len_differs = r->ok && s_p.stats.n_ep > 0 && fabsf(r->len_m - h) > 3.0f * r->len_sd_m &&
                       fabsf(r->len_m - h) > 0.02f;
    if (s_p.stats.n_ep == 0) {
        s_st.accept_why = "no episodes yet: stand the pole on a point and turn it";
    } else if (!r->ok) {
        s_st.accept_why = "no solution";
    } else if (r->tilt_sd_deg > APP_POLE_MOUNT_SD_MAX_DEG) {
        s_st.accept_why = "not enough twist yet: turn the pole once on a point";
    } else if (r->tilt_deg > APP_POLE_MOUNT_TILT_MAX_DEG) {
        s_st.accept_why = "tilt over the range limit: check the mounting";
    } else if (fabsf(r->len_m - h) > APP_POLE_MOUNT_LEN_TOL_M) {
        s_st.accept_why = "pole length far from the antenna height setting";
    } else {
        s_st.accept_why = NULL;
    }
    if (s_p.hold) {
        s_st.next_boot = s_p.use_valid ? "stored set (hold)" : "body down axis (hold, nothing stored)";
    } else {
        s_st.next_boot = s_st.accept_why == NULL ? "learned axis" : (s_p.use_valid ? "stored set" : "body down axis");
    }
}

static void use(const float u[3])
{
    for (int i = 0; i < 3; i++) s_u[i] = u[i];
    s_st.applied = !(u[0] == 0.0f && u[1] == 0.0f && u[2] == 1.0f);
    memcpy(s_st.use_u, s_u, sizeof s_st.use_u);
    s_st.use_tilt_deg = atan2f(hypotf(u[0], u[1]), u[2]) * R2D;
    s_st.use_azim_deg = atan2f(u[1], u[0]) * R2D;
}

static void store(const float u[3])
{
    memcpy(s_p.use_u, u, sizeof s_p.use_u);
    s_p.use_valid = 1;
}

void pole_mount_init(bool active, float h_m)
{
    memset(&s_st, 0, sizeof s_st);
    s_st.active = active;
    s_st.last_rms_m = -1.0f;
    s_st.last_save_rc = 1;
    prior_set(h_m);
    float nominal[3];
    pm_axis_default(nominal);
    use(nominal);
    flash_rec_init(&s_rec, REC_SECTOR, REC_MAGIC, REC_VERSION, (uint16_t)sizeof(payload_t));
    if (!flash_rec_load(&s_rec, &s_p)) {
        memset(&s_p, 0, sizeof s_p);
        s_st.note = "no record: body down axis";
    }
    s_st.rec_seq = flash_rec_seq(&s_rec);
    solve();
    if (!active) {
        if (s_st.note == NULL) s_st.note = "not on a pole with an antenna height: body down axis";
        s_st.hold = s_p.hold;
        s_st.stored = s_p.use_valid;
        return;
    }
    if (!s_p.hold && s_st.accept_why == NULL) {
        use(s_st.learned.u);
        if (!s_p.use_valid || memcmp(s_p.use_u, s_st.learned.u, sizeof s_p.use_u) != 0) {
            store(s_st.learned.u);
            s_dirty = true;
        }
        s_st.note = "learned axis applied";
    } else if (s_p.use_valid) {
        use(s_p.use_u);
        s_st.note = s_p.hold ? "stored set (hold)" : "stored set (learned axis not usable yet)";
    } else if (s_st.note == NULL) {
        s_st.note = s_p.hold ? "body down axis (hold)" : "body down axis (nothing learned yet)";
    }
    s_st.hold = s_p.hold;
    s_st.stored = s_p.use_valid;
    solve();
}

void pole_mount_set_height(float h_m)
{
    prior_set(h_m);
    solve();
}

void pole_mount_axis(float u[3])
{
    for (int i = 0; i < 3; i++) u[i] = s_st.active ? s_u[i] : (i == 2 ? 1.0f : 0.0f);
}

void pole_mount_add(const pole_episode_t *e)
{
    if (!s_st.active) return;
    float rms;
    bool ok = pm_add_episode(&s_p.stats, e, &s_prior, &rms);
    s_st.last_rms_m = rms;
    s_st.last_twist_deg = e->twist_deg;
    s_st.last_added = ok;
    if (ok) {
        s_st.ep_boot++;
        s_dirty = true;
    } else {
        s_st.ep_rejected++;
    }
    solve();
}

static int save(void)
{
    int rc = flash_rec_save(&s_rec, &s_p);
    s_st.last_save_rc = rc;
    if (rc == 0) {
        s_dirty = false;
        s_st.rec_seq = flash_rec_seq(&s_rec);
    }
    return rc;
}

void pole_mount_poll(uint64_t now_us)
{
    if (!s_dirty || now_us - s_last_save_us < (uint64_t)APP_POLE_MOUNT_SAVE_S * 1000000u) return;
    s_last_save_us = now_us;
    save();
}

int pole_mount_action(int op, char *msg, size_t cap)
{
    if (!s_st.active) {
        snprintf(msg, cap, "refused: pole axis: not on a pole with an antenna height");
        return -1;
    }
    float before[3];
    memcpy(before, s_u, sizeof before);
    const char *what = "";
    switch (op) {
    case IMU_CALIB_OP_ACCEPT:
        if (s_st.accept_why != NULL) {
            snprintf(msg, cap, "refused: pole axis: %s", s_st.accept_why);
            return -1;
        }
        use(s_st.learned.u);
        store(s_st.learned.u);
        what = "learned axis in use and stored";
        break;
    case IMU_CALIB_OP_HOLD:
        s_p.hold = 1;
        what = "hold: boot keeps the stored set";
        break;
    case IMU_CALIB_OP_AUTO:
        s_p.hold = 0;
        what = "auto: boot applies the learned axis";
        break;
    case IMU_CALIB_OP_CLEAR:
        memset(&s_p.stats, 0, sizeof s_p.stats);
        what = "episodes cleared; the axis in use stays";
        break;
    case IMU_CALIB_OP_FACTORY: {
        memset(&s_p, 0, sizeof s_p);
        float nominal[3];
        pm_axis_default(nominal);
        use(nominal);
        what = "factory: episodes and stored set erased, body down axis";
        break;
    }
    default:
        snprintf(msg, cap, "refused: pole axis: unknown operation");
        return -1;
    }
    s_st.hold = s_p.hold;
    s_st.stored = s_p.use_valid;
    s_st.note = what;
    solve();
    int rc = save();
    snprintf(msg, cap, "pole axis: %s%s", what, rc == 0 ? "" : " (flash save failed)");
    return memcmp(before, s_u, sizeof before) != 0 ? 1 : 0;
}

void pole_mount_status(pole_mount_status_t *out) { *out = s_st; }
