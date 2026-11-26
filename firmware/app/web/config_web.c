#include "config_web.h"

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico/platform/sections.h"

#include "app_config.h"
#include "imu_calib.h"
#include "pole_mount.h"
#include "rtcm_src.h"
#include "jsonw.h"
#include "web.h"

/* Shared with the POST handler (lwIP context); touched by the main loop only
 * inside web_locked(). */
static settings_t s_base;          /* settings in use, the start of a save */
static config_cmd_t s_pending;     /* kind NONE = free */
static uint32_t s_pending_rid;     /* its request ID, 0 = none */
/* Taken by the main loop and not yet reported by config_web_publish(): no
 * new command is accepted meanwhile, so a save never starts from settings
 * that the one before it is still changing (s_base is set at that publish). */
static bool s_running;
static uint32_t s_running_rid;

/* The commands accepted lately, by request ID (rid, common.js) and a hash of
 * the rest of the form: a page that got no answer sends the same form with
 * the same ID again, and that one is answered as a repeat instead of being
 * queued twice. Kept in RAM that a reboot leaves alone (as reset_reason.c's
 * marker), so a reboot command retried after the unit is back is not run
 * again. ran: its result was reported; one accepted but not run when the
 * unit reset is forgotten at boot, so its retry runs it. result_seq is this
 * boot's last_result seq of its outcome, 0 = not yet or from before the
 * reboot. */
#define RID_KEEP 8
#define RID_MAGIC 0x52494432u
typedef struct {
    uint32_t rid, hash, result_seq, ran;
} rid_rec_t;
typedef struct {
    uint32_t magic;
    uint32_t next;
    rid_rec_t r[RID_KEEP];
    uint32_t check;
} rid_log_t;
static rid_log_t __uninitialized_ram(s_rids);

static uint32_t rid_check(void)
{
    const uint32_t *w = (const uint32_t *)&s_rids;
    uint32_t c = 0x9E3779B9u;
    for (size_t i = 0; i < offsetof(rid_log_t, check) / 4; i++) c = (c ^ w[i]) * 16777619u;
    return c;
}

static void rid_init(void)
{
    if (s_rids.magic != RID_MAGIC || s_rids.check != rid_check() || s_rids.next >= RID_KEEP) {
        memset(&s_rids, 0, sizeof s_rids);
        s_rids.magic = RID_MAGIC;
    }
    for (int i = 0; i < RID_KEEP; i++) {
        if (!s_rids.r[i].ran) s_rids.r[i].rid = 0; /* accepted, never run */
        s_rids.r[i].result_seq = 0;                /* last boot's seqs */
    }
    s_rids.check = rid_check();
}

static rid_rec_t *rid_find(uint32_t rid, uint32_t hash)
{
    if (rid == 0) return NULL;
    for (int k = 1; k <= RID_KEEP; k++) { /* newest first */
        rid_rec_t *r = &s_rids.r[(s_rids.next + RID_KEEP - k) % RID_KEEP];
        if (r->rid == rid && r->hash == hash) return r;
    }
    return NULL;
}

static void rid_add(uint32_t rid, uint32_t hash)
{
    if (rid == 0) return;
    s_rids.r[s_rids.next] = (rid_rec_t){ rid, hash, 0, 0 };
    s_rids.next = (s_rids.next + 1) % RID_KEEP;
    s_rids.check = rid_check();
}

/* FNV-1a of one decoded field "key=value"; the form's hash is the sum of
 * its fields' (rid left out), so the same fields in another order match. */
static uint32_t fnv(uint32_t h, const char *s)
{
    for (; *s; s++) h = (h ^ (uint8_t)*s) * 16777619u;
    return h;
}

static uint32_t field_hash(const char *k, const char *v)
{
    uint32_t h = fnv(fnv(fnv(2166136261u, k), "="), v);
    h ^= h >> 15; /* spread before summing */
    return h * 0x2C1B3C6Du;
}

static uint32_t s_result_seq;      /* written in web_locked() only */
static uint32_t s_result_rid;      /* the request ID of that result, 0 = none */
static char s_result[160] = "none";

/* ---- form parsing (lwIP context) ------------------------------------------ */

static int hexval(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

/* Decodes %XX and '+' in place over [s, s+n); returns the new length. */
static size_t url_decode(char *s, size_t n)
{
    size_t o = 0;
    for (size_t i = 0; i < n; i++) {
        if (s[i] == '+') {
            s[o++] = ' ';
        } else if (s[i] == '%' && i + 2 < n && hexval(s[i + 1]) >= 0 && hexval(s[i + 2]) >= 0) {
            s[o++] = (char)(hexval(s[i + 1]) * 16 + hexval(s[i + 2]));
            i += 2;
        } else {
            s[o++] = s[i];
        }
    }
    s[o] = '\0';
    return o;
}

typedef struct {
    char *buf;
    size_t len, pos;
} form_t;

/* Next key=value pair, both decoded and NUL-terminated in place. */
static bool form_next(form_t *f, char **key, char **val)
{
    if (f->pos >= f->len) {
        return false;
    }
    char *start = f->buf + f->pos;
    char *end = memchr(start, '&', f->len - f->pos);
    size_t n = end ? (size_t)(end - start) : f->len - f->pos;
    f->pos += n + 1;
    start[n] = '\0';
    char *eq = memchr(start, '=', n);
    if (eq == NULL) {
        *key = start;
        url_decode(start, n);
        *val = start + strlen(start); /* "" */
        return true;
    }
    *eq = '\0';
    url_decode(start, (size_t)(eq - start));
    url_decode(eq + 1, n - (size_t)(eq - start) - 1);
    *key = start;
    *val = eq + 1;
    return true;
}

static bool parse_float(const char *v, float *out)
{
    char *end;
    float x = strtof(v, &end);
    if (end == v) {
        return false;
    }
    while (*end == ' ') {
        end++;
    }
    if (*end != '\0') {
        return false;
    }
    *out = x;
    return true;
}

typedef struct {
    const char *key;
    size_t offset; /* into settings_t */
} float_field_t;

#define FF(k, m) { k, offsetof(settings_t, m) }
static const float_field_t k_floats[] = {
    FF("lever_x", leverarm_m[0]),
    FF("lever_y", leverarm_m[1]),
    FF("lever_z", leverarm_m[2]),
    FF("acc_psd", acc_psd),
    FF("gyr_psd", gyr_psd),
    FF("gyr_bias_rw", gyr_bias_rw),
    FF("acc_bias_rw", acc_bias_rw),
    FF("automotive_min_speed", automotive_min_speed),
    FF("max_deadreckoning_s", max_deadreckoning_s),
    FF("gnss_pos_decim", gnss_pos_decim),
    FF("offset_interval_s", offset_interval_s),
    FF("offset_inflation", offset_inflation),
    FF("offset_rw_mps", offset_rw_mps),
    FF("offset_baro_sd_m", offset_baro_sd_m),
    FF("baseline_yaw_deg", baseline_yaw_deg),
    FF("baseline_gate_m", baseline_gate_m),
    FF("flow_height_m", flow_height_m),
    FF("flow_cam_yaw_deg", flow_cam_yaw_deg),
    FF("flow_lever_x", flow_leverarm_m[0]),
    FF("flow_lever_y", flow_leverarm_m[1]),
    FF("flow_lever_z", flow_leverarm_m[2]),
    FF("front_imu_roll_deg", front_imu_rpy_deg[0]),
    FF("front_imu_pitch_deg", front_imu_rpy_deg[1]),
    FF("front_imu_yaw_deg", front_imu_rpy_deg[2]),
    FF("front_imu_lever_x", front_imu_lever_m[0]),
    FF("front_imu_lever_y", front_imu_lever_m[1]),
    FF("front_imu_lever_z", front_imu_lever_m[2]),
    FF("cab_roll_centre_m", cab_roll_centre_m),
    FF("ant_height_m", ant_height_m),
};

/* Sensors fitted: "on" / "off", uint8_t in settings_t. */
static const float_field_t k_onoff[] = {
    FF("sensor_baro", sensor_baro),
    FF("sensor_mag", sensor_mag),
    FF("sensor_flow", sensor_flow),
    FF("sensor_wheel", sensor_wheel),
    FF("eth_fitted", eth_fitted),
    FF("front_imu_backup", front_imu_backup),
};

/* IPv4 addresses as dotted quads, uint8_t[4] in settings_t. */
static const float_field_t k_ipv4[] = {
    FF("eth_ip", eth_ip),
    FF("eth_mask", eth_mask),
    FF("eth_gw", eth_gw),
};

static bool parse_ipv4(const char *v, uint8_t out[4])
{
    uint8_t b[4];
    for (int i = 0; i < 4; i++) {
        char *end;
        if (*v < '0' || *v > '9') {
            return false;
        }
        unsigned long x = strtoul(v, &end, 10);
        if (x > 255 || end - v > 3 || (i < 3 && *end != '.') || (i == 3 && *end != '\0')) {
            return false;
        }
        b[i] = (uint8_t)x;
        v = end + 1;
    }
    memcpy(out, b, 4);
    return true;
}

/* Applies one form field to s. NULL, or an error message in err. */
static const char *apply_field(settings_t *s, const char *key, const char *val, char *err,
                               size_t cap)
{
    for (size_t i = 0; i < sizeof k_floats / sizeof k_floats[0]; i++) {
        if (strcmp(key, k_floats[i].key) == 0) {
            float *dst = (float *)((char *)s + k_floats[i].offset);
            if (!parse_float(val, dst)) {
                snprintf(err, cap, "%s: not a number", key);
                return err;
            }
            return NULL;
        }
    }
    for (size_t i = 0; i < sizeof k_onoff / sizeof k_onoff[0]; i++) {
        if (strcmp(key, k_onoff[i].key) == 0) {
            uint8_t *dst = (uint8_t *)s + k_onoff[i].offset;
            if (strcmp(val, "on") == 0) *dst = 1;
            else if (strcmp(val, "off") == 0) *dst = 0;
            else { snprintf(err, cap, "%s: on or off", key); return err; }
            return NULL;
        }
    }
    for (size_t i = 0; i < sizeof k_ipv4 / sizeof k_ipv4[0]; i++) {
        if (strcmp(key, k_ipv4[i].key) == 0) {
            if (!parse_ipv4(val, (uint8_t *)s + k_ipv4[i].offset)) {
                snprintf(err, cap, "%s: an IPv4 address like 192.168.5.124", key);
                return err;
            }
            return NULL;
        }
    }
    if (strcmp(key, "rtcm_src") == 0) {
        for (unsigned m = RTCM_SRC_MODE_AUTO; m <= RTCM_SRC_MODE_BT; m++) {
            if (strcmp(val, rtcm_src_mode_name(m)) == 0) {
                s->rtcm_src = (uint8_t)m;
                return NULL;
            }
        }
        snprintf(err, cap, "rtcm_src: auto, ethernet or bluetooth");
        return err;
    }
    if (strcmp(key, "antenna") == 0) {
        if (strcmp(val, "single") == 0) s->antenna_dual = 0;
        else if (strcmp(val, "dual") == 0) s->antenna_dual = 1;
        else { snprintf(err, cap, "antenna: single or dual"); return err; }
        return NULL;
    }
    if (strcmp(key, "platform") == 0) {
        unsigned p;
        for (p = 0; p <= APP_PLATFORM_FIXED_WING; p++) {
            if (strcmp(val, settings_platform_name(p)) == 0) {
                s->platform_p1 = (uint8_t)(p + 1);
                return NULL;
            }
        }
        snprintf(err, cap, "platform: tractor, pole, multirotor or fixed_wing");
        return err;
    }
    if (strcmp(key, "height_src") == 0) {
        for (unsigned h = SETTINGS_HEIGHT_AUTO; h <= SETTINGS_HEIGHT_GNSS; h++) {
            if (strcmp(val, settings_height_src_name(h)) == 0) {
                s->height_src = (uint8_t)h;
                return NULL;
            }
        }
        snprintf(err, cap, "height_src: auto, baro or gnss");
        return err;
    }
    if (strcmp(key, "wifi_mode") == 0) {
        if (strcmp(val, "auto") == 0) s->wifi_mode = APP_WIFI_AUTO;
        else if (strcmp(val, "sta") == 0) s->wifi_mode = APP_WIFI_STA;
        else if (strcmp(val, "ap") == 0) s->wifi_mode = APP_WIFI_AP;
        else { snprintf(err, cap, "wifi_mode: auto, sta or ap"); return err; }
        return NULL;
    }
    if (strcmp(key, "sta_ssid") == 0 || strcmp(key, "sta_password") == 0) {
        bool ssid = key[4] == 's';
        char *dst = ssid ? s->sta_ssid : s->sta_password;
        size_t max = ssid ? SETTINGS_SSID_MAX : SETTINGS_PW_MAX;
        if (strlen(val) > max) {
            snprintf(err, cap, "%s: at most %u characters", key, (unsigned)max);
            return err;
        }
        snprintf(dst, max + 1, "%s", val);
        return NULL;
    }
    if (strcmp(key, "action") == 0 || strcmp(key, "rid") == 0) {
        return NULL;
    }
    snprintf(err, cap, "unknown field: %.40s", key);
    return err;
}

static size_t reply(char *out, size_t cap, bool ok, const char *msg)
{
    char buf[200];
    jw_t w;
    jw_init(&w, buf, sizeof buf);
    jw_obj(&w, NULL);
    jw_bool(&w, "ok", ok);
    jw_str(&w, ok ? "queued" : "error", msg);
    jw_end(&w, '}');
    size_t n = w.len < cap ? w.len : cap;
    memcpy(out, buf, n);
    return n;
}

static void post_handler(const char *uri, const char *body, size_t len, char *out, size_t cap,
                         size_t *out_len)
{
    if (strcmp(uri, "/config") != 0) {
        *out_len = reply(out, cap, false, "unknown URI");
        return;
    }
    static char buf[WEB_POST_MAX + 1];
    memcpy(buf, body, len);
    buf[len] = '\0';

    /* First pass: the action. */
    config_cmd_kind_t kind = CFG_CMD_NONE;
    const char *action = "";
    uint32_t rid = 0;
    form_t f = { buf, len, 0 };
    char *k, *v;
    static char copy[WEB_POST_MAX + 1];
    memcpy(copy, buf, len + 1);
    form_t f2 = { copy, len, 0 };
    uint32_t hash = 0;
    while (form_next(&f2, &k, &v)) {
        if (strcmp(k, "rid") == 0) {
            rid = (uint32_t)strtoul(v, NULL, 10);
            continue;
        }
        if (strcmp(k, "action") == 0) {
            action = v;
        }
        hash += field_hash(k, v);
    }
    if (strcmp(action, "save") == 0) kind = CFG_CMD_SAVE;
    else if (strcmp(action, "defaults") == 0) kind = CFG_CMD_DEFAULTS;
    else if (strcmp(action, "reboot") == 0) kind = CFG_CMD_REBOOT;
    else if (strcmp(action, "reconfigure_gnss") == 0) kind = CFG_CMD_RECONFIGURE_GNSS;
    else if (strcmp(action, "reconfigure_imu") == 0) kind = CFG_CMD_RECONFIGURE_IMU;
    else if (strcmp(action, "calib_reset") == 0) kind = CFG_CMD_CALIB_RESET;
    else if (strcmp(action, "calib") == 0) kind = CFG_CMD_CALIB;
    else {
        *out_len = reply(out, cap, false, "action must be save, defaults, reboot, "
                                          "reconfigure_gnss, reconfigure_imu, calib_reset or calib");
        return;
    }
    /* A repeat of a command accepted lately (same ID and form): not queued
     * again. "repeat" says whether it is still to run ("pending") or done,
     * with its last_result seq when that was in this boot (common.js). */
    const rid_rec_t *seen = rid_find(rid, hash);
    if (seen != NULL) {
        const bool pending = (s_pending.kind != CFG_CMD_NONE && s_pending_rid == rid) || (s_running && s_running_rid == rid);
        char rb[160];
        jw_t w;
        jw_init(&w, rb, sizeof rb);
        jw_obj(&w, NULL);
        jw_bool(&w, "ok", true);
        jw_str(&w, "queued", action);
        jw_str(&w, "repeat", pending ? "pending" : "done");
        if (!pending && seen->result_seq != 0) jw_int(&w, "result_seq", seen->result_seq);
        jw_end(&w, '}');
        size_t n = w.len < cap ? w.len : cap;
        memcpy(out, rb, n);
        *out_len = n;
        return;
    }
    if (s_pending.kind != CFG_CMD_NONE || s_running) {
        *out_len = reply(out, cap, false, "busy: the previous command is still running");
        return;
    }
    /* The open access point has no password, so anyone in range can reach
     * this page. Over it, only saving settings and the hold / auto calibration
     * choices are accepted; the rest (erasing, rebooting, re-provisioning the
     * receiver or IMU, learned-calibration changes) needs the station network. */
    web_info_t wi;
    web_get_info(&wi);
    const bool on_open_ap = wi.state == WEB_WIFI_AP;
    if (on_open_ap && kind != CFG_CMD_SAVE && kind != CFG_CMD_CALIB) {
        *out_len = reply(out, cap, false, "refused over the open access point: use the station network");
        return;
    }

    static config_cmd_t cmd;
    memset(&cmd, 0, sizeof cmd);
    cmd.kind = kind;
    if (kind == CFG_CMD_SAVE) {
        char err[96];
        cmd.settings = s_base;
        while (form_next(&f, &k, &v)) {
            if (apply_field(&cmd.settings, k, v, err, sizeof err) != NULL) {
                *out_len = reply(out, cap, false, err);
                return;
            }
        }
        const char *bad = settings_validate(&cmd.settings);
        if (bad != NULL) {
            *out_len = reply(out, cap, false, bad);
            return;
        }
    }
    if (kind == CFG_CMD_CALIB) {
        cmd.calib_group = -1;
        cmd.calib_op = -1;
        while (form_next(&f, &k, &v)) {
            if (strcmp(k, "group") == 0) {
                cmd.calib_group = strcmp(v, "pole") == 0 ? POLE_MOUNT_GROUP : imu_calib_group_from_name(v);
            } else if (strcmp(k, "op") == 0) {
                cmd.calib_op = imu_calib_op_from_name(v);
            }
        }
        if (cmd.calib_group < 0 || cmd.calib_op < 0 ||
            (cmd.calib_group == IMU_CALIB_GROUPS && cmd.calib_op != IMU_CALIB_OP_FACTORY)) {
            *out_len = reply(out, cap, false, "calib needs group=gyr|acc|mag|gyr_m|pole (all: factory only) "
                                              "and op=accept|hold|auto|clear|factory");
            return;
        }
        if (on_open_ap && cmd.calib_op != IMU_CALIB_OP_HOLD && cmd.calib_op != IMU_CALIB_OP_AUTO) {
            *out_len = reply(out, cap, false, "refused over the open access point: use the station network");
            return;
        }
    }
    s_pending = cmd;
    s_pending_rid = rid;
    rid_add(rid, hash);
    *out_len = reply(out, cap, true, action);
}

void config_web_init(void)
{
    rid_init();
    web_set_post_handler(post_handler);
}

/* ---- main loop -------------------------------------------------------------- */

static void take_locked(void *arg)
{
    config_cmd_t *cmd = arg;
    *cmd = s_pending;
    if (s_pending.kind != CFG_CMD_NONE) {
        s_running = true;
        s_running_rid = s_pending_rid;
    }
    s_pending.kind = CFG_CMD_NONE;
}

bool config_web_take(config_cmd_t *cmd)
{
    if (s_pending.kind == CFG_CMD_NONE) { /* cheap check without the lock */
        return false;
    }
    web_locked(take_locked, cmd);
    return cmd->kind != CFG_CMD_NONE;
}

typedef struct {
    const settings_t *cur;
    bool result;
} publish_arg_t;

/* The settings in use; with a result, the running command is done: its
 * request ID gets the result's seq, and the next command may be accepted. */
static void publish_locked(void *arg)
{
    const publish_arg_t *a = arg;
    s_base = *a->cur;
    if (!a->result) return;
    s_result_seq++;
    if (s_running) {
        for (int k = 1; k <= RID_KEEP; k++) { /* the newest with that ID */
            rid_rec_t *r = &s_rids.r[(s_rids.next + RID_KEEP - k) % RID_KEEP];
            if (s_running_rid != 0 && r->rid == s_running_rid) {
                r->result_seq = s_result_seq;
                r->ran = 1;
                s_rids.check = rid_check();
                break;
            }
        }
        s_result_rid = s_running_rid;
        s_running = false;
    } else {
        s_result_rid = 0;
    }
}

static const char *wifi_mode_name(uint8_t m)
{
    return m == APP_WIFI_AP ? "ap" : m == APP_WIFI_STA ? "sta" : "auto";
}

static void settings_obj(jw_t *w, const char *key, const settings_t *s)
{
    jw_obj(w, key);
    jw_num(w, "lever_x", (double)s->leverarm_m[0], 3);
    jw_num(w, "lever_y", (double)s->leverarm_m[1], 3);
    jw_num(w, "lever_z", (double)s->leverarm_m[2], 3);
    char t[24];
    /* %g keeps the small noise figures readable; jw_num would round them. */
    static const char *const k_noise[] = { "acc_psd", "gyr_psd", "gyr_bias_rw", "acc_bias_rw" };
    const float v[] = { s->acc_psd, s->gyr_psd, s->gyr_bias_rw, s->acc_bias_rw };
    for (int i = 0; i < 4; i++) {
        snprintf(t, sizeof t, "%g", (double)v[i]);
        jw_str(w, k_noise[i], t);
    }
    jw_num(w, "automotive_min_speed", (double)s->automotive_min_speed, 2);
    jw_num(w, "max_deadreckoning_s", (double)s->max_deadreckoning_s, 1);
    jw_num(w, "gnss_pos_decim", (double)s->gnss_pos_decim, 0);
    jw_num(w, "offset_interval_s", (double)s->offset_interval_s, 2);
    jw_num(w, "offset_inflation", (double)s->offset_inflation, 2);
    jw_num(w, "offset_rw_mps", (double)s->offset_rw_mps, 3);
    jw_num(w, "offset_baro_sd_m", (double)s->offset_baro_sd_m, 3);
    jw_num(w, "baseline_yaw_deg", (double)s->baseline_yaw_deg, 2);
    jw_num(w, "baseline_gate_m", (double)s->baseline_gate_m, 3);
    jw_num(w, "flow_height_m", (double)s->flow_height_m, 3);
    jw_num(w, "flow_cam_yaw_deg", (double)s->flow_cam_yaw_deg, 1);
    jw_num(w, "flow_lever_x", (double)s->flow_leverarm_m[0], 3);
    jw_num(w, "flow_lever_y", (double)s->flow_leverarm_m[1], 3);
    jw_num(w, "flow_lever_z", (double)s->flow_leverarm_m[2], 3);
    jw_num(w, "front_imu_roll_deg", (double)s->front_imu_rpy_deg[0], 2);
    jw_num(w, "front_imu_pitch_deg", (double)s->front_imu_rpy_deg[1], 2);
    jw_num(w, "front_imu_yaw_deg", (double)s->front_imu_rpy_deg[2], 2);
    jw_num(w, "front_imu_lever_x", (double)s->front_imu_lever_m[0], 3);
    jw_num(w, "front_imu_lever_y", (double)s->front_imu_lever_m[1], 3);
    jw_num(w, "front_imu_lever_z", (double)s->front_imu_lever_m[2], 3);
    jw_str(w, "front_imu_backup", s->front_imu_backup ? "on" : "off");
    jw_num(w, "cab_roll_centre_m", (double)s->cab_roll_centre_m, 3);
    jw_str(w, "sensor_baro", s->sensor_baro ? "on" : "off");
    jw_str(w, "sensor_mag", s->sensor_mag ? "on" : "off");
    jw_str(w, "sensor_flow", s->sensor_flow ? "on" : "off");
    jw_str(w, "sensor_wheel", s->sensor_wheel ? "on" : "off");
    jw_str(w, "eth_fitted", s->eth_fitted ? "on" : "off");
    const char *rs = rtcm_src_mode_name(s->rtcm_src);
    jw_str(w, "rtcm_src", rs ? rs : "auto");
    static const char *const k_ip_keys[] = { "eth_ip", "eth_mask", "eth_gw" };
    const uint8_t *const ips[] = { s->eth_ip, s->eth_mask, s->eth_gw };
    for (int i = 0; i < 3; i++) {
        snprintf(t, sizeof t, "%u.%u.%u.%u", ips[i][0], ips[i][1], ips[i][2], ips[i][3]);
        jw_str(w, k_ip_keys[i], t);
    }
    const char *hs = settings_height_src_name(s->height_src);
    jw_str(w, "height_src", hs ? hs : "auto");
    jw_str(w, "antenna", s->antenna_dual ? "dual" : "single");
    jw_str(w, "platform", settings_platform_name(settings_platform(s)));
    jw_num(w, "ant_height_m", (double)s->ant_height_m, 3);
    jw_str(w, "wifi_mode", wifi_mode_name(s->wifi_mode));
    jw_str(w, "sta_ssid", s->sta_ssid);
    jw_bool(w, "sta_password_set", s->sta_password[0] != '\0');
    jw_end(w, '}');
}

void config_web_publish(const settings_t *cur, const settings_t *boot, bool from_flash,
                        const char *result)
{
    static char buf[WEB_DOC_MAX];
    static settings_t defaults;
    jw_t w;

    if (result != NULL) {
        snprintf(s_result, sizeof s_result, "%s", result);
    }
    publish_arg_t pa = { cur, result != NULL };
    web_locked(publish_locked, &pa);
    settings_defaults(&defaults);

    jw_init(&w, buf, sizeof buf);
    jw_obj(&w, NULL);
    jw_str(&w, "source", from_flash ? "flash" : "defaults");
    jw_int(&w, "seq", settings_seq());
    settings_obj(&w, "settings", cur);
    settings_obj(&w, "defaults", &defaults);
    /* The limits settings_validate() checks, so the page marks a bad value
     * before posting. */
    jw_obj(&w, "range");
    for (unsigned i = 0; i < settings_n_ranges; i++) {
        jw_arr(&w, settings_ranges[i].key);
        jw_numg(&w, NULL, (double)settings_ranges[i].lo);
        jw_numg(&w, NULL, (double)settings_ranges[i].hi);
        jw_end(&w, ']');
    }
    jw_end(&w, '}');
    jw_arr(&w, "reboot_pending");
    if (cur->antenna_dual != boot->antenna_dual) jw_str(&w, NULL, "antenna");
    if (settings_platform(cur) != settings_platform(boot)) jw_str(&w, NULL, "platform");
    if (cur->wifi_mode != boot->wifi_mode) jw_str(&w, NULL, "wifi_mode");
    if (strcmp(cur->sta_ssid, boot->sta_ssid) != 0) jw_str(&w, NULL, "sta_ssid");
    if (strcmp(cur->sta_password, boot->sta_password) != 0) jw_str(&w, NULL, "sta_password");
    jw_end(&w, ']');
    jw_obj(&w, "last_result");
    jw_int(&w, "seq", s_result_seq);
    jw_int(&w, "rid", s_result_rid);
    jw_str(&w, "text", s_result);
    jw_end(&w, '}');
    jw_end(&w, '}');
    if (jw_ok(&w)) {
        web_set_doc(WEB_DOC_CONFIG, buf, w.len);
    }
}
