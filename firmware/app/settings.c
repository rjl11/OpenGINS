#include "settings.h"

#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "hardware/flash.h"

#include "app_config.h"
#include "flash_rec.h"
#include "rtcm_src.h"

#define REC_MAGIC       0x46435347u /* "GSCF" */
/* The last two 4 KB sectors of the flash. */
#define SECTOR_A        (PICO_FLASH_SIZE_BYTES - 2u * FLASH_SECTOR_SIZE)

_Static_assert(sizeof(settings_t) <= FLASH_REC_MAX_PAYLOAD, "settings must fit in one flash record");
/* platform_p1 went into the padding byte at the end of version 1: records
 * from before it have the same size. Version 2 appends the flow geometry,
 * so a version 1 record is an exact prefix of it. */
_Static_assert(offsetof(settings_t, flow_height_m) == SETTINGS_V1_SIZE, "version 1 must be a prefix");
_Static_assert(offsetof(settings_t, sensor_baro) == SETTINGS_V2_SIZE, "version 2 must be a prefix");
_Static_assert(offsetof(settings_t, max_deadreckoning_s) == SETTINGS_V3_SIZE, "version 3 must be a prefix");
_Static_assert(offsetof(settings_t, ant_height_m) == SETTINGS_V4_SIZE, "version 4 must be a prefix");
_Static_assert(offsetof(settings_t, height_src) == SETTINGS_V5_SIZE, "version 5 must be a prefix");
_Static_assert(offsetof(settings_t, offset_interval_s) == SETTINGS_V6_SIZE, "version 6 must be a prefix");
_Static_assert(offsetof(settings_t, gnss_pos_decim) == SETTINGS_V7_SIZE, "version 7 must be a prefix");
_Static_assert(offsetof(settings_t, offset_baro_sd_m) == SETTINGS_V8_SIZE, "version 8 must be a prefix");
_Static_assert(offsetof(settings_t, eth_fitted) == SETTINGS_V9_SIZE, "version 9 must be a prefix");
_Static_assert(sizeof(settings_t) == 212,"settings_t size changed: bump SETTINGS_VERSION");

static flash_rec_t s_rec;
static unsigned s_loaded_version; /* of the record settings_load() read, 0 = none */

unsigned settings_loaded_version(void) { return s_loaded_version; }

static void rec_init(void)
{
    if (s_rec.magic == 0) {
        flash_rec_init(&s_rec, SECTOR_A, REC_MAGIC, SETTINGS_VERSION, (uint16_t)sizeof(settings_t));
    }
}

void settings_defaults(settings_t *s)
{
    memset(s, 0, sizeof *s);
    s->leverarm_m[0] = APP_GNSS_LEVERARM_X_M;
    s->leverarm_m[1] = APP_GNSS_LEVERARM_Y_M;
    s->leverarm_m[2] = APP_GNSS_LEVERARM_Z_M;
    s->acc_psd = APP_ACC_PSD;
    s->gyr_psd = APP_GYR_PSD;
    s->gyr_bias_rw = APP_GYR_BIAS_RW;
    s->acc_bias_rw = APP_ACC_BIAS_RW;
    s->automotive_min_speed = APP_AUTOMOTIVE_MIN_SPEED;
    s->baseline_yaw_deg = APP_BASELINE_YAW_DEG;
    s->baseline_gate_m = APP_BASELINE_GATE_M;
    s->antenna_dual = APP_GNSS_ANTENNA_MODE == APP_ANTENNA_DUAL;
    s->wifi_mode = APP_WIFI_MODE;
    snprintf(s->sta_ssid, sizeof s->sta_ssid, "%s", APP_WIFI_STA_SSID);
    snprintf(s->sta_password, sizeof s->sta_password, "%s", APP_WIFI_STA_PASSWORD);
    s->platform_p1 = (uint8_t)(APP_PLATFORM + 1);
    s->flow_height_m = APP_FLOW_HEIGHT_M;
    s->flow_cam_yaw_deg = APP_FLOW_CAM_YAW_DEG;
    s->flow_leverarm_m[0] = APP_FLOW_LEVERARM_X_M;
    s->flow_leverarm_m[1] = APP_FLOW_LEVERARM_Y_M;
    s->flow_leverarm_m[2] = APP_FLOW_LEVERARM_Z_M;
    s->sensor_baro = APP_SENSOR_BARO;
    s->sensor_mag = APP_FEED_MAG;
    s->sensor_flow = APP_SENSOR_FLOW;
    s->sensor_wheel = APP_SENSOR_WHEEL;
    s->max_deadreckoning_s = APP_MAX_DEADRECKONING_SEC;
    s->ant_height_m = APP_ANT_HEIGHT_M;
    s->height_src = SETTINGS_HEIGHT_AUTO;
    s->offset_interval_s = APP_HEIGHT_OFFSET_INTERVAL_S;
    s->offset_inflation = APP_HEIGHT_OFFSET_INFLATION;
    s->offset_rw_mps = APP_HEIGHT_OFFSET_RW_MPS;
    s->gnss_pos_decim = (float)APP_GNSS_POS_DECIMATION;
    s->offset_baro_sd_m = APP_HEIGHT_OFFSET_BARO_SD_M;
    static const uint8_t k_ip[4] = APP_ETH_IP, k_mask[4] = APP_ETH_MASK, k_gw[4] = APP_ETH_GW;
    s->eth_fitted = APP_ETH_FITTED;
    memcpy(s->eth_ip, k_ip, 4);
    memcpy(s->eth_mask, k_mask, 4);
    memcpy(s->eth_gw, k_gw, 4);
    s->rtcm_src = RTCM_SRC_MODE_AUTO;
}

bool settings_ins_gnss_height(const settings_t *s, unsigned platform)
{
    switch (s->height_src) {
    case SETTINGS_HEIGHT_BARO: return false;
    case SETTINGS_HEIGHT_GNSS: return true;
    default:                   return platform == APP_PLATFORM_POLE;
    }
}

const char *settings_height_src_name(unsigned height_src)
{
    switch (height_src) {
    case SETTINGS_HEIGHT_AUTO: return "auto";
    case SETTINGS_HEIGHT_BARO: return "baro";
    case SETTINGS_HEIGHT_GNSS: return "gnss";
    default:                   return NULL;
    }
}

unsigned settings_platform(const settings_t *s)
{
    return s->platform_p1 == 0 ? (unsigned)APP_PLATFORM : (unsigned)(s->platform_p1 - 1u);
}

float settings_ant_height(const settings_t *s, unsigned platform)
{
    return platform == APP_PLATFORM_TRACTOR || platform == APP_PLATFORM_POLE ? s->ant_height_m : 0.0f;
}

const char *settings_platform_name(unsigned platform)
{
    switch (platform) {
    case APP_PLATFORM_TRACTOR:    return "tractor";
    case APP_PLATFORM_POLE:       return "pole";
    case APP_PLATFORM_MULTIROTOR: return "multirotor";
    case APP_PLATFORM_FIXED_WING: return "fixed_wing";
    default:                      return "?";
    }
}

bool settings_load(settings_t *s)
{
    rec_init();
    if (flash_rec_load(&s_rec, s)) {
        s_loaded_version = SETTINGS_VERSION;
    } else {
        /* An older record: its fields over the defaults of the newer ones.
         * The next save writes the current version. Newest first. */
        static const struct { uint16_t version, size; } k_old[] = {
            { 9, SETTINGS_V9_SIZE },
            { 8, SETTINGS_V8_SIZE },
            { 7, SETTINGS_V7_SIZE },
            { 6, SETTINGS_V6_SIZE },
            { 5, SETTINGS_V5_SIZE },
            { 4, SETTINGS_V4_SIZE },
            { 3, SETTINGS_V3_SIZE },
            { 2, SETTINGS_V2_SIZE },
            { 1, SETTINGS_V1_SIZE },
        };
        settings_t old;
        size_t i;
        for (i = 0; i < sizeof k_old / sizeof k_old[0]; i++) {
            if (flash_rec_load_as(&s_rec, k_old[i].version, k_old[i].size, &old)) {
                break;
            }
        }
        if (i == sizeof k_old / sizeof k_old[0]) {
            s_loaded_version = 0;
            settings_defaults(s);
            return false;
        }
        settings_defaults(s);
        memcpy(s, &old, k_old[i].size);
        s_loaded_version = k_old[i].version;
    }
    /* Make sure the strings end, whatever is in flash. */
    s->sta_ssid[SETTINGS_SSID_MAX] = '\0';
    s->sta_password[SETTINGS_PW_MAX] = '\0';
    return true;
}

static bool in_range(float v, float lo, float hi) { return isfinite(v) && v >= lo && v <= hi; }

static uint32_t ip_u32(const uint8_t a[4])
{
    return ((uint32_t)a[0] << 24) | ((uint32_t)a[1] << 16) | ((uint32_t)a[2] << 8) | a[3];
}

/* NULL if the Ethernet addresses make sense together. */
static const char *eth_validate(const settings_t *s)
{
    if (s->eth_fitted > 1) {
        return "Ethernet fitted: on or off";
    }
    uint32_t ip = ip_u32(s->eth_ip), mask = ip_u32(s->eth_mask), gw = ip_u32(s->eth_gw);
    /* A contiguous mask, /8 .. /30. */
    if ((~mask & (~mask + 1u)) != 0 || mask < 0xFF000000u || mask > 0xFFFFFFFCu) {
        return "Ethernet subnet mask: contiguous, 255.0.0.0 .. 255.255.255.252";
    }
    if (s->eth_ip[0] == 0 || s->eth_ip[0] == 127 || s->eth_ip[0] >= 224 || (ip & ~mask) == 0 ||
        (ip & ~mask) == ~mask) {
        return "Ethernet IP address: a host address (not the network or broadcast address)";
    }
    if (gw != 0 && ((gw & mask) != (ip & mask) || gw == ip)) {
        return "Ethernet gateway: 0.0.0.0 (none) or another address in the same subnet";
    }
    return NULL;
}

const char *settings_validate(const settings_t *s)
{
    for (int i = 0; i < 3; i++) {
        if (!in_range(s->leverarm_m[i], -10.0f, 10.0f)) {
            return "lever arm: each axis must be within -10 .. 10 m";
        }
    }
    if (!in_range(s->acc_psd, 1e-9f, 1e-1f)) {
        return "acc_psd: must be within 1e-9 .. 1e-1 (m/s^2)^2/Hz";
    }
    if (!in_range(s->gyr_psd, 1e-12f, 1e-3f)) {
        return "gyr_psd: must be within 1e-12 .. 1e-3 (rad/s)^2/Hz";
    }
    if (!in_range(s->gyr_bias_rw, 1e-9f, 1e-2f)) {
        return "gyr_bias_rw: must be within 1e-9 .. 1e-2 rad/s/sqrt(s)";
    }
    if (!in_range(s->acc_bias_rw, 1e-7f, 1e-1f)) {
        return "acc_bias_rw: must be within 1e-7 .. 1e-1 m/s^2/sqrt(s)";
    }
    if (!in_range(s->automotive_min_speed, 0.0f, 20.0f)) {
        return "automotive_min_speed: must be within 0 .. 20 m/s";
    }
    if (!in_range(s->max_deadreckoning_s, 0.0f, 3600.0f)) {
        return "max_deadreckoning_s: must be within 0 .. 3600 s (0 = INSLIB default, 10 s)";
    }
    if (!in_range(s->ant_height_m, 0.0f, 10.0f)) {
        return "antenna height over ground: must be within 0 .. 10 m (0 = antenna position)";
    }
    if (!in_range(s->baseline_yaw_deg, -180.0f, 360.0f)) {
        return "baseline_yaw_deg: must be within -180 .. 360";
    }
    if (!in_range(s->baseline_gate_m, 0.001f, 1.0f)) {
        return "baseline_gate_m: must be within 0.001 .. 1 m";
    }
    if (s->antenna_dual > 1) {
        return "antenna: must be single or dual";
    }
    if (s->platform_p1 > APP_PLATFORM_FIXED_WING + 1) {
        return "platform: tractor, pole, multirotor or fixed_wing";
    }
    if (!in_range(s->flow_height_m, 0.1f, 10.0f)) {
        return "flow camera height: must be within 0.1 .. 10 m";
    }
    if (!in_range(s->flow_cam_yaw_deg, -180.0f, 360.0f)) {
        return "flow camera yaw: must be within -180 .. 360";
    }
    for (int i = 0; i < 3; i++) {
        if (!in_range(s->flow_leverarm_m[i], -10.0f, 10.0f)) {
            return "flow camera lever arm: each axis must be within -10 .. 10 m";
        }
    }
    if (s->sensor_baro > 1 || s->sensor_mag > 1 || s->sensor_flow > 1 || s->sensor_wheel > 1) {
        return "sensors: each must be on or off";
    }
    if (!in_range(s->offset_interval_s, 0.0f, 600.0f)) {
        return "offset fusion interval: must be within 0 .. 600 s (0 = INSLIB default, 10 s)";
    }
    if (!in_range(s->offset_inflation, 0.0f, 10.0f)) {
        return "offset sigma inflation: must be within 0 .. 10 (0 = INSLIB default, 3)";
    }
    if (!in_range(s->offset_rw_mps, 0.0f, 10.0f)) {
        return "offset random walk: must be within 0 .. 10 m/sqrt(s) (0 = INSLIB default, 0.3)";
    }
    if (!in_range(s->offset_baro_sd_m, 0.0f, 10.0f)) {
        return "offset barometer sigma: must be within 0 .. 10 m (0 = INSLIB default, 2 m)";
    }
    if (!in_range(s->gnss_pos_decim, 0.0f, 100.0f) || s->gnss_pos_decim != (float)(int)s->gnss_pos_decim) {
        return "GNSS position decimation: a whole number 0 .. 100 (0 = INSLIB default, 1)";
    }
    if (rtcm_src_mode_name(s->rtcm_src) == NULL) {
        return "correction source: auto, ethernet or bluetooth";
    }
    const char *eth = eth_validate(s);
    if (eth != NULL) {
        return eth;
    }
    if (settings_height_src_name(s->height_src) == NULL) {
        return "INS height source: auto, baro or gnss";
    }
    if (s->wifi_mode > APP_WIFI_AP) {
        return "wifi_mode: must be auto, sta or ap";
    }
    size_t n = strnlen(s->sta_ssid, sizeof s->sta_ssid);
    if (n == 0 || n > SETTINGS_SSID_MAX) {
        return "sta_ssid: 1 .. 32 characters";
    }
    n = strnlen(s->sta_password, sizeof s->sta_password);
    if (n != 0 && (n < 8 || n > SETTINGS_PW_MAX)) {
        return "sta_password: empty (open network) or 8 .. 63 characters";
    }
    return NULL;
}

int settings_save(const settings_t *s)
{
    rec_init();
    return flash_rec_save(&s_rec, s);
}

int settings_erase(void)
{
    rec_init();
    return flash_rec_erase(&s_rec);
}

uint32_t settings_seq(void) { return flash_rec_seq(&s_rec); }

bool settings_reboot_fields_differ(const settings_t *a, const settings_t *b)
{
    return a->antenna_dual != b->antenna_dual || a->wifi_mode != b->wifi_mode ||
           settings_platform(a) != settings_platform(b) ||
           strcmp(a->sta_ssid, b->sta_ssid) != 0 || strcmp(a->sta_password, b->sta_password) != 0;
}
