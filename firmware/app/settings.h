/*
 * User settings: defaults from app_config.h / cmake, overridden by the copy
 * saved in flash from the web config page (plans/web-config-server.md,
 * "Configuration page" and "Storing configuration").
 *
 * Flash layout: the last two 4 KB sectors, written alternately. Each holds
 * one record (header with a sequence number and a CRC, then the struct); the
 * valid record with the higher sequence number wins, so a power cut during a
 * save leaves the previous one.
 *
 * Core 0 only.
 */
#ifndef SETTINGS_H
#define SETTINGS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "flow_link.h"

#define SETTINGS_SSID_MAX   32
#define SETTINGS_PW_MAX     63

/* Bump SETTINGS_VERSION whenever this struct changes: a record with another
 * version is ignored and the defaults are used, unless settings_load()
 * migrates it. Add new fields at the end, so an older record is a prefix
 * and can be migrated: version 1 (140 bytes, up to platform_p1), version 2
 * (160 bytes, up to the flow geometry), version 3 (164 bytes, up to the
 * sensors fitted), version 4 (168 bytes, up to the dead-reckoning
 * window), version 5 (172 bytes, up to the antenna height), version 6
 * (176 bytes, up to the height source), version 7 (188 bytes, up to the
 * offset filter), version 8 (192 bytes, up to the GNSS position
 * decimation), version 9 (196 bytes, up to the offset barometer sigma),
 * version 10 (212 bytes, up to the correction source) and version 11 (224
 * bytes, up to the front TM171 mounting) are read with the newer fields at
 * their defaults.
 * (platform_p1 was added without a bump: see there.) */
#define SETTINGS_VERSION    12
#define SETTINGS_V1_SIZE    140u
#define SETTINGS_V2_SIZE    160u
#define SETTINGS_V3_SIZE    164u
#define SETTINGS_V4_SIZE    168u
#define SETTINGS_V5_SIZE    172u
#define SETTINGS_V6_SIZE    176u
#define SETTINGS_V7_SIZE    188u
#define SETTINGS_V8_SIZE    192u
#define SETTINGS_V9_SIZE    196u
#define SETTINGS_V10_SIZE   212u
#define SETTINGS_V11_SIZE   224u

/* settings_t.height_src: where the INS takes its height from. */
#define SETTINGS_HEIGHT_AUTO 0 /* by platform: GNSS on a survey pole, else the barometer */
#define SETTINGS_HEIGHT_BARO 1 /* the barometer (INSLIB's choice when one streams) */
#define SETTINGS_HEIGHT_GNSS 2 /* the GNSS height (INSLIB baro_height_disable) */

typedef struct {
    /* Filter: applied live, the filter re-initialises. */
    float leverarm_m[3];         /* GNSS antenna from the IMU, body FRD */
    float acc_psd;               /* (m/s^2)^2/Hz */
    float gyr_psd;               /* (rad/s)^2/Hz */
    float gyr_bias_rw;           /* rad/s/sqrt(s) */
    float acc_bias_rw;           /* m/s^2/sqrt(s) */
    float automotive_min_speed;  /* m/s, 0 = INSLIB default */

    /* Dual-antenna heading: applied live on core 0. */
    float baseline_yaw_deg;
    float baseline_gate_m;

    /* Applied after a reboot. */
    uint8_t antenna_dual;        /* 0 single, 1 dual */
    uint8_t wifi_mode;           /* APP_WIFI_AUTO / _STA / _AP */
    char sta_ssid[SETTINGS_SSID_MAX + 1];
    char sta_password[SETTINGS_PW_MAX + 1]; /* "" = open network */
    /* APP_PLATFORM_* + 1. It sits in what was padding, so the struct keeps
     * its size and older records read 0 = the build default. Use
     * settings_platform(). */
    uint8_t platform_p1;

    /* Version 2. Optical-flow camera geometry (app/flow_nav.h): applied live
     * on core 0, the filter keeps running. */
    float flow_height_m;         /* lens above level ground */
    float flow_cam_yaw_deg;      /* image top from body forward, clockwise from above */
    float flow_leverarm_m[3];    /* camera from the IMU, body FRD */

    /* Version 3. Sensors fitted, 1 = on: not every vehicle has every sensor.
     * Applied live; the barometer and the magnetometer restart the filter. */
    uint8_t sensor_baro;         /* BMP581: polled and fed to INSLIB */
    uint8_t sensor_mag;          /* TM210 magnetometer fed to INSLIB */
    uint8_t sensor_flow;         /* camera trigger, strobe, flow into INSLIB */
    uint8_t sensor_wheel;        /* wheel / ground-speed pulse inputs */

    /* Version 4. Longest dead reckoning after the last GNSS position, s,
     * 0 = INSLIB default (10 s). Applied live without a filter restart. */
    float max_deadreckoning_s;

    /* Version 5. Tractor and survey pole only: GNSS antenna above the
     * ground along the body down axis, m, 0 = output the antenna position.
     * Applied live on core 0 (the output position only). */
    float ant_height_m;

    /* Version 6. Height source of the INS, SETTINGS_HEIGHT_*. INSLIB picks
     * it when the INS starts, so a change restarts the filter (live, no
     * reboot). Only matters while the barometer streams: without one the
     * INS uses the GNSS height anyway. Use settings_ins_gnss_height(). */
    uint8_t height_src;

    /* Version 7. INSLIB's offset filter (local_gnss_alt: the barometer's
     * offset to the ellipsoid, the absolute height during an outage, and
     * with the barometric INS height also the drift taken back off the
     * published ellipsoid height), with either height source.
     * 0 = INSLIB's default. Defaults
     * APP_HEIGHT_OFFSET_*. A change restarts the filter. */
    float offset_interval_s;     /* min_update_interval_sec, INSLIB 10 s */
    float offset_inflation;      /* stddev_inflation_factor, INSLIB 3 */
    float offset_rw_mps;         /* rw_stddev_mps [m/sqrt(s)], INSLIB 0.3 */

    /* Version 8. INSLIB gnss_pos_decimation: the GNSS position on 1 of
     * every N fused epochs, the velocity alone on the others. A whole
     * number 0 .. 100 (a float, so it goes through the web form's number
     * fields); 0 = INSLIB's default (1). Default APP_GNSS_POS_DECIMATION.
     * Applied live, no filter restart. */
    float gnss_pos_decim;

    /* Version 9. The offset filter's barometer 1-sigma per pair
     * (local_gnss_cfg.local_stddev_m) [m], used only with the barometric
     * INS height (INSLIB's default with the GNSS height); 0 = INSLIB's default
     * (INS_DEFAULT_BARO_STDDEV_M, 2 m). Default APP_HEIGHT_OFFSET_BARO_SD_M.
     * A change restarts the filter. */
    float offset_baro_sd_m;

    /* Version 10. W5500 Ethernet (app/eth.h, plans/ethernet-w5500-agopenweb.md):
     * fitted or not (1 = on; off leaves its pins alone), and the static IPv4
     * address, subnet mask and gateway. Applied live: the chip is started,
     * held in reset or given the new addresses. Defaults APP_ETH_*. */
    uint8_t eth_fitted;
    uint8_t eth_ip[4];
    uint8_t eth_mask[4];
    uint8_t eth_gw[4];
    /* Which RTCM source feeds the receiver (app/rtcm_src.h,
     * RTCM_SRC_MODE_*): 0 auto (the first to deliver, until it goes
     * silent), 1 Ethernet only, 2 Bluetooth only. Applied live. */
    uint8_t rtcm_src;

    /* Version 11. The front camera unit's TM171 against the vehicle body
     * (FRD): roll, pitch, yaw in degrees, TM171 -> body = Rz Ry Rx. Sent
     * to the unit with the camera height and image direction once a second
     * (FLOW_MSG_UNIT_CFG, settings_unit_cfg()). Defaults APP_FRONT_IMU_*. */
    float front_imu_rpy_deg[3];

    /* Version 12. The front unit's TM171 as the backup IMU
     * (plans/front-camera-unit.md, "Backup IMU for the roof TM210"): its
     * position from the roof IMU (body FRD, m), so the antenna lever arm for
     * the filter on the TM171 is leverarm_m minus this; and whether core 0
     * may switch to it when the TM210 is lost (1 = on). Applied live.
     * Defaults APP_FRONT_IMU_LEVER_*, APP_FRONT_IMU_BACKUP. */
    float front_imu_lever_m[3];
    /* Tractor with a front unit: the cab's roll centre above the ground
     * [m]. Above 0 (and below the antenna height), the ground point goes
     * from the antenna to the roll centre along the cab's attitude and on
     * along the chassis attitude from the TM171 (ground_point_cab()); 0 =
     * one roll over the whole antenna height. Applied live. Default
     * APP_CAB_ROLL_CENTRE_M. */
    float cab_roll_centre_m;
    uint8_t front_imu_backup;
} settings_t;

/* The platform (APP_PLATFORM_*) the settings ask for. */
unsigned settings_platform(const settings_t *s);

/* ant_height_m when platform (APP_PLATFORM_*) is a tractor or a survey
 * pole, else 0. The caller passes the platform in effect, which changes
 * only at a reboot. */
float settings_ant_height(const settings_t *s, unsigned platform);
const char *settings_platform_name(unsigned platform);

/* True when the INS should keep the GNSS height (INSLIB
 * baro_height_disable), from height_src and, for auto, the platform in
 * effect (APP_PLATFORM_*). */
bool settings_ins_gnss_height(const settings_t *s, unsigned platform);
/* "auto", "baro" or "gnss"; NULL for an unknown value. */
const char *settings_height_src_name(unsigned height_src);

void settings_defaults(settings_t *s);

/* Loads the newest valid record, or the defaults if there is none. Returns
 * true if the values came from flash. */
bool settings_load(settings_t *s);

/* Version of the record settings_load() read (older = migrated, the newer
 * fields at their defaults until the next save), 0 = defaults. */
unsigned settings_loaded_version(void);

/* NULL if every field is in range, else a message naming the field. */
const char *settings_validate(const settings_t *s);

/* The float fields' limits, one entry per web form field (the key is its
 * name on config.html), checked by settings_validate() and published in
 * /config.json so the page can mark a bad value before posting. */
typedef struct {
    const char *key;
    uint16_t offset; /* of the float in settings_t */
    float lo, hi;
    const char *msg; /* settings_validate()'s answer when out of range */
} settings_range_t;
extern const settings_range_t settings_ranges[];
extern const unsigned settings_n_ranges;

/* The front camera unit's settings message (common/flow_link.h,
 * FLOW_MSG_UNIT_CFG) from these settings, with this sequence number. */
void settings_unit_cfg(const settings_t *s, uint32_t seq, flow_msg_unit_cfg_t *out);

/* Writes a new record into the older sector. Pauses core 1 (flash lockout)
 * and core 0's interrupts for one sector erase, about 50 ms (estimate, not
 * yet measured). Returns 0 or a negative error. */
int settings_save(const settings_t *s);

/* Erases both sectors: the next boot uses the defaults. */
int settings_erase(void);

/* Sequence number of the record in use, 0 = defaults. */
uint32_t settings_seq(void);

/* True if the fields that only apply at boot differ between a and b. */
bool settings_reboot_fields_differ(const settings_t *a, const settings_t *b);

#endif /* SETTINGS_H */
