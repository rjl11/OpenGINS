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

#define SETTINGS_SSID_MAX   32
#define SETTINGS_PW_MAX     63

/* Bump SETTINGS_VERSION whenever this struct changes: a record with another
 * version is ignored and the defaults are used, unless settings_load()
 * migrates it. Add new fields at the end, so an older record is a prefix
 * and can be migrated: version 1 (140 bytes, up to platform_p1), version 2
 * (160 bytes, up to the flow geometry), version 3 (164 bytes, up to the
 * sensors fitted), version 4 (168 bytes, up to the dead-reckoning
 * window), version 5 (172 bytes, up to the antenna height) and version 6
 * (176 bytes, up to the height source) are read with the newer fields at
 * their defaults.
 * (platform_p1 was added without a bump: see there.) */
#define SETTINGS_VERSION    7
#define SETTINGS_V1_SIZE    140u
#define SETTINGS_V2_SIZE    160u
#define SETTINGS_V3_SIZE    164u
#define SETTINGS_V4_SIZE    168u
#define SETTINGS_V5_SIZE    172u
#define SETTINGS_V6_SIZE    176u

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
     * offset to the ellipsoid, the absolute height during an outage), used
     * only while the INS keeps the GNSS height (settings_ins_gnss_height());
     * otherwise INSLIB's defaults apply. 0 = INSLIB's default. Defaults
     * APP_HEIGHT_OFFSET_*. A change restarts the filter. */
    float offset_interval_s;     /* min_update_interval_sec, INSLIB 10 s */
    float offset_inflation;      /* stddev_inflation_factor, INSLIB 3 */
    float offset_rw_mps;         /* rw_stddev_mps [m/sqrt(s)], INSLIB 0.3 */
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
