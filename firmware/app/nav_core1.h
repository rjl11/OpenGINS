/*
 * The INSLIB nav_suite filter on core 1 (plans/inslib-nav-suite.md,
 * plans/rtos-decision.md).
 *
 * Core 0 pushes sensor samples into three queues. Core 1 runs a plain loop
 * with nothing that preempts it: one nav_suite_update() per IMU sample, with
 * any GNSS epoch and barometer sample that arrived attached to it. Every
 * 100 ms it publishes its outputs for core 0 to read.
 *
 * All functions here are called from core 0.
 */
#ifndef NAV_CORE1_H
#define NAV_CORE1_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "pole_aid.h"

#include "cpu_load.h"

typedef enum {
    NAV_ANT_SINGLE = 0,    /* yaw from the GNSS course over ground          */
    NAV_ANT_DUAL           /* UM982: yaw from the dual-antenna heading      */
} nav_antenna_t;

/* IMU sample in FRD SI units. */
typedef struct {
    uint64_t t_us;         /* sample time, Pico clock                       */
    float    dt_s;         /* spacing to the previous sample                */
    float    acc[3];       /* specific force, m/s^2                         */
    float    gyr[3];       /* rad/s                                         */
    float    mag[3];       /* uT                                            */
    bool     mag_new;
} nav_imu_t;

/* GNSS epoch: a BESTNAV position/velocity, or a dual-antenna heading. Each
 * kind comes as its own item with its own time of validity. */
typedef struct {
    uint64_t t_valid_us;   /* time of validity, Pico clock                  */
    bool     pos_valid;
    bool     vel_valid;
    bool     yaw_valid;
    float    yaw_rad;      /* body yaw, NED, -pi..pi                        */
    float    yaw_sd_rad;
    double   lat_rad;
    double   lon_rad;
    double   h_ell_m;      /* height above the WGS84 ellipsoid              */
    float    pos_sd_ned[3];
    float    vel_ned[3];
    float    vel_sd_ned[3];
    uint16_t week;
    uint32_t tow_ms;
} nav_gnss_t;

typedef struct {
    uint64_t t_us;
    float    pressure_pa;
} nav_baro_t;

/* Ground speed for INSLIB's scalar speed input (REQ-NAV-068): horizontal
 * speed at the IMU point, unsigned. From optical flow (flow_nav.h) now,
 * wheel speed later; both share this queue. */
enum { NAV_SPEED_FLOW = 0, NAV_SPEED_WHEEL = 1, NAV_SPEED_SOURCES };
typedef struct {
    uint64_t t_valid_us;   /* time of validity, Pico clock                  */
    float    speed_mps;
    float    sd_mps;       /* 1-sigma                                       */
    uint8_t  source;       /* NAV_SPEED_*                                   */
} nav_speed_t;

/* Which INSLIB filter the attitude comes from, in nav_suite_get_rpy()'s
 * order: the INS if ready, else the magnetometer AHRS, else the gyro-only
 * ARS (relative yaw). */
typedef enum {
    NAV_ATT_NONE = 0,
    NAV_ATT_INS,
    NAV_ATT_AHRS,
    NAV_ATT_ARS,
} nav_att_source_t;

/* Published by core 1 every 100 ms. */
typedef struct {
    uint64_t t_us;         /* when it was published, 0 = filter not started */
    int      mode;         /* nav_suite_mode_t                              */
    bool     rpy_valid;
    float    rpy_rad[3];   /* best source: INS, else AHRS, else ARS         */
    bool     pos_valid;
    double   llh[3];       /* lat [rad], lon [rad], ellipsoid height [m]    */
    bool     vel_valid;
    float    vel_ned[3];
    bool     height_valid;
    float    height_m;     /* above the start point, positive up            */
    bool     bias_valid;
    float    gyr_bias[3];  /* rad/s                                         */
    float    acc_bias[3];  /* m/s^2                                         */
    bool     zaru_active;
    int      att_source;   /* nav_att_source_t: the filter rpy_rad came from */
    bool     rpy_sd_valid; /* att_source is INS: its own attitude 1-sigma   */
    float    rpy_sd_rad[3];

    /* Magnetometer AHRS (INSLIB's ahrs filter). */
    bool     ahrs_valid;   /* initialized and healthy                       */
    int32_t  mag_fusion_age_ms; /* since the last magnetometer correction, -1 = none yet */
    bool     mag_declination;   /* a position set the declination: true north */
    bool     mag_on_magnetic_north; /* yaw still carries pre-declination fusion */
    uint32_t n_mag;        /* magnetometer samples given to the filter      */
    uint32_t n_mag_withheld; /* pole: samples kept back, magnetometer untrusted */
    pole_aid_status_t pole; /* survey-pole aiding (pole_aid.h)              */

    /* IMU loss. Once INSLIB's timeout (0.2 s) passes without an IMU sample it
     * stops every filter: mode NONE, no position, velocity or attitude until
     * the IMU is back and the filter has re-initialised. */
    int32_t  imu_age_ms;   /* since the last IMU sample, -1 = none yet      */
    uint32_t n_imu_loss;   /* IMU losses that stopped the filter (nav_suite) */
    uint32_t n_stall_epochs; /* epochs run without an IMU sample            */

    uint32_t n_updates;    /* nav_suite_update() calls                      */
    uint32_t n_gnss;       /* GNSS epochs given to the filter               */
    uint32_t n_gnss_late;  /* GNSS epochs dropped: older than INSLIB's limit */
    uint32_t n_yaw;        /* dual-antenna headings given to the filter     */
    bool     dual;         /* antenna mode the filter was started with      */
    uint32_t n_baro;
    uint32_t n_baro_late;  /* barometer samples dropped: too old            */
    uint32_t n_speed[NAV_SPEED_SOURCES]; /* speed samples given to the filter, per source */
    uint32_t n_speed_late; /* speed samples dropped: older than INSLIB's limit */
    uint32_t n_speed_used; /* INSLIB fused them (ins_diag_t)                */
    uint32_t n_speed_skipped; /* INSLIB skipped them: too slow, or no history */
    uint32_t update_us_max;  /* longest nav_suite_update() since the last report */
    uint32_t update_us_mean; /* mean over the same interval                 */
    cpu_load_result_t load;  /* core 1 loop load over the last whole second (cpu_load.h) */
    uint32_t n_reinit;     /* filter (re)initialisations                    */
    uint64_t t_settled_us; /* when this run's startup counts were cleared, 0 = not yet */
} nav_output_t;

/* Filter settings from the web config page (settings.h). */
typedef struct {
    float leverarm_m[3];         /* GNSS antenna from the IMU, body FRD */
    float acc_psd;               /* (m/s^2)^2/Hz */
    float gyr_psd;               /* (rad/s)^2/Hz */
    float gyr_bias_rw;           /* rad/s/sqrt(s) */
    float acc_bias_rw;           /* m/s^2/sqrt(s) */
    float automotive_min_speed;  /* m/s, 0 = INSLIB default */
    float max_deadreckoning_s;   /* s, 0 = INSLIB default; a change alone
                                    doesn't restart the filter */
    float gyr_fixed_bias[3];     /* rad/s, learned at standstills (imu_calib.h) */
    float gyr_misalignment[9];   /* column-major, all zero = identity (imu_calib.h) */
    float acc_misalignment[9];   /* column-major, all zero = identity (imu_calib.h) */
    float acc_fixed_bias[3];     /* m/s^2 */
    float mag_misalignment[9];   /* column-major, all zero = identity (imu_calib.h) */
    float mag_fixed_bias[3];     /* uT, hard iron */
    bool  use_mag;               /* sensors fitted (settings.h): a change restarts */
    bool  use_baro;              /*   the filter, which picks its vertical channel at init */
    /* GNSS course over ground may count as the heading (INSLIB
     * automotive_mode, single antenna only): true on a tractor or a fixed
     * wing, false on a survey pole or a multirotor, where the direction of
     * travel says nothing about where the body points. From the platform in
     * effect since boot. */
    bool  course_yaw;
    /* Survey pole with an antenna height (pole_aid.h): planted-tip
     * constraint, yaw fixes from sway, magnetometer trust gate. With the
     * tip lever arm (IMU to tip, body FRD: the lever arm plus the antenna
     * height along the pole axis, pole_mount.h), both applied live without
     * a filter restart. */
    bool  pole_aid;
    float tip_leverarm_m[3];
} nav_params_t;

/* Creates the queues and starts core 1. The filter starts at once with the
 * antenna-mode setting, which sets automotive_mode. Core 1 also registers
 * for flash lockout, so settings_save() can pause it. */
void nav_start(nav_antenna_t mode, const nav_params_t *params);

/* New filter settings: core 1 picks them up before its next sample and
 * re-initialises the filter (re-align, then navigate). Output pauses until
 * then; n_reinit counts it. A change to max_deadreckoning_s alone is applied
 * to the running filter instead. */
void nav_set_params(const nav_params_t *params);

/* Each returns false if the queue was full and the sample was dropped. */
bool nav_push_imu(const nav_imu_t *s);
bool nav_push_gnss(const nav_gnss_t *s);
bool nav_push_baro(const nav_baro_t *s);
bool nav_push_speed(const nav_speed_t *s);

/* The effective antenna mode, once the receiver is identified. Core 1
 * re-initialises the filter only if it differs from the mode it runs with
 * (dual set, but a UM980 fitted). */
void nav_set_antenna_mode(nav_antenna_t mode);

void nav_get_output(nav_output_t *out);

/* Counts core 1's publishes (10 per second). A poller compares it with the
 * value it saw last and calls nav_get_output() only when it changed: the
 * copy takes the output lock with interrupts off, which is too costly to
 * repeat on every main-loop pass. */
uint32_t nav_output_seq(void);

/* Samples dropped because a queue to core 1 was full, per queue. Core 1
 * takes GNSS and baro samples only alongside an IMU sample, so GNSS and
 * baro drops that climb while imu stays flat mean no IMU data is flowing. */
typedef struct {
    uint32_t imu;
    uint32_t gnss;
    uint32_t baro;
    uint32_t speed;
} nav_drops_t;

/* Core 0 only (the counters are written by the nav_push_*() calls). */
void nav_queue_drops(nav_drops_t *out);

/* Startup counts: samples queue up (and get dropped or arrive late) while
 * the filter has no IMU data yet, at boot while the TM210 is detected and
 * after a settings change. Core 1 clears its late counts NAV_SETTLE_US after
 * the first IMU sample of each filter run and sets nav_output_t.t_settled_us;
 * core 0 then calls nav_queue_drops_settle() to clear the drop counts. */
#define NAV_SETTLE_US 1000000u
void nav_queue_drops_settle(void);

/* Next survey-pole episode record for the pole-axis fit (pole_mount.h). */
bool nav_pop_pole_episode(pole_episode_t *out);

/* Next INSLIB log message, if any. Print it from core 0. */
bool nav_pop_log(char *buf, size_t len);

/* Highest core 1 stack use so far [bytes], and the stack size. */
uint32_t nav_stack_used(void);
uint32_t nav_stack_size(void);

const char *nav_mode_name(int mode);

#endif /* NAV_CORE1_H */
