/*
 * Background IMU calibration on core 0 (plans/imu-auto-calibration.md):
 * gyro bias vs temperature (Phase 1) and the accelerometer from static
 * poses (Phase 2), learned at standstills, stored in flash, applied at the
 * next boot.
 *
 * - Raw TM210 samples (before any calibration) go into a stillness detector
 *   (the same 0.2 s window test as INSLIB's, APP_STILL_*). A run of still
 *   windows makes chunks of APP_CALIB_CHUNK_MIN_S .. _MAX_S; each gives the
 *   mean gyro at one temperature.
 * - The Earth rate is removed with the attitude from core 1 (INS or AHRS
 *   only, so yaw is known) and the latitude of the last GNSS fix, else
 *   APP_CALIB_DEFAULT_LAT_DEG.
 * - Each still run (at least APP_CALIB_POSE_MIN_S, averaged up to
 *   APP_CALIB_POSE_MAX_S) is also an accelerometer pose, kept in one of 30
 *   bins by its gravity direction; the fit is imu_calib_acc.h.
 * - A window counts as still only if the mean rate and the accelerometer
 *   magnitude look like rest too, so a steady turn is not taken for one.
 * - Magnetometer (Phase 3): groups of samples taken while turning slowly go
 *   into 54 bins by direction, each with the down direction when the
 *   accelerometer is quiet; the ellipsoid fit (imu_calib_mag.h) needs a GNSS
 *   fix for the WMM field. When the INS yaw comes from a GNSS heading (dual
 *   antenna, or the course on platforms where it is the heading), the WMM
 *   vector in the body frame adds reference samples at 1 Hz.
 * - The gyro fit is imu_calib_fit.h. Its statistics are kept in their own flash
 *   record (flash_rec.h, the two sectors below the settings), tied to the
 *   TM210 node ID.
 * - At boot the stored set (after the range check) is the correction for
 *   this run: b0 goes to INSLIB as the fixed gyro bias (nav_params_t), and
 *   imu_calib_correct() subtracts k * (T - T_ref) from each sample. The
 *   accelerometer set goes to INSLIB as imu_acc_misalignment / _fixed_bias.
 * - Per group (Phase 5, imu_calib_action()): mode auto applies the fit of
 *   the data at each boot (the default), hold keeps the stored set until it
 *   is accepted. The set in use is stored too, so a group without usable
 *   data keeps it. Accept and factory change the correction at once, only
 *   while still, and the caller restarts the filter.
 *
 * Core 0 only.
 */
#ifndef IMU_CALIB_H
#define IMU_CALIB_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "imu_calib_acc.h"
#include "imu_calib_fit.h"
#include "imu_calib_mag.h"
#include "imu_calib_rot.h"
#include "tm210.h"

typedef enum {
    IMU_CALIB_ID_WAIT = 0,   /* TM210 not identified yet                    */
    IMU_CALIB_ID_OK,         /* node ID matches the record (or adopted)     */
    IMU_CALIB_ID_UNKNOWN,    /* runs without a node ID: no learning         */
} imu_calib_id_t;

/* Calibration groups (imu_calib_action() and the status). */
typedef enum {
    IMU_CALIB_GYR = 0,       /* gyro bias vs temperature                    */
    IMU_CALIB_ACC,           /* accelerometer                               */
    IMU_CALIB_MAG,           /* magnetometer                                */
    IMU_CALIB_GYR_M,         /* gyro scale and misalignment                 */
    IMU_CALIB_GROUPS,        /* as a group: all of them (factory only)      */
} imu_calib_group_t;

typedef enum {
    IMU_CALIB_OP_ACCEPT = 0, /* the learned set is used now and stored (still only) */
    IMU_CALIB_OP_HOLD,       /* boot keeps the stored set; learning goes on */
    IMU_CALIB_OP_AUTO,       /* boot applies the fit of the data (default)  */
    IMU_CALIB_OP_CLEAR,      /* erase the collected data; the set in use stays */
    IMU_CALIB_OP_FACTORY,    /* data and stored set erased, no correction from now (still only) */
    IMU_CALIB_OPS,
} imu_calib_op_t;

typedef struct {
    bool          applied;      /* a learned set is the correction this boot */
    gcal_result_t use;          /* the correction in use (zero if not applied) */
    gcal_result_t learned;      /* posterior of the statistics so far        */
    const char   *learned_bad;  /* NULL, or why learned fails the range check */
    gcal_stats_t  stats;
    uint32_t      rec_seq;      /* flash record in use, 0 = none             */
    const char   *boot_note;    /* what happened to the record at boot       */
    imu_calib_id_t id_state;
    uint16_t      node_id;
    uint32_t      chunks_boot;  /* chunks added this boot                    */
    uint32_t      skip_att;     /* chunks skipped: no INS/AHRS attitude      */
    uint32_t      skip_temp;    /* chunks skipped: temperature span, no Status */
    uint32_t      unsaved;      /* chunks since the last save                */
    float         chunk_s;      /* duration of the chunk being collected     */
    bool          still;        /* inside a still run now                    */
    int           last_save_rc; /* 0 ok, negative flash error, 1 = none yet  */
    uint32_t      windows_not_rest; /* quiet windows refused: rate or |a| not at rest */

    /* Accelerometer (Phase 2). */
    bool          acc_applied;
    acal_result_t acc_use;      /* the correction in use                     */
    acal_result_t acc_learned;  /* fit of the stored poses                   */
    const char   *acc_learned_bad;
    const char   *acc_boot_note;
    uint32_t      acc_poses;    /* bins holding a pose (of 30)               */
    float         acc_coverage; /* 0 .. 1/3, see acal_coverage()             */
    uint32_t      poses_boot;   /* poses stored this boot                    */
    uint32_t      poses_kept_old; /* poses not stored: the bin had a better one */

    /* Magnetometer (Phase 3). */
    bool          mag_applied;
    mcal_cal_t    mag_use;      /* the correction in use                     */
    bool          mag_solved;   /* mag_learned / mag_ell are valid           */
    mcal_ell_t    mag_ell;      /* ellipsoid fit of the stored samples       */
    mcal_cal_t    mag_learned;  /* ellipsoid refined by the reference data   */
    const char   *mag_learned_bad;
    const char   *mag_boot_note; /* what happened to the record at boot     */
    const char   *mag_state;   /* what the fit is doing now                  */
    bool          mag_wait_fix; /* no GNSS position yet: no WMM field, no fit */
    uint32_t      mag_bins;     /* bins holding a sample (of 54)             */
    float         mag_coverage;
    uint32_t      mag_groups_boot;
    uint32_t      mag_outliers; /* groups not stored: far off the fit        */
    uint32_t      mag_n_ref;    /* reference samples in the statistics       */
    uint32_t      mag_ref_boot;
    const char   *mag_ref_source; /* what gives the INS yaw now, NULL = nothing usable */

    /* Gyro scale and misalignment (Phase 4). */
    bool          gyr_m_applied; /* a learned matrix is in use (not identity) */
    float         gyr_m[9];      /* in use, row-major                         */
    const char   *gyr_m_boot_note;
    rot_result_t  gyr_learned;   /* dM around gyr_m from the statistics       */
    float         gyr_next[9];   /* what the next boot would use              */
    int           gyr_taken;     /* elements of gyr_next that would change    */
    const char   *gyr_next_bad;
    uint32_t      n_rot, n_turn; /* in the statistics                         */
    uint32_t      rot_boot, turn_boot, rot_small;
    const char   *rot_state;     /* idle / armed / waiting for the pose       */
    bool          turn_active;

    /* Per group (Phase 5). The *_boot_note fields say what happened at boot,
     * or the last imu_calib_action() on the group. */
    bool          hold[IMU_CALIB_GROUPS];
    bool          stored[IMU_CALIB_GROUPS];     /* a set is stored (the fallback) */
    const char   *accept_why[IMU_CALIB_GROUPS]; /* NULL: the learned set can be accepted */
    const char   *next_boot[IMU_CALIB_GROUPS];  /* what the next boot will use */
} imu_calib_status_t;

/* Loads the record and works out this boot's correction. gyr_psd and
 * acc_psd are the white noise [(rad/s)^2/Hz, (m/s^2)^2/Hz] used to weight
 * the chunks and the poses. */
void imu_calib_init(float gyr_psd, float acc_psd, unsigned platform);

/* The latest usable GNSS fix: latitude, longitude [rad], ellipsoid height
 * [m] and the decimal year, for the local g, the Earth rate and the WMM. */
void imu_calib_set_position(double lat_rad, double lon_rad, float h_m, float year);

/* This boot's magnetometer correction for nav_params_t, as for the
 * accelerometer. */
void imu_calib_mag_calibration(float m_colmajor[9], float bias[3]);

/* This boot's gyro misalignment / scale matrix for nav_params_t (column-major,
 * explicit identity when nothing is learned). */
void imu_calib_gyr_misalignment(float m_colmajor[9]);

/* A GNSS heading epoch for the gyro turn fit: heading [rad, true north],
 * its sigma, and whether it is the course over ground (then it counts only
 * on a tractor driving forward or a fixed wing, by the platform). */
void imu_calib_gnss_heading(uint64_t t_us, float heading_rad, float sd_rad, bool is_course);
void imu_calib_set_psd(float gyr_psd, float acc_psd);

/* b0 [rad/s] of this boot's correction, for nav_params_t (zero if none). */
void imu_calib_gyr_fixed_bias(float out[3]);

/* This boot's accelerometer correction for nav_params_t: INSLIB's
 * column-major misalignment (all zero = identity) and fixed bias [m/s^2]. */
void imu_calib_acc_calibration(float m_colmajor[9], float bias[3]);

/* Subtracts k * (T - T_ref) from gyr when a correction is applied and the
 * TM210 temperature is known. */
void imu_calib_correct(float gyr[3], const tm210_stats_t *st);

/* Every raw TM210 sample, before imu_calib_correct(). */
void imu_calib_push(const tm210_sample_t *s, const tm210_stats_t *st);

/* Once per main loop. Checks the TM210 identity once it runs; returns true
 * if this boot's correction was dropped (another IMU), so the caller must
 * restart the filter without it. Also saves the record when due. */
bool imu_calib_poll(uint64_t now_us, const tm210_t *imu);

/* A web page action on group g (IMU_CALIB_GROUPS = all, factory only) and
 * saves the record. Returns 1 when the correction in use changed (the
 * caller restarts the filter with it), 0 when it did not, -1 when refused;
 * msg gets the outcome ("refused: ..." when refused). Accept and factory
 * need the unit still. */
int imu_calib_action(int g, int op, char *msg, size_t cap);

/* Names for the web API: "gyr", "acc", "mag", "gyr_m", "all"; "accept",
 * "hold", "auto", "clear", "factory". -1 if unknown. No state: safe in
 * lwIP context. */
int imu_calib_group_from_name(const char *s);
int imu_calib_op_from_name(const char *s);
const char *imu_calib_group_key(int g);

void imu_calib_status(imu_calib_status_t *out);

#endif /* IMU_CALIB_H */
