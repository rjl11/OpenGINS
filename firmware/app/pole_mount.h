/*
 * The pole axis in the IMU frame, learned on the survey pole
 * (plans/pole-tilt-aiding.md, item 5). Core 0 only.
 *
 * - Core 1's pole_aid hands over one record per planted episode
 *   (pole_episode_t, through nav_pop_pole_episode()); pole_mount_fit.h adds
 *   it to the stored normal equations.
 * - The statistics, the axis in use and the mode are kept in their own flash
 *   record (flash_rec.h, the two sectors below the IMU calibration's).
 * - The axis replaces the body down axis in the tip lever arm (nav_params_t)
 *   and in the output ground point. The pole length is learned too, but
 *   only shown against the antenna height setting, never applied.
 * - Mode auto (default): each boot uses the learned axis when it passes
 *   the checks, else the stored set. Hold: the stored set until accepted.
 *   Accept and factory change the axis at once (no stillness needed: the
 *   tip lever arm is applied without a filter restart).
 */
#ifndef POLE_MOUNT_H
#define POLE_MOUNT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "pole_aid.h"
#include "pole_mount_fit.h"

/* The web API's group number for this group (beside imu_calib_group_t). */
#define POLE_MOUNT_GROUP 100

typedef struct {
    bool        active;       /* pole platform with an antenna height, this boot */
    bool        applied;      /* the axis in use is not the body down axis  */
    bool        hold;
    bool        stored;       /* a set is stored                            */
    float       use_u[3];     /* axis in use, unit, antenna to tip, body FRD */
    float       use_tilt_deg, use_azim_deg;
    pm_result_t learned;
    pm_stats_t  stats;
    uint32_t    ep_boot;      /* episodes added this boot                   */
    uint32_t    ep_rejected;  /* dropped this boot: fit worse than the gates */
    float       last_rms_m;   /* the last episode's fit, -1 = none          */
    float       last_twist_deg;
    bool        last_added;
    float       ant_height_m;
    bool        len_differs;  /* learned length vs the setting: over 3 sigma and 2 cm */
    const char *accept_why;   /* NULL: the learned axis can be accepted     */
    const char *next_boot;
    const char *note;         /* boot, or the last action                   */
    uint32_t    rec_seq;
    int         last_save_rc; /* 0 ok, negative flash error, 1 = none yet   */
} pole_mount_status_t;

/* Loads the record and picks this boot's axis. active: the pole platform
 * with an antenna height h_m > 0. Before the first nav_params. */
void pole_mount_init(bool active, float h_m);

/* New antenna height (the prior's length). */
void pole_mount_set_height(float h_m);

/* The axis in use: unit, antenna to tip, body FRD; (0, 0, 1) when off. */
void pole_mount_axis(float u[3]);

void pole_mount_add(const pole_episode_t *e);

/* Once per main loop: saves the record when due. */
void pole_mount_poll(uint64_t now_us);

/* A web page action (imu_calib_op_t). Returns 1 when the axis in use
 * changed (the caller sends new nav params), 0 when not, -1 when refused;
 * msg gets the outcome. */
int pole_mount_action(int op, char *msg, size_t cap);

void pole_mount_status(pole_mount_status_t *out);

#endif /* POLE_MOUNT_H */
