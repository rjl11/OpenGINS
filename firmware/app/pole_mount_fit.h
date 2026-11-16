/*
 * The pole axis in the IMU frame, from planted episodes (plans/pole-tilt-aiding.md,
 * item 5): the least-squares part, plain C for the host test.
 *
 * Per RTK fix i of episode k: p_i = t_k + Rz(psi_k) G_i d, with d = antenna -
 * tip in the body frame, t_k the episode's tip (north, east), psi_k its yaw
 * correction and G_i the top two rows of the gyro-integrated attitude
 * (pole_episode_t). The model is linear in d, so the reduced normal
 * equations of every episode (t_k eliminated) add up across episodes and
 * boots, whatever d is in use.
 *
 * Only a twist of the pole about its own axis shows the axis tilt (at a
 * constant yaw the tilt is a constant shift that t_k takes up); the sway
 * shows the length |d|.
 *
 * psi_k is refitted per episode against the current d (2-D Procrustes),
 * so a mounting error doesn't leave a yaw error behind.
 */
#ifndef POLE_MOUNT_FIT_H
#define POLE_MOUNT_FIT_H

#include <stdbool.h>
#include <stdint.h>

#include "pole_aid.h"

/* Accumulated normal equations of all accepted episodes: the residual sum
 * of squares at d is c - 2 b.d + d.N d [m^2]. */
typedef struct {
    double   N[3][3];
    double   b[3];
    double   c;
    uint32_t n_fix;
    uint32_t n_ep;
    float    twist_deg;    /* sum of the episodes' twist ranges             */
    float    pad;
} pm_stats_t;

typedef struct {
    double d0[3];          /* prior mean: (0, 0, -h), the pole along body down */
    double sd[3];          /* prior sigma per axis [m]                      */
    double fix_sd_m;       /* 1-sigma of one horizontal fix component       */
    double corr;           /* variance inflation for time-correlated RTK errors */
    float  ep_rms_max_m;   /* an episode fitting worse than this is dropped */
    float  ep_rms_rel;     /* once PM_REL_MIN_EP episodes are in: also worse
                              than this times the overall fix RMS (but at
                              least PM_REL_FLOOR_M)                       */
} pm_prior_t;

#define PM_REL_MIN_EP   5
#define PM_REL_FLOOR_M  0.01f

typedef struct {
    bool  ok;              /* solved                                        */
    double d[3];           /* antenna - tip, body FRD [m]                   */
    double sd[3];
    float u[3];            /* pole axis, unit, antenna to tip               */
    float len_m, len_sd_m; /* |d|                                           */
    float tilt_deg;        /* u from the body down axis                     */
    float azim_deg;        /* direction of the tilt in the body x/y plane   */
    float tilt_sd_deg;     /* larger of the two horizontal sigmas, as an angle */
    float rms_m;           /* fix residual over all episodes                */
} pm_result_t;

/* Prior for an antenna height h [m]: tilt sigma tilt_sd_deg, length sigma
 * len_sd_m; the fix noise and the episode gates as in pm_prior_t. */
void pm_prior_init(pm_prior_t *p, float h_m, float tilt_sd_deg, float len_sd_m, float fix_sd_m, float corr,
                   float ep_rms_max_m, float ep_rms_rel);

void pm_solve(const pm_stats_t *s, const pm_prior_t *p, pm_result_t *r);

/* Yaw correction of episode e that best fits d. */
float pm_episode_psi(const pole_episode_t *e, const double d[3]);

/* e's normal equations with t eliminated, at yaw correction psi. */
void pm_episode_reduce(const pole_episode_t *e, float psi, double N[3][3], double b[3], double *c);

/* Adds e to s unless it fits worse than the gates (pm_prior_t) at the
 * solution that includes it. rms_out (may be NULL) gets that fit; returns whether
 * it was added. */
bool pm_add_episode(pm_stats_t *s, const pole_episode_t *e, const pm_prior_t *p, float *rms_out);

/* The nominal axis, (0, 0, 1): the pole along the body down axis. */
void pm_axis_default(float u[3]);

#endif /* POLE_MOUNT_FIT_H */
