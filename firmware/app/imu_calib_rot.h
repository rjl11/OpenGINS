/*
 * Gyro scale and misalignment (plans/imu-auto-calibration.md, Phase 4).
 *
 * Model (INSLIB's REQ-NAV-037 form): w = M * (raw - b), b the bias from
 * Phase 1. M = M0 + dM, M0 the matrix in use, dM (9 values, row-major) the
 * unknown. Two kinds of observation, both linear in dM after linearising
 * at M0, so their sufficient statistics are one 9x9 normal matrix and one
 * right-hand side:
 *
 * 1. A rotation between two still poses (imu_tk's gyro cost). Integrated
 *    with M0, the gyro gives dR0; the gravity directions g1, g2 of the two
 *    poses must satisfy dR^T g1 = g2. The integration also carries the 3x9
 *    Jacobian J of the rotation (right perturbation, dR = dR0 Exp(J dM)),
 *    the way IMU preintegration carries the bias Jacobian (Forster et al.,
 *    IEEE T-RO 2017), so nothing but dR0 and J is kept per rotation.
 * 2. A turn with a GNSS heading before and after it: the integrated yaw
 *    change must equal the heading change. Its gradient h (9 values) is
 *    accumulated along the turn.
 *
 * The solve is MAP with a zero-mean prior on dM. A rotation about gravity
 * is invisible to (1), and (2) only sees what projects on the vertical, so
 * unseen directions keep the prior.
 *
 * Plain C with no Pico SDK headers, so it can be built and tested on a host.
 */
#ifndef IMU_CALIB_ROT_H
#define IMU_CALIB_ROT_H

#include <stdbool.h>
#include <stdint.h>

#define ROT_NPAR 9

typedef struct {
    double dr[9];           /* integrated rotation, row-major, body(t0) <- body(t) */
    double j[3 * ROT_NPAR]; /* 3x9 Jacobian, row-major                     */
    float t_s;              /* integrated time                             */
} rot_preint_t;

/* Rotation angle of p->dr [rad]. */
float rot_preint_angle(const rot_preint_t *p);

void rot_preint_reset(rot_preint_t *p);
/* One gyro sample: u = raw - b [rad/s], m0 the matrix in use (row-major,
 * NULL = identity), dt [s]. */
void rot_preint_step(rot_preint_t *p, const float u[3], const float *m0, float dt);

/* Sufficient statistics. Stored in flash as is. */
typedef struct {
    double h[45];          /* upper triangle of sum H^T W H, row by row     */
    double g[ROT_NPAR];    /* sum H^T W r0                                  */
    double w;              /* sum of weights (for the residual)             */
    double rr;             /* sum r0^T W r0                                 */
    uint32_t n_rot;
    uint32_t n_turn;
} rot_stats_t;

void rot_stats_clear(rot_stats_t *s);
const char *rot_stats_validate(const rot_stats_t *s);

/* A rotation from pose 1 to pose 2: g1, g2 the gravity directions (any
 * length, normalised here) in the body frame, sd the sigma per component
 * of the direction residual [rad]. Returns false for bad input. */
bool rot_add_rotation(rot_stats_t *s, const rot_preint_t *p, const float g1[3], const float g2[3],
                      float sd);

/* A turn: pred the yaw change integrated with M0, meas the GNSS heading
 * change [rad], h the gradient of pred with respect to dM, sd [rad]. */
bool rot_add_turn(rot_stats_t *s, float pred, float meas, const float h[ROT_NPAR], float sd);

typedef struct {
    float dm[ROT_NPAR];    /* posterior mean of dM                          */
    float sd[ROT_NPAR];    /* posterior sigma                               */
    bool  ok;
} rot_result_t;

/* sd_scale: prior sigma of the diagonal, sd_mis of the off-diagonal. */
void rot_solve(const rot_stats_t *s, float sd_scale, float sd_mis, rot_result_t *r);

/* M = M0 + dM where the posterior sigma is below the threshold (diagonal:
 * max_sd_scale, off-diagonal: max_sd_mis), M0 elsewhere. m0 NULL =
 * identity. Returns the number of elements taken. */
int rot_accept(const rot_result_t *r, const float *m0, float max_sd_scale, float max_sd_mis,
               float m_out[9]);

/* NULL if every M - I element is within the limits, else a message. */
const char *rot_validate(const float m[9], float max_scale_dev, float max_mis_rad);

/* Small helpers, also used by the turn integration. */
void rot_exp(const float phi[3], float r[9]);

#endif /* IMU_CALIB_ROT_H */
