/*
 * Small nonlinear least squares with a prior (MAP), shared by the
 * accelerometer and magnetometer fits (plans/imu-auto-calibration.md).
 *
 * Minimises  sum_j r_j(p)^2 + sum_i ((p_i - p0_i) / sd_i)^2  by Gauss-Newton
 * with step halving, where r_j is already divided by its sigma. A parameter
 * direction the observations don't reach keeps its prior mean and sigma.
 * The posterior sigma comes from the inverse normal matrix at the solution.
 *
 * Plain C with no Pico SDK headers, so it can be built and tested on a host.
 */
#ifndef IMU_CALIB_LSQ_H
#define IMU_CALIB_LSQ_H

#include <stdbool.h>

#define LSQ_MAX_PAR 12

/* Residual j at p, divided by its sigma; fills jac[0..np-1] (also divided)
 * when jac is not NULL. */
typedef double (*lsq_resid_fn)(void *ctx, int j, const double *p, double *jac);

typedef struct {
    int  iterations;
    bool ok;               /* converged with a positive definite normal matrix */
} lsq_info_t;

/* p0: prior mean, sd: prior sigma per parameter, start: first guess (NULL =
 * p0). p_out and sd_out get the posterior mean and sigma. */
void lsq_solve(int np, const double *p0, const double *sd, const double *start, int nobs,
               lsq_resid_fn f, void *ctx, double *p_out, double *sd_out, lsq_info_t *info);

/* Solves the symmetric positive definite n x n system A x = b (A row-major,
 * overwritten); with inv not NULL also returns the diagonal of A^-1.
 * Returns false if A is not positive definite. */
bool lsq_spd_solve(int n, double *a, double *b, double *inv_diag);

/* Coverage of directions: the smallest eigenvalue of the mean of u*u^T over
 * the unit vectors u from centre to the n points at xyz, xyz + stride, ...
 * (stride in floats). 1/3 = spread over the whole sphere, 0 = all in one
 * plane (or one direction). */
float calib_dir_coverage(const float *xyz, int stride, int n, const float centre[3]);

#endif /* IMU_CALIB_LSQ_H */
