/* Ground point below the GNSS antenna (tractor, survey pole).
 *
 * Moves a WGS84 position by an offset given in the body frame (FRD), rotated
 * to north-east-down with the attitude: the antenna height along the body
 * down axis, plus the lever arm when the position is the IMU point (INSLIB's
 * output). Used by best_position() in main.c when the web setting
 * ant_height_m is above 0. Plain host C, core 0 only; host test
 * firmware/tools/test_ground_point.c. */
#ifndef GROUND_POINT_H
#define GROUND_POINT_H

#include <stdbool.h>

/* llh_in / llh_out: lat, lon [rad], ellipsoid height [m]. rpy: roll, pitch,
 * yaw [rad], ZYX, body FRD to NED. Without an attitude (rpy NULL) only the
 * down part of the offset is applied, straight down. The offset is small
 * (metres), so the move uses the local radii of curvature. */
void ground_point(const double llh_in[3], const double *rpy, const float off_b[3], double llh_out[3]);

/* Tilt of the body down axis from the vertical [rad]. */
double ground_tilt(const double rpy[3]);

/* 1-sigma of the body down axis's direction [rad], from the attitude
 * 1-sigma sd (roll, pitch, yaw). Roll and pitch errors move the axis
 * directly; a yaw error only moves it as far as the axis leans, by
 * sin(tilt). The ground point's horizontal error is about the offset
 * length times this. */
double ground_axis_sd(const double rpy[3], const float sd[3]);

#endif
