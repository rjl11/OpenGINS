#include "ground_point.h"

#include <math.h>
#include <stddef.h>

#define WGS84_A   6378137.0
#define WGS84_E2  6.69437999014e-3

void ground_point(const double llh_in[3], const double *rpy, const float off_b[3], double llh_out[3])
{
    double n, e, d;
    if (rpy != NULL) {
        double cr = cos(rpy[0]), sr = sin(rpy[0]);
        double cp = cos(rpy[1]), sp = sin(rpy[1]);
        double cy = cos(rpy[2]), sy = sin(rpy[2]);
        double x = off_b[0], y = off_b[1], z = off_b[2];
        n = cp * cy * x + (sr * sp * cy - cr * sy) * y + (cr * sp * cy + sr * sy) * z;
        e = cp * sy * x + (sr * sp * sy + cr * cy) * y + (cr * sp * sy - sr * cy) * z;
        d = -sp * x + sr * cp * y + cr * cp * z;
    } else {
        n = 0.0;
        e = 0.0;
        d = off_b[2];
    }
    double lat = llh_in[0], h = llh_in[2];
    double sl = sin(lat);
    double w = sqrt(1.0 - WGS84_E2 * sl * sl);
    double rn = WGS84_A / w;                          /* prime vertical */
    double rm = WGS84_A * (1.0 - WGS84_E2) / (w * w * w); /* meridian */
    llh_out[0] = lat + n / (rm + h);
    llh_out[1] = llh_in[1] + e / ((rn + h) * cos(lat));
    llh_out[2] = h - d;
}

double ground_tilt(const double rpy[3])
{
    double c = cos(rpy[0]) * cos(rpy[1]);
    return acos(c > 1.0 ? 1.0 : (c < -1.0 ? -1.0 : c));
}

double ground_axis_sd(const double rpy[3], const float sd[3])
{
    double sy = sin(ground_tilt(rpy)) * sd[2];
    return sqrt((double)sd[0] * sd[0] + (double)sd[1] * sd[1] + sy * sy);
}
