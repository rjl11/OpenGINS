#include "geoid.h"

#include <math.h>

bool geoid_height_m(const geoid_grid_t *g, double lat_deg, double lon_deg, float *n_m)
{
    double y = (lat_deg - g->lat0_deg) / g->dlat_deg;
    double x = (lon_deg - g->lon0_deg) / g->dlon_deg;
    /* A point on the edge may land a rounding error outside; NaN fails too. */
    const double eps = 1e-9;
    if (!(y >= -eps && x >= -eps && y <= g->rows - 1 + eps && x <= g->cols - 1 + eps)) return false;
    if (y < 0.0) y = 0.0;
    if (x < 0.0) x = 0.0;

    int r = (int)y, c = (int)x;
    if (r > g->rows - 2) r = g->rows - 2;   /* on the north or east edge */
    if (c > g->cols - 2) c = g->cols - 2;
    double fy = y - r, fx = x - c;

    const int16_t *p = g->mm + r * g->cols + c;
    double s = (1.0 - fx) * p[0] + fx * p[1];
    double n = (1.0 - fx) * p[g->cols] + fx * p[g->cols + 1];
    *n_m = g->offset_m + (float)(((1.0 - fy) * s + fy * n) * 1e-3);
    return true;
}
