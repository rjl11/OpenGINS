/* Host test for firmware/app/geoid.c against the generated grid.
 *
 * Checks the lookup at grid points and cell centres, the edges, outside
 * points, and (with the source grid as an argument) every 7th grid point of
 * the cut against SWEN17_RH2000.txt itself. Build and run from the repo root:
 *
 *   gcc -O2 -Wall -Wextra -I firmware/app firmware/tools/test_geoid.c \
 *       firmware/app/geoid.c firmware/app/geoid_grid.c -lm -o temp/test_geoid
 *   temp/test_geoid geoid/SWEN17_RH2000.txt
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "geoid.h"

static int s_fail;

#define CHECK(c, ...) do { if (!(c)) { printf("FAIL %s:%d: ", __FILE__, __LINE__); \
                                       printf(__VA_ARGS__); printf("\n"); s_fail++; } } while (0)

static float node(const geoid_grid_t *g, int r, int c)
{
    return g->offset_m + g->mm[r * g->cols + c] * 1e-3f;
}

/* The source grid's value at a lat / lon grid point; the file runs north to south. */
static int check_source(const geoid_grid_t *g, const char *path)
{
    FILE *f = fopen(path, "r");
    if (!f) { printf("can't open %s\n", path); return 1; }
    double h[6];
    if (fscanf(f, "%lf %lf %lf %lf %lf %lf", &h[0], &h[1], &h[2], &h[3], &h[4], &h[5]) != 6) return 1;
    int rows = (int)lround((h[1] - h[0]) / h[4]) + 1, cols = (int)lround((h[3] - h[2]) / h[5]) + 1;
    float *v = malloc(sizeof(float) * (size_t)rows * (size_t)cols);
    for (long i = 0; i < (long)rows * cols; i++) {
        if (fscanf(f, "%f", &v[i]) != 1) { printf("short file\n"); return 1; }
    }
    fclose(f);

    int n = 0;
    double worst = 0;
    for (int r = 0; r < g->rows; r += 7) {
        for (int c = 0; c < g->cols; c += 7) {
            double lat = g->lat0_deg + r * g->dlat_deg, lon = g->lon0_deg + c * g->dlon_deg;
            int rs = (int)lround((h[1] - lat) / h[4]), cs = (int)lround((lon - h[2]) / h[5]);
            float want = v[(long)rs * cols + cs], got;
            CHECK(geoid_height_m(g, lat, lon, &got), "no value at %.2f %.2f", lat, lon);
            double e = fabs(got - want);
            if (e > worst) worst = e;
            n++;
        }
    }
    CHECK(worst <= 0.0006, "worst %.4f m against the source", worst);
    printf("%d grid points against %s: worst %.2f mm\n", n, path, worst * 1e3);
    free(v);
    return 0;
}

int main(int argc, char **argv)
{
    const geoid_grid_t *g = &geoid_grid;
    float n;

    /* Grid points give the stored value. */
    for (int r = 0; r < g->rows; r += 13) {
        for (int c = 0; c < g->cols; c += 11) {
            double lat = g->lat0_deg + r * g->dlat_deg, lon = g->lon0_deg + c * g->dlon_deg;
            CHECK(geoid_height_m(g, lat, lon, &n) && fabsf(n - node(g, r, c)) < 1e-4f,
                  "node %d,%d", r, c);
        }
    }

    /* A cell centre is the mean of its four corners. */
    int r = g->rows / 2, c = g->cols / 2;
    float want = 0.25f * (node(g, r, c) + node(g, r, c + 1) + node(g, r + 1, c) + node(g, r + 1, c + 1));
    CHECK(geoid_height_m(g, g->lat0_deg + (r + 0.5) * g->dlat_deg, g->lon0_deg + (c + 0.5) * g->dlon_deg, &n) &&
          fabsf(n - want) < 1e-4f, "cell centre %.4f, want %.4f", n, want);

    /* North-east corner is inside; just past any edge, or NaN, is outside. */
    double lat1 = g->lat0_deg + (g->rows - 1) * g->dlat_deg, lon1 = g->lon0_deg + (g->cols - 1) * g->dlon_deg;
    CHECK(geoid_height_m(g, lat1, lon1, &n) && fabsf(n - node(g, g->rows - 1, g->cols - 1)) < 1e-4f, "NE corner");
    CHECK(!geoid_height_m(g, g->lat0_deg - 1e-6, g->lon0_deg + 0.1, &n), "south of the grid");
    CHECK(!geoid_height_m(g, lat1 + 1e-6, g->lon0_deg + 0.1, &n), "north of the grid");
    CHECK(!geoid_height_m(g, g->lat0_deg + 0.1, g->lon0_deg - 1e-6, &n), "west of the grid");
    CHECK(!geoid_height_m(g, g->lat0_deg + 0.1, lon1 + 1e-6, &n), "east of the grid");
    CHECK(!geoid_height_m(g, NAN, g->lon0_deg + 0.1, &n), "NaN latitude");

    if (argc > 1 && check_source(g, argv[1])) s_fail++;

    printf("%s: %s grid %d x %d\n", s_fail ? "FAILED" : "ok", g->name, g->rows, g->cols);
    return s_fail != 0;
}
