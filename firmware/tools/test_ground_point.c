/* Host test for firmware/app/ground_point.c. Build and run from the repo root:
 *
 *   gcc -O2 -Wall -Wextra -I firmware/app firmware/tools/test_ground_point.c \
 *       firmware/app/ground_point.c -lm -o temp/test_ground_point
 *   temp/test_ground_point
 */
#include <math.h>
#include <stdio.h>

#include "ground_point.h"

static int s_fail;

#define CHECK(c, ...) do { if (!(c)) { printf("FAIL %s:%d: ", __FILE__, __LINE__); \
                                       printf(__VA_ARGS__); printf("\n"); s_fail++; } } while (0)

#define D2R (3.14159265358979323846 / 180.0)

/* Metres north, east, up of b from a (local, small offsets). */
static void diff_m(const double a[3], const double b[3], double neu[3])
{
    const double r = 6378137.0;
    neu[0] = (b[0] - a[0]) * r;
    neu[1] = (b[1] - a[1]) * r * cos(a[0]);
    neu[2] = b[2] - a[2];
}

static void expect(const char *name, const double *rpy, const float off[3], double n, double e, double u)
{
    const double p0[3] = { 59.0 * D2R, 17.0 * D2R, 40.0 };
    double p1[3], neu[3];
    ground_point(p0, rpy, off, p1);
    diff_m(p0, p1, neu);
    /* 1 cm: the test's spherical radius against the WGS84 radii (~0.5 %). */
    CHECK(fabs(neu[0] - n) < 0.01 && fabs(neu[1] - e) < 0.01 && fabs(neu[2] - u) < 1e-9,
          "%s: got N %.4f E %.4f U %.4f, want %.4f %.4f %.4f", name, neu[0], neu[1], neu[2], n, e, u);
}

int main(void)
{
    const float down2[3] = { 0.0f, 0.0f, 2.0f };
    const double level[3] = { 0, 0, 0 };
    expect("level, no attitude", NULL, down2, 0, 0, -2);
    expect("level", level, down2, 0, 0, -2);

    /* Right side down 90 deg: the body down axis points to the left (west
     * at yaw 0). */
    const double roll90[3] = { 90 * D2R, 0, 0 };
    expect("roll 90", roll90, down2, 0, -2, 0);

    /* Nose up 30 deg: down axis tilts forward (north at yaw 0). */
    const double pitch30[3] = { 0, 30 * D2R, 0 };
    expect("pitch 30", pitch30, down2, 2 * sin(30 * D2R), 0, -2 * cos(30 * D2R));

    /* Yaw 90 (facing east): a lever arm 1 m forward goes east. */
    const float fwd1_down2[3] = { 1.0f, 0.0f, 2.0f };
    const double yaw90[3] = { 0, 0, 90 * D2R };
    expect("yaw 90 + lever", yaw90, fwd1_down2, 0, 1, -2);

    /* Tractor on a 5 deg side slope, 3 m antenna: ground point 26 cm left. */
    const float down3[3] = { 0.0f, 0.0f, 3.0f };
    const double roll5[3] = { 5 * D2R, 0, 0 };
    expect("roll 5, 3 m", roll5, down3, 0, -3 * sin(5 * D2R), -3 * cos(5 * D2R));

    /* Axis sigma: level, yaw doesn't count; tilted 30 deg, half of it. */
    const float sd[3] = { 0.01f, 0.01f, 0.2f };
    double ax = ground_axis_sd(level, sd);
    CHECK(fabs(ax - sqrt(2.0) * 0.01) < 1e-6, "axis sd level %.6f", ax);
    const double roll30[3] = { 30 * D2R, 0, 1.0 };
    CHECK(fabs(ground_tilt(roll30) - 30 * D2R) < 1e-9, "tilt %.6f", ground_tilt(roll30));
    ax = ground_axis_sd(roll30, sd);
    CHECK(fabs(ax - sqrt(2e-4 + 0.01)) < 1e-6, "axis sd roll 30 %.6f", ax);
    const double rp[3] = { 20 * D2R, 20 * D2R, 0 };
    double t = ground_tilt(rp);
    CHECK(fabs(cos(t) - cos(20 * D2R) * cos(20 * D2R)) < 1e-9, "tilt rp %.6f", t);

    /* Cab roll: antenna 3 m up, roll centre 1 m up. The cab rolled 2 deg on
     * its suspension over a level chassis: the ground point moves by the 2 m
     * above the roll centre only, not by all 3 m. */
    {
        const double p0[3] = { 59.0 * D2R, 17.0 * D2R, 40.0 };
        const double cab2[3] = { 2 * D2R, 0, 0 };
        double p1[3], neu[3];
        ground_point_cab(p0, cab2, 0.0, 0.0, 3.0f, 1.0f, p1);
        diff_m(p0, p1, neu);
        CHECK(fabs(neu[1] + 2 * sin(2 * D2R)) < 0.002 && fabs(neu[0]) < 0.002 &&
                  fabs(neu[2] + 2 * cos(2 * D2R) + 1) < 1e-6,
              "cab 2 deg: got N %.4f E %.4f U %.4f", neu[0], neu[1], neu[2]);
        /* Cab and chassis on the same 5 deg slope (no sway): the same point
         * as one roll over the whole height. */
        const double cab5[3] = { 5 * D2R, 0, 0.7 };
        double q1[3], q2[3];
        ground_point_cab(p0, cab5, 5 * D2R, 0.0, 3.0f, 1.0f, q1);
        const float down3b[3] = { 0.0f, 0.0f, 3.0f };
        ground_point(p0, cab5, down3b, q2);
        diff_m(q1, q2, neu);
        CHECK(fabs(neu[0]) < 1e-6 && fabs(neu[1]) < 1e-6 && fabs(neu[2]) < 1e-9,
              "cab = chassis: differs by N %.6f E %.6f U %.6f", neu[0], neu[1], neu[2]);
        /* The chassis pitched 1 deg nose up under a level cab: the lower part
         * leans forward (north at yaw 0). */
        ground_point_cab(p0, level, 0.0, 1 * D2R, 3.0f, 1.0f, p1);
        diff_m(p0, p1, neu);
        CHECK(fabs(neu[0] - sin(1 * D2R)) < 0.002 && fabs(neu[1]) < 0.002, "chassis pitch: N %.4f E %.4f", neu[0],
              neu[1]);
    }

    printf(s_fail ? "%d FAILED\n" : "all passed\n", s_fail);
    return s_fail != 0;
}
