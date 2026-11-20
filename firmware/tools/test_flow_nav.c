/* Host test for app/flow_nav.c: the trigger/gap/strobe events, the gyro and
 * the Pi's messages are simulated from a camera that moves and rotates over
 * flat ground. The flow in each message is the projection of a ground point
 * (the one under the image centre in the older frame) into both frames, so
 * the signs of the rotation correction are checked against geometry, not
 * against flow_nav's own equations. Build and run on any host with a C
 * compiler:
 *   gcc -O2 -Wall -Wextra -I firmware/app -I firmware/drivers/flow_io -I common \
 *       firmware/app/flow_nav.c firmware/tools/test_flow_nav.c -o /tmp/t -lm && /tmp/t
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "flow_nav.h"

static int fails;
#define CHECK(c)                                                         \
    do {                                                                 \
        if (!(c)) {                                                      \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c);          \
            fails++;                                                     \
        }                                                                \
    } while (0)
#define CHECK_NEAR(a, b, tol)                                                                          \
    do {                                                                                               \
        double a_ = (a), b_ = (b);                                                                     \
        if (!(fabs(a_ - b_) <= (tol))) {                                                               \
            printf("FAIL %s:%d: %s = %.5f, want %.5f +- %.3g\n", __FILE__, __LINE__, #a, a_, b_, (double)(tol)); \
            fails++;                                                                                   \
        }                                                                                              \
    } while (0)

/* ---- scenario ----------------------------------------------------------- */

typedef struct {
    const char *name;
    double      v_b[3];      /* body velocity of the IMU, constant [m/s]   */
    double      w_b[3];      /* body rate, one axis at a time [rad/s]      */
    double      roll0;       /* constant roll offset [rad]                 */
    double      leverarm[3]; /* camera from the IMU, body FRD              */
    double      cam_yaw;     /* rad                                        */
    double      height;      /* camera above ground when level             */
    int         strobe;      /* strobe wired                               */
    int         triggered;
    int         jitter;      /* arrival jitter on the link                 */
    int         drop_at;     /* ordinal whose message carries FLOW_F_DROPPED, 0 = none */
    uint32_t    p_first, p_last;
} scen_t;

#define PERIOD_US 10000u
#define T0_US     1000000ull
#define STROBE_US 50u
#define EXP_US    1000u
#define LAT_US    20000u /* exposure to message arrival */

static uint64_t t_trig(uint32_t p) { return T0_US + (uint64_t)p * PERIOD_US; }
static uint64_t t_mid(uint32_t p) { return t_trig(p) + STROBE_US + EXP_US / 2u; }

/* Motion starts at the run's first period (angles and position are 0 there). */
static double s_t_ref;
static double since_ref(uint64_t t_us) { return (double)(t_us - T0_US) * 1e-6 - s_t_ref; }

/* Rotation body -> nav for single-axis motion: Rz(yaw) Ry(pitch) Rx(roll). */
static void r_nb(const scen_t *s, double t, double R[9])
{
    double r = s->roll0 + s->w_b[0] * t, p = s->w_b[1] * t, y = s->w_b[2] * t;
    double cr = cos(r), sr = sin(r), cp = cos(p), sp = sin(p), cy = cos(y), sy = sin(y);
    double m[9] = { cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
                    sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
                    -sp,     cp * sr,                cp * cr };
    memcpy(R, m, sizeof m);
}

static void mul(const double R[9], const double v[3], double o[3])
{
    for (int i = 0; i < 3; i++) o[i] = R[i * 3] * v[0] + R[i * 3 + 1] * v[1] + R[i * 3 + 2] * v[2];
}
static void mul_t(const double R[9], const double v[3], double o[3])
{
    for (int i = 0; i < 3; i++) o[i] = R[i] * v[0] + R[3 + i] * v[1] + R[6 + i] * v[2];
}

/* IMU position in nav [m]; ground is the plane z = height (NED, down). The
 * velocity is fixed in the body frame, so with a yaw rate the path is an arc. */
static void imu_pos(const scen_t *s, double t, double p[3])
{
    double wz = s->w_b[2];
    if (fabs(wz) < 1e-9) {
        double R[9];
        r_nb(s, 0, R);
        double v[3];
        mul(R, s->v_b, v);
        for (int i = 0; i < 3; i++) p[i] = v[i] * t;
        return;
    }
    double vx = s->v_b[0], vy = s->v_b[1], a = wz * t;
    p[0] = (vx * sin(a) - vy * (1 - cos(a))) / wz;
    p[1] = (vx * (1 - cos(a)) + vy * sin(a)) / wz;
    p[2] = 0;
}

static void cam_pose(const scen_t *s, const float r_cb[9], double t, double pos[3], double R_nc[9])
{
    double R[9];
    r_nb(s, t, R);
    double la[3];
    mul(R, s->leverarm, la);
    imu_pos(s, t, pos);
    for (int i = 0; i < 3; i++) pos[i] += la[i];
    double rcb[9];
    for (int i = 0; i < 9; i++) rcb[i] = r_cb[i];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            double a = 0;
            for (int k = 0; k < 3; k++) a += R[i * 3 + k] * rcb[k * 3 + j];
            R_nc[i * 3 + j] = a;
        }
}

/* Flow between the exposures of periods pa and pb, as the Pi measures it
 * near the image centre: normalised displacement of the ground point seen
 * at the centre in frame a. */
static void flow_between(const scen_t *s, const float r_cb[9], uint32_t pa, uint32_t pb, float d[2])
{
    double ta = since_ref(t_mid(pa)), tb = since_ref(t_mid(pb));
    double pa_n[3], Ra[9], pb_n[3], Rb[9];
    cam_pose(s, r_cb, ta, pa_n, Ra);
    cam_pose(s, r_cb, tb, pb_n, Rb);
    double axis[3] = { 0, 0, 1 }, ray[3];
    mul(Ra, axis, ray);
    double k = (s->height - pa_n[2]) / ray[2];
    double g[3] = { pa_n[0] + k * ray[0], pa_n[1] + k * ray[1], pa_n[2] + k * ray[2] };
    double rel[3] = { g[0] - pb_n[0], g[1] - pb_n[1], g[2] - pb_n[2] }, pc[3];
    mul_t(Rb, rel, pc);
    d[0] = (float)(pc[0] / pc[2]);
    d[1] = (float)(pc[1] / pc[2]);
}

typedef struct {
    int      n;
    double   sum, min, max;
    double   vx, vy; /* last */
    flow_nav_t fn;
} result_t;

static void run(const scen_t *s, result_t *res)
{
    flow_nav_config_t c = {
        .height_m = (float)s->height, .height_rel_sd = 0.05f, .cam_yaw_rad = (float)s->cam_yaw,
        .leverarm_m = { (float)s->leverarm[0], (float)s->leverarm[1], (float)s->leverarm[2] },
        .exposure_delay_us = STROBE_US, .strobe_window_us = 5000u, .fuse_period_s = 0.1f,
        .min_sd_mps = 0.05f, .max_mps = 20.0f, .max_tilt_rad = 0.6f, .min_features = 15,
        .free_latency_us = LAT_US,
    };
    memset(res, 0, sizeof *res);
    s_t_ref = (double)(t_trig(s->p_first) - T0_US) * 1e-6;
    res->min = 1e9;
    res->max = -1e9;
    flow_nav_t *fn = &res->fn;
    flow_nav_init(fn, &c);

    /* Step through time in 250 us ticks, feeding what is due. */
    uint64_t t_end = t_trig(s->p_last) + 2 * LAT_US + PERIOD_US;
    uint32_t next_p = s->p_first;
    uint64_t next_gyro = t_trig(s->p_first) - 200000u;
    uint64_t next_att = next_gyro;
    uint32_t ord_first = flow_nav_ordinal_of(s->p_first);
    for (uint64_t t = next_gyro; t <= t_end; t += 250u) {
        while (next_gyro <= t) {
            float g[3] = { (float)s->w_b[0], (float)s->w_b[1], (float)s->w_b[2] };
            flow_nav_gyro(fn, next_gyro, g);
            next_gyro += 1250u;
        }
        if (next_att <= t) {
            double tt = since_ref(next_att);
            flow_nav_attitude(fn, true, (float)(s->roll0 + s->w_b[0] * tt), (float)(s->w_b[1] * tt));
            next_att += 100000u;
        }
        while (next_p <= s->p_last && t_trig(next_p) <= t) {
            uint32_t p = next_p++;
            flow_event_t e = { .t_us = t_trig(p), .seq = p };
            if (p % FLOW_LINK_GAP_PERIOD == 0) {
                e.kind = FLOW_EV_GAP;
                flow_nav_event(fn, &e);
                continue;
            }
            e.kind = FLOW_EV_TRIGGER;
            e.level = 1;
            flow_nav_event(fn, &e);
            if (s->strobe) {
                flow_event_t f = { .t_us = t_trig(p) + STROBE_US, .kind = FLOW_EV_STROBE, .level = 0 };
                flow_nav_event(fn, &f);
                f.t_us += EXP_US;
                f.level = 1;
                flow_nav_event(fn, &f);
            }
        }
        /* Messages whose arrival time is in this tick. */
        for (uint32_t ord = ord_first + 1u; ; ord++) {
            uint32_t pb = flow_nav_period_of(ord);
            if (pb > s->p_last) break;
            /* Arrival jitter (free-running test): 0 .. 2.4 ms, minimum every 9th frame. */
            uint64_t arr = t_mid(pb) + LAT_US + (s->jitter ? (uint64_t)((ord * 5u) % 9u) * 300u : 0u);
            if (arr > t) break;
            if (arr <= t - 250u) continue;
            uint32_t pa = flow_nav_period_of(ord - 1u);
            flow_msg_flow_t m = { 0 };
            m.frame_seq = ord + 7000u; /* the Pi's own count, any offset */
            /* The Pi's clock: any offset, wrapping its low 32 bits mid-run. */
            m.pi_time_us = (uint32_t)(t_mid(pb) - t_mid(s->p_first) + 0xFFFE0000u);
            m.pi_dt_us = (uint32_t)(t_mid(pb) - t_mid(pa));
            float d[2];
            flow_between(s, fn->r_cb, pa, pb, d);
            m.disp_x_rad = d[0];
            m.disp_y_rad = d[1];
            m.stddev_rad = 1e-4f;
            m.n_tracked = 50;
            m.exposure_100us = EXP_US / 100u;
            m.flags = FLOW_F_VALID;
            if (s->triggered) m.flags |= FLOW_F_TRIGGERED;
            if (pb - pa == 2u) m.flags |= FLOW_F_AFTER_GAP;
            if ((int)ord == s->drop_at) m.flags |= FLOW_F_DROPPED;
            flow_nav_speed_t sp;
            if (flow_nav_message(fn, &m, arr, &sp)) {
                res->n++;
                res->sum += sp.speed_mps;
                if (sp.speed_mps < res->min) res->min = sp.speed_mps;
                if (sp.speed_mps > res->max) res->max = sp.speed_mps;
                res->vx = sp.v_b[0];
                res->vy = sp.v_b[1];
                CHECK(sp.t_valid_us < arr && arr - sp.t_valid_us < 150000u);
                CHECK(sp.n_frames >= 9 && sp.n_frames <= 11);
            }
        }
    }
    printf("%-34s samples %2d  speed %.4f .. %.4f  last body %+.4f %+.4f  used %lu/%lu strobe %lu anchors %lu\n",
           s->name, res->n, res->n ? res->min : 0.0, res->n ? res->max : 0.0, res->vx, res->vy,
           (unsigned long)fn->st.used, (unsigned long)fn->st.msgs, (unsigned long)fn->st.strobe_timed,
           (unsigned long)fn->st.anchors);
}

static scen_t base(const char *name)
{
    scen_t s = { 0 };
    s.name = name;
    s.height = 1.0;
    s.strobe = 1;
    s.triggered = 1;
    s.p_first = 200;
    s.p_last = 330; /* the gap at 256 anchors; about 7 samples after it */
    return s;
}

int main(void)
{
    for (uint32_t k = 1; k < 3000; k++) {
        uint32_t p = flow_nav_period_of(k);
        CHECK(p % FLOW_LINK_GAP_PERIOD != 0);
        CHECK(flow_nav_ordinal_of(p) == k);
        CHECK(flow_nav_period_of(k + 1) == p + 1 + ((p + 1) % FLOW_LINK_GAP_PERIOD == 0));
    }

    result_t r;
    scen_t s;

    s = base("forward 2 m/s");
    s.v_b[0] = 2.0;
    run(&s, &r);
    CHECK(r.n >= 6);
    CHECK_NEAR(r.min, 2.0, 0.01);
    CHECK_NEAR(r.max, 2.0, 0.01);
    CHECK_NEAR(r.vx, 2.0, 0.01);
    CHECK_NEAR(r.vy, 0.0, 0.01);
    CHECK(r.fn.st.not_anchored > 0); /* before the gap */
    CHECK(r.fn.st.anchors == 1);

    s = base("right 1 m/s");
    s.v_b[1] = 1.0;
    run(&s, &r);
    CHECK_NEAR(r.vx, 0.0, 0.01);
    CHECK_NEAR(r.vy, 1.0, 0.01);

    s = base("roll 0.2 rad/s, standing");
    s.w_b[0] = 0.2;
    run(&s, &r);
    CHECK(r.n >= 6);
    CHECK(r.max < 0.02); /* a wrong sign would read 0.4 m/s */

    s = base("pitch 0.2 rad/s, standing");
    s.w_b[1] = 0.2;
    run(&s, &r);
    CHECK(r.n >= 6);
    CHECK(r.max < 0.02);

    s = base("yaw 0.5 rad/s, 2 m/s, cam 1 m ahead");
    s.v_b[0] = 2.0;
    s.w_b[2] = 0.5;
    s.leverarm[0] = 1.0;
    run(&s, &r);
    CHECK_NEAR(r.min, 2.0, 0.02);
    CHECK_NEAR(r.max, 2.0, 0.02);

    s = base("camera turned 90 deg");
    s.v_b[0] = 2.0;
    s.cam_yaw = 1.5707963;
    run(&s, &r);
    CHECK_NEAR(r.vx, 2.0, 0.01);
    CHECK_NEAR(r.vy, 0.0, 0.01);

    s = base("roll 0.3 rad held, 2 m/s");
    s.v_b[0] = 2.0;
    s.roll0 = 0.3;
    run(&s, &r);
    CHECK_NEAR(r.min, 2.0, 0.02);
    CHECK_NEAR(r.max, 2.0, 0.02);

    s = base("height 2 m, 3 m/s");
    s.v_b[0] = 3.0;
    s.height = 2.0;
    run(&s, &r);
    CHECK_NEAR(r.min, 3.0, 0.015);

    s = base("no strobe (trigger timing)");
    s.v_b[0] = 2.0;
    s.strobe = 0;
    run(&s, &r);
    CHECK_NEAR(r.min, 2.0, 0.01);
    CHECK(r.fn.st.strobe_timed == 0);

    s = base("Pi drops a frame after the gap");
    s.v_b[0] = 2.0;
    s.p_last = 560;
    s.drop_at = (int)flow_nav_ordinal_of(300);
    run(&s, &r);
    CHECK(r.fn.st.breaks == 1);
    CHECK(r.fn.st.anchors == 2);    /* gaps at 256 and 512 */
    CHECK(r.fn.st.not_anchored > 200); /* 300..512 lost */
    CHECK_NEAR(r.min, 2.0, 0.01);

    /* No trigger or strobe wired: timed by the Pi's clock, no gap needed. */
    s = base("free-running, jitter, yaw, cam ahead");
    s.v_b[0] = 2.0;
    s.w_b[2] = 0.5;
    s.leverarm[0] = 1.0;
    s.triggered = 0;
    s.jitter = 1;
    run(&s, &r);
    CHECK(r.n >= 10); /* from the start, not only after the gap */
    CHECK_NEAR(r.min, 2.0, 0.02);
    CHECK_NEAR(r.max, 2.0, 0.02);
    CHECK(r.fn.st.untriggered > 100);
    CHECK(r.fn.st.not_anchored == 0);
    CHECK(r.fn.st.free_resets == 0);

    s = base("free-running, roll 0.2 rad/s standing");
    s.w_b[0] = 0.2;
    s.triggered = 0;
    s.jitter = 1;
    run(&s, &r);
    CHECK(r.n >= 10);
    CHECK(r.max < 0.02);

    printf(fails ? "test_flow_nav: %d FAILED\n" : "test_flow_nav: OK\n", fails);
    return fails ? EXIT_FAILURE : EXIT_SUCCESS;
}
