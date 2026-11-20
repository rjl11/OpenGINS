/*
 * flow_nav: turns the Pi's optical-flow messages into a ground speed for
 * INSLIB (plans/optical-flow-aiding.md, "Split of work"). Core 0, plain C
 * with no SDK calls, so it runs in the host test
 * (firmware/tools/test_flow_nav.c).
 *
 * Inputs, all with Pico timestamps:
 *  - every IMU sample's gyro (body FRD, calibrated), for the rotation;
 *  - the trigger, gap and strobe events from flow_io;
 *  - the attitude from core 1 (roll and pitch tilt the line of sight);
 *  - each FLOW_MSG_FLOW message from the Pi.
 *
 * Frame matching: the trigger leaves out every FLOW_LINK_GAP_PERIOD-th
 * pulse (periods p with p % FLOW_LINK_GAP_PERIOD == 0). The Pi flags the
 * first frame after it (FLOW_F_AFTER_GAP); that frame is the pulse right
 * after the latest gap, and later frames count on from there by the Pi's
 * frame_seq. FLOW_F_DROPPED (a frame lost on the Pi) breaks the count until
 * the next gap.
 *
 * Free-running camera (no FLOW_F_TRIGGERED: no trigger or strobe wired, or
 * the link on USB for bench tests): the frame is timed by the Pi's own
 * sensor clock instead. The offset between the two clocks is the smallest
 * arrival time minus pi_time_us seen (so arrival jitter drops out), creeping
 * up by FLOW_NAV_FREE_LEAK_PPM to follow drift; the frame time is its
 * arrival time less the jitter above that minimum and free_latency_us, and
 * the interval is pi_dt_us. The constant latency is a guess until it is
 * measured, and an error in it only shifts the gyro de-rotation in time.
 *
 * Clock sync (the front camera unit over UDP, app/clock_sync.h): while the
 * clock callback set with flow_nav_set_clock() can map pi_time_us onto the
 * Pico clock, a free-running frame is timed by that instead, plus
 * sync_ts_to_mid_us (the unit's timestamp to the exposure middle), and the
 * arrival-time guess above is not used. Counted in synced.
 *
 * Exposure time of a frame: the middle of its strobe pulse (falling edge =
 * start) if the strobe is wired, else the trigger edge + exposure_delay_us +
 * half the exposure time from the message.
 *
 * Velocity from one message (camera axes x right, y down in the image, z
 * along the optical axis to the ground; flow measured near the image centre):
 *     dx = -(vx / Z) dt - dth_y          dy = -(vy / Z) dt + dth_x
 * with dth the rotation of the camera over the same interval (gyro) and Z
 * the distance to the ground along the optical axis. So
 *     vx = -Z (dx + dth_y) / dt          vy = -Z (dy - dth_x) / dt
 * The signs follow from the pinhole model; check them on the bench (a pure
 * rotation over texture must give zero speed).
 *
 * The camera looks straight down along body z. cam_yaw_rad turns the image
 * about the optical axis: 0 = image top towards body forward. The velocity
 * is moved from the camera to the IMU point with the lever arm, then the
 * horizontal body speed is averaged over fuse_period_s and handed out as a
 * scalar speed sample (INSLIB's speed input has no lever arm and no
 * direction).
 */
#ifndef FLOW_NAV_H
#define FLOW_NAV_H

#include <stdbool.h>
#include <stdint.h>

#include "flow_io.h"
#include "flow_link.h"

#define FLOW_NAV_GYRO_RING 512u /* 640 ms at 800 Hz */
#define FLOW_NAV_EXP_RING  64u  /* trigger periods kept, 640 ms at 100 Hz */

/* Free-running timing (above): the offset minimum creeps up this fast to
 * follow the Pi's clock against the Pico's, and starts over when a frame
 * comes this much later than the minimum (Pi rebooted, link stalled). */
#define FLOW_NAV_FREE_LEAK_PPM   200u
#define FLOW_NAV_FREE_MAX_LAG_US 200000u

typedef struct {
    float    height_m;          /* camera lens above the ground, level        */
    float    height_rel_sd;     /* 1-sigma of the height as a fraction        */
    float    cam_yaw_rad;       /* image top from body forward, about +z      */
    float    leverarm_m[3];     /* camera from the IMU, body FRD              */
    uint32_t exposure_delay_us; /* trigger edge to exposure start (no strobe) */
    uint32_t strobe_window_us;  /* a strobe edge this soon after a trigger belongs to it */
    float    fuse_period_s;     /* averaging window = output period           */
    float    min_sd_mps;        /* floor of the speed 1-sigma                 */
    float    max_mps;           /* faster: rejected                           */
    float    max_tilt_rad;      /* steeper roll or pitch: rejected            */
    uint16_t min_features;
    uint32_t free_latency_us;   /* free-running: exposure middle to the earliest arrival */
    int32_t  sync_ts_to_mid_us; /* clock-synced: unit timestamp to exposure middle */
} flow_nav_config_t;

/* Maps the unit's low 32 bits of time [us] onto the Pico clock at now;
 * false while there is no valid sync. */
typedef bool (*flow_nav_clock_fn)(void *ctx, uint32_t unit_us, uint64_t now, uint64_t *pico_us);

typedef struct {
    uint64_t t_valid_us; /* mean exposure time of the frames averaged      */
    float    speed_mps;  /* horizontal body speed at the IMU, >= 0          */
    float    sd_mps;
    float    v_b[2];     /* the vector it came from, body x / y [m/s]       */
    uint16_t n_frames;
} flow_nav_speed_t;

typedef struct {
    uint32_t msgs;          /* FLOW_MSG_FLOW messages seen                   */
    uint32_t used;          /* went into a speed sample                      */
    uint32_t untriggered;   /* camera free-running: timed by the Pi's clock  */
    uint32_t free_resets;   /* free-running clock offset started over        */
    uint32_t synced;        /* free-running, timed by the clock sync         */
    uint32_t not_anchored;  /* no gap seen yet, or the count was broken      */
    uint32_t invalid;       /* Pi said not valid, or too few features        */
    uint32_t no_exposure;   /* trigger/strobe of the frame not in the ring   */
    uint32_t no_gyro;       /* gyro doesn't cover the interval               */
    uint32_t bad_dt;
    uint32_t too_fast;
    uint32_t too_tilted;
    uint32_t anchors;       /* AFTER_GAP frames used to (re)anchor           */
    uint32_t resyncs;       /* re-anchor disagreed with the running count    */
    uint32_t breaks;        /* FLOW_F_DROPPED seen                           */
    uint32_t strobe_timed;  /* frames timed by the strobe (others: trigger)  */
    uint32_t samples;       /* speed samples handed out                      */
} flow_nav_stats_t;

typedef struct {
    uint64_t t;
    float    g[3];
} flow_nav_gyro_t;

typedef struct {
    uint32_t period;
    uint64_t t_trig, t_fall, t_rise;
} flow_nav_exp_t;

typedef struct {
    flow_nav_config_t cfg;
    float             r_cb[9]; /* camera -> body, row-major */

    flow_nav_gyro_t gyro[FLOW_NAV_GYRO_RING];
    uint32_t        gyro_n; /* samples written */

    flow_nav_exp_t exp[FLOW_NAV_EXP_RING];
    uint32_t       last_trig_period;
    bool           have_gap;
    uint32_t       gap_period;
    uint64_t       gap_t;

    bool     anchored;
    uint32_t anchor_seq;
    uint32_t anchor_ord; /* pulse ordinal of the anchor frame */

    /* free-running clock offset (Pico arrival - Pi time, low 32 bits) */
    bool     free_have;
    uint32_t free_off;
    uint64_t free_leak_t;

    float cos_tilt; /* cos(roll) cos(pitch), 1 until an attitude arrives */
    bool  tilt_ok;

    /* averaging window */
    uint32_t win_n;
    double   win_vx, win_vy, win_var, win_t;
    uint64_t win_t0;

    flow_nav_speed_t last;
    bool             have_last;
    flow_nav_clock_fn clock;
    void             *clock_ctx;

    flow_nav_stats_t st;
} flow_nav_t;

void flow_nav_init(flow_nav_t *s, const flow_nav_config_t *cfg);

/* New camera geometry (web config page): height, image yaw, lever arm.
 * Frames being averaged are dropped; matching and the rings carry on. */
/* The clock sync for free-running frames, or NULL for none. flow_nav_init()
 * clears it. */
void flow_nav_set_clock(flow_nav_t *s, flow_nav_clock_fn fn, void *ctx);

void flow_nav_set_geometry(flow_nav_t *s, float height_m, float cam_yaw_rad, const float leverarm_m[3]);

/* Every IMU sample: its time and the calibrated gyro in body FRD [rad/s]. */
void flow_nav_gyro(flow_nav_t *s, uint64_t t_us, const float gyr_b[3]);

/* Every trigger, gap and strobe event from flow_io. */
void flow_nav_event(flow_nav_t *s, const flow_event_t *e);

/* The latest attitude (core 1 publishes it at 10 Hz). */
void flow_nav_attitude(flow_nav_t *s, bool valid, float roll_rad, float pitch_rad);

/* One message from the Pi. Returns true when a speed sample is complete. */
bool flow_nav_message(flow_nav_t *s, const flow_msg_flow_t *m, uint64_t t_arrival_us, flow_nav_speed_t *out);

/* Pulse ordinal <-> trigger period (periods start at 1; multiples of
 * FLOW_LINK_GAP_PERIOD carry no pulse). Exposed for the test. */
uint32_t flow_nav_period_of(uint32_t ordinal);
uint32_t flow_nav_ordinal_of(uint32_t period);

#endif /* FLOW_NAV_H */
