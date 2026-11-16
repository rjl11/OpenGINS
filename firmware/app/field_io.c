#include "field_io.h"

#include <stdio.h>
#include <string.h>

#include "board_pins.h"
#include "flow_io.h"
#include "flow_nav.h"
#include "nav_core1.h"
#include "pulse_in.h"
#include "speed_ppm.h"
#include "usb_log.h"

/* ---- speed input -------------------------------------------------------- */

/* Quadrature order 00 -> 01 -> 11 -> 10 (A leads B) counts as forward. */
static const int8_t k_gray_idx[4] = { 0, 1, 3, 2 };

typedef struct {
    uint32_t rising[2], edges[2];
    uint32_t fwd, rev, bad; /* quadrature steps */
} speed_win_t;

static speed_win_t s_sw;                 /* current window */
static struct {
    float    hz[2];                      /* rising edges per second */
    float    edge_hz[2];
    uint32_t fwd, rev, bad;
    bool     quad;                       /* B moves in quadrature with A */
    float    gnss_mps;                   /* mean GNSS speed over the last run */
    float    ppm_span_s;                 /* that run, first to last edge */
    bool     ppm_valid;
    float    ppm;                        /* A rising edges per metre, edge-timed (speed_ppm.h) */
    uint32_t overflows;
} s_sp;
static speed_ppm_t s_ppm;
static uint64_t s_ppm_t_us;              /* last result's end edge, 0 = none */
static uint32_t s_ppm_overflows;         /* pulse_in overflows the run has seen */

/* Both inputs start enabled in field_io_init(); the settings turn them off. */
static bool s_flow_on = true, s_wheel_on = true;
static uint8_t s_last_lv;
static bool s_have_lv;

static void speed_edge(const pulse_edge_t *e)
{
    int ch = (e->levels & PULSE_IN_EDGE_B) ? 1 : 0;
    uint8_t lv = e->levels & 3u;
    s_sw.edges[ch]++;
    if (lv & (1u << ch)) {
        s_sw.rising[ch]++;
        if (ch == 0) speed_ppm_edge(&s_ppm, e->t_us);
    }
    if (s_have_lv) {
        int d = (k_gray_idx[lv] - k_gray_idx[s_last_lv]) & 3;
        if (d == 1) s_sw.fwd++;
        else if (d == 3) s_sw.rev++;
        else if (d == 2) s_sw.bad++;
    }
    s_last_lv = lv;
    s_have_lv = true;
    usb_log_speed_edge(e);
}

void field_io_gnss(const um980_nav_t *g)
{
    if (!s_wheel_on) {
        return;
    }
    /* Lost edges break the count: start the run over. */
    pulse_in_stats_t ps;
    pulse_in_stats(&ps);
    if (ps.overflows != s_ppm_overflows) {
        s_ppm_overflows = ps.overflows;
        speed_ppm_reset(&s_ppm);
    }
    speed_ppm_result_t r;
    if (speed_ppm_gnss(&s_ppm, g->t_valid_us, g->hor_speed_mps, g->vel_usable, &r)) {
        s_sp.ppm = r.ppm;
        s_sp.gnss_mps = r.mps;
        s_sp.ppm_span_s = r.span_s;
        s_ppm_t_us = r.t_end_us;
    }
}

/* ---- flow link ---------------------------------------------------------- */

static struct {
    uint32_t triggers, gaps, strobes, frames;
} s_fw;
static struct {
    float triggers_hz, strobes_hz, frames_hz;
    uint32_t gaps;
    bool have_last;
    flow_msg_flow_t last;
} s_fl;
static flow_io_stats_t s_fst;
static bool s_link_ok;

/* Flow to speed (flow_nav.h) and on to core 1. */
static flow_nav_t s_fn;
static struct {
    uint32_t pushed, push_drops; /* this second */
    float    pushed_hz;
    uint32_t to_filter, late, used, skipped; /* core 1's counts (cumulative) */
} s_fs;
static uint32_t s_pushed_win;

/* ---- public ------------------------------------------------------------- */

void field_io_init(void)
{
    const pulse_in_config_t pc = { .pin_a = PIN_SPEED_A, .pin_b = PIN_SPEED_B, .pull_up = APP_SPEED_PULL_UP };
    pulse_in_init(&pc);
    const speed_ppm_config_t sc = {
        .min_mps = APP_SPEED_PPM_MIN_MPS, .min_span_s = APP_SPEED_PPM_MIN_SPAN_S, .min_edges = APP_SPEED_PPM_MIN_EDGES,
    };
    speed_ppm_init(&s_ppm, &sc);

    const flow_io_config_t fc = {
        .rx_pin = PIN_FLOW_RX, .trigger_pin = PIN_FLOW_TRIGGER, .strobe_pin = PIN_FLOW_STROBE,
        .trigger_hz = APP_FLOW_TRIGGER_HZ, .trigger_us = APP_FLOW_TRIGGER_US, .baud = FLOW_LINK_BAUD,
    };
    s_link_ok = flow_io_init(&fc);

    const flow_nav_config_t nc = {
        .height_m = APP_FLOW_HEIGHT_M,
        .height_rel_sd = APP_FLOW_HEIGHT_REL_SD,
        .cam_yaw_rad = APP_FLOW_CAM_YAW_DEG * 0.017453293f,
        .leverarm_m = { APP_FLOW_LEVERARM_X_M, APP_FLOW_LEVERARM_Y_M, APP_FLOW_LEVERARM_Z_M },
        .exposure_delay_us = APP_FLOW_EXPOSURE_DELAY_US,
        .strobe_window_us = APP_FLOW_STROBE_WINDOW_US,
        .fuse_period_s = APP_FLOW_FUSE_PERIOD_S,
        .min_sd_mps = APP_FLOW_MIN_SD_MPS,
        .max_mps = APP_FLOW_MAX_MPS,
        .max_tilt_rad = APP_FLOW_MAX_TILT_DEG * 0.017453293f,
        .min_features = APP_FLOW_MIN_FEATURES,
    };
    flow_nav_init(&s_fn, &nc);
    printf("[field] speed inputs GP%d/GP%d, flow link GP%d %s, trigger GP%d at %u Hz, strobe GP%d\n",
           PIN_SPEED_A, PIN_SPEED_B, PIN_FLOW_RX, s_link_ok ? "on" : "FAILED (no PIO/DMA free)",
           PIN_FLOW_TRIGGER, (unsigned)APP_FLOW_TRIGGER_HZ, PIN_FLOW_STROBE);
}

void field_io_apply_settings(const settings_t *s)
{
    const bool flow = s->sensor_flow != 0, wheel = s->sensor_wheel != 0;
    if (flow != s_flow_on) {
        flow_io_set_enabled(flow);
        if (flow) {
            /* Matching and the rings start over: everything in them is from
             * before the switch. */
            const flow_nav_config_t c = s_fn.cfg;
            flow_nav_init(&s_fn, &c);
        }
        s_flow_on = flow;
    }
    if (wheel != s_wheel_on) {
        pulse_in_set_enabled(wheel);
        speed_ppm_reset(&s_ppm);
        s_ppm_t_us = 0;
        s_wheel_on = wheel;
    }
    flow_nav_set_geometry(&s_fn, s->flow_height_m, s->flow_cam_yaw_deg * 0.017453293f, s->flow_leverarm_m);
}

void field_io_imu(const nav_imu_t *n)
{
    if (s_flow_on) {
        flow_nav_gyro(&s_fn, n->t_us, n->gyr);
    }
}

bool field_io_poll(uint64_t now)
{
    (void)now;
    bool worked = false;

    /* Attitude for the flow's line of sight, when core 1 has published. */
    static uint32_t s_nav_seq;
    uint32_t seq = nav_output_seq();
    if (seq != s_nav_seq) {
        s_nav_seq = seq;
        nav_output_t o;
        nav_get_output(&o);
        flow_nav_attitude(&s_fn, o.rpy_valid, o.rpy_rad[0], o.rpy_rad[1]);
        s_fs.to_filter = o.n_speed[NAV_SPEED_FLOW];
        s_fs.late = o.n_speed_late;
        s_fs.used = o.n_speed_used;
        s_fs.skipped = o.n_speed_skipped;
    }

    /* While an input is off, whatever is still queued from before the switch
     * (or a Pi still sending) is dropped. */
    pulse_edge_t e;
    while (pulse_in_pop(&e)) {
        if (s_wheel_on) speed_edge(&e);
        worked = true;
    }
    flow_event_t ev;
    while (flow_io_pop_event(&ev)) {
        if (!s_flow_on) continue;
        if (ev.kind == FLOW_EV_TRIGGER) s_fw.triggers++;
        else if (ev.kind == FLOW_EV_GAP) s_fw.gaps++;
        else if (ev.level == 0) s_fw.strobes++; /* falling = exposure start */
        flow_nav_event(&s_fn, &ev);
        usb_log_flow_event(&ev);
        worked = true;
    }
    flow_frame_t f;
    while (flow_io_poll_frame(&f)) {
        if (!s_flow_on) continue;
        s_fw.frames++;
        if (f.type == FLOW_MSG_FLOW && flow_link_decode_flow(f.payload, f.len, &s_fl.last) == 0) {
            s_fl.have_last = true;
            flow_nav_speed_t sp;
            if (flow_nav_message(&s_fn, &s_fl.last, f.t_arrival_us, &sp)) {
                s_pushed_win++;
                const nav_speed_t ns = {
                    .t_valid_us = sp.t_valid_us, .speed_mps = sp.speed_mps, .sd_mps = sp.sd_mps,
                    .source = NAV_SPEED_FLOW,
                };
                if (!nav_push_speed(&ns)) s_fs.push_drops++;
            }
        }
        usb_log_flow_frame(&f);
        worked = true;
    }
    return worked;
}

void field_io_roll(uint64_t now)
{
    static uint64_t s_t0;
    float dt = s_t0 ? (float)(now - s_t0) * 1e-6f : 1.0f;
    if (dt <= 0.0f) dt = 1.0f;
    s_t0 = now;

    for (int ch = 0; ch < 2; ch++) {
        s_sp.hz[ch] = (float)s_sw.rising[ch] / dt;
        s_sp.edge_hz[ch] = (float)s_sw.edges[ch] / dt;
    }
    s_sp.fwd = s_sw.fwd;
    s_sp.rev = s_sw.rev;
    s_sp.bad = s_sw.bad;
    s_sp.quad = s_sw.edges[1] > 0 && s_sw.fwd + s_sw.rev > 4u * (s_sw.bad + 1u);
    /* The last edge-timed figure, while it is recent. */
    s_sp.ppm_valid = s_ppm_t_us != 0 && now - s_ppm_t_us < (uint64_t)(APP_SPEED_PPM_HOLD_S * 1e6f);
    pulse_in_stats_t ps;
    pulse_in_stats(&ps);
    s_sp.overflows = ps.overflows;
    memset(&s_sw, 0, sizeof s_sw);

    s_fl.triggers_hz = (float)s_fw.triggers / dt;
    s_fl.strobes_hz = (float)s_fw.strobes / dt;
    s_fl.frames_hz = (float)s_fw.frames / dt;
    s_fl.gaps = s_fw.gaps;
    memset(&s_fw, 0, sizeof s_fw);
    s_fs.pushed_hz = (float)s_pushed_win / dt;
    s_pushed_win = 0;
    flow_io_stats(&s_fst);
}

static void print_speed(void)
{
    if (!s_wheel_on) {
        printf("SPEED off (settings)\n");
        return;
    }
    printf("SPEED A %.1f Hz  B %.1f Hz (rising edges)", (double)s_sp.hz[0], (double)s_sp.hz[1]);
    if (s_sp.quad) {
        printf("  quadrature %s (steps fwd %lu rev %lu bad %lu)", s_sp.fwd >= s_sp.rev ? "forward" : "reverse",
               (unsigned long)s_sp.fwd, (unsigned long)s_sp.rev, (unsigned long)s_sp.bad);
    }
    if (s_sp.ppm_valid) {
        printf("  %.2f pulses/m at GNSS %.2f m/s over %.1f s (raw, no turn correction; ISO 11786: %.0f)",
               (double)s_sp.ppm, (double)s_sp.gnss_mps, (double)s_sp.ppm_span_s, (double)APP_SPEED_ISO11786_PPM);
    }
    if (s_sp.overflows) {
        printf("  edges lost %lu", (unsigned long)s_sp.overflows);
    }
    printf("\n");
}

static void print_flow(void)
{
    if (!s_flow_on) {
        printf("FLOW  off (settings): no trigger, nothing to INSLIB\n");
        return;
    }
    printf("FLOW  trigger %.0f Hz  strobe %.0f Hz  frames %.0f Hz  crc %lu  link %s  camera %.2f m, top %.0f deg,"
           " lever %.2f %.2f %.2f m",
           (double)s_fl.triggers_hz, (double)s_fl.strobes_hz, (double)s_fl.frames_hz,
           (unsigned long)s_fst.crc_errors, s_link_ok ? "on" : "off", (double)s_fn.cfg.height_m,
           (double)s_fn.cfg.cam_yaw_rad * 57.29578, (double)s_fn.cfg.leverarm_m[0], (double)s_fn.cfg.leverarm_m[1],
           (double)s_fn.cfg.leverarm_m[2]);
    if (s_fst.rx_overflows || s_fst.event_overflows) {
        printf("  overflows rx %lu ev %lu", (unsigned long)s_fst.rx_overflows, (unsigned long)s_fst.event_overflows);
    }
    printf("\n");
    if (s_fl.have_last) {
        const flow_msg_flow_t *m = &s_fl.last;
        printf("      last frame %lu  disp %.5f %.5f rad (sd %.5f)  features %u  exposure %.1f ms  flags 0x%02x\n",
               (unsigned long)m->frame_seq, (double)m->disp_x_rad, (double)m->disp_y_rad, (double)m->stddev_rad,
               m->n_tracked, (double)m->exposure_100us * 0.1, m->flags);
    }
    const flow_nav_stats_t *ns = &s_fn.st;
    if (ns->msgs) {
        printf("      speed");
        if (s_fn.have_last) {
            const flow_nav_speed_t *sp = &s_fn.last;
            printf(" %.2f m/s (sd %.2f, body %+.2f %+.2f, %u frames)", (double)sp->speed_mps, (double)sp->sd_mps,
                   (double)sp->v_b[0], (double)sp->v_b[1], sp->n_frames);
        } else {
            printf(" none yet");
        }
        printf("  %.0f Hz to INSLIB  %s  used %lu/%lu  timing strobe %lu", (double)s_fs.pushed_hz,
               s_fn.anchored ? "anchored" : "NOT anchored",
               (unsigned long)ns->used, (unsigned long)ns->msgs, (unsigned long)ns->strobe_timed);
        if (ns->untriggered) printf("  untriggered %lu", (unsigned long)ns->untriggered);
        if (ns->not_anchored) printf("  unanchored %lu", (unsigned long)ns->not_anchored);
        if (ns->invalid) printf("  invalid %lu", (unsigned long)ns->invalid);
        if (ns->no_exposure) printf("  no-exposure %lu", (unsigned long)ns->no_exposure);
        if (ns->no_gyro) printf("  no-gyro %lu", (unsigned long)ns->no_gyro);
        if (ns->bad_dt || ns->too_fast || ns->too_tilted) {
            printf("  rejected dt %lu fast %lu tilt %lu", (unsigned long)ns->bad_dt, (unsigned long)ns->too_fast,
                   (unsigned long)ns->too_tilted);
        }
        if (ns->resyncs || ns->breaks) printf("  resyncs %lu breaks %lu", (unsigned long)ns->resyncs, (unsigned long)ns->breaks);
        printf("\n");
        printf("      INSLIB speed: %lu given, %lu used, %lu skipped, %lu late, %lu queue drops\n",
               (unsigned long)s_fs.to_filter, (unsigned long)s_fs.used, (unsigned long)s_fs.skipped,
               (unsigned long)s_fs.late, (unsigned long)s_fs.push_drops);
    }
}

void field_io_print_status(void)
{
    print_speed();
    print_flow();

    usb_log_status_t ls;
    usb_log_status(&ls);
    if (ls.on) {
        printf("LOG   on  %lu B/s  dropped %lu B in %lu records  ring max %lu B\n", (unsigned long)ls.bytes_per_s,
               (unsigned long)ls.dropped_bytes, (unsigned long)ls.dropped_recs, (unsigned long)ls.ring_max);
    }
}

#if APP_WEB_BUILT
void field_io_json(jw_t *w)
{
    jw_obj(w, "speed");
    jw_bool(w, "enabled", s_wheel_on);
    jw_num(w, "a_hz", (double)s_sp.hz[0], 1);
    jw_num(w, "b_hz", (double)s_sp.hz[1], 1);
    jw_bool(w, "quadrature", s_sp.quad);
    if (s_sp.quad) {
        jw_str(w, "direction", s_sp.fwd >= s_sp.rev ? "forward" : "reverse");
    }
    if (s_sp.ppm_valid) {
        jw_num(w, "pulses_per_m", (double)s_sp.ppm, 2);
        jw_num(w, "gnss_mps", (double)s_sp.gnss_mps, 2);
        jw_num(w, "ppm_span_s", (double)s_sp.ppm_span_s, 1);
    } else {
        jw_null(w, "pulses_per_m");
    }
    jw_int(w, "edges_lost", s_sp.overflows);
    jw_end(w, '}');

    jw_obj(w, "flow");
    jw_bool(w, "enabled", s_flow_on);
    jw_bool(w, "link", s_link_ok);
    jw_num(w, "trigger_hz", (double)s_fl.triggers_hz, 0);
    jw_num(w, "strobe_hz", (double)s_fl.strobes_hz, 0);
    jw_num(w, "frames_hz", (double)s_fl.frames_hz, 0);
    jw_int(w, "crc_errors", s_fst.crc_errors);
    if (s_fl.have_last) {
        jw_int(w, "features", s_fl.last.n_tracked);
        jw_int(w, "flags", s_fl.last.flags);
    }
    const flow_nav_stats_t *ns = &s_fn.st;
    jw_num(w, "camera_height_m", (double)s_fn.cfg.height_m, 3);
    jw_num(w, "camera_yaw_deg", (double)s_fn.cfg.cam_yaw_rad * 57.29578, 1);
    jw_arr(w, "camera_lever_m");
    for (int i = 0; i < 3; i++) jw_num(w, NULL, (double)s_fn.cfg.leverarm_m[i], 3);
    jw_end(w, ']');
    jw_bool(w, "anchored", s_fn.anchored);
    if (s_fn.have_last) {
        jw_num(w, "speed_mps", (double)s_fn.last.speed_mps, 3);
        jw_num(w, "speed_sd_mps", (double)s_fn.last.sd_mps, 3);
    } else {
        jw_null(w, "speed_mps");
    }
    jw_num(w, "speed_hz", (double)s_fs.pushed_hz, 1);
    jw_int(w, "msgs", ns->msgs);
    jw_int(w, "used", ns->used);
    jw_int(w, "strobe_timed", ns->strobe_timed);
    jw_int(w, "not_anchored", ns->not_anchored);
    jw_int(w, "untriggered", ns->untriggered);
    jw_int(w, "invalid", ns->invalid);
    jw_int(w, "no_exposure", ns->no_exposure);
    jw_int(w, "no_gyro", ns->no_gyro);
    jw_int(w, "resyncs", ns->resyncs);
    jw_int(w, "breaks", ns->breaks);
    jw_int(w, "inslib_given", s_fs.to_filter);
    jw_int(w, "inslib_used", s_fs.used);
    jw_int(w, "inslib_skipped", s_fs.skipped);
    jw_int(w, "inslib_late", s_fs.late);
    jw_end(w, '}');

    usb_log_status_t ls;
    usb_log_status(&ls);
    jw_obj(w, "usb_log");
    jw_bool(w, "on", ls.on);
    jw_int(w, "bytes_per_s", ls.bytes_per_s);
    jw_int(w, "dropped_bytes", ls.dropped_bytes);
    jw_end(w, '}');
}
#endif
