/*
 * speed_ppm: the raw pulses per metre of speed input A against the GNSS,
 * timed by the edges (plans/wheel-odometry-aiding.md, "First sensor").
 * Core 0, plain C with no SDK calls, so it runs in the host test
 * (firmware/tools/test_speed_ppm.c).
 *
 * A coarse Hall pickup gives a few rising edges a second, so counting edges
 * in a fixed window is off by up to one count. Here a run goes from one
 * rising edge to a later one: pulses = the edges between them, distance =
 * the GNSS distance travelled between the two edge times (the horizontal
 * speed integrated over the epochs, trapezoid, linearly interpolated at the
 * edges). A result needs a run of at least min_span_s and min_edges; the next
 * run starts at its last edge. Every epoch in a run must be usable, at least
 * min_mps, and at most 0.5 s after the one before, else the run starts over.
 *
 * Only rising edges are given: the high time depends on the target width and
 * the air gap, not on distance. No turn correction: a raw figure for checking
 * the wiring and the expected count, not the learned scale.
 *
 * GNSS times are the epoch's time of validity (PPS); without PPS they are the
 * arrival time, late by a near-constant latency that hardly matters over a
 * run of several seconds at a steady speed.
 */
#ifndef SPEED_PPM_H
#define SPEED_PPM_H

#include <stdbool.h>
#include <stdint.h>

#define SPEED_PPM_RING 64u /* recent rising edges; 0.1 s at 650 Hz (ISO 11786 at 5 m/s) */

typedef struct {
    float    min_mps;    /* GNSS speed needed on every epoch of a run */
    float    min_span_s; /* run length from first to last edge        */
    uint32_t min_edges;  /* pulses in a run                           */
} speed_ppm_config_t;

typedef struct {
    float    ppm;      /* pulses per metre                         */
    float    mps;      /* mean GNSS speed over the run             */
    float    span_s;   /* first to last edge                       */
    uint32_t pulses;
    uint64_t t_end_us; /* time of the run's last edge               */
} speed_ppm_result_t;

typedef struct {
    speed_ppm_config_t cfg;

    uint64_t edge_t[SPEED_PPM_RING];
    uint32_t n_edges; /* rising edges given since init (edge i is edge_t[i % RING]) */

    bool     have_epoch; /* the last epoch passed the gates            */
    uint64_t t_prev;
    double   d_prev;     /* GNSS distance at t_prev, from any origin   */
    double   v_prev;

    bool     run;        /* a run has its first edge                   */
    uint32_t idx0;
    uint64_t t0;
    double   d0;
} speed_ppm_t;

void speed_ppm_init(speed_ppm_t *s, const speed_ppm_config_t *cfg);

/* Starts the run over: edges were lost, or the input was switched. */
void speed_ppm_reset(speed_ppm_t *s);

/* A rising edge on input A, in time order. */
void speed_ppm_edge(speed_ppm_t *s, uint64_t t_us);

/* One GNSS epoch (time of validity, horizontal speed). true and *out when a
 * run completes. Edges up to t_us must have been given first. */
bool speed_ppm_gnss(speed_ppm_t *s, uint64_t t_us, double speed_mps, bool usable, speed_ppm_result_t *out);

#endif /* SPEED_PPM_H */
