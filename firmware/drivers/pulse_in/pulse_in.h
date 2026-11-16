/*
 * pulse_in: edge timestamps from up to two pulse inputs (wheel or ground
 * speed, plans/wheel-odometry-aiding.md).
 *
 * Every edge, rising and falling, on input A and (if used) input B is
 * timestamped with time_us_64() in the GPIO interrupt (in SRAM) and queued
 * with the level of both inputs right after it. Nothing is decoded here:
 * one Hall sensor or an ISO 11786 signal uses A only, a quadrature encoder
 * uses A and B, and the caller (or a log) works out speed and direction.
 *
 * Core 0 only, one instance. Raw IO_IRQ_BANK0 handler, so it shares the
 * bank with the PPS and BMP581 handlers.
 */
#ifndef PULSE_IN_H
#define PULSE_IN_H

#include <stdbool.h>
#include <stdint.h>

#define PULSE_IN_RING 512u /* edges; 0.1 s at 5 kHz */

typedef struct {
    int  pin_a;   /* GPIO of input A                     */
    int  pin_b;   /* GPIO of input B, or -1              */
    bool pull_up; /* internal pull-up (open collector)   */
} pulse_in_config_t;

/* One edge. levels: bit 0 = A, bit 1 = B after the edge; bit 7 = the edge
 * was on B (else on A). */
#define PULSE_IN_EDGE_B 0x80u
typedef struct {
    uint64_t t_us;
    uint8_t  levels;
} pulse_edge_t;

typedef struct {
    uint32_t edges[2];  /* all edges per input, since init    */
    uint32_t rising[2]; /* rising edges per input             */
    uint32_t overflows; /* edges lost: the ring was full      */
} pulse_in_stats_t;

void pulse_in_init(const pulse_in_config_t *cfg);

/* Turns the edge interrupts off (no sensor fitted) or back on. */
void pulse_in_set_enabled(bool on);

/* Oldest edge not yet taken; false if none. */
bool pulse_in_pop(pulse_edge_t *e);

/* A copy of the counters. */
void pulse_in_stats(pulse_in_stats_t *out);

#endif /* PULSE_IN_H */
