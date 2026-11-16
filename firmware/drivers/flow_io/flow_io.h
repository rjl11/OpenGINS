/*
 * flow_io: the Pico's side of the optical-flow hardware link
 * (plans/optical-flow-aiding.md, common/flow_link.h), without any flow
 * processing yet:
 *
 *  - camera trigger: a PWM pulse train at a fixed rate on one GPIO, with one
 *    pulse left out every FLOW_LINK_GAP_PERIOD so the Pi can align its frame
 *    count. Each pulse's rising edge time is computed from the PWM counter
 *    in the wrap interrupt, so it is exact to the counter tick.
 *  - strobe: both edges of the camera's strobe output (active low),
 *    timestamped in the GPIO interrupt.
 *  - link: a receive-only PIO UART into a DMA ring; frames are checked with
 *    the link CRC and handed out with their arrival time.
 *
 * The interrupt handlers are in SRAM. Core 0 only, one instance.
 */
#ifndef FLOW_IO_H
#define FLOW_IO_H

#include <stdbool.h>
#include <stdint.h>

#include "flow_link.h"

typedef struct {
    int      rx_pin;      /* PIO UART RX, or -1                         */
    int      trigger_pin; /* PWM trigger output, or -1                  */
    int      strobe_pin;  /* strobe input, or -1                        */
    uint32_t trigger_hz;  /* pulse rate                                 */
    uint32_t trigger_us;  /* pulse width                                */
    uint32_t baud;        /* link baud, FLOW_LINK_BAUD                  */
} flow_io_config_t;

/* A trigger or strobe event. */
enum {
    FLOW_EV_TRIGGER = 0, /* a trigger pulse started                          */
    FLOW_EV_GAP     = 1, /* the left-out pulse: t is when it would have been */
    FLOW_EV_STROBE  = 2, /* strobe edge; level = line level after the edge   */
};
typedef struct {
    uint64_t t_us;
    uint32_t seq;   /* trigger period count (TRIGGER, GAP), strobe edge count */
    uint8_t  kind;  /* FLOW_EV_*                                              */
    uint8_t  level;
} flow_event_t;

/* A link frame with a good CRC. */
typedef struct {
    uint64_t t_arrival_us; /* when its last byte was seen in the ring (poll time) */
    uint8_t  type;         /* FLOW_MSG_*                                          */
    uint8_t  len;
    uint8_t  payload[FLOW_LINK_MAX_PAYLOAD];
} flow_frame_t;

typedef struct {
    uint32_t triggers, gaps, strobe_edges;
    uint32_t frames, crc_errors, bad_version, long_frames;
    uint32_t event_overflows, rx_overflows;
    uint32_t rx_bytes;
} flow_io_stats_t;

/* Returns false if no PIO state machine or DMA channel was free (the link is
 * then off; trigger and strobe still run). */
bool flow_io_init(const flow_io_config_t *cfg);

/* Stops the trigger pulses and the strobe interrupt (no camera fitted), or
 * starts them again. The link keeps receiving. */
void flow_io_set_enabled(bool on);

/* Oldest trigger/strobe event; false if none. */
bool flow_io_pop_event(flow_event_t *e);

/* Reads the link ring; returns true with the next good frame. Call until it
 * returns false. */
bool flow_io_poll_frame(flow_frame_t *f);

void flow_io_stats(flow_io_stats_t *out);

#endif /* FLOW_IO_H */
