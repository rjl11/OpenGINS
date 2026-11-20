/*
 * Clock sync with the front camera unit over UDP (plans/front-camera-unit.md,
 * "Link"; messages FLOW_MSG_TIME_REQ / _RESP in common/flow_link.h).
 *
 * The Pico sends a request and notes its time P1; the unit notes its clock
 * U1 on receipt and answers with U1 and its turnaround d; the Pico notes the
 * arrival P2. With the network delay taken as equal both ways,
 *     delay  = (P2 - P1 - d) / 2
 *     offset = P1 + delay - U1          (Pico time = unit time + offset)
 * A late poll or a queued datagram only lengthens the round trip, so as in
 * NTP only the replies with the shortest round trips are used: those within
 * CLOCK_SYNC_RTT_SLACK_US of the shortest in the window. A line through
 * them (offset against Pico time) gives the offset and the drift between
 * the two crystals; with less than CLOCK_SYNC_MIN_SPAN_US of them the mean
 * offset is used with no drift.
 *
 * The unit's clock is 64-bit in the replies, but frames carry only its low
 * 32 bits (pi_time_us); they are unwrapped against the last reply, so the
 * frame must be within +-35 minutes of it.
 *
 * Starts over when a reply disagrees with the line by more than
 * CLOCK_SYNC_RESET_US (the unit rebooted, or another unit answers). Not
 * valid with no reply for CLOCK_SYNC_STALE_US.
 *
 * Plain C, no SDK calls: host-tested in firmware/tools/test_clock_sync.c.
 * Figures are estimates, not yet measured.
 */
#ifndef CLOCK_SYNC_H
#define CLOCK_SYNC_H

#include <stdbool.h>
#include <stdint.h>

#define CLOCK_SYNC_N             32u        /* replies kept (32 s at 1 Hz)       */
#define CLOCK_SYNC_RTT_SLACK_US  300u
#define CLOCK_SYNC_MIN_SPAN_US   5000000u   /* fit the drift over at least this  */
#define CLOCK_SYNC_MAX_DRIFT     500e-6     /* |drift| above this: clamped       */
#define CLOCK_SYNC_RESET_US      50000      /* reply this far off the line: restart */
#define CLOCK_SYNC_STALE_US      10000000u
#define CLOCK_SYNC_TIMEOUT_US    100000u    /* no reply this long: request lost  */
#define CLOCK_SYNC_MAX_RTT_US    50000u     /* longer: not used at all           */

typedef struct {
    uint64_t t;      /* Pico time the unit stamped the request */
    int64_t  off;    /* Pico - unit [us]                       */
    uint32_t rtt;    /* round trip less the turnaround [us]    */
} clock_sync_sample_t;

typedef struct {
    clock_sync_sample_t s[CLOCK_SYNC_N];
    uint32_t n, head;

    /* request in flight */
    uint32_t seq;
    bool     pending;
    uint64_t t_req;

    /* the fit: off(t) = a + b (t - t_ref) */
    bool     have;
    double   a, b;
    uint64_t t_ref;
    uint64_t t_last;     /* Pico time of the last reply used */
    uint64_t unit_last;  /* unit clock of the last reply, for unwrapping */
    uint32_t rtt_min;
    uint32_t used;       /* replies in the fit */
    float    resid_us;   /* rms of the replies in the fit about the line */

    /* counters */
    uint32_t requests, replies, timeouts, stray, too_slow, resets;
} clock_sync_t;

void clock_sync_init(clock_sync_t *c);

/* The next request's number, and marks it sent at now. */
uint32_t clock_sync_request(clock_sync_t *c, uint64_t now);

/* True while a request waits for its reply; times it out after
 * CLOCK_SYNC_TIMEOUT_US. */
bool clock_sync_pending(clock_sync_t *c, uint64_t now);

/* A reply that arrived at now. False if it doesn't answer the request in
 * flight (late, duplicate) or its round trip is too long. */
bool clock_sync_reply(clock_sync_t *c, uint32_t seq, uint32_t turnaround_us, uint64_t unit_rx_us, uint64_t now);

/* True when there is a fit and it is not stale at now. */
bool clock_sync_valid(const clock_sync_t *c, uint64_t now);

/* The unit's low 32 bits of time [us] on the Pico clock. False if not valid. */
bool clock_sync_to_pico(const clock_sync_t *c, uint32_t unit_us, uint64_t now, uint64_t *pico_us);

/* The current offset (Pico - unit) at now [us], and its drift [ppm]
 * (negative when the unit's clock runs fast against the Pico's). */
double clock_sync_offset_us(const clock_sync_t *c, uint64_t now);
double clock_sync_drift_ppm(const clock_sync_t *c);

#endif /* CLOCK_SYNC_H */
