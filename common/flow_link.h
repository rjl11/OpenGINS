/* flow_link.h - optical-flow link from the Raspberry Pi to the Pico.
 *
 * Shared by both sides: the Pico firmware and the Pi program (pi/flow, C++)
 * include this header; the Python reference (pi/flow/python/flow_link.py)
 * mirrors it byte for byte, so change both together.
 * firmware/tools/test_flow_link.c, pi/flow/tests/test_flow.cpp and
 * pi/flow/python/test_flow_link.py check the same example frame
 * (FLOW_LINK_EXAMPLE_HEX, FLOW_LINK_EXAMPLE_TIME_REQ_HEX and
 * FLOW_LINK_EXAMPLE_TIME_RESP_HEX below). Design: plans/optical-flow-aiding.md,
 * plans/front-camera-unit.md.
 *
 * Direction: Pi -> Pico on a PIO UART or the Pico's USB port; there the Pico
 * never talks back, it triggers the camera and timestamps the strobe on
 * GPIOs instead. Over UDP (the front camera unit on the Pico's W5500, one
 * frame per datagram) the Pico also sends FLOW_MSG_TIME_REQ, and the unit
 * answers each with FLOW_MSG_TIME_RESP: the clock sync that replaces the
 * trigger and strobe wires.
 *
 * Frame (all multi-byte fields little-endian, floats IEEE 754 binary32):
 *
 *   off  size  field
 *   0    1     sync0   0xF7
 *   1    1     sync1   0x10
 *   2    1     version FLOW_LINK_VERSION
 *   3    1     type    FLOW_MSG_*
 *   4    1     len     payload bytes (N)
 *   5    N     payload
 *   5+N  2     crc     CRC-16/CCITT-FALSE over bytes 2 .. 4+N
 *                      (poly 0x1021, init 0xFFFF, no reflection, no xor out;
 *                      check value 0x29B1 for "123456789")
 *
 * A receiver that sees a wrong CRC or an unknown version resynchronises on
 * the next sync pair. An unknown type with a good CRC is skipped.
 *
 * Matching frames to the Pico's exposures: the Pi has no access to the
 * Pico's clock or trigger count. The Pico leaves out one trigger pulse every
 * FLOW_LINK_GAP_PERIOD pulses. The Pi sees the doubled interval in its sensor
 * timestamps and sets FLOW_F_AFTER_GAP on the first frame after it; the Pico
 * knows which of its strobes came after its own gap, and from there counts
 * frames. frame_seq is the Pi's count of frames received. A frame the Pi
 * lost shows as an interval it can't explain and sets FLOW_F_DROPPED; the
 * Pico then drops its count until the next gap.
 */
#ifndef FLOW_LINK_H
#define FLOW_LINK_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define FLOW_LINK_SYNC0       0xF7u
#define FLOW_LINK_SYNC1       0x10u
#define FLOW_LINK_VERSION     1u
#define FLOW_LINK_HEADER_LEN  5u /* sync0, sync1, version, type, len */
#define FLOW_LINK_CRC_LEN     2u
#define FLOW_LINK_MAX_PAYLOAD 64u
#define FLOW_LINK_BAUD        115200u

/* One trigger left out every this many pulses (see above). */
#define FLOW_LINK_GAP_PERIOD 256u

enum {
    FLOW_MSG_FLOW      = 1, /* flow between two consecutive frames (unit -> Pico) */
    FLOW_MSG_TIME_REQ  = 2, /* clock sync request (Pico -> unit, UDP only)        */
    FLOW_MSG_TIME_RESP = 3, /* clock sync reply (unit -> Pico, UDP only)          */
};

/* FLOW_MSG_TIME_REQ payload: u32 seq (the Pico's request number). The unit
 * answers at once, from the socket the request came to. */
#define FLOW_MSG_TIME_REQ_LEN 4u

/* FLOW_MSG_TIME_RESP payload, byte offsets
 * 0  u32 seq            copied from the request
 * 4  u32 turnaround_us  unit's time from receiving the request to sending
 *                       this reply [us] (the Pico takes it off the round trip)
 * 8  u64 unit_rx_us     unit's clock when the request was received [us]:
 *                       the same clock as pi_time_us in FLOW_MSG_FLOW
 *                       (CLOCK_MONOTONIC on Linux), all 64 bits
 */
#define FLOW_MSG_TIME_RESP_LEN 16u

typedef struct {
    uint32_t seq;
    uint32_t turnaround_us;
    uint64_t unit_rx_us;
} flow_msg_time_resp_t;

/* FLOW_MSG_FLOW flags */
#define FLOW_F_VALID     0x01u /* displacement usable (enough features tracked) */
#define FLOW_F_AFTER_GAP 0x02u /* the newer frame is the first after a trigger gap */
#define FLOW_F_TRIGGERED 0x04u /* camera is in external-trigger mode */
#define FLOW_F_DROPPED   0x08u /* the Pi saw an interval it can't explain (lost frame) */

/* FLOW_MSG_FLOW payload, byte offsets */
#define FLOW_MSG_FLOW_LEN 28u
/* 0  u32 frame_seq     Pi's count of frames received, for the newer frame
 * 4  u32 pi_time_us    Pi sensor timestamp of the newer frame, low 32 bits [us]
 *                      (diagnostics only; the Pico uses its strobe times)
 * 8  u32 pi_dt_us      Pi's interval between the two frames [us] (diagnostics)
 * 12 f32 disp_x_rad    image displacement between the two frames in
 * 16 f32 disp_y_rad    normalised image coordinates (x/z, y/z; radians near
 *                      the centre), camera x (right) / y (down), undistorted,
 *                      sign: where the ground pattern moved in the image.
 *                      Over flat ground it is v*dt/h for a translation,
 *                      the same everywhere in the image.
 *                      NOT divided by time: the Pico divides by its own
 *                      strobe interval, and removes rotation with its gyro.
 * 20 f32 stddev_rad    1-sigma of each displacement axis [rad]
 * 24 u16 n_tracked     features tracked between the two frames
 * 26 u8  exposure_100us exposure time [100 us], saturates at 255
 * 27 u8  flags         FLOW_F_*
 */

typedef struct {
    uint32_t frame_seq;
    uint32_t pi_time_us;
    uint32_t pi_dt_us;
    float    disp_x_rad;
    float    disp_y_rad;
    float    stddev_rad;
    uint16_t n_tracked;
    uint8_t  exposure_100us;
    uint8_t  flags;
} flow_msg_flow_t;

/* Example used by both test sides: a FLOW_MSG_FLOW frame with
 * frame_seq 1000, pi_time_us 123456789, pi_dt_us 3333, disp 0.01 / -0.002 rad,
 * stddev 0.0005 rad, 87 features, exposure 1.2 ms (12), flags VALID|TRIGGERED. */
#define FLOW_LINK_EXAMPLE_HEX                                                   \
    "f710" "01" "01" "1c"                       /* sync, version, type, len */  \
    "e8030000" "15cd5b07" "050d0000"            /* seq, pi_time, pi_dt */       \
    "0ad7233c" "6f1203bb" "6f12033a"            /* disp x, disp y, stddev */    \
    "5700" "0c" "05"                            /* n, exposure, flags */        \
    "8ee4"                                      /* crc */

/* Clock sync examples, checked by the same three tests: a request with seq 7,
 * and its reply with turnaround 42 us and unit_rx_us 0x1234567890. */
#define FLOW_LINK_EXAMPLE_TIME_REQ_HEX                                          \
    "f710" "01" "02" "04"                       /* sync, version, type, len */  \
    "07000000"                                  /* seq */                       \
    "c41a"                                      /* crc */

#define FLOW_LINK_EXAMPLE_TIME_RESP_HEX                                         \
    "f710" "01" "03" "10"                       /* sync, version, type, len */  \
    "07000000" "2a000000"                       /* seq, turnaround */           \
    "9078563412000000"                          /* unit_rx_us */                \
    "db31"                                      /* crc */

static inline uint16_t flow_link_crc16(const uint8_t *p, size_t n)
{
    uint16_t crc = 0xFFFFu;
    while (n--) {
        crc ^= (uint16_t)((uint16_t)*p++ << 8);
        for (int i = 0; i < 8; i++) {
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static inline uint32_t flow_link_rd_u32(const uint8_t *b)
{
    return (uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
}

static inline float flow_link_rd_f32(const uint8_t *b)
{
    uint32_t u = flow_link_rd_u32(b);
    float    f;
    memcpy(&f, &u, sizeof f);
    return f;
}

/* Decode a FLOW_MSG_FLOW payload (CRC already checked by the caller).
 * Returns 0, or -1 if len is wrong. */
static inline int flow_link_decode_flow(const uint8_t *payload, size_t len, flow_msg_flow_t *out)
{
    if (len != FLOW_MSG_FLOW_LEN) return -1;
    out->frame_seq      = flow_link_rd_u32(&payload[0]);
    out->pi_time_us     = flow_link_rd_u32(&payload[4]);
    out->pi_dt_us       = flow_link_rd_u32(&payload[8]);
    out->disp_x_rad     = flow_link_rd_f32(&payload[12]);
    out->disp_y_rad     = flow_link_rd_f32(&payload[16]);
    out->stddev_rad     = flow_link_rd_f32(&payload[20]);
    out->n_tracked      = (uint16_t)(payload[24] | (payload[25] << 8));
    out->exposure_100us = payload[26];
    out->flags          = payload[27];
    return 0;
}

static inline void flow_link_wr_u32(uint8_t *b, uint32_t v)
{
    b[0] = (uint8_t)v;
    b[1] = (uint8_t)(v >> 8);
    b[2] = (uint8_t)(v >> 16);
    b[3] = (uint8_t)(v >> 24);
}

static inline void flow_link_wr_f32(uint8_t *b, float f)
{
    uint32_t u;
    memcpy(&u, &f, sizeof u);
    flow_link_wr_u32(b, u);
}

/* Total frame bytes for a FLOW_MSG_FLOW message. */
#define FLOW_LINK_FLOW_FRAME_LEN (FLOW_LINK_HEADER_LEN + FLOW_MSG_FLOW_LEN + FLOW_LINK_CRC_LEN)

/* Build a complete FLOW_MSG_FLOW frame (the Pi side; the Pico only decodes).
 * Returns the frame length, or 0 if cap is too small. */
static inline size_t flow_link_encode_flow(const flow_msg_flow_t *m, uint8_t *out, size_t cap)
{
    if (cap < FLOW_LINK_FLOW_FRAME_LEN) return 0;
    uint8_t *p = &out[FLOW_LINK_HEADER_LEN];
    out[0] = FLOW_LINK_SYNC0;
    out[1] = FLOW_LINK_SYNC1;
    out[2] = FLOW_LINK_VERSION;
    out[3] = FLOW_MSG_FLOW;
    out[4] = FLOW_MSG_FLOW_LEN;
    flow_link_wr_u32(&p[0], m->frame_seq);
    flow_link_wr_u32(&p[4], m->pi_time_us);
    flow_link_wr_u32(&p[8], m->pi_dt_us);
    flow_link_wr_f32(&p[12], m->disp_x_rad);
    flow_link_wr_f32(&p[16], m->disp_y_rad);
    flow_link_wr_f32(&p[20], m->stddev_rad);
    p[24] = (uint8_t)m->n_tracked;
    p[25] = (uint8_t)(m->n_tracked >> 8);
    p[26] = m->exposure_100us;
    p[27] = m->flags;
    uint16_t crc = flow_link_crc16(&out[2], 3u + FLOW_MSG_FLOW_LEN);
    out[FLOW_LINK_HEADER_LEN + FLOW_MSG_FLOW_LEN]      = (uint8_t)crc;
    out[FLOW_LINK_HEADER_LEN + FLOW_MSG_FLOW_LEN + 1u] = (uint8_t)(crc >> 8);
    return FLOW_LINK_FLOW_FRAME_LEN;
}

/* Wrap a payload of len bytes into a complete frame. Returns the frame
 * length, or 0 if cap is too small or the payload too long. */
static inline size_t flow_link_encode(uint8_t type, const uint8_t *payload, size_t len, uint8_t *out, size_t cap)
{
    const size_t n = FLOW_LINK_HEADER_LEN + len + FLOW_LINK_CRC_LEN;
    if (len > FLOW_LINK_MAX_PAYLOAD || cap < n) return 0;
    out[0] = FLOW_LINK_SYNC0;
    out[1] = FLOW_LINK_SYNC1;
    out[2] = FLOW_LINK_VERSION;
    out[3] = type;
    out[4] = (uint8_t)len;
    if (len) memcpy(&out[FLOW_LINK_HEADER_LEN], payload, len);
    const uint16_t crc = flow_link_crc16(&out[2], 3u + len);
    out[FLOW_LINK_HEADER_LEN + len]      = (uint8_t)crc;
    out[FLOW_LINK_HEADER_LEN + len + 1u] = (uint8_t)(crc >> 8);
    return n;
}

/* Checks one whole frame (sync, version, length, CRC). Returns the payload
 * length and sets *type, or -1 if it is not a good frame. */
static inline int flow_link_check(const uint8_t *b, size_t n, uint8_t *type)
{
    if (n < FLOW_LINK_HEADER_LEN + FLOW_LINK_CRC_LEN || b[0] != FLOW_LINK_SYNC0 || b[1] != FLOW_LINK_SYNC1 ||
        b[2] != FLOW_LINK_VERSION || b[4] > FLOW_LINK_MAX_PAYLOAD ||
        n != (size_t)FLOW_LINK_HEADER_LEN + b[4] + FLOW_LINK_CRC_LEN) {
        return -1;
    }
    const size_t   len = b[4];
    const uint16_t crc = (uint16_t)(b[FLOW_LINK_HEADER_LEN + len] | (b[FLOW_LINK_HEADER_LEN + len + 1u] << 8));
    if (flow_link_crc16(&b[2], 3u + len) != crc) return -1;
    *type = b[3];
    return (int)len;
}

#define FLOW_LINK_TIME_REQ_FRAME_LEN  (FLOW_LINK_HEADER_LEN + FLOW_MSG_TIME_REQ_LEN + FLOW_LINK_CRC_LEN)
#define FLOW_LINK_TIME_RESP_FRAME_LEN (FLOW_LINK_HEADER_LEN + FLOW_MSG_TIME_RESP_LEN + FLOW_LINK_CRC_LEN)

static inline size_t flow_link_encode_time_req(uint32_t seq, uint8_t *out, size_t cap)
{
    uint8_t p[FLOW_MSG_TIME_REQ_LEN];
    flow_link_wr_u32(p, seq);
    return flow_link_encode(FLOW_MSG_TIME_REQ, p, sizeof p, out, cap);
}

static inline int flow_link_decode_time_req(const uint8_t *payload, size_t len, uint32_t *seq)
{
    if (len != FLOW_MSG_TIME_REQ_LEN) return -1;
    *seq = flow_link_rd_u32(payload);
    return 0;
}

static inline size_t flow_link_encode_time_resp(const flow_msg_time_resp_t *m, uint8_t *out, size_t cap)
{
    uint8_t p[FLOW_MSG_TIME_RESP_LEN];
    flow_link_wr_u32(&p[0], m->seq);
    flow_link_wr_u32(&p[4], m->turnaround_us);
    flow_link_wr_u32(&p[8], (uint32_t)m->unit_rx_us);
    flow_link_wr_u32(&p[12], (uint32_t)(m->unit_rx_us >> 32));
    return flow_link_encode(FLOW_MSG_TIME_RESP, p, sizeof p, out, cap);
}

static inline int flow_link_decode_time_resp(const uint8_t *payload, size_t len, flow_msg_time_resp_t *out)
{
    if (len != FLOW_MSG_TIME_RESP_LEN) return -1;
    out->seq           = flow_link_rd_u32(&payload[0]);
    out->turnaround_us = flow_link_rd_u32(&payload[4]);
    out->unit_rx_us    = (uint64_t)flow_link_rd_u32(&payload[8]) | ((uint64_t)flow_link_rd_u32(&payload[12]) << 32);
    return 0;
}

#endif /* FLOW_LINK_H */
