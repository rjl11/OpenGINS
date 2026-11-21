/* flow_link.h - optical-flow link from the Raspberry Pi to the Pico.
 *
 * Shared by both sides: the Pico firmware and the Pi program (pi/flow, C++)
 * include this header; the Python reference (pi/flow/python/flow_link.py)
 * mirrors it byte for byte, so change both together.
 * firmware/tools/test_flow_link.c, pi/flow/tests/test_flow.cpp and
 * pi/flow/python/test_flow_link.py check the same example frame
 * (FLOW_LINK_EXAMPLE_HEX, FLOW_LINK_EXAMPLE_TIME_REQ_HEX,
 * FLOW_LINK_EXAMPLE_TIME_RESP_HEX, FLOW_LINK_EXAMPLE_CAM_VEL_HEX,
 * FLOW_LINK_EXAMPLE_UNIT_CFG_HEX and FLOW_LINK_EXAMPLE_IMU_BATCH_HEX below). Design: plans/optical-flow-aiding.md,
 * plans/front-camera-unit.md.
 *
 * Direction: Pi -> Pico on a PIO UART or the Pico's USB port; there the Pico
 * never talks back, it triggers the camera and timestamps the strobe on
 * GPIOs instead. Over UDP (the front camera unit on the Pico's W5500, one
 * frame per datagram) the Pico also sends FLOW_MSG_TIME_REQ, and the unit
 * answers each with FLOW_MSG_TIME_RESP: the clock sync that replaces the
 * trigger and strobe wires. Once a second the Pico also sends
 * FLOW_MSG_UNIT_CFG, the unit's settings and the heartbeat (over UDP, and on
 * the Pico's USB port as a "flowcfg <hex>" text line for bench tests).
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
#define FLOW_LINK_MAX_PAYLOAD 128u /* FLOW_MSG_IMU_BATCH is the longest, 120 */
#define FLOW_LINK_BAUD        115200u

/* One trigger left out every this many pulses (see above). */
#define FLOW_LINK_GAP_PERIOD 256u

enum {
    FLOW_MSG_FLOW      = 1, /* flow between two consecutive frames (unit -> Pico) */
    FLOW_MSG_TIME_REQ  = 2, /* clock sync request (Pico -> unit, UDP only)        */
    FLOW_MSG_TIME_RESP = 3, /* clock sync reply (unit -> Pico, UDP only)          */
    FLOW_MSG_CAM_VEL   = 4, /* camera velocity, de-rotated on the unit (unit -> Pico) */
    FLOW_MSG_UNIT_CFG  = 5, /* the unit's settings and the heartbeat (Pico -> unit) */
    FLOW_MSG_IMU_BATCH = 6, /* TM171 samples, the backup IMU (unit -> Pico)        */
};

/* FLOW_MSG_IMU_BATCH payload, byte offsets: up to FLOW_IMU_BATCH_MAX TM171
 * samples (plans/front-camera-unit.md, "Backup IMU for the roof TM210"),
 * in the TM171's own axes converted to FRD (its mounting is applied on the
 * Pico), specific force and rate, fixed point.
 * 0  u32 t0_us        unit's time of the first sample, low 32 bits [us],
 *                     the clock of FLOW_MSG_TIME_RESP
 * 4  u16 seq          the unit's batch count; a gap is a lost batch
 * 6  u8  n            samples in this batch, 1 .. FLOW_IMU_BATCH_MAX
 * 7  u8  flags        FLOW_IMU_F_*
 * 8  then n times 14 bytes:
 *    u16 dt_us        sample time minus t0_us [us]
 *    i16 gyr[3]       rate, FRD [FLOW_IMU_GYR_LSB rad/s], +-8 rad/s
 *    i16 acc[3]       specific force, FRD [FLOW_IMU_ACC_LSB m/s^2], +-32 m/s^2
 *                     (level and at rest: (0, 0, -9.81))
 * Values beyond the range are clamped and flagged.
 */
#define FLOW_IMU_BATCH_MAX     8u
#define FLOW_IMU_SAMPLE_LEN    14u
#define FLOW_IMU_BATCH_HDR_LEN 8u
#define FLOW_IMU_GYR_LSB       (1.0f / 4096.0f)
#define FLOW_IMU_ACC_LSB       (1.0f / 1024.0f)
#define FLOW_IMU_F_GAP         0x01u /* the unit lost samples before this batch */
#define FLOW_IMU_F_CLAMPED     0x02u /* a value was beyond the range and clamped */

typedef struct {
    uint16_t dt_us;
    float    gyr[3]; /* rad/s */
    float    acc[3]; /* m/s^2 */
} flow_imu_sample_t;

typedef struct {
    uint32_t          t0_us;
    uint16_t          seq;
    uint8_t           n;
    uint8_t           flags;
    flow_imu_sample_t s[FLOW_IMU_BATCH_MAX];
} flow_msg_imu_batch_t;

/* FLOW_MSG_UNIT_CFG payload, byte offsets: the front camera unit's
 * settings from the Pico's web page, sent once a second whether they changed
 * or not, so the message is also the Pico's heartbeat. The unit uses them in
 * place of its command-line defaults from the first one on.
 * 0  u32 seq            the Pico's count of these messages
 * 4  f32 height_m       camera lens above level ground [m], used when the
 *                       unit has no range
 * 8  f32 cam_yaw_rad    image top from body forward, clockwise from above
 * 12 f32 imu_roll_rad   TM171 mounting against the vehicle body (FRD):
 * 16 f32 imu_pitch_rad  roll, pitch, yaw, the rotation TM171 -> body is
 * 20 f32 imu_yaw_rad    Rz(yaw) Ry(pitch) Rx(roll)
 * 24 u8  flags          FLOW_CFG_F_*
 * 25 u8[3]              reserved, 0
 */
#define FLOW_MSG_UNIT_CFG_LEN 28u
#define FLOW_CFG_F_FLOW_ON    0x01u /* the Pico has the optical flow switched on */

typedef struct {
    uint32_t seq;
    float    height_m;
    float    cam_yaw_rad;
    float    imu_rpy_rad[3];
    uint8_t  flags;
} flow_msg_unit_cfg_t;

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
#define FLOW_F_PHASE     0x10u /* displacement from phase correlation on the image centre
                               * (too few features for Lucas-Kanade); n_tracked then
                               * carries the correlation response x 1000 instead */
#define FLOW_F_RANGE     0x20u /* FLOW_MSG_CAM_VEL only: height from the rangefinder
                               * (else the unit's configured height) */

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
 * 24 u16 n_tracked     features tracked between the two frames; with
 *                      FLOW_F_PHASE the correlation response x 1000 (0..1000)
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

/* FLOW_MSG_CAM_VEL payload, byte offsets: the front camera unit's own
 * conversion (plans/front-camera-unit.md, "The TM171's three jobs"). The
 * unit removes the rotation with its own gyro over the frame interval, in its
 * own clock, and scales by its height; the Pico then only moves the velocity
 * to the IMU point, gates and averages it.
 * 0  u32 frame_seq     as in FLOW_MSG_FLOW
 * 4  u32 pi_time_us    unit's timestamp of the newer frame, low 32 bits [us],
 *                      the clock of FLOW_MSG_TIME_RESP (timed as FLOW_MSG_FLOW)
 * 8  u32 pi_dt_us      interval between the two frames [us]
 * 12 f32 vel_x_mps     mean velocity of the camera over the ground across the
 * 16 f32 vel_y_mps     interval, camera axes x (right) / y (down in the
 *                      image) [m/s]: rotation removed, scaled by height_m.
 *                      Sign: the camera's motion, the opposite of where the
 *                      ground pattern moved in the image.
 * 20 f32 stddev_mps    1-sigma of each axis from the image noise [m/s],
 *                      without the height error
 * 24 f32 height_m      height above the ground plane used, vertical (the
 *                      range already corrected for tilt) [m]
 * 28 f32 height_sd_m   1-sigma of height_m [m]; 0 = unknown (the Pico then
 *                      uses its own relative height error)
 * 32 u16 n_tracked     as in FLOW_MSG_FLOW
 * 34 u8  exposure_100us as in FLOW_MSG_FLOW
 * 35 u8  flags         FLOW_F_*, with FLOW_F_RANGE
 */
#define FLOW_MSG_CAM_VEL_LEN 36u

typedef struct {
    uint32_t frame_seq;
    uint32_t pi_time_us;
    uint32_t pi_dt_us;
    float    vel_x_mps;
    float    vel_y_mps;
    float    stddev_mps;
    float    height_m;
    float    height_sd_m;
    uint16_t n_tracked;
    uint8_t  exposure_100us;
    uint8_t  flags;
} flow_msg_cam_vel_t;

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

/* Camera-velocity example, checked by the same three tests: frame_seq 2000,
 * pi_time_us 987654321, pi_dt_us 10000, velocity 1.5 / -0.25 m/s, stddev
 * 0.02 m/s, height 1.2 m with 1-sigma 0.03 m, 64 features, exposure 0.8 ms
 * (8), flags VALID|RANGE. */
#define FLOW_LINK_EXAMPLE_CAM_VEL_HEX                                           \
    "f710" "01" "04" "24"                       /* sync, version, type, len */  \
    "d0070000" "b168de3a" "10270000"            /* seq, pi_time, pi_dt */       \
    "0000c03f" "000080be" "0ad7a33c"            /* vel x, vel y, stddev */      \
    "9a99993f" "8fc2f53c"                       /* height, height sd */         \
    "4000" "08" "21"                            /* n, exposure, flags */        \
    "41b5"                                      /* crc */

/* Unit settings example, checked by the same three tests: seq 3, height
 * 1.25 m, image yaw 0.5 rad, TM171 mounting roll 0 / pitch -0.25 / yaw 1.5
 * rad, flags FLOW_ON. */
#define FLOW_LINK_EXAMPLE_UNIT_CFG_HEX                                          \
    "f710" "01" "05" "1c"                       /* sync, version, type, len */  \
    "03000000" "0000a03f" "0000003f"            /* seq, height, cam yaw */      \
    "00000000" "000080be" "0000c03f"            /* TM171 roll, pitch, yaw */    \
    "01000000"                                  /* flags, reserved */           \
    "18ca"                                      /* crc */

/* IMU batch example, checked by the same three tests: t0_us 1000000, seq 9,
 * two samples, no flags. Sample 0 at dt 0: gyro (0.5, -0.25, 0) rad/s, acc
 * (0, 0, -9.8125) m/s^2; sample 1 at dt 1250: gyro (0, 0, 1), acc (1, -1, -9.8125). */
#define FLOW_LINK_EXAMPLE_IMU_BATCH_HEX                                         \
    "f710" "01" "06" "24"                       /* sync, version, type, len */  \
    "40420f00" "0900" "02" "00"                 /* t0, seq, n, flags */         \
    "0000" "0008" "00fc" "0000"                 /* dt, gyro */                  \
    "0000" "0000" "c0d8"                        /* acc */                       \
    "e204" "0000" "0000" "0010"                 /* dt, gyro */                  \
    "0004" "00fc" "c0d8"                        /* acc */                       \
    "eac3"                                      /* crc */

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

#define FLOW_LINK_CAM_VEL_FRAME_LEN (FLOW_LINK_HEADER_LEN + FLOW_MSG_CAM_VEL_LEN + FLOW_LINK_CRC_LEN)

static inline size_t flow_link_encode_cam_vel(const flow_msg_cam_vel_t *m, uint8_t *out, size_t cap)
{
    uint8_t p[FLOW_MSG_CAM_VEL_LEN];
    flow_link_wr_u32(&p[0], m->frame_seq);
    flow_link_wr_u32(&p[4], m->pi_time_us);
    flow_link_wr_u32(&p[8], m->pi_dt_us);
    flow_link_wr_f32(&p[12], m->vel_x_mps);
    flow_link_wr_f32(&p[16], m->vel_y_mps);
    flow_link_wr_f32(&p[20], m->stddev_mps);
    flow_link_wr_f32(&p[24], m->height_m);
    flow_link_wr_f32(&p[28], m->height_sd_m);
    p[32] = (uint8_t)m->n_tracked;
    p[33] = (uint8_t)(m->n_tracked >> 8);
    p[34] = m->exposure_100us;
    p[35] = m->flags;
    return flow_link_encode(FLOW_MSG_CAM_VEL, p, sizeof p, out, cap);
}

/* Decode a FLOW_MSG_CAM_VEL payload (CRC already checked by the caller).
 * Returns 0, or -1 if len is wrong. */
static inline int flow_link_decode_cam_vel(const uint8_t *payload, size_t len, flow_msg_cam_vel_t *out)
{
    if (len != FLOW_MSG_CAM_VEL_LEN) return -1;
    out->frame_seq      = flow_link_rd_u32(&payload[0]);
    out->pi_time_us     = flow_link_rd_u32(&payload[4]);
    out->pi_dt_us       = flow_link_rd_u32(&payload[8]);
    out->vel_x_mps      = flow_link_rd_f32(&payload[12]);
    out->vel_y_mps      = flow_link_rd_f32(&payload[16]);
    out->stddev_mps     = flow_link_rd_f32(&payload[20]);
    out->height_m       = flow_link_rd_f32(&payload[24]);
    out->height_sd_m    = flow_link_rd_f32(&payload[28]);
    out->n_tracked      = (uint16_t)(payload[32] | (payload[33] << 8));
    out->exposure_100us = payload[34];
    out->flags          = payload[35];
    return 0;
}

#define FLOW_LINK_UNIT_CFG_FRAME_LEN (FLOW_LINK_HEADER_LEN + FLOW_MSG_UNIT_CFG_LEN + FLOW_LINK_CRC_LEN)

static inline size_t flow_link_encode_unit_cfg(const flow_msg_unit_cfg_t *m, uint8_t *out, size_t cap)
{
    uint8_t p[FLOW_MSG_UNIT_CFG_LEN];
    memset(p, 0, sizeof p);
    flow_link_wr_u32(&p[0], m->seq);
    flow_link_wr_f32(&p[4], m->height_m);
    flow_link_wr_f32(&p[8], m->cam_yaw_rad);
    flow_link_wr_f32(&p[12], m->imu_rpy_rad[0]);
    flow_link_wr_f32(&p[16], m->imu_rpy_rad[1]);
    flow_link_wr_f32(&p[20], m->imu_rpy_rad[2]);
    p[24] = m->flags;
    return flow_link_encode(FLOW_MSG_UNIT_CFG, p, sizeof p, out, cap);
}

static inline int flow_link_decode_unit_cfg(const uint8_t *payload, size_t len, flow_msg_unit_cfg_t *out)
{
    if (len != FLOW_MSG_UNIT_CFG_LEN) return -1;
    out->seq            = flow_link_rd_u32(&payload[0]);
    out->height_m       = flow_link_rd_f32(&payload[4]);
    out->cam_yaw_rad    = flow_link_rd_f32(&payload[8]);
    out->imu_rpy_rad[0] = flow_link_rd_f32(&payload[12]);
    out->imu_rpy_rad[1] = flow_link_rd_f32(&payload[16]);
    out->imu_rpy_rad[2] = flow_link_rd_f32(&payload[20]);
    out->flags          = payload[24];
    return 0;
}

static inline int16_t flow_link_q16(float v, float lsb, uint8_t *flags)
{
    float q = v / lsb;
    if (!(q > -32767.0f)) { /* also NaN */
        *flags |= FLOW_IMU_F_CLAMPED;
        return -32767;
    }
    if (q > 32767.0f) {
        *flags |= FLOW_IMU_F_CLAMPED;
        return 32767;
    }
    return (int16_t)(q < 0.0f ? q - 0.5f : q + 0.5f);
}

static inline void flow_link_wr_u16(uint8_t *b, uint16_t v)
{
    b[0] = (uint8_t)v;
    b[1] = (uint8_t)(v >> 8);
}

/* Returns the frame length, or 0 if n is 0 or too large or cap too small. */
static inline size_t flow_link_encode_imu_batch(const flow_msg_imu_batch_t *m, uint8_t *out, size_t cap)
{
    if (m->n == 0 || m->n > FLOW_IMU_BATCH_MAX) return 0;
    uint8_t p[FLOW_IMU_BATCH_HDR_LEN + FLOW_IMU_BATCH_MAX * FLOW_IMU_SAMPLE_LEN];
    uint8_t flags = m->flags;
    flow_link_wr_u32(&p[0], m->t0_us);
    flow_link_wr_u16(&p[4], m->seq);
    p[6] = m->n;
    for (unsigned i = 0; i < m->n; i++) {
        uint8_t *q = &p[FLOW_IMU_BATCH_HDR_LEN + i * FLOW_IMU_SAMPLE_LEN];
        flow_link_wr_u16(&q[0], m->s[i].dt_us);
        for (unsigned k = 0; k < 3; k++) {
            flow_link_wr_u16(&q[2 + 2 * k], (uint16_t)flow_link_q16(m->s[i].gyr[k], FLOW_IMU_GYR_LSB, &flags));
            flow_link_wr_u16(&q[8 + 2 * k], (uint16_t)flow_link_q16(m->s[i].acc[k], FLOW_IMU_ACC_LSB, &flags));
        }
    }
    p[7] = flags;
    return flow_link_encode(FLOW_MSG_IMU_BATCH, p, FLOW_IMU_BATCH_HDR_LEN + (size_t)m->n * FLOW_IMU_SAMPLE_LEN, out,
                            cap);
}

/* Returns 0, or -1 if the length doesn't match a whole number of samples. */
static inline int flow_link_decode_imu_batch(const uint8_t *payload, size_t len, flow_msg_imu_batch_t *out)
{
    if (len < FLOW_IMU_BATCH_HDR_LEN + FLOW_IMU_SAMPLE_LEN) return -1;
    const unsigned n = payload[6];
    if (n == 0 || n > FLOW_IMU_BATCH_MAX || len != FLOW_IMU_BATCH_HDR_LEN + n * FLOW_IMU_SAMPLE_LEN) return -1;
    out->t0_us = flow_link_rd_u32(&payload[0]);
    out->seq   = (uint16_t)(payload[4] | (payload[5] << 8));
    out->n     = (uint8_t)n;
    out->flags = payload[7];
    for (unsigned i = 0; i < n; i++) {
        const uint8_t *q = &payload[FLOW_IMU_BATCH_HDR_LEN + i * FLOW_IMU_SAMPLE_LEN];
        out->s[i].dt_us = (uint16_t)(q[0] | (q[1] << 8));
        for (unsigned k = 0; k < 3; k++) {
            out->s[i].gyr[k] = (float)(int16_t)(q[2 + 2 * k] | (q[3 + 2 * k] << 8)) * FLOW_IMU_GYR_LSB;
            out->s[i].acc[k] = (float)(int16_t)(q[8 + 2 * k] | (q[9 + 2 * k] << 8)) * FLOW_IMU_ACC_LSB;
        }
    }
    return 0;
}

#endif /* FLOW_LINK_H */
