// TM171 IMU (SYD Dynamics TransducerM, EasyProfile protocol): the packet
// codec, its Setting object, the clock mapping, the gyro history and the
// flow de-rotation. No OS calls here, so the host tests use it as is; the
// serial reader is tm171_reader.hpp.
//
// The protocol facts (packet layout, Setting fields, switch bits, the layout
// guard, the bench results on this TM171) are in plans/imu-tm210-config.md;
// the TM171 shares the TM210's layout. Its job here: plans/front-camera-unit.md,
// "The TM171's three jobs" (1: de-rotating the flow at the camera).
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace flow::tm171 {

constexpr uint8_t CMD_REQUEST = 12;
constexpr uint8_t CMD_ACK = 13;
constexpr uint8_t CMD_SETTING = 21;
constexpr uint8_t CMD_STATUS = 22;
constexpr uint8_t CMD_RPY = 35;
constexpr uint8_t CMD_RAW = 41;

constexpr uint16_t ID_BROADCAST = 0;
constexpr uint16_t ID_HOST = 2;

constexpr size_t SETTING_LEN = 20;
constexpr size_t RAW_LEN = 40;
constexpr size_t STATUS_LEN = 12;
constexpr size_t RPY_LEN = 16;  // timestamp and three floats on the TM171 (bench 2026-11-16)

// CRC-16 Modbus (poly 0xA001 reflected, init 0xFFFF): over length, header
// and payload, stored little-endian. Check value 0x4B37 for "123456789".
uint16_t crc16(const uint8_t* p, size_t n);

// AA 55 | len | header (CMD 7 bits, 3 ignored, fromId 11, toId 11) | payload | CRC.
std::vector<uint8_t> encode(uint8_t cmd, uint16_t to_id, const uint8_t* payload, size_t n);
// A Request for an object (CMD 12, payload {cmd, 0, 0, 0}), broadcast.
std::vector<uint8_t> request(uint8_t cmd);

struct Packet {
    uint8_t              cmd = 0;
    uint16_t             from_id = 0;
    std::vector<uint8_t> payload;
    int64_t              t_ns = 0;  // arrival of the read that completed it
};

class Parser {
public:
    // Appends every complete packet to out.
    void     feed(const uint8_t* p, size_t n, int64_t t_ns, std::vector<Packet>& out);
    uint64_t crc_errors() const { return crc_errors_; }
    uint64_t skipped() const { return skipped_; }

private:
    std::vector<uint8_t> buf_;
    uint64_t             crc_errors_ = 0, skipped_ = 0;
};

// ---- Setting object (CMD 21, 20 bytes) -----------------------------------

using Setting = std::array<uint8_t, SETTING_LEN>;

constexpr uint32_t SW_STATUS_OUT = 1u << 5;
constexpr uint32_t SW_RAW_OUT = 1u << 6;
constexpr uint32_t SW_QUAT_OUT = 1u << 8;
constexpr uint32_t SW_EULER_OUT = 1u << 10;
constexpr uint32_t SW_RPY_OUT = 1u << 11;
constexpr uint32_t SW_GRAVITY_OUT = 1u << 12;
constexpr uint32_t SW_RESERVED3 = 1u << 13;
constexpr uint32_t SW_RESERVED4_MASK = 7u << 14;
constexpr uint32_t SW_RESERVED4_VALUE = 2u << 14;
constexpr uint32_t SW_REQUEST_ACK = 1u << 17;
constexpr uint32_t SW_SAVE_FLASH = 1u << 18;
constexpr uint32_t SW_USB_OUT = 1u << 20;
constexpr uint32_t SW_TEMP_CALIB = 1u << 23;
constexpr uint32_t SW_BOOT_MODE_MASK = 3u << 24;
constexpr uint32_t SW_CMD42_OUT = 1u << 26;
constexpr uint32_t SW_COMBO_OUT = 1u << 27;
constexpr uint32_t SW_SLOW_OUT = 1u << 28;

constexpr uint16_t TARGET_ODR_HZ = 800;
constexpr uint16_t TARGET_BAUD_100 = 9216;  // 921600, for a UART later (no effect on USB)

uint32_t switches(const Setting& s);
uint16_t baud_100(const Setting& s);   // bytes 6-7, unit 100 bps
uint16_t silent_s(const Setting& s);   // bytes 16-17
uint16_t odr_hz(const Setting& s);     // bytes 18-19

// The layout this code knows (bit 13 = 1, bits 14-16 = 0b010, as on the
// TM210 and this TM171): anything else is not written to.
bool layout_ok(const Setting& s);

// The unit's target, read-modify-write from a read-back: raw, Status and
// RPY output (RPY: the chassis tilt for the height), no other fused output,
// USB on, temperature calibration off, Auto boot, silent time 0, 800 Hz,
// 921600 baud. Bit 17 (request an Ack) set; bit 18 (save to flash) as asked.
Setting target(const Setting& from, bool save_flash);

// The read-back already is the target (bits 17 and 18 not compared).
bool matches_target(const Setting& s);

// ---- data objects --------------------------------------------------------

// CMD 41: timestamp [us, the TM171's clock], gyro [rad/s], acc [g], mag;
// converted from the sensor's FLU frame to FRD as the Pico's tm210 driver
// does (gyro (x, -y, -z)).
struct Raw {
    uint32_t ts_us = 0;
    float    gyr_frd[3] = {};
    float    acc_flu_g[3] = {};  // as sent (not used for the flow)
};
bool decode_raw(const std::vector<uint8_t>& p, Raw& out);

// CMD 35 on the TM171: timestamp and roll, pitch, yaw in degrees.
struct Rpy {
    uint32_t ts_us = 0;
    float    roll_deg = 0, pitch_deg = 0, yaw_deg = 0;
};
bool decode_rpy(const std::vector<uint8_t>& p, Rpy& out);

// CMD 22: timestamp, temperature [C], internal rate [Hz], QoS.
struct Status {
    uint32_t ts_us = 0;
    float    temp_c = 0;
    uint16_t rate_hz = 0;
    uint8_t  qos = 0;
};
bool decode_status(const std::vector<uint8_t>& p, Status& out);

// ---- the TM171's clock on CLOCK_MONOTONIC --------------------------------

// Maps the 32-bit microsecond timestamps onto the host clock: unwrapped,
// then offset by the smallest (arrival - TM171 time) of the current and the
// last window (~1 s each), the packet that waited least in USB. A jump in
// the timestamps (a restart of the TM171) starts over.
class ClockMap {
public:
    // Returns the host time [ns] of the sample, or 0 for the first sample
    // after a start or a jump (no offset yet).
    int64_t map(uint32_t ts_us, int64_t arrival_ns);
    uint64_t jumps() const { return jumps_; }

private:
    bool     have_ = false;
    uint32_t last_ts_ = 0;
    int64_t  t_us_ = 0;  // unwrapped
    int64_t  off_cur_ = 0, off_prev_ = 0;
    bool     have_prev_ = false;
    int64_t  win_end_ns_ = 0;
    int64_t  last_out_ = 0;
    uint64_t jumps_ = 0;
};

// ---- gyro history and the rotation over a frame interval ----------------

struct GyroSample {
    int64_t t_ns = 0;
    float   g[3] = {};  // FRD [rad/s]
};

class GyroBuffer {
public:
    explicit GyroBuffer(size_t cap = 2048) : ring_(cap) {}
    void push(int64_t t_ns, const float g[3]);
    // Rotation over (ta, tb] [rad, FRD], each sample held over the interval
    // since the one before (the flow_nav.c rule). False when the buffer
    // doesn't cover the interval, or a gap longer than max_gap_ns is in it.
    bool integral(int64_t ta, int64_t tb, float out[3], int64_t max_gap_ns = 20000000) const;
    size_t size() const { return n_ < ring_.size() ? n_ : ring_.size(); }

private:
    std::vector<GyroSample> ring_;
    size_t                  n_ = 0;  // written
};

// ---- the de-rotation (same model as firmware/app/flow_nav.c) -------------

// Camera -> TM171 body (FRD), row-major: image x (right) = body y, image y
// (down in the image) = body -x (image top looks forward), optical axis =
// body z (down), then turned by cam_yaw about body z. The Pico's flow_nav.c
// builds the same matrix for the roof IMU.
void cam_to_body(float cam_yaw_rad, float r_cb[9]);

// TM171 -> vehicle body (FRD) from its mounting roll, pitch, yaw [rad]:
// Rz(yaw) Ry(pitch) Rx(roll), row-major (FLOW_MSG_UNIT_CFG's convention).
void mount_to_body(const float rpy_rad[3], float r_bm[9]);
// out = R v (row-major 3x3).
void mul3(const float r[9], const float v[3], float out[3]);

// Camera velocity [m/s, camera x / y] from one frame pair: the displacement
// [rad, the FLOW_MSG_FLOW sign], the body rotation over the interval, the
// distance to the ground along the optical axis and the interval.
//     vx = -z (dx + dth_c,y) / dt        vy = -z (dy - dth_c,x) / dt
void derotate(const float r_cb[9], float disp_x, float disp_y, const float dth_b[3], float z_m, float dt_s,
              float v_cam[2]);

}  // namespace flow::tm171
