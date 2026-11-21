// TF-Luna single-point ToF lidar (Benewake): its UART protocol and the
// ranges kept for the flow. No OS calls here, so the host tests use it as is;
// the serial port and the thread are in tfluna_reader.hpp.
//
// Source: Benewake's product manual SJ-PM-TF-Luna A05,
// docs/datasheets/SJ-PM-TF-Luna_A05_Product_Manual.pdf (section 6, appendices
// I and II). Design: plans/front-camera-unit.md, "Rangefinder: TF-Luna".
//
// Data frame (formats 0x01 cm and 0x06 mm, the same 9 bytes):
//   59 59 Dist_L Dist_H Amp_L Amp_H Temp_L Temp_H Sum
// Command and reply: 5A Len ID payload... Sum, Len counting all bytes.
// Sum is the low 8 bits of the sum of every byte before it.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace flow::tfluna {

// Command IDs (appendix II).
constexpr uint8_t ID_GET_VERSION = 0x01;
constexpr uint8_t ID_SOFT_RESET = 0x02;
constexpr uint8_t ID_SAMPLE_FREQ = 0x03;
constexpr uint8_t ID_OUTPUT_FORMAT = 0x05;
constexpr uint8_t ID_OUTPUT_EN = 0x07;
constexpr uint8_t ID_DEALIAS_EN = 0x29;
constexpr uint8_t ID_FILTER_BIT_MAP = 0x39;
constexpr uint8_t ID_GET_CONFIG = 0x3F;

constexpr uint8_t FMT_9B_CM = 0x01;
constexpr uint8_t FMT_9B_MM = 0x06;

constexpr size_t DATA_FRAME_LEN = 9;

// Validity (section 6.4, 3.4): Amp below 100 is too weak, 65535 overexposed,
// above 32768 ambient light overload; below 0.2 m is the blind zone and
// beyond 8 m outside the rated range.
constexpr unsigned AMP_MIN = 100;
constexpr unsigned AMP_OVERLOAD = 32768;
constexpr double   RANGE_MIN_M = 0.2;
constexpr double   RANGE_MAX_M = 8.0;

enum class Status : uint8_t { Ok = 0, LowAmp, Overload, TooClose, TooFar };
constexpr size_t N_STATUS = static_cast<size_t>(Status::TooFar) + 1;
const char* status_name(Status s);
Status      classify(double range_m, unsigned amp);

// A whole command frame with its sum.
std::vector<uint8_t> command(uint8_t id, const uint8_t* payload = nullptr, size_t n = 0);
std::vector<uint8_t> cmd_output_enable(bool on);
std::vector<uint8_t> cmd_sample_freq(uint16_t hz);  // 0 = trigger mode
std::vector<uint8_t> cmd_output_format(uint8_t fmt);
std::vector<uint8_t> cmd_filter(uint8_t bitmap);    // bit 0 Kalman, bit 1 median
std::vector<uint8_t> cmd_dealias(bool dual);        // true: dual frequency (default)
std::vector<uint8_t> cmd_get_version();

struct Sample {
    int64_t  t_ns = 0;     // estimated middle of the ranges averaged into it (CLOCK_MONOTONIC)
    float    range_m = 0;  // along the beam
    uint16_t amp = 0;
    float    temp_c = 0;
    Status   status = Status::Ok;
    bool     ok() const { return status == Status::Ok; }
};

struct Reply {
    uint8_t              id = 0;
    std::vector<uint8_t> payload;
};

// Byte-stream parser for data frames and, while replies are accepted,
// command replies. Times: the caller gives the arrival time of the last byte
// it feeds and the time one byte takes on the line; a sample's time is its
// first byte's arrival minus meas_delay_ns (the sensor averages the ranges
// of the last output interval, so about half of it).
class Parser {
public:
    void set_unit_m(double unit_m) { unit_m_ = unit_m; }  // 0.01 (cm) or 0.001 (mm)
    void set_meas_delay_ns(int64_t d) { meas_delay_ns_ = d; }
    // Replies start with 0x5A, which also turns up inside data frames, so
    // they are only looked for while the caller expects one.
    void accept_replies(bool on) { replies_ = on; }

    void feed(const uint8_t* p, size_t n, int64_t t_last_ns, int64_t byte_ns, std::vector<Sample>& samples,
              std::vector<Reply>& replies);

    uint64_t n_frames() const { return n_frames_; }
    uint64_t n_bad_sum() const { return n_bad_sum_; }
    uint64_t n_skipped() const { return n_skipped_; }  // bytes outside any frame

private:
    enum class St : uint8_t { Hunt, DataH2, Data, ReplyLen, Reply };
    void step(uint8_t b, int64_t t_ns, int64_t byte_ns, std::vector<Sample>& samples, std::vector<Reply>& replies);
    void resync(int64_t t_ns, int64_t byte_ns, std::vector<Sample>& samples, std::vector<Reply>& replies);

    St                       st_ = St::Hunt;
    std::array<uint8_t, 256> buf_{};
    size_t                   n_ = 0, want_ = 0;
    double                   unit_m_ = 0.01;
    int64_t                  meas_delay_ns_ = 0;
    bool                     replies_ = false;
    uint64_t                 n_frames_ = 0, n_bad_sum_ = 0, n_skipped_ = 0;
};

// The range near a camera frame.
struct Range {
    bool     valid = false;
    bool     held = false;  // no sample near the frame: the last valid one, younger than the hold time
    float    range_m = 0;   // median of the valid samples used
    uint16_t amp = 0;       // median Amp of the same samples
    int      n = 0;         // valid samples used
    int64_t  age_ns = 0;    // frame time minus the newest sample used
};

// The last samples, for picking the range at each camera frame.
class RangeBuffer {
public:
    void push(const Sample& s);
    // Valid samples within half_window_ns of t_ns, median; with none, the
    // last valid sample before t_ns + half_window_ns if it is at most
    // hold_ns older than t_ns (held); else not valid.
    Range at(int64_t t_ns, int64_t half_window_ns, int64_t hold_ns) const;
    // The newest sample's time, 0 before the first.
    int64_t newest_ns() const { return count_ ? ring_[(head_ + N - 1) % N].t_ns : 0; }

private:
    static constexpr size_t N = 128;  // 0.5 s at 250 Hz
    std::array<Sample, N> ring_{};
    size_t                head_ = 0, count_ = 0;
};

}  // namespace flow::tfluna
