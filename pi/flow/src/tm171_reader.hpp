// TM171 on a serial port (its USB virtual COM port, or a UART): its settings
// checked at start, then read in its own thread.
//
// At start the Setting object is read back. If it isn't the target
// (tm171.hpp, target()), the target is written to RAM, or to flash with
// save_flash. A Setting that matches is never written: the read-back can't
// tell RAM from flash (bits 17 and 18 read 0 after a power cycle), so a
// TM171 saved once is not written to flash again at every start. The layout
// guard (plans/imu-tm210-config.md) stops any write to a TM171 whose Setting
// doesn't look like the known layout.
// Calibration commands are never sent.
//
// Sample times are CLOCK_MONOTONIC (the frames' clock), from the TM171's own
// timestamps through ClockMap. Design: plans/front-camera-unit.md, "The
// TM171's three jobs".
#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

#include "flow_link.h"
#include "serial.hpp"
#include "tm171.hpp"

namespace flow::tm171 {

struct ReaderParams {
    std::string port;
    unsigned    baud = 115200;  // no effect on USB; 921600 on a UART once saved
    bool        save_flash = false;
};

struct Stats {
    uint64_t n_raw = 0, n_rpy = 0, n_status = 0, n_other = 0;
    uint64_t crc_errors = 0;  // since start
    uint64_t clock_jumps = 0; // since start
    double   gyr_sum[3] = {}, gyr_sum2[3] = {};  // FRD, over the raw samples
    Status   last_status;
    Rpy      last_rpy;
    bool     gone = false;
};

class Reader {
public:
    Reader() = default;
    ~Reader() { stop(); }
    Reader(const Reader&) = delete;
    Reader& operator=(const Reader&) = delete;

    // Opens the port, checks and if needed writes the settings, waits for
    // raw output and starts the thread. Throws std::runtime_error on failure.
    void start(const ReaderParams& p);
    void stop();
    bool running() const { return thread_.joinable(); }
    bool gone() const { return gone_; }

    uint16_t node_id() const { return node_; }
    // What start() did: "matched", "written to RAM" or "saved to flash".
    const std::string& setup() const { return setup_; }
    const Setting&     setting() const { return setting_; }

    // Rotation over (ta, tb] [rad, body FRD]; false if not covered.
    bool rotation(int64_t ta_ns, int64_t tb_ns, float dth[3]) const;
    // The latest roll and pitch [rad], if one came within max_age_ns of t.
    bool tilt(int64_t t_ns, int64_t max_age_ns, float* roll_rad, float* pitch_rad) const;
    Stats take_stats();

    static int64_t now_ns();

    // Every FLOW_IMU_BATCH_MAX raw samples as a FLOW_MSG_IMU_BATCH (FRD, the
    // TM171's own axes; specific force from its gravity output), from the
    // reader's thread: the backup IMU on the Pico. Set before start().
    std::function<void(const flow_msg_imu_batch_t&)> on_batch;

private:
    Setting read_setting();
    void    write_setting(const Setting& s);
    void    loop();
    void    handle(const Packet& pk);

    SerialPort        ser_;
    Parser            parser_;
    uint16_t          node_ = 0;
    Setting           setting_{};
    std::string       setup_;
    std::thread       thread_;
    std::atomic<bool> stop_{false};
    std::atomic<bool> gone_{false};
    mutable std::mutex mu_;
    ClockMap          clock_;  // reader thread only
    flow_msg_imu_batch_t batch_{};  // reader thread only
    int64_t           batch_t0_ns_ = 0, last_raw_ns_ = 0;
    uint16_t          batch_seq_ = 0;
    bool              batch_gap_ = false;
    GyroBuffer        gyro_;   // under mu_
    Rpy               rpy_;    // under mu_
    int64_t           rpy_t_ns_ = 0;
    Stats             stats_;  // under mu_
};

}  // namespace flow::tm171
