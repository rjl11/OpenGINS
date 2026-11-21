// TF-Luna on a serial port: configured at start, then read in its own thread.
//
// The settings are sent at every start and never saved in the sensor (no
// ID_SAVE_SETTINGS), as the Pico does with the GNSS receiver: a sensor that
// was set up elsewhere comes back to its defaults at power-off. Only the
// baud rate has to match what the sensor runs at (115200 unless saved).
// Wiring: pin 5 open or at 3.3 V (UART mode), 3.3 V logic, 5 V supply.
//
// Sample times are CLOCK_MONOTONIC, the clock the flow's frame times are on
// (V4L2's CLOCK_BOOTTIME is the same on a board that never suspends).
// Design: plans/front-camera-unit.md, "Rangefinder: TF-Luna".
#pragma once

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <mutex>
#include <string>
#include <thread>

#include "serial.hpp"
#include "tfluna.hpp"

namespace flow::tfluna {

struct ReaderParams {
    std::string port;
    unsigned    baud = 115200;
    unsigned    rate_hz = 100;          // output rate; the sensor runs 500/n Hz, n = 2..500
    bool        mm = true;              // 9-byte/mm format (0x06) instead of cm (0x01)
    bool        device_filter = false;  // keep the sensor's own Kalman and median filters
    int         dealias = -1;           // -1 leave as is, 0 single frequency, 1 dual (default)
    std::string log_path;               // CSV of every sample, "" for none
};

// Counts over a period (take_stats() starts a new one).
struct Stats {
    uint64_t n = 0, n_ok = 0;
    uint64_t n_status[N_STATUS] = {};  // by Status
    double   sum_m = 0, sum2_m = 0, sum_amp = 0;  // over the valid samples
    float    temp_c = 0;                          // the last one
    uint64_t n_bad_sum = 0;                       // since start
    bool     gone = false;                        // the port went away
    double   mean_m() const { return n_ok ? sum_m / static_cast<double>(n_ok) : 0; }
    double   sd_m() const;
    double   mean_amp() const { return n_ok ? sum_amp / static_cast<double>(n_ok) : 0; }
};

class Reader {
public:
    Reader() = default;
    ~Reader() { stop(); }
    Reader(const Reader&) = delete;
    Reader& operator=(const Reader&) = delete;

    // Opens the port, sets the sensor up and starts the thread. Throws
    // std::runtime_error when the port can't be opened or the sensor
    // doesn't answer; the sensor's output is then switched back on if it
    // was switched off.
    void start(const ReaderParams& p);
    void stop();
    // start() succeeded (until stop()). The thread may have ended since:
    // see gone().
    bool running() const { return thread_.joinable(); }
    // The port went away and the thread ended; no new samples will come.
    bool gone() const { return gone_; }

    // "major.minor.revision" as the sensor reported it.
    const std::string& version() const { return version_; }
    // The rate the sensor confirmed.
    unsigned rate_hz() const { return rate_hz_; }

    Range at(int64_t t_ns, int64_t half_window_ns, int64_t hold_ns) const;
    Stats take_stats();

    // Every sample also goes here, from the reader's thread (the tfluna tool
    // prints them). Set before start().
    void (*on_sample)(const Sample&, void*) = nullptr;
    void* on_sample_arg = nullptr;

    static int64_t now_ns();  // CLOCK_MONOTONIC

private:
    void                 configure(const ReaderParams& p);        // port and sensor setup
    void                 configure_quiet(const ReaderParams& p);  // the part with output off
    std::vector<uint8_t> transact(const std::vector<uint8_t>& cmd, uint8_t id);
    void                 loop();

    SerialPort        ser_;
    Parser            parser_;
    int64_t           byte_ns_ = 0;
    std::string       version_;
    unsigned          rate_hz_ = 0;
    std::FILE*        log_ = nullptr;
    std::thread       thread_;
    std::atomic<bool> stop_{false};
    std::atomic<bool> gone_{false};
    mutable std::mutex mu_;
    RangeBuffer       buf_;    // under mu_
    Stats             stats_;  // under mu_
};

}  // namespace flow::tfluna
