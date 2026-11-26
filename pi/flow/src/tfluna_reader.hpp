// TF-Luna on a serial port or on I2C: configured at start, then read in its
// own thread.
//
// UART: the settings are sent at every start and not saved in the sensor,
// as the Pico does with the GNSS receiver: a sensor that was set up
// elsewhere comes back to its defaults at power-off. Only the baud rate has
// to match what the sensor runs at (115200 unless saved). The one exception
// is save_settings (tfluna --save), which writes them to the sensor's flash
// for I2C use: in I2C mode its own filters can't be set, so they stay as
// saved. Wiring: pin 5 open or at 3.3 V (UART mode), 3.3 V logic, 5 V.
//
// I2C: pin 5 at GND before power-on, pin 6 (data ready) on a GPIO. The mode,
// rate and enable registers are written at every start, not saved. Each
// rising edge of pin 6 is a new result: the registers are read right after
// it, and the sample time is the edge's kernel timestamp less half an
// output interval.
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

#include "i2c_gpio.hpp"
#include "serial.hpp"
#include "tfluna.hpp"

namespace flow::tfluna {

struct ReaderParams {
    std::string port;                   // serial port; empty with i2c_dev
    unsigned    baud = 115200;
    unsigned    rate_hz = 100;          // output rate; the sensor runs 500/n Hz, n = 2..500
    bool        mm = true;              // 9-byte/mm format (0x06) instead of cm (0x01); UART only
    bool        device_filter = false;  // keep the sensor's own Kalman and median filters; UART only
    int         dealias = -1;           // -1 leave as is, 0 single frequency, 1 dual (default); UART only
    bool        save_settings = false;  // UART: also save the settings in the sensor (ID 0x11)
    std::string i2c_dev;                // "/dev/i2c-7": I2C mode instead of the serial port
    uint8_t     i2c_addr = I2C_ADDR_DEFAULT;
    std::string drdy;                   // pin 6's GPIO, "gpiochip2:11"; needed with i2c_dev
    std::string log_path;               // CSV of every sample, "" for none
};

// Counts over a period (take_stats() starts a new one).
struct Stats {
    uint64_t n = 0, n_ok = 0;
    uint64_t n_status[N_STATUS] = {};  // by Status
    double   sum_m = 0, sum2_m = 0, sum_amp = 0;  // over the valid samples
    float    temp_c = 0;                          // the last one
    uint64_t n_bad_sum = 0;                       // since start
    // I2C, since start: failed register reads, data-ready edges more than
    // 1.5 intervals apart (results missed), waits of 3 intervals with no
    // edge, and reads whose tick hadn't changed (dropped).
    uint64_t n_bus_err = 0, n_late = 0, n_no_drdy = 0, n_same_tick = 0;
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
    bool               i2c() const { return i2c_mode_; }
    // The port or bus and address, for messages.
    const std::string& where() const { return where_; }
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
    void                 configure_i2c(const ReaderParams& p);
    void                 loop();
    void                 loop_i2c();
    void                 publish(const std::vector<Sample>& samples);  // buffer, stats, log, callback

    bool              i2c_mode_ = false;
    std::string       where_;
    SerialPort        ser_;
    I2cDevice         i2c_;
    GpioEdge          drdy_;
    int64_t           i2c_delay_ns_ = 0;  // edge to the middle of the averaged ranges
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
