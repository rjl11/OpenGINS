#include "tfluna_reader.hpp"

#include <chrono>
#include <cmath>
#include <ctime>
#include <stdexcept>

namespace flow::tfluna {

double Stats::sd_m() const
{
    if (n_ok < 2) return 0;
    const double n = static_cast<double>(n_ok);
    const double v = (sum2_m - sum_m * sum_m / n) / (n - 1);
    return v > 0 ? std::sqrt(v) : 0;
}

int64_t Reader::now_ns()
{
    timespec t{};
    ::clock_gettime(CLOCK_MONOTONIC, &t);
    return static_cast<int64_t>(t.tv_sec) * 1000000000 + t.tv_nsec;
}

// Sends a command and waits for its reply (same ID); 3 tries of 300 ms.
// Data frames read meanwhile are dropped.
std::vector<uint8_t> Reader::transact(const std::vector<uint8_t>& cmd, uint8_t id)
{
    std::vector<Sample> samples;
    std::vector<Reply>  replies;
    uint8_t             b[256];
    for (int attempt = 0; attempt < 3; attempt++) {
        if (!ser_.write_all(cmd.data(), cmd.size())) throw std::runtime_error("TF-Luna: write failed");
        const int64_t until = now_ns() + 300000000;
        while (now_ns() < until) {
            const long n = ser_.read_some(b, sizeof b, 50);
            if (n < 0) throw std::runtime_error("TF-Luna: the port went away");
            if (n == 0) continue;
            replies.clear();
            parser_.feed(b, static_cast<size_t>(n), now_ns(), byte_ns_, samples, replies);
            samples.clear();
            for (const Reply& r : replies) {
                if (r.id == id) return r.payload;
            }
        }
    }
    char m[96];
    std::snprintf(m, sizeof m, "TF-Luna: no reply to command 0x%02x", id);
    throw std::runtime_error(m);
}

void Reader::start(const ReaderParams& p)
{
    if (p.rate_hz < 1 || p.rate_hz > 250) throw std::runtime_error("TF-Luna: rate must be 1..250 Hz");
    // The log first: failing here leaves the sensor untouched.
    if (!p.log_path.empty()) {
        log_ = std::fopen(p.log_path.c_str(), "w");
        if (!log_) throw std::runtime_error("can't write " + p.log_path);
        std::fprintf(log_, "t_mono_s,range_m,amp,temp_c,status\n");
    }
    try {
        configure(p);
    } catch (...) {
        if (log_) {
            std::fclose(log_);
            log_ = nullptr;
        }
        throw;
    }
    stop_ = false;
    gone_ = false;
    thread_ = std::thread([this] { loop(); });
}

void Reader::configure(const ReaderParams& p)
{
    ser_.open(p.port, p.baud);
    byte_ns_ = static_cast<int64_t>(10e9 / p.baud);  // 8N1: 10 bits a byte

    // Quiet the data stream first, so the replies stand alone, and drop
    // what was already on its way.
    {
        const auto off = cmd_output_enable(false);
        if (!ser_.write_all(off.data(), off.size()))
            throw std::runtime_error("TF-Luna: can't write to " + p.port);
        uint8_t       b[256];
        const int64_t until = now_ns() + 1000000000;
        while (now_ns() < until) {
            const long n = ser_.read_some(b, sizeof b, 60);
            if (n < 0) throw std::runtime_error("TF-Luna: " + p.port + " went away");
            if (n == 0) break;
        }
    }
    // From here a failure would leave the sensor silent: switch its output
    // back on (best effort) before passing the error on.
    try {
        configure_quiet(p);
    } catch (...) {
        const auto on = cmd_output_enable(true);
        ser_.write_all(on.data(), on.size());
        parser_.accept_replies(false);
        throw;
    }
}

void Reader::configure_quiet(const ReaderParams& p)
{
    parser_.accept_replies(true);
    try {
        const auto v = transact(cmd_get_version(), ID_GET_VERSION);
        if (v.size() >= 3) version_ = std::to_string(v[2]) + "." + std::to_string(v[1]) + "." + std::to_string(v[0]);
    } catch (const std::runtime_error&) {
        throw std::runtime_error("no answer from a TF-Luna on " + p.port + " at " + std::to_string(p.baud) +
                                 " baud (pin 5 open for UART? TX/RX crossed? 5 V?)");
    }
    {
        const auto r = transact(cmd_sample_freq(static_cast<uint16_t>(p.rate_hz)), ID_SAMPLE_FREQ);
        rate_hz_ = r.size() >= 2 ? static_cast<unsigned>(r[0] | (r[1] << 8)) : p.rate_hz;
        if (rate_hz_ == 0) rate_hz_ = p.rate_hz;
    }
    {
        const uint8_t fmt = p.mm ? FMT_9B_MM : FMT_9B_CM;
        const auto    r = transact(cmd_output_format(fmt), ID_OUTPUT_FORMAT);
        if (r.empty() || r[0] != fmt) throw std::runtime_error("TF-Luna: output format not taken");
        parser_.set_unit_m(p.mm ? 0.001 : 0.01);
    }
    transact(cmd_filter(p.device_filter ? 0x03 : 0x00), ID_FILTER_BIT_MAP);
    if (p.dealias >= 0) transact(cmd_dealias(p.dealias != 0), ID_DEALIAS_EN);
    transact(cmd_output_enable(true), ID_OUTPUT_EN);
    parser_.accept_replies(false);

    // Each output is the mean of the ranges of the last interval (500 Hz
    // inside): its middle is half an interval before the output.
    parser_.set_meas_delay_ns(static_cast<int64_t>(0.5e9 / rate_hz_));
}

void Reader::stop()
{
    stop_ = true;
    if (thread_.joinable()) thread_.join();
    if (log_) {
        std::fclose(log_);
        log_ = nullptr;
    }
}

void Reader::loop()
{
    std::vector<Sample> samples;
    std::vector<Reply>  replies;
    uint8_t             b[256];
    while (!stop_) {
        const long n = ser_.read_some(b, sizeof b, 200);
        if (n < 0) {
            std::lock_guard<std::mutex> l(mu_);
            stats_.gone = true;
            gone_ = true;
            return;
        }
        if (n == 0) continue;
        // Taken right after the read returns, as the last byte's arrival
        // (late by the wake-up, and on a USB adapter by its latency).
        const int64_t t = now_ns();
        samples.clear();
        replies.clear();
        parser_.feed(b, static_cast<size_t>(n), t, byte_ns_, samples, replies);
        {
            // Also with no sample: a stream of bad sums must still show.
            std::lock_guard<std::mutex> l(mu_);
            for (const Sample& s : samples) {
                buf_.push(s);
                stats_.n++;
                stats_.n_status[static_cast<int>(s.status)]++;
                stats_.temp_c = s.temp_c;
                if (s.ok()) {
                    stats_.n_ok++;
                    stats_.sum_m += s.range_m;
                    stats_.sum2_m += static_cast<double>(s.range_m) * s.range_m;
                    stats_.sum_amp += s.amp;
                }
            }
            stats_.n_bad_sum = parser_.n_bad_sum();
        }
        for (const Sample& s : samples) {
            if (log_)
                std::fprintf(log_, "%.6f,%.3f,%u,%.1f,%s\n", static_cast<double>(s.t_ns) * 1e-9,
                             static_cast<double>(s.range_m), s.amp, static_cast<double>(s.temp_c),
                             status_name(s.status));
            if (on_sample) on_sample(s, on_sample_arg);
        }
    }
}

Range Reader::at(int64_t t_ns, int64_t half_window_ns, int64_t hold_ns) const
{
    std::lock_guard<std::mutex> l(mu_);
    return buf_.at(t_ns, half_window_ns, hold_ns);
}

Stats Reader::take_stats()
{
    std::lock_guard<std::mutex> l(mu_);
    Stats s = stats_;
    const bool     gone = stats_.gone;
    const uint64_t bad = stats_.n_bad_sum;
    stats_ = Stats{};
    stats_.gone = gone;
    stats_.n_bad_sum = bad;
    return s;
}

}  // namespace flow::tfluna
