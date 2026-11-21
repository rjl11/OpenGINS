#include "tm171_reader.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <stdexcept>

namespace flow::tm171 {

int64_t Reader::now_ns()
{
    timespec t{};
    ::clock_gettime(CLOCK_MONOTONIC, &t);
    return static_cast<int64_t>(t.tv_sec) * 1000000000 + t.tv_nsec;
}

// A Setting request every 300 ms for up to 4 s: right after it enumerates,
// the USB port may not answer the first requests (plans/imu-tm210-config.md).
Setting Reader::read_setting()
{
    uint8_t             b[1024];
    std::vector<Packet> pk;
    const std::vector<uint8_t> req = request(CMD_SETTING);
    const int64_t       until = now_ns() + 4000000000;
    int64_t             next = 0;
    while (now_ns() < until) {
        if (now_ns() >= next) {
            if (!ser_.write_all(req.data(), req.size())) throw std::runtime_error("TM171: write failed");
            next = now_ns() + 300000000;
        }
        const long n = ser_.read_some(b, sizeof b, 50);
        if (n < 0) throw std::runtime_error("TM171: the port went away");
        pk.clear();
        parser_.feed(b, static_cast<size_t>(n), now_ns(), pk);
        for (const Packet& p : pk) {
            if (p.cmd == CMD_SETTING && p.payload.size() == SETTING_LEN) {
                node_ = p.from_id;
                Setting s;
                for (size_t i = 0; i < SETTING_LEN; i++) s[i] = p.payload[i];
                return s;
            }
        }
    }
    throw std::runtime_error("TM171: no Setting reply in 4 s");
}

// Written to the node ID (writes sent to ID 0 are ignored on a TM171), then
// the Acknowledge (CMD 13, byte 0 = 21) within 1.5 s; three tries.
void Reader::write_setting(const Setting& s)
{
    const std::vector<uint8_t> w = encode(CMD_SETTING, node_, s.data(), s.size());
    uint8_t                    b[1024];
    std::vector<Packet>        pk;
    for (int attempt = 0; attempt < 3; attempt++) {
        if (!ser_.write_all(w.data(), w.size())) throw std::runtime_error("TM171: write failed");
        const int64_t until = now_ns() + 1500000000;
        while (now_ns() < until) {
            const long n = ser_.read_some(b, sizeof b, 50);
            if (n < 0) throw std::runtime_error("TM171: the port went away");
            pk.clear();
            parser_.feed(b, static_cast<size_t>(n), now_ns(), pk);
            for (const Packet& p : pk) {
                if (p.cmd == CMD_ACK && !p.payload.empty() && p.payload[0] == CMD_SETTING) return;
            }
        }
    }
    throw std::runtime_error("TM171: no Acknowledge for the Setting write");
}

void Reader::start(const ReaderParams& p)
{
    ser_.open(p.port, p.baud);
    const Setting s = read_setting();
    if (!layout_ok(s)) throw std::runtime_error("TM171: Setting layout not recognised, nothing written");
    // Bits 17 and 18 don't say where a Setting came from: after a power
    // cycle from flash both read 0 (bench 2026-11-21). So a Setting that
    // matches is left alone, with or without save_flash.
    if (!matches_target(s)) {
        write_setting(target(s, p.save_flash));
        const Setting back = read_setting();
        if (!matches_target(back)) throw std::runtime_error("TM171: the read-back after the write is not the target");
        setting_ = back;
        setup_ = p.save_flash ? "saved to flash" : "written to RAM";
    } else {
        setting_ = s;
        setup_ = "matched";
    }

    // Raw output within 2 s.
    uint8_t             b[4096];
    std::vector<Packet> pk;
    const int64_t       until = now_ns() + 2000000000;
    unsigned            n_raw = 0;
    while (now_ns() < until && n_raw < 50) {
        const long n = ser_.read_some(b, sizeof b, 50);
        if (n < 0) throw std::runtime_error("TM171: the port went away");
        pk.clear();
        parser_.feed(b, static_cast<size_t>(n), now_ns(), pk);
        for (const Packet& q : pk) n_raw += q.cmd == CMD_RAW;
    }
    if (n_raw < 50) throw std::runtime_error("TM171: no raw output after the setup");
    stop_ = false;
    gone_ = false;
    thread_ = std::thread([this] { loop(); });
}

void Reader::stop()
{
    stop_ = true;
    if (thread_.joinable()) thread_.join();
}

void Reader::handle(const Packet& pk)
{
    if (pk.cmd == CMD_RAW) {
        Raw r;
        if (!decode_raw(pk.payload, r)) return;
        const int64_t t = clock_.map(r.ts_us, pk.t_ns);
        {
            std::lock_guard<std::mutex> l(mu_);
            stats_.n_raw++;
            for (int i = 0; i < 3; i++) {
                stats_.gyr_sum[i] += r.gyr_frd[i];
                stats_.gyr_sum2[i] += static_cast<double>(r.gyr_frd[i]) * r.gyr_frd[i];
            }
            if (t != 0) gyro_.push(t, r.gyr_frd);
        }
        if (t == 0) {
            batch_gap_ = true;  // a clock restart: the samples before don't join on
            return;
        }
        if (!on_batch) return;
        // More than two periods since the last sample: some were lost.
        if (last_raw_ns_ != 0 && t - last_raw_ns_ > 2500000) batch_gap_ = true;
        last_raw_ns_ = t;
        // A gap inside a batch would break its fixed spacing: send what is
        // there and start a new one.
        if (batch_.n > 0 && (batch_gap_ || t - batch_t0_ns_ > 60000000)) {
            batch_.seq = batch_seq_++;
            on_batch(batch_);
            batch_.n = 0;
        }
        if (batch_.n == 0) {
            batch_t0_ns_ = t;
            batch_.t0_us = static_cast<uint32_t>(t / 1000);
            batch_.flags = batch_gap_ ? FLOW_IMU_F_GAP : 0;
            batch_gap_ = false;
        }
        flow_imu_sample_t& s = batch_.s[batch_.n++];
        s.dt_us = static_cast<uint16_t>((t - batch_t0_ns_) / 1000);
        // Gravity output (like the TM210's): FLU gravity -> FRD specific
        // force is (-x, y, z) times g (firmware/drivers/tm210, acc_is_gravity).
        constexpr float k_g = 9.80665f;
        s.acc[0] = -r.acc_flu_g[0] * k_g;
        s.acc[1] = r.acc_flu_g[1] * k_g;
        s.acc[2] = r.acc_flu_g[2] * k_g;
        for (int i = 0; i < 3; i++) s.gyr[i] = r.gyr_frd[i];
        if (batch_.n == FLOW_IMU_BATCH_MAX) {
            batch_.seq = batch_seq_++;
            on_batch(batch_);
            batch_.n = 0;
        }
    } else if (pk.cmd == CMD_RPY) {
        Rpy r;
        if (!decode_rpy(pk.payload, r)) return;
        std::lock_guard<std::mutex> l(mu_);
        stats_.n_rpy++;
        stats_.last_rpy = r;
        rpy_ = r;
        rpy_t_ns_ = pk.t_ns;
    } else if (pk.cmd == CMD_STATUS) {
        Status s;
        if (!decode_status(pk.payload, s)) return;
        std::lock_guard<std::mutex> l(mu_);
        stats_.n_status++;
        stats_.last_status = s;
    } else {
        std::lock_guard<std::mutex> l(mu_);
        stats_.n_other++;
    }
}

void Reader::loop()
{
    uint8_t             b[4096];
    std::vector<Packet> pk;
    while (!stop_) {
        const long n = ser_.read_some(b, sizeof b, 100);
        if (n < 0) {
            gone_ = true;
            std::lock_guard<std::mutex> l(mu_);
            stats_.gone = true;
            return;
        }
        if (n == 0) continue;
        pk.clear();
        parser_.feed(b, static_cast<size_t>(n), now_ns(), pk);
        for (const Packet& p : pk) handle(p);
        std::lock_guard<std::mutex> l(mu_);
        stats_.crc_errors = parser_.crc_errors();
        stats_.clock_jumps = clock_.jumps();
    }
}

bool Reader::rotation(int64_t ta_ns, int64_t tb_ns, float dth[3]) const
{
    std::lock_guard<std::mutex> l(mu_);
    return gyro_.integral(ta_ns, tb_ns, dth);
}

bool Reader::tilt(int64_t t_ns, int64_t max_age_ns, float* roll_rad, float* pitch_rad) const
{
    std::lock_guard<std::mutex> l(mu_);
    if (rpy_t_ns_ == 0 || std::llabs(t_ns - rpy_t_ns_) > max_age_ns) return false;
    constexpr float k = 3.14159265f / 180.0f;
    *roll_rad = rpy_.roll_deg * k;
    *pitch_rad = rpy_.pitch_deg * k;
    return true;
}

Stats Reader::take_stats()
{
    std::lock_guard<std::mutex> l(mu_);
    Stats s = stats_;
    const Status st = stats_.last_status;
    const Rpy    rp = stats_.last_rpy;
    const bool   g = stats_.gone;
    stats_ = Stats{};
    stats_.last_status = st;
    stats_.last_rpy = rp;
    stats_.gone = g;
    stats_.crc_errors = s.crc_errors;
    stats_.clock_jumps = s.clock_jumps;
    return s;
}

}  // namespace flow::tm171
