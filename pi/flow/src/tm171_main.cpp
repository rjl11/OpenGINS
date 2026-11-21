// tm171: bench tool for the TM171 on the front camera unit.
//
//   tm171 /dev/ttyACM2              check the settings (fixed in RAM if needed),
//                                   then one line a second until Ctrl-C
//   tm171 /dev/ttyACM2 --save       the same, but the target goes to flash
//   tm171 /dev/ttyACM2 --seconds 10
//
// Each line: packets per second (raw, RPY, Status), the gyro mean and
// 1-sigma per FRD axis, roll / pitch, temperature, internal rate, QoS, CRC
// errors. tm171_reader.hpp says what the setup does.
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>

#include "tm171_reader.hpp"

namespace {
std::atomic<bool> g_stop{false};
void              on_signal(int) { g_stop = true; }
}  // namespace

int main(int argc, char** argv)
{
    flow::tm171::ReaderParams p;
    double                    seconds = 0;
    for (int i = 1; i < argc; i++) {
        const std::string k = argv[i];
        if (k == "--save") p.save_flash = true;
        else if (k == "--seconds" && i + 1 < argc) seconds = std::atof(argv[++i]);
        else if (k == "--baud" && i + 1 < argc) p.baud = static_cast<unsigned>(std::atoi(argv[++i]));
        else if (p.port.empty() && k[0] != '-') p.port = k;
        else {
            std::fprintf(stderr, "usage: tm171 PORT [--save] [--seconds N] [--baud N]\n");
            return 2;
        }
    }
    if (p.port.empty()) {
        std::fprintf(stderr, "usage: tm171 PORT [--save] [--seconds N] [--baud N]\n");
        return 2;
    }
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);
    flow::tm171::Reader r;
    try {
        r.start(p);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "error: %s\n", e.what());
        return 1;
    }
    const flow::tm171::Setting& s = r.setting();
    std::printf("TM171 node %u on %s: settings %s; switches 0x%08x, %u Hz, baud word %u, silent %u s\n", r.node_id(),
                p.port.c_str(), r.setup().c_str(), flow::tm171::switches(s), flow::tm171::odr_hz(s),
                flow::tm171::baud_100(s), flow::tm171::silent_s(s));
    std::fflush(stdout);
    const auto t0 = std::chrono::steady_clock::now();
    auto       last = t0;
    while (!g_stop) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        const auto   now = std::chrono::steady_clock::now();
        const double el = std::chrono::duration<double>(now - last).count();
        last = now;
        const flow::tm171::Stats st = r.take_stats();
        const double             n = static_cast<double>(st.n_raw);
        std::printf("raw %.0f/s rpy %.0f/s status %.0f/s  gyro", n / el, static_cast<double>(st.n_rpy) / el,
                    static_cast<double>(st.n_status) / el);
        for (int i = 0; i < 3; i++) {
            const double m = n ? st.gyr_sum[i] / n : 0;
            const double v = n > 1 ? (st.gyr_sum2[i] - st.gyr_sum[i] * st.gyr_sum[i] / n) / (n - 1) : 0;
            std::printf(" %+.4f (%.4f)", m, v > 0 ? std::sqrt(v) : 0.0);
        }
        std::printf(" rad/s  roll %+.2f pitch %+.2f deg  %.1f C  %u Hz  QoS %u  crc %llu", static_cast<double>(st.last_rpy.roll_deg),
                    static_cast<double>(st.last_rpy.pitch_deg), static_cast<double>(st.last_status.temp_c),
                    st.last_status.rate_hz, st.last_status.qos, static_cast<unsigned long long>(st.crc_errors));
        if (st.clock_jumps) std::printf("  clock jumps %llu", static_cast<unsigned long long>(st.clock_jumps));
        if (st.gone) std::printf("  PORT GONE");
        std::printf("\n");
        std::fflush(stdout);
        if (st.gone) return 1;
        if (seconds > 0 && std::chrono::duration<double>(now - t0).count() >= seconds) break;
    }
    r.stop();
    return 0;
}
