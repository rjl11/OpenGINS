// TF-Luna on its own, for bench checks (plans/front-camera-unit.md,
// "Verification", TF-Luna): every sample as a CSV line on stdout, and once a
// second on stderr the rate, the valid share, the mean range with its
// standard deviation, the mean Amp and the temperature.
//
//   tfluna --port /dev/ttyUSB0                 100 Hz, mm format, own filters off
//   tfluna --port /dev/ttyS4 --rate 250 --seconds 60 > board_1m.csv
//   tfluna --port /dev/ttyUSB0 --save --seconds 2   save filters off etc. before I2C use
//   tfluna --i2c /dev/i2c-7 --drdy gpiochip2:11     I2C, pin 6 on ROCK 4SE header pin 7
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <thread>

#include "tfluna_reader.hpp"

namespace {

std::atomic<bool> g_stop{false};
void on_signal(int) { g_stop = true; }

void usage()
{
    std::fprintf(stderr,
                 "usage: tfluna --port DEV [--baud N] [--rate HZ] [--cm] [--device-filter]\n"
                 "              [--single-freq | --dual-freq] [--save] [--seconds S] [--quiet]\n"
                 "       tfluna --i2c DEV --drdy CHIP:LINE [--addr 0xNN] [--rate HZ] [--seconds S]\n"
                 "              [--quiet]\n"
                 "  --baud: what the sensor runs at (115200 unless saved otherwise)\n"
                 "  --save: also save the settings in the sensor's flash (UART only). Do it once\n"
                 "         before moving the sensor to I2C, where its own filters can't be set\n"
                 "  --i2c: I2C mode (pin 5 at GND before power-on), cm only; --drdy: the GPIO its\n"
                 "         pin 6 goes to (gpiochip2:11 = ROCK 4SE header pin 7); --addr: 0x10\n"
                 "         unless saved otherwise\n"
                 "  --rate: output rate, 1..250 Hz (default 100; the sensor uses 500/n Hz)\n"
                 "  --cm: 9-byte/cm format instead of mm\n"
                 "  --device-filter: keep the sensor's own Kalman and median filters (default off)\n"
                 "  --single-freq / --dual-freq: set the de-aliasing mode (default: leave it)\n"
                 "  --quiet: no CSV, only the 1 s summary\n");
}

using flow::tfluna::Status;

void print_sample(const flow::tfluna::Sample& s, void*)
{
    std::printf("%.6f,%.3f,%u,%.1f,%s\n", static_cast<double>(s.t_ns) * 1e-9, static_cast<double>(s.range_m), s.amp,
                static_cast<double>(s.temp_c), flow::tfluna::status_name(s.status));
}

}  // namespace

int main(int argc, char** argv)
{
    flow::tfluna::ReaderParams p;
    double                     seconds = 0;
    bool                       quiet = false;
    for (int i = 1; i < argc; i++) {
        const std::string k = argv[i];
        auto              val = [&]() -> std::string {
            if (i + 1 >= argc) {
                usage();
                std::exit(2);
            }
            return argv[++i];
        };
        try {
            if (k == "--port") p.port = val();
            else if (k == "--i2c") p.i2c_dev = val();
            else if (k == "--drdy") p.drdy = val();
            else if (k == "--addr") {
                const unsigned long a = std::stoul(val(), nullptr, 0);
                if (a < 0x08 || a > 0x77) throw std::out_of_range("--addr");
                p.i2c_addr = static_cast<uint8_t>(a);
            } else if (k == "--save") p.save_settings = true;
            else if (k == "--baud") p.baud = static_cast<unsigned>(std::stoul(val()));
            else if (k == "--rate") p.rate_hz = static_cast<unsigned>(std::stoul(val()));
            else if (k == "--cm") p.mm = false;
            else if (k == "--device-filter") p.device_filter = true;
            else if (k == "--single-freq") p.dealias = 0;
            else if (k == "--dual-freq") p.dealias = 1;
            else if (k == "--seconds") seconds = std::stod(val());
            else if (k == "--quiet") quiet = true;
            else {
                usage();
                return k == "-h" || k == "--help" ? 0 : 2;
            }
        } catch (const std::logic_error&) {  // std::stoul / std::stod: not a number, or out of range
            std::fprintf(stderr, "bad value for %s\n", k.c_str());
            usage();
            return 2;
        }
    }
    if (p.port.empty() == p.i2c_dev.empty()) {  // one of the two
        usage();
        return 2;
    }
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    try {
        flow::tfluna::Reader r;
        if (!quiet) {
            r.on_sample = print_sample;
            std::printf("t_mono_s,range_m,amp,temp_c,status\n");
        }
        r.start(p);
        std::fprintf(stderr, "TF-Luna %s on %s, %u Hz%s\n", r.version().c_str(), r.where().c_str(), r.rate_hz(),
                     p.save_settings ? ", settings saved in the sensor" : "");
        const auto t0 = std::chrono::steady_clock::now();
        while (!g_stop) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            const flow::tfluna::Stats s = r.take_stats();
            std::fprintf(stderr,
                         "%llu Hz, %.0f %% ok, range %.3f m sd %.3f m, amp %.0f, %.1f C"
                         " (low amp %llu, overload %llu, <0.2 m %llu, >8 m %llu, bad sums %llu)\n",
                         static_cast<unsigned long long>(s.n),
                         s.n ? 100.0 * static_cast<double>(s.n_ok) / static_cast<double>(s.n) : 0.0, s.mean_m(),
                         s.sd_m(), s.mean_amp(), static_cast<double>(s.temp_c),
                         static_cast<unsigned long long>(s.n_status[static_cast<size_t>(Status::LowAmp)]),
                         static_cast<unsigned long long>(s.n_status[static_cast<size_t>(Status::Overload)]),
                         static_cast<unsigned long long>(s.n_status[static_cast<size_t>(Status::TooClose)]),
                         static_cast<unsigned long long>(s.n_status[static_cast<size_t>(Status::TooFar)]),
                         static_cast<unsigned long long>(s.n_bad_sum));
            if (r.i2c())
                std::fprintf(stderr, "  i2c since start: bus errors %llu, missed %llu, no data ready %llu, same tick %llu\n",
                             static_cast<unsigned long long>(s.n_bus_err), static_cast<unsigned long long>(s.n_late),
                             static_cast<unsigned long long>(s.n_no_drdy),
                             static_cast<unsigned long long>(s.n_same_tick));
            std::fflush(stdout);
            if (r.gone()) throw std::runtime_error(r.i2c() ? "the sensor stopped answering" : "the port went away");
            if (seconds > 0 && std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() >= seconds)
                break;
        }
        r.stop();
    } catch (const std::exception& e) {
        std::fprintf(stderr, "error: %s\n", e.what());
        return 1;
    }
    return 0;
}
