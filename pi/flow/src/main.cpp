// Optical flow on the Pi, sent to the Pico over the serial link.
//
//   flow --port /dev/serial0          camera, send to the Pico
//   flow --dry-run                    camera, print messages
//   flow --dry-run --frames DIR       recorded PNGs (any Linux machine)
//   flow --dry-run --frames DIR --loop --realtime
//                                     replay them endlessly at --fps (soak test)
//
// Protocol: common/flow_link.h. Design: plans/optical-flow-aiding.md.
// Same behaviour as python/main.py, which stays as the reference.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "flow_link.h"
#include "flow_lk.hpp"
#include "interval_watch.hpp"
#include "serial.hpp"
#ifdef FLOW_WITH_LIBCAMERA
#include "camera.hpp"
#else
namespace flow {
struct Frame {
    cv::Mat grey;
    int64_t sensor_ns = 0;
    int     exposure_us = 0;
};
}  // namespace flow
#endif

namespace {

std::atomic<bool> g_stop{false};
void on_signal(int) { g_stop = true; }

struct Args {
    std::string port = "/dev/serial0";
    unsigned    baud = FLOW_LINK_BAUD;
    bool        dry_run = false;
    std::string frames;  // folder of PNGs instead of the camera
    bool        loop = false;      // replay the folder endlessly (frames kept in memory)
    bool        realtime = false;  // hand out folder frames at --fps in real time
    int         threads = 1;       // OpenCV worker threads (more only adds overhead here)
    int         width = 320, height = 200;
    double      fps = 100.0;
    int         exposure_us = 1000;
    float       gain = 4.0f;
    int         camera = 0;
    bool        triggered = false;
    std::string calib;
    int         corners = 60;
};

void usage()
{
    std::fprintf(stderr,
                 "usage: flow [--port DEV] [--baud N] [--dry-run] [--frames DIR] [--size WxH]\n"
                 "            [--fps F] [--exposure-us N] [--gain G] [--camera N] [--triggered]\n"
                 "            [--calib FILE.json] [--corners N] [--loop] [--realtime] [--threads N]\n"
                 "  --size is the processing size (default 320x200; the sensor runs 640x400)\n"
                 "  --fps is the free-running rate (default 100; ignored when triggered)\n"
                 "  --triggered: camera is in external-trigger mode (set by ov9281_trigger)\n"
                 "  --loop: replay --frames endlessly from memory (make the sequence a closed loop)\n"
                 "  --realtime: hand out --frames at --fps in real time, like the camera\n"
                 "  --threads N: OpenCV worker threads (default 1; on 2026-11-15 OpenCV's own\n"
                 "               default ran 2.6 cores and was slower than one thread)\n");
}

Args parse(int argc, char** argv)
{
    Args a;
    for (int i = 1; i < argc; i++) {
        std::string k = argv[i];
        auto        val = [&]() -> std::string {
            if (i + 1 >= argc) {
                usage();
                std::exit(2);
            }
            return argv[++i];
        };
        if (k == "--port") a.port = val();
        else if (k == "--baud") a.baud = static_cast<unsigned>(std::stoul(val()));
        else if (k == "--dry-run") a.dry_run = true;
        else if (k == "--frames") a.frames = val();
        else if (k == "--size") {
            std::string s = val();
            if (std::sscanf(s.c_str(), "%dx%d", &a.width, &a.height) != 2) {
                usage();
                std::exit(2);
            }
        } else if (k == "--fps") a.fps = std::stod(val());
        else if (k == "--exposure-us") a.exposure_us = std::stoi(val());
        else if (k == "--gain") a.gain = std::stof(val());
        else if (k == "--camera") a.camera = std::stoi(val());
        else if (k == "--triggered") a.triggered = true;
        else if (k == "--calib") a.calib = val();
        else if (k == "--corners") a.corners = std::stoi(val());
        else if (k == "--loop") a.loop = true;
        else if (k == "--realtime") a.realtime = true;
        else if (k == "--threads") a.threads = std::stoi(val());
        else {
            usage();
            std::exit(k == "-h" || k == "--help" ? 0 : 2);
        }
    }
    return a;
}

// PNG frames from a folder, as if the camera ran at fps. With loop, all
// frames are decoded once into memory and handed out endlessly, so decoding
// doesn't load the CPU during a soak test; with realtime, each frame is held
// back until its time at fps, so the flow runs at the camera's pace.
class FolderSource {
public:
    FolderSource(const std::string& dir, double fps, bool loop, bool realtime, int w, int h)
        : dt_ns_(static_cast<int64_t>(1e9 / fps)), loop_(loop), realtime_(realtime), w_(w), h_(h)
    {
        for (auto& e : std::filesystem::directory_iterator(dir)) {
            if (e.path().extension() == ".png") files_.push_back(e.path().string());
        }
        std::sort(files_.begin(), files_.end());
        if (files_.empty()) throw std::runtime_error("no .png files in " + dir);
        if (loop_) {
            for (const std::string& p : files_) mem_.push_back(load(p));
        }
        t0_ = std::chrono::steady_clock::now();
    }
    bool read(flow::Frame& f)
    {
        const size_t n = files_.size();
        if (!loop_ && n_ >= n) return false;
        if (realtime_) {
            std::this_thread::sleep_until(t0_ + std::chrono::nanoseconds(static_cast<int64_t>(n_) * dt_ns_));
        }
        f.grey = loop_ ? mem_[n_ % n] : load(files_[n_]);
        if (f.grey.empty()) return false;
        f.sensor_ns = static_cast<int64_t>(n_) * dt_ns_;
        f.exposure_us = 0;
        n_++;
        return true;
    }

private:
    cv::Mat load(const std::string& p) const
    {
        cv::Mat g = cv::imread(p, cv::IMREAD_GRAYSCALE);
        if (!g.empty() && (g.cols != w_ || g.rows != h_)) cv::resize(g, g, cv::Size(w_, h_), 0, 0, cv::INTER_AREA);
        return g;
    }

    std::vector<std::string>              files_;
    std::vector<cv::Mat>                  mem_;
    uint64_t                              n_ = 0;
    int64_t                               dt_ns_;
    bool                                  loop_, realtime_;
    int                                   w_, h_;
    std::chrono::steady_clock::time_point t0_;
};

}  // namespace

int main(int argc, char** argv)
{
    const Args a = parse(argc, argv);
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);
    cv::setNumThreads(a.threads);

    try {
        flow::Intrinsics intr;
        if (!a.calib.empty()) {
            intr = flow::Intrinsics::load(a.calib);
        } else {
            intr = flow::Intrinsics::nominal(a.width, a.height);
            std::fprintf(stderr, "warning: no --calib, using the nominal lens with no distortion model\n");
        }
        flow::FlowParams fp;
        fp.max_corners = a.corners;
        fp.min_keep = std::max(fp.min_tracked, a.corners / 2);
        flow::FlowEstimator est(intr, fp);

        flow::SerialPort ser;
        if (!a.dry_run) ser.open(a.port, a.baud);

        std::unique_ptr<FolderSource> folder;
#ifdef FLOW_WITH_LIBCAMERA
        std::unique_ptr<flow::Camera> cam;
#endif
        if (!a.frames.empty()) {
            folder = std::make_unique<FolderSource>(a.frames, a.fps, a.loop, a.realtime, a.width, a.height);
        } else {
#ifdef FLOW_WITH_LIBCAMERA
            flow::CameraParams cp;
            cp.index = a.camera;
            cp.width = a.width;
            cp.height = a.height;
            cp.fps = a.fps;
            cp.exposure_us = a.exposure_us;
            cp.gain = a.gain;
            cam = std::make_unique<flow::Camera>();
            cam->open(cp);
            cam->start();
#else
            std::fprintf(stderr, "built without libcamera: use --frames DIR\n");
            return 2;
#endif
        }

        flow::IntervalWatch watch;
        flow::Frame         f, prev;
        bool                have_prev = false;
        uint32_t            seq = 0;
        uint64_t            n_sent = 0, n_valid = 0, n_write_err = 0;
        double              proc_sum_ms = 0, proc_max_ms = 0;
        auto                t_report = std::chrono::steady_clock::now();

        auto report = [&](std::chrono::steady_clock::time_point now) {
            const double el = std::chrono::duration<double>(now - t_report).count();
            std::fprintf(stderr, "%.1f msg/s, %.0f %% valid, flow %.2f ms mean %.2f ms max, %d detections",
                         static_cast<double>(n_sent) / el,
                         n_sent ? 100.0 * static_cast<double>(n_valid) / static_cast<double>(n_sent) : 0.0,
                         n_sent ? proc_sum_ms / static_cast<double>(n_sent) : 0.0, proc_max_ms, est.n_detect());
#ifdef FLOW_WITH_LIBCAMERA
            if (cam) std::fprintf(stderr, ", %llu dropped", static_cast<unsigned long long>(cam->n_dropped()));
#endif
            if (n_write_err) std::fprintf(stderr, ", %llu write errors", static_cast<unsigned long long>(n_write_err));
            std::fputc('\n', stderr);
            n_sent = n_valid = 0;
            proc_sum_ms = proc_max_ms = 0;
            t_report = now;
        };

        while (!g_stop) {
            bool ok;
            if (folder) {
                ok = folder->read(f);
                if (!ok) break;
            } else {
#ifdef FLOW_WITH_LIBCAMERA
                ok = cam->read(f, 1000);
                if (!ok) continue;  // triggered and no pulse, or a stall
#endif
            }
            seq++;
            const auto t0 = std::chrono::steady_clock::now();
            flow::FlowResult r;
            if (est.update(f.grey, r) && have_prev) {
                const int64_t   dt_ns = f.sensor_ns - prev.sensor_ns;
                flow_msg_flow_t m{};
                m.frame_seq = seq;
                m.pi_time_us = static_cast<uint32_t>(f.sensor_ns / 1000);
                m.pi_dt_us = static_cast<uint32_t>(std::max<int64_t>(0, dt_ns / 1000));
                m.disp_x_rad = static_cast<float>(r.disp_x);
                m.disp_y_rad = static_cast<float>(r.disp_y);
                m.stddev_rad = static_cast<float>(r.stddev);
                m.n_tracked = static_cast<uint16_t>(std::min(r.n_tracked, 0xFFFF));
                m.exposure_100us = static_cast<uint8_t>(std::min((f.exposure_us + 50) / 100, 255));
                m.flags = watch.classify(dt_ns);
                if (r.valid) m.flags |= FLOW_F_VALID;
                if (a.triggered) m.flags |= FLOW_F_TRIGGERED;

                if (a.dry_run) {
                    std::printf("seq %u dt %u us disp %+.6f %+.6f sd %.2e n %u flags 0x%02x\n", m.frame_seq, m.pi_dt_us,
                                static_cast<double>(m.disp_x_rad), static_cast<double>(m.disp_y_rad),
                                static_cast<double>(m.stddev_rad), m.n_tracked, m.flags);
                } else {
                    uint8_t buf[FLOW_LINK_FLOW_FRAME_LEN];
                    size_t  n = flow_link_encode_flow(&m, buf, sizeof buf);
                    if (!ser.write_all(buf, n)) n_write_err++;
                }
                n_sent++;
                if (r.valid) n_valid++;
            }
            const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
            proc_sum_ms += ms;
            proc_max_ms = std::max(proc_max_ms, ms);
            prev = f;
            have_prev = true;

            const auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration<double>(now - t_report).count() >= 5.0) report(now);
        }
        if (n_sent) report(std::chrono::steady_clock::now());
#ifdef FLOW_WITH_LIBCAMERA
        if (cam) cam->stop();
#endif
    } catch (const std::exception& e) {
        std::fprintf(stderr, "error: %s\n", e.what());
        return 1;
    }
    return 0;
}
