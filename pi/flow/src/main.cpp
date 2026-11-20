// Optical flow on the Pi, sent to the Pico over the serial link.
//
//   flow --port /dev/serial0          camera, send to the Pico
//   flow --usb --port /dev/ttyACM0    camera, send on the Pico's USB port
//                                     (bench test, no link wired: frames as
//                                     "flow <hex>" lines, the Pico's own
//                                     output copied to stdout)
//   flow --udp 192.168.5.124          camera, one frame per UDP datagram to
//                                     the Pico's W5500 (front camera unit,
//                                     port 5800 unless HOST:PORT is given;
//                                     answers the Pico's clock sync requests)
//   flow --dry-run                    camera, print messages
//   flow --dry-run --v4l2             camera through V4L2 (ROCK 4SE, no libcamera)
//   flow --dry-run --frames DIR       recorded PNGs (any Linux machine)
//   flow --dry-run --frames DIR --loop --realtime
//                                     replay them endlessly at --fps (soak test)
//
// Protocol: common/flow_link.h. Design: plans/optical-flow-aiding.md.
// Same behaviour as python/main.py, which stays as the reference.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <ctime>
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
#include "frame_source.hpp"
#include "serial.hpp"
#include "udp.hpp"
#include "v4l2_camera.hpp"
#ifdef FLOW_WITH_LIBCAMERA
#include "camera.hpp"
#endif

namespace {

std::atomic<bool> g_stop{false};
void on_signal(int) { g_stop = true; }

struct Args {
    std::string port = "/dev/serial0";
    unsigned    baud = FLOW_LINK_BAUD;
    bool        dry_run = false;
    bool        usb = false;  // the Pico's USB port: hex lines, Pico output to stdout
    std::string udp;          // HOST[:PORT]: UDP datagrams to the Pico's W5500
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
#ifdef FLOW_WITH_LIBCAMERA
    bool        v4l2 = false;  // capture through V4L2 instead of libcamera
#else
    bool        v4l2 = true;
#endif
    std::string video, subdev;  // V4L2 nodes ("" = found by name)
    std::string calib;
    int         corners = 60;
};

void usage()
{
    std::fprintf(stderr,
                 "usage: flow [--port DEV] [--baud N] [--usb] [--udp HOST[:PORT]] [--dry-run] [--frames DIR]\n"
                 "            [--size WxH]\n"
                 "            [--fps F] [--exposure-us N] [--gain G] [--camera N] [--triggered]\n"
                 "            [--calib FILE.json] [--corners N] [--loop] [--realtime] [--threads N]\n"
                 "            [--v4l2] [--video DEV] [--subdev DEV]\n"
                 "  --usb: --port is the Pico's USB port (/dev/ttyACM0): each frame goes as a\n"
                 "         \"flow <hex>\" line and the Pico's output is copied to stdout; for\n"
                 "         bench tests before the link UART is wired (without --triggered the\n"
                 "         Pico times the frames by the Pi's clock)\n"
                 "  --udp: send each frame as one UDP datagram to the Pico's Ethernet (W5500),\n"
                 "         port 5800 by default (front camera unit). Answers the Pico's clock sync\n"
                 "         requests from the camera's clock (CLOCK_MONOTONIC), so the Pico times\n"
                 "         frames by their sensor timestamps; not with --frames\n"
                 "  --size is the processing size (default 320x200; the sensor runs 640x400)\n"
                 "  --fps is the free-running rate (default 100; ignored when triggered)\n"
                 "  --triggered: camera is in external-trigger mode (set by ov9281_trigger)\n"
                 "  --v4l2: capture through V4L2, not libcamera (ROCK 4SE; run rock4se/setup.sh\n"
                 "          pipeline first); --video / --subdev name the capture node and the\n"
                 "          sensor subdevice (default: found as rkisp1_mainpath and ov9281)\n"
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
        else if (k == "--usb") a.usb = true;
        else if (k == "--udp") a.udp = val();
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
        else if (k == "--v4l2") a.v4l2 = true;
        else if (k == "--video") {
            a.video = val();
            a.v4l2 = true;
        } else if (k == "--subdev") {
            a.subdev = val();
            a.v4l2 = true;
        }
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

        if (a.usb && !a.udp.empty()) throw std::runtime_error("--usb and --udp exclude each other");
        flow::SerialPort ser;
        flow::UdpSender  udp;
        if (!a.dry_run && !a.udp.empty()) udp.open(a.udp, 5800);  // APP_ETH_FRONT_PORT
        else if (!a.dry_run) ser.open(a.port, a.baud);

        // On the Pico's USB port its status text has to be read, or its
        // printf stalls once the CDC buffer is full while the port is open.
        std::thread pico_out;
        std::atomic<bool> pico_gone{false};
        if (!a.dry_run && a.usb) {
            pico_out = std::thread([&] {
                uint8_t b[512];
                while (!g_stop) {
                    const long n = ser.read_some(b, sizeof b, 200);
                    if (n < 0) {
                        pico_gone = true;
                        return;
                    }
                    if (n > 0) {
                        std::fwrite(b, 1, static_cast<size_t>(n), stdout);
                        std::fflush(stdout);
                    }
                }
            });
        }
        struct Joiner {
            std::thread& t;
            ~Joiner()
            {
                g_stop = true;
                if (t.joinable()) t.join();
            }
        } join_pico{pico_out};

        // Over UDP the Pico sends clock sync requests (common/flow_link.h,
        // FLOW_MSG_TIME_REQ); each is answered at once with this clock,
        // the one V4L2 stamps buffers with. Not for --frames: their
        // timestamps are made up, so the Pico should keep timing them by
        // arrival.
        std::thread sync_thread;
        std::atomic<uint32_t> n_sync{0};
        if (!a.dry_run && !a.udp.empty() && a.frames.empty()) {
            sync_thread = std::thread([&] {
                uint8_t b[128];
                while (!g_stop) {
                    int64_t    rx_ns = 0;
                    const long n = udp.recv(b, sizeof b, 200, &rx_ns);
                    if (n <= 0) continue;
                    if (rx_ns == 0) {  // no kernel timestamp: the time now
                        timespec now{};
                        ::clock_gettime(CLOCK_MONOTONIC, &now);
                        rx_ns = static_cast<int64_t>(now.tv_sec) * 1000000000 + now.tv_nsec;
                    }
                    uint8_t  type = 0;
                    uint32_t seq = 0;
                    const int len = flow_link_check(b, static_cast<size_t>(n), &type);
                    if (len < 0 || type != FLOW_MSG_TIME_REQ ||
                        flow_link_decode_time_req(&b[FLOW_LINK_HEADER_LEN], static_cast<size_t>(len), &seq) != 0) {
                        continue;
                    }
                    flow_msg_time_resp_t r{};
                    r.seq = seq;
                    r.unit_rx_us = static_cast<uint64_t>(rx_ns / 1000);
                    timespec tx{};
                    ::clock_gettime(CLOCK_MONOTONIC, &tx);
                    const int64_t turn_ns = static_cast<int64_t>(tx.tv_sec) * 1000000000 + tx.tv_nsec - rx_ns;
                    r.turnaround_us = static_cast<uint32_t>(std::max<int64_t>(0, turn_ns / 1000));
                    uint8_t out[FLOW_LINK_TIME_RESP_FRAME_LEN];
                    const size_t k = flow_link_encode_time_resp(&r, out, sizeof out);
                    if (udp.send(out, k)) n_sync++;
                }
            });
        }
        Joiner join_sync{sync_thread};

        std::unique_ptr<FolderSource>      folder;
        std::unique_ptr<flow::FrameSource> cam;
        if (!a.frames.empty()) {
            folder = std::make_unique<FolderSource>(a.frames, a.fps, a.loop, a.realtime, a.width, a.height);
        } else {
            flow::CameraParams cp;
            cp.index = a.camera;
            cp.width = a.width;
            cp.height = a.height;
            cp.fps = a.fps;
            cp.exposure_us = a.exposure_us;
            cp.gain = a.gain;
            cp.triggered = a.triggered;
            cp.video = a.video;
            cp.subdev = a.subdev;
            if (a.v4l2) {
                auto v = std::make_unique<flow::V4l2Camera>();
                v->open(cp);
                std::fprintf(stderr, "v4l2: capture %s, sensor %s\n", v->video_path().c_str(), v->subdev_path().c_str());
                cam = std::move(v);
            } else {
#ifdef FLOW_WITH_LIBCAMERA
                auto c = std::make_unique<flow::Camera>();
                c->open(cp);
                cam = std::move(c);
#endif
            }
            cam->start();
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
            if (cam) std::fprintf(stderr, ", %llu dropped", static_cast<unsigned long long>(cam->n_dropped()));
            if (n_write_err) std::fprintf(stderr, ", %llu write errors", static_cast<unsigned long long>(n_write_err));
            if (sync_thread.joinable()) std::fprintf(stderr, ", %u clock sync replies", n_sync.exchange(0));
            std::fputc('\n', stderr);
            n_sent = n_valid = 0;
            proc_sum_ms = proc_max_ms = 0;
            t_report = now;
        };

        while (!g_stop) {
            if (pico_gone) throw std::runtime_error("the Pico's USB port went away (rebooted?)");
            bool ok;
            if (folder) {
                ok = folder->read(f);
                if (!ok) break;
            } else {
                ok = cam->read(f, 1000);
                if (!ok) continue;  // triggered and no pulse, or a stall
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
                    if (!a.udp.empty()) {
                        if (!udp.send(buf, n)) n_write_err++;
                    } else if (a.usb) {
                        // "flow <hex>\n" (firmware/app/usb_log.c)
                        static const char k_hex[] = "0123456789abcdef";
                        char              line[8 + 2 * FLOW_LINK_FLOW_FRAME_LEN];
                        size_t            k = 0;
                        for (const char* c = "flow "; *c; c++) line[k++] = *c;
                        for (size_t i = 0; i < n; i++) {
                            line[k++] = k_hex[buf[i] >> 4];
                            line[k++] = k_hex[buf[i] & 15];
                        }
                        line[k++] = '\n';
                        if (!ser.write_all(reinterpret_cast<const uint8_t*>(line), k)) n_write_err++;
                    } else if (!ser.write_all(buf, n)) {
                        n_write_err++;
                    }
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
        if (cam) cam->stop();
    } catch (const std::exception& e) {
        std::fprintf(stderr, "error: %s\n", e.what());
        return 1;
    }
    return 0;
}
