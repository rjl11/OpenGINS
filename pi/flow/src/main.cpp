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
//   flow ... --lidar /dev/ttyUSB0     also read a TF-Luna and give each frame
//                                     the range at its time (tfluna_reader.hpp;
//                                     shown in --dry-run, the 5 s line and the
//                                     preview page; with --tm171 it is the
//                                     height the velocity is scaled by)
//   flow ... --tm171 /dev/ttyACM2     front camera unit: de-rotate each frame
//                                     pair with the TM171 gyro here and send
//                                     FLOW_MSG_CAM_VEL (camera velocity and
//                                     height) instead of the raw displacement
//                                     (tm171_reader.hpp; plans/front-camera-unit.md)
//
// Any of these also serves the camera preview page on port 8080 (aiming and
// focusing the camera, preview_http.hpp) unless --no-preview is given; in a
// build with FLOW_PREVIEW_DEFAULT=OFF only with --preview.
//
// Protocol: common/flow_link.h. Design: plans/optical-flow-aiding.md.
// Same behaviour as python/main.py, which stays as the reference.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <ctime>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <memory>
#include <mutex>
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
#include "preview_http.hpp"
#include "serial.hpp"
#include "tfluna_reader.hpp"
#include "tm171_reader.hpp"
#include "udp.hpp"
#include "v4l2_camera.hpp"
#ifdef FLOW_WITH_LIBCAMERA
#include "camera.hpp"
#endif

#ifndef FLOW_PREVIEW_DEFAULT
#define FLOW_PREVIEW_DEFAULT 1
#endif

namespace {

std::atomic<bool> g_stop{false};
void on_signal(int) { g_stop = true; }

// The unit's settings and the Pico's heartbeat (FLOW_MSG_UNIT_CFG), from the
// UDP thread or the Pico's USB output; read by the frame loop.
struct PicoCfg {
    std::mutex          mu;
    bool                have = false;
    flow_msg_unit_cfg_t cfg{};
    int64_t             t_rx_ns = 0;
    uint64_t            n = 0;
    void put(const flow_msg_unit_cfg_t& c, int64_t t)
    {
        std::lock_guard<std::mutex> l(mu);
        cfg = c;
        have = true;
        t_rx_ns = t;
        n++;
    }
};
PicoCfg g_pico_cfg;

int64_t mono_ns()
{
    timespec t{};
    ::clock_gettime(CLOCK_MONOTONIC, &t);
    return static_cast<int64_t>(t.tv_sec) * 1000000000 + t.tv_nsec;
}

// A whole flow_link frame carrying FLOW_MSG_UNIT_CFG: stored, true.
bool take_unit_cfg(const uint8_t* b, size_t n, int64_t t)
{
    uint8_t             type = 0;
    const int           len = flow_link_check(b, n, &type);
    flow_msg_unit_cfg_t c{};
    if (len < 0 || type != FLOW_MSG_UNIT_CFG ||
        flow_link_decode_unit_cfg(&b[FLOW_LINK_HEADER_LEN], static_cast<size_t>(len), &c) != 0) {
        return false;
    }
    g_pico_cfg.put(c, t);
    return true;
}

// "flowcfg <hex>" on the Pico's USB output.
bool take_unit_cfg_line(const std::string& line, int64_t t)
{
    if (line.rfind("flowcfg ", 0) != 0) return false;
    std::vector<uint8_t> b;
    for (size_t i = 8; i + 1 < line.size(); i += 2) {
        const auto nib = [](char c) { return c >= '0' && c <= '9' ? c - '0' : c >= 'a' && c <= 'f' ? c - 'a' + 10 : -1; };
        const int  hi = nib(line[i]), lo = nib(line[i + 1]);
        if (hi < 0 || lo < 0) break;
        b.push_back(static_cast<uint8_t>(hi << 4 | lo));
    }
    return take_unit_cfg(b.data(), b.size(), t);
}

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
    bool        phase = true;          // phase-correlation fallback
    double      phase_min_resp = 0.2;  // its response gate
    bool        preview = FLOW_PREVIEW_DEFAULT;  // camera preview web page
    int         preview_port = 8080;
    std::string lidar;  // TF-Luna serial port, "" for none
    unsigned    lidar_baud = 115200;
    unsigned    lidar_rate = 100;
    bool        lidar_cm = false;
    bool        lidar_device_filter = false;
    std::string lidar_log;  // CSV of every TF-Luna sample
    std::string tm171;      // TM171 serial port, "" for none
    bool        tm171_save = false;
    double      cam_yaw_deg = 0;  // image top from the TM171's forward, clockwise from above
    double      height_m = 1.0;   // lens above level ground when there is no range
};

void usage()
{
    std::fprintf(stderr,
                 "usage: flow [--port DEV] [--baud N] [--usb] [--udp HOST[:PORT]] [--dry-run] [--frames DIR]\n"
                 "            [--size WxH]\n"
                 "            [--fps F] [--exposure-us N] [--gain G] [--camera N] [--triggered]\n"
                 "            [--calib FILE.json] [--corners N] [--loop] [--realtime] [--threads N]\n"
                 "            [--v4l2] [--video DEV] [--subdev DEV] [--no-phase] [--phase-min R]\n"
                 "            [--preview | --no-preview] [--preview-port N]\n"
                 "            [--lidar DEV] [--lidar-baud N] [--lidar-rate HZ] [--lidar-cm]\n"
                 "            [--lidar-device-filter] [--lidar-log FILE.csv]\n"
                 "            [--tm171 DEV] [--tm171-save] [--cam-yaw DEG] [--height M]\n"
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
                 "  --preview / --no-preview: the camera preview web page on --preview-port\n"
                 "         (default 8080) for aiming and focusing the camera; on by default\n"
                 "         unless built with -DFLOW_PREVIEW_DEFAULT=OFF\n"
                 "  --threads N: OpenCV worker threads (default 1; on 2026-11-15 OpenCV's own\n"
                 "               default ran 2.6 cores and was slower than one thread)\n"
                 "  --no-phase: no phase-correlation fallback when too few features are tracked\n"
                 "  --phase-min R: its minimum correlation response, 0..1 (default 0.2)\n"
                 "  --lidar DEV: a TF-Luna on this serial port (115200 unless --lidar-baud),\n"
                 "         set up at start: --lidar-rate (default 100 Hz), mm output, its own\n"
                 "         filters off (--lidar-cm, --lidar-device-filter change that). Each\n"
                 "         frame gets the median valid range within 2.5 lidar intervals of its\n"
                 "         time, else the last one up to 0.5 s old. --lidar-log writes every\n"
                 "         sample\n"
                 "  With --udp or --usb the Pico sends the unit's settings once a second\n"
                 "         (FLOW_MSG_UNIT_CFG: camera height, image direction, TM171 mounting);\n"
                 "         from the first one on they replace --height and --cam-yaw\n"
                 "  --tm171 DEV: a TM171 on this port (its USB port, e.g. /dev/ttyACM2): its\n"
                 "         settings are checked at start and fixed in RAM (--tm171-save: in\n"
                 "         flash). Each frame pair is then de-rotated with its gyro and sent as a\n"
                 "         camera velocity (FLOW_MSG_CAM_VEL), scaled by the range from --lidar or\n"
                 "         else --height (default 1.0 m), tilt-corrected with its roll and pitch.\n"
                 "         --cam-yaw is the image top from the TM171's x axis, clockwise seen\n"
                 "         from above (default 0). A frame its gyro doesn't cover goes as the raw\n"
                 "         displacement (FLOW_MSG_FLOW). Not with --frames\n");
}

Args parse(int argc, char** argv)
{
    Args        a;
    std::string k;
    try {
        for (int i = 1; i < argc; i++) {
            k = argv[i];
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
            else if (k == "--no-phase") a.phase = false;
            else if (k == "--phase-min") a.phase_min_resp = std::stod(val());
            else if (k == "--lidar") a.lidar = val();
            else if (k == "--lidar-baud") a.lidar_baud = static_cast<unsigned>(std::stoul(val()));
            else if (k == "--lidar-rate") a.lidar_rate = static_cast<unsigned>(std::stoul(val()));
            else if (k == "--lidar-cm") a.lidar_cm = true;
            else if (k == "--lidar-device-filter") a.lidar_device_filter = true;
            else if (k == "--lidar-log") a.lidar_log = val();
            else if (k == "--tm171") a.tm171 = val();
            else if (k == "--tm171-save") a.tm171_save = true;
            else if (k == "--cam-yaw") a.cam_yaw_deg = std::stod(val());
            else if (k == "--height") a.height_m = std::stod(val());
            else if (k == "--preview") a.preview = true;
            else if (k == "--no-preview") a.preview = false;
            else if (k == "--preview-port") {
                a.preview_port = std::stoi(val());
                a.preview = true;
            }
            else {
                usage();
                std::exit(k == "-h" || k == "--help" ? 0 : 2);
            }
        }
    } catch (const std::logic_error&) {  // std::stoi / stoul / stod: not a number, or out of range
        std::fprintf(stderr, "bad value for %s\n", k.c_str());
        usage();
        std::exit(2);
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
        f.mid_ns = f.sensor_ns;
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
        fp.phase_fallback = a.phase;
        fp.phase_min_response = a.phase_min_resp;
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
                uint8_t     b[512];
                std::string line;
                while (!g_stop) {
                    const long n = ser.read_some(b, sizeof b, 200);
                    if (n < 0) {
                        pico_gone = true;
                        return;
                    }
                    // Line by line: its "flowcfg" lines are taken, the rest
                    // is copied to stdout.
                    for (long i = 0; i < n; i++) {
                        const char c = static_cast<char>(b[i]);
                        if (c != '\n') {
                            if (line.size() < 4096) line += c;
                            continue;
                        }
                        if (!line.empty() && line.back() == '\r') line.pop_back();
                        if (!take_unit_cfg_line(line, mono_ns())) {
                            std::fwrite(line.data(), 1, line.size(), stdout);
                            std::fputc('\n', stdout);
                        }
                        line.clear();
                    }
                    std::fflush(stdout);
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
                    if (n < 0) {
                        // A lasting socket error (interface gone, ...) makes
                        // poll() return at once: wait instead of spinning.
                        std::this_thread::sleep_for(std::chrono::milliseconds(200));
                        continue;
                    }
                    if (n == 0) continue;
                    if (rx_ns == 0) {  // no kernel timestamp: the time now
                        timespec now{};
                        ::clock_gettime(CLOCK_MONOTONIC, &now);
                        rx_ns = static_cast<int64_t>(now.tv_sec) * 1000000000 + now.tv_nsec;
                    }
                    if (take_unit_cfg(b, static_cast<size_t>(n), rx_ns)) continue;
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

        // Not being able to serve the page is no reason to stop the flow.
        flow::PreviewServer preview;
        if (a.preview) {
            std::string err;
            const auto  port = static_cast<uint16_t>(a.preview_port);
            if (preview.start(port, err)) {
                for (const std::string& u : flow::PreviewServer::urls(port)) {
                    std::fprintf(stderr, "preview on %s\n", u.c_str());
                }
            } else {
                std::fprintf(stderr, "warning: no preview page: %s\n", err.c_str());
            }
        }
        flow::PreviewInfo pv;  // the 5 s figures, kept for the page

        // The TF-Luna: asked for, so not finding it is an error.
        flow::tfluna::Reader lidar;
        if (!a.lidar.empty()) {
            flow::tfluna::ReaderParams lp;
            lp.port = a.lidar;
            lp.baud = a.lidar_baud;
            lp.rate_hz = a.lidar_rate;
            lp.mm = !a.lidar_cm;
            lp.device_filter = a.lidar_device_filter;
            lp.log_path = a.lidar_log;
            lidar.start(lp);
            std::fprintf(stderr, "TF-Luna %s on %s, %u Hz\n", lidar.version().c_str(), a.lidar.c_str(),
                         lidar.rate_hz());
            pv.lidar = true;
        }
        // Range near a frame: the samples within 2.5 lidar intervals (25 ms
        // at 100 Hz), else the last valid one up to 0.5 s old. Samples after
        // the frame have mostly not arrived yet when it is processed, and
        // each is stamped half an interval back, so the newest one is often
        // 15-20 ms older than the frame (seen 2026-11-20 with a fake sensor:
        // a 15 ms window left a third of the frames "held").
        const int64_t     k_lidar_half_window_ns =
            lidar.running() ? static_cast<int64_t>(2.5e9 / lidar.rate_hz()) : 0;
        constexpr int64_t k_lidar_hold_ns = 500000000;
        // The port going away doesn't stop the flow (the range isn't sent
        // to the Pico yet): said once here, then "PORT GONE" on each 5 s line.
        bool lidar_gone_told = false;

        // The TM171 (front camera unit): asked for, so not finding it is an
        // error. Its gyro de-rotates the flow here, in this clock.
        flow::tm171::Reader imu;
        // Geometry: the command line until the Pico's settings come.
        float r_cb[9], r_bm[9];
        float height_m = static_cast<float>(a.height_m);
        float mount_rpy[3] = {0, 0, 0};
        flow::tm171::cam_to_body(static_cast<float>(a.cam_yaw_deg * M_PI / 180.0), r_cb);
        flow::tm171::mount_to_body(mount_rpy, r_bm);
        flow_msg_unit_cfg_t applied{};
        bool                have_applied = false;
        const bool          linked = !a.dry_run && (a.usb || !a.udp.empty());

        // One frame to the Pico: a datagram, a "flow <hex>" line on its USB
        // port, or raw bytes on the link UART. Called from the frame loop and
        // from the TM171's thread, so under a lock.
        std::mutex            send_mu;
        std::atomic<uint64_t> n_send_err{0};
        auto send_frame = [&](const uint8_t* buf, size_t n) {
            std::lock_guard<std::mutex> l(send_mu);
            bool ok;
            if (!a.udp.empty()) {
                ok = udp.send(buf, n);
            } else if (a.usb) {
                // "flow <hex>\n" (firmware/app/usb_log.c)
                static const char k_hex[] = "0123456789abcdef";
                char line[8 + 2 * (FLOW_LINK_HEADER_LEN + FLOW_LINK_MAX_PAYLOAD + FLOW_LINK_CRC_LEN)];
                size_t k = 0;
                for (const char* c = "flow "; *c; c++) line[k++] = *c;
                for (size_t i = 0; i < n; i++) {
                    line[k++] = k_hex[buf[i] >> 4];
                    line[k++] = k_hex[buf[i] & 15];
                }
                line[k++] = '\n';
                ok = ser.write_all(reinterpret_cast<const uint8_t*>(line), k);
            } else {
                ok = ser.write_all(buf, n);
            }
            if (!ok) n_send_err++;
        };
        // The TM171's samples to the Pico (the backup IMU and the chassis
        // tilt), whenever there is a link.
        std::atomic<uint64_t> n_batches{0};
        if (linked && !a.tm171.empty()) {
            imu.on_batch = [&](const flow_msg_imu_batch_t& b) {
                uint8_t      f[FLOW_LINK_HEADER_LEN + FLOW_LINK_MAX_PAYLOAD + FLOW_LINK_CRC_LEN];
                const size_t n = flow_link_encode_imu_batch(&b, f, sizeof f);
                if (n) {
                    send_frame(f, n);
                    n_batches++;
                }
            };
        }
        // Stops the TM171's thread before send_frame goes out of scope,
        // also when an exception leaves this block.
        struct ImuStop {
            flow::tm171::Reader& r;
            ~ImuStop() { r.stop(); }
        } imu_stop{imu};
        if (!a.tm171.empty()) {
            if (!a.frames.empty()) throw std::runtime_error("--tm171 needs the camera: --frames have made-up times");
            if (!(a.height_m > 0.05)) throw std::runtime_error("--height must be above 0.05 m");
            flow::tm171::ReaderParams ip;
            ip.port = a.tm171;
            ip.save_flash = a.tm171_save;
            imu.start(ip);
            std::fprintf(stderr, "TM171 node %u on %s, settings %s\n", imu.node_id(), a.tm171.c_str(),
                         imu.setup().c_str());
        }
        bool imu_gone_told = false;
        // TF-Luna 1-sigma for the message (estimate from the manual's
        // +-6 cm accuracy, not measured here).
        constexpr float k_range_sd_m = 0.03f;
        constexpr int64_t k_tilt_max_age_ns = 500000000;

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
        uint64_t            n_sent = 0, n_valid = 0, n_phase = 0, n_write_err = 0;
        uint64_t            n_cam_vel = 0, n_no_gyro = 0, n_no_tilt = 0, n_ranged = 0;
        double              proc_sum_ms = 0, proc_max_ms = 0;
        auto                t_report = std::chrono::steady_clock::now();

        auto report = [&](std::chrono::steady_clock::time_point now) {
            const double el = std::chrono::duration<double>(now - t_report).count();
            std::fprintf(stderr, "%.1f msg/s, %.0f %% valid, flow %.2f ms mean %.2f ms max, %d detections",
                         static_cast<double>(n_sent) / el,
                         n_sent ? 100.0 * static_cast<double>(n_valid) / static_cast<double>(n_sent) : 0.0,
                         n_sent ? proc_sum_ms / static_cast<double>(n_sent) : 0.0, proc_max_ms, est.n_detect());
            if (n_phase)
                std::fprintf(stderr, ", %.0f %% phase corr",
                             100.0 * static_cast<double>(n_phase) / static_cast<double>(n_sent));
            if (cam) std::fprintf(stderr, ", %llu dropped", static_cast<unsigned long long>(cam->n_dropped()));
            n_write_err = n_send_err.load();
            if (n_write_err) std::fprintf(stderr, ", %llu write errors", static_cast<unsigned long long>(n_write_err));
            if (sync_thread.joinable()) std::fprintf(stderr, ", %u clock sync replies", n_sync.exchange(0));
            if (imu.running()) {
                const flow::tm171::Stats is = imu.take_stats();
                if (linked) {
                    std::lock_guard<std::mutex> l(g_pico_cfg.mu);
                    if (!g_pico_cfg.have)
                        std::fprintf(stderr, ", no settings from the Pico yet");
                    else
                        std::fprintf(stderr, ", Pico settings %u (%.1f s ago, flow %s)", g_pico_cfg.cfg.seq,
                                     static_cast<double>(mono_ns() - g_pico_cfg.t_rx_ns) * 1e-9,
                                     (g_pico_cfg.cfg.flags & FLOW_CFG_F_FLOW_ON) ? "on" : "off");
                }
                if (n_batches.load()) std::fprintf(stderr, ", %.0f IMU batches/s", static_cast<double>(n_batches.exchange(0)) / el);
                std::fprintf(stderr, ", TM171 %.0f Hz %.1f C, %llu camera velocities (%llu ranged), %llu without gyro, %llu without tilt",
                             static_cast<double>(is.n_raw) / el, static_cast<double>(is.last_status.temp_c),
                             static_cast<unsigned long long>(n_cam_vel), static_cast<unsigned long long>(n_ranged),
                             static_cast<unsigned long long>(n_no_gyro), static_cast<unsigned long long>(n_no_tilt));
                if (is.crc_errors) std::fprintf(stderr, " crc %llu", static_cast<unsigned long long>(is.crc_errors));
                if (is.gone) std::fprintf(stderr, " TM171 PORT GONE");
                n_cam_vel = n_no_gyro = n_no_tilt = n_ranged = 0;
            }
            if (lidar.running()) {
                const flow::tfluna::Stats ls = lidar.take_stats();
                const double pct = ls.n ? 100.0 * static_cast<double>(ls.n_ok) / static_cast<double>(ls.n) : 0.0;
                std::fprintf(stderr, ", lidar %.0f Hz %.0f %% ok %.3f m sd %.3f amp %.0f %.0f C",
                             static_cast<double>(ls.n) / el, pct, ls.mean_m(), ls.sd_m(), ls.mean_amp(),
                             static_cast<double>(ls.temp_c));
                if (ls.n_bad_sum) std::fprintf(stderr, " %llu bad sums", static_cast<unsigned long long>(ls.n_bad_sum));
                if (ls.gone) std::fprintf(stderr, " PORT GONE");
                pv.lidar_hz = static_cast<double>(ls.n) / el;
                pv.lidar_pct_ok = pct;
            }
            std::fputc('\n', stderr);
            pv.msg_per_s = static_cast<double>(n_sent) / el;
            pv.pct_valid = n_sent ? 100.0 * static_cast<double>(n_valid) / static_cast<double>(n_sent) : 0.0;
            pv.flow_ms = n_sent ? proc_sum_ms / static_cast<double>(n_sent) : 0.0;
            n_sent = n_valid = n_phase = 0;
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
            // New settings from the Pico: the geometry they carry.
            {
                std::lock_guard<std::mutex> l(g_pico_cfg.mu);
                if (g_pico_cfg.have) {
                    const flow_msg_unit_cfg_t& c = g_pico_cfg.cfg;
                    const bool changed = !have_applied || c.height_m != applied.height_m ||
                                         c.cam_yaw_rad != applied.cam_yaw_rad ||
                                         std::memcmp(c.imu_rpy_rad, applied.imu_rpy_rad, sizeof c.imu_rpy_rad) != 0 ||
                                         c.flags != applied.flags;
                    if (changed) {
                        if (c.height_m > 0.05f) height_m = c.height_m;
                        flow::tm171::cam_to_body(c.cam_yaw_rad, r_cb);
                        for (int i = 0; i < 3; i++) mount_rpy[i] = c.imu_rpy_rad[i];
                        flow::tm171::mount_to_body(mount_rpy, r_bm);
                        constexpr double k = 180.0 / M_PI;
                        std::fprintf(stderr,
                                     "settings from the Pico: height %.2f m, image top %.1f deg, TM171 mounting "
                                     "%.1f %.1f %.1f deg, flow %s\n",
                                     static_cast<double>(height_m), c.cam_yaw_rad * k, mount_rpy[0] * k,
                                     mount_rpy[1] * k, mount_rpy[2] * k, (c.flags & FLOW_CFG_F_FLOW_ON) ? "on" : "off");
                    }
                    applied = c;
                    have_applied = true;
                }
            }
            // The range at the frame's mid-exposure (not sensor_ns: libcamera
            // stamps the exposure start, V4L2 the frame end). Recorded frames
            // have made-up times, so they take the newest samples instead.
            flow::tfluna::Range rng;
            if (lidar.running()) {
                if (lidar.gone() && !lidar_gone_told) {
                    std::fprintf(stderr, "TF-Luna: %s went away, no more ranges\n", a.lidar.c_str());
                    lidar_gone_told = true;
                }
                const int64_t t = folder ? flow::tfluna::Reader::now_ns() : f.mid_ns;
                rng = lidar.at(t, k_lidar_half_window_ns, k_lidar_hold_ns);
            }
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
                // With FLOW_F_PHASE, n_tracked carries the response x 1000.
                m.n_tracked = r.phase ? static_cast<uint16_t>(std::lround(std::clamp(r.response, 0.0, 1.0) * 1000))
                                      : static_cast<uint16_t>(std::min(r.n_tracked, 0xFFFF));
                m.exposure_100us = static_cast<uint8_t>(std::min((f.exposure_us + 50) / 100, 255));
                m.flags = watch.classify(dt_ns);
                if (r.valid) m.flags |= FLOW_F_VALID;
                if (r.phase) m.flags |= FLOW_F_PHASE;
                if (a.triggered) m.flags |= FLOW_F_TRIGGERED;

                // The front camera unit's own conversion: rotation over the
                // exposure middles, then the height.
                flow_msg_cam_vel_t cv{};
                bool               use_cv = false;
                if (imu.running()) {
                    if (imu.gone() && !imu_gone_told) {
                        std::fprintf(stderr, "TM171: %s went away, raw flow from now on\n", a.tm171.c_str());
                        imu_gone_told = true;
                    }
                    float dth_m[3], dth[3];
                    const int64_t dmid_ns = f.mid_ns - prev.mid_ns;
                    if (dmid_ns > 0 && imu.rotation(prev.mid_ns, f.mid_ns, dth_m)) {
                        flow::tm171::mul3(r_bm, dth_m, dth);  // TM171 axes -> body
                        // The chassis tilt: the TM171's own less its mounting
                        // (small angles).
                        float roll = 0, pitch = 0;
                        if (imu.tilt(f.mid_ns, k_tilt_max_age_ns, &roll, &pitch)) {
                            roll -= mount_rpy[0];
                            pitch -= mount_rpy[1];
                        } else {
                            n_no_tilt++;
                        }
                        const float c = std::max(0.2f, std::cos(roll) * std::cos(pitch));
                        // The range is along the beam, parallel to the optical
                        // axis: the distance the flow scales by. Without it,
                        // the set height over the tilt.
                        const bool  ranged = rng.valid;
                        const float z = ranged ? rng.range_m : height_m / c;
                        const float dt = static_cast<float>(dmid_ns) * 1e-9f;
                        float       v[2];
                        flow::tm171::derotate(r_cb, m.disp_x_rad, m.disp_y_rad, dth, z, dt, v);
                        cv.frame_seq = m.frame_seq;
                        cv.pi_time_us = m.pi_time_us;
                        cv.pi_dt_us = m.pi_dt_us;
                        cv.vel_x_mps = v[0];
                        cv.vel_y_mps = v[1];
                        cv.stddev_mps = z * m.stddev_rad / dt;
                        cv.height_m = z * c;
                        cv.height_sd_m = ranged ? k_range_sd_m * c : 0.0f;
                        cv.n_tracked = m.n_tracked;
                        cv.exposure_100us = m.exposure_100us;
                        cv.flags = static_cast<uint8_t>(m.flags | (ranged ? FLOW_F_RANGE : 0u));
                        use_cv = true;
                        n_cam_vel++;
                        if (ranged) n_ranged++;
                    } else {
                        n_no_gyro++;
                    }
                }

                if (a.dry_run) {
                    std::printf("seq %u dt %u us disp %+.6f %+.6f sd %.2e n %u flags 0x%02x", m.frame_seq, m.pi_dt_us,
                                static_cast<double>(m.disp_x_rad), static_cast<double>(m.disp_y_rad),
                                static_cast<double>(m.stddev_rad), m.n_tracked, m.flags);
                    if (use_cv)
                        std::printf(" vel %+.3f %+.3f m/s sd %.3f height %.3f m%s", static_cast<double>(cv.vel_x_mps),
                                    static_cast<double>(cv.vel_y_mps), static_cast<double>(cv.stddev_mps),
                                    static_cast<double>(cv.height_m), (cv.flags & FLOW_F_RANGE) ? " ranged" : "");
                    else if (imu.running())
                        std::printf(" vel - (no gyro)");
                    if (lidar.running()) {
                        if (rng.valid)
                            std::printf(" range %.3f m%s n %d amp %u age %.1f ms", static_cast<double>(rng.range_m),
                                        rng.held ? " held" : "", rng.n, rng.amp,
                                        static_cast<double>(rng.age_ns) * 1e-6);
                        else
                            std::printf(" range -");
                    }
                    std::putchar('\n');
                } else {
                    uint8_t buf[FLOW_LINK_HEADER_LEN + FLOW_LINK_MAX_PAYLOAD + FLOW_LINK_CRC_LEN];
                    size_t  n = use_cv ? flow_link_encode_cam_vel(&cv, buf, sizeof buf)
                                       : flow_link_encode_flow(&m, buf, sizeof buf);
                    send_frame(buf, n);
                }
                n_sent++;
                if (r.valid) n_valid++;
                if (r.valid && r.phase) n_phase++;
            }
            const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();

            // The preview page asked for a frame: at the sensor size where the
            // capture has it (asked for here, so it comes with the next frame).
            if (preview.running() && preview.wanted()) {
                const bool full = !f.full.empty();
                if (full || !cam || !cam->request_full()) {
                    pv.seq = seq;
                    pv.proc_w = f.grey.cols;
                    pv.proc_h = f.grey.rows;
                    pv.n_tracked = r.n_tracked;
                    pv.valid = r.valid;
                    pv.phase = r.phase;
                    pv.exposure_us = f.exposure_us;
                    pv.gain = a.gain;
                    pv.dropped = cam ? cam->n_dropped() : 0;
                    pv.range_valid = rng.valid;
                    pv.range_held = rng.held;
                    pv.range_m = rng.range_m;
                    pv.range_amp = rng.amp;
                    preview.offer(full ? f.full : f.grey, est.points(), pv);
                }
            }
            proc_sum_ms += ms;
            proc_max_ms = std::max(proc_max_ms, ms);
            prev = f;
            have_prev = true;

            const auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration<double>(now - t_report).count() >= 5.0) report(now);
        }
        if (n_sent) report(std::chrono::steady_clock::now());
        if (cam) cam->stop();
        lidar.stop();
        imu.stop();
    } catch (const std::exception& e) {
        std::fprintf(stderr, "error: %s\n", e.what());
        return 1;
    }
    return 0;
}
