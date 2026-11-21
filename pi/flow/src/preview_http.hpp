// Camera preview web page, for aiming and focusing the camera when the front
// unit is mounted (plans/front-camera-unit.md). Read-only: it shows the image
// and figures and changes nothing.
//
//   /            the page: the image refreshed about 4 times a second, the
//                tracked features, the focus figure with its peak, exposure
//   /frame.jpg   a new frame as JPEG (sensor size where the capture has it)
//   /stats.json  figures for the last frame /frame.jpg served
//
// One thread serves one request at a time. The flow loop never waits for it:
// it checks wanted() once a frame and, only then, hands a copy over with
// offer(). JPEG encoding and the figures are done in the server's thread,
// which runs at a lower priority than the flow.
#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>

namespace flow {

// What the flow loop knows about the frame it hands over.
struct PreviewInfo {
    uint32_t seq = 0;
    int      proc_w = 0, proc_h = 0;  // processing size, the points' frame
    int      n_tracked = 0;
    bool     valid = false;
    bool     phase = false;  // the flow came from the phase-correlation fallback
    int      exposure_us = 0;
    float    gain = 0;
    uint64_t dropped = 0;
    // From the last 5 s stats line (0 before the first one).
    double msg_per_s = 0, pct_valid = 0, flow_ms = 0;
    // TF-Luna (flow --lidar): the range at this frame, and from the 5 s line
    // its rate and valid share.
    bool     lidar = false;
    bool     range_valid = false, range_held = false;
    float    range_m = 0;
    unsigned range_amp = 0;
    double   lidar_hz = 0, lidar_pct_ok = 0;
};

class PreviewServer {
public:
    PreviewServer() = default;
    ~PreviewServer() { stop(); }
    PreviewServer(const PreviewServer&) = delete;
    PreviewServer& operator=(const PreviewServer&) = delete;

    // Listens on port (all addresses). False with err filled if it can't.
    bool start(uint16_t port, std::string& err);
    void stop();
    bool running() const { return fd_ >= 0; }

    // True while a request waits for a frame. Cheap: one atomic load.
    bool wanted() const { return want_.load(std::memory_order_relaxed); }
    // Hands over a frame (copied) with the features located in it.
    void offer(const cv::Mat& img, const std::vector<cv::Point2f>& pts, const PreviewInfo& info);

    // http://ADDR:PORT/ for each IPv4 address of this host but loopback.
    static std::vector<std::string> urls(uint16_t port);

private:
    struct Snap {
        cv::Mat                  img;
        std::vector<cv::Point2f> pts;
        PreviewInfo              info;
        uint64_t                 n = 0;  // counts offers
    };

    void loop();
    void serve(int c);
    bool fresh(Snap& out, int timeout_ms);  // waits for the next offer
    std::string stats_json(const Snap& s) const;

    int                     fd_ = -1;
    std::thread             thread_;
    std::atomic<bool>       stop_{false};
    std::atomic<bool>       want_{false};
    std::mutex              mu_;
    std::condition_variable cv_;
    Snap                    latest_;  // under mu_
    Snap                    served_;  // the last frame /frame.jpg served (server thread only)
};

}  // namespace flow
