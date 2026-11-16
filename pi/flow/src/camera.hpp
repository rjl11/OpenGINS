// OV9281 capture through libcamera (Raspberry Pi OS's own ov9281 driver).
//
// The sensor runs its 640x400 mode (full field of view, up to 309 fps RAW8)
// and the ISP scales it to the processing size; the Y plane of the YUV420
// output is the grey frame. Built only when libcamera is found.
//
// Not yet checked on the camera: that the Raw stream size selects the
// 640x400 sensor mode, the rates reached, and trigger mode (frames only
// arrive when the Pico pulses TRIG; libcamera's own timeout must be raised,
// see README).
#pragma once

#include <condition_variable>
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace libcamera {
class Camera;
class CameraConfiguration;
class CameraManager;
class FrameBuffer;
class FrameBufferAllocator;
class Request;
class Stream;
}  // namespace libcamera

namespace flow {

struct Frame {
    cv::Mat grey;           // 8-bit, one channel, owned (copied out of the buffer)
    int64_t sensor_ns = 0;  // SensorTimestamp: start of exposure, Pi monotonic clock
    int     exposure_us = 0;
};

struct CameraParams {
    int    index = 0;
    int    width = 320, height = 200;  // processing size
    double fps = 100.0;                // free-running rate
    int    exposure_us = 1000;
    float  gain = 4.0f;
    int    max_queued = 4;  // frames waiting for the flow; older ones are dropped
};

class Camera {
public:
    Camera();
    ~Camera();
    Camera(const Camera&) = delete;
    Camera& operator=(const Camera&) = delete;

    // Throws std::runtime_error on failure.
    void open(const CameraParams& p);
    void start();
    void stop();
    // Waits up to timeout_ms for the next frame. False on timeout.
    bool read(Frame& f, int timeout_ms);

    uint64_t n_dropped() const { return n_dropped_; }

private:
    void on_complete(libcamera::Request* req);
    void requeue(libcamera::Request* req);

    CameraParams                                      p_;
    std::unique_ptr<libcamera::CameraManager>         cm_;
    std::shared_ptr<libcamera::Camera>                cam_;
    std::unique_ptr<libcamera::CameraConfiguration>   cfg_;
    std::unique_ptr<libcamera::FrameBufferAllocator>  alloc_;
    libcamera::Stream*                                stream_ = nullptr;
    unsigned                                          stride_ = 0;
    std::vector<std::unique_ptr<libcamera::Request>>  requests_;
    std::map<const libcamera::FrameBuffer*, std::pair<void*, size_t>> maps_;

    std::mutex                       mu_;
    std::condition_variable          cv_;
    std::deque<libcamera::Request*>  done_;
    bool                             running_ = false;
    uint64_t                         n_dropped_ = 0;
};

}  // namespace flow
