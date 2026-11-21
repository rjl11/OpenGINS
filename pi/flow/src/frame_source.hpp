// What every camera capture hands the flow: a grey frame with its time.
#pragma once

#include <cstdint>
#include <string>

#include <opencv2/core.hpp>

namespace flow {

struct Frame {
    cv::Mat grey;           // 8-bit, one channel, at the processing size, owned
    int64_t sensor_ns = 0;  // capture time on the Pi's clock (see each source for which instant)
    // The middle of the exposure (of the middle row) on the same clock, the
    // same instant whatever the source; for matching other sensors' samples.
    int64_t mid_ns = 0;
    int     exposure_us = 0;
    cv::Mat full;           // sensor-size image, only after request_full(); else empty
};

struct CameraParams {
    int    index = 0;                  // libcamera: camera number
    int    width = 320, height = 200;  // processing size
    double fps = 100.0;                // free-running rate
    int    exposure_us = 1000;
    float  gain = 4.0f;
    bool   triggered = false;          // external trigger: the rate is the Pico's, fps is ignored
    int    max_queued = 4;             // libcamera: frames waiting for the flow; older ones are dropped
    std::string video;                 // V4L2: capture node ("" = find rkisp1_mainpath)
    std::string subdev;                // V4L2: sensor subdevice ("" = find the ov9281)
};

class FrameSource {
public:
    virtual ~FrameSource() = default;
    virtual void start() = 0;
    virtual void stop() = 0;
    // Waits up to timeout_ms for the next frame. False on timeout.
    virtual bool read(Frame& f, int timeout_ms) = 0;
    // Frames the sensor delivered that never reached read().
    virtual uint64_t n_dropped() const = 0;
    // Asks for the sensor-size image in Frame::full of the next frame read
    // (for the preview page). False if this source has none (the frame is
    // only at the processing size).
    virtual bool request_full() { return false; }
};

}  // namespace flow
