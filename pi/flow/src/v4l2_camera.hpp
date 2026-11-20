// OV9281 capture straight from V4L2, for boards where libcamera can't drive
// the sensor (the ROCK 4SE's rkisp1: libcamera 0.4 has no ov9281 helper for
// it). See README.md, "Setup on a Radxa ROCK 4SE".
//
// The media pipeline (sensor -> CSI -> ISP in bypass -> mainpath, 8-bit raw)
// is set up beforehand by rock4se/setup.sh pipeline; this class takes the
// capture node's format as it finds it, scales the frame to the processing
// size, and sets exposure, gain and frame rate through the sensor's controls.
//
// Timing: the driver's line time ((width + hblank) / pixel_rate) is about 4 %
// short in the binned 640x400 mode, so the frame period is measured over the
// first 100 frames and the blanking and exposure are set again from it.
// sensor_ns is the ISP's frame-end time (CLOCK_BOOTTIME, the same as
// CLOCK_MONOTONIC on a board that never suspends), not the exposure start.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "frame_source.hpp"

namespace flow {

class V4l2Camera : public FrameSource {
public:
    V4l2Camera() = default;
    ~V4l2Camera() override;
    V4l2Camera(const V4l2Camera&) = delete;
    V4l2Camera& operator=(const V4l2Camera&) = delete;

    // Throws std::runtime_error on failure.
    void open(const CameraParams& p);
    void start() override;
    void stop() override;
    bool read(Frame& f, int timeout_ms) override;
    uint64_t n_dropped() const override { return n_dropped_; }

    const std::string& video_path() const { return video_; }
    const std::string& subdev_path() const { return subdev_; }

private:
    struct Buf {
        void*  mem = nullptr;
        size_t len = 0;
    };

    int  get_ctrl(uint32_t id) const;
    bool set_ctrl(uint32_t id, int value);
    void apply_timing();  // vblank and exposure from line_s_

    CameraParams     p_;
    std::string      video_, subdev_;
    int              vfd_ = -1, sfd_ = -1;
    unsigned         w_ = 0, h_ = 0, stride_ = 0;  // capture format
    std::vector<Buf> bufs_;
    bool             running_ = false;

    double  line_s_ = 0;  // sensor line period
    int     exp_lines_ = 0;
    bool    timing_checked_ = false;
    int64_t t_first_ns_ = 0;
    uint32_t seq_first_ = 0;
    int     n_timed_ = 0;

    bool     have_seq_ = false;
    uint32_t last_seq_ = 0;
    uint64_t n_dropped_ = 0;
};

}  // namespace flow
