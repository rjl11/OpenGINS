#include "v4l2_camera.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <stdexcept>

#include <fcntl.h>
#include <linux/v4l2-subdev.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <opencv2/imgproc.hpp>

namespace flow {

namespace {

constexpr int kBuffers = 6;
constexpr int kTimingFrames = 100;  // frames measured before the one timing correction
constexpr int kExposureOffset = 25; // lines the OV9281 needs between exposure and frame length

int xioctl(int fd, unsigned long req, void* arg)
{
    int r;
    do r = ioctl(fd, req, arg);
    while (r < 0 && errno == EINTR);
    return r;
}

[[noreturn]] void fail(const std::string& what)
{
    throw std::runtime_error("v4l2: " + what + ": " + std::strerror(errno));
}

// /dev node of the first V4L2 device or subdevice whose name starts with prefix.
std::string find_node(const std::string& prefix)
{
    namespace fs = std::filesystem;
    std::error_code ec;
    for (const auto& e : fs::directory_iterator("/sys/class/video4linux", ec)) {
        std::ifstream f(e.path() / "name");
        std::string   name;
        if (std::getline(f, name) && name.rfind(prefix, 0) == 0) return "/dev/" + e.path().filename().string();
    }
    return {};
}

int64_t get_ctrl64(int fd, uint32_t id)
{
    v4l2_ext_control  c{};
    v4l2_ext_controls cs{};
    c.id = id;
    cs.which = V4L2_CTRL_WHICH_CUR_VAL;
    cs.count = 1;
    cs.controls = &c;
    if (xioctl(fd, VIDIOC_G_EXT_CTRLS, &cs) < 0) fail("reading control " + std::to_string(id));
    return c.value64;
}

}  // namespace

V4l2Camera::~V4l2Camera()
{
    stop();
    for (Buf& b : bufs_) {
        if (b.mem) munmap(b.mem, b.len);
    }
    if (vfd_ >= 0) {
        v4l2_requestbuffers rb{};
        rb.count = 0;
        rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        rb.memory = V4L2_MEMORY_MMAP;
        xioctl(vfd_, VIDIOC_REQBUFS, &rb);
        close(vfd_);
    }
    if (sfd_ >= 0) close(sfd_);
}

int V4l2Camera::get_ctrl(uint32_t id) const
{
    v4l2_control c{};
    c.id = id;
    if (xioctl(sfd_, VIDIOC_G_CTRL, &c) < 0) fail("reading control " + std::to_string(id));
    return c.value;
}

bool V4l2Camera::set_ctrl(uint32_t id, int value)
{
    v4l2_queryctrl q{};
    q.id = id;
    if (xioctl(sfd_, VIDIOC_QUERYCTRL, &q) == 0) value = std::clamp(value, q.minimum, q.maximum);
    v4l2_control c{};
    c.id = id;
    c.value = value;
    return xioctl(sfd_, VIDIOC_S_CTRL, &c) == 0;
}

void V4l2Camera::apply_timing()
{
    if (!p_.triggered) {
        // Frame length in lines = active height + vertical blanking.
        v4l2_subdev_format sf{};
        sf.which = V4L2_SUBDEV_FORMAT_ACTIVE;
        sf.pad = 0;
        if (xioctl(sfd_, VIDIOC_SUBDEV_G_FMT, &sf) < 0) fail("sensor format");
        const int lines = static_cast<int>(std::lround(1.0 / (p_.fps * line_s_)));
        if (!set_ctrl(V4L2_CID_VBLANK, lines - static_cast<int>(sf.format.height))) fail("setting vertical blanking");
    }
    // The exposure's range follows the frame length, so set it after the blanking.
    exp_lines_ = std::max(1, static_cast<int>(std::lround(p_.exposure_us * 1e-6 / line_s_)));
    if (!set_ctrl(V4L2_CID_EXPOSURE, exp_lines_)) fail("setting exposure");
    exp_lines_ = get_ctrl(V4L2_CID_EXPOSURE);
}

void V4l2Camera::open(const CameraParams& p)
{
    p_ = p;
    video_ = p.video.empty() ? find_node("rkisp1_mainpath") : p.video;
    subdev_ = p.subdev.empty() ? find_node("ov9281") : p.subdev;
    if (video_.empty()) throw std::runtime_error("v4l2: no rkisp1_mainpath capture node (driver loaded?), give --video");
    if (subdev_.empty()) throw std::runtime_error("v4l2: no ov9281 sensor subdevice, give --subdev");

    vfd_ = ::open(video_.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
    if (vfd_ < 0) fail("opening " + video_);
    sfd_ = ::open(subdev_.c_str(), O_RDWR | O_CLOEXEC);
    if (sfd_ < 0) fail("opening " + subdev_);

    v4l2_capability cap{};
    if (xioctl(vfd_, VIDIOC_QUERYCAP, &cap) < 0) fail("querycap " + video_);
    const uint32_t caps = (cap.capabilities & V4L2_CAP_DEVICE_CAPS) ? cap.device_caps : cap.capabilities;
    if (!(caps & V4L2_CAP_VIDEO_CAPTURE_MPLANE) || !(caps & V4L2_CAP_STREAMING)) {
        throw std::runtime_error("v4l2: " + video_ + " is not a multi-planar streaming capture node");
    }

    // The format is the one rock4se/setup.sh pipeline set: 8-bit, one plane.
    v4l2_format fmt{};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    if (xioctl(vfd_, VIDIOC_G_FMT, &fmt) < 0) fail("reading the format of " + video_);
    const v4l2_pix_format_mplane& pm = fmt.fmt.pix_mp;
    const uint32_t                pf = pm.pixelformat;
    const bool eight_bit = pf == V4L2_PIX_FMT_GREY || pf == V4L2_PIX_FMT_SRGGB8 || pf == V4L2_PIX_FMT_SGRBG8 ||
                           pf == V4L2_PIX_FMT_SGBRG8 || pf == V4L2_PIX_FMT_SBGGR8;
    if (!eight_bit || pm.num_planes != 1) {
        char fc[5] = {static_cast<char>(pf), static_cast<char>(pf >> 8), static_cast<char>(pf >> 16),
                      static_cast<char>(pf >> 24), 0};
        throw std::runtime_error("v4l2: " + video_ + " is " + fc + " " + std::to_string(pm.width) + "x" +
                                 std::to_string(pm.height) + ", not 8-bit raw: run rock4se/setup.sh pipeline");
    }
    w_ = pm.width;
    h_ = pm.height;
    stride_ = pm.plane_fmt[0].bytesperline;
    if (static_cast<int>(w_) < p.width || static_cast<int>(h_) < p.height) {
        throw std::runtime_error("v4l2: capture is " + std::to_string(w_) + "x" + std::to_string(h_) +
                                 ", smaller than the processing size");
    }

    v4l2_requestbuffers rb{};
    rb.count = kBuffers;
    rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    rb.memory = V4L2_MEMORY_MMAP;
    if (xioctl(vfd_, VIDIOC_REQBUFS, &rb) < 0) fail("requesting buffers");
    if (rb.count < 2) throw std::runtime_error("v4l2: got only " + std::to_string(rb.count) + " buffers");
    bufs_.resize(rb.count);
    for (unsigned i = 0; i < rb.count; i++) {
        v4l2_plane  pl{};
        v4l2_buffer b{};
        b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        b.memory = V4L2_MEMORY_MMAP;
        b.index = i;
        b.length = 1;
        b.m.planes = &pl;
        if (xioctl(vfd_, VIDIOC_QUERYBUF, &b) < 0) fail("querybuf");
        void* mem = mmap(nullptr, pl.length, PROT_READ | PROT_WRITE, MAP_SHARED, vfd_, pl.m.mem_offset);
        if (mem == MAP_FAILED) fail("mmap");
        bufs_[i] = {mem, pl.length};
    }

    // Nominal line time from the driver; corrected from the measured frame
    // period once streaming (see the header).
    const int64_t pixel_rate = get_ctrl64(sfd_, V4L2_CID_PIXEL_RATE);
    const int     hblank = get_ctrl(V4L2_CID_HBLANK);
    v4l2_subdev_format sf{};
    sf.which = V4L2_SUBDEV_FORMAT_ACTIVE;
    if (xioctl(sfd_, VIDIOC_SUBDEV_G_FMT, &sf) < 0) fail("sensor format");
    line_s_ = static_cast<double>(sf.format.width + static_cast<unsigned>(hblank)) / static_cast<double>(pixel_rate);
    // OV9282 driver: analogue_gain 16 is 1x (4 fractional bits).
    if (!set_ctrl(V4L2_CID_ANALOGUE_GAIN, static_cast<int>(std::lround(p.gain * 16.0f)))) fail("setting gain");
    apply_timing();
}

void V4l2Camera::start()
{
    if (running_) return;
    for (unsigned i = 0; i < bufs_.size(); i++) {
        v4l2_plane  pl{};
        v4l2_buffer b{};
        b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        b.memory = V4L2_MEMORY_MMAP;
        b.index = i;
        b.length = 1;
        b.m.planes = &pl;
        if (xioctl(vfd_, VIDIOC_QBUF, &b) < 0) fail("qbuf");
    }
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    if (xioctl(vfd_, VIDIOC_STREAMON, &type) < 0) {
        fail("streamon (EPIPE: formats along the pipeline don't match, run rock4se/setup.sh pipeline)");
    }
    running_ = true;
    have_seq_ = false;
    timing_checked_ = p_.triggered;
    n_timed_ = 0;
}

void V4l2Camera::stop()
{
    if (!running_) return;
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    xioctl(vfd_, VIDIOC_STREAMOFF, &type);  // also returns every queued buffer
    running_ = false;
}

bool V4l2Camera::read(Frame& f, int timeout_ms)
{
    pollfd pfd{vfd_, POLLIN, 0};
    const int r = poll(&pfd, 1, timeout_ms);
    if (r < 0 && errno != EINTR) fail("poll");
    if (r <= 0) return false;

    v4l2_plane  pl{};
    v4l2_buffer b{};
    b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    b.memory = V4L2_MEMORY_MMAP;
    b.length = 1;
    b.m.planes = &pl;
    if (xioctl(vfd_, VIDIOC_DQBUF, &b) < 0) {
        if (errno == EAGAIN) return false;
        fail("dqbuf");
    }
    // QBUF below rewrites b, so take what is needed from it first.
    const int64_t  t_ns = static_cast<int64_t>(b.timestamp.tv_sec) * 1000000000 + b.timestamp.tv_usec * 1000;
    const uint32_t seq = b.sequence;
    if (have_seq_ && seq != last_seq_ + 1) n_dropped_ += seq - last_seq_ - 1;
    have_seq_ = true;
    last_seq_ = seq;

    const bool bad = (b.flags & V4L2_BUF_FLAG_ERROR) != 0;
    if (!bad) {
        const cv::Mat raw(static_cast<int>(h_), static_cast<int>(w_), CV_8UC1, bufs_[b.index].mem, stride_);
        // A new Mat every frame: the flow estimator still holds the previous one.
        if (raw.cols == p_.width && raw.rows == p_.height) f.grey = raw.clone();
        else cv::resize(raw, f.grey, cv::Size(p_.width, p_.height), 0, 0, cv::INTER_AREA);
        f.sensor_ns = t_ns;
        f.exposure_us = static_cast<int>(std::lround(exp_lines_ * line_s_ * 1e6));
    }
    if (xioctl(vfd_, VIDIOC_QBUF, &b) < 0) fail("qbuf");
    if (bad) return false;

    // One correction of the line time from the measured frame period.
    if (!timing_checked_) {
        if (n_timed_ == 0) {
            t_first_ns_ = t_ns;
            seq_first_ = seq;
        }
        if (++n_timed_ > kTimingFrames) {
            timing_checked_ = true;
            // Sequence numbers count every sensor frame, delivered or not.
            const double period = static_cast<double>(t_ns - t_first_ns_) * 1e-9 /
                                  static_cast<double>(seq - seq_first_);
            const double want = 1.0 / p_.fps;
            if (std::fabs(period / want - 1.0) > 0.002) {
                line_s_ *= period / want;
                apply_timing();
                std::fprintf(stderr, "v4l2: frame period %.3f ms for %.3f ms wanted, line time now %.3f us\n",
                             period * 1e3, want * 1e3, line_s_ * 1e6);
            }
        }
    }
    return true;
}

}  // namespace flow
