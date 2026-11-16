#include "camera.hpp"

#include <chrono>
#include <cstring>
#include <stdexcept>

#include <sys/mman.h>

#include <libcamera/libcamera.h>

namespace flow {

using namespace libcamera;

Camera::Camera() = default;

Camera::~Camera()
{
    stop();
    for (auto& m : maps_) munmap(m.second.first, m.second.second);
    requests_.clear();
    alloc_.reset();
    if (cam_) cam_->release();
    cam_.reset();
    if (cm_) cm_->stop();
}

void Camera::open(const CameraParams& p)
{
    p_ = p;
    cm_ = std::make_unique<CameraManager>();
    if (cm_->start() != 0) throw std::runtime_error("libcamera: camera manager didn't start");
    auto cams = cm_->cameras();
    if (p.index < 0 || static_cast<size_t>(p.index) >= cams.size()) {
        throw std::runtime_error("libcamera: camera " + std::to_string(p.index) + " not found (" +
                                 std::to_string(cams.size()) + " present)");
    }
    cam_ = cams[static_cast<size_t>(p.index)];
    if (cam_->acquire() != 0) throw std::runtime_error("libcamera: can't acquire " + cam_->id());

    // Main stream: YUV420 at the processing size. Raw stream sized 640x400 so
    // the pipeline picks the sensor's 640x400 mode (full view, fastest); its
    // buffers are never queued.
    cfg_ = cam_->generateConfiguration({StreamRole::VideoRecording, StreamRole::Raw});
    if (!cfg_ || cfg_->size() < 1) throw std::runtime_error("libcamera: no configuration");
    StreamConfiguration& sc = cfg_->at(0);
    sc.pixelFormat = formats::YUV420;
    sc.size = Size(static_cast<unsigned>(p.width), static_cast<unsigned>(p.height));
    sc.bufferCount = 6;
    if (cfg_->size() > 1) cfg_->at(1).size = Size(640, 400);
    if (cfg_->validate() == CameraConfiguration::Invalid) throw std::runtime_error("libcamera: invalid configuration");
    if (sc.size.width != static_cast<unsigned>(p.width) || sc.size.height != static_cast<unsigned>(p.height) ||
        sc.pixelFormat != formats::YUV420) {
        throw std::runtime_error("libcamera: adjusted the stream to " + sc.toString());
    }
    if (cam_->configure(cfg_.get()) != 0) throw std::runtime_error("libcamera: configure failed");
    stream_ = sc.stream();
    stride_ = sc.stride;

    alloc_ = std::make_unique<FrameBufferAllocator>(cam_);
    if (alloc_->allocate(stream_) < 0) throw std::runtime_error("libcamera: buffer allocation failed");
    for (const std::unique_ptr<FrameBuffer>& buf : alloc_->buffers(stream_)) {
        const FrameBuffer::Plane& y = buf->planes()[0];
        void* mem = mmap(nullptr, y.length, PROT_READ, MAP_SHARED, y.fd.get(), y.offset);
        if (mem == MAP_FAILED) throw std::runtime_error("libcamera: mmap failed");
        maps_[buf.get()] = {mem, y.length};
        std::unique_ptr<Request> req = cam_->createRequest();
        if (!req || req->addBuffer(stream_, buf.get()) != 0) throw std::runtime_error("libcamera: request setup failed");
        requests_.push_back(std::move(req));
    }
    cam_->requestCompleted.connect(this, &Camera::on_complete);
}

void Camera::start()
{
    ControlList ctrls(controls::controls);
    const int64_t frame_us = static_cast<int64_t>(1e6 / p_.fps);
    const int64_t limits[2] = {frame_us, frame_us};
    ctrls.set(controls::FrameDurationLimits, Span<const int64_t, 2>(limits));
    ctrls.set(controls::AeEnable, false);
    ctrls.set(controls::ExposureTime, p_.exposure_us);
    ctrls.set(controls::AnalogueGain, p_.gain);
    if (cam_->start(&ctrls) != 0) throw std::runtime_error("libcamera: start failed");
    {
        std::lock_guard<std::mutex> lk(mu_);
        running_ = true;
    }
    for (auto& r : requests_) cam_->queueRequest(r.get());
}

void Camera::stop()
{
    {
        std::lock_guard<std::mutex> lk(mu_);
        if (!running_) return;
        running_ = false;
        done_.clear();
    }
    cam_->stop();
}

// libcamera's thread. Keeps at most max_queued frames; older ones go back to
// the camera (counted as dropped, and seen by IntervalWatch as a long gap).
void Camera::on_complete(Request* req)
{
    if (req->status() == Request::RequestCancelled) return;
    Request* drop = nullptr;
    {
        std::lock_guard<std::mutex> lk(mu_);
        if (!running_) return;
        done_.push_back(req);
        if (static_cast<int>(done_.size()) > p_.max_queued) {
            drop = done_.front();
            done_.pop_front();
            n_dropped_++;
        }
    }
    cv_.notify_one();
    if (drop) requeue(drop);
}

void Camera::requeue(Request* req)
{
    req->reuse(Request::ReuseBuffers);
    cam_->queueRequest(req);
}

bool Camera::read(Frame& f, int timeout_ms)
{
    Request* req = nullptr;
    {
        std::unique_lock<std::mutex> lk(mu_);
        if (!cv_.wait_for(lk, std::chrono::milliseconds(timeout_ms), [this] { return !done_.empty(); })) return false;
        req = done_.front();
        done_.pop_front();
    }
    const FrameBuffer* buf = req->buffers().at(stream_);
    const uint8_t*     y = static_cast<const uint8_t*>(maps_.at(buf).first);
    // A new Mat every frame: the flow estimator still holds the previous one.
    f.grey = cv::Mat(p_.height, p_.width, CV_8UC1);
    for (int r = 0; r < p_.height; r++) std::memcpy(f.grey.ptr(r), y + static_cast<size_t>(r) * stride_, static_cast<size_t>(p_.width));
    const ControlList& md = req->metadata();
    f.sensor_ns = md.get(controls::SensorTimestamp).value_or(0);
    f.exposure_us = md.get(controls::ExposureTime).value_or(0);
    requeue(req);
    return true;
}

}  // namespace flow
