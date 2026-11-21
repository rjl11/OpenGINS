// Optical flow between consecutive grey frames (C++ port of python/flow_lk.py).
//
// Sparse Lucas-Kanade on corner features. Features found in one frame are
// tracked into the next and kept, so corner detection only runs when too few
// are left. One forward LK pass (small window, 2 pyramid levels), outliers
// removed against the median shift. The result is the median displacement in
// normalised image coordinates (x/z, y/z; radians near the centre) after
// undistortion. Rotation is NOT removed and nothing is divided by time or
// scaled by height: the Pico does that (plans/optical-flow-aiding.md).
//
// Fallback: when too few features survive (smooth concrete, fine soil, dusk),
// phase correlation on a square in the image centre gives the shift instead.
// It needs texture, not corners. Idea from Clover's optical_flow.cpp
// (CopterExpress/clover, MIT), with the shift undistorted around the
// calibrated principal point. It sees one translation for the whole square, so
// yaw within a frame and repeating patterns (crop rows) can fool it; the
// response gates it.
#pragma once

#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace flow {

// Pinhole camera at the processing resolution, plus distortion. fisheye uses
// OpenCV's fisheye model (k1..k4), which suits the 148 deg 2.8 mm lens;
// otherwise the standard model (k1, k2, p1, p2[, k3]). Empty dist: none.
struct Intrinsics {
    double              fx = 0, fy = 0, cx = 0, cy = 0;
    std::vector<double> dist;
    bool                fisheye = false;

    // Rough default for the CAM-MIPI9281RAW-V2 lens: 2.8 mm on 3 um pixels
    // at 1280x800 (933 px), scaled to the processing size. No distortion.
    static Intrinsics nominal(int width, int height);
    // JSON {"fx":..,"fy":..,"cx":..,"cy":..,"dist":[..],"fisheye":true},
    // the same file python/flow_lk.py reads. Throws on error.
    static Intrinsics load(const std::string& path);

    // Pixel points to undistorted normalised image coordinates.
    void to_normalised(const std::vector<cv::Point2f>& px, std::vector<cv::Point2d>& out) const;
};

struct FlowResult {
    double disp_x = 0;  // normalised image displacement, camera x (right)
    double disp_y = 0;  // camera y (down); sign: where the pattern moved
    double stddev = 0;  // 1-sigma of each axis
    int    n_tracked = 0;
    bool   valid = false;
    bool   phase = false;   // from the phase-correlation fallback
    double response = 0;    // its peak response, 0..1 (phase only)
};

struct FlowParams {
    int    max_corners = 60;
    int    min_keep = 30;     // detect again below this many kept features
    int    min_tracked = 15;  // fewer inliers: result invalid
    int    win = 15;          // LK window [px]
    int    levels = 2;        // LK pyramid levels above the base
    double outlier_px = 1.0;  // max distance from the median shift [px]
    int    border_px = 8;     // features closer to the edge are dropped

    // Phase-correlation fallback (estimates, not yet tuned on real ground).
    bool   phase_fallback = true;
    double phase_roi_rad = 0.5;       // angular width of the centre square
    double phase_min_response = 0.2;  // lower: result invalid
    double phase_min_contrast = 2.0;  // grey-level 1-sigma in the square; lower: invalid
    double phase_sd_px = 0.1;         // 1-sigma [px] at response 1, grows as 1/response
};

class FlowEstimator {
public:
    explicit FlowEstimator(const Intrinsics& intr, const FlowParams& p = FlowParams());

    // Give the next frame (8-bit, one channel; the estimator keeps a
    // reference, so don't write into it afterwards). Returns false for the
    // first frame, else fills r with the flow from the previous frame.
    bool update(const cv::Mat& cur, FlowResult& r);
    // Stateless flow from prev to cur, used by the tests.
    FlowResult estimate(const cv::Mat& prev, const cv::Mat& cur);
    void       reset();

    int n_detect() const { return n_detect_; }  // corner detections run
    int n_phase() const { return n_phase_; }    // phase correlations run

    // Phase correlation alone, prev to cur (used by the fallback and tests).
    FlowResult phase(const cv::Mat& prev, const cv::Mat& cur);

    // Features located in the last frame given to update() (processing-size
    // pixels), for the preview page.
    const std::vector<cv::Point2f>& points() const { return pts_; }

private:
    void       detect(const cv::Mat& img, std::vector<cv::Point2f>& pts);
    FlowResult track(const cv::Mat& prev, const cv::Mat& cur, const std::vector<cv::Point2f>& p0,
                     std::vector<cv::Point2f>* keep);

    Intrinsics               intr_;
    FlowParams               p_;
    cv::Mat                  prev_;
    std::vector<cv::Point2f> pts_;  // features located in prev_
    int                      n_detect_ = 0;
    int                      n_phase_ = 0;

    // Scratch, kept to avoid allocations per frame.
    std::vector<cv::Point2f> p1_;
    std::vector<uchar>       st_;
    std::vector<float>       err_;
    cv::Rect                 roi_;   // phase-correlation square, set on first use
    cv::Mat                  hann_, fa_, fb_;
};

}  // namespace flow
