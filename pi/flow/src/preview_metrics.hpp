// Figures for the camera preview page (preview_http.hpp): focus and exposure,
// for aiming and focusing the front camera unit (plans/front-camera-unit.md).
#pragma once

#include <opencv2/core.hpp>

namespace flow {

// Variance of the Laplacian over roi (clipped to the image): high when the
// image is sharp. Relative only, since it also grows with the texture of the
// scene; turn the lens until it peaks. 0 for an empty region.
double focus_laplacian_var(const cv::Mat& grey, const cv::Rect& roi);

// The central region the focus figure is taken over: the middle half of the
// image in each direction.
cv::Rect focus_roi(const cv::Size& s);

struct ExposureStats {
    double mean = 0;      // mean grey level, 0..255
    double pct_black = 0; // % of pixels at 0
    double pct_white = 0; // % of pixels at 255
};

ExposureStats exposure_stats(const cv::Mat& grey);

}  // namespace flow
