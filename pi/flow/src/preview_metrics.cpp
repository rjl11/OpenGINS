#include "preview_metrics.hpp"

#include <opencv2/imgproc.hpp>

namespace flow {

double focus_laplacian_var(const cv::Mat& grey, const cv::Rect& roi)
{
    const cv::Rect r = roi & cv::Rect(0, 0, grey.cols, grey.rows);
    if (r.width < 3 || r.height < 3) return 0;
    cv::Mat lap;
    cv::Laplacian(grey(r), lap, CV_32F, 3);
    cv::Scalar mean, sd;
    cv::meanStdDev(lap, mean, sd);
    return sd[0] * sd[0];
}

cv::Rect focus_roi(const cv::Size& s)
{
    return cv::Rect(s.width / 4, s.height / 4, s.width / 2, s.height / 2);
}

ExposureStats exposure_stats(const cv::Mat& grey)
{
    ExposureStats e;
    const double  n = static_cast<double>(grey.total());
    if (n == 0) return e;
    e.mean = cv::mean(grey)[0];
    cv::Mat m;
    cv::compare(grey, 0, m, cv::CMP_EQ);
    e.pct_black = 100.0 * cv::countNonZero(m) / n;
    cv::compare(grey, 255, m, cv::CMP_EQ);
    e.pct_white = 100.0 * cv::countNonZero(m) / n;
    return e;
}

}  // namespace flow
