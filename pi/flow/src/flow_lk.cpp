#include "flow_lk.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

namespace flow {

Intrinsics Intrinsics::nominal(int width, int height)
{
    Intrinsics k;
    k.fx = k.fy = 2.8e-3 / 3e-6 * width / 1280.0;
    k.cx = (width - 1) / 2.0;
    k.cy = (height - 1) / 2.0;
    return k;
}

Intrinsics Intrinsics::load(const std::string& path)
{
    cv::FileStorage fs(path, cv::FileStorage::READ | cv::FileStorage::FORMAT_JSON);
    if (!fs.isOpened()) throw std::runtime_error("can't read calibration " + path);
    Intrinsics k;
    fs["fx"] >> k.fx;
    fs["fy"] >> k.fy;
    fs["cx"] >> k.cx;
    fs["cy"] >> k.cy;
    cv::FileNode d = fs["dist"];
    for (auto it = d.begin(); it != d.end(); ++it) k.dist.push_back(static_cast<double>(*it));
    int fe = 0;
    if (!fs["fisheye"].empty()) {
        // JSON true/false is read as an int by FileStorage.
        fs["fisheye"] >> fe;
    }
    k.fisheye = fe != 0;
    if (k.fx <= 0 || k.fy <= 0) throw std::runtime_error("calibration " + path + ": fx/fy missing");
    return k;
}

void Intrinsics::to_normalised(const std::vector<cv::Point2f>& px, std::vector<cv::Point2d>& out) const
{
    out.resize(px.size());
    if (dist.empty()) {
        for (size_t i = 0; i < px.size(); i++) out[i] = {(px[i].x - cx) / fx, (px[i].y - cy) / fy};
        return;
    }
    cv::Matx33d               K(fx, 0, cx, 0, fy, cy, 0, 0, 1);
    std::vector<cv::Point2f>  u;
    if (fisheye) {
        cv::fisheye::undistortPoints(px, u, K, dist);
    } else {
        cv::undistortPoints(px, u, K, dist);
    }
    for (size_t i = 0; i < u.size(); i++) out[i] = {u[i].x, u[i].y};
}

FlowEstimator::FlowEstimator(const Intrinsics& intr, const FlowParams& p) : intr_(intr), p_(p) {}

void FlowEstimator::reset()
{
    prev_.release();
    pts_.clear();
}

void FlowEstimator::detect(const cv::Mat& img, std::vector<cv::Point2f>& pts)
{
    n_detect_++;
    cv::goodFeaturesToTrack(img, pts, p_.max_corners, 0.01, 8, cv::noArray(), 7);
}

bool FlowEstimator::update(const cv::Mat& cur, FlowResult& r)
{
    cv::Mat prev = prev_;
    prev_ = cur;
    if (prev.empty()) {
        pts_.clear();
        return false;
    }
    if (static_cast<int>(pts_.size()) < p_.min_keep) detect(prev, pts_);
    std::vector<cv::Point2f> keep;
    r = track(prev, cur, pts_, &keep);
    pts_.swap(keep);
    if (!r.valid && p_.phase_fallback) r = phase(prev, cur);
    return true;
}

FlowResult FlowEstimator::estimate(const cv::Mat& prev, const cv::Mat& cur)
{
    std::vector<cv::Point2f> p0;
    detect(prev, p0);
    FlowResult r = track(prev, cur, p0, nullptr);
    if (!r.valid && p_.phase_fallback) r = phase(prev, cur);
    return r;
}

FlowResult FlowEstimator::phase(const cv::Mat& prev, const cv::Mat& cur)
{
    FlowResult r;
    r.phase = true;
    n_phase_++;
    if (roi_.width == 0 || roi_.x + roi_.width > cur.cols || roi_.y + roi_.height > cur.rows) {
        // Square of phase_roi_rad around the principal point (pinhole is
        // close enough in the centre), inside the border, even size.
        const int lim = std::min(cur.cols, cur.rows) - 2 * p_.border_px;
        int       n = static_cast<int>(2.0 * intr_.fx * std::tan(0.5 * p_.phase_roi_rad));
        n = std::max(16, std::min(n, lim)) & ~1;
        int x = static_cast<int>(std::lround(intr_.cx)) - n / 2;
        int y = static_cast<int>(std::lround(intr_.cy)) - n / 2;
        x = std::max(0, std::min(x, cur.cols - n));
        y = std::max(0, std::min(y, cur.rows - n));
        roi_ = cv::Rect(x, y, n, n);
        cv::createHanningWindow(hann_, roi_.size(), CV_32F);
    }
    // Mean removed before the window: otherwise the window's own shape
    // correlates with itself and pulls weak texture towards zero shift.
    cv::Scalar ma, sa, mb, sb;
    cv::meanStdDev(prev(roi_), ma, sa);
    cv::meanStdDev(cur(roi_), mb, sb);
    if (std::min(sa[0], sb[0]) < p_.phase_min_contrast) return r;
    prev(roi_).convertTo(fa_, CV_32F, 1.0, -ma[0]);
    cur(roi_).convertTo(fb_, CV_32F, 1.0, -mb[0]);
    double            resp = 0;
    const cv::Point2d s = cv::phaseCorrelate(fa_, fb_, hann_, &resp);
    r.response = resp;
    // A shift past a quarter of the square is ambiguous (the correlation wraps).
    if (!(resp >= p_.phase_min_response) || std::fabs(s.x) > roi_.width / 4.0 ||
        std::fabs(s.y) > roi_.height / 4.0)
        return r;

    // Undistort the centre and its shifted copy, around the calibrated
    // principal point.
    const cv::Point2f              c(static_cast<float>(roi_.x + 0.5 * (roi_.width - 1)),
                                     static_cast<float>(roi_.y + 0.5 * (roi_.height - 1)));
    const std::vector<cv::Point2f> px = {c, c + cv::Point2f(static_cast<float>(s.x), static_cast<float>(s.y))};
    std::vector<cv::Point2d>       nm;
    intr_.to_normalised(px, nm);
    r.disp_x = nm[1].x - nm[0].x;
    r.disp_y = nm[1].y - nm[0].y;
    r.stddev = p_.phase_sd_px / resp / std::min(intr_.fx, intr_.fy);
    r.valid = true;
    return r;
}

static double median_inplace(std::vector<double>& v)
{
    auto mid = v.begin() + static_cast<long>(v.size() / 2);
    std::nth_element(v.begin(), mid, v.end());
    double m = *mid;
    if (v.size() % 2 == 0) m = 0.5 * (m + *std::max_element(v.begin(), mid));
    return m;
}

FlowResult FlowEstimator::track(const cv::Mat& prev, const cv::Mat& cur, const std::vector<cv::Point2f>& p0,
                                std::vector<cv::Point2f>* keep)
{
    FlowResult r;
    if (keep) keep->clear();
    r.n_tracked = static_cast<int>(p0.size());
    if (r.n_tracked < p_.min_tracked) return r;

    cv::calcOpticalFlowPyrLK(prev, cur, p0, p1_, st_, err_, cv::Size(p_.win, p_.win), p_.levels,
                             cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 10, 0.03));

    const float b = static_cast<float>(p_.border_px);
    const float w = static_cast<float>(cur.cols) - b, h = static_cast<float>(cur.rows) - b;
    std::vector<int>    idx;
    std::vector<double> dx, dy;
    idx.reserve(p0.size());
    for (size_t i = 0; i < p0.size(); i++) {
        const cv::Point2f& q = p1_[i];
        if (st_[i] && q.x >= b && q.x < w && q.y >= b && q.y < h) {
            idx.push_back(static_cast<int>(i));
            dx.push_back(q.x - p0[i].x);
            dy.push_back(q.y - p0[i].y);
        }
    }
    r.n_tracked = static_cast<int>(idx.size());
    if (r.n_tracked < p_.min_tracked) return r;

    // Outliers against the median shift, in pixels (cheap, before undistortion).
    std::vector<double> sx(dx), sy(dy);
    const double        mx = median_inplace(sx), my = median_inplace(sy);
    std::vector<cv::Point2f> a, c;
    for (size_t k = 0; k < idx.size(); k++) {
        if (std::max(std::fabs(dx[k] - mx), std::fabs(dy[k] - my)) < p_.outlier_px) {
            a.push_back(p0[static_cast<size_t>(idx[k])]);
            c.push_back(p1_[static_cast<size_t>(idx[k])]);
        }
    }
    r.n_tracked = static_cast<int>(a.size());
    if (r.n_tracked < p_.min_tracked) return r;

    // Displacement in normalised image coordinates. Over flat ground facing
    // the camera, a translation moves every point by the same v*dt/h here,
    // wherever it is in the image (an angle would shrink towards the edges).
    std::vector<cv::Point2d> na, nc;
    intr_.to_normalised(a, na);
    intr_.to_normalised(c, nc);
    std::vector<double> ex(na.size()), ey(na.size());
    for (size_t k = 0; k < na.size(); k++) {
        ex[k] = nc[k].x - na[k].x;
        ey[k] = nc[k].y - na[k].y;
    }
    std::vector<double> tx(ex), ty(ey);
    r.disp_x = median_inplace(tx);
    r.disp_y = median_inplace(ty);
    // Robust spread (MAD), then the error of the median of n samples.
    for (size_t k = 0; k < ex.size(); k++) {
        tx[k] = std::fabs(ex[k] - r.disp_x);
        ty[k] = std::fabs(ey[k] - r.disp_y);
    }
    const double mad = std::max(median_inplace(tx), median_inplace(ty)) * 1.4826;
    r.stddev = mad * 1.2533 / std::sqrt(static_cast<double>(ex.size()));
    r.valid = true;
    if (keep) *keep = c;
    return r;
}

}  // namespace flow
