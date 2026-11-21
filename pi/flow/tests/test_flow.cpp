// Tests for the C++ flow, mirroring python/test_flow_lk.py, plus the link
// example frame and a speed run. Exit code 0 when everything passes.
//
//   pi/flow/build/test_flow            tests and speed
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include <opencv2/imgproc.hpp>

#include "flow_link.h"
#include "flow_lk.hpp"
#include "interval_watch.hpp"
#include "preview_metrics.hpp"

static int g_fails = 0;

#define CHECK(c)                                                       \
    do {                                                               \
        if (!(c)) {                                                    \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c);   \
            g_fails++;                                                 \
        }                                                              \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                                                   \
    do {                                                                                        \
        const double a_ = (a), b_ = (b);                                                        \
        if (!(std::fabs(a_ - b_) <= (tol))) {                                                   \
            std::printf("FAIL %s:%d: %s = %.7f, want %.7f +- %.1e\n", __FILE__, __LINE__, #a, a_, \
                        b_, static_cast<double>(tol));                                          \
            g_fails++;                                                                          \
        }                                                                                       \
    } while (0)

// Smooth random texture, like texture() in python/test_flow_lk.py.
static cv::Mat texture(int w, int h, uint64_t seed = 1)
{
    cv::RNG rng(seed);
    cv::Mat small(h / 8, w / 8, CV_8UC1);
    rng.fill(small, cv::RNG::UNIFORM, 0, 256);
    cv::Mat img;
    cv::resize(small, img, cv::Size(w, h), 0, 0, cv::INTER_CUBIC);
    cv::GaussianBlur(img, img, cv::Size(5, 5), 1.0);
    return img;
}

// Centre crop of img with its content moved by (dx, dy) px.
static cv::Mat moved(const cv::Mat& img, double dx, double dy, int w = 640, int h = 400)
{
    cv::Mat M = (cv::Mat_<double>(2, 3) << 1, 0, dx, 0, 1, dy);
    cv::Mat m;
    cv::warpAffine(img, m, M, img.size(), cv::INTER_LINEAR);
    return m(cv::Rect((img.cols - w) / 2, (img.rows - h) / 2, w, h)).clone();
}

static void test_shift(double dx, double dy)
{
    const cv::Mat          tex = texture(800, 560);
    const flow::Intrinsics k = flow::Intrinsics::nominal(640, 400);
    flow::FlowEstimator    est(k);
    flow::FlowResult       r = est.estimate(moved(tex, 0, 0), moved(tex, dx, dy));
    CHECK(r.valid);
    CHECK(r.n_tracked > 30);
    CHECK_NEAR(r.disp_x, dx / k.fx, 2e-4);
    CHECK_NEAR(r.disp_y, dy / k.fy, 2e-4);
    CHECK(r.stddev < 1e-4);
}

static void test_sequence()
{
    const cv::Mat          big = texture(1400, 900);
    const flow::Intrinsics k = flow::Intrinsics::nominal(640, 400);
    flow::FlowEstimator    est(k);
    flow::FlowResult       r;
    const double           dx = 2.5, dy = -1.5;
    CHECK(!est.update(moved(big, 0, 0), r));
    for (int i = 1; i < 60; i++) {
        CHECK(est.update(moved(big, dx * i, dy * i), r));
        CHECK(r.valid);
        CHECK_NEAR(r.disp_x, dx / k.fx, 2e-4);
        CHECK_NEAR(r.disp_y, dy / k.fy, 2e-4);
    }
    CHECK(est.n_detect() < 15);
}

static void test_flat()
{
    flow::FlowEstimator est(flow::Intrinsics::nominal(640, 400));
    cv::Mat             flat(400, 640, CV_8UC1, cv::Scalar(128));
    CHECK(!est.estimate(flat, flat).valid);
}

// Phase correlation alone, and as the fallback when Lucas-Kanade is made to
// fail (min_tracked above the corner count).
static void test_phase(double dx, double dy)
{
    const cv::Mat          tex = texture(800, 560, 3);
    const flow::Intrinsics k = flow::Intrinsics::nominal(640, 400);
    flow::FlowEstimator    est(k);
    flow::FlowResult       r = est.phase(moved(tex, 0, 0), moved(tex, dx, dy));
    CHECK(r.valid && r.phase);
    CHECK(r.response > 0.3);
    CHECK_NEAR(r.disp_x, dx / k.fx, 0.1 / k.fx);
    CHECK_NEAR(r.disp_y, dy / k.fy, 0.1 / k.fy);
    CHECK(r.stddev > 0 && r.stddev < 1.0 / k.fx);

    flow::FlowParams p;
    p.min_tracked = 1000;
    flow::FlowEstimator fb(k, p);
    r = fb.estimate(moved(tex, 0, 0), moved(tex, dx, dy));
    CHECK(r.valid && r.phase);
    CHECK_NEAR(r.disp_x, dx / k.fx, 0.1 / k.fx);
    CHECK_NEAR(r.disp_y, dy / k.fy, 0.1 / k.fy);

    p.phase_fallback = false;
    flow::FlowEstimator off(k, p);
    CHECK(!off.estimate(moved(tex, 0, 0), moved(tex, dx, dy)).valid);
}

// The fallback in a moving sequence through update(), at the default size.
static void test_phase_sequence()
{
    const cv::Mat          big = texture(1400, 900, 5);
    const flow::Intrinsics k = flow::Intrinsics::nominal(320, 200);
    flow::FlowParams       p;
    p.min_tracked = 1000;
    flow::FlowEstimator est(k, p);
    flow::FlowResult    r;
    const double        dx = 3.0, dy = -2.0;  // at 640x400, 1.5 / -1.0 px at 320x200
    auto frame = [&](int i) {
        cv::Mat m = moved(big, dx * i, dy * i), s;
        cv::resize(m, s, cv::Size(320, 200), 0, 0, cv::INTER_AREA);
        return s;
    };
    CHECK(!est.update(frame(0), r));
    for (int i = 1; i < 30; i++) {
        CHECK(est.update(frame(i), r));
        CHECK(r.valid && r.phase);
        CHECK_NEAR(r.disp_x, 0.5 * dx / k.fx, 0.1 / k.fx);
        CHECK_NEAR(r.disp_y, 0.5 * dy / k.fy, 0.1 / k.fy);
    }
    CHECK(est.n_phase() == 29);
}

// Blank (it would correlate perfectly at zero shift), and a shift past a
// quarter of the square (238 px here): not valid.
static void test_phase_reject()
{
    flow::FlowEstimator est(flow::Intrinsics::nominal(640, 400));
    cv::Mat             flat(400, 640, CV_8UC1, cv::Scalar(128));
    flow::FlowResult    r = est.phase(flat, flat);
    CHECK(!r.valid && r.phase);
    const cv::Mat tex = texture(800, 560, 7);
    CHECK(!est.phase(moved(tex, 0, 0), moved(tex, 80, 0)).valid);
}

static void test_link_example()
{
    flow_msg_flow_t m{};
    m.frame_seq = 1000;
    m.pi_time_us = 123456789;
    m.pi_dt_us = 3333;
    m.disp_x_rad = 0.01f;
    m.disp_y_rad = -0.002f;
    m.stddev_rad = 0.0005f;
    m.n_tracked = 87;
    m.exposure_100us = 12;
    m.flags = FLOW_F_VALID | FLOW_F_TRIGGERED;
    uint8_t buf[FLOW_LINK_FLOW_FRAME_LEN];
    CHECK(flow_link_encode_flow(&m, buf, sizeof buf) == FLOW_LINK_FLOW_FRAME_LEN);
    std::string hex;
    char        h[3];
    for (uint8_t b : buf) {
        std::snprintf(h, sizeof h, "%02x", b);
        hex += h;
    }
    CHECK(hex == FLOW_LINK_EXAMPLE_HEX);

    // Clock sync: the unit's side decodes the request and builds the reply.
    auto to_hex = [](const uint8_t* p, size_t n) {
        std::string s;
        char        t[3];
        for (size_t i = 0; i < n; i++) {
            std::snprintf(t, sizeof t, "%02x", p[i]);
            s += t;
        }
        return s;
    };
    uint8_t q[FLOW_LINK_TIME_REQ_FRAME_LEN];
    CHECK(flow_link_encode_time_req(7, q, sizeof q) == sizeof q);
    CHECK(to_hex(q, sizeof q) == FLOW_LINK_EXAMPLE_TIME_REQ_HEX);
    uint8_t  type = 0;
    uint32_t seq = 0;
    CHECK(flow_link_check(q, sizeof q, &type) == static_cast<int>(FLOW_MSG_TIME_REQ_LEN));
    CHECK(type == FLOW_MSG_TIME_REQ);
    CHECK(flow_link_decode_time_req(&q[FLOW_LINK_HEADER_LEN], FLOW_MSG_TIME_REQ_LEN, &seq) == 0 && seq == 7);
    flow_msg_time_resp_t tr{};
    tr.seq = seq;
    tr.turnaround_us = 42;
    tr.unit_rx_us = 0x1234567890ull;
    uint8_t r[FLOW_LINK_TIME_RESP_FRAME_LEN];
    CHECK(flow_link_encode_time_resp(&tr, r, sizeof r) == sizeof r);
    CHECK(to_hex(r, sizeof r) == FLOW_LINK_EXAMPLE_TIME_RESP_HEX);

    flow_msg_cam_vel_t cv{};
    cv.frame_seq = 2000;
    cv.pi_time_us = 987654321u;
    cv.pi_dt_us = 10000;
    cv.vel_x_mps = 1.5f;
    cv.vel_y_mps = -0.25f;
    cv.stddev_mps = 0.02f;
    cv.height_m = 1.2f;
    cv.height_sd_m = 0.03f;
    cv.n_tracked = 64;
    cv.exposure_100us = 8;
    cv.flags = FLOW_F_VALID | FLOW_F_RANGE;
    uint8_t c[FLOW_LINK_CAM_VEL_FRAME_LEN];
    CHECK(flow_link_encode_cam_vel(&cv, c, sizeof c) == sizeof c);
    CHECK(to_hex(c, sizeof c) == FLOW_LINK_EXAMPLE_CAM_VEL_HEX);

    flow_msg_unit_cfg_t uc{};
    uc.seq = 3;
    uc.height_m = 1.25f;
    uc.cam_yaw_rad = 0.5f;
    uc.imu_rpy_rad[1] = -0.25f;
    uc.imu_rpy_rad[2] = 1.5f;
    uc.flags = FLOW_CFG_F_FLOW_ON;
    uint8_t u[FLOW_LINK_UNIT_CFG_FRAME_LEN];
    CHECK(flow_link_encode_unit_cfg(&uc, u, sizeof u) == sizeof u);
    CHECK(to_hex(u, sizeof u) == FLOW_LINK_EXAMPLE_UNIT_CFG_HEX);
    flow_msg_unit_cfg_t ud{};
    CHECK(flow_link_decode_unit_cfg(&u[FLOW_LINK_HEADER_LEN], FLOW_MSG_UNIT_CFG_LEN, &ud) == 0 && ud.seq == 3 &&
          ud.imu_rpy_rad[2] == 1.5f);

    flow_msg_imu_batch_t ib{};
    ib.t0_us = 1000000;
    ib.seq = 9;
    ib.n = 2;
    ib.s[0].gyr[0] = 0.5f;
    ib.s[0].gyr[1] = -0.25f;
    ib.s[0].acc[2] = -9.8125f;
    ib.s[1].dt_us = 1250;
    ib.s[1].gyr[2] = 1.0f;
    ib.s[1].acc[0] = 1.0f;
    ib.s[1].acc[1] = -1.0f;
    ib.s[1].acc[2] = -9.8125f;
    uint8_t      bb[FLOW_LINK_HEADER_LEN + FLOW_LINK_MAX_PAYLOAD + FLOW_LINK_CRC_LEN];
    const size_t bn = flow_link_encode_imu_batch(&ib, bb, sizeof bb);
    CHECK(bn == 5 + 36 + 2);
    CHECK(to_hex(bb, bn) == FLOW_LINK_EXAMPLE_IMU_BATCH_HEX);
}

static void test_interval_watch()
{
    flow::IntervalWatch w;
    for (int i = 0; i < 10; i++) CHECK(w.classify(10'000'000) == 0);
    CHECK(w.classify(20'000'000) == FLOW_F_AFTER_GAP);  // the left-out trigger
    CHECK(w.classify(10'000'000) == 0);
    CHECK(w.classify(35'000'000) == FLOW_F_DROPPED);

    // A lost frame makes the same two-pulse interval as the gap, but it skips
    // an ordinary pulse: DROPPED, and the next real gap is still on time.
    flow::IntervalWatch v;
    for (int i = 0; i < 10; i++) CHECK(v.classify(10'000'000) == 0);
    CHECK(v.classify(20'000'000) == FLOW_F_AFTER_GAP);  // anchor
    for (int i = 0; i < 30; i++) CHECK(v.classify(10'000'000) == 0);
    CHECK(v.classify(20'000'000) == FLOW_F_DROPPED);    // lost frame
    for (int i = 0; i < 222; i++) CHECK(v.classify(10'000'000) == 0);
    CHECK(v.classify(20'000'000) == FLOW_F_AFTER_GAP);  // the real gap, 256 pulses on
}

// Speed on a moving sequence at the default processing size (320x200).
static void speed()
{
    const cv::Mat        big = texture(2400, 1500);
    std::vector<cv::Mat> seq;
    for (int i = 0; i < 300; i++) {
        cv::Mat m = moved(big, -2.0 * i, -0.8 * i, 640, 400), s;
        cv::resize(m, s, cv::Size(320, 200), 0, 0, cv::INTER_AREA);
        seq.push_back(s);
    }
    flow::FlowEstimator est(flow::Intrinsics::nominal(320, 200));
    flow::FlowResult    r;
    est.update(seq[0], r);
    const auto t0 = std::chrono::steady_clock::now();
    for (size_t i = 1; i < seq.size(); i++) est.update(seq[i], r);
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count() /
                      static_cast<double>(seq.size() - 1);
    std::printf("speed: %.2f ms per frame at 320x200, %d detections in %zu frames\n", ms, est.n_detect(),
                seq.size() - 1);

    // Phase correlation alone on the same sequence.
    const auto t1 = std::chrono::steady_clock::now();
    int        n_ok = 0;
    for (size_t i = 1; i < seq.size(); i++) n_ok += est.phase(seq[i - 1], seq[i]).valid;
    const double pms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t1).count() /
                       static_cast<double>(seq.size() - 1);
    std::printf("speed: %.2f ms per phase correlation at 320x200, %d of %zu valid\n", pms, n_ok, seq.size() - 1);
}

// The preview page's focus figure peaks on the sharpest image, and its
// exposure figures count what they should.
static void test_preview_metrics()
{
    cv::RNG rng(7);
    cv::Mat sharp(400, 640, CV_8UC1);
    rng.fill(sharp, cv::RNG::UNIFORM, 0, 256);
    const cv::Rect roi = flow::focus_roi(sharp.size());
    CHECK(roi == cv::Rect(160, 100, 320, 200));
    double last = flow::focus_laplacian_var(sharp, roi);
    CHECK(last > 0);
    for (double sigma : {0.5, 1.0, 2.0, 4.0}) {
        cv::Mat b;
        cv::GaussianBlur(sharp, b, cv::Size(0, 0), sigma);
        const double f = flow::focus_laplacian_var(b, roi);
        CHECK(f < last);
        last = f;
    }
    CHECK(flow::focus_laplacian_var(cv::Mat(400, 640, CV_8UC1, cv::Scalar(90)), roi) == 0);
    CHECK(flow::focus_laplacian_var(sharp, cv::Rect(700, 0, 10, 10)) == 0);

    cv::Mat e(10, 10, CV_8UC1, cv::Scalar(100));
    e(cv::Rect(0, 0, 10, 1)).setTo(0);
    e(cv::Rect(0, 1, 10, 2)).setTo(255);
    const flow::ExposureStats s = flow::exposure_stats(e);
    CHECK_NEAR(s.pct_black, 10.0, 1e-9);
    CHECK_NEAR(s.pct_white, 20.0, 1e-9);
    CHECK_NEAR(s.mean, (7 * 10 * 100 + 2 * 10 * 255) / 100.0, 1e-9);
}

int main()
{
    cv::setNumThreads(1);
    test_shift(0.0, 0.0);
    test_shift(0.4, -0.3);
    test_shift(12.0, 7.0);
    test_sequence();
    test_flat();
    test_phase(0.0, 0.0);
    test_phase(0.4, -0.3);
    test_phase(12.0, 7.0);
    test_phase_sequence();
    test_phase_reject();
    test_link_example();
    test_interval_watch();
    test_preview_metrics();
    speed();
    std::printf(g_fails ? "test_flow: %d FAILED\n" : "test_flow: OK\n", g_fails);
    return g_fails ? 1 : 0;
}
