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
}

static void test_interval_watch()
{
    flow::IntervalWatch w;
    for (int i = 0; i < 10; i++) CHECK(w.classify(10'000'000) == 0);
    CHECK(w.classify(20'000'000) == FLOW_F_AFTER_GAP);  // the left-out trigger
    CHECK(w.classify(10'000'000) == 0);
    CHECK(w.classify(35'000'000) == FLOW_F_DROPPED);
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
}

int main()
{
    cv::setNumThreads(1);
    test_shift(0.0, 0.0);
    test_shift(0.4, -0.3);
    test_shift(12.0, 7.0);
    test_sequence();
    test_flat();
    test_link_example();
    test_interval_watch();
    speed();
    std::printf(g_fails ? "test_flow: %d FAILED\n" : "test_flow: OK\n", g_fails);
    return g_fails ? 1 : 0;
}
