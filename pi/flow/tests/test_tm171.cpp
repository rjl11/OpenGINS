// Host test for src/tm171.cpp: the EasyProfile codec against the Python
// bench code (firmware/tools/tm210_ep.py) and the bytes our TM171 sent on
// 2026-11-21, the Setting target, the clock map, the gyro integral and the
// de-rotation (the flow_nav.c model).
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "tm171.hpp"

using namespace flow::tm171;

static int fails;
#define CHECK(c)                                                    \
    do {                                                            \
        if (!(c)) {                                                 \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); \
            fails++;                                                \
        }                                                           \
    } while (0)
#define CHECK_NEAR(a, b, tol)                                                                              \
    do {                                                                                                   \
        const double a_ = (a), b_ = (b);                                                                   \
        if (!(std::fabs(a_ - b_) <= (tol))) {                                                              \
            std::printf("FAIL %s:%d: %s = %.7f, want %.7f +- %.2g\n", __FILE__, __LINE__, #a, a_, b_, (double)(tol)); \
            fails++;                                                                                       \
        }                                                                                                  \
    } while (0)

static std::vector<uint8_t> hex(const char* s)
{
    std::vector<uint8_t> v;
    for (; s[0] && s[1]; s += 2) {
        if (s[0] == ' ') {
            s--;
            continue;
        }
        v.push_back(static_cast<uint8_t>(std::stoul(std::string(s, 2), nullptr, 16)));
    }
    return v;
}
static std::string to_hex(const std::vector<uint8_t>& v)
{
    std::string s;
    char        b[3];
    for (uint8_t x : v) {
        std::snprintf(b, sizeof b, "%02x", x);
        s += b;
    }
    return s;
}

// The Setting our TM171 sent as delivered (2026-11-16 and 2026-11-21).
static const char* k_factory = "3fa8180000008004c4095e010a000a0001001900";

static void test_codec()
{
    const char* t = "123456789";
    CHECK(crc16(reinterpret_cast<const uint8_t*>(t), 9) == 0x4B37);
    // Same bytes as tm210_ep.request(21) and tm210_ep.encode(21, 125, ...).
    CHECK(to_hex(request(CMD_SETTING)) == "aa55080c08000015000000e0a9");
    const std::vector<uint8_t> f = hex(k_factory);
    CHECK(to_hex(encode(CMD_SETTING, 125, f.data(), f.size())) ==
          "aa55181508a00f3fa8180000008004c4095e010a000a0001001900395c");

    // Parse with noise, a corrupted copy, and a split feed.
    std::vector<uint8_t> pkt = encode(CMD_SETTING, 125, f.data(), f.size());
    std::vector<uint8_t> bad = pkt;
    bad[10] ^= 0x01;
    std::vector<uint8_t> stream{0x00, 0xAA, 0x13};
    stream.insert(stream.end(), bad.begin(), bad.end());
    stream.insert(stream.end(), pkt.begin(), pkt.end());
    Parser              p;
    std::vector<Packet> out;
    p.feed(stream.data(), 17, 1, out);
    p.feed(stream.data() + 17, stream.size() - 17, 2, out);
    CHECK(out.size() == 1);
    CHECK(p.crc_errors() == 1);
    if (!out.empty()) {
        CHECK(out[0].cmd == CMD_SETTING && out[0].from_id == ID_HOST && out[0].payload == f && out[0].t_ns == 2);
    }
}

static void test_setting()
{
    Setting s;
    const std::vector<uint8_t> f = hex(k_factory);
    std::memcpy(s.data(), f.data(), SETTING_LEN);
    CHECK(layout_ok(s));
    CHECK(switches(s) == 0x0018a83fu && odr_hz(s) == 25 && silent_s(s) == 1 && baud_100(s) == 1152);
    CHECK(!matches_target(s));
    const Setting t = target(s, false);
    // Raw (6), RPY (11, already on), Status (5, on), USB (20, on), Ack (17).
    CHECK(switches(t) == 0x001aa87fu);
    CHECK(odr_hz(t) == 800 && silent_s(t) == 0 && baud_100(t) == 9216);
    CHECK(std::memcmp(&t[8], &s[8], 8) == 0);  // CAN baud, gains, inhibit copied
    CHECK(matches_target(t));
    const Setting ts = target(s, true);
    CHECK(switches(ts) == (0x001aa87fu | SW_SAVE_FLASH) && matches_target(ts));
    // A fused output switched on elsewhere is switched off again.
    Setting q = t;
    q[1] |= 0x01;  // bit 8, quaternion
    CHECK(!matches_target(q) && (switches(target(q, false)) & SW_QUAT_OUT) == 0);
    // Another layout is refused.
    Setting o = s;
    o[1] &= static_cast<uint8_t>(~0x20);  // bit 13
    CHECK(!layout_ok(o));
}

static void test_objects()
{
    Rpy r;
    CHECK(decode_rpy(hex("c01f5539 30a00f3c c6187b3f ac676d43"), r));
    CHECK(r.ts_us == 961880000u);
    CHECK_NEAR(r.roll_deg, 0.008766, 1e-6);
    CHECK_NEAR(r.pitch_deg, 0.980847, 1e-6);
    CHECK_NEAR(r.yaw_deg, 237.40497, 1e-4);
    Status st;
    CHECK(decode_status(hex("c01f5539 cdccb441 2203 0500"), st));
    CHECK_NEAR(st.temp_c, 22.6, 1e-5);
    CHECK(st.rate_hz == 802 && st.qos == 5);
    // Raw: gyro FLU (0.1, 0.2, 0.3) -> FRD (0.1, -0.2, -0.3).
    std::vector<uint8_t> raw(RAW_LEN, 0);
    const float          g[3] = {0.1f, 0.2f, 0.3f};
    std::memcpy(&raw[4], g, sizeof g);
    raw[0] = 0x10;
    Raw w;
    CHECK(decode_raw(raw, w) && w.ts_us == 0x10);
    CHECK(w.gyr_frd[0] == 0.1f && w.gyr_frd[1] == -0.2f && w.gyr_frd[2] == -0.3f);
    CHECK(!decode_raw(std::vector<uint8_t>(RAW_LEN - 1), w));
}

static void test_clock()
{
    // 1250 us steps, arrivals 2..5 ms late with the least late every 7th:
    // the mapped time is the TM171 time plus the smallest offset.
    ClockMap  c;
    const int64_t off_ns = 5000000000LL;
    uint32_t  ts = 0xFFFF0000u;  // wraps mid-run
    int64_t   last = 0;
    int       n = 0;
    for (int i = 0; i < 4000; i++, ts += 1250u) {
        const int64_t true_ns = static_cast<int64_t>(i) * 1250000;
        const int64_t late = (i % 7 == 0) ? 2000000 : 2000000 + (i * 37 % 3000) * 1000;
        const int64_t t = c.map(ts, off_ns + true_ns + late);
        if (i == 0) {
            CHECK(t == 0);
            continue;
        }
        CHECK(t > last);
        last = t;
        if (i > 1000) {  // the first window has settled
            CHECK_NEAR(static_cast<double>(t - (off_ns + true_ns)), 2000000.0, 1.0);
            n++;
        }
    }
    CHECK(n > 2000 && c.jumps() == 0);
    c.map(ts + 5000000u, off_ns);  // a restart
    CHECK(c.jumps() == 1);
}

static void test_gyro()
{
    GyroBuffer b(64);
    const float g[3] = {0.5f, -0.2f, 0.0f};
    for (int i = 0; i < 40; i++) b.push(static_cast<int64_t>(i) * 1250000, g);
    float d[3];
    CHECK(b.integral(5000000, 15000000, d));
    CHECK_NEAR(d[0], 0.005, 1e-7);
    CHECK_NEAR(d[1], -0.002, 1e-7);
    CHECK(!b.integral(5000000, 60000000, d));  // not covered yet
    for (int i = 40; i < 100; i++) b.push(static_cast<int64_t>(i) * 1250000, g);
    CHECK(!b.integral(1000000, 10000000, d));  // gone from the ring
    // A gap longer than 20 ms inside the interval.
    GyroBuffer h(64);
    h.push(0, g);
    h.push(1250000, g);
    h.push(40000000, g);
    h.push(41250000, g);
    CHECK(!h.integral(1000000, 41000000, d));
}

static void test_derotate()
{
    // Camera -> body at yaw 0: camera x = body y, camera y = body -x.
    float r[9];
    cam_to_body(0.0f, r);
    CHECK_NEAR(r[0 * 3 + 0], 0, 1e-7);
    CHECK_NEAR(r[1 * 3 + 0], 1, 1e-7);
    CHECK_NEAR(r[0 * 3 + 1], -1, 1e-7);
    CHECK_NEAR(r[2 * 3 + 2], 1, 1e-7);

    // A pure rotation in place: the image moves by the rotation alone
    // (dx = -dth_c,y, dy = dth_c,x), and the velocity must read zero.
    const float dt = 0.01f, z = 1.2f;
    const float rots[3][3] = {{0.004f, 0, 0}, {0, 0.004f, 0}, {0.002f, -0.003f, 0.001f}};
    for (const auto& dth : rots) {
        float dc[3];
        for (int i = 0; i < 3; i++) dc[i] = r[0 * 3 + i] * dth[0] + r[1 * 3 + i] * dth[1] + r[2 * 3 + i] * dth[2];
        float v[2];
        derotate(r, -dc[1], dc[0], dth, z, dt, v);
        CHECK_NEAR(v[0], 0, 1e-6);
        CHECK_NEAR(v[1], 0, 1e-6);
    }
    // Forward at 2 m/s, no rotation: the ground moves down the image
    // (camera +y), so disp_y = +v dt / z, and camera y velocity is -2.
    float v[2];
    const float none[3] = {0, 0, 0};
    derotate(r, 0, 2.0f * dt / z, none, z, dt, v);
    CHECK_NEAR(v[0], 0, 1e-6);
    CHECK_NEAR(v[1], -2.0, 1e-5);
    // Back in body axes that is +2 forward: v_b = r_cb v_cam.
    CHECK_NEAR(r[0 * 3 + 0] * v[0] + r[0 * 3 + 1] * v[1], 2.0, 1e-5);
}

static void test_mount()
{
    float r[9];
    const float zero[3] = {0, 0, 0};
    mount_to_body(zero, r);
    for (int i = 0; i < 9; i++) CHECK_NEAR(r[i], (i % 4 == 0) ? 1.0 : 0.0, 1e-7);
    // Yawed 90 deg (TM171 x to the right): its x is body y.
    const float yaw[3] = {0, 0, 1.5707963f};
    mount_to_body(yaw, r);
    const float x[3] = {1, 0, 0};
    float       o[3];
    mul3(r, x, o);
    CHECK_NEAR(o[0], 0, 1e-6);
    CHECK_NEAR(o[1], 1, 1e-6);
    // Same convention as the Pico's ground-point and the flow_nav test: a
    // pure roll of 0.3 rad maps body y onto (0, cos, sin).
    const float roll[3] = {0.3f, 0, 0};
    mount_to_body(roll, r);
    const float y[3] = {0, 1, 0};
    mul3(r, y, o);
    CHECK_NEAR(o[1], std::cos(0.3), 1e-6);
    CHECK_NEAR(o[2], std::sin(0.3), 1e-6);
}

int main()
{
    test_mount();
    test_codec();
    test_setting();
    test_objects();
    test_clock();
    test_gyro();
    test_derotate();
    std::printf(fails ? "test_tm171: %d FAILED\n" : "test_tm171: OK\n", fails);
    return fails ? 1 : 0;
}
