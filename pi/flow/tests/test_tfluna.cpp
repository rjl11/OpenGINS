// Tests for the TF-Luna protocol and range buffer (src/tfluna.hpp), against
// the frames in Benewake's manual SJ-PM-TF-Luna A05. Exit code 0 when all pass.
//
//   pi/flow/build/test_tfluna
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

#include "tfluna.hpp"

using namespace flow::tfluna;

static int g_fails = 0;

#define CHECK(c)                                                     \
    do {                                                             \
        if (!(c)) {                                                  \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); \
            g_fails++;                                               \
        }                                                            \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                                                          \
    do {                                                                                               \
        const double a_ = (a), b_ = (b);                                                               \
        if (!(std::fabs(a_ - b_) <= (tol))) {                                                          \
            std::printf("FAIL %s:%d: %s = %.6f, want %.6f\n", __FILE__, __LINE__, #a, a_, b_);         \
            g_fails++;                                                                                 \
        }                                                                                              \
    } while (0)

using Bytes = std::vector<uint8_t>;

// A data frame, summed here independently of the code under test.
static Bytes data_frame(unsigned dist, unsigned amp, double temp_c)
{
    const unsigned t = static_cast<unsigned>(std::lround((temp_c + 256.0) * 8.0));
    Bytes          f = {0x59, 0x59, static_cast<uint8_t>(dist), static_cast<uint8_t>(dist >> 8),
                        static_cast<uint8_t>(amp), static_cast<uint8_t>(amp >> 8), static_cast<uint8_t>(t),
                        static_cast<uint8_t>(t >> 8)};
    unsigned       s = 0;
    for (uint8_t b : f) s += b;
    f.push_back(static_cast<uint8_t>(s));
    return f;
}

struct Fed {
    std::vector<Sample> samples;
    std::vector<Reply>  replies;
};

static Fed feed(Parser& p, const Bytes& b, int64_t t_last = 1000000000, int64_t byte_ns = 86806)
{
    Fed f;
    p.feed(b.data(), b.size(), t_last, byte_ns, f.samples, f.replies);
    return f;
}

static void test_commands()
{
    // The manual's examples that carry a real sum.
    CHECK((command(0x08, Bytes{0x00}.data(), 1) == Bytes{0x5A, 0x05, 0x08, 0x00, 0x67}));
    // The rest give 00 as the sum (not checked by default); the bytes before it.
    const Bytes f10 = cmd_sample_freq(10);
    CHECK((Bytes(f10.begin(), f10.end() - 1) == Bytes{0x5A, 0x06, 0x03, 0x0A, 0x00}));
    CHECK(f10.back() == 0x6D);
    const Bytes f250 = cmd_sample_freq(250);
    CHECK((Bytes(f250.begin(), f250.end() - 1) == Bytes{0x5A, 0x06, 0x03, 0xFA, 0x00}));
    const Bytes ver = cmd_get_version();
    CHECK((Bytes(ver.begin(), ver.end() - 1) == Bytes{0x5A, 0x04, 0x01}));
    const Bytes off = cmd_output_enable(false);
    CHECK((Bytes(off.begin(), off.end() - 1) == Bytes{0x5A, 0x05, 0x07, 0x00}));
    const Bytes nofilt = cmd_filter(0);
    CHECK((Bytes(nofilt.begin(), nofilt.end() - 1) == Bytes{0x5A, 0x05, 0x39, 0x00}));
    const Bytes mm = cmd_output_format(FMT_9B_MM);
    CHECK((Bytes(mm.begin(), mm.end() - 1) == Bytes{0x5A, 0x05, 0x05, 0x06}));
}

static void test_classify()
{
    CHECK(classify(1.0, 1500) == Status::Ok);
    CHECK(classify(1.0, 99) == Status::LowAmp);
    CHECK(classify(1.0, 100) == Status::Ok);
    CHECK(classify(0.0, 50) == Status::LowAmp);  // the dummy distance
    CHECK(classify(1.0, 32769) == Status::Overload);
    CHECK(classify(1.0, 65535) == Status::Overload);
    CHECK(classify(0.19, 1500) == Status::TooClose);
    CHECK(classify(8.01, 1500) == Status::TooFar);
}

static void test_data_frame()
{
    Parser p;
    p.set_meas_delay_ns(5000000);
    const Fed f = feed(p, data_frame(123, 1500, 31.5), 2000000000, 100000);
    CHECK(f.samples.size() == 1);
    if (f.samples.size() != 1) return;
    const Sample& s = f.samples[0];
    CHECK_NEAR(s.range_m, 1.23, 1e-6);
    CHECK(s.amp == 1500);
    CHECK_NEAR(s.temp_c, 31.5, 1e-6);
    CHECK(s.ok());
    // First byte's arrival (8 bytes before the last), less the averaging delay.
    CHECK(s.t_ns == 2000000000 - 8 * 100000 - 5000000);

    Parser pm;
    pm.set_unit_m(0.001);
    const Fed g = feed(pm, data_frame(1234, 800, 25));
    CHECK(g.samples.size() == 1 && std::fabs(g.samples[0].range_m - 1.234) < 1e-6);
}

static void test_stream()
{
    // Junk, a frame split over three reads, a frame with a bad sum, a lone
    // 0x59 and 0x5A bytes (replies off), then two good frames back to back.
    Parser p;
    Bytes  a = data_frame(100, 1000, 20), b = data_frame(200, 2000, 21), c = data_frame(300, 3000, 22);
    Bytes  bad = data_frame(999, 999, 20);
    bad[8] ^= 0x55;
    Bytes s = {0x00, 0x13, 0x5A, 0x59, 0x01};
    s.insert(s.end(), a.begin(), a.end());
    s.insert(s.end(), bad.begin(), bad.end());
    s.push_back(0x5A);
    s.insert(s.end(), b.begin(), b.end());
    s.insert(s.end(), c.begin(), c.end());
    std::vector<Sample> got;
    size_t              i = 0;
    for (size_t chunk : {3u, 7u, 11u, 1u, 100u}) {
        const size_t n = std::min(chunk, s.size() - i);
        Fed          f = feed(p, Bytes(s.begin() + static_cast<long>(i), s.begin() + static_cast<long>(i + n)));
        got.insert(got.end(), f.samples.begin(), f.samples.end());
        i += n;
    }
    CHECK(got.size() == 3);
    if (got.size() == 3) {
        CHECK_NEAR(got[0].range_m, 1.00, 1e-6);
        CHECK_NEAR(got[1].range_m, 2.00, 1e-6);
        CHECK_NEAR(got[2].range_m, 3.00, 1e-6);
    }
    CHECK(p.n_bad_sum() == 1);
    CHECK(p.n_frames() == 3);

    // A good frame hidden right after the start of a broken one: 59 59 then
    // the real frame. The resync must find it.
    Parser q;
    Bytes  t = {0x59, 0x59, 0x10};
    t.insert(t.end(), a.begin(), a.end());
    t.insert(t.end(), b.begin(), b.end());
    const Fed f = feed(q, t);
    CHECK(f.samples.size() >= 1);
    if (!f.samples.empty()) CHECK_NEAR(f.samples.back().range_m, 2.00, 1e-6);
}

static void test_replies()
{
    // Version 9.50.112 (the manual's example), then the frequency echo.
    Bytes v = {0x5A, 0x07, 0x01, 112, 50, 9};
    unsigned sum = 0;
    for (uint8_t x : v) sum += x;
    v.push_back(static_cast<uint8_t>(sum));
    Bytes fr = cmd_sample_freq(100);  // a reply has the same layout as the command

    Parser off;
    CHECK(feed(off, v).replies.empty());

    Parser p;
    p.accept_replies(true);
    Bytes s = {0x00};
    s.insert(s.end(), v.begin(), v.end());
    s.insert(s.end(), fr.begin(), fr.end());
    const Fed f = feed(p, s);
    CHECK(f.replies.size() == 2);
    if (f.replies.size() == 2) {
        CHECK(f.replies[0].id == ID_GET_VERSION);
        CHECK((f.replies[0].payload == Bytes{112, 50, 9}));
        CHECK(f.replies[1].id == ID_SAMPLE_FREQ);
        CHECK((f.replies[1].payload == Bytes{100, 0}));
    }
    // Data frames still come through while replies are accepted.
    const Fed g = feed(p, data_frame(150, 500, 20));
    CHECK(g.samples.size() == 1);
}

static Sample smp(int64_t t_ms, float r, Status st = Status::Ok, uint16_t amp = 1000)
{
    Sample s;
    s.t_ns = t_ms * 1000000;
    s.range_m = r;
    s.amp = amp;
    s.status = st;
    return s;
}

static void test_range_buffer()
{
    RangeBuffer b;
    CHECK(!b.at(0, 15000000, 500000000).valid);
    // 100 Hz around t = 1000 ms; one outlier and one invalid sample.
    b.push(smp(970, 1.00f));
    b.push(smp(980, 1.01f));
    b.push(smp(990, 1.02f));
    b.push(smp(1000, 5.00f));  // a stalk tip: the median ignores it
    b.push(smp(1005, 0.0f, Status::LowAmp));
    b.push(smp(1010, 1.03f));
    Range r = b.at(1000 * 1000000LL, 15000000, 500000000);
    CHECK(r.valid && !r.held);
    CHECK(r.n == 3);  // within 15 ms: 990, 1000, 1010 (980 is 20 ms off, 1005 invalid)
    CHECK_NEAR(r.range_m, 1.03, 1e-6);  // median of 1.02, 5.00, 1.03

    // Far past the last sample: held within 0.5 s, then gone.
    r = b.at(1300 * 1000000LL, 15000000, 500000000);
    CHECK(r.valid && r.held);
    CHECK_NEAR(r.range_m, 1.03, 1e-6);
    CHECK(r.age_ns == 290 * 1000000LL);
    CHECK(!b.at(1600 * 1000000LL, 15000000, 500000000).valid);
    // Before the first sample: nothing to hold.
    CHECK(!b.at(900 * 1000000LL, 15000000, 500000000).valid);

    // The ring keeps the newest 128.
    RangeBuffer c;
    for (int k = 0; k < 300; k++) c.push(smp(k * 4, 2.0f + 0.001f * static_cast<float>(k)));
    CHECK(c.newest_ns() == 299 * 4 * 1000000LL);
    r = c.at(299 * 4 * 1000000LL, 1000000, 0);
    CHECK(r.valid && r.n == 1);
    CHECK_NEAR(r.range_m, 2.299, 1e-5);
    CHECK(!c.at(10 * 4 * 1000000LL, 1000000, 0).valid);  // long gone from the ring
}

static void test_i2c()
{
    // ID_SAVE_SETTINGS: the manual's full sequence is 5A 04 11 00 (sum not
    // checked by default); ours carries the real sum.
    const Bytes save = cmd_save_settings();
    CHECK((save == Bytes{0x5A, 0x04, 0x11, 0x6F}));

    // Registers 0x00..0x09: 123 cm, Amp 1500, 31.25 C (3125 in 0.01 C),
    // tick 0x1234, error 0.
    const uint8_t r[I2C_DATA_LEN] = {123, 0, 0xDC, 0x05, 0x35, 0x0C, 0x34, 0x12, 0, 0};
    const I2cData d = decode_i2c(r, 5000);
    CHECK(d.s.t_ns == 5000);
    CHECK_NEAR(d.s.range_m, 1.23, 1e-6);
    CHECK(d.s.amp == 1500);
    CHECK_NEAR(d.s.temp_c, 31.25, 1e-4);
    CHECK(d.s.ok());
    CHECK(d.tick == 0x1234);
    CHECK(d.error == 0);
    // Weak return: Amp 50, the dummy distance 0.
    const uint8_t w[I2C_DATA_LEN] = {0, 0, 50, 0, 0x35, 0x0C, 0, 0, 0, 0};
    CHECK(decode_i2c(w, 0).s.status == Status::LowAmp);
    // 8.5 m: past the rated range.
    const uint8_t f[I2C_DATA_LEN] = {0x52, 0x03, 0xDC, 0x05, 0x35, 0x0C, 0, 0, 0, 0};
    CHECK(decode_i2c(f, 0).s.status == Status::TooFar);
    // Below 0 C the temperature register is signed.
    const uint8_t c[I2C_DATA_LEN] = {100, 0, 0xDC, 0x05, 0x0C, 0xFE, 0, 0, 0, 0};  // -500
    CHECK_NEAR(decode_i2c(c, 0).s.temp_c, -5.0, 1e-4);

    std::string chip;
    unsigned    line = 0;
    CHECK(parse_gpio_spec("gpiochip2:11", chip, line) && chip == "/dev/gpiochip2" && line == 11);
    CHECK(parse_gpio_spec("/dev/gpiochip4:26", chip, line) && chip == "/dev/gpiochip4" && line == 26);
    CHECK(parse_gpio_spec("2:11", chip, line) && chip == "/dev/gpiochip2" && line == 11);
    CHECK(!parse_gpio_spec("gpiochip2", chip, line));
    CHECK(!parse_gpio_spec("gpiochip2:", chip, line));
    CHECK(!parse_gpio_spec(":11", chip, line));
    CHECK(!parse_gpio_spec("gpiochip2x:11", chip, line));
    CHECK(!parse_gpio_spec("gpiochip2:1a", chip, line));
    CHECK(!parse_gpio_spec("/dev/i2c-7:11", chip, line));
}

int main()
{
    test_commands();
    test_i2c();
    test_classify();
    test_data_frame();
    test_stream();
    test_replies();
    test_range_buffer();
    if (g_fails) {
        std::printf("%d failed\n", g_fails);
        return 1;
    }
    std::printf("all TF-Luna tests passed\n");
    return 0;
}
