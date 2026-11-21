#include "tm171.hpp"

#include <cmath>
#include <cstring>

namespace flow::tm171 {

namespace {

uint32_t rd_u32(const uint8_t* b)
{
    return static_cast<uint32_t>(b[0]) | (static_cast<uint32_t>(b[1]) << 8) | (static_cast<uint32_t>(b[2]) << 16) |
           (static_cast<uint32_t>(b[3]) << 24);
}
uint16_t rd_u16(const uint8_t* b) { return static_cast<uint16_t>(b[0] | (b[1] << 8)); }
float    rd_f32(const uint8_t* b)
{
    const uint32_t u = rd_u32(b);
    float          f;
    std::memcpy(&f, &u, sizeof f);
    return f;
}
void wr_u16(uint8_t* b, uint16_t v)
{
    b[0] = static_cast<uint8_t>(v);
    b[1] = static_cast<uint8_t>(v >> 8);
}
void wr_u32(uint8_t* b, uint32_t v)
{
    for (int i = 0; i < 4; i++) b[i] = static_cast<uint8_t>(v >> (8 * i));
}

constexpr uint32_t MUST_BE_0 = SW_QUAT_OUT | SW_EULER_OUT | SW_GRAVITY_OUT | SW_TEMP_CALIB | SW_BOOT_MODE_MASK |
                               SW_CMD42_OUT | SW_COMBO_OUT | SW_SLOW_OUT;
constexpr uint32_t MUST_BE_1 = SW_STATUS_OUT | SW_RAW_OUT | SW_RPY_OUT | SW_USB_OUT;

}  // namespace

uint16_t crc16(const uint8_t* p, size_t n)
{
    uint16_t c = 0xFFFF;
    while (n--) {
        c ^= *p++;
        for (int i = 0; i < 8; i++) c = (c & 1) ? static_cast<uint16_t>((c >> 1) ^ 0xA001) : static_cast<uint16_t>(c >> 1);
    }
    return c;
}

std::vector<uint8_t> encode(uint8_t cmd, uint16_t to_id, const uint8_t* payload, size_t n)
{
    std::vector<uint8_t> out{0xAA, 0x55, static_cast<uint8_t>(4 + n)};
    uint8_t              h[4];
    wr_u32(h, static_cast<uint32_t>(cmd & 0x7F) | (static_cast<uint32_t>(ID_HOST) << 10) |
                  (static_cast<uint32_t>(to_id & 0x7FF) << 21));
    out.insert(out.end(), h, h + 4);
    out.insert(out.end(), payload, payload + n);
    const uint16_t c = crc16(&out[2], out.size() - 2);
    out.push_back(static_cast<uint8_t>(c));
    out.push_back(static_cast<uint8_t>(c >> 8));
    return out;
}

std::vector<uint8_t> request(uint8_t cmd)
{
    const uint8_t p[4] = {cmd, 0, 0, 0};
    return encode(CMD_REQUEST, ID_BROADCAST, p, sizeof p);
}

void Parser::feed(const uint8_t* p, size_t n, int64_t t_ns, std::vector<Packet>& out)
{
    buf_.insert(buf_.end(), p, p + n);
    size_t i = 0;
    for (;;) {
        // Find AA 55.
        size_t j = i;
        while (j + 1 < buf_.size() && !(buf_[j] == 0xAA && buf_[j + 1] == 0x55)) j++;
        if (j + 1 >= buf_.size()) {
            // Keep a trailing 0xAA: it may start the next packet.
            const size_t keep = (!buf_.empty() && buf_.back() == 0xAA && buf_.size() - 1 >= i) ? buf_.size() - 1 : buf_.size();
            skipped_ += keep - i;
            i = keep;
            break;
        }
        skipped_ += j - i;
        if (j + 3 > buf_.size()) {
            i = j;
            break;
        }
        const size_t len = buf_[j + 2];
        if (len < 4) {
            skipped_++;
            i = j + 1;
            continue;
        }
        const size_t end = j + 3 + len + 2;
        if (end > buf_.size()) {
            i = j;
            break;
        }
        const uint16_t c = crc16(&buf_[j + 2], 1 + len);
        if (c != rd_u16(&buf_[j + 3 + len])) {
            crc_errors_++;
            i = j + 1;
            continue;
        }
        const uint32_t h = rd_u32(&buf_[j + 3]);
        Packet         pk;
        pk.cmd = static_cast<uint8_t>(h & 0x7F);
        pk.from_id = static_cast<uint16_t>((h >> 10) & 0x7FF);
        pk.payload.assign(buf_.begin() + static_cast<long>(j + 7), buf_.begin() + static_cast<long>(j + 3 + len));
        pk.t_ns = t_ns;
        out.push_back(std::move(pk));
        i = end;
    }
    buf_.erase(buf_.begin(), buf_.begin() + static_cast<long>(i));
}

uint32_t switches(const Setting& s) { return rd_u32(&s[0]); }
uint16_t baud_100(const Setting& s) { return rd_u16(&s[6]); }
uint16_t silent_s(const Setting& s) { return rd_u16(&s[16]); }
uint16_t odr_hz(const Setting& s) { return rd_u16(&s[18]); }

bool layout_ok(const Setting& s)
{
    const uint32_t sw = switches(s);
    return (sw & SW_RESERVED3) && (sw & SW_RESERVED4_MASK) == SW_RESERVED4_VALUE;
}

Setting target(const Setting& from, bool save_flash)
{
    Setting  t = from;
    uint32_t sw = switches(from);
    sw &= ~(MUST_BE_0 | SW_SAVE_FLASH);
    sw |= MUST_BE_1 | SW_REQUEST_ACK;
    if (save_flash) sw |= SW_SAVE_FLASH;
    wr_u32(&t[0], sw);
    wr_u16(&t[6], TARGET_BAUD_100);
    wr_u16(&t[16], 0);
    wr_u16(&t[18], TARGET_ODR_HZ);
    return t;
}

bool matches_target(const Setting& s)
{
    const uint32_t ignore = SW_REQUEST_ACK | SW_SAVE_FLASH;
    const Setting  t = target(s, false);
    return (switches(s) & ~ignore) == (switches(t) & ~ignore) && std::memcmp(&s[4], &t[4], SETTING_LEN - 4) == 0;
}

bool decode_raw(const std::vector<uint8_t>& p, Raw& out)
{
    if (p.size() != RAW_LEN) return false;
    out.ts_us = rd_u32(&p[0]);
    const float gx = rd_f32(&p[4]), gy = rd_f32(&p[8]), gz = rd_f32(&p[12]);
    out.gyr_frd[0] = gx;
    out.gyr_frd[1] = -gy;
    out.gyr_frd[2] = -gz;
    for (int i = 0; i < 3; i++) out.acc_flu_g[i] = rd_f32(&p[16 + 4 * static_cast<size_t>(i)]);
    return true;
}

bool decode_rpy(const std::vector<uint8_t>& p, Rpy& out)
{
    if (p.size() != RPY_LEN) return false;
    out.ts_us = rd_u32(&p[0]);
    out.roll_deg = rd_f32(&p[4]);
    out.pitch_deg = rd_f32(&p[8]);
    out.yaw_deg = rd_f32(&p[12]);
    return true;
}

bool decode_status(const std::vector<uint8_t>& p, Status& out)
{
    if (p.size() != STATUS_LEN) return false;
    out.ts_us = rd_u32(&p[0]);
    out.temp_c = rd_f32(&p[4]);
    out.rate_hz = rd_u16(&p[8]);
    out.qos = p[10];
    return true;
}

int64_t ClockMap::map(uint32_t ts_us, int64_t arrival_ns)
{
    constexpr int64_t k_window_ns = 1000000000;
    constexpr int32_t k_max_step_us = 100000;  // a larger step is a restart
    if (have_) {
        const int32_t d = static_cast<int32_t>(ts_us - last_ts_);
        if (d <= 0 || d > k_max_step_us) {
            have_ = false;
            jumps_++;
        }
    }
    if (!have_) {
        have_ = true;
        have_prev_ = false;
        last_ts_ = ts_us;
        t_us_ = ts_us;
        off_cur_ = arrival_ns - t_us_ * 1000;
        win_end_ns_ = arrival_ns + k_window_ns;
        return 0;
    }
    t_us_ += static_cast<int32_t>(ts_us - last_ts_);
    last_ts_ = ts_us;
    const int64_t d = arrival_ns - t_us_ * 1000;
    if (arrival_ns >= win_end_ns_) {
        off_prev_ = off_cur_;
        have_prev_ = true;
        off_cur_ = d;
        win_end_ns_ = arrival_ns + k_window_ns;
    } else if (d < off_cur_) {
        off_cur_ = d;
    }
    const int64_t off = (have_prev_ && off_prev_ < off_cur_) ? off_prev_ : off_cur_;
    int64_t       t = t_us_ * 1000 + off;
    if (t <= last_out_) t = last_out_ + 1;  // a falling offset must not turn time back
    last_out_ = t;
    return t;
}

void GyroBuffer::push(int64_t t_ns, const float g[3])
{
    GyroSample& s = ring_[n_ % ring_.size()];
    s.t_ns = t_ns;
    for (int i = 0; i < 3; i++) s.g[i] = g[i];
    n_++;
}

bool GyroBuffer::integral(int64_t ta, int64_t tb, float out[3], int64_t max_gap_ns) const
{
    out[0] = out[1] = out[2] = 0;
    const size_t cap = ring_.size();
    if (n_ < 2 || tb <= ta) return false;
    const size_t oldest = n_ > cap ? n_ - cap : 0;
    if (ring_[(n_ - 1) % cap].t_ns < tb) return false;  // not yet covered
    if (ring_[oldest % cap].t_ns > ta) return false;    // already gone
    double acc[3] = {0, 0, 0};
    for (size_t i = n_ - 1; i > oldest; i--) {
        const GyroSample& cur = ring_[i % cap];
        const GyroSample& prev = ring_[(i - 1) % cap];
        if (cur.t_ns <= ta) break;
        if (cur.t_ns - prev.t_ns > max_gap_ns && prev.t_ns < tb) return false;
        const int64_t a = prev.t_ns > ta ? prev.t_ns : ta;
        const int64_t b = cur.t_ns < tb ? cur.t_ns : tb;
        if (b > a) {
            const double dt = static_cast<double>(b - a) * 1e-9;
            for (int k = 0; k < 3; k++) acc[k] += static_cast<double>(cur.g[k]) * dt;
        }
    }
    for (int k = 0; k < 3; k++) out[k] = static_cast<float>(acc[k]);
    return true;
}

void cam_to_body(float cam_yaw_rad, float r_cb[9])
{
    const float c = std::cos(cam_yaw_rad), sn = std::sin(cam_yaw_rad);
    const float m0[9] = {0, -1, 0, 1, 0, 0, 0, 0, 1};
    const float rz[9] = {c, -sn, 0, sn, c, 0, 0, 0, 1};
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            float a = 0;
            for (int k = 0; k < 3; k++) a += rz[i * 3 + k] * m0[k * 3 + j];
            r_cb[i * 3 + j] = a;
        }
    }
}

void mount_to_body(const float rpy[3], float r[9])
{
    const float cr = std::cos(rpy[0]), sr = std::sin(rpy[0]);
    const float cp = std::cos(rpy[1]), sp = std::sin(rpy[1]);
    const float cy = std::cos(rpy[2]), sy = std::sin(rpy[2]);
    const float m[9] = {cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
                        sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
                        -sp,     cp * sr,                cp * cr};
    for (int i = 0; i < 9; i++) r[i] = m[i];
}

void mul3(const float r[9], const float v[3], float out[3])
{
    for (int i = 0; i < 3; i++) out[i] = r[i * 3] * v[0] + r[i * 3 + 1] * v[1] + r[i * 3 + 2] * v[2];
}

void derotate(const float r_cb[9], float disp_x, float disp_y, const float dth_b[3], float z_m, float dt_s,
              float v_cam[2])
{
    float dth_c[3];
    for (int i = 0; i < 3; i++) dth_c[i] = r_cb[0 * 3 + i] * dth_b[0] + r_cb[1 * 3 + i] * dth_b[1] + r_cb[2 * 3 + i] * dth_b[2];
    v_cam[0] = -z_m * (disp_x + dth_c[1]) / dt_s;
    v_cam[1] = -z_m * (disp_y - dth_c[0]) / dt_s;
}

}  // namespace flow::tm171
