#include "tfluna.hpp"

#include <algorithm>

namespace flow::tfluna {

const char* status_name(Status s)
{
    switch (s) {
    case Status::Ok: return "ok";
    case Status::LowAmp: return "low_amp";
    case Status::Overload: return "overload";
    case Status::TooClose: return "too_close";
    case Status::TooFar: return "too_far";
    }
    return "?";
}

Status classify(double range_m, unsigned amp)
{
    if (amp > AMP_OVERLOAD) return Status::Overload;  // includes 65535, overexposed
    if (amp < AMP_MIN) return Status::LowAmp;         // and Dist is then the dummy value, 0
    if (range_m < RANGE_MIN_M) return Status::TooClose;
    if (range_m > RANGE_MAX_M) return Status::TooFar;
    return Status::Ok;
}

static uint8_t sum8(const uint8_t* p, size_t n)
{
    unsigned s = 0;
    for (size_t i = 0; i < n; i++) s += p[i];
    return static_cast<uint8_t>(s);
}

std::vector<uint8_t> command(uint8_t id, const uint8_t* payload, size_t n)
{
    std::vector<uint8_t> f;
    f.reserve(n + 4);
    f.push_back(0x5A);
    f.push_back(static_cast<uint8_t>(n + 4));
    f.push_back(id);
    for (size_t i = 0; i < n; i++) f.push_back(payload[i]);
    f.push_back(sum8(f.data(), f.size()));
    return f;
}

std::vector<uint8_t> cmd_output_enable(bool on)
{
    const uint8_t p[1] = {static_cast<uint8_t>(on ? 1 : 0)};
    return command(ID_OUTPUT_EN, p, 1);
}

std::vector<uint8_t> cmd_sample_freq(uint16_t hz)
{
    const uint8_t p[2] = {static_cast<uint8_t>(hz), static_cast<uint8_t>(hz >> 8)};
    return command(ID_SAMPLE_FREQ, p, 2);
}

std::vector<uint8_t> cmd_output_format(uint8_t fmt) { return command(ID_OUTPUT_FORMAT, &fmt, 1); }
std::vector<uint8_t> cmd_filter(uint8_t bitmap) { return command(ID_FILTER_BIT_MAP, &bitmap, 1); }

std::vector<uint8_t> cmd_dealias(bool dual)
{
    const uint8_t p[1] = {static_cast<uint8_t>(dual ? 1 : 0)};
    return command(ID_DEALIAS_EN, p, 1);
}

std::vector<uint8_t> cmd_get_version() { return command(ID_GET_VERSION); }

void Parser::feed(const uint8_t* p, size_t n, int64_t t_last_ns, int64_t byte_ns, std::vector<Sample>& samples,
                  std::vector<Reply>& replies)
{
    for (size_t i = 0; i < n; i++) {
        const int64_t t = t_last_ns - static_cast<int64_t>(n - 1 - i) * byte_ns;
        step(p[i], t, byte_ns, samples, replies);
    }
}

// A frame failed its sum (or a reply its length): drop its first byte and
// look for a frame in the rest, which may hold the start of a good one.
void Parser::resync(int64_t t_ns, int64_t byte_ns, std::vector<Sample>& samples, std::vector<Reply>& replies)
{
    std::array<uint8_t, 256> b = buf_;
    const size_t              n = n_;
    st_ = St::Hunt;
    n_ = 0;
    n_skipped_++;
    for (size_t k = 1; k < n; k++) step(b[k], t_ns - static_cast<int64_t>(n - 1 - k) * byte_ns, byte_ns, samples, replies);
}

void Parser::step(uint8_t b, int64_t t_ns, int64_t byte_ns, std::vector<Sample>& samples, std::vector<Reply>& replies)
{
    switch (st_) {
    case St::Hunt:
        if (b == 0x59) {
            st_ = St::DataH2;
        } else if (b == 0x5A && replies_) {
            buf_[0] = b;
            n_ = 1;
            st_ = St::ReplyLen;
        } else {
            n_skipped_++;
        }
        return;
    case St::DataH2:
        if (b == 0x59) {
            buf_[0] = buf_[1] = 0x59;
            n_ = 2;
            st_ = St::Data;
        } else {
            n_skipped_++;  // the lone 0x59
            st_ = St::Hunt;
            step(b, t_ns, byte_ns, samples, replies);
        }
        return;
    case St::Data:
        buf_[n_++] = b;
        if (n_ < DATA_FRAME_LEN) return;
        if (sum8(buf_.data(), DATA_FRAME_LEN - 1) != buf_[DATA_FRAME_LEN - 1]) {
            n_bad_sum_++;
            resync(t_ns, byte_ns, samples, replies);
            return;
        }
        {
            Sample         s;
            const unsigned dist = buf_[2] | (buf_[3] << 8);
            const unsigned amp = buf_[4] | (buf_[5] << 8);
            const unsigned temp = buf_[6] | (buf_[7] << 8);
            s.t_ns = t_ns - static_cast<int64_t>(DATA_FRAME_LEN - 1) * byte_ns - meas_delay_ns_;
            s.range_m = static_cast<float>(dist * unit_m_);
            s.amp = static_cast<uint16_t>(amp);
            s.temp_c = static_cast<float>(temp) / 8.0f - 256.0f;
            s.status = classify(s.range_m, amp);
            samples.push_back(s);
            n_frames_++;
        }
        st_ = St::Hunt;
        n_ = 0;
        return;
    case St::ReplyLen:
        // Our replies are at most 30 bytes (ID_GET_FULL_VERSION); a longer
        // length is a 0x5A inside something else.
        if (b < 4 || b > 64) {
            n_skipped_++;  // the 0x5A
            st_ = St::Hunt;
            n_ = 0;
            step(b, t_ns, byte_ns, samples, replies);
            return;
        }
        buf_[n_++] = b;
        want_ = b;
        st_ = St::Reply;
        return;
    case St::Reply:
        buf_[n_++] = b;
        if (n_ < want_) return;
        if (sum8(buf_.data(), want_ - 1) != buf_[want_ - 1]) {
            n_bad_sum_++;
            resync(t_ns, byte_ns, samples, replies);
            return;
        }
        {
            Reply r;
            r.id = buf_[2];
            r.payload.assign(buf_.begin() + 3, buf_.begin() + static_cast<std::ptrdiff_t>(want_ - 1));
            replies.push_back(std::move(r));
        }
        st_ = St::Hunt;
        n_ = 0;
        return;
    }
}

void RangeBuffer::push(const Sample& s)
{
    ring_[head_] = s;
    head_ = (head_ + 1) % N;
    if (count_ < N) count_++;
}

// Median of v[0..n), n > 0; reorders v.
template <typename T>
static T median(T* v, size_t n)
{
    const size_t m = n / 2;
    std::nth_element(v, v + m, v + n);
    if (n % 2) return v[m];
    const T hi = v[m];
    const T lo = *std::max_element(v, v + m);
    return static_cast<T>((lo + hi) / 2);
}

Range RangeBuffer::at(int64_t t_ns, int64_t half_window_ns, int64_t hold_ns) const
{
    // On the stack: this runs at every camera frame, under the reader's lock.
    Range                   r;
    std::array<float, N>    ranges;
    std::array<unsigned, N> amps;
    size_t                  m = 0;
    int64_t                 newest = 0;
    const Sample*         last_before = nullptr;  // newest valid sample up to t + half window
    for (size_t k = 0; k < count_; k++) {
        const Sample& s = ring_[(head_ + N - count_ + k) % N];  // oldest first
        if (!s.ok() || s.t_ns > t_ns + half_window_ns) continue;
        last_before = &s;
        if (s.t_ns >= t_ns - half_window_ns) {
            ranges[m] = s.range_m;
            amps[m] = s.amp;
            m++;
            newest = s.t_ns;
        }
    }
    if (m) {
        r.valid = true;
        r.n = static_cast<int>(m);
        r.range_m = median(ranges.data(), m);
        r.amp = static_cast<uint16_t>(median(amps.data(), m));
        r.age_ns = t_ns - newest;
        return r;
    }
    if (last_before && t_ns - last_before->t_ns <= hold_ns) {
        r.valid = true;
        r.held = true;
        r.n = 1;
        r.range_m = last_before->range_m;
        r.amp = last_before->amp;
        r.age_ns = t_ns - last_before->t_ns;
    }
    return r;
}

}  // namespace flow::tfluna
