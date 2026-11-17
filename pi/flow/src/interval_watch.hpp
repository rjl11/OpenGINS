// Classifies the interval between two received frames (Pi side).
//
// The Pico leaves out one trigger pulse every FLOW_LINK_GAP_PERIOD (256). A
// frame interval is one pulse (normal), two pulses (the left-out pulse,
// FLOW_F_AFTER_GAP), or more (frames lost on the Pi, FLOW_F_DROPPED).
//
// A lost frame also makes a two-pulse interval, so the length alone cannot
// tell it from the gap. The watch counts trigger pulses instead: a two-pulse
// interval is a gap only when the pulse it skips is in the phase of the last
// gap (mod FLOW_LINK_GAP_PERIOD). A lost frame skips an ordinary pulse, so it
// is DROPPED. Mirror of pi/flow/python/interval_watch.py; keep both in step.
#pragma once

#include <algorithm>
#include <cstdint>
#include <deque>
#include <vector>

#include "flow_link.h"

namespace flow {

class IntervalWatch {
public:
    explicit IntervalWatch(size_t n = 32) : n_(n) {}

    uint8_t classify(int64_t dt_ns)
    {
        if (recent_.size() < 8) {
            remember(dt_ns);
            return 0;
        }
        std::vector<int64_t> v(recent_.begin(), recent_.end());
        std::nth_element(v.begin(), v.begin() + static_cast<long>(v.size() / 2), v.end());
        const double nominal = static_cast<double>(v[v.size() / 2]);
        const int64_t k = std::max<int64_t>(1, static_cast<int64_t>(static_cast<double>(dt_ns) / nominal + 0.5));
        if (k == 1) {
            remember(dt_ns);
            pulses_ += 1;
            return 0;
        }

        // The pulses this interval skips; a gap is one of them in the phase.
        const uint64_t first_skipped = pulses_ + 1;
        const bool expired = anchored_ && pulses_ - anchor_p_ >= FLOW_LINK_GAP_PERIOD;
        bool in_phase = false;
        if (anchored_) {
            for (int64_t i = 0; i < k - 1; i++) {
                if ((first_skipped + static_cast<uint64_t>(i)) % FLOW_LINK_GAP_PERIOD == phase_) in_phase = true;
            }
        }
        const uint64_t frame_p = pulses_ + static_cast<uint64_t>(k);
        pulses_ = frame_p;

        if (k == 2 && (!anchored_ || expired || first_skipped % FLOW_LINK_GAP_PERIOD == phase_)) {
            // The first gap seen, a gap in phase, or the anchor expired: a real
            // gap comes at most 255 pulses after the frame before it, so a full
            // period without one means the anchor is wrong.
            anchored_ = true;
            phase_ = first_skipped % FLOW_LINK_GAP_PERIOD;
            anchor_p_ = frame_p;
            return FLOW_F_AFTER_GAP;
        }
        if (in_phase) {
            // A gap that came with a lost frame (k > 2): not a usable gap, but
            // the phase is still right, so the anchor stays fresh.
            anchor_p_ = frame_p;
        }
        return FLOW_F_DROPPED;
    }

private:
    void remember(int64_t dt_ns)
    {
        recent_.push_back(dt_ns);
        if (recent_.size() > n_) recent_.pop_front();
    }

    size_t              n_;
    std::deque<int64_t> recent_; // normal intervals, for the nominal period
    uint64_t            pulses_ = 0;   // trigger pulse index of the last received frame
    bool                anchored_ = false;
    uint64_t            phase_ = 0;    // skipped-pulse index mod the period, of the last gap
    uint64_t            anchor_p_ = 0; // pulse index of the frame after that gap
};

}  // namespace flow
