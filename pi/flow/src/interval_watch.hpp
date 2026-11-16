// Classifies the interval between two frames against the recent normal one:
// normal, the Pico's left-out trigger pulse (about 2x, FLOW_F_AFTER_GAP), or
// a frame lost on the Pi (anything longer, FLOW_F_DROPPED).
// Same rules as IntervalWatch in python/main.py.
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
        uint8_t flags = 0;
        if (recent_.size() >= 8) {
            std::vector<int64_t> v(recent_.begin(), recent_.end());
            std::nth_element(v.begin(), v.begin() + static_cast<long>(v.size() / 2), v.end());
            const double r = static_cast<double>(dt_ns) / static_cast<double>(v[v.size() / 2]);
            if (r > 1.5 && r < 2.5) {
                flags = FLOW_F_AFTER_GAP;
            } else if (r >= 2.5) {
                flags = FLOW_F_DROPPED;
            }
        }
        if (flags == 0) {
            recent_.push_back(dt_ns);
            if (recent_.size() > n_) recent_.pop_front();
        }
        return flags;
    }

private:
    size_t              n_;
    std::deque<int64_t> recent_;
};

}  // namespace flow
