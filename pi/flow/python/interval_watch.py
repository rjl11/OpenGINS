"""Classifies the interval between two received frames (Pi side).

The Pico leaves out one trigger pulse every FLOW_LINK_GAP_PERIOD (256). A
frame interval is therefore one pulse (normal), two pulses (the left-out
pulse, FLOW_F_AFTER_GAP), or more (frames lost on the Pi, FLOW_F_DROPPED).

A lost frame also makes a two-pulse interval, so the length alone cannot tell
it from the gap. The watch counts trigger pulses instead: a two-pulse interval
is a gap only when the pulse it skips is in the phase of the last gap (mod
FLOW_LINK_GAP_PERIOD). A lost frame skips an ordinary pulse, so it is
DROPPED. Mirror of pi/flow/src/interval_watch.hpp; keep both in step.
"""

from collections import deque

import flow_link as fl

GAP_PERIOD = fl.GAP_PERIOD


class IntervalWatch:
    def __init__(self, n=32):
        self.recent = deque(maxlen=n)  # normal intervals, for the nominal period
        self.pulses = 0  # trigger pulse index of the last received frame
        self.anchored = False
        self.phase = 0  # skipped-pulse index mod GAP_PERIOD of the last accepted gap
        self.anchor_p = 0  # pulse index of the frame after that gap

    def classify(self, dt_ns: int) -> int:
        # The middle element of the sorted recent intervals (as the C++ side).
        nominal = sorted(self.recent)[len(self.recent) // 2] if len(self.recent) >= 8 else None
        if nominal is None:
            self.recent.append(dt_ns)
            return 0
        k = max(1, int(dt_ns / nominal + 0.5))  # pulses in this interval
        if k == 1:
            self.recent.append(dt_ns)
            self.pulses += 1
            return 0
        # Pulses this interval skips: the Pico's gap if one is in the phase.
        expired = self.anchored and self.pulses - self.anchor_p >= GAP_PERIOD
        skipped = range(self.pulses + 1, self.pulses + k)
        in_phase = self.anchored and any(s % GAP_PERIOD == self.phase for s in skipped)
        frame_p = self.pulses + k
        self.pulses = frame_p
        if k == 2 and (not self.anchored or expired or skipped[0] % GAP_PERIOD == self.phase):
            # The first gap seen, a gap in phase, or the anchor expired: a real
            # gap comes at most 255 pulses after the frame before it, so a full
            # period without one means the anchor is wrong.
            self.anchored = True
            self.phase = skipped[0] % GAP_PERIOD
            self.anchor_p = frame_p
            return fl.F_AFTER_GAP
        if in_phase:
            # A gap that came with a lost frame (k > 2): not a usable gap, but
            # the phase is still right, so the anchor stays fresh.
            self.anchor_p = frame_p
        return fl.F_DROPPED
