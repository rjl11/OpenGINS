"""IntervalWatch: the gap is told apart from a frame lost on the Pi.

    python3 -m unittest -v test_interval_watch        (from pi/flow/python)
"""

import random
import unittest

import flow_link as fl
from interval_watch import IntervalWatch

PERIOD_NS = 10_000_000  # 100 Hz trigger


def run(pulses_per_frame, drops=frozenset()):
    """Feed the watch the intervals of received frames.

    pulses_per_frame: the trigger pulse index of every triggered frame, in
    order, with the Pico's left-out pulses already skipped. drops: indices of
    frames the Pi did not receive. Returns (received index, flags) per
    received frame after the first."""
    watch = IntervalWatch()
    out = []
    prev = None
    for i, p in enumerate(pulses_per_frame):
        if i in drops:
            continue
        if prev is not None:
            out.append((i, watch.classify((p - prev) * PERIOD_NS)))
        prev = p
    return out


def pulses_with_gaps(n_frames):
    """Trigger pulse index of frame i: pulse 256k is left out."""
    pulses = []
    p = 1
    while len(pulses) < n_frames:
        if p % fl.GAP_PERIOD != 0:
            pulses.append(p)
        p += 1
    return pulses


class IntervalWatchTest(unittest.TestCase):
    def test_gap_then_normal(self):
        w = IntervalWatch()
        for _ in range(10):
            self.assertEqual(w.classify(PERIOD_NS), 0)
        self.assertEqual(w.classify(2 * PERIOD_NS), fl.F_AFTER_GAP)
        self.assertEqual(w.classify(PERIOD_NS), 0)
        self.assertEqual(w.classify(4 * PERIOD_NS), fl.F_DROPPED)

    def test_every_real_gap_is_a_gap(self):
        frames = pulses_with_gaps(256 * 10)
        flags = dict(run(frames))
        for i in range(1, len(frames)):
            # Pulse distance 2 across a left-out pulse (the Pico's gap).
            if frames[i] - frames[i - 1] == 2:
                self.assertEqual(flags.get(i, 0) & fl.F_AFTER_GAP, fl.F_AFTER_GAP, f"frame {i}")
        self.assertEqual(sum(1 for v in flags.values() if v & fl.F_DROPPED), 0)

    def test_lost_frame_is_dropped_not_gap(self):
        # A frame lost in the middle of a gap cycle: two pulses, no gap.
        frames = pulses_with_gaps(256 * 3)
        lost = 300  # after the first gap (frame 254): the anchor is right
        flags = dict(run(frames, drops={lost}))
        # The interval over the lost frame is DROPPED (index lost + 1).
        self.assertEqual(flags[lost + 1], fl.F_DROPPED)
        # The real gaps after it are still gaps.
        for i in range(1, len(frames)):
            if i == lost + 1 or i <= lost:
                continue
            if frames[i] - frames[i - 1] == 2:
                self.assertEqual(flags[i] & fl.F_AFTER_GAP, fl.F_AFTER_GAP, f"frame {i}")

    def test_lost_frame_right_after_gap_is_dropped(self):
        frames = pulses_with_gaps(256 * 3)
        gap_i = next(i for i in range(1, len(frames)) if frames[i] - frames[i - 1] == 2 and i > 300)
        flags = dict(run(frames, drops={gap_i + 3}))
        self.assertEqual(flags[gap_i + 4], fl.F_DROPPED)

    def test_wrong_anchor_recovers_within_a_period(self):
        # First gap seen is really a lost frame: the watch anchors on it
        # wrongly, then the real gaps come at another phase and must be
        # accepted again within one gap period.
        frames = pulses_with_gaps(256 * 6)
        drops = {5}  # its interval looks like a gap at the start
        flags = dict(run(frames, drops=drops))
        real = [i for i in range(10, len(frames)) if frames[i] - frames[i - 1] == 2 and i not in drops]
        later = [i for i in real if i > 256 * 3]
        for i in later:
            self.assertEqual(flags.get(i, 0) & fl.F_AFTER_GAP, fl.F_AFTER_GAP, f"frame {i}")

    def test_random_drops_match_the_pulse_rule(self):
        # Ground truth from the pulse indices: a two-pulse interval is a gap
        # when its skipped pulse is a left-out one, else a lost frame. Drops
        # start after the first gap (frame 254), so the anchor is the real one.
        rng = random.Random(1)
        frames = pulses_with_gaps(256 * 20)
        drops = {i for i in range(300, len(frames)) if rng.random() < 0.02}
        got = dict(run(frames, drops=drops))
        received = [i for i in range(len(frames)) if i not in drops]
        for a, b in zip(received, received[1:]):
            k = frames[b] - frames[a]
            if k == 1:
                want = 0
            elif k == 2 and (frames[a] + 1) % fl.GAP_PERIOD == 0:
                want = fl.F_AFTER_GAP
            else:
                want = fl.F_DROPPED
            if b >= 300:
                self.assertEqual(got[b], want, f"frame {b} (after {a}, k={k})")

if __name__ == "__main__":
    unittest.main()
