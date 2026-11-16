"""Flow tests on synthetic texture: a known pixel shift must come out as the
matching normalised displacement d/f, with the sign "where the pattern moved in the image".

    python3 -m unittest -v test_flow_lk          (from pi/flow, needs OpenCV)
"""

import unittest

import cv2
import numpy as np

from flow_lk import FlowEstimator, Intrinsics


def texture(w=800, h=560, seed=1):
    rng = np.random.default_rng(seed)
    img = rng.integers(0, 256, (h // 8, w // 8), dtype=np.uint8)
    img = cv2.resize(img, (w, h), interpolation=cv2.INTER_CUBIC)
    return cv2.GaussianBlur(img, (5, 5), 1.0)


def shifted(img, dx, dy, w=640, h=400):
    """Crop w x h from img, the second one with the content moved by (dx, dy) px."""
    M = np.float32([[1, 0, dx], [0, 1, dy]])
    moved = cv2.warpAffine(img, M, (img.shape[1], img.shape[0]), flags=cv2.INTER_LINEAR)
    y0, x0 = (img.shape[0] - h) // 2, (img.shape[1] - w) // 2
    return img[y0 : y0 + h, x0 : x0 + w], moved[y0 : y0 + h, x0 : x0 + w]


class FlowTest(unittest.TestCase):
    def setUp(self):
        self.intr = Intrinsics.nominal(640, 400)
        self.est = FlowEstimator(self.intr)
        self.tex = texture()

    def check_shift(self, dx, dy):
        a, b = shifted(self.tex, dx, dy)
        r = self.est.estimate(a, b)
        self.assertTrue(r.valid)
        self.assertGreater(r.n_tracked, 30)
        # A uniform shift of d px is d / f in normalised coordinates.
        self.assertAlmostEqual(r.disp_x_rad, dx / self.intr.fx, delta=2e-4)
        self.assertAlmostEqual(r.disp_y_rad, dy / self.intr.fy, delta=2e-4)
        self.assertLess(r.stddev_rad, 1e-4)

    def test_zero(self):
        self.check_shift(0.0, 0.0)

    def test_subpixel(self):
        self.check_shift(0.4, -0.3)

    def test_large(self):
        self.check_shift(12.0, 7.0)

    def test_sequence_keeps_features(self):
        """update() over a moving sequence: right shift every frame, and corner
        detection only now and then (features leave at the edge)."""
        big = texture(1400, 900)
        dx, dy = 2.5, -1.5
        est = FlowEstimator(self.intr)
        self.assertIsNone(est.update(shifted(big, 0, 0)[1]))
        for i in range(1, 60):
            r = est.update(shifted(big, dx * i, dy * i)[1])
            self.assertTrue(r.valid, i)
            self.assertAlmostEqual(r.disp_x_rad, dx / self.intr.fx, delta=2e-4)
            self.assertAlmostEqual(r.disp_y_rad, dy / self.intr.fy, delta=2e-4)
        self.assertLess(est.n_detect, 15)

    def test_flat_image_is_invalid(self):
        flat = np.full((400, 640), 128, np.uint8)
        self.assertFalse(self.est.estimate(flat, flat).valid)


if __name__ == "__main__":
    unittest.main()
