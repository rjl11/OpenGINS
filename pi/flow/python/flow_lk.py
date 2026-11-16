"""Optical flow between two consecutive grey frames.

Sparse Lucas-Kanade on corner features with a forward-backward check; the
result is the median image displacement in normalised image coordinates
(x/z, y/z: radians near the centre) on the camera's x (right) and y (down)
axes, after removing lens distortion. Rotation is NOT removed and
nothing is divided by time or scaled by height: the Pico does that with its
gyro, its strobe timestamps and the height above ground
(plans/optical-flow-aiding.md, "Split of work").

Needs numpy and OpenCV only, so it runs on a PC on recorded frames too.
"""

from dataclasses import dataclass, field
import json
from pathlib import Path

import cv2
import numpy as np


@dataclass
class Intrinsics:
    """Pinhole camera at the processing resolution, plus distortion.

    fisheye=True uses OpenCV's fisheye model (4 coefficients k1..k4), which
    suits the module's 148 deg 2.8 mm lens; otherwise the standard model
    (k1, k2, p1, p2[, k3]). An empty dist means no distortion correction."""

    fx: float
    fy: float
    cx: float
    cy: float
    dist: list = field(default_factory=list)
    fisheye: bool = False

    @classmethod
    def load(cls, path: Path) -> "Intrinsics":
        return cls(**json.loads(Path(path).read_text(encoding="utf-8")))

    @classmethod
    def nominal(cls, width: int, height: int) -> "Intrinsics":
        """Rough default for the CAM-MIPI9281RAW-V2 lens: 2.8 mm on 3 um pixels
        at 1280x800 (933 px), scaled to the processing size. No distortion
        model, so only the image centre is accurate: calibrate before trusting
        the scale (README, "Calibration")."""
        f = 2.8e-3 / 3e-6 * width / 1280.0
        return cls(fx=f, fy=f, cx=(width - 1) / 2.0, cy=(height - 1) / 2.0)

    def to_normalised(self, pts: np.ndarray) -> np.ndarray:
        """Pixel points (N,2) to normalised image coordinates (N,2), undistorted."""
        p = pts.reshape(-1, 1, 2).astype(np.float64)
        K = np.array([[self.fx, 0, self.cx], [0, self.fy, self.cy], [0, 0, 1]], np.float64)
        if not self.dist:
            n = (p.reshape(-1, 2) - (self.cx, self.cy)) / (self.fx, self.fy)
            return n
        d = np.array(self.dist, np.float64)
        if self.fisheye:
            return cv2.fisheye.undistortPoints(p, K, d).reshape(-1, 2)
        return cv2.undistortPoints(p, K, d).reshape(-1, 2)


@dataclass
class FlowResult:
    disp_x_rad: float = 0.0
    disp_y_rad: float = 0.0
    stddev_rad: float = 0.0
    n_tracked: int = 0
    valid: bool = False


class FlowEstimator:
    """Stateful flow: features found in one frame are tracked into the next
    and kept, so corner detection only runs when too few are left.

    Speed choices (plans/optical-flow-aiding.md, "Speed"): one forward LK pass
    with a small window and 2 pyramid levels (at 100 fps the shift per frame
    is small), no backward check; outliers are removed against the median
    instead. estimate() is the stateless form, used by the tests."""

    def __init__(
        self,
        intr: Intrinsics,
        max_corners: int = 60,
        min_keep: int = 30,
        min_tracked: int = 15,
        win: int = 15,
        levels: int = 2,
        outlier_px: float = 1.0,
        border_px: int = 8,
    ):
        self.intr = intr
        self.max_corners = max_corners
        self.min_keep = min_keep
        self.min_tracked = min_tracked
        self.outlier_px = outlier_px
        self.border_px = border_px
        self.lk = dict(
            winSize=(win, win),
            maxLevel=levels,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03),
        )
        self.prev = None  # last frame given to update()
        self.pts = None  # features in self.prev, (N,1,2) float32
        self.n_detect = 0  # corner detections run (diagnostics)

    def reset(self):
        self.prev = None
        self.pts = None

    def _detect(self, img):
        self.n_detect += 1
        return cv2.goodFeaturesToTrack(
            img, maxCorners=self.max_corners, qualityLevel=0.01, minDistance=8, blockSize=7
        )

    def update(self, cur: np.ndarray):
        """Give the next frame; returns the flow from the previous frame to it,
        or None for the first frame."""
        prev, self.prev = self.prev, cur
        if prev is None:
            self.pts = None
            return None
        if self.pts is None or len(self.pts) < self.min_keep:
            self.pts = self._detect(prev)
        r, self.pts = self._track(prev, cur, self.pts)
        return r

    def estimate(self, prev: np.ndarray, cur: np.ndarray) -> FlowResult:
        """Stateless: flow from prev to cur (8-bit single channel, same size)."""
        r, _ = self._track(prev, cur, self._detect(prev))
        return r

    def _track(self, prev, cur, p0):
        """Returns (result, features to keep, located in cur)."""
        if p0 is None or len(p0) < self.min_tracked:
            return FlowResult(n_tracked=0 if p0 is None else len(p0)), None

        p1, st, _ = cv2.calcOpticalFlowPyrLK(prev, cur, p0, None, **self.lk)
        q0 = p0.reshape(-1, 2)
        q1 = p1.reshape(-1, 2)
        h, w = cur.shape[:2]
        b = self.border_px
        ok = (st.ravel() == 1) & (q1[:, 0] >= b) & (q1[:, 0] < w - b) & (q1[:, 1] >= b) & (q1[:, 1] < h - b)
        if int(ok.sum()) < self.min_tracked:
            return FlowResult(n_tracked=int(ok.sum())), None

        # Outliers against the median shift, in pixels (cheap, before undistortion).
        dp = q1 - q0
        med_px = np.median(dp[ok], axis=0)
        ok &= np.max(np.abs(dp - med_px), axis=1) < self.outlier_px
        n = int(ok.sum())
        if n < self.min_tracked:
            return FlowResult(n_tracked=n), None

        # Displacement in normalised image coordinates. Over flat ground facing
        # the camera, a translation moves every point by the same v*dt/h here,
        # wherever it is in the image (an angle would shrink towards the edges).
        d = self.intr.to_normalised(q1[ok]) - self.intr.to_normalised(q0[ok])
        med = np.median(d, axis=0)
        # Robust spread (MAD), then the error of the median of n samples.
        mad = np.median(np.abs(d - med), axis=0) * 1.4826
        sigma = float(np.max(mad) * 1.2533 / np.sqrt(n))
        res = FlowResult(
            disp_x_rad=float(med[0]),
            disp_y_rad=float(med[1]),
            stddev_rad=sigma,
            n_tracked=n,
            valid=True,
        )
        return res, p1[ok].reshape(-1, 1, 2)
