#!/usr/bin/env python3
"""Synthetic test frames for replaying the flow program without a camera.

The view moves round a circle over a smooth random texture, so the last frame
leads straight back to the first and `flow --frames DIR --loop` replays it
without a jump. Each step is about --step px at the output size.

    python3 make_test_frames.py OUT_DIR [--n 300] [--size 320x200] [--step 3]
"""

import argparse
import math
from pathlib import Path

import cv2
import numpy as np

from test_flow_lk import texture


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("out", type=Path)
    ap.add_argument("--n", type=int, default=300, help="frames in one loop")
    ap.add_argument("--size", default="320x200", help="output size WxH")
    ap.add_argument("--step", type=float, default=3.0, help="shift per frame at the output size [px]")
    a = ap.parse_args()

    w, h = (int(v) for v in a.size.lower().split("x"))
    scale = 2  # draw at twice the output size, then shrink (like the 640x400 mode)
    r = a.n * a.step / (2 * math.pi) * scale
    cw, ch = w * scale, h * scale
    tw, th = int(cw + 2 * r + 64), int(ch + 2 * r + 64)
    tex = texture(tw - tw % 8, th - th % 8)
    a.out.mkdir(parents=True, exist_ok=True)
    for i in range(a.n):
        t = 2 * math.pi * i / a.n
        x0 = (tex.shape[1] - cw) / 2 + r * math.cos(t)
        y0 = (tex.shape[0] - ch) / 2 + r * math.sin(t)
        M = np.float32([[1, 0, -x0], [0, 1, -y0]])
        crop = cv2.warpAffine(tex, M, (cw, ch), flags=cv2.INTER_LINEAR)
        cv2.imwrite(str(a.out / f"{i:05d}.png"), cv2.resize(crop, (w, h), interpolation=cv2.INTER_AREA))
    print(f"{a.n} frames, {w}x{h}, step {a.step} px, in {a.out}")


if __name__ == "__main__":
    main()
