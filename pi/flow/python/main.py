#!/usr/bin/env python3
"""Optical flow on the Pi, sent to the Pico over the serial link.

    python3 main.py --port /dev/serial0              # send to the Pico
    python3 main.py --dry-run                        # print instead
    python3 main.py --dry-run --frames DIR           # recorded PNGs, on any PC

Protocol: common/flow_link.h. Design: plans/optical-flow-aiding.md.
"""

import argparse
import sys
import time
from pathlib import Path

import flow_link as fl
from interval_watch import IntervalWatch
from flow_lk import FlowEstimator, Intrinsics


def frames_from_dir(d: Path, fps: float):
    import cv2

    from camera import Frame

    t = 0
    for p in sorted(d.glob("*.png")):
        g = cv2.imread(str(p), cv2.IMREAD_GRAYSCALE)
        yield Frame(grey=g, sensor_ns=t, exposure_us=0)
        t += int(1e9 / fps)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/serial0", help="serial port to the Pico")
    ap.add_argument("--baud", type=int, default=fl.BAUD)
    ap.add_argument("--dry-run", action="store_true", help="print messages instead of sending")
    ap.add_argument("--frames", type=Path, help="read PNG frames from this folder instead of the camera")
    ap.add_argument("--size", default="320x200", help="processing size, WxH (the sensor runs 640x400)")
    ap.add_argument("--fps", type=float, default=100.0, help="free-running rate (ignored when triggered)")
    ap.add_argument("--corners", type=int, default=60, help="features tracked per frame")
    ap.add_argument("--exposure-us", type=int, default=1000)
    ap.add_argument("--gain", type=float, default=4.0)
    ap.add_argument("--camera", type=int, default=0)
    ap.add_argument("--triggered", action="store_true", help="camera is in external-trigger mode (set by ov9281_trigger)")
    ap.add_argument("--calib", type=Path, help="intrinsics JSON (fx, fy, cx, cy, dist, fisheye) at the processing size")
    args = ap.parse_args(argv)

    w, h = (int(v) for v in args.size.lower().split("x"))
    if args.calib:
        intr = Intrinsics.load(args.calib)
    else:
        intr = Intrinsics.nominal(w, h)
        print("warning: no --calib, using the nominal lens with no distortion model", file=sys.stderr)
    est = FlowEstimator(intr, max_corners=args.corners)

    ser = None
    if not args.dry_run:
        import serial  # python3-serial

        ser = serial.Serial(args.port, args.baud, timeout=0)

    if args.frames:
        source = frames_from_dir(args.frames, args.fps)
        cam = None
    else:
        from camera import Camera

        cam = Camera(w, h, args.fps, args.exposure_us, args.gain, args.camera)
        cam.start()

        def camera_frames():
            while True:
                yield cam.read()

        source = camera_frames()

    watch = IntervalWatch()
    prev = None
    seq = 0
    n_sent = 0
    t_report = time.monotonic()
    try:
        for f in source:
            seq += 1
            if prev is not None:
                dt_ns = f.sensor_ns - prev.sensor_ns
                flags = watch.classify(dt_ns)
                r = est.update(f.grey)
                if r.valid:
                    flags |= fl.F_VALID
                if args.triggered:
                    flags |= fl.F_TRIGGERED
                msg = fl.FlowMsg(
                    frame_seq=seq,
                    pi_time_us=f.sensor_ns // 1000,
                    pi_dt_us=dt_ns // 1000,
                    disp_x_rad=r.disp_x_rad,
                    disp_y_rad=r.disp_y_rad,
                    stddev_rad=r.stddev_rad,
                    n_tracked=r.n_tracked,
                    exposure_100us=(f.exposure_us + 50) // 100,
                    flags=flags,
                )
                if ser:
                    ser.write(fl.frame(fl.MSG_FLOW, msg.payload()))
                else:
                    print(msg)
                n_sent += 1
            else:
                est.update(f.grey)
            prev = f
            now = time.monotonic()
            if ser and now - t_report >= 5.0:
                print(f"{n_sent / (now - t_report):.1f} msg/s, seq {seq}", file=sys.stderr)
                n_sent, t_report = 0, now
    except KeyboardInterrupt:
        pass
    finally:
        if cam:
            cam.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
