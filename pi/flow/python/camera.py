"""OV9281 capture through Picamera2 (Raspberry Pi OS's own ov9281 driver).

Gives grey 8-bit frames with the sensor timestamp and exposure. Only imported
on the Pi; flow_lk.py and flow_link.py don't need it.

Not yet checked on the camera: the stream format the ISP hands out for this
mono sensor, whether 300 fps holds at 640x400, and behaviour in trigger mode
(frames only arrive when the Pico pulses TRIG, so libcamera's timeout must be
raised, see README).
"""

from dataclasses import dataclass

import numpy as np


@dataclass
class Frame:
    grey: np.ndarray  # (h, w) uint8
    sensor_ns: int  # SensorTimestamp: start of exposure, Pi monotonic clock [ns]
    exposure_us: int


class Camera:
    def __init__(self, width=320, height=200, fps=100.0, exposure_us=1000, gain=4.0, camera=0):
        from picamera2 import Picamera2  # Pi only

        self.w, self.h = width, height
        self.cam = Picamera2(camera)
        frame_us = int(1e6 / fps)
        cfg = self.cam.create_video_configuration(
            main={"size": (width, height), "format": "YUV420"},
            # The sensor's 640x400 mode is the full field of view and its
            # fastest (309 fps); the ISP scales it down to the main stream.
            raw={"size": (640, 400)},
            buffer_count=6,
            controls={
                "FrameDurationLimits": (frame_us, frame_us),
                "AeEnable": False,
                "ExposureTime": int(exposure_us),
                "AnalogueGain": float(gain),
            },
        )
        self.cam.configure(cfg)

    def start(self):
        self.cam.start()

    def stop(self):
        self.cam.stop()

    def read(self) -> Frame:
        req = self.cam.capture_request()
        try:
            yuv = req.make_array("main")
            md = req.get_metadata()
        finally:
            req.release()
        grey = np.ascontiguousarray(yuv[: self.h, : self.w])  # Y plane
        return Frame(
            grey=grey,
            sensor_ns=int(md.get("SensorTimestamp", 0)),
            exposure_us=int(md.get("ExposureTime", 0)),
        )
