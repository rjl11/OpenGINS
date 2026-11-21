"""Pi side of the optical-flow link to the Pico.

Mirrors common/flow_link.h byte for byte (the C++ program in pi/flow uses
the header itself); change both together.
test_flow_link.py checks the example frames (FLOW_LINK_EXAMPLE_HEX, the
two FLOW_LINK_EXAMPLE_TIME_*_HEX, FLOW_LINK_EXAMPLE_CAM_VEL_HEX,
FLOW_LINK_EXAMPLE_UNIT_CFG_HEX and FLOW_LINK_EXAMPLE_IMU_BATCH_HEX in the
header).
"""

import re
import struct
from dataclasses import dataclass
from pathlib import Path

SYNC0 = 0xF7
SYNC1 = 0x10
VERSION = 1
BAUD = 115200
GAP_PERIOD = 256

MSG_FLOW = 1
MSG_TIME_REQ = 2  # clock sync request, Pico -> unit (UDP only)
MSG_TIME_RESP = 3  # clock sync reply, unit -> Pico (UDP only)
MSG_CAM_VEL = 4  # camera velocity, de-rotated on the unit, unit -> Pico
MSG_UNIT_CFG = 5  # the unit's settings and the heartbeat, Pico -> unit
MSG_IMU_BATCH = 6  # TM171 samples, the backup IMU, unit -> Pico
MAX_PAYLOAD = 128
IMU_BATCH_MAX = 8
IMU_GYR_LSB = 1.0 / 4096.0
IMU_ACC_LSB = 1.0 / 1024.0
IMU_F_GAP = 0x01
IMU_F_CLAMPED = 0x02
CFG_F_FLOW_ON = 0x01

F_VALID = 0x01
F_AFTER_GAP = 0x02
F_TRIGGERED = 0x04
F_DROPPED = 0x08
F_PHASE = 0x10  # phase correlation (C++ only); n_tracked = response x 1000
F_RANGE = 0x20  # MSG_CAM_VEL: height from the rangefinder

_FLOW_FMT = "<IIIfffHBB"
FLOW_LEN = struct.calcsize(_FLOW_FMT)
assert FLOW_LEN == 28

_CAM_VEL_FMT = "<IIIfffffHBB"
CAM_VEL_LEN = struct.calcsize(_CAM_VEL_FMT)
assert CAM_VEL_LEN == 36

_UNIT_CFG_FMT = "<IfffffB3x"  # seq, height, cam yaw, TM171 roll/pitch/yaw, flags
UNIT_CFG_LEN = struct.calcsize(_UNIT_CFG_FMT)
assert UNIT_CFG_LEN == 28


def unit_cfg_payload(seq: int, height_m: float, cam_yaw_rad: float, imu_rpy_rad, flags: int) -> bytes:
    return struct.pack(_UNIT_CFG_FMT, seq & 0xFFFFFFFF, height_m, cam_yaw_rad, *imu_rpy_rad, flags & 0xFF)


def imu_batch_payload(t0_us: int, seq: int, samples, flags: int = 0) -> bytes:
    """samples: [(dt_us, (gx, gy, gz), (ax, ay, az))], FRD, rad/s and m/s^2."""
    if not 1 <= len(samples) <= IMU_BATCH_MAX:
        raise ValueError("1 .. 8 samples")
    body = b""

    def q(v, lsb):
        nonlocal flags
        x = v / lsb
        if not x > -32767.0:
            flags |= IMU_F_CLAMPED
            return -32767
        if x > 32767.0:
            flags |= IMU_F_CLAMPED
            return 32767
        return int(x - 0.5) if x < 0 else int(x + 0.5)

    for dt, g, a in samples:
        body += struct.pack("<H3h3h", dt, *(q(v, IMU_GYR_LSB) for v in g), *(q(v, IMU_ACC_LSB) for v in a))
    return struct.pack("<IHBB", t0_us & 0xFFFFFFFF, seq & 0xFFFF, len(samples), flags & 0xFF) + body


def imu_batch_from_payload(p: bytes):
    """(t0_us, seq, flags, [(dt_us, (gx, gy, gz), (ax, ay, az))])"""
    t0, seq, n, flags = struct.unpack_from("<IHBB", p)
    if not 1 <= n <= IMU_BATCH_MAX or len(p) != 8 + 14 * n:
        raise ValueError("bad IMU batch length")
    out = []
    for i in range(n):
        v = struct.unpack_from("<H3h3h", p, 8 + 14 * i)
        out.append((v[0], tuple(x * IMU_GYR_LSB for x in v[1:4]), tuple(x * IMU_ACC_LSB for x in v[4:7])))
    return t0, seq, flags, out


def unit_cfg_from_payload(p: bytes):
    """(seq, height_m, cam_yaw_rad, (roll, pitch, yaw), flags)"""
    if len(p) != UNIT_CFG_LEN:
        raise ValueError(f"UNIT_CFG payload is {len(p)} bytes, expected {UNIT_CFG_LEN}")
    seq, h, y, r, pi, ya, fl = struct.unpack(_UNIT_CFG_FMT, p)
    return seq, h, y, (r, pi, ya), fl


_TIME_REQ_FMT = "<I"  # seq
_TIME_RESP_FMT = "<IIQ"  # seq, turnaround_us, unit_rx_us
TIME_REQ_LEN = struct.calcsize(_TIME_REQ_FMT)
TIME_RESP_LEN = struct.calcsize(_TIME_RESP_FMT)
assert TIME_REQ_LEN == 4 and TIME_RESP_LEN == 16


def time_req_payload(seq: int) -> bytes:
    return struct.pack(_TIME_REQ_FMT, seq & 0xFFFFFFFF)


def time_req_from_payload(p: bytes) -> int:
    if len(p) != TIME_REQ_LEN:
        raise ValueError(f"TIME_REQ payload is {len(p)} bytes, expected {TIME_REQ_LEN}")
    return struct.unpack(_TIME_REQ_FMT, p)[0]


def time_resp_payload(seq: int, turnaround_us: int, unit_rx_us: int) -> bytes:
    return struct.pack(_TIME_RESP_FMT, seq & 0xFFFFFFFF, min(max(turnaround_us, 0), 0xFFFFFFFF),
                       unit_rx_us & 0xFFFFFFFFFFFFFFFF)


def time_resp_from_payload(p: bytes):
    """(seq, turnaround_us, unit_rx_us)"""
    if len(p) != TIME_RESP_LEN:
        raise ValueError(f"TIME_RESP payload is {len(p)} bytes, expected {TIME_RESP_LEN}")
    return struct.unpack(_TIME_RESP_FMT, p)


def crc16(data: bytes) -> int:
    """CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no xor out."""
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


@dataclass
class FlowMsg:
    frame_seq: int
    pi_time_us: int
    pi_dt_us: int
    disp_x_rad: float
    disp_y_rad: float
    stddev_rad: float
    n_tracked: int
    exposure_100us: int
    flags: int

    def payload(self) -> bytes:
        return struct.pack(
            _FLOW_FMT,
            self.frame_seq & 0xFFFFFFFF,
            self.pi_time_us & 0xFFFFFFFF,
            min(max(self.pi_dt_us, 0), 0xFFFFFFFF),
            self.disp_x_rad,
            self.disp_y_rad,
            self.stddev_rad,
            min(max(self.n_tracked, 0), 0xFFFF),
            min(max(self.exposure_100us, 0), 0xFF),
            self.flags & 0xFF,
        )

    @classmethod
    def from_payload(cls, p: bytes) -> "FlowMsg":
        if len(p) != FLOW_LEN:
            raise ValueError(f"FLOW payload is {len(p)} bytes, expected {FLOW_LEN}")
        return cls(*struct.unpack(_FLOW_FMT, p))


@dataclass
class CamVelMsg:
    """MSG_CAM_VEL: the camera's velocity in camera axes, rotation removed on
    the unit and scaled by height_m (field meanings in common/flow_link.h)."""
    frame_seq: int
    pi_time_us: int
    pi_dt_us: int
    vel_x_mps: float
    vel_y_mps: float
    stddev_mps: float
    height_m: float
    height_sd_m: float
    n_tracked: int
    exposure_100us: int
    flags: int

    def payload(self) -> bytes:
        return struct.pack(
            _CAM_VEL_FMT,
            self.frame_seq & 0xFFFFFFFF,
            self.pi_time_us & 0xFFFFFFFF,
            min(max(self.pi_dt_us, 0), 0xFFFFFFFF),
            self.vel_x_mps,
            self.vel_y_mps,
            self.stddev_mps,
            self.height_m,
            self.height_sd_m,
            min(max(self.n_tracked, 0), 0xFFFF),
            min(max(self.exposure_100us, 0), 0xFF),
            self.flags & 0xFF,
        )

    @classmethod
    def from_payload(cls, p: bytes) -> "CamVelMsg":
        if len(p) != CAM_VEL_LEN:
            raise ValueError(f"CAM_VEL payload is {len(p)} bytes, expected {CAM_VEL_LEN}")
        return cls(*struct.unpack(_CAM_VEL_FMT, p))


def frame(msg_type: int, payload: bytes) -> bytes:
    """Wrap a payload in sync, version, type, length and CRC."""
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload too long")
    body = bytes((VERSION, msg_type, len(payload))) + payload
    return bytes((SYNC0, SYNC1)) + body + struct.pack("<H", crc16(body))


def parse(buf: bytes):
    """Find complete frames in buf. Returns ([(type, payload)], bytes left over).

    Resynchronises on the next sync pair after a bad CRC or version, as the
    Pico's parser does. Used by the tests and for checking a capture."""
    out = []
    i = 0
    while True:
        j = buf.find(bytes((SYNC0, SYNC1)), i)
        if j < 0:
            return out, buf[-1:] if buf[-1:] == bytes((SYNC0,)) else b""
        if len(buf) - j < 5:
            return out, buf[j:]
        n = buf[j + 4]
        end = j + 5 + n + 2
        if len(buf) < end:
            return out, buf[j:]
        body = buf[j + 2 : j + 5 + n]
        (crc,) = struct.unpack_from("<H", buf, j + 5 + n)
        if body[0] == VERSION and crc16(body) == crc:
            out.append((body[1], bytes(body[3:])))
            i = end
        else:
            i = j + 1


def example_hex_from_header(header: Path, name: str = "FLOW_LINK_EXAMPLE_HEX") -> str:
    """An example frame string (FLOW_LINK_EXAMPLE_HEX by default) from common/flow_link.h."""
    lines = header.read_text(encoding="utf-8").splitlines()
    start = next(i for i, l in enumerate(lines) if l.split()[:2] == ["#define", name])
    block = []
    for line in lines[start:]:
        block.append(line)
        if not line.rstrip().endswith("\\"):
            break
    text = re.sub(r"/\*.*?\*/", "", "\n".join(block))
    return "".join(re.findall(r'"([0-9a-fA-F]*)"', text)).lower()
