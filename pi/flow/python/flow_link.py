"""Pi side of the optical-flow link to the Pico.

Mirrors common/flow_link.h byte for byte (the C++ program in pi/flow uses
the header itself); change both together.
test_flow_link.py checks the example frame (FLOW_LINK_EXAMPLE_HEX in the header).
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

F_VALID = 0x01
F_AFTER_GAP = 0x02
F_TRIGGERED = 0x04
F_DROPPED = 0x08

_FLOW_FMT = "<IIIfffHBB"
FLOW_LEN = struct.calcsize(_FLOW_FMT)
assert FLOW_LEN == 28


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


def frame(msg_type: int, payload: bytes) -> bytes:
    """Wrap a payload in sync, version, type, length and CRC."""
    if len(payload) > 64:
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


def example_hex_from_header(header: Path) -> str:
    """The FLOW_LINK_EXAMPLE_HEX string from common/flow_link.h."""
    lines = header.read_text(encoding="utf-8").splitlines()
    start = next(i for i, l in enumerate(lines) if l.startswith("#define FLOW_LINK_EXAMPLE_HEX"))
    block = []
    for line in lines[start:]:
        block.append(line)
        if not line.rstrip().endswith("\\"):
            break
    text = re.sub(r"/\*.*?\*/", "", "\n".join(block))
    return "".join(re.findall(r'"([0-9a-fA-F]*)"', text)).lower()
