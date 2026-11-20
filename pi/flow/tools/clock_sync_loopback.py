"""Loopback check of `flow --udp`'s clock sync responder (common/flow_link.h,
FLOW_MSG_TIME_REQ / _RESP): plays the Pico on 127.0.0.1:5800, waits for the
first flow frame, sends 20 requests and prints the round trip and the offset
to its own CLOCK_MONOTONIC. On one host the offset should be near 0; what is
left is mostly this script's send time, taken in user space.

    python3 pi/flow/tools/clock_sync_loopback.py pi/flow/python &
    pi/flow/build/flow --udp 127.0.0.1 --v4l2      (from the repo root)

ROCK 4SE, 2026-11-20, 3 runs with the camera at 100 Hz: shortest round trip
305-350 us, offset at the shortest round trip -58 .. -82 us."""
import socket
import struct
import sys
import time

sys.path.insert(0, sys.argv[1])
import flow_link as fl  # noqa: E402

SO_TIMESTAMPNS = 35  # Linux; not in the socket module
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, SO_TIMESTAMPNS, 1)
s.bind(("127.0.0.1", 5800))


def recv_ts():
    d, anc, _, _ = s.recvmsg(256, 64)
    rt = time.clock_gettime_ns(time.CLOCK_REALTIME)
    mono = time.clock_gettime_ns(time.CLOCK_MONOTONIC)
    for lvl, typ, b in anc:
        if lvl == socket.SOL_SOCKET and typ == SO_TIMESTAMPNS:
            sec, nsec = struct.unpack("qq", b[:16])
            return d, (mono - (rt - (sec * 10**9 + nsec))) // 1000
    return d, mono // 1000
s.settimeout(20)
data, peer = s.recvfrom(256)
print("first datagram from", peer, "type", data[3])
s.settimeout(1)
offs, rtts = [], []
for seq in range(1, 21):
    t1 = time.clock_gettime_ns(time.CLOCK_MONOTONIC) // 1000
    s.sendto(fl.frame(fl.MSG_TIME_REQ, fl.time_req_payload(seq)), peer)
    while True:
        d, t2 = recv_ts()
        msgs, _ = fl.parse(d)
        if msgs and msgs[0][0] == fl.MSG_TIME_RESP:
            rseq, turn, urx = fl.time_resp_from_payload(msgs[0][1])
            if rseq == seq:
                break
    rtt = t2 - t1 - turn
    offs.append(t1 + rtt / 2 - urx)
    rtts.append(rtt)
    time.sleep(0.05)
print(f"rtt min {min(rtts)} us, median {sorted(rtts)[10]} us; offset at min rtt {offs[rtts.index(min(rtts))]:.0f} us, "
      f"range {min(offs):.0f} .. {max(offs):.0f} us")
