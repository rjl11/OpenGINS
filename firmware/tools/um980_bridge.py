#!/usr/bin/env python3
"""Bench bridge: a UM980/UM982 on another host, as the Pico's GNSS on UART1.

Runs on the host with the Pico and a Debug Probe (dietpi). The probe's UART
is wired to the Pico's UART1 (probe TX -> GP5, probe RX <- GP4, GND), and
the receiver's USB-C (COM3) sits on another host that runs
um980_tcp_relay.py (miniPi). So the real um980 driver can detect, check and
provision a receiver that isn't wired to the Pico. Used on 2026-11-14 to
find two driver bugs on R4.10Build25621 (plans/gnss-um98x-config.md).

The driver expects the receiver on COM1 at 460800, but here it is reached
through COM3, and the relay host's CH340 is only clean at 115200. So:

  Pico -> receiver   swap COM1<->COM3 in command lines; "CONFIG COM3 <b>"
                     (after the swap) is sent as "CONFIG COM3 115200" and
                     <b> remembered as the baud the Pico believes COM1 has.
                     Non-printable lines (Pico scanning the wrong baud) are
                     dropped.
  receiver -> Pico   binary frames (AA 44 B5) pass verbatim; ASCII lines get
                     the same swap, "COM1 115200" shows the remembered baud,
                     and the NMEA *XX checksum is recomputed.
  probe UART baud    follows the Pico: 115200 after FRESET, <b> after
                     "CONFIG COM1 <b>", switched once the reply has gone out
                     (or after 1 s, as the driver's OP_CMD_OPT does).

The remembered baud is saved on SAVECONFIG to ~/um980_bridge_state.json.
Provisioning through the bridge saves to the receiver: logs on COM3, COM3
at 115200. Send "UNLOG COM3" + "SAVECONFIG" (or FRESET) to undo that.

There is no PPS wire, so the Pico times GNSS epochs by arrival, which
includes the network jitter: use this for driver tests, not for timing.

GNSS outage test: SIGUSR1 toggles a mute that drops everything from the
receiver to the Pico (whole binary frames and lines), while the bridge keeps
reading the relay, so no stale data piles up for the end of the outage.
Commands from the Pico still go through.

    pkill -USR1 -f um980_bridge.py      # mute, and again to unmute

RTCM corrections: the bridge also reads the RTK base's raw RTCM3 stream
(--rtcm, default rtkbase 198.51.100.47:5016 over Tailscale) and sends each
CRC-checked frame whole to the receiver through the relay, between the
Pico's command lines, so the receiver can reach an RTK fix. A lost base
connection is retried every 5 s; --rtcm off turns it off. The mute above
doesn't stop the corrections.

    python3 um980_bridge.py --host 192.0.2.50
"""
import argparse
import glob
import json
import os
import re
import select
import signal
import socket
import sys
import termios
import time

BAUDS = {9600: termios.B9600, 19200: termios.B19200, 38400: termios.B38400,
         57600: termios.B57600, 115200: termios.B115200, 230400: termios.B230400,
         460800: termios.B460800, 921600: termios.B921600}
RX_BAUD = 115200                      # receiver COM3 always stays here
STATE = os.path.expanduser("~/um980_bridge_state.json")
HEADER_LEN, CRC_LEN, OFS_MSG_LEN, FRAME_MAX = 24, 4, 6, 4096

CRC_TABLE = []
for i in range(256):
    c = i
    for _ in range(8):
        c = (c >> 1) ^ 0xEDB88320 if c & 1 else c >> 1
    CRC_TABLE.append(c)


def crc32(b):
    c = 0
    for x in b:
        c = CRC_TABLE[(c ^ x) & 0xFF] ^ (c >> 8)
    return c


CRC24Q_TABLE = []
for i in range(256):
    c = i << 16
    for _ in range(8):
        c = ((c << 1) ^ 0x1864CFB) if c & 0x800000 else c << 1
    CRC24Q_TABLE.append(c & 0xFFFFFF)


def crc24q(b):
    c = 0
    for x in b:
        c = ((c << 8) & 0xFFFFFF) ^ CRC24Q_TABLE[(c >> 16) ^ x]
    return c


def ts():
    return time.strftime("%H:%M:%S") + f".{int(time.time() * 1000) % 1000:03d}"


def log(*a):
    print(ts(), *a, flush=True)


def swap_ports(s):
    return re.sub(r"COM([13])", lambda m: "COM3" if m.group(1) == "1" else "COM1", s)


def fix_checksum(s):
    m = re.match(r"^\$([^*]*)\*([0-9A-Fa-f]{2})$", s)
    if not m:
        return s
    cs = 0
    for ch in m.group(1).encode("latin-1"):
        cs ^= ch
    return f"${m.group(1)}*{cs:02X}"


class Probe:
    def __init__(self, path, baud):
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
        self.baud = None
        self.set_baud(baud)
        termios.tcflush(self.fd, termios.TCIOFLUSH)

    def set_baud(self, baud):
        a = termios.tcgetattr(self.fd)
        a[0] = 0
        a[1] = 0
        a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        a[3] = 0
        a[4] = a[5] = BAUDS[baud]
        a[6][termios.VMIN] = 0
        a[6][termios.VTIME] = 0
        termios.tcdrain(self.fd)
        termios.tcsetattr(self.fd, termios.TCSANOW, a)
        if baud != self.baud:
            log(f"probe UART -> {baud} baud")
        self.baud = baud

    def write(self, b):
        os.write(self.fd, b)


class Bridge:
    def __init__(self, probe, sock, pico_baud):
        self.probe = probe
        self.sock = sock
        self.pico_baud = pico_baud     # what the Pico believes COM1 runs at
        self.pending = None            # (baud, reply keyword, deadline)
        self.up = bytearray()          # Pico -> receiver line buffer
        self.dn = bytearray()          # receiver -> Pico buffer
        self.dn_t = 0.0
        self.frames = self.crc_bad = self.dropped = 0
        self.muted = False
        self.muted_bytes = 0

    # ---- Pico -> receiver ----
    def from_pico(self, data):
        for x in data:
            self.up.append(x)
            if x == 0x0A or len(self.up) > 256:
                self.pico_line(bytes(self.up))
                self.up.clear()

    def pico_line(self, raw):
        text = raw.rstrip(b"\r\n")
        # Garbage from the Pico scanning other baud rates has no newline, so it
        # sits in front of the next clean command: keep the printable tail.
        bad = [i for i, c in enumerate(text) if not 0x20 <= c < 0x7F]
        if bad:
            self.dropped += 1
            text = text[bad[-1] + 1:]
        if not text or not raw.endswith(b"\n"):
            self.dropped += 1
            return
        s = text.decode("ascii")
        out = swap_ports(s)
        up = out.upper()
        m = re.match(r"^CONFIG COM3 (\d+)$", up)
        if m:
            b = int(m.group(1))
            out = f"CONFIG COM3 {RX_BAUD}"
            self.pico_baud = b
            if b in BAUDS:
                self.pending = (b, "CONFIG", time.time() + 1.0)
        elif up == "FRESET":
            self.pico_baud = 115200
            self.pending = (115200, "FRESET", time.time() + 1.0)
        elif up == "SAVECONFIG":
            with open(STATE, "w") as f:
                json.dump({"pico_com1_baud": self.pico_baud}, f)
        log(f"P> {s}" + (f"   => {out}" if out != s else ""))
        self.sock.sendall(out.encode("ascii") + b"\r\n")

    # ---- receiver -> Pico ----
    def from_rx(self, data):
        self.dn += data
        self.dn_t = time.time()
        self.parse_dn(final=False)

    def parse_dn(self, final):
        d = self.dn
        while d:
            if d[0] == 0xAA:
                if len(d) >= 2 and d[1] != 0x44 or len(d) >= 3 and d[2] != 0xB5:
                    self.to_pico(bytes(d[:1]))
                    del d[:1]
                    continue
                if len(d) < OFS_MSG_LEN + 2:
                    break
                need = HEADER_LEN + int.from_bytes(d[OFS_MSG_LEN:OFS_MSG_LEN + 2], "little") + CRC_LEN
                if need > FRAME_MAX:
                    self.to_pico(bytes(d[:1]))
                    del d[:1]
                    continue
                if len(d) < need:
                    break
                fr = bytes(d[:need])
                del d[:need]
                self.frames += 1
                if crc32(fr[:-CRC_LEN]) != int.from_bytes(fr[-CRC_LEN:], "little"):
                    self.crc_bad += 1
                    log(f"CRC error in binary frame id {int.from_bytes(fr[4:6], 'little')}")
                if int.from_bytes(fr[4:6], "little") == 37:  # VERSION
                    txt = re.findall(rb"[\x20-\x7e]{3,}", fr[HEADER_LEN:-CRC_LEN])
                    log("VERSION: " + " | ".join(t.decode() for t in txt))
                self.to_pico(fr)
                continue
            nl = d.find(b"\n")
            aa = d.find(b"\xaa")
            if aa != -1 and (nl == -1 or aa < nl):
                self.to_pico(bytes(d[:aa]))   # no line end before a frame
                del d[:aa]
                continue
            if nl == -1:
                if final or len(d) > 512:
                    self.to_pico(bytes(d))
                    d.clear()
                break
            line = bytes(d[:nl + 1])
            del d[:nl + 1]
            self.rx_line(line)

    def rx_line(self, raw):
        body = raw.rstrip(b"\r\n")
        try:
            s = body.decode("ascii")
        except UnicodeDecodeError:
            self.to_pico(raw)
            return
        out = swap_ports(s)
        out = re.sub(rf"COM1 {RX_BAUD}\b", f"COM1 {self.pico_baud}", out)
        out = fix_checksum(out) if out != s else out
        if out != s or s.startswith("$command") or s.startswith("$CONFIG") or s.startswith("<"):
            log(f"R< {s}" + (f"   => {out}" if out != s else ""))
        self.to_pico(out.encode("ascii") + raw[len(body):])
        if self.pending and s.startswith("$command") and self.pending[1] in s.upper():
            self.switch()

    def to_pico(self, b):
        if self.muted:
            self.muted_bytes += len(b)
        elif b:
            self.probe.write(b)

    def toggle_mute(self, *_):
        self.muted = not self.muted
        log(("MUTED: receiver -> Pico dropped" if self.muted else
             f"UNMUTED ({self.muted_bytes} B dropped while muted)"))
        self.muted_bytes = 0

    def switch(self):
        b = self.pending[0]
        self.pending = None
        self.probe.set_baud(b)

    def tick(self):
        now = time.time()
        if self.pending and now >= self.pending[2]:
            log("no reply in 1 s, switching anyway")
            self.switch()
        if self.dn and now - self.dn_t > 0.5:
            self.parse_dn(final=True)


class RtcmIn:
    """Raw RTCM3 over TCP from the base, cut into CRC-checked frames.

    Connects without blocking, so the bridge loop never stalls on the base;
    after a drop or a failed connect it waits RETRY s and tries again."""
    RETRY = 5.0

    def __init__(self, host, port, out):
        self.addr = (host, port)
        self.out = out                 # called with each whole, valid frame
        self.sock = None
        self.connecting = False
        self.next_try = 0.0
        self.buf = bytearray()
        self.frames = self.crc_bad = self.bytes = self.connects = 0
        self.types = {}

    def tick(self):
        if self.sock is None and time.time() >= self.next_try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.setblocking(False)
            s.connect_ex(self.addr)
            self.sock, self.connecting = s, True

    def rlist(self):
        return [self.sock] if self.sock is not None and not self.connecting else []

    def wlist(self):
        return [self.sock] if self.connecting else []

    def drop(self, why):
        log(f"RTCM {self.addr[0]}:{self.addr[1]} {why}, retry in {self.RETRY:.0f} s")
        self.sock.close()
        self.sock, self.connecting = None, False
        self.buf.clear()
        self.next_try = time.time() + self.RETRY

    def writable(self):
        err = self.sock.getsockopt(socket.SOL_SOCKET, socket.SO_ERROR)
        if err:
            self.drop(f"connect failed ({os.strerror(err)})")
            return
        self.connecting = False
        self.connects += 1
        log(f"RTCM connected to {self.addr[0]}:{self.addr[1]}")

    def readable(self):
        try:
            d = self.sock.recv(4096)
        except OSError as e:
            self.drop(f"error ({e})")
            return
        if not d:
            self.drop("closed")
            return
        self.bytes += len(d)
        self.buf += d
        b = self.buf
        while True:
            i = b.find(b"\xd3")
            if i < 0:
                b.clear()
                return
            del b[:i]
            if len(b) < 3:
                return
            n = ((b[1] & 0x03) << 8) | b[2]
            if len(b) < 3 + n + 3:
                return
            fr = bytes(b[:3 + n + 3])
            if b[1] & 0xFC or crc24q(fr[:-3]) != int.from_bytes(fr[-3:], "big"):
                self.crc_bad += 1
                del b[:1]                # not a frame start: resync
                continue
            del b[:len(fr)]
            self.frames += 1
            if n >= 2:
                t = (fr[3] << 4) | (fr[4] >> 4)
                self.types[t] = self.types.get(t, 0) + 1
            self.out(fr)

    def stat(self):
        state = "up" if self.sock is not None and not self.connecting else "down"
        top = " ".join(f"{t}:{c}" for t, c in sorted(self.types.items()))
        return f"RTCM {state}, {self.frames} frames ({self.crc_bad} bad), {self.bytes} B; {top}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="192.0.2.50")
    ap.add_argument("--port", type=int, default=5017)
    ap.add_argument("--probe", default=None, help="probe UART (default: Debug Probe by-id)")
    ap.add_argument("--rtcm", default="198.51.100.47:5016",
                    help="RTK base raw RTCM3 stream HOST:PORT, or 'off' (default rtkbase)")
    args = ap.parse_args()

    path = args.probe or (sorted(glob.glob("/dev/serial/by-id/usb-Raspberry_Pi_Debug_Probe*")) or [None])[0]
    if path is None:
        sys.exit("no Debug Probe UART found")
    pico_baud = 115200
    try:
        with open(STATE) as f:
            pico_baud = int(json.load(f)["pico_com1_baud"])
    except (OSError, ValueError, KeyError):
        pass
    log(f"probe {path}, Pico COM1 baud starts at {pico_baud}")
    probe = Probe(path, pico_baud)
    sock = socket.create_connection((args.host, args.port), timeout=10)
    sock.settimeout(None)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    log(f"connected to {args.host}:{args.port}")

    br = Bridge(probe, sock, pico_baud)
    signal.signal(signal.SIGUSR1, br.toggle_mute)
    rtcm = None
    if args.rtcm != "off":
        h, _, p = args.rtcm.rpartition(":")
        rtcm = RtcmIn(h, int(p), sock.sendall)
    last_stat = time.time()
    while True:
        if rtcm:
            rtcm.tick()
        try:
            r, w, _ = select.select([probe.fd, sock] + (rtcm.rlist() if rtcm else []),
                                    rtcm.wlist() if rtcm else [], [], 0.05)
        except InterruptedError:
            continue
        if rtcm and rtcm.sock is not None:
            if rtcm.sock in w:
                rtcm.writable()
            elif rtcm.sock in r:
                rtcm.readable()
        if probe.fd in r:
            br.from_pico(os.read(probe.fd, 4096))
        if sock in r:
            d = sock.recv(4096)
            if not d:
                sys.exit("relay closed the connection")
            br.from_rx(d)
        br.tick()
        if time.time() - last_stat >= 10:
            last_stat = time.time()
            log(f"stat: {br.frames} frames, {br.crc_bad} CRC bad, {br.dropped} Pico lines dropped, "
                f"probe {probe.baud}, Pico COM1 {br.pico_baud}")
            if rtcm:
                log("stat: " + rtcm.stat())


if __name__ == "__main__":
    main()
