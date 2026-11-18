#!/usr/bin/env python3
"""Rotate UM980 / UM982 RTK settings in blocks and log the solution of each.

Bench test for "Bench procedure: rejecting bad RTK fixes" in
plans/gnss-um98x-config.md. Runs on the host with the receiver's USB-C (COM3,
a CH340 at 115200, no pyserial: termios). It forwards RTCM3 from a raw TCP
stream (CRC-checked frames only, as the firmware does), sets one candidate
configuration per block, resets the RTK solution, and logs BESTNAVA at 5 Hz
and RTKSTATUSA at 1 Hz. The order of the configurations is shuffled in each
cycle, so every setting samples different satellite geometry and time of day.

Nothing is saved on the receiver (no SAVECONFIG). At the end it sends RESET,
which reloads the saved configuration, so the receiver is left as found.

Output: one text file per block in the run directory, every line from the
receiver prefixed with the host time, plus '#'-comment lines from this script
(block start with the configuration, command replies, RTCM up/down and the
block's RTCM statistics). Analyse with rtk_settings_analyze.py.

    nohup python3 rtk_settings_test.py --until 2026-11-20T07:30 > rtktest.out 2>&1 &
    python3 rtk_settings_test.py --resume runs/20261118_2130 --until ...   # after a crash
"""
import argparse, json, os, random, select, socket, sys, termios, time

# name: (RTK RELIABILITY, RTK CN0THD, RTK MMPL, elevation MASK)
CONFIGS = {
    "base":   ("3 1", 0, 0, 5),    # the bench unit's saved state
    "rel4":   ("4 1", 0, 0, 5),    # stricter ambiguity validation
    "adr3":   ("3 3", 0, 0, 5),    # ADR threshold at the manual's default
    "mmpl":   ("3 1", 0, 1, 5),    # stringent multipath threshold
    "cn0":    ("3 1", 1, 0, 5),    # stringent C/N0 requirement
    "mask15": ("3 1", 0, 0, 15),   # elevation mask 15 deg
    "strict": ("4 3", 1, 1, 15),   # all of them
}

def cfg_commands(name):
    rel, cn0, mmpl, mask = CONFIGS[name]
    return [f"CONFIG RTK RELIABILITY {rel}", f"CONFIG RTK CN0THD {cn0}",
            f"CONFIG RTK MMPL {mmpl}", f"MASK {mask}"]

def crc24q(data):
    crc = 0
    for b in data:
        crc ^= b << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
    return crc & 0xFFFFFF


class Rtcm:
    """Raw RTCM3 over TCP, split into CRC-checked frames, with reconnect."""
    def __init__(self, host, port, note):
        self.host, self.port, self.note = host, port, note
        self.sock = None; self.buf = b""; self.next_try = 0.0
        self.reset_stats()

    def reset_stats(self):
        self.frames = {}; self.bad = 0; self.bytes = 0; self.last_frame = None; self.max_gap = 0.0
        self.downs = 0

    def fileno(self):
        return self.sock.fileno() if self.sock else None

    def poll_connect(self):
        if self.sock or time.time() < self.next_try:
            return
        try:
            s = socket.create_connection((self.host, self.port), timeout=10)
            s.setblocking(False); self.sock = s; self.buf = b""
            self.note(f"RTCM up {self.host}:{self.port}")
        except OSError as e:
            self.next_try = time.time() + 10
            self.note(f"RTCM connect failed: {e}")

    def drop(self, why):
        if self.sock:
            self.sock.close()
        self.sock = None; self.downs += 1; self.next_try = time.time() + 5
        self.note(f"RTCM down: {why}")

    def read_frames(self):
        try:
            d = self.sock.recv(8192)
        except BlockingIOError:
            return []
        except OSError as e:
            self.drop(e); return []
        if not d:
            self.drop("closed by peer"); return []
        self.buf += d; out = []
        while True:
            i = self.buf.find(b"\xd3")
            if i < 0:
                self.buf = b""; break
            self.buf = self.buf[i:]
            if len(self.buf) < 3:
                break
            n = ((self.buf[1] & 0x03) << 8) | self.buf[2]
            if len(self.buf) < n + 6:
                break
            fr = self.buf[:n + 6]
            if crc24q(fr[:-3]) == int.from_bytes(fr[-3:], "big") and n >= 2:
                t = (fr[3] << 4) | (fr[4] >> 4)
                self.frames[t] = self.frames.get(t, 0) + 1; self.bytes += len(fr)
                now = time.time()
                if self.last_frame:
                    self.max_gap = max(self.max_gap, now - self.last_frame)
                self.last_frame = now
                out.append(fr); self.buf = self.buf[n + 6:]
            else:
                self.bad += 1; self.buf = self.buf[1:]
        return out


class Receiver:
    def __init__(self, dev):
        self.dev = dev; self.fd = None; self.buf = b""; self.open()

    def open(self):
        if self.fd is not None:
            try: os.close(self.fd)
            except OSError: pass
        self.fd = os.open(self.dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        at = termios.tcgetattr(self.fd)
        at[0] = at[1] = at[3] = 0
        at[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        at[4] = at[5] = termios.B115200
        termios.tcsetattr(self.fd, termios.TCSANOW, at)
        termios.tcflush(self.fd, termios.TCIOFLUSH)
        self.buf = b""

    def write(self, data):
        view = memoryview(data)
        end = time.time() + 2
        while view and time.time() < end:
            try:
                k = os.write(self.fd, view); view = view[k:]
            except BlockingIOError:
                select.select([], [self.fd], [], 0.1)

    def lines(self):
        try:
            self.buf += os.read(self.fd, 8192)
        except BlockingIOError:
            pass
        out = []
        while b"\n" in self.buf:
            ln, self.buf = self.buf.split(b"\n", 1)
            ln = ln.strip(b"\r").decode("ascii", errors="replace")
            if ln:
                out.append(ln)
        if len(self.buf) > 65536:
            self.buf = b""
        return out


class Runner:
    def __init__(self, a, run_dir):
        self.a = a; self.dir = run_dir; self.f = None; self.t_flush = 0.0
        self.rx = Receiver(a.dev)
        self.rtcm = Rtcm(a.rtcm_host, a.rtcm_port, self.note)
        self.last_nav = time.time(); self.pending = None; self.reply = None
        self.counts = {}

    def note(self, msg):
        line = f"{time.time():.3f} # {msg}"
        if self.f:
            self.f.write(line + "\n")
        print(time.strftime("%m-%d %H:%M:%S"), msg, flush=True)

    def pump(self, secs, stop=None):
        end = time.time() + secs
        while time.time() < end:
            self.rtcm.poll_connect()
            fds = [self.rx.fd] + ([self.rtcm.fileno()] if self.rtcm.sock else [])
            r, _, _ = select.select(fds, [], [], 0.2)
            if self.rtcm.sock and self.rtcm.fileno() in r:
                for fr in self.rtcm.read_frames():
                    self.rx.write(fr)
            if self.rx.fd in r:
                try:
                    lines = self.rx.lines()
                except OSError as e:
                    self.note(f"serial error {e}, reopening"); time.sleep(2)
                    self.reopen(); lines = []
                now = time.time()
                for ln in lines:
                    if self.f:
                        self.f.write(f"{now:.3f} {ln}\n")
                    if ln.startswith("#BESTNAVA"):
                        self.last_nav = now
                        try:
                            pt = ln.split(";", 1)[1].split(",")[1]
                            self.counts[pt] = self.counts.get(pt, 0) + 1
                        except IndexError:
                            pass
                    elif self.pending and ln.startswith("$command") and self.pending in ln:
                        self.reply = ln
            if self.f and time.time() - self.t_flush > 5:
                self.f.flush(); self.t_flush = time.time()
            if time.time() - self.last_nav > 30:
                self.note("no BESTNAVA for 30 s, reopening the port and sending the logs again")
                self.reopen()
            if stop and stop():
                return True
        return False

    def reopen(self):
        try:
            self.rx.open()
        except OSError as e:
            self.note(f"open failed: {e}")
        self.last_nav = time.time()
        self.start_logs(quiet=True)

    def cmd(self, c, wait=3.0):
        self.pending, self.reply = c, None
        self.rx.write((c + "\r\n").encode())
        self.pump(wait, lambda: self.reply is not None)
        ok = self.reply is not None and "OK" in self.reply
        self.note(f"cmd {c!r} -> {self.reply or 'no reply'}")
        self.pending = None
        return ok

    def start_logs(self, quiet=False):
        for c in ("UNLOG COM3", f"BESTNAVA COM3 {self.a.nav_period}", "RTKSTATUSA COM3 1"):
            self.rx.write((c + "\r\n").encode()); self.pump(0.5)

    def run_block(self, idx, cfg, secs):
        path = os.path.join(self.dir, f"block_{idx:03d}_{cfg}.log")
        self.f = open(path, "w")
        self.rtcm.reset_stats(); self.counts = {}
        self.note("BLOCK " + json.dumps({"index": idx, "config": cfg, "commands": cfg_commands(cfg),
                                         "planned_s": secs, "start_unix": time.time()}))
        ok = all([self.cmd(c) for c in cfg_commands(cfg)])
        self.cmd("CONFIG RTK RESET")
        self.note("CONFIG_APPLIED " + ("ok" if ok else "FAILED"))
        self.rx.write(b"CONFIG\r\n"); self.pump(1.5)
        self.rx.write(b"MASK\r\n"); self.pump(1.0)
        self.note("LOGGING")
        end = time.time() + secs
        while time.time() < end and not os.path.exists(os.path.join(self.dir, "STOP")):
            self.pump(min(60, end - time.time()))
        n = sum(self.counts.values()) or 1
        fixed = sum(v for k, v in self.counts.items() if k in ("NARROW_INT", "WIDE_INT", "L1_INT"))
        self.note("RTCM_STATS " + json.dumps({"frames": self.rtcm.frames, "bad": self.rtcm.bad,
                                              "bytes": self.rtcm.bytes, "max_gap_s": round(self.rtcm.max_gap, 1),
                                              "downs": self.rtcm.downs}))
        self.note(f"BLOCK_END {cfg}: {n} epochs, fixed {100.0 * fixed / n:.1f} %, types {self.counts}")
        self.f.close(); self.f = None


def schedule(seed, cycles):
    rnd = random.Random(seed); out = []
    for _ in range(cycles):
        names = list(CONFIGS); rnd.shuffle(names)
        if out and names[0] == out[-1]:
            names.append(names.pop(0))
        out += names
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dev", default="/dev/ttyUSB0")
    ap.add_argument("--rtcm-host", default="198.51.100.47")
    ap.add_argument("--rtcm-port", type=int, default=5016)
    ap.add_argument("--block-min", type=float, default=45.0)
    ap.add_argument("--nav-period", default="0.2", help="BESTNAVA period [s]")
    ap.add_argument("--until", help="local time to stop, e.g. 2026-11-20T07:30")
    ap.add_argument("--blocks", type=int, help="stop after this many blocks")
    ap.add_argument("--configs", help="comma-separated subset of " + ",".join(CONFIGS))
    ap.add_argument("--out", default="runs")
    ap.add_argument("--resume", help="run directory to continue")
    ap.add_argument("--no-reset", action="store_true", help="don't RESET the receiver at the end")
    a = ap.parse_args()

    if a.configs:
        for k in [k for k in CONFIGS if k not in a.configs.split(",")]:
            del CONFIGS[k]
    until = time.mktime(time.strptime(a.until, "%Y-%m-%dT%H:%M")) if a.until else None
    if a.resume:
        run_dir = a.resume
        meta = json.load(open(os.path.join(run_dir, "run.json")))
        done = len([f for f in os.listdir(run_dir) if f.startswith("block_")])
    else:
        run_dir = os.path.join(a.out, time.strftime("%Y%m%d_%H%M"))
        os.makedirs(run_dir, exist_ok=True)
        meta = {"seed": int(time.time()), "configs": CONFIGS, "block_min": a.block_min,
                "nav_period": a.nav_period, "dev": a.dev, "rtcm": f"{a.rtcm_host}:{a.rtcm_port}"}
        json.dump(meta, open(os.path.join(run_dir, "run.json"), "w"), indent=1)
        done = 0
    order = schedule(meta["seed"], 200)

    r = Runner(a, run_dir)
    r.f = open(os.path.join(run_dir, "setup.log" if not a.resume else f"setup_resume_{done:03d}.log"), "w")
    r.note(f"run dir {run_dir}, starting at block {done}")
    r.rx.write(b"UNLOG COM3\r\n"); r.pump(1.0)
    for q in ("VERSIONA", "CONFIG", "MASK", "UNILOGLIST"):
        r.rx.write((q + "\r\n").encode()); r.pump(1.5)
    r.start_logs()
    r.pump(5)
    r.f.close(); r.f = None

    idx = done
    try:
        while True:
            if a.blocks is not None and idx - done >= a.blocks:
                break
            if os.path.exists(os.path.join(run_dir, "STOP")):
                break
            secs = a.block_min * 60
            if until:
                left = until - time.time()
                if left < 10 * 60:
                    break
                secs = min(secs, left)
            r.run_block(idx, order[idx], secs)
            idx += 1
    finally:
        r.f = open(os.path.join(run_dir, "teardown.log"), "a")
        for c in cfg_commands("base") if "base" in CONFIGS else []:
            r.cmd(c)
        r.rx.write(b"UNLOG COM3\r\n"); r.pump(1.0)
        if not a.no_reset:
            r.cmd("RESET", wait=5.0)
        r.note(f"done after {idx} blocks")
        r.f.close()


if __name__ == "__main__":
    main()
