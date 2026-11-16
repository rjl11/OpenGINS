#!/usr/bin/env python3
"""Overnight static log of a TM210 (or TM171) over USB, for the Allan variance.

Plan: plans/imu-tm210.md, "Calibration steps" 1, and TODO.md.

What it does:
  1. Opens the IMU's USB virtual COM port, reads its Setting object (retrying,
     because the port may not answer for a few seconds after it enumerates)
     and, unless it already matches, writes the logging target to RAM only:
     raw and Status output, 800 Hz, USB output on. Every other field is
     copied. Nothing is saved to flash, so a power cycle restores the IMU's
     own settings.
  2. Logs every byte the IMU sends to <out>/raw.bin, unchanged, from the
     start. The IMU sends a Status packet after every raw packet; they land
     in raw.bin too, and one per second is written with the host time to
     <out>/status.csv. After the setup nothing is sent to the IMU: Status
     requests are not used, because a few hundred of them make the TM210
     stop answering Setting requests (plans/imu-tm210-config.md).
  3. Marks the record start in <out>/meta.json once the warm-up time has
     passed AND Status QoS has reached --qos (the user guide's heat-up). If
     QoS never gets there, it starts anyway --qos-timeout minutes after the
     warm-up, and says so.
  4. Stops --hours after the record start (or on Ctrl-C / SIGTERM).

The port is read in its own thread, because the host's tty holds only about
4 KB (about 70 ms of data) and an fsync to an SD card can take a second:
with reads and disk writes in one loop, every slow fsync lost packets.

Host read timing goes to <out>/timing.csv (one line per second: reads, bytes,
longest time between reads, largest read, largest queue lag of the main
loop, slowest fsync, packets missing), and every gap in the IMU timestamps
gets an events.log line with the timing of the read it was found in. A gap
after a normal read interval means the packets were lost before they reached
the host's tty; a gap after a read interval of 70 ms or more means the
reader thread was held up (a 4095-byte read is the full tty buffer).

raw.bin is flushed and fsynced every 10 s, so a crash or power loss costs at
most 10 s. If the port disappears (USB re-enumeration) it is reopened every
2 s, the settings are checked again, and the event is logged.

Convert afterwards with tm210_to_imu_csv.py <out>, then run
external/INSLIB/tools/allan_variance.py on the CSV.

Two IMUs: run one instance per port, each with its own --out. On Linux use
the /dev/serial/by-id/ names; the node ID in meta.json tells them apart.

Needs Python 3.8+. Uses pyserial when installed, otherwise termios (POSIX).

Example (on a Linux host, in the background):
  nohup python3 firmware/tools/tm210_allan_log.py --port /dev/ttyACM0 \\
      --out ~/allan/tm210 > ~/allan/tm210.log 2>&1 &
"""

import argparse
import json
import os
import queue
import shutil
import signal
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import tm210_ep as ep  # noqa: E402

STEP_US = 1250                      # 800 Hz
BYTES_PER_HOUR = (49 + 21) * 800 * 3600  # raw + Status packets, about 202 MB/h
STATUS_CSV_S = 1.0                  # status.csv: one line per second
FSYNC_S = 10.0
REOPEN_S = 2.0


# ---- Serial port -----------------------------------------------------------

class Port:
    """Minimal raw serial port: pyserial if available, else termios."""

    def __init__(self, path):
        self.path = path
        self.ser = None
        self.fd = None
        try:
            import serial  # type: ignore
        except ImportError:
            serial = None
        if serial is not None:
            # The baud has no effect on the TM210's USB port.
            self.ser = serial.Serial(path, 921600, timeout=0.05)
            return
        import termios
        import tty
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        tty.setraw(self.fd)
        attr = termios.tcgetattr(self.fd)
        attr[2] |= termios.CLOCAL | termios.CREAD
        termios.tcsetattr(self.fd, termios.TCSANOW, attr)
        termios.tcflush(self.fd, termios.TCIOFLUSH)
        # O_NONBLOCK was only for the open (no wait for carrier). Reads wait
        # in select(), and a non-blocking write fails with EAGAIN as soon as
        # the USB transmit buffer is busy.
        os.set_blocking(self.fd, True)

    def read(self):
        if self.ser is not None:
            n = self.ser.in_waiting
            return self.ser.read(max(1, min(n, 65536)))
        import select
        r, _, _ = select.select([self.fd], [], [], 0.05)
        if not r:
            return b""
        data = os.read(self.fd, 65536)
        if not data:
            raise OSError("port closed (EOF)")
        return data

    def write(self, data):
        if self.ser is not None:
            self.ser.write(data)
        else:
            os.write(self.fd, data)

    def close(self):
        try:
            if self.ser is not None:
                self.ser.close()
            elif self.fd is not None:
                os.close(self.fd)
        except OSError:
            pass


class Reader(threading.Thread):
    """Reads the port and queues (monotonic time, bytes), and does nothing else.

    The host's tty holds only about 4 KB of what the IMU sends (about 70 ms at
    raw + Status, 800 Hz); past that the bytes are lost. So the port must be
    read even while the main loop waits for the SD card (an fsync measured up
    to 1 s on dietpi), which is why reading has its own thread. The queue is
    unbounded; a disk stall only makes it grow for a while.
    """

    def __init__(self, port):
        super().__init__(daemon=True)
        self.port = port
        self.q = queue.Queue()
        self.running = True

    def run(self):
        while self.running:
            try:
                data = self.port.read()
            except Exception as e:  # noqa: BLE001 - handed to the main loop
                if self.running:
                    self.q.put(e if isinstance(e, OSError) else OSError(str(e)))
                return
            if data:
                self.q.put((time.monotonic(), data))


# ---- Logger ----------------------------------------------------------------

class Logger:
    def __init__(self, args):
        self.args = args
        self.out = args.out
        os.makedirs(self.out, exist_ok=True)
        self.raw = open(os.path.join(self.out, "raw.bin"), "ab")
        self.raw_offset = self.raw.tell()
        if self.raw_offset:
            sys.exit(f"{self.out}/raw.bin already holds data; use a new --out")
        self.status_f = open(os.path.join(self.out, "status.csv"), "a", buffering=1)
        self.status_f.write("# host_time_s, tm_ts_us, temp_c, rate_hz, qos,"
                            " raw_packets, missing, backward, crc_errors\n")
        self.events_f = open(os.path.join(self.out, "events.log"), "a", buffering=1)
        self.timing_f = open(os.path.join(self.out, "timing.csv"), "a", buffering=1)
        self.timing_f.write("# host_time_s, reads, bytes, max_read_gap_ms, max_read_bytes,"
                            " max_queue_lag_ms, fsync_ms, missing\n")
        self.parser = ep.Parser()
        self.port = None
        self.reader = None
        self.stop = False
        self.t0 = time.time()
        self.meta = {"tool": "tm210_allan_log.py", "port": args.port,
                     "host_start": self.t0, "host_start_iso": _iso(self.t0),
                     "warmup_min": args.warmup_min, "qos_min": args.qos,
                     "status_output": True,
                     "hours": args.hours, "step_us": STEP_US,
                     "record_start_offset": None}
        # Stats.
        self.n_raw = 0
        self.n_missing = 0
        self.n_backward = 0
        self.n_other = 0
        self.last_ts = None
        self.last_tm_ts = None
        self.status = None          # (temp, rate, qos)
        self.status_t = 0.0
        self.next_status_csv = 0.0
        self.record_t = None
        self.qos_seen_t = None
        self.last_fsync = time.time()
        self.next_report = time.time() + 60
        self.reconnects = 0
        # Read timing (monotonic), to tell a host stall from a loss upstream.
        # The kernel buffers what arrives while this loop is busy, so a host
        # stall shows up as a long read gap followed by one large read whose
        # samples are continuous. A gap in the IMU timestamps inside a normal
        # read means the packets never reached the host's tty.
        self.last_read_t = None
        self.read_gap = 0.0         # time since the previous non-empty read
        self.read_len = 0
        self.read_call = 0.0
        self.recent_reads = []      # (t, gap, len) of the last 2 s
        self.last_fsync_s = 0.0
        self.last_fsync_mono = None
        self.sec = self._new_sec(time.monotonic())
        self.n_gaps = 0
        self.waiting = None
        self.got = None

    # -- helpers --

    def event(self, msg):
        line = f"{_iso(time.time())}  {msg}"
        print(line, flush=True)
        self.events_f.write(line + "\n")

    def save_meta(self):
        tmp = os.path.join(self.out, "meta.json.tmp")
        with open(tmp, "w") as f:
            json.dump(self.meta, f, indent=2)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, os.path.join(self.out, "meta.json"))

    def open_port(self):
        while not self.stop:
            try:
                self.port = Port(self.args.port)
                self.parser.reset()
                self.reader = Reader(self.port)
                self.reader.start()
                return True
            except OSError as e:
                self.event(f"cannot open {self.args.port}: {e}; retrying")
                self._sleep(REOPEN_S)
        return False

    def _sleep(self, s):
        end = time.time() + s
        while not self.stop and time.time() < end:
            time.sleep(0.1)

    def close_port(self):
        if self.reader is not None:
            self.reader.running = False
        if self.port is not None:
            self.port.close()
        if self.reader is not None:
            self.reader.join(1.0)
            self.reader = None

    def pump(self):
        """Take one read from the reader thread, log the bytes, handle the packets."""
        if self.reader is None:
            raise OSError("port not open")
        try:
            item = self.reader.q.get(timeout=0.05)
        except queue.Empty:
            return
        if isinstance(item, Exception):
            raise item
        t, data = item
        self.read_call = time.monotonic() - t     # queue lag: how far this loop is behind
        self.read_gap = t - self.last_read_t if self.last_read_t is not None else 0.0
        self.last_read_t = t
        self.read_len = len(data)
        self.recent_reads.append((t, self.read_gap, self.read_len))
        while self.recent_reads and t - self.recent_reads[0][0] > 2.0:
            self.recent_reads.pop(0)
        s = self.sec
        s["reads"] += 1
        s["bytes"] += self.read_len
        s["max_gap"] = max(s["max_gap"], self.read_gap)
        s["max_len"] = max(s["max_len"], self.read_len)
        s["max_call"] = max(s["max_call"], self.read_call)
        self.raw.write(data)
        self.raw_offset += len(data)
        for cmd, from_id, p in self.parser.feed(data):
            self.on_packet(cmd, from_id, p)

    def on_packet(self, cmd, from_id, p):
        if cmd == ep.CMD_RAW and len(p) == ep.RAW_LEN:
            self.n_raw += 1
            ts = int.from_bytes(p[0:4], "little")
            if self.last_ts is not None:
                d = (ts - self.last_ts) & 0xFFFFFFFF
                if d >= 0x80000000:
                    self.n_backward += 1
                elif d > STEP_US + STEP_US // 2:
                    lost = round(d / STEP_US) - 1
                    self.n_missing += lost
                    self.sec["missing"] += lost
                    self.log_gap(lost, d)
            self.last_ts = ts
        elif cmd == ep.CMD_STATUS and len(p) >= ep.STATUS_LEN:
            self.status = ep.parse_status(p)
            self.status_t = time.time()
            if self.status_t < self.next_status_csv:
                return
            self.next_status_csv = self.status_t + STATUS_CSV_S
            temp, rate, qos = self.status
            self.status_f.write(
                f"{self.status_t:.3f},{self.last_ts if self.last_ts is not None else ''},"
                f"{temp:.3f},{rate},{qos},{self.n_raw},{self.n_missing},"
                f"{self.n_backward},{self.parser.crc_errors}\n")
        else:
            self.n_other += 1
        if self.waiting is not None and cmd in self.waiting:
            self.got = (cmd, from_id, p)

    @staticmethod
    def _new_sec(t):
        return {"t": t, "reads": 0, "bytes": 0, "max_gap": 0.0, "max_len": 0,
                "max_call": 0.0, "fsync": 0.0, "missing": 0}

    def log_gap(self, lost, d_us):
        """One events.log line per timestamp gap, with the host read timing."""
        self.n_gaps += 1
        if self.n_gaps > 1000:
            if self.n_gaps == 1001:
                self.event("more than 1000 gaps; no more gap lines")
            return
        now = time.monotonic()
        recent = [r for r in self.recent_reads if now - r[0] <= 2.0]
        max_gap = max((r[1] for r in recent), default=0.0)
        fs = (f"{(now - self.last_fsync_mono):.1f} s ago, took {self.last_fsync_s * 1e3:.1f} ms"
              if self.last_fsync_mono is not None else "none yet")
        backlog = self.read_len / ((49 + 21) * 800)   # seconds of data in this read
        self.event(f"GAP {lost} packets ({d_us / 1e3:.1f} ms) at raw #{self.n_raw}: "
                   f"this read {self.read_len} B (= {backlog * 1e3:.0f} ms of data) "
                   f"{self.read_gap * 1e3:.1f} ms after the previous one, queue lag "
                   f"{self.read_call * 1e3:.1f} ms; max read gap in last 2 s "
                   f"{max_gap * 1e3:.1f} ms over {len(recent)} reads; last fsync {fs}")

    def wait_for(self, cmds, timeout):
        self.waiting = cmds
        self.got = None
        end = time.time() + timeout
        while self.got is None and time.time() < end and not self.stop:
            self.pump()
        self.waiting = None
        return self.got

    def read_setting(self, tries=10):
        for _ in range(tries):
            self.port.write(ep.request(ep.CMD_SETTING))
            r = self.wait_for({ep.CMD_SETTING}, 1.0)
            if r and len(r[2]) >= ep.SETTING_LEN:
                return r[1], r[2][:ep.SETTING_LEN]
        return None, None

    def configure(self):
        node, s = self.read_setting()
        if s is None:
            self.event("no Setting reply; is it a TM210 or TM171 USB port?")
            return False
        self.event(f"node ID {node}, {ep.describe_setting(s)}")
        self.meta.setdefault("node_id", node)
        self.meta.setdefault("setting_as_found", s.hex())
        if node != self.meta["node_id"]:
            self.event(f"node ID changed from {self.meta['node_id']} to {node}")
        if ep.log_target_matches(s):
            self.event("settings already raw and Status, 800 Hz, USB on; nothing written")
            return True
        if self.args.no_configure:
            self.event("settings differ from the logging target and --no-configure is set")
            return False
        if not ep.layout_recognised(s):
            self.event("Setting layout not recognised (bit 13 / bits 14-16); nothing written")
            return False
        target = ep.log_target(s)
        for attempt in range(5):
            self.port.write(ep.encode(ep.CMD_SETTING, node, target))
            r = self.wait_for({ep.CMD_ACK}, 1.5)
            if r and r[2][:1] == bytes([ep.CMD_SETTING]):
                break
        else:
            self.event("no Ack for the RAM write")
            return False
        _, s2 = self.read_setting(tries=3)
        if s2 is None or not ep.log_target_matches(s2):
            self.event("read-back after the RAM write does not match"
                       + (f": {ep.describe_setting(s2)}" if s2 else ""))
            return False
        self.event(f"RAM target written and read back: {ep.describe_setting(s2)}")
        return True

    def check_disk(self):
        need = (self.args.warmup_min / 60 + self.args.hours + 0.5) * BYTES_PER_HOUR
        free = shutil.disk_usage(self.out).free
        msg = f"disk: {free / 1e9:.1f} GB free, about {need / 1e9:.1f} GB needed"
        self.event(msg)
        if free < 1.1 * need and not self.args.force:
            sys.exit(msg + " (use --force to log anyway)")

    # -- main loop --

    def run(self):
        self.check_disk()
        self.event(f"start: port {self.args.port}, out {self.out}, warm-up "
                   f"{self.args.warmup_min} min, QoS >= {self.args.qos}, {self.args.hours} h")
        if not self.open_port():
            return
        if not self.configure():
            self.event("configuration failed; stopping")
            return
        self.save_meta()
        while not self.stop:
            try:
                self.pump()
                self.tick()
            except OSError as e:
                self.event(f"port error: {e}; reopening")
                self.close_port()
                self.reconnects += 1
                self.meta["reconnects"] = self.reconnects
                self.save_meta()
                self._sleep(REOPEN_S)
                if not self.open_port():
                    break
                try:
                    if not self.configure():
                        self.event("configuration after reconnect failed; will retry")
                        raise OSError("configure failed")
                except OSError:
                    self.close_port()
                    continue
        self.finish()

    def tick(self):
        now = time.time()
        if now - self.last_fsync >= FSYNC_S:
            t = time.monotonic()
            self.raw.flush()
            os.fsync(self.raw.fileno())
            self.last_fsync_mono = time.monotonic()
            self.last_fsync_s = self.last_fsync_mono - t
            self.sec["fsync"] = max(self.sec["fsync"], self.last_fsync_s)
            self.last_fsync = now
        mono = time.monotonic()
        if mono - self.sec["t"] >= 1.0:
            s = self.sec
            self.timing_f.write(f"{now:.3f},{s['reads']},{s['bytes']},{s['max_gap'] * 1e3:.1f},"
                                f"{s['max_len']},{s['max_call'] * 1e3:.1f},"
                                f"{s['fsync'] * 1e3:.1f},{s['missing']}\n")
            self.sec = self._new_sec(mono)
        if self.record_t is None:
            self.check_record_start(now)
        elif now - self.record_t >= self.args.hours * 3600:
            self.event("duration reached")
            self.stop = True
        if now >= self.next_report:
            self.report(now)
            self.next_report = now + 60

    def check_record_start(self, now):
        warm = now - self.t0 >= self.args.warmup_min * 60
        qos_ok = (self.status is not None and self.status[2] >= self.args.qos
                  and now - self.status_t < 3)
        if not warm:
            return
        if qos_ok:
            why = f"warm-up done and QoS {self.status[2]}"
        elif now - self.t0 >= (self.args.warmup_min + self.args.qos_timeout) * 60:
            why = (f"QoS still {self.status[2] if self.status else 'unknown'} "
                   f"{self.args.qos_timeout} min after the warm-up; starting anyway")
        else:
            return
        self.raw.flush()
        self.record_t = now
        self.meta["record_start_offset"] = self.raw_offset
        self.meta["record_start_host"] = now
        self.meta["record_start_iso"] = _iso(now)
        self.meta["record_start_tm_ts"] = self.last_ts
        self.meta["record_start_status"] = self.status
        self.save_meta()
        self.event(f"RECORD START at raw.bin offset {self.raw_offset}: {why}")

    def report(self, now):
        temp = f"{self.status[0]:.2f} C, QoS {self.status[2]}" if self.status else "no Status"
        phase = ("recording " + _hms(now - self.record_t) + " of " + _hms(self.args.hours * 3600)
                 if self.record_t else "warm-up " + _hms(now - self.t0))
        print(f"{_iso(now)}  {phase}: {self.n_raw} raw, {self.n_missing} missing, "
              f"{self.n_backward} backward, {self.parser.crc_errors} CRC, {self.n_other} other, "
              f"{temp}, {self.raw_offset / 1e6:.0f} MB", flush=True)

    def finish(self):
        self.raw.flush()
        os.fsync(self.raw.fileno())
        self.raw.close()
        self.close_port()
        now = time.time()
        self.meta.update({"host_end": now, "host_end_iso": _iso(now),
                          "raw_bytes": self.raw_offset, "raw_packets": self.n_raw,
                          "missing_packets": self.n_missing,
                          "gaps": self.n_gaps,
                          "backward_steps": self.n_backward,
                          "crc_errors": self.parser.crc_errors,
                          "skipped_bytes": self.parser.skipped,
                          "other_packets": self.n_other,
                          "reconnects": self.reconnects})
        self.save_meta()
        self.report(now)
        self.event("stopped" + ("" if self.record_t else " before the record start"))


def _iso(t):
    return time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(t))


def _hms(s):
    s = int(s)
    return f"{s // 3600}:{s // 60 % 60:02d}:{s % 60:02d}"


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog=__doc__[__doc__.index("What it does"):])
    ap.add_argument("--port", required=True, help="USB serial port (COM13, /dev/ttyACM0, ...)")
    ap.add_argument("--out", required=True, help="new output directory")
    ap.add_argument("--warmup-min", type=float, default=30,
                    help="minutes from start before the record may begin (default 30)")
    ap.add_argument("--qos", type=int, default=5, help="Status QoS needed to begin (default 5)")
    ap.add_argument("--qos-timeout", type=float, default=15,
                    help="minutes after the warm-up to begin without that QoS (default 15)")
    ap.add_argument("--hours", type=float, default=12,
                    help="record length after the record start (default 12)")
    ap.add_argument("--no-configure", action="store_true",
                    help="never write settings; stop if they are not the logging target")
    ap.add_argument("--force", action="store_true", help="log even if the disk looks too small")
    args = ap.parse_args()

    lg = Logger(args)

    def on_signal(signum, frame):
        lg.stop = True
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    if hasattr(signal, "SIGHUP"):
        signal.signal(signal.SIGHUP, signal.SIG_IGN)
    lg.run()


if __name__ == "__main__":
    main()
