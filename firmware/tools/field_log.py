#!/usr/bin/env python3
"""Record and decode the Pico's USB log (firmware/app/usb_log.h).

  record PORT [-o FILE]   turn the log on, save the raw stream, show the
                          status text and a summary every few seconds;
                          Ctrl-C turns the log off again
  summary FILE            record counts, rates, time span, CRC errors
  decode FILE [-o DIR]    one CSV per record type, and segments.csv

The raw file is the exact byte stream, so it can be decoded again later with
a newer version of this script. Needs pyserial for 'record' only.

GNSS time: the Pico's clock (time_us_64) restarts at 0 when it reboots, so
'record' reopens the port after a reboot, turns the log on again and appends
to the same file, and 'decode' adds two columns to every CSV with a Pico
time: 'seg', the log session (one per START record), and 'gps_s', that time
as GPS seconds since 1980-01-06, mapped from the GNSS records' PPS-tagged
epochs of the same session. Rows from before and after a reboot then line up
on gps_s. segments.csv lists the sessions, with UTC from the TIME records'
leap seconds.

Field test use: plans/wheel-odometry-aiding.md, "Field testing".
"""
import argparse
import csv
import os
import signal
import struct
import sys
import time
from bisect import bisect_right
from collections import Counter
from datetime import datetime, timedelta, timezone

SYNC0, SYNC1 = 0xA7, 0x4C

START, TEXT, IMU, GNSS, HEADING, BARO, NAV, SPEED_EDGES, FLOW_EVENT, FLOW_FRAME, DROPPED, TIME = range(1, 13)
NAMES = {START: "start", TEXT: "text", IMU: "imu", GNSS: "gnss", HEADING: "heading", BARO: "baro",
         NAV: "nav", SPEED_EDGES: "speed_edges", FLOW_EVENT: "flow_event", FLOW_FRAME: "flow_frame",
         DROPPED: "dropped", TIME: "time"}

# Fixed-layout records: struct format and column names (usb_log.c).
LAYOUT = {
    IMU: ("<Qf3f3f3fB", ["t_us", "dt_s", "gyr_x", "gyr_y", "gyr_z", "acc_x", "acc_y", "acc_z",
                          "mag_x", "mag_y", "mag_z", "flags"]),
    GNSS: ("<QQHIBBBBdddfffffBBfffff", ["t_valid_us", "t_arrival_us", "week", "tow_ms", "pos_type", "sol_status",
                                       "vel_type", "flags", "lat_deg", "lon_deg", "hgt_msl_m", "undulation_m",
                                       "sd_lat_m", "sd_lon_m", "sd_hgt_m", "diff_age_s", "n_tracked", "n_used",
                                       "hor_speed_mps", "track_deg", "vert_speed_mps", "sd_hor_mps", "sd_vert_mps"]),
    HEADING: ("<QHIBBfffffBB", ["t_valid_us", "week", "tow_ms", "pos_type", "flags", "length_m", "heading_deg",
                                "pitch_deg", "sd_heading_deg", "sd_pitch_deg", "n_tracked", "n_used"]),
    BARO: ("<Qff", ["t_us", "pressure_pa", "temperature_c"]),
    NAV: ("<QBBB3f3d3f3f3ff", ["t_us", "mode", "att_source", "flags", "roll_rad", "pitch_rad", "yaw_rad",
                               "lat_rad", "lon_rad", "h_ell_m", "vel_n", "vel_e", "vel_d",
                               "gyr_bias_x", "gyr_bias_y", "gyr_bias_z", "acc_bias_x", "acc_bias_y", "acc_bias_z",
                               "height_m"]),
    FLOW_EVENT: ("<QIBB", ["t_us", "seq", "kind", "level"]),
    DROPPED: ("<II", ["bytes", "records"]),
    TIME: ("<QHIB", ["t_us", "week", "tow_ms", "leap_s"]),
}
FLOW_KINDS = {0: "trigger", 1: "gap", 2: "strobe"}
FLOW_MSG_FLOW = 1
FLOW_MSG_FLOW_FMT = "<IIIfffHBB"
FLOW_MSG_FLOW_COLS = ["frame_seq", "pi_time_us", "pi_dt_us", "disp_x_rad", "disp_y_rad", "stddev_rad",
                      "n_tracked", "exposure_100us", "flags"]


def crc16(data):
    """CRC-16/CCITT-FALSE, as flow_link_crc16()."""
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


class Parser:
    """Splits the stream into records; bytes outside a good frame are text
    from before the log was on (or noise), returned as ('raw', bytes)."""

    def __init__(self):
        self.buf = bytearray()
        self.crc_errors = 0

    def feed(self, data):
        self.buf += data
        out = []
        raw = bytearray()
        i = 0
        b = self.buf
        while True:
            j = b.find(bytes([SYNC0, SYNC1]), i)
            if j < 0:
                keep = 1 if len(b) and b[-1] == SYNC0 else 0
                raw += b[i:len(b) - keep]
                i = len(b) - keep
                break
            raw += b[i:j]
            if j + 4 > len(b):
                i = j
                break
            n = b[j + 3]
            end = j + 4 + n + 2
            if end > len(b):
                i = j
                break
            body = bytes(b[j + 2:j + 4 + n])
            got = b[j + 4 + n] | (b[j + 5 + n] << 8)
            if crc16(body) != got:
                self.crc_errors += 1
                raw += b[j:j + 1]
                i = j + 1
                continue
            if raw:
                out.append(("raw", bytes(raw)))
                raw = bytearray()
            out.append((body[0], body[2:]))
            i = end
        if raw:
            out.append(("raw", bytes(raw)))
        del self.buf[:i]
        return out


def records(path):
    p = Parser()
    with open(path, "rb") as f:
        while True:
            chunk = f.read(1 << 16)
            if not chunk:
                break
            yield from p.feed(chunk)
    records.crc_errors = p.crc_errors


def unpack(rtype, pl):
    """Rows (lists) for one record."""
    if rtype in LAYOUT:
        fmt, _ = LAYOUT[rtype]
        return [list(struct.unpack(fmt, pl[:struct.calcsize(fmt)]))]
    if rtype == SPEED_EDGES:
        (t0,) = struct.unpack_from("<Q", pl, 0)
        rows = []
        for k in range((len(pl) - 8) // 5):
            dt, lv = struct.unpack_from("<IB", pl, 8 + 5 * k)
            rows.append([t0 + dt, "B" if lv & 0x80 else "A", lv & 1, (lv >> 1) & 1])
        return rows
    if rtype == FLOW_FRAME:
        t, mtype = struct.unpack_from("<QB", pl, 0)
        msg = pl[9:]
        if mtype == FLOW_MSG_FLOW and len(msg) == struct.calcsize(FLOW_MSG_FLOW_FMT):
            return [[t, mtype] + list(struct.unpack(FLOW_MSG_FLOW_FMT, msg))]
        return [[t, mtype] + [""] * len(FLOW_MSG_FLOW_COLS)]
    if rtype == START:
        ver, t, hz = struct.unpack_from("<BQI", pl, 0)
        return [[ver, t, hz, pl[13:].decode(errors="replace")]]
    return []


COLUMNS = {k: v[1] for k, v in LAYOUT.items()}
COLUMNS[SPEED_EDGES] = ["t_us", "input", "a", "b"]
COLUMNS[FLOW_FRAME] = ["t_arrival_us", "msg_type"] + FLOW_MSG_FLOW_COLS
COLUMNS[START] = ["version", "t_us", "sys_hz", "name"]
# Column holding the record's Pico time (the first one, but START's second).
T_COL = {k: 0 for k in COLUMNS if k != DROPPED}
T_COL[START] = 1

GPS_EPOCH = datetime(1980, 1, 6, tzinfo=timezone.utc)
WEEK_S = 604800
MIN_WEEK = 2000      # a lower week number is a receiver without time yet
CLOCK_BACK_US = 1000000  # the Pico clock going back this far is a new session


def new_session(rtype, t, last):
    """A START, or the Pico clock went back (a reboot whose START was lost)."""
    return last is None or rtype == START or t + CLOCK_BACK_US < last


class Segment:
    """One log session (a START record to the next): the Pico time to GPS
    time map, from the GNSS records' (t_valid_us, week, tow_ms)."""

    def __init__(self, index):
        self.index = index
        self.t_first = self.t_last = None
        self.pps = []      # (t_us, gps_s) of PPS-tagged epochs
        self.arrival = []  # epochs without PPS: late by the output latency
        self.leap_s = None
        self.t, self.g, self.rate = [], [], 1.0

    def see(self, t):
        self.t_first = t if self.t_first is None else min(self.t_first, t)
        self.t_last = t if self.t_last is None else max(self.t_last, t)

    def finish(self):
        pts = sorted(self.pps or self.arrival)
        self.t = [p[0] for p in pts]
        self.g = [p[1] for p in pts]
        # GPS seconds per Pico second, for times before the first epoch or
        # after the last.
        if len(pts) >= 2 and self.t[-1] > self.t[0]:
            self.rate = (self.g[-1] - self.g[0]) / ((self.t[-1] - self.t[0]) * 1e-6)

    def gps(self, t_us):
        """GPS seconds for a Pico time, or None without GNSS time."""
        t, g = self.t, self.g
        if not t:
            return None
        i = bisect_right(t, t_us)
        if 0 < i < len(t) and t[i] > t[i - 1]:
            return g[i - 1] + (g[i] - g[i - 1]) * (t_us - t[i - 1]) / (t[i] - t[i - 1])
        k = 0 if i == 0 else len(t) - 1
        return g[k] + (t_us - t[k]) * 1e-6 * self.rate

    def ppm(self):
        """How fast the Pico clock runs against GPS [ppm]."""
        return (1.0 / self.rate - 1.0) * 1e6

    def utc(self, gps_s):
        if gps_s is None or self.leap_s is None:
            return ""
        return (GPS_EPOCH + timedelta(seconds=gps_s - self.leap_s)).isoformat(timespec="milliseconds")


def segments(path):
    """First pass: the sessions in a file, with their time maps."""
    segs = []
    cur = None
    for rtype, pl in records(path):
        if rtype == "raw" or rtype not in T_COL:
            continue
        row = unpack(rtype, pl)[0]
        t = row[T_COL[rtype]]
        if new_session(rtype, t, None if cur is None else cur.t_last):
            cur = Segment(len(segs))
            segs.append(cur)
        cur.see(t)
        if rtype == GNSS:
            t_valid, week, tow_ms, flags = row[0], row[2], row[3], row[7]
            if week >= MIN_WEEK:
                (cur.pps if flags & 1 else cur.arrival).append((t_valid, week * WEEK_S + tow_ms * 1e-3))
        elif rtype == TIME:
            cur.leap_s = row[3]
    for sg in segs:
        sg.finish()
    return segs


def describe(sg):
    span = (sg.t_last - sg.t_first) * 1e-6
    g0, g1 = sg.gps(sg.t_first), sg.gps(sg.t_last)
    if g0 is None:
        return f"  session {sg.index}: {span:.1f} s, Pico t {sg.t_first} .. {sg.t_last} us, no GNSS time"
    src = f"{len(sg.pps)} PPS epochs" if sg.pps else f"{len(sg.arrival)} epochs without PPS (late by the latency)"
    when = f"UTC {sg.utc(g0)} .. {sg.utc(g1)}" if sg.leap_s is not None else f"GPS {g0:.3f} .. {g1:.3f} s"
    return f"  session {sg.index}: {span:.1f} s, {when}, from {src}, Pico clock {sg.ppm():+.1f} ppm"


def cmd_summary(args):
    counts = Counter()
    edges = 0
    drops = [0, 0]
    segs = segments(args.file)
    span = sum((sg.t_last - sg.t_first) * 1e-6 for sg in segs)
    for rtype, pl in records(args.file):
        if rtype == "raw":
            continue
        counts[rtype] += 1
        if rtype == SPEED_EDGES:
            edges += (len(pl) - 8) // 5
        elif rtype == DROPPED:
            b, r = struct.unpack("<II", pl)
            drops[0] += b
            drops[1] += r
    print(f"{args.file}: {span:.1f} s in {len(segs)} session(s), CRC errors {records.crc_errors}, "
          f"dropped {drops[0]} B in {drops[1]} records")
    for sg in segs:
        print(describe(sg))
    for k in sorted(counts):
        rate = f"{counts[k] / span:8.1f}/s" if span > 0 else ""
        print(f"  {NAMES.get(k, k):12s} {counts[k]:9d} {rate}")
    if edges:
        print(f"  {'(edges)':12s} {edges:9d}" + (f" {edges / span:8.1f}/s" if span > 0 else ""))


def cmd_decode(args):
    out_dir = args.o or os.path.splitext(args.file)[0]
    os.makedirs(out_dir, exist_ok=True)
    files, writers = {}, {}
    segs = segments(args.file)
    seg, seg_last = None, None
    text = open(os.path.join(out_dir, "text.txt"), "w", encoding="utf-8", newline="")
    try:
        for rtype, pl in records(args.file):
            if rtype == "raw":
                text.write(pl.decode(errors="replace"))
                continue
            if rtype == TEXT:
                text.write(pl.decode(errors="replace"))
                continue
            if rtype not in COLUMNS:
                continue
            timed = rtype in T_COL
            rows = unpack(rtype, pl)
            if timed:
                # The same split into sessions as segments().
                t = rows[0][T_COL[rtype]]
                if new_session(rtype, t, seg_last):
                    seg = segs[0 if seg is None else seg.index + 1]
                    seg_last = t
                seg_last = max(seg_last, t)
            if rtype not in writers:
                f = open(os.path.join(out_dir, NAMES[rtype] + ".csv"), "w", newline="")
                files[rtype] = f
                writers[rtype] = csv.writer(f)
                writers[rtype].writerow(COLUMNS[rtype] + (["seg", "gps_s"] if timed else []))
            for row in rows:
                if rtype == FLOW_EVENT:
                    row[2] = FLOW_KINDS.get(row[2], row[2])
                if timed:
                    g = seg.gps(row[T_COL[rtype]])
                    row += [seg.index, "" if g is None else f"{g:.6f}"]
                writers[rtype].writerow(row)
    finally:
        text.close()
        for f in files.values():
            f.close()
    with open(os.path.join(out_dir, "segments.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["seg", "t_first_us", "t_last_us", "gps_first_s", "gps_last_s", "utc_first", "utc_last",
                    "pps_epochs", "arrival_epochs", "leap_s", "clock_ppm"])
        for sg in segs:
            g0, g1 = sg.gps(sg.t_first), sg.gps(sg.t_last)
            w.writerow([sg.index, sg.t_first, sg.t_last, "" if g0 is None else f"{g0:.6f}",
                        "" if g1 is None else f"{g1:.6f}", sg.utc(g0), sg.utc(g1), len(sg.pps), len(sg.arrival),
                        "" if sg.leap_s is None else sg.leap_s, f"{sg.ppm():.2f}" if sg.t else ""])
    print(f"wrote {out_dir}/ ({', '.join(NAMES[k] for k in sorted(files))}, segments, text)")
    for sg in segs:
        print(describe(sg))


def cmd_record(args):
    import serial  # pyserial

    path = args.o or time.strftime("field_%Y%m%d_%H%M%S.bin")

    def open_port():
        ser = serial.Serial(args.port, 115200, timeout=0.05, write_timeout=1.0)
        ser.dtr = True
        ser.write(b"\nlog on\n")
        return ser

    ser = open_port()
    parser = Parser()
    counts = Counter()
    nbytes = 0
    t_print = time.time() + args.every
    t_rx = time.time()
    stop = []
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.append(1))
    print(f"recording to {path} (Ctrl-C to stop)", file=sys.stderr)
    try:
        with open(path, "wb") as f:
            while not stop:
                try:
                    data = ser.read(65536)
                    if data:
                        t_rx = time.time()
                    elif time.time() - t_rx > args.silence:
                        # The stream never pauses while the log is on.
                        raise serial.SerialException(f"no data for {args.silence:.0f} s")
                except (serial.SerialException, OSError) as e:
                    # The Pico rebooted (or the cable came out): its USB port
                    # went away. Reopen it, turn the log on again and append;
                    # the new START record starts a new session.
                    print(f"\n[field_log] port lost ({e}); waiting for the Pico", file=sys.stderr)
                    ser.close()
                    f.flush()
                    ser = None
                    while not stop and ser is None:
                        time.sleep(0.5)
                        try:
                            ser = open_port()
                        except (serial.SerialException, OSError):
                            ser = None
                    if ser is None:
                        break
                    t_rx = time.time()
                    print("[field_log] reconnected, log on again", file=sys.stderr)
                    continue
                if data:
                    f.write(data)
                    nbytes += len(data)
                    for rtype, pl in parser.feed(data):
                        if rtype == "raw" or rtype == TEXT:
                            if not args.quiet:
                                sys.stdout.write(pl.decode(errors="replace"))
                        else:
                            counts[rtype] += 1
                            if rtype == DROPPED:
                                b, r = struct.unpack("<II", pl)
                                print(f"\n[field_log] Pico dropped {b} B in {r} records", file=sys.stderr)
                now = time.time()
                if now >= t_print:
                    dt = args.every
                    rates = "  ".join(f"{NAMES.get(k, k)} {counts[k] / dt:.0f}/s" for k in sorted(counts))
                    print(f"[field_log] {nbytes / dt / 1000:.1f} kB/s  {rates}  crc errors {parser.crc_errors}",
                          file=sys.stderr)
                    counts.clear()
                    nbytes = 0
                    t_print = now + args.every
    finally:
        if ser is not None:
            try:
                ser.write(b"log off\n")
                ser.flush()
            except (serial.SerialException, serial.SerialTimeoutException, OSError):
                pass
            ser.close()
    print(f"saved {path}", file=sys.stderr)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("record")
    r.add_argument("port")
    r.add_argument("-o", help="output file (default field_YYYYmmdd_HHMMSS.bin)")
    r.add_argument("--every", type=float, default=5.0, help="summary interval [s]")
    r.add_argument("--silence", type=float, default=3.0, help="reopen the port after this long without data [s]")
    r.add_argument("-q", "--quiet", action="store_true", help="don't print the status text")
    r.set_defaults(func=cmd_record)
    s = sub.add_parser("summary")
    s.add_argument("file")
    s.set_defaults(func=cmd_summary)
    d = sub.add_parser("decode")
    d.add_argument("file")
    d.add_argument("-o", help="output directory (default: the file name without .bin)")
    d.set_defaults(func=cmd_decode)
    args = ap.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
