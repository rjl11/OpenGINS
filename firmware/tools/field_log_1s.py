#!/usr/bin/env python3
"""Reduce a field log (field_log.py record) to 1 s rows: barometer, GNSS
height and fix, INSLIB height. Fast enough for a whole night (1.4 GB in
about 30 s on a PC): C CRC (binascii), only the barometer, GNSS and NAV
records unpacked. Safe on a log that is still being written (reads up to
the current end).

  field_log_1s.py LOG.bin OUT.csv

Columns: seg (log session, one per START), t_s (Pico second), gps_s (mean
GNSS epoch time of that second, from the GNSS records' week/tow), p_pa,
temp_c, gnss_h_msl, undulation, sd_hgt, pos_type (the worst of the second),
gnss_vz, diff_age, n_used, nav_mode, nav_h_ell, nav_height_m.
Plot with baro_night_plot.py.
"""
import argparse
import binascii
import csv
import mmap
import os
import struct
import sys
from collections import defaultdict

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import field_log as fl  # noqa: E402


def reduce(path):
    gs = struct.Struct(fl.LAYOUT[fl.GNSS][0])
    bs = struct.Struct(fl.LAYOUT[fl.BARO][0])
    ns = struct.Struct(fl.LAYOUT[fl.NAV][0])
    sync = bytes([fl.SYNC0, fl.SYNC1])
    seg, crc_err = -1, 0
    bins = defaultdict(lambda: defaultdict(list))
    with open(path, "rb") as f:
        b = mmap.mmap(f.fileno(), 0, access=mmap.ACCESS_READ)
        n_all, i = len(b), 0
        while True:
            j = b.find(sync, i)
            if j < 0 or j + 4 > n_all:
                break
            n = b[j + 3]
            end = j + 6 + n
            if end > n_all:
                break
            body = b[j + 2:j + 4 + n]
            # CRC-16/CCITT-FALSE, as field_log.crc16()
            if binascii.crc_hqx(body, 0xFFFF) != (b[j + 4 + n] | (b[j + 5 + n] << 8)):
                crc_err += 1
                i = j + 1
                continue
            i = end
            rt, pl = body[0], body[2:]
            if rt == fl.START:
                seg += 1
            elif rt == fl.BARO:
                t, p, tc = bs.unpack_from(pl)
                d = bins[(seg, t // 1_000_000)]
                d["p"].append(p)
                d["tc"].append(tc)
            elif rt == fl.GNSS:
                v = gs.unpack_from(pl)
                d = bins[(seg, v[0] // 1_000_000)]
                d["gps"].append(v[2] * 604800 + v[3] * 1e-3)
                d["pt"].append(v[4])
                d["h"].append(v[10])
                d["und"].append(v[11])
                d["sdh"].append(v[14])
                d["age"].append(v[15])
                d["nu"].append(v[17])
                d["vz"].append(v[20])
            elif rt == fl.NAV:
                v = ns.unpack_from(pl)
                d = bins[(seg, v[0] // 1_000_000)]
                d["mode"].append(v[1])
                d["nh"].append(v[9])
                d["nhgt"].append(v[-1])
    return bins, seg + 1, crc_err


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log")
    ap.add_argument("out")
    a = ap.parse_args()
    bins, nseg, crc_err = reduce(a.log)

    def m(x):
        return "" if not x else f"{sum(x) / len(x):.4f}"

    # pos_type: the lowest code of the second (lower = weaker fix in the
    # UM980 BESTNAV numbering used here: 16 single .. 50 narrow int)
    with open(a.out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["seg", "t_s", "gps_s", "p_pa", "temp_c", "gnss_h_msl", "undulation", "sd_hgt", "pos_type",
                    "gnss_vz", "diff_age", "n_used", "nav_mode", "nav_h_ell", "nav_height_m"])
        for (s, sec), d in sorted(bins.items()):
            w.writerow([s, sec, m(d["gps"]), m(d["p"]), m(d["tc"]), m(d["h"]), m(d["und"]), m(d["sdh"]),
                        min(d["pt"]) if d["pt"] else "", m(d["vz"]), m(d["age"]), m(d["nu"]),
                        d["mode"][-1] if d["mode"] else "", m(d["nh"]), m(d["nhgt"])])
    print(f"{len(bins)} rows, {nseg} session(s), {crc_err} CRC errors -> {a.out}")


if __name__ == "__main__":
    main()
