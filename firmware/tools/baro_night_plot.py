#!/usr/bin/env python3
"""Plot a field log's barometer against the GNSS height, from the 1 s rows
of field_log_1s.py: GNSS height by fix type with the barometric (ISA)
height laid over it, their difference while RTK fixed, pressure and sensor
temperature, satellites and the GNSS height sigma, and INSLIB's height_m.

  baro_night_plot.py LOG_1s.csv OUT.png [--utc-offset 1] [--leap 18]

The barometric height is offset to the median GNSS height of the fixed
seconds (or of all, without any). Needs numpy and matplotlib.
"""
import argparse
import csv
from datetime import datetime, timedelta, timezone

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import matplotlib.dates as mdates  # noqa: E402

FIX_NAMES = {16: "single", 17: "DGPS", 18: "SBAS", 32: "L1 float", 34: "float", 48: "L1 int", 49: "WL int",
             50: "fixed"}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv")
    ap.add_argument("out")
    ap.add_argument("--utc-offset", type=float, default=1.0, help="local time zone [h] (default CET)")
    ap.add_argument("--leap", type=float, default=18.0, help="GPS-UTC leap seconds (the log has no TIME record)")
    a = ap.parse_args()

    rows = [r for r in csv.DictReader(open(a.csv)) if r["p_pa"] and r["gps_s"] and r["gnss_h_msl"]]
    f = lambda k: np.array([float(r[k]) if r[k] else np.nan for r in rows])  # noqa: E731
    gps, p, tc, h, sdh, nh, nu = (f("gps_s"), f("p_pa"), f("temp_c"), f("gnss_h_msl"), f("sd_hgt"),
                                  f("nav_height_m"), f("n_used"))
    pt = np.array([int(r["pos_type"]) for r in rows])
    epoch = datetime(1980, 1, 6, tzinfo=timezone.utc)
    tz = timezone(timedelta(hours=a.utc_offset))
    t = np.array([(epoch + timedelta(seconds=g - a.leap)).astimezone(tz) for g in gps])

    fix = pt == 50
    ref = fix if fix.any() else np.ones_like(fix)
    hb_raw = 44330.0 * (1.0 - (p / 101325.0) ** (1 / 5.255))  # ISA, standard sea-level pressure
    off = np.nanmedian((h - hb_raw)[ref])
    hb = hb_raw + off
    d = hb - h

    fig, ax = plt.subplots(5, 1, figsize=(13, 15), sharex=True)
    for k in sorted(set(pt)):
        s = pt == k
        ax[0].plot(t[s], h[s], ".", ms=1, label=f"GNSS {FIX_NAMES.get(k, k)} ({s.sum()} s)")
    ax[0].plot(t, hb, lw=0.8, color="k", label=f"baro ISA {off:+.1f} m")
    ax[0].set_ylabel("height MSL [m]")
    ax[0].legend(loc="upper left", fontsize=8, markerscale=8)
    lo, hi = np.nanpercentile(np.r_[h[ref], hb], [0.5, 99.5])
    ax[0].set_ylim(lo - 3, hi + 3)

    ax[1].plot(t[ref], d[ref], ".", ms=1, color="C3")
    ax[1].axhline(0, color="k", lw=0.5)
    ax[1].set_ylabel("baro − GNSS [m]")
    ax[1].set_title(f"{'fixed' if fix.any() else 'all'} only: sd {np.nanstd(d[ref]):.2f} m, 5–95 % "
                    f"{np.nanpercentile(d[ref], 5):.2f} .. {np.nanpercentile(d[ref], 95):.2f} m", fontsize=9)
    ax[1].set_ylim(np.nanpercentile(d[ref], 1) - 2, np.nanpercentile(d[ref], 99) + 2)

    ax[2].plot(t, p / 100, lw=0.8, color="C2")
    ax[2].set_ylabel("pressure [hPa]", color="C2")
    a2 = ax[2].twinx()
    a2.plot(t, tc, lw=0.8, color="C1")
    a2.set_ylabel("baro temp [°C]", color="C1")

    ax[3].plot(t, nu, lw=0.6, color="C0")
    ax[3].set_ylabel("sats used")
    a3 = ax[3].twinx()
    a3.plot(t, sdh, lw=0.6, color="C5")
    a3.set_ylabel("GNSS sd hgt [m]", color="C5")
    a3.set_yscale("log")

    ax[4].plot(t, nh, lw=0.8, color="C4")
    ax[4].set_ylabel("INSLIB height_m")
    ax[4].xaxis.set_major_formatter(mdates.DateFormatter("%H:%M", tz=tz))
    ax[4].set_xlabel(f"local time (UTC{a.utc_offset:+g})")
    fig.suptitle(f"Barometer log {t[0]:%Y-%m-%d %H:%M} – {t[-1]:%m-%d %H:%M} ({len(t) / 3600:.1f} h)")
    fig.tight_layout()
    fig.savefig(a.out, dpi=100)

    types = {FIX_NAMES.get(k, k): int((pt == k).sum()) for k in sorted(set(pt))}
    print("pos types [s]:", types)
    print(f"offset {off:.2f} m; diff sd {np.nanstd(d[ref]):.3f} m; temp {np.nanmin(tc):.2f}..{np.nanmax(tc):.2f} C; "
          f"p {np.nanmin(p) / 100:.2f}..{np.nanmax(p) / 100:.2f} hPa "
          f"({np.nanmax(hb_raw) - np.nanmin(hb_raw):.1f} m equiv)")
    print(f"INSLIB height_m: {np.nanmin(nh):.2f}..{np.nanmax(nh):.2f}")


if __name__ == "__main__":
    main()
