#!/usr/bin/env python3
"""Plot inslib_replay output: the published ellipsoid height against the
simulated truth, the local heights, the height an RTK outage would hand over
to (barometer + learned offset), the offset filter, pressure and temperature.

  inslib_replay_plot.py OUT.png RUN.csv[=label] [RUN.csv[=label] ...]
      [--settle 600] [--utc-offset 1] [--leap 18]

Each run is one inslib_replay output (e.g. --height gnss and --height baro
of the same log). Statistics skip the first --settle seconds after the
filter reaches FULL. Needs numpy and matplotlib.
"""
import argparse
from datetime import datetime, timedelta, timezone

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import matplotlib.dates as mdates  # noqa: E402


def load(f):
    return np.genfromtxt(f, delimiter=",", names=True, missing_values="nan", filling_values=np.nan)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("out")
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--settle", type=float, default=600.0, help="seconds after FULL left out of the statistics")
    ap.add_argument("--utc-offset", type=float, default=1.0, help="local time zone [h] (default CET)")
    ap.add_argument("--leap", type=float, default=18.0, help="GPS-UTC leap seconds")
    a = ap.parse_args()

    epoch = datetime(1980, 1, 6, tzinfo=timezone.utc)
    tz = timezone(timedelta(hours=a.utc_offset))

    fig, ax = plt.subplots(5, 1, figsize=(13, 16), sharex=True)
    first = None
    for i, spec in enumerate(a.runs):
        path, _, label = spec.partition("=")
        label = label or path
        D = load(path)
        D = D[np.isfinite(D["gps_s"])]
        t = [(epoch + timedelta(seconds=g - a.leap)).astimezone(tz) for g in D["gps_s"]]
        full = D["mode"] == 3
        s = D["t_s"] > (D["t_s"][np.argmax(full)] + a.settle if full.any() else np.inf)
        col = f"C{(0, 3, 2, 4, 5)[i % 5]}"
        truth = D["truth_h_ell"]
        e = D["h_ell"] - truth
        o = D["baro_alt_h"] + D["offset_m"] - truth
        ax[0].plot(t, e, lw=0.6, color=col, label=f"{label}: rms {np.sqrt(np.nanmean(e[s] ** 2)):.3f} m, "
                                                  f"max |e| {np.nanmax(np.abs(e[s])):.2f} m")
        ax[1].plot(t, D["height_m"], lw=0.8, color=col, label=f"{label}: height_m")
        ax[1].plot(t, D["ins_h"] - truth, lw=0.8, color=col, ls="--", label=f"{label}: ins h − truth")
        ax[2].plot(t, o, lw=0.6, color=col, label=f"{label}: rms {np.sqrt(np.nanmean(o[s] ** 2)):.2f} m, "
                                                  f"offset sd ~{np.nanmedian(D['offset_sd'][s]):.2f} m")
        ax[3].plot(t, D["offset_m"], lw=0.8, color=col, label=f"{label}: offset_m")
        ax[3].plot(t, D["datum_offset"], lw=0.8, color=col, ls=":", label=f"{label}: datum_offset")
        if first is None:
            first = (t, D)
        print(f"{label}: h_ell err rms {np.sqrt(np.nanmean(e[s] ** 2)):.3f} mean {np.nanmean(e[s]):+.3f} "
              f"p1/p99 {np.nanpercentile(e[s], 1):+.2f}/{np.nanpercentile(e[s], 99):+.2f}; "
              f"handover err rms {np.sqrt(np.nanmean(o[s] ** 2)):.2f} p1/p99 {np.nanpercentile(o[s], 1):+.2f}/"
              f"{np.nanpercentile(o[s], 99):+.2f}; offset sd median {np.nanmedian(D['offset_sd'][s]):.2f}; "
              f"height_m end {D['height_m'][-1]:.2f}; FULL {np.mean(full[s]) * 100:.1f} % of settled")

    t, D = first
    ax[1].plot(t, D["baro_alt_h"], lw=0.8, color="k", ls=":", label="baro_alt h")
    ax[0].axhline(0, color="k", lw=0.5)
    ax[0].set_ylabel("h_ell − truth [m]")
    ax[0].set_title("published ellipsoid height (nav_suite_get_height_ellipsoid) against the simulated truth",
                    fontsize=9)
    ax[1].set_ylabel("[m]")
    ax[1].set_title("local heights above the start point, and INS's own ellipsoid height error", fontsize=9)
    ax[2].axhline(0, color="k", lw=0.5)
    ax[2].set_ylabel("baro_alt + offset − truth [m]")
    ax[2].set_title("height an RTK outage would hand over to (barometer + learned offset)", fontsize=9)
    ax[3].set_ylabel("offset filter [m]")
    ax[3].set_title("local_gnss offset (ellipsoid height of the datum origin) and the anchored datum_offset",
                    fontsize=9)
    for k in range(4):
        ax[k].legend(fontsize=8, loc="best")
    ax[4].plot(t, D["p_pa"] / 100, lw=0.8, color="C2")
    ax[4].set_ylabel("pressure [hPa]", color="C2")
    a4 = ax[4].twinx()
    a4.plot(t, D["temp_c"], lw=0.8, color="C1")
    a4.set_ylabel("baro temp [°C]", color="C1")
    ax[4].xaxis.set_major_formatter(mdates.DateFormatter("%H:%M", tz=tz))
    ax[4].set_xlabel(f"local time (UTC{a.utc_offset:+g})")
    fig.suptitle("INSLIB replay: logged IMU + barometer, simulated RTK fixed")
    fig.tight_layout()
    fig.savefig(a.out, dpi=100)


if __name__ == "__main__":
    main()
