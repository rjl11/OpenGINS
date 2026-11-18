#!/usr/bin/env python3
"""Summarise a run of rtk_settings_test.py: fix rate, wrong fixes, jumps, sd honesty per setting.

    python3 rtk_settings_analyze.py runs/20261118_2130 [--blocks] [--csv blocks.csv]

The antenna doesn't move, so the reference is the median of all fixed epochs
of the run (refined once: the median of the fixed epochs within 0.5 m of the
first median). Definitions, from "Bench procedure: rejecting bad RTK fixes" in
plans/gnss-um98x-config.md:
  fixed       pos type NARROW_INT, WIDE_INT or L1_INT
  episode     consecutive fixed epochs (no gap over 1 s)
  wrong fix   an episode whose median is more than --wrong-h m horizontally or
              --wrong-v m vertically from the reference
  jump        a step of more than --jump m between epochs at most 0.5 s apart
  honest      |error| under 3 x the reported sd on all three axes
Only epochs after the block's LOGGING mark count; the time to first fix is
measured from that mark (just after CONFIG RTK RESET).
"""
import argparse, csv, glob, json, math, os, statistics, sys

FIXED = {"NARROW_INT", "WIDE_INT", "L1_INT"}
FLOAT = {"NARROW_FLOAT", "L1_FLOAT", "IONOFREE_FLOAT"}
R_EARTH = 6378137.0

_T = []
for n in range(256):
    c = n
    for _ in range(8):
        c = (c >> 1) ^ 0xEDB88320 if c & 1 else c >> 1
    _T.append(c)

def crc32_unicore(s):
    """CRC-32 as Unicore / NovAtel use it on ASCII logs: reflected 0xEDB88320, init 0, no final xor."""
    c = 0
    for b in s.encode("ascii", errors="replace"):
        c = _T[(c ^ b) & 0xFF] ^ (c >> 8)
    return c

def ascii_ok(line):
    i, j = line.find("#"), line.rfind("*")
    if i < 0 or j < i:
        return False
    try:
        return crc32_unicore(line[i + 1:j]) == int(line[j + 1:j + 9], 16)
    except ValueError:
        return False


def load_block(path):
    meta, t_log, epochs, rtks, bad = None, None, [], [], 0
    rtcm = {}
    with open(path, errors="replace") as f:
        for ln in f:
            sp = ln.find(" ")
            if sp < 0:
                continue
            try:
                t = float(ln[:sp])
            except ValueError:
                continue
            body = ln[sp + 1:].rstrip("\n")
            if body.startswith("# BLOCK "):
                meta = json.loads(body[8:])
            elif body.startswith("# LOGGING"):
                t_log = t
            elif body.startswith("# RTCM_STATS "):
                rtcm = json.loads(body[13:])
            elif body.startswith("#BESTNAVA") or body.startswith("#RTKSTATUSA"):
                if not ascii_ok(body):
                    bad += 1; continue
                if t_log is None:
                    continue
                f2 = body.split(";", 1)[1].split("*")[0].split(",")
                if body.startswith("#BESTNAVA"):
                    try:
                        epochs.append(dict(t=t, type=f2[1], lat=float(f2[2]), lon=float(f2[3]), hgt=float(f2[4]),
                                           sdn=float(f2[7]), sde=float(f2[8]), sdu=float(f2[9]),
                                           age=float(f2[11]), ntrk=int(f2[13]), nused=int(f2[14])))
                    except (IndexError, ValueError):
                        bad += 1
                else:
                    rtks.append((t, f2[12] if len(f2) > 12 else "?", f2[13] if len(f2) > 13 else "?"))
    return dict(path=path, meta=meta or {}, t_log=t_log, epochs=epochs, rtk=rtks, bad=bad, rtcm=rtcm)


def reference(blocks):
    fx = [e for b in blocks for e in b["epochs"] if e["type"] in FIXED]
    if not fx:
        fl = [e for b in blocks for e in b["epochs"]]
        if not fl:
            return None, 0
        fx = fl
    lat0 = statistics.median(e["lat"] for e in fx)
    lon0 = statistics.median(e["lon"] for e in fx)
    h0 = statistics.median(e["hgt"] for e in fx)
    ref = (lat0, lon0, h0)
    near = [e for e in fx if math.hypot(*enu(e, ref)[:2]) < 0.5]
    if len(near) > 10:
        ref = (statistics.median(e["lat"] for e in near), statistics.median(e["lon"] for e in near),
               statistics.median(e["hgt"] for e in near))
    return ref, len(fx)


def enu(e, ref):
    lat0, lon0, h0 = ref
    dn = math.radians(e["lat"] - lat0) * R_EARTH
    de = math.radians(e["lon"] - lon0) * R_EARTH * math.cos(math.radians(lat0))
    return de, dn, e["hgt"] - h0


def block_stats(b, ref, a):
    ep = b["epochs"]
    for e in ep:
        e["de"], e["dn"], e["du"] = enu(e, ref)
    s = dict(config=b["meta"].get("config", "?"), index=b["meta"].get("index", -1), n=len(ep),
             secs=(ep[-1]["t"] - ep[0]["t"]) if len(ep) > 1 else 0.0, bad_crc=b["bad"],
             rtcm_downs=b["rtcm"].get("downs", 0), rtcm_max_gap=b["rtcm"].get("max_gap_s", 0.0))
    fx = [e for e in ep if e["type"] in FIXED]
    fl = [e for e in ep if e["type"] in FLOAT]
    s["n_fixed"], s["n_float"] = len(fx), len(fl)
    s["start_fixed"] = bool(ep) and ep[0]["type"] in FIXED
    first = next((e for e in ep if e["type"] in FIXED), None)
    s["ttff"] = (first["t"] - b["t_log"]) if first else None
    # episodes
    eps, cur = [], []
    for e in ep:
        if e["type"] in FIXED and (not cur or e["t"] - cur[-1]["t"] <= 1.0):
            cur.append(e)
        else:
            if cur:
                eps.append(cur)
            cur = [e] if e["type"] in FIXED else []
    if cur:
        eps.append(cur)
    s["episodes"] = [len(x) for x in eps]
    s["episode_s"] = [x[-1]["t"] - x[0]["t"] for x in eps]
    wrong = []
    for x in eps:
        mh = statistics.median(math.hypot(e["de"], e["dn"]) for e in x)
        mv = statistics.median(abs(e["du"]) for e in x)
        if mh > a.wrong_h or mv > a.wrong_v:
            wrong.append((x[-1]["t"] - x[0]["t"], mh, mv, len(x)))
    s["wrong"] = wrong
    jumps = 0
    for p, q in zip(ep, ep[1:]):
        if q["t"] - p["t"] <= 0.5:
            d = math.sqrt((q["de"] - p["de"]) ** 2 + (q["dn"] - p["dn"]) ** 2 + (q["du"] - p["du"]) ** 2)
            if d > a.jump:
                jumps += 1
    s["jumps"] = jumps

    def honest(es):
        return sum(1 for e in es if abs(e["dn"]) < 3 * e["sdn"] and abs(e["de"]) < 3 * e["sde"]
                   and abs(e["du"]) < 3 * e["sdu"])
    s["honest_fixed"], s["honest_float"] = honest(fx), honest(fl)
    s["herr_fixed"] = [math.hypot(e["de"], e["dn"]) for e in fx]
    s["herr_float"] = [math.hypot(e["de"], e["dn"]) for e in fl]
    s["nused_fixed"] = [e["nused"] for e in fx]
    s["nused_all"] = [e["nused"] for e in ep]
    return s


def pct(a, b):
    return 100.0 * a / b if b else float("nan")

def q(xs, p):
    if not xs:
        return float("nan")
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(p * len(xs)))]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run_dir")
    ap.add_argument("--wrong-h", type=float, default=0.2)
    ap.add_argument("--wrong-v", type=float, default=0.3)
    ap.add_argument("--jump", type=float, default=0.3)
    ap.add_argument("--blocks", action="store_true", help="also print one line per block")
    ap.add_argument("--csv", help="write the per-block figures to this CSV")
    a = ap.parse_args()

    files = sorted(glob.glob(os.path.join(a.run_dir, "block_*.log")))
    blocks = [load_block(p) for p in files]
    blocks = [b for b in blocks if b["epochs"]]
    if not blocks:
        sys.exit("no epochs found")
    ref, nref = reference(blocks)
    print(f"{len(blocks)} blocks, reference from {nref} epochs: lat {ref[0]:.9f} lon {ref[1]:.9f} h {ref[2]:.3f} m")
    stats = [block_stats(b, ref, a) for b in blocks]

    if a.blocks:
        print("\n idx config    min  fixed%  ttff_s  eps  wrong  jumps  start_fixed  rtcm_gap")
        for s in stats:
            tt = f"{s['ttff']:.0f}" if s["ttff"] is not None else "-"
            print(f"{s['index']:4d} {s['config']:8s} {s['secs'] / 60:5.1f} {pct(s['n_fixed'], s['n']):6.1f} {tt:>7s}"
                  f" {len(s['episodes']):4d} {len(s['wrong']):6d} {s['jumps']:6d} {str(s['start_fixed']):>12s}"
                  f" {s['rtcm_max_gap']:8.1f}")

    by = {}
    for s in stats:
        by.setdefault(s["config"], []).append(s)
    hdr = ("config   blocks  hours  fixed%  float%  never  ttff_med  eps/h  ep_med_s  wrong  wrong_min  wrong_max_m"
           "  jumps/h  honest_fix%  honest_flt%  fix_p95_m  flt_p95_m  nused_fix")
    print("\n" + hdr)
    for cfg in sorted(by, key=lambda k: (k != "base", k)):
        ss = by[cfg]
        n = sum(s["n"] for s in ss); nf = sum(s["n_fixed"] for s in ss); nfl = sum(s["n_float"] for s in ss)
        hours = sum(s["secs"] for s in ss) / 3600 or float("nan")
        tt = [s["ttff"] for s in ss if s["ttff"] is not None]
        never = sum(1 for s in ss if s["ttff"] is None)
        eps = [x for s in ss for x in s["episode_s"]]
        wrong = [w for s in ss for w in s["wrong"]]
        wrong_s = sum(w[0] for w in wrong)
        wmax = max((max(w[1], w[2]) for w in wrong), default=0.0)
        jumps = sum(s["jumps"] for s in ss)
        hf = sum(s["honest_fixed"] for s in ss); hfl = sum(s["honest_float"] for s in ss)
        ef = [x for s in ss for x in s["herr_fixed"]]; efl = [x for s in ss for x in s["herr_float"]]
        nu = [x for s in ss for x in s["nused_fixed"]]
        print(f"{cfg:8s} {len(ss):6d} {hours:6.2f} {pct(nf, n):7.1f} {pct(nfl, n):7.1f} {never:6d}"
              f" {statistics.median(tt) if tt else float('nan'):9.0f} {len(eps) / hours:6.1f}"
              f" {statistics.median(eps) if eps else float('nan'):9.0f} {len(wrong):6d} {wrong_s / 60:10.1f}"
              f" {wmax:12.2f} {jumps / hours:8.1f} {pct(hf, nf):12.1f} {pct(hfl, nfl):12.1f}"
              f" {q(ef, 0.95):10.3f} {q(efl, 0.95):10.3f} {statistics.median(nu) if nu else float('nan'):10.0f}")
    starts = sum(1 for s in stats if s["start_fixed"])
    print(f"\nblocks that started already fixed (RTK RESET not clearing the fix): {starts} of {len(stats)}")
    print(f"bad CRC lines: {sum(s['bad_crc'] for s in stats)}, RTCM drops: {sum(s['rtcm_downs'] for s in stats)},"
          f" max RTCM gap {max(s['rtcm_max_gap'] for s in stats):.1f} s")

    if a.csv:
        with open(a.csv, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["index", "config", "minutes", "epochs", "fixed_pct", "ttff_s", "episodes", "wrong",
                        "wrong_s", "jumps", "honest_fixed_pct", "honest_float_pct", "fix_p95_m", "start_fixed"])
            for s in stats:
                w.writerow([s["index"], s["config"], round(s["secs"] / 60, 1), s["n"],
                            round(pct(s["n_fixed"], s["n"]), 2), s["ttff"] and round(s["ttff"], 1),
                            len(s["episodes"]), len(s["wrong"]), round(sum(x[0] for x in s["wrong"]), 1),
                            s["jumps"], round(pct(s["honest_fixed"], s["n_fixed"]), 1),
                            round(pct(s["honest_float"], s["n_float"]), 1), round(q(s["herr_fixed"], 0.95), 3),
                            s["start_fixed"]])


if __name__ == "__main__":
    main()
