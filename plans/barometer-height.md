# Barometer height: weather drift, the offset filter and the published height

## Context

The BMP581 barometer (100 Hz on I2C1, `firmware/drivers/bmp581`) feeds INSLIB `nav_suite` in two ways, depending on the web setting "INS height from" (`height_src`, see [inslib-nav-suite.md](inslib-nav-suite.md)):

- **GNSS height** (`baro_height_disable`; auto on the survey pole): the INS fuses the full 3D RTK position. The barometer runs only in `baro_alt` and in the offset filter `local_gnss_alt`, which learns the barometer's offset to the ellipsoid. During a GNSS outage the absolute height is `baro_alt` + offset.
- **Barometric height** (auto on the tractor, multirotor and fixed wing): the INS takes its height from the barometer for the whole run and never re-ties it to GNSS (INSLIB REQ-NAV-055). The published ellipsoid height (`nav_suite_get_height_ellipsoid()`, the `llh[2]` in `nav_core1.c`) is the INS height less the drift the offset filter has seen since the datum was fixed.

This plan records what an overnight log showed about the barometer, why the published height in barometric mode was metres off, what was changed (2026-11-18), and what is still open. Figures from the replay are from one stationary night with simulated GNSS; treat them as a bench result, not as tuned for use in motion.

## The overnight log (2026-11-17)

`log on` over USB (`firmware/tools/field_log.py record`), 22:36 to 06:03 CET, 7.4 h, the unit standing still, every sensor at full rate (IMU 803/s, barometer 101/s, GNSS 20/s, NAV 10/s), no CRC errors or dropped records. File `baro_overnight_20261117_223640.bin` (1.4 GB, not in the repo).

| | Measured |
|---|---|
| Pressure | 975.25 → 970.65 hPa, falling steadily: weather. As ISA height that is 39.6 m; `baro_alt` rose 33 m over the INS run (about 4.5 m/h, 1.2 mm/s) |
| Barometer temperature | 23.07 → 20.75 °C. Too little change, and too mixed with the weather, to separate a temperature effect |
| Noise | 1 s means are smooth (a few cm). One bad sample at 04:28: 130 558 Pa and +1.3 °C for one read; `baro_alt` dropped it as implausible |
| INSLIB `height_m` (logged) | 0.5 → 33.7 m, following the weather: the unit ran with the barometric INS height that night |
| GNSS | RTK fixed 19 898 s, float 4 051 s, DGPS 2 609 s, L1 int 226 s. The "fixed" height sat at different levels between about 108 and 128 m MSL, each held 10–60 min, with the unit not moving: wrong integer fixes from a poor antenna view. Most of the fixed epochs are at 113.05 m MSL (ellipsoid 147.05 m), taken as the truth below |

Because the logged GNSS height was not a usable reference, the INS was judged by replaying the log with simulated GNSS (next section).

## Replay with simulated RTK

`firmware/tools/inslib_replay.c` runs the logged IMU, magnetometer and barometer through `nav_suite` with the firmware's settings (`app_config.h`) and replaces the GNSS by an RTK fix for the whole run: the median position of the logged fixed epochs, white noise of 2.5 cm north/east and 5 cm up, at every logged epoch's validity and arrival time. Not modelled: the pole aiding, the IMU calibration from flash, real (time-correlated) RTK noise. How to run it: [BUILDING.md](../BUILDING.md), "Log tools".

Published ellipsoid height against the truth, statistics from 10 min after FULL; "handover" is `baro_alt` + offset − truth, the height a GNSS outage would start from:

| Run | Published height − truth | Handover | Offset σ (filter's own) |
|---|---|---|---|
| GNSS height (pole default) | 0.000 m mean, 1.1 cm rms | 2 cm rms | 0.93 m |
| Barometric height, firmware before 2026-11-18 (INSLIB offset defaults 10 s, 3×) | **+3.44 m** mean | 10 cm rms | 2.29 m |
| Barometric, offset 1 s, 1.5× | +1.36 m | 2 cm | 0.93 m |
| Barometric, 1 s, 1.5×, random walk 0.03 / 0.01 / 0.003 m/√s | +0.71 / +0.88 / +1.69 m | 16 / 44 / 130 cm | 0.30 / 0.17 / 0.10 m |
| Barometric, 1 s, 1.5×, barometer σ per pair 0.2 m | +0.30 m | 3 cm | 0.24 m |
| **Barometric, 1 s, 1.5×, barometer σ 0.1 m (firmware since 2026-11-18)** | **+0.16 m mean, 0.17 m rms**, under 0.4 m from the start | 4 cm | 0.15 m |
| Barometric, 1 s, 1.5×, barometer σ 0.05 m | +0.09 m mean, 0.105 m rms | 5 cm | 0.10 m |
| GNSS height with barometer σ 0.1 m | unchanged (1.1 cm) | 4 cm | 0.15 m |

## Why the barometric height was metres off

`nav_suite_get_height_ellipsoid()` (`external/INSLIB/src/nav_suite.c`) does not take the whole drift back off: it soft-thresholds it by one σ of the drift estimate, `sqrt(offset_σ² + anchor variance)`, about √2 × the offset filter's σ. A drift inside that margin is ignored; a larger one is corrected less the margin, on every epoch. Checked on the last row of the default run: INS 180.17 m − drift 32.95 m + margin ≈ 3.25 m = 150.47 m, published 150.52 m, truth 147.05 m.

That σ was far too pessimistic (2.3 m against a real error of 2–10 cm), and the reason is the barometer σ each baro/GNSS pair is weighted with: `R = (local_σ × inflation)² + (gnss_σ × inflation)²` (`baro_alt.c`), where the firmware passed no σ, so INSLIB used `INS_DEFAULT_BARO_STDDEV_M` = 2 m. Over a 1 s pair the barometer's real noise is centimetres.

Lowering the offset random walk also shrinks the σ, but then the offset follows the weather late, and the bias stays (table above). The barometer σ shrinks it without slowing the tracking.

A patched INSLIB that applies the whole drift once it exceeds the margin (a hard instead of a soft threshold) was also tried on a copy: +0.27 m with INSLIB's offset defaults, +0.03 m mean (0.14 m rms) with 1 s, 1.5×, but no correction at all in the first 15–25 min until the drift passed the margin (errors up to 1.4–3.6 m there). With the barometer σ at 0.1 m it adds little (0.08 m rms), so it is not needed; it could go to INSLIB upstream as a suggestion.

## What changed (2026-11-18)

- Web setting "offset barometer sigma" (`settings_t.offset_baro_sd_m`, settings version 9), default `APP_HEIGHT_OFFSET_BARO_SD_M` = 0.1 m, set as `local_gnss_cfg.local_stddev_m` in `filter_init()` (`nav_core1.c`), **only with the barometric INS height**. With the GNSS height the offset's σ reaches no output, and INSLIB's 2 m averages more pairs (2 cm against 4 cm rms handover error).
- The other offset settings (1 s, 1.5×, INSLIB's 0.3 m/√s) now apply with either height source; before, the barometric height used INSLIB's 10 s, 3×.
- 0.1 m rather than the slightly better 0.05 m because in motion the ISA model error grows with the height change, and a σ that is too small turns real pairs into outliers (chi² downweighting).

## Open

1. **Test in motion.** A log with height changes (a tractor on a slope, a multirotor climbing) and real RTK, replayed with `inslib_replay.sh` at barometer σ 0.05 / 0.1 / 0.2 m: does the offset filter downweight real pairs (`local_gnss.n_downweighted`), and how does the published height compare with RTK?
2. **A real RTK dropout.** The handover error was only computed, not seen: mask GNSS in a replay (or log a real outage) and check the step and the drift during the outage with both height sources.
3. **Antenna view for bench logs.** The night's "fixed" heights jumped by up to 15 m. For a usable GNSS reference the antenna needs a clear sky; otherwise only the simulated-GNSS replay is meaningful.
4. **Temperature.** The barometer cooled 2.3 °C while the weather moved 4.6 hPa; a separate test (a temperature change over a short time, weather steady) would show whether the BMP581 needs a temperature correction.
5. **INSLIB upstream (optional).** The soft threshold leaves a bias of one σ on any real drift, and the offset filter's σ with the default barometer σ (2 m) is about 50× its real error on a stationary unit. Worth reporting to INSLIB with these numbers; the firmware does not depend on it.
