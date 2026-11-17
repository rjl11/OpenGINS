# IMU auto-calibration in the background

## Context
The IMU and magnetometer calibration is done by hand today, with INSLIB's offline tools: the multi-pose fit `external/INSLIB/tools/inslib_imu_calib.py` on the bench, and `inslib_mag_calib.py` with the unit in its final place ([imu-tm210.md](imu-tm210.md), "Calibration steps"). Only the biases are learned while running: gyro and accelerometer bias by the INS states, and the magnetometer hard iron with `estimate_mag_bias` ([inslib-nav-suite.md](inslib-nav-suite.md), "Magnetometer"). None of it survives a power cycle.

This plan adds a background calibrator on core 0. It collects the motion the unit sees during normal use, refines every IMU and magnetometer term that this motion makes observable, and stores the result in flash. On the next boot the result becomes INSLIB's fixed calibration. Status: design only, no code. Every CPU, memory and accuracy figure here is an estimate, not yet measured.

The firmware is meant to be universal, not tractor-only. The same unit goes on a tractor, a GNSS survey pole and a UAV, and these see very different motion. So the design is **driven by observability, not by platform**: one estimator for all platforms, which updates a term only when the collected data pins it down, and otherwise keeps the prior. The platform setting changes only the gates (when data may be used) and which reference sources count as heading.

## The calibration model
The calibrator learns the same terms INSLIB already applies (`ins_options_t` in `external/INSLIB/src/ins.h`, REQ-NAV-037/039), so its output goes straight into INSLIB with no new filter code:

| Term | INSLIB field | Parameters |
|---|---|---|
| Accelerometer | `imu_acc_misalignment`, `imu_acc_fixed_bias`: `a = M_a·(raw − b_a)` | 9 + 3 |
| Gyro | `imu_gyr_misalignment`, `imu_gyr_fixed_bias` | 9 + 3 |
| Magnetometer | `mag_misalignment` (soft iron, scale, alignment), `mag_fixed_bias` (hard iron) | 9 + 3 |
| Gyro bias vs temperature | none in INSLIB; core 0 subtracts `k·(T − T_ref)` before queueing the sample | 3 (+3 for the accelerometer if the data shows it) |

The IMU mounting rotation (`mount_rpy_deg` / `imu_mount_compose()`) and the GNSS lever arm are not part of this plan. On the survey pole the IMU's tilt against the pole axis is learned in [pole-tilt-aiding.md](pole-tilt-aiding.md), item 5, and only changes the tip lever arm, not these sensor-frame sets.

## What each platform makes observable

| Term | Tractor | Survey pole | UAV |
|---|---|---|---|
| Gyro bias | yes, at every standstill | yes, at every held point | yes, on the ground before take-off |
| Gyro bias vs temperature | yes, over weeks of standstills | yes | partly: take-off warms the unit quickly, few still periods in flight |
| Accel bias | horizontal from headland turns with GNSS; vertical weakly | **yes**, from tilted static poses | from ground poses only |
| Accel scale and misalignment | no (±15° of tilt) | **mostly**: points held at many tilts, the pole laid down for transport; poses pointing down are rare | no |
| Gyro Z scale | **yes**, turns against GNSS heading | yes, rotations between poses | yes, yaw turns against GNSS course (fixed wing) or heading |
| Gyro X/Y scale and misalignment | no | **yes**, rotations between poses | weakly, from banked turns |
| Mag hard iron | yes (INS state and the fit) | yes | on the ground; in flight see "UAV" |
| Mag soft iron | mostly horizontal; the vertical terms are weak | **yes**: the pole is turned about every axis | see "UAV" |

The survey pole is the best calibration platform, and it also needs the result most. Tilt compensation turns heading error straight into position error: on a 2 m pole tilted 30°, the antenna is 1 m to one side of the tip, so 1° of heading error moves the tip 17 mm.

## Estimator: least squares with the current calibration as prior
Each term group is a small least-squares problem whose prior is the calibration in use (bench result, factory default or the last learned set), with its covariance. The solve gives a posterior mean and covariance. A parameter direction the data doesn't reach keeps its prior and its prior σ, so a tractor never "learns" an accelerometer scale it can't see, and no per-platform switch is needed. A new value counts as learned only where the posterior σ is below a threshold per term and much smaller than the prior σ.

**Static poses** (accelerometer, gyro bias, magnetometer):
- Stillness is detected on core 0 with a rolling variance of accelerometer and gyro, the same idea as `inslib_imu_calib.py`. INSLIB's own auto-ZUPT flag from core 1 is a cross-check.
- For each still period of at least about 2 s: means of accelerometer, gyro and magnetometer, their variances, temperature, duration, time.
- Poses are binned by gravity direction on a sphere grid (about 30 bins), keeping the best (longest, quietest) pose per bin, plus a bounded ring of recent poses for the temperature fit. Memory: about 30 × 64 B ≈ 2 KB (estimate).
- Accelerometer fit: imu_tk's cost `|g| − ‖M_a·(a − b_a)‖` over the poses, Gauss-Newton with the prior. With GNSS it can be done from a local gravity value; without GNSS, a standard value for the latitude is enough.
- Coverage check: the scatter matrix `Σ ĝ ĝᵀ` of the pose directions. Its smallest eigenvalue says how well the poses span the sphere, which is what the web page shows as "coverage".

**Rotations between poses** (gyro scale and misalignment):
- The imu_tk method re-integrates the raw gyro across each rotation for every candidate `M_g`. Keeping 800 Hz samples for that doesn't fit (a 5 s rotation is about 50 KB).
- Instead, core 0 pre-integrates each rotation at 800 Hz around the current `M_g`: the rotation `ΔR` and its 3×9 Jacobian with respect to `M_g`, the way IMU preintegration handles the bias (Forster et al., "On-Manifold Preintegration for Real-Time Visual-Inertial Odometry", IEEE T-RO 2017). Stored per rotation: `ΔR`, the Jacobian, the gyro bias from the pose before it, and the gravity directions of the two poses it joins. About 200 B each (estimate).
- The fit is first order around `M_g`. That is fine as long as the error is small (a factory-calibrated part). After an accepted update the stored rotations are linearized at the old value, so they are dropped and collection starts again.
- Cost on core 0: a few hundred flops per gyro sample while a rotation is being recorded (estimate, not measured).

**Turns with a GNSS heading** (gyro Z scale on tractor and UAV): integrate the yaw rate over a turn and compare it with the change in GNSS heading (dual-antenna UNIHEADING, or course over ground on a vehicle moving forward). Use only turns with RTK or a good heading σ, between two straight sections.

**Magnetometer**, two methods into the same parameters:
- **Ellipsoid fit**, no attitude needed: the samples must lie on a sphere of the WMM field strength. This is the method for the pole and for a UAV turned by hand on the ground. It fixes `M_m` only up to a rotation, so that rotation is taken from the prior (or from `inslib_frame_align.align_by_dip`, see `inslib_mag_calib.py`).
- **Reference fit**, when the attitude is known without the magnetometer: rotate the WMM field vector into the body frame with the INS attitude. Then `m_ref = M_m·(raw − b_m)` is linear in the 12 unknowns, so a recursive least squares at about 1 Hz is enough. It also fixes the rotation onto the IMU axes. Allowed only when the heading comes from GNSS (dual antenna, or course in INS mode FULL with a small yaw σ). INSLIB also fuses the magnetometer, but only weakly and at most once per second, so the attitude is close to independent of it. The fit takes the raw magnetometer, before INSLIB's online bias.
- **Ownership of the hard iron:** the calibrator owns `mag_fixed_bias`. The INS state `estimate_mag_bias` stays on for what changes during a run (a different implement, a tool in a pocket). It starts at zero after a new fixed bias is applied.
- Every magnetometer sample passes the field-strength gate first (|B| against WMM, as in INSLIB) and a gate on its rate of change.

**Temperature** (gyro and possibly accelerometer bias): a straight-line fit per axis of the bias at each standstill against the TM210 temperature, with the value at `T_ref` handled by the fixed bias. Core 0 subtracts the slope term from each sample before it goes into the core 1 queue. INSLIB's bias state then only has to follow what is left. This also answers the gyro bias vs temperature item in `TODO.md` on the unit itself, though the dedicated lab log is still the way to get a first value.

## Gates per platform
A platform setting chooses the gates: tractor, pole, multirotor or fixed-wing (user decision 2026-11-15). It is a cmake default (`-DPLATFORM=…`, default tractor) that the web page can override, stored in `settings_t` and applied at reboot, like the antenna mode.
- **Common:** samples only from a healthy IMU stream (no gaps, CRC good); poses only after the warm-up (the TM210 QoS reaches 5, [imu-tm210.md](imu-tm210.md)); reference fits only in INS mode FULL with RTK or a small attitude σ.
- **Tractor:** course over ground counts as a heading only above the course minimum speed, driving forward. Engine and hydraulics disturb the magnetometer, so the field gate stays on; log how often it rejects samples.
- **Survey pole:** no course heading (it is walked about, not driven); the INS heading comes from the planted-pole aiding instead ([pole-tilt-aiding.md](pole-tilt-aiding.md)). Poses and rotations are the main data. The pole is usually turned by hand, so the rotations are slow and smooth: good for the pre-integration.
- **UAV:** course counts as a heading only on a fixed wing in straight flight, never on a multirotor. Vibration from the motors makes in-flight "still" periods impossible, so poses come only from the ground. The magnetometer error from the motor current moves with the throttle, and calibration can't remove it without a current or throttle input, which the firmware doesn't have. So on a UAV the magnetometer learns only on the ground, with the motors stopped, until there is such an input.

## Accepting and storing a result
- **Never during operation.** A learned set goes into flash. INSLIB gets it at the next boot, or at a re-init that core 0 starts while the unit is still (the live-apply path of the web config, [web-config-server.md](web-config-server.md), "Phase 4"). On a UAV only while still on the ground for a set time, since the firmware has no arm signal.
- **Sanity check before saving:** with the new set, the residuals over the stored data must be smaller than with the old set, and every parameter must be inside a plausible range (for example scale 0.95–1.05, misalignment under 2°, bias under the datasheet limit). Otherwise the set is kept as "rejected" with the reason, for the web page.
- **Flash:** a calibration record in its own two 4 KB sectors, apart from `settings_t`, with the same scheme (magic, version, size, sequence number, CRC-32, written alternately so a power cut loses nothing). It holds the parameters with their covariances, the source of each (default, bench, learned), the TM210's node ID (a different IMU invalidates the record), the temperature model, and the pose bins, so a pole that is used for 10 minutes a day still adds up poses over weeks. Writes are rare (at most a few per day, only at standstill), so flash wear doesn't matter.
- **Order of trust:** a bench result from the offline tools is the prior and is never overwritten silently. A learned set replaces it only through the checks above, and "reset to bench / factory" is always available.

## Web page
Per term group: value, σ, source, number of poses / rotations / turns, sphere coverage, last update, and the reason for any rejection. Buttons: accept a pending set, reject it, reset to bench / factory, clear the collected data. Built in Phase 5 (below): a mode per group (auto or hold) instead of a separate "pending" state.

## Where it runs
Core 0, as its own module with queues to the rest ("Core split" in `CLAUDE.md`). It takes the raw samples core 0 already parses (before any calibration) and core 1's 100 ms outputs (attitude, mode, σ, ZUPT flag). Nothing is added to core 1. The solves are small (at most 12 parameters over about 30 poses or a few hundred magnetometer samples), so a few milliseconds each when there is new data (estimate). The only new per-sample work is the pre-integration during a rotation.

## Phases
1. **Gyro bias and its temperature model, plus persistence.** The calibration record in flash, the stillness detector, the bias-vs-temperature fit, applied at boot. Useful on every platform.
2. **Static poses and the accelerometer fit**, with the coverage measure. First real test on the pole.
3. **Magnetometer:** ellipsoid fit, then the reference fit with a GNSS heading.
4. **Gyro scale and misalignment:** pre-integrated rotations (pole), GNSS heading turns (gyro Z, tractor and UAV).
5. **Web page** with the accept / reject / reset actions.

### Phase 1 as built (2026-11-15, bench-tested on dietpi the same day)
Code: `firmware/app/imu_calib.c/.h` (core 0 module), `imu_calib_fit.c/.h` (the math, plain C, host test `firmware/tools/test_imu_calib_fit.c`), `flash_rec.c/.h` (the two-sector record store, now shared with `settings.c`). Constants are `APP_CALIB_*` in `firmware/app/app_config.h`.
- **Decisions (user, 2026-11-15):**
  - A learned set is **applied automatically at the next boot** after the range check. The web page has a "Reset IMU calibration" action (`calib_reset`) that erases it; the correction in use stays until the next boot.
  - Earth rotation is removed using the **last GNSS latitude, else a build default** (60° N). An error of 5° costs about 6e-6 rad/s.
- **Model:** `b(T) = b0 + k·(T − 25 °C)` per axis.
  - `b0` goes to INSLIB as `imu_gyr_fixed_bias` (`nav_params_t.gyr_fixed_bias`). `nav_suite` applies it to every sub-filter.
  - Core 0 subtracts `k·(T − T_ref)` from each sample, with the latest TM210 Status temperature.
- **Chunks:**
  - Built from 0.2 s windows that pass INSLIB's stillness thresholds. The first and last window of a still run are dropped.
  - A chunk lasts 10–60 s, so a long stop gives several points, and its temperature span must be ≤ 0.5 °C.
  - No QoS gate: the warm-up is the temperature swing this phase needs.
- **Earth rate:** the attitude must come from the INS or the AHRS, which know the yaw; with the ARS only, the chunk is skipped (`skip_att`). The Earth rate is subtracted in the body frame.
- **Fit:**
  - Bayesian least squares with a zero-mean prior: σ_b0 1 °/s (check against the TM210 datasheet), σ_k 0.004 °/s/°C (datasheet).
  - The weight per chunk is `gyr_psd / T + (0.002 °/s)²`.
  - Only the sufficient statistics are stored: a 2×2 information matrix and three right-hand sides.
  - `k` is fitted only once the stored chunks span at least 3 °C (`APP_CALIB_K_MIN_SPAN_C`). Below that, `k = 0` and `b0` is the prior-shrunk weighted mean bias. Added after the bench run below: over a 0.16 °C span, bias drift in time was read as a slope.
  - The variance floor is 0.005 °/s per chunk (`APP_CALIB_FLOOR_DPS`), raised from 0.002 after the bench scatter below. It is a first figure from 3 chunks, not a measurement.
  - At each boot that applies a record, the information is scaled by 0.95, so turn-on repeatability and ageing don't freeze the fit.
- **Range check:** |b0| ≤ 2 °/s, |k| ≤ 0.02 °/s/°C. A record that fails is ignored at boot and learning starts anew.
- **Flash:**
  - Record "GSCL" in the two sectors below the settings (flash end − 16 KB).
  - The payload holds the statistics, the last latitude and the TM210 node ID.
  - Saved only at standstill, after at least 3 new chunks, at most every 10 min.
- **Identity:** the correction is applied at boot, before the TM210 is identified. Once it runs:
  - Another node ID: the statistics start anew and core 0 restarts the filter without the correction.
  - No Setting reply (`TM210_E_DEAF`): nothing is learned and nothing is applied.
- **Output:** a `CALIB` line in the USB status and a `"calib"` object in `/status.json` (in use, learned with σ, temperature range, chunk counts, skips). `WEB_DOC_MAX` went from 2048 to 3072 B for it.
- **Checked:**
  - Build, `-DWEB_MODE=off` build and `check_ram_path.py` (nothing reachable from core 1 or the ISRs in flash).
  - The host test on dietpi: recovery of `b0` and `k` within 4σ, `k` stays at its prior without a temperature span, forgetting, rejection of bad values, Earth rate in four attitudes.
- **Bench results (2026-11-15, Pico 2 W on dietpi, TM210 node 925, no GNSS, so the 60° N default latitude):**
  - Boot with no record: "no record". The TM210 was identified (`id ok`) and the unit counted as still at once. Core 1 timing was unchanged (`update` mean 117 µs, max 231 µs).
  - Attitude came from the AHRS (magnetometer), so no chunk was skipped. Three 60 s chunks at 19.24, 19.16 and 19.08 °C measured bias x −0.0027/−0.0031/−0.0060, y 0.0019/0.0052/0.0123, z 0.0014/0.0060/0.0084 °/s with the Earth rate removed. That is small, as expected from a TM210 that compensates temperature itself (`SW_TEMP_CALIB`). The y bias drifted 0.01 °/s in 3 minutes at a constant temperature.
  - The first save came after the third chunk: record 1 in 2 ms, with no queue drops and no ring overflows.
  - **Found:** with the first build, the fit read that drift as a temperature slope over the 0.16 °C span (k_y −0.0013 °/s/°C, held only by the prior). Extrapolated 10 °C away, that is 0.013 °/s wrong. Fixed with the 3 °C minimum span above.
  - After a reboot on the fixed build, record 1 re-solved to `b0` = (−0.0039, 0.0065, 0.0053) °/s, the mean of the chunks, with `k` = 0, "not fitted". Read over SWD with GDB, `s_suite.ins.opt.imu_gyr_fixed_bias` was (−6.85e-5, 1.13e-4, 9.23e-5) rad/s, the same values. INSLIB's init log line didn't reach USB (lost from the 8-message log queue at boot).
  - **Settings after the `flash_rec` refactor:** the record saved by the old code loaded (`source flash`, seq 1). Then save (seq 2), revert (erased, defaults) and save (seq 1) all worked over `POST /config`.
  - **`calib_reset`:** "erased; the one in use stays until the next boot", with record 0 in `/status.json`. After a reboot: "not applied", 0 chunks.
  - **Not checked here:** INSLIB's own gyro bias estimate with the correction applied (`bias_valid` needs the INS, so GNSS), and a temperature span. That needs the cold-start / fridge run from `TODO.md`, which also fits `k` for the first time.
  - **Note:** GDB's detach left core 0 halted ("target needs reset"); an OpenOCD `reset run` brought it back.

### Phase 2 as built (2026-11-15, bench-checked on dietpi with one pose)
Code: `firmware/app/imu_calib_acc.c/.h` (the fit, plain C, host test `firmware/tools/test_imu_calib_acc.c`), with the pose collection in `imu_calib.c`. Constants are `APP_CALIB_POSE_*`, `APP_CALIB_ACC_*` and `APP_CALIB_STILL_MAX_*` in `firmware/app/app_config.h`.
- **Model:** imu_tk's accelerometer cost, as in `inslib_imu_calib.py`. `a = M·(raw − b)`, M upper triangular (a rotation can't be seen from magnitudes), 9 parameters, fitted so that |a| equals the local normal gravity in every pose.
  - The local g is WGS84 Somigliana at the last GNSS latitude and height, else 60° N at 0 m.
  - The fit is Gauss-Newton with step halving and a prior on every parameter: scale σ 0.02, misalignment σ 0.5°, bias σ 0.1 m/s². The posterior σ comes from the inverse normal matrix.
  - The result goes to INSLIB as `imu_acc_misalignment` (column-major) and `imu_acc_fixed_bias` through `nav_params_t`, applied at the next boot like the gyro set.
- **Poses:** every still run is also a pose. It needs at least 3 s and is averaged over at most 30 s, from the same windows as the gyro chunks, with the first and last window dropped.
  - It goes into one of 30 bins by gravity direction: 6 cube faces × (a centre patch within 20° of the face axis, plus 4 quadrants around it).
  - A bin keeps the longer pose up to the 30 s cap. At the cap the newer one wins, so old poses get replaced over time.
  - The pose σ is `√(acc_psd / T + (0.01 m/s²)²)`. The floor allows for the TM210's bias settling (`plans/imu-tm210.md`).
  - Poses count towards the same save rule as the gyro chunks. The record is version 2 (pose bins added; a version 1 record is ignored) and is about 0.8 KB, so `flash_rec` now writes records of up to 4 pages.
- **Coverage:** the smallest eigenvalue of the mean of `u·uᵀ` over the pose directions. 1/3 means spread over the sphere; 0 means all in one plane. It is shown with the pose count.
- **Range check:** |scale − 1| ≤ 0.05, misalignment ≤ 2°, bias ≤ 0.5 m/s². A stored pose set that fails at boot is cleared and learning starts anew.
- **Fix for Phase 1 found while building this:** the stillness test was variance-only, so a steady turn passed (constant rate, constant centripetal acceleration). That would have given a gyro "bias" equal to the turn rate and a pose that isn't gravity. A window now also needs a mean rate ≤ 0.5 °/s and | |a| − 9.81 | ≤ 0.5 m/s² (raw), counted as `windows_not_rest`.
- **Output:** a `CALIB acc` line in the USB status (applied, poses of 30, coverage, the in-use and learned sets with σ and the residual before → after) and a `"calib_acc"` object in `/status.json`.
- **Checked:**
  - Builds, normal and `-DWEB_MODE=off`, with no warnings. `check_ram_path.py` finds nothing in flash once the fit's Cholesky routine had its own name: the checker matches functions by name, and a second `cholesky` looked like INSLIB's.
  - Host test on dietpi: 24 random poses recover all 9 parameters within 4σ (residual 0.13 → 0.002 m/s²); level-only poses fix the magnitude and leave the unseen terms at their prior; correct prior, rejection, column-major layout, normal gravity and coverage values.
- **Bench results (2026-11-15, Pico 2 W on dietpi, TM210 node 925 lying still about 9° off level, no GNSS fix, so 60° N and 0 m):**
  - The first pose was stored after 30 s, in bin 25 (the −z face, centre patch): raw acc (1.1521, −0.9353, −9.6897) m/s², |a| 9.8027 against a local g of 9.8192.
  - The 1-pose fit behaves as the level-only host test predicts: z scale 1.0013 and z bias 0.0035 m/s² take up the 0.17 % magnitude gap (residual 0.0165 → 0.0000 m/s²), and every other term stays at its prior (σ unchanged, e.g. misalignment σ 0.5°). Coverage 0.000.
  - Two gyro chunks at 18.3 °C followed. The version 2 record (about 0.8 KB, 4 pages) saved as record 1 in 34 ms (an erase was needed), with no queue drops and no ring overflows. `/status.json` was 1855 B, well inside 3072.
  - After a reset, both sets were "applied". Over SWD, `s_suite.ins.opt.imu_acc_misalignment` was the learned upper-triangular matrix in column-major order (z scale 1.00134 at element 8), `imu_acc_fixed_bias` (−0.00041, 0.00033, 0.00347) m/s² and `imu_gyr_fixed_bias` (3.9e-5, 2.8e-5, 7.8e-5) rad/s, all as stored. A first read gave all zeros: it was taken early in a boot, before `nav_start()`.
  - **Not checked here:**
    - The `windows_not_rest` gate. It needs a steady turn (stayed 0 on the bench).
    - A multi-pose session: someone has to turn the unit through 12 or more orientations, as for `inslib_imu_calib.py`, and the result should be compared with that tool.
    - Whether the applied set lowers INSLIB's own accelerometer bias estimate (needs the INS, so GNSS).
  - **Side effect:** a `um980_bridge.py` run (started 09:03 by other work) had provisioned the Pico's UM980 through the Debug Probe UART. After this flash's reset, the bridge logged no more lines from the Pico, and the Pico reported "no receiver answered", while the bridge kept sending receiver frames. The bridge seems not to recover from a Pico reset in the middle of a run; restarting it is up to whoever runs it.

### Phase 3 as built (2026-11-15, host-tested; bench-checked without GNSS)
Code: `firmware/app/imu_calib_mag.c/.h` (both fits, plain C, host test `firmware/tools/test_imu_calib_mag.c`) and `imu_calib_lsq.c/.h` (the Gauss-Newton solver with a prior, now shared with the accelerometer fit), with the collection in `imu_calib.c`. Constants are `APP_CALIB_MAG_*` in `firmware/app/app_config.h`.
- **Decisions (user, 2026-11-15):**
  - Both fits in this phase.
  - The platform setting as above.
  - A fit only with a GNSS fix: without one there is no WMM field strength to scale to. The last fix is kept in the record, so a fix from an earlier boot counts.
- **Samples:** a group of 8 consecutive magnetometer samples (the TM210 gave about 176 per second on the bench, so about 45 ms per group), taken while the unit turns slower than 30 °/s, is one sample.
  - It also carries the down direction (from the accelerometer) when |a| stays within 0.3 m/s² of g.
  - Samples go into 54 bins (6 cube faces × 3 × 3) by raw direction, and the newest wins.
  - Once a fit exists, a sample more than 15 % off the field strength is dropped (steel nearby, a motor).
- **Ellipsoid fit:** symmetric M plus centre, 9 parameters, `|M(raw − c)| = F` (WMM total field at the stored position). The prior is M ≈ I (the raw unit) with σ 0.3 on the diagonal and 0.1 off it, and the centre with σ 100 µT.
  - **Dip constraint (found while testing):** with magnitudes alone, samples from turns on level ground (a ring) lie on a whole family of spheres. In the host test the heading came out wrong by up to 180°. Each sample with a down direction therefore adds a residual `down · M(raw − c) = F sin I` (WMM inclination), as in `inslib_frame_align.align_by_dip`. On the ring that leaves about 1° of heading error, from xz/yz soft iron coupling with the unseen z centre and scale; on full-sphere data the fit is exact to the noise.
  - **First guess:** the algebraic sphere fit is used only when the samples surround its centre (coverage ≥ 0.1). For a ring it returns a small sphere through the ring, far off in z, so the sample mean is used instead.
- **Reference fit:** `m_ref = M·raw + d`, 12 unknowns, linear, with the ellipsoid result as its prior (σ 0.05 on each M element allows about 3° of rotation, σ 5 µT on the hard iron).
  - The sufficient statistics (one 4×4 matrix, three right-hand sides) are stored and scaled by 0.95 per boot, as for the gyro.
  - A sample is taken once per second when:
    - the INS has been in FULL for 60 s, with the attitude from the INS;
    - the unit turns slower than 5 °/s;
    - a magnetometer sample is under 50 ms old;
    - the INS yaw comes from a GNSS heading: the dual-antenna heading within the last 2 s on any platform, or the course over ground on a tractor (forward, 3 m/s or more) or a fixed wing (straight flight, |yaw rate| ≤ 2 °/s). Never on a pole or a multirotor.
  - The true field is the WMM NED vector rotated into the body frame.
  - With no reference data the result is the ellipsoid's. In the host test, 180 reference samples on the level ring bring the heading error under 0.3°, including a 2° mounting offset that the ellipsoid alone can't see.
- **Applied at boot** like the other sets: INSLIB's `mag_misalignment` (column-major) and `mag_fixed_bias`, through `nav_params_t`.
  - Range check: M diagonal 0.5–2, off-diagonal ≤ 0.3 of the mean diagonal, hard iron ≤ 200 µT, ellipsoid residual ≤ 5 % of F.
  - The INS keeps estimating a hard-iron bias on top (`estimate_mag_bias`), starting from zero.
- **Storage:** record version 3 (magnetometer bins, reference statistics, and the fix's longitude and epoch for the WMM), about 2.6 KB. `flash_rec` now allows up to a whole 4 KB sector.
- **Output:** a `CALIB mag` status line (samples of 54, outliers, coverage, reference samples and what gives the INS yaw, waiting for a fix) and `"calib_mag"` in `/status.json`.
- **Also fixed:** the gyro set reported "applied" from a record with no chunks (the prior's zeros); it now says "no chunks".
- **Checked:**
  - Builds (normal; `-DWEB_MODE=off -DPLATFORM=pole`); a bad `-DPLATFORM` stops cmake with a message. `check_ram_path.py` finds nothing in flash.
  - Host tests: magnetometer (full-sphere recovery of centre and symmetric M; the reference fit recovers the full M including a 2° rotation; the level ring with dip residual and reference; empty, rejection, layout), accelerometer and gyro (unchanged after the solver refactor).
  - Bench on dietpi, without GNSS: the version 3 record (11 pages) saved and loaded. Samples are collected; the fit waits for a fix ("no GNSS fix yet (no WMM field)"). `/status.json` is 1770 B.
  - Platform over `POST /config`: the old settings record reads tractor (the build default); saving pole shows `reboot_pending: ["platform"]`; a bad value is refused; back to tractor clears it.
- **Not checked on hardware:** any magnetometer fit (needs a GNSS fix), samples from many orientations, the reference fit (needs a GNSS heading), and the outlier gate. The record version bump also dropped the bench's Phase 2 record (25 gyro chunks, 1 pose).

### Phase 4 as built (2026-11-15, host-tested; bench-checked without motion)
Code: `firmware/app/imu_calib_rot.c/.h` (preintegration and the linear fit, plain C, host test `firmware/tools/test_imu_calib_rot.c`), with the collection in `imu_calib.c`. Constants are `APP_CALIB_ROT_*`, `APP_CALIB_TURN_*` and `APP_CALIB_GYR_*` in `firmware/app/app_config.h`.
- **Model:** `w = M·(raw − b)`, `b` the Phase 1 bias, `M = M0 + dM`, with M0 the matrix in use and dM (9 values) the unknown. Both observation kinds are linear in dM after linearising at M0, so the record keeps one 9×9 normal matrix and one right-hand side (`rot_stats_t`), not the rotations themselves.
- **Rotations between still poses (pole):**
  - Core 0 integrates every gyro sample with M0 and the bias in use, carrying the rotation and its 3×9 Jacobian with respect to dM (right perturbation, as IMU preintegration carries the bias Jacobian).
  - The integration restarts at every still window, so when motion starts it already holds the samples of the window that detected it.
  - It is armed when a still run of at least 3 s ends (that run's mean gravity direction is g1), and frozen when the next still run starts. That run's mean gravity direction (at its end, or after 30 s) is g2.
  - The residual is `dRᵀ·g1 − g2`, with gradient `[v]×·J` (σ 0.003 per component).
  - A rotation must turn by at least 30° within 20 s.
- **Turns with a GNSS heading (tractor, fixed wing, any platform with dual antenna):**
  - **Segments:** a segment starts at a GNSS heading epoch where the body yaw rate is below 2 °/s. The yaw rate `(sin φ·w_y + cos φ·w_z)/cos θ` plus the Earth's vertical rate is integrated with the INS roll and pitch, together with its gradient with respect to dM.
  - **End:** the segment ends at the next straight epoch once it has turned 90° or more. A small heading change restarts it, and so does a gap of more than 2 s or a segment longer than 90 s.
  - **Residual:** integrated minus measured change (unwrapped by the prediction), σ from both heading sigmas plus 0.002 rad.
  - **Heading sources:** the dual-antenna heading (gated as for INSLIB), or the course over ground at 3 m/s or more, which counts only on a tractor driving forward (within about 29° of the INS yaw) or a fixed wing.
- **Applied at boot:** the stored M0, plus every element whose posterior σ is below 0.001 (diagonal) or 0.05° (off-diagonal). Prior: σ 0.02 on the diagonal and 0.5° off it. A changed matrix re-linearises, so the statistics start anew and the new M0 is saved.
  - A tractor therefore learns its z scale and nothing else.
  - Range check: scale within 3 %, misalignment ≤ 2°.
  - The matrix goes to INSLIB as `imu_gyr_misalignment`.
- **Storage:** record version 4 (gyro matrix and its statistics added, about 3.1 KB).
- **Output:** a `CALIB gyrM` status line (rotations, turns, the tracker state, the learned dM with σ and how many elements the next boot would change) and `"calib_gyr_m"` in `/status.json`.
- **Checked:**
  - Builds with no warnings, `-DWEB_MODE=off` too; `check_ram_path.py` finds nothing in flash.
  - Host tests:
    - The preintegrated angle is within 2 % of the truth.
    - 40 random rotations at 800 Hz recover all 9 elements of a matrix with 1 % scale errors and about 0.3° misalignment, within 4σ (σ below 0.002).
    - 20 level turns recover only the z scale (σ below 0.0005); just that element is accepted.
    - Empty statistics, rejection and bad input.
    - The other three host tests still pass.
  - **Bench on dietpi (unit still, single antenna, GNSS through the bridge):**
    - The version 4 record started anew, saved in 40 ms and loaded after a reset ("kept", the identity).
    - No IMU queue drops and no ring overflows with the preintegration running on core 0 at 800 Hz. Core 1 timing unchanged (127 µs mean).
    - `/status.json` 2228 B.
- **Not checked on hardware:** any rotation or turn. The bench unit doesn't move, and with a single antenna at rest the INS stays in ATTITUDE_ONLY. It needs a pole-style session turning the unit between still poses, compared with `inslib_imu_calib.py`, and turns on a vehicle with a GNSS heading. The record version bump dropped the bench's Phase 3 record.

### Phase 5 as built (2026-11-15, bench-tested on dietpi the same day)
Code: `imu_calib_action()` and the per-group status in `firmware/app/imu_calib.c/.h`, `/calib.json` (`publish_calib_json()` in `firmware/app/main.c`), the page `firmware/app/web/www/calib.html`, and the `calib` action in `firmware/app/web/config_web.c`.
- **Groups:** gyro bias vs temperature (`gyr`), accelerometer (`acc`), magnetometer (`mag`), gyro scale and misalignment (`gyr_m`).
- **Mode per group** (stored in the record):
  - **auto** (the default, the 2026-11-15 decision): each boot applies the fit of the data after the range checks.
  - **hold:** each boot applies the stored set; the learning goes on, and a new set is used only when accepted.
- **Stored set:** the set in use of each group is kept in the record. In auto mode a group with no usable data (none collected, cleared, or out of range) falls back to it at boot instead of to no correction. A stored set that fails the range check at boot is dropped. For the gyro matrix the stored set is the matrix `M0` the statistics are linearised at.
- **Actions** (`POST /config` with `action=calib&group=gyr|acc|mag|gyr_m&op=…`; each saves the record at once):
  - **accept:** the learned set is used now and stored. Only while still; the filter restarts with it (`nav_set_params()`). Refused with the reason when there is nothing usable (no data, range check failed, no element of the gyro matrix pinned down). A new gyro bias or matrix restarts the rotation and turn trackers; a new matrix also starts its statistics anew.
  - **hold / auto:** set the mode; the correction in use doesn't change.
  - **clear:** erase the group's collected data (the "reject"); the set in use and the stored set stay.
  - **factory:** data and stored set erased, mode auto, no correction from now. Only while still; the filter restarts. `group=all` (also `action=calib_reset`, the config page's button) does it for every group.
- **Output:** `/calib.json` holds every group in detail (mode, in use, stored, what happened last, what the next boot will use, why accept is refused, the in-use and learned sets with σ, the data counts). `/status.json` keeps only a summary per group (it went from 2228 to 1696 B); the USB `CALIB` lines are unchanged. The page shows a card per group with the four buttons; accept and factory are disabled while the unit moves.
- **Storage:** record version 5 (the hold flags and the three stored sets added). The version bump dropped the bench's version 4 record.
- **Checked on the bench (2026-11-15, Pico 2 W on dietpi, unit still, single antenna, GNSS through the bridge):**
  - Builds with no warnings, normal and `-DWEB_MODE=off`; `check_ram_path.py` finds nothing in flash.
  - `/calib.json` 1456 B; the page loads (7.7 KB) and refreshes every 2 s.
  - Refusals: accept with no data names the reason; `group=all` with another op than factory is refused by the POST handler.
  - Accelerometer: accept of the one-pose set put it in use at once (INSLIB re-initialised); hold, clear, reboot gave "held set applied"; auto, clear, reboot gave "no usable data: the stored set is applied"; factory dropped it at once.
  - Gyro: accept of the first 60 s chunk put b0 in use at once; the factory reset of every group then left no correction.
- **Not checked:** an accept or factory refused while moving (the bench unit doesn't move), and accepting a magnetometer or gyro matrix set (none was fitted on the bench).

## Verification
1. **Replay first:** INSLIB's simulated datasets (`external/INSLIB/datasets/simulated/`) and replay tool, with known errors injected into the raw samples (a 1 % scale, 1° misalignment, a soft-iron matrix). The calibrator must recover them on pole-like motion and must leave the unobservable terms at their prior on tractor-like motion.
2. **Against the offline tools:** the same session through the calibrator and through `inslib_imu_calib.py` / `inslib_mag_calib.py`. Results within the posterior σ.
3. **Field:** on the pole, a set of points held at several tilts, before and after a learned set, against a surveyed point. On the tractor: heading after headland turns and standstill drift. On the UAV: the ground magnetometer calibration only.
4. **Bad data:** steel nearby, a running engine, a dropped IMU: the gates reject it and no set is saved.

## Open decisions
- **Platform setting:** decided 2026-11-15 (tractor, pole, multirotor, fixed-wing; cmake default with a web override, at reboot). It gates the calibration and, since 2026-11-17, INSLIB's `automotive_mode`: on the tractor only (single antenna), not on a fixed wing, whose course differs from its heading in a crosswind. The calibration still takes the course as a heading on a fixed wing.
- **Applying a learned set:** decided 2026-11-15: automatically at the next boot after the range checks, with a reset to clear it. Phase 5 adds a hold mode per group that waits for an accept instead.
- **Bench calibration still required?** On the pole, the background fit may make the bench multi-pose run unnecessary. On the tractor and UAV it can't replace it.
- **UAV magnetometer in flight:** needs a motor current or throttle input to compensate for. Not available today.
