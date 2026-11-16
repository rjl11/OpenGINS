# Non-holonomic and standstill aiding without GNSS

## Context

The question was what can improve the navigation when GNSS is poor or missing, without adding sensors. Two aids cover most of it, and both are about what constrains drift during a dropout:

- **Non-holonomic constraint (NHC):** a wheeled vehicle does not travel sideways or up/down relative to its body, so the body-frame lateral and vertical velocity should be near zero. Without GNSS, the lateral channel is otherwise unobserved, and a roll error leaks gravity into it.
- **Zero-velocity update (ZUPT / ZARU):** when the platform is standing still, velocity is zero and the gyro bias can be re-estimated.

Checked against the code on 2026-11-16, both already exist in INSLIB, so this plan is about turning them on, measuring them and closing the gaps, not about writing new filter code:

| Aid | INSLIB | Firmware now |
|---|---|---|
| NHC, lateral row | `ins_fuse_lateral_constraint()` in `external/INSLIB/src/ins.c`, option `automotive_lateral_constraint` (REQ-NAV-077) in `ins.h` | Not set in `firmware/app/nav_core1.c`, so it is **off**. It needs `automotive_mode`, which the firmware sets on a tractor (`course_yaw`, single antenna) |
| NHC, vertical row | Deliberately not implemented (REQ-NAV-077) | n/a |
| Auto ZUPT / ZARU (stillness detector) | `auto_zupt_disable` = false by default, `ins_auto_zupt_active()`, `nav_suite_get_zaru_active()` | On. Static thresholds from `APP_STILL_GYR_STDDEV_DPS` in `firmware/app/app_config.h` (`nav_core1.c:110`) |
| Explicit zero-velocity from a stopped wheel | Input exists as `zero_velocity_update` in `ins_measurements_t` | Not wired. Listed in [wheel-odometry-aiding.md](wheel-odometry-aiding.md) |

INSLIB's own notes say the NHC "needs a mounting calibration first", and that a body-velocity measurement is the same as the NHC with all three axes measured ([optical-flow-aiding.md](optical-flow-aiding.md), step 2). The rotation calibration in [imu-auto-calibration.md](imu-auto-calibration.md) is meant to provide that mounting calibration, so the NHC depends on it being valid for the tractor.

Platforms: the NHC applies to the tractor (and any wheeled car-like platform). It does not apply to a survey pole, a multirotor or a fixed wing; for those, ZUPT (when stopped) is the only one of the two that applies.

## Goal

During a 60 s GNSS dropout on a tractor, reduce the position and heading drift compared with the current build. Gains are not measured yet; this plan defines how to measure them.

## Steps

1. **Check the prerequisite.** On the tractor build, confirm on `calib.html` that the rotation (mounting) calibration is valid and applied at boot. Without it, do not enable the NHC.
2. **Make the NHC switchable.** Add a build default (`APP_NHC_ENABLE`, default off in the first commit) and a web setting appended to `settings.c`, applied live the same way as the other sensor switches. The setting is off until step 4 passes.
3. **Replay with simulated dropouts.** Use `firmware/tools/field_log.py` on recorded tractor logs (they already hold GNSS and the INSLIB output). Mask GNSS for 60 s windows and run the filter with the NHC off, then on. Compare position error against the masked-away GNSS truth, and the heading error, over at least three drives, both directions. Include straight runs, headland turns and a circle.
4. **Tune the three INSLIB knobs** from the replay, not by guess: `automotive_lateral_stddev_mps` (set above the residual's scatter, per the INSLIB note), `automotive_lateral_max_yaw_rate` (side slip breaks the assumption in fast turns) and `automotive_lateral_after_sec` (it should stay off while GNSS is live). Record the chosen values in the table below.
5. **Check ZUPT on standstill.** In the same logs, confirm the auto-ZUPT/ZARU flag arms during the standstills the driving program already includes, and does not arm while moving (the INSLIB notes warn about mid-flight arming, so check it on the log's speed). If it does not arm reliably, tune `APP_STILL_*` in `app_config.h`; do not add a new detector.
6. **Wire the wheel stop as a zero-velocity input.** When the wheel has no pulses for a set time and the IMU is still, send a zero-velocity measurement. This is the input already listed in [wheel-odometry-aiding.md](wheel-odometry-aiding.md). It is a separate change and waits for that plan's pulse input.
7. **Decide.** If the replay shows a drift gain in every dropout case, turn the NHC on by default for the tractor build. If the gain is marginal, keep it as an option and say so in the plan.

## Tuning record

| Setting | Value | Source |
|---|---|---|
| `automotive_lateral_stddev_mps` | not yet set | step 4 |
| `automotive_lateral_max_yaw_rate` | not yet set | step 4 |
| `automotive_lateral_after_sec` | not yet set | step 4 |

## Other sensors considered (not planned yet)

These came up in the same review. Each needs its own plan before work starts.

| Sensor | Platform | Why |
|---|---|---|
| Downward ToF or 1D lidar | UAV | Absolute height for the flow scale; the barometer drifts with weather |
| Tractor CAN bus (ISO 11783) | Tractor | Wheel and radar speed, steering angle from the tractor's ECUs. Steering angle is deliberately not fused ([wheel-odometry-aiding.md](wheel-odometry-aiding.md)); revisit only with a reason |
| Airspeed (pitot or differential pressure) | Fixed wing | Speed through the air, the main velocity aid without GNSS |
| UWB ranging with fixed anchors | Any | Local position where GNSS is blocked; needs site setup |
| Second, cheaper IMU | Any | Redundancy and a check on the TM210 |
| Stereo or visual-inertial odometry | UAV, tractor | Metric scale and landmark tracking on the existing OV9281 pair; more Pi CPU and calibration |

## Open questions

- Is the rotation calibration on the tractor good enough for the NHC? Replay will show it, but the mounting error needs a figure first.
- Does the tractor's log set include standstills of at least a minute (the driving program in [wheel-odometry-aiding.md](wheel-odometry-aiding.md) asks for them)?

## Related plans

- [inslib-nav-suite.md](inslib-nav-suite.md): the filter that hosts both aids
- [wheel-odometry-aiding.md](wheel-odometry-aiding.md): the body-velocity input and the wheel-stop zero-velocity input
- [optical-flow-aiding.md](optical-flow-aiding.md): the body-velocity input, step 2
- [imu-auto-calibration.md](imu-auto-calibration.md): the mounting calibration the NHC relies on
- [pole-tilt-aiding.md](pole-tilt-aiding.md): the planted-pole aid, a separate form of standstill aiding
