# Wheel and ground-speed aiding on a tractor (and why not the steering angle)

## Context
Optical flow ([optical-flow-aiding.md](optical-flow-aiding.md)) keeps the velocity through GNSS outages, but on a tractor the camera can be blinded by dust, mud on the lens, tall crop or darkness. Should the unit also read wheel speed and the steering angle and feed them to INSLIB? This plan covers what INSLIB takes today, the speed sources a tractor offers, how turns change the measurement, the steering angle, the input electronics, pins, timing, the auto-learning, the field tests (live on the tractor plus a USB log, with optical flow logged alongside when the camera is fitted), and the steps. It applies to tractors and other wheeled vehicles only; on the survey pole and the UAV it is off. Every accuracy figure here is an estimate, not yet measured.

Related: [inslib-nav-suite.md](inslib-nav-suite.md) (options and epochs), [imu-auto-calibration.md](imu-auto-calibration.md) (background fits, platform setting), [rp2350-feasibility.md](rp2350-feasibility.md) (ports and pins), [ethernet-w5500-agopenweb.md](ethernet-w5500-agopenweb.md) (AgOpenGPS output, on hold).

## Verdict
- **Wheel or ground speed: yes.** It is cheap, INSLIB takes it today, and it fails differently from flow: dust, mud, crop and darkness don't affect it, while flow doesn't suffer from wheel slip and sees sideways slip. The two complement each other.
- **Steering angle: not fused.** See "Steering angle" below.

## What INSLIB has today
`external/INSLIB/src/ins.h`, `ins_measurements_t` (passed unchanged through `nav_suite_update()`):
- `speed` (REQ-NAV-068, `ins_meas_speed_t`): scalar ground speed, `h(x) = ||v_n||` at the IMU point, unsigned, with `speed_delay_ms` / `speed_delay_us`. Options `speed_scale`, `speed_stddev_rel`, `speed_min_mps`. The header names wheel odometry as a use. **No lever arm.**
- Non-holonomic lateral constraint (REQ-NAV-077, `automotive_lateral_constraint`, `automotive_lateral_stddev_mps`, `automotive_lateral_max_yaw_rate`): body-frame lateral velocity at the IMU fused against zero, held off above a yaw rate. **No lever arm either.**
- Zero-velocity input: a stopped wheel (no pulses for a set time) plus a still IMU is a strong standstill cue.

Measured forward speed plus the lateral constraint is the usual odometer-aided INS: forward measured, sideways held at zero, so the velocity stays bounded in an outage and the position drifts with distance instead of with time squared.

## Speed sources on a tractor
1. **ISO 11786 signal socket** (7-pin, fitted to most tractors since the 1990s): ground speed from the tractor's radar and wheel speed, each as a pulse train of nominally **130 pulses per metre**, plus PTO speed and hitch position. Radar ground speed doesn't slip and works in dust (radar is fine in dust; mud on the sensor face is the risk). First choice where it exists: the radar signal (or the wheel signal if no radar is fitted) goes to input A.
2. **ISOBUS / J1939 CAN**: ground-based speed (PGN 65097) and wheel-based speed (PGN 65096), both with direction. Needs a CAN controller: MCP2515 on SPI1, or a PIO CAN (can2040, if it runs on the RP2350). Later option; SPI0 stays reserved for the W5500.
3. **Own sensor on a rear wheel** (or the gearbox output), on any tractor: either a **coarse Hall pickup** on a tooth ring or a few magnets (input A only; no direction, few pulses at low speed), or a **quadrature encoder** (inputs A and B; gives the direction and more pulses per metre). Driven wheels slip.

The Pico has **two speed inputs, A and B** (pins below). Either one pulse signal (ISO 11786 or a Hall pickup) on A, or a quadrature encoder on A and B. Pulses per metre is the figure shown everywhere (status report, web page), as is usual on agricultural machines; internally the fit uses metres per pulse.

Weak points of wheel speed: slip of driven wheels (a few % on firm ground, 10–20 % in tillage, estimate), tyre rolling radius with pressure, load and wear, and few pulses at low speed. Hence the preference for radar, and the scale learning below.

## First sensor: a coarse Hall pickup on the right rear wheel (decided 2026-11-16)
The first drives use one coarse Hall pickup on the **right rear wheel**, on input A (GP12); input B stays unused. It is mounted on a bracket on the axle housing (not turning) and sees a few targets on the wheel or hub: magnets, or the wheel nuts. Example: 10 targets on a rear tyre of 1.7 m diameter (5.3 m round) give about **1.9 pulses per metre** (estimate; the learning measures it). ISO 11786, ISOBUS and an encoder stay later options. What this choice means for the rest of the plan:

- **Unsigned, no direction.** The sign comes from the filter's forward velocity (from `u` while learning); near zero speed, or while the sign is unsure, the sample is skipped. Reversing gives no wheel samples.
- **Right wheel: `y_w > 0`, so it is the inner wheel in right turns** (ω_z > 0). The tight-turn skip below only ever fires in right turns; in left turns it is the outer wheel and always usable. The learned `c` should come out positive, about half the rear track if the IMU sits near the centreline: a sign check on the fit and the axes.
- **One gap at a time, not a 10 Hz window.** At 2 m/s and 1.9 pulses/m there are about 4 rising edges a second, so a fixed 100 ms window is mostly empty. Each speed sample spans whole gaps between **rising edges** (same edge type only: the high time depends on target width and air gap, not on distance): the shortest run of gaps of at least 0.25 s, sent when its closing edge arrives, at most 10 Hz. Speed = gaps × metres per pulse / (last − first edge time); time of validity the middle of the run. The edge times are exact to µs, so one gap already gives a precise mean speed; the coarseness costs only update rate and delay.
- **The delay limit sets a minimum speed.** INSLIB keeps 500 ms of history (`INS_MAX_DELAY_MS` in `external/INSLIB/src/ins.h`). A one-gap sample is half a gap old when its closing edge comes, plus the queue latency, so it must hold `1 / (P · v) ≲ 0.9 s`: `v ≳ 1.1 / P`, about 0.6 m/s at P = 1.9. That is below INSLIB's `speed_min_mps` default of 1 m/s, so it doesn't bind; with fewer than about 1 pulse per metre it would, and then the history (`INS_HISTORY_ITEMS_MAX`) would have to grow.
- **Turn-rate correction over the gap.** Core 0 keeps a running sum of the bias-corrected body z rate at 100 Hz for the last 2 s, so the mean ω_z over any gap run is a difference of two sums (the same holds for the learning below).
- **Learning per gap.** Instead of rates in a window, use distances per gap: the forward distance at the IMU over one gap is `k + c · Δθ`, where Δθ is the yaw change over that gap (the gyro sum) and the distance is the GNSS forward velocity at the IMU integrated over the gap from the 20 Hz epochs. Same unknowns (k, c), the same 2×2 least squares, and no timing error from the window edges.
- **Uneven target spacing.** Hand-fitted magnets are not evenly spaced; a 1 cm error on a 0.5 m gap is 2 % on that one gap (estimate), repeating once per revolution. Wheel nuts are machined and should be even. First look at the gap pattern in the log (ratio of each gap's GNSS distance to the mean, against the gap number modulo the target count). If the spread is over about 1 %, learn a table of gap lengths: the target count N is the period of that pattern, the gap number is counted modulo N and re-locked by matching the pattern after a standstill or lost edges. Until then, the spread goes into `speed_stddev_rel`.
- **Standstill.** "No pulse for a set time" only says the wheel moved less than one gap (about 0.5 m), so the wheel alone is not a zero-velocity cue: set the timeout from the learned k (no edge for as long as one gap takes at 0.3 m/s) and use it only together with the IMU still detector.
- **The raw status figure is edge-timed (built 2026-11-16).** It used to count input A's rising edges in a 1 s window over the mean GNSS speed, which the ±1 count makes 20–50 % noisy at a few pulses a second. Now `firmware/app/speed_ppm.c` divides the pulses between two rising edges by the GNSS distance travelled between those edge times (speed integrated over the epochs, interpolated at the edges), over runs of at least 5 s and 10 pulses with every epoch at 1 m/s or more (`APP_SPEED_PPM_*`); lost edges or a slow or missing epoch start the run over. Host test `firmware/tools/test_speed_ppm.c` (synthetic: 1.9 pulses/m steady and varying speed, 130 pulses/m, a slow second), not yet on a tractor. That is the figure step 3 checks on the first drive.

## Turns: each wheel runs on its own radius
In a turn with yaw rate ω, a point at lateral offset y from the rear-axle centre moves forward at `v_axle − ω·y`. The inner and outer rear wheels differ by `ω·track`, and the front wheels (Ackermann) run on larger radii than the rear. So one wheel's speed is not the speed at the IMU, and since `speed` has no lever arm, the correction is done on core 0 before the sample is queued.

The lateral constraint has the same gap: zero sideways velocity holds at the rear-axle centre, but an IMU a distance d ahead of the rear axle moves sideways at `ω·d` in a turn. Today it is only gated by `automotive_lateral_max_yaw_rate`.

### Baseline: one rear wheel plus the IMU turn rate
One speed sensor on one rear wheel (Hall pickup, quadrature encoder, or the ISO 11786 wheel signal), and the turn rate from the TM210 gyro, give the speed at the rear-axle centre.

Body frame forward-right-down (x forward, y right, z down), so ω_z > 0 is a right turn. On a rigid body `v_P = v_axle + ω × r_P`; for a wheel at lateral offset `y_w` from the axle centre (right positive, about half the rear track):

    v_wheel = v_axle − ω_z · y_w     →     v_axle = v_wheel + ω_z · y_w

Check: in a right turn the right wheel (y_w > 0) is the inner one and runs slower, and the formula adds the difference back.

- **Which turn rate.** The gyro rate rotated into the vehicle frame by the mounting rotation, z component, bias corrected (INSLIB's gyro bias estimate from the 100 ms publication, or the calibration record before that). Use the vehicle's own z axis, not the navigation vertical: the wheel geometry is fixed in the vehicle, so on a side slope the body-z rate is the right one. Where on the vehicle the IMU sits doesn't matter for ω (one rate for a rigid body).
- **Same window.** Average the 800 Hz gyro z samples over the same interval as the pulse window (first to last edge). Core 0 has the TM210 samples already, so this is a running sum, nothing on core 1.
- **Size.** A headland turn at 2 m/s on a 6 m radius is ω_z ≈ 0.33 rad/s; with y_w = 0.9 m the wheel is off by 0.3 m/s, 15 % (estimate). Uncorrected, that would look like slip and pull the velocity in every turn.
- **Error budget.** Gyro bias is negligible (0.01 °/s × 0.9 m ≈ 0.2 mm/s). An error of 5 cm in `y_w` gives 1.7 cm/s at that turn rate. Slip of the measured wheel in the turn remains and is covered by `speed_stddev_rel`. `y_w` is not a setting: it is learned from turns under RTK (see "Auto-learning").
- **Direction.** The formula needs a signed speed. A quadrature encoder gives it; ISO 11786 and a single Hall pickup don't, so then take the sign of the filter's forward velocity. Around zero speed, or while the sign is unsure, skip the sample.
- **Tight turns.** When the turn radius at the axle centre falls below about `y_w` (|ω_z · y_w| ≥ |v_axle|), the inner wheel stops or reverses and the pulse rate is no longer a speed; skip the sample if the measured wheel is the inner one and `|ω_z · y_w| > 0.5 · |v_wheel + ω_z · y_w|`.
- **What one wheel can't do:** no slip flag of its own. Slip is seen as a residual against GNSS while it is there, and INSLIB's speed gate rejects gross outliers in an outage. (Sensors on both rear wheels would give one, but would take both inputs, leaving none for a quadrature channel; not planned.)
- **Radar** (ISO 11786) is corrected the same way, with the radar's lateral offset; its mounting angle is absorbed in its scale.

### Then
1. **Move it to the IMU point:** body-frame horizontal velocity at the IMU is `[v_axle, ω_z·d]` (d = IMU ahead of the rear axle, learned, see "Auto-learning"). In practice the learned model goes straight from pulses to the IMU point without the axle centre. INSLIB step 1 fuses its magnitude in `m.speed`, and the lateral constraint stays gated to gentle turns.
2. **INSLIB step 2: the body-velocity input** planned in [optical-flow-aiding.md](optical-flow-aiding.md) ("INSLIB integration", with `body_vel_leverarm_b`). Fuse `vel_b = [v_axle, 0, (unused)]` with the lever arm at the rear-axle centre and a small lateral variance. That one input carries the wheel odometry and a correctly placed non-holonomic constraint, and flow uses the same input with its own lever arm. A requirement for that INSLIB change: a lever arm, axes switched off by variance, and use in place of REQ-NAV-077 when present.

Sensors on the front wheels would also need each wheel's steering angle to project their speed onto the body axes. Rear wheels don't, so put the sensors on the rear axle.

## Steering angle
Not fused:
- With one rear-wheel speed and the gyro turn rate, nothing in a turn needs it.
- What it would add is yaw rate, `v·tan(δ)/L`, which the gyro measures far better. Tyre slip, articulated tractors and a pivoting front axle make the model poor.
- Heading drift in a 10–60 s outage comes from gyro bias, which the angle doesn't observe.

It belongs to autosteer: AgOpenGPS reads a wheel-angle sensor (WAS) for its steering controller. If autosteer is added, read the WAS on a free ADC input (GP26–28; never GP29 / ADC 3, the CYW43439 shares it) and send it to AgOpenGPS. In the filter it could at most hold the lateral constraint off in tight turns, which the yaw-rate gate already does.

## Input electronics
Tractor signals are 12 V pulse outputs (ISO 11786 is open collector or push-pull into a 12 V system, check per tractor). Each channel: series resistor, reverse-polarity diode and TVS into an optocoupler, its output with a pull-up to 3.3 V on the GPIO. Hall pickups on own tooth rings: 5–12 V supply, open-collector output into the same stage. Never wire a tractor signal straight to a Pico pin.

## Pins
Free after the flow plan (GP8, GP10, GP11): **GP12** = input A (Hall pickup, ISO 11786 radar or wheel signal, or encoder channel A), **GP13** = input B (encoder channel B; unused with a single pulse signal). Both have the internal pull-up on (open-collector outputs). GP16–21 stay reserved for the W5500, GP22 is the web switch. Like every pin, they go in `firmware/app/board_pins.h` and the table in [rp2350-feasibility.md](rp2350-feasibility.md).

## Timing and the core 0 module
- **Built (2026-11-15):** `firmware/drivers/pulse_in` timestamps every edge, rising and falling, on both inputs with `time_us_64()` in a GPIO interrupt in SRAM, with the level of both inputs after the edge, into a 512-edge ring (0.1 s at 5 kHz). GPIO edge interrupts suffice at a few kHz; a PIO counter is the fallback if a fine encoder needs more.
- Speed over a window: counted pulses over the time between the first and last edge in the window (not the window length), so the result is exact at any rate. At low speed the window stretches to the last few edges; with no edge for a set time the speed is zero.
- Time of validity: the middle of the window. Its age goes into `speed_delay_us`.
- Rate into INSLIB: 10 Hz is plenty (the error is a slowly changing scale, not white noise, as with flow).
- On core 0, `firmware/app/field_io.c` takes the edges today (status figures, the log); the learning and the speed sample for INSLIB will go next to it, queued to core 1 like the other measurements. Nothing new on core 1.

## Auto-learning: the user only wires the pulses
The user connects a pulse signal to the input and sets nothing: no pulses per metre, no tyre size, no wheel offset, no IMU-to-axle distance, not even which wheel or whether it is radar. While RTK is working, core 0 learns how the pulse rate relates to the vehicle's motion, and from then on the wheel speed is fused, also through GNSS outages. It runs in the background like `imu_calib*.c`, on core 0 only.

### What is learned, and from what
The reference is the GNSS velocity, not INSLIB's, so the fit stays independent of the wheel speed once that is fused. Per GNSS epoch (20 Hz BESTNAVB): take the receiver velocity at the antenna, move it to the IMU point with the antenna lever arm (`v_imu = v_ant − ω × r_ant`), and rotate it into the vehicle frame with the filter's attitude. That gives the forward velocity `u` and the lateral velocity `w` at the IMU. The pulse rate `f` (pulses/s) over a window centred on the epoch's time of validity, and the gyro z rate `ω_z` averaged over the same window, come from the edge timestamps and the 800 Hz TM210 samples.

The forward component of the velocity of a rigid body changes across it only with the sideways offset (`ω × r`), so for a wheel the fore-aft position doesn't matter:

    u = k · f + c · ω_z        k: metres per pulse,  c: the wheel's lateral offset from the IMU (right positive)
    w = d · ω_z + ψ · u        d: distance of the IMU ahead of the point that doesn't slide sideways (rear axle)
                               ψ: small yaw misalignment between the IMU frame and the direction of travel

Both lines are linear in their unknowns, so they are plain least squares: accumulate the 2×2 normal equations for (k, c) and for (d, ψ), with priors, and solve with `lsq_spd_solve()` from `firmware/app/imu_calib_lsq.c`. Priors: k from the first few seconds of straight driving (`u / f`) with a wide sigma, c = 0 ± 1.5 m, d = 0 ± 3 m, ψ = 0 ± 5°. The user sees k as pulses per metre, `1 / k`; on an ISO 11786 signal it should come out near the nominal 130, which is a quick check that the wiring and the fit are right.

- **k** shows on any driving; straight driving pins it alone. It changes with surface, load and tyre pressure, so its normal equations decay with a forgetting factor (time constant about 10 minutes of qualifying driving, estimate). The long-run value (tyre size) is stored as the base.
- **c, d** show only in turns, both ways, so they converge over the first headlands. They are geometry and don't change, so they decay much slower (hours) and are stored.
- **ψ** separates from d because its regressor is the speed, not the turn rate. Only learn it when the heading is from the UM982's dual antenna or after long straight runs; a filter heading error looks the same. Otherwise leave the mounting yaw to [imu-auto-calibration.md](imu-auto-calibration.md).

With all four learned, core 0 has the full horizontal velocity at the IMU from the pulses and the gyro, `[k f + c ω_z,  d ω_z + ψ u]`, without any axle position. INSLIB step 1 fuses its magnitude in `m.speed`; step 2 fuses the vector through the body-velocity input with a zero lever arm, since the correction to the IMU point is already done.

### Gates (which epochs are used)
- RTK fixed, GNSS velocity sigma under about 3 cm/s, no recent jump in the solution.
- |u| above about 0.5 m/s, and a full pulse window (at least a few edges).
- No hard acceleration or braking (|a_x| limit): that is when driven wheels spin or lock.
- Before c is known: skip epochs with `|ω_z| · 1.5 m > 0.5 |u|` (the wheel might be the inner one in a tight turn). After: skip when the wheel is the inner one and `|c · ω_z| > 0.5 |k f + c ω_z|`.
- After convergence: drop epochs with a residual over 3 sigma (slip spikes), but count them; the slip rate is a status figure.

### Direction
A single pulse signal is unsigned: during learning the sign comes from `u`, in use from the filter's forward velocity, and near zero speed the sample is skipped. A quadrature encoder on A and B is detected by itself (the two inputs step through the Gray sequence 00 → 01 → 11 → 10; built in `firmware/app/field_io.c`, shown as forward / reverse in the status report), and its polarity (which way is forward) is learned from `u`, so swapped wires don't matter.

### States
| State | When | Fused into INSLIB |
|---|---|---|
| idle | no pulses on the channel | no |
| learning | pulses, k not yet within about 1 % (estimate: a few hundred metres of straight RTK driving) | no |
| straight only | k known, c not yet (no turns both ways) | only while `|ω_z|` is small |
| ready | k, c (and d) known | yes; `speed_stddev_rel` from the recent residual spread plus a slip margin |
| holding | RTK lost | yes, with the last k; its sigma grows with time, because slip can change while nothing checks it |

A radar and a wheel signal are handled the same way; the radar simply ends up with a small slip spread.

**Relearn:** when the residual mean stays off by more than a few percent for minutes under RTK (tyre change, a different tractor, the sensor moved), or the pulse rate per metre jumps, k is reset to the new straight-line ratio; if the turn residual still grows with `ω_z`, c and d are reset too.

### Storage and the web page
k (base), c, d, ψ, their sigmas and the amount of data behind them go in a flash record with `flash_rec.c`, applied at boot, so a tractor that was learned once is ready on the first metre. `calib.html` shows them with the state, **pulses per metre** (current and base, the difference being the slip), the wheel offset and IMU-to-axle distance in metres, and the same auto / hold / clear buttons as the IMU groups. Until the learning is built, the status report and `/status.json` (`speed.pulses_per_m`, with `gnss_mps` and `ppm_span_s`) show a raw live figure: input A's pulses over the GNSS distance between edges, per run of at least 5 s above 1 m/s, without the turn correction ("First sensor").

## Per platform
| Platform | Use |
|---|---|
| Tractor (and other wheeled vehicles) | On by itself when pulses appear on a channel; learns under RTK, fused once learned. The web page can turn a channel off |
| Survey pole | Off |
| Multirotor, fixed wing | Off |

## Field testing: live on the tractor, and a log of everything
Develop on the tractor live, and log the raw data of every drive at the same time.
- **Live:** the unit (GNSS, IMU, barometer, speed input, and the camera with its Pi when fitted) on the tractor, the Pico on the USB of a Linux box riding along (dietpi on a 12 V → 5 V supply), reached over Tailscale through a phone hotspot. Flash over SWD as on the bench and watch the 1 s status report (`SPEED`, `FLOW`, `LOG` lines) and the web page. This is what shows at once whether the inputs work, the pulse rate tracks the GNSS speed and the direction reads right.
- **Log:** every drive is recorded raw, so the learning can be tuned again and again on the same data, compared fairly between versions, and fed to INSLIB's `replay.c` with GNSS cut. A live change costs a rebuild, a flash (which restarts the learning) and another lap that is never quite the same.
- The learning is written as one plain C module that builds both into the firmware and on the host, like `imu_calib_*.c`: it runs live on the Pico during the drive and replays the log afterwards.

**USB log mode (built 2026-11-15):** `firmware/app/usb_log.c`. Off at boot; the line `log on` on the USB port starts it, `log off` (or DTR down for 2 s) stops it. Framed binary records with a CRC on the same CDC port; while it is on, `printf` output (the status report) goes into TEXT records in the same stream. What it carries:
| Record | Rate | Content |
|---|---|---|
| IMU | 800 Hz | TM210 sample as sent to INSLIB (learned gyro correction applied) |
| GNSS | 20 Hz | BESTNAV: position, velocity, sigmas, fix type, PPS flag |
| HEADING | 20 Hz | UNIHEADING (UM982 dual antenna) |
| BARO | 100 Hz | BMP581 pressure and temperature |
| NAV | 10 Hz | INSLIB output: attitude, position, velocity, biases, mode |
| SPEED_EDGES | batches of up to 48 edges, at most 50 ms old | every edge on inputs A and B with both levels |
| FLOW_EVENT | 100 Hz trigger + strobe edges | camera trigger pulses (and the left-out one), strobe edges |
| FLOW_FRAME | camera rate | each Pi flow message with a good CRC, raw, with its arrival time |
| TIME | 1 Hz | the PPS second on both clocks (Pico time, GPS week and time of week) and the leap seconds |
| TEXT, START, DROPPED | | status text; log start; bytes lost when the ring was full |

The flow part needs no camera: the trigger runs from boot (100 Hz, `APP_FLOW_TRIGGER_HZ`), and whatever the Pi sends on GP8 is logged. So optical flow is recorded alongside the wheel speed whenever the camera rig is on the tractor. The Pi-side flow program and the frame format are [optical-flow-aiding.md](optical-flow-aiding.md) and `common/flow_link.h`.

`firmware/tools/field_log.py record PORT` turns the log on, saves the raw stream to a file and shows the status text with a rate summary; `summary FILE` and `decode FILE` (one CSV per record type) work on the saved file.

**GNSS time across reboots (added 2026-11-15):** the Pico's clock restarts at 0 when it reboots, so the Pico times alone can't join a log that spans a reboot. `record` reopens the port when it goes away (or sends nothing for 3 s, `--silence`), sends `log on` again and appends to the same file. `decode` splits the file into sessions at each START record (or where the Pico clock goes back), maps each session's Pico times to GPS time from its PPS-tagged GNSS epochs (interpolated between epochs, extrapolated at the ends with the session's clock rate; epochs without PPS only when a session has none, late by the output latency), and adds `seg` and `gps_s` (GPS seconds since 1980-01-06) to every CSV that has a Pico time. `segments.csv` and `summary` list the sessions with their UTC span (leap seconds from the TIME records) and the Pico clock rate against GPS. Records before the receiver has time get an extrapolated `gps_s` if the session has epochs later, else none. Tested on a synthetic two-boot log (20 ppm clock error, 1 µs agreement); the reconnect after a real reboot is not tested yet.

Bench result (2026-11-15, dietpi, no tractor): 54 kB/s, 0 bytes dropped in 60 s (ring peak 7.5 KB of 32 KB), also while `/status.json` was polled 5 times a second; 0 CRC errors; every IMU sample there (no gap over 1.25 ms); trigger pulses 10.000 ms apart with the left-out pulse every 256; inputs A/B driven from SWD (800 edges) all logged, quadrature forward and reverse read correctly; 50 strobe pulses gave 100 edges. Core 0 load 15–19 % with the log off, 26–27 % with it on.

**Driving program for the first drive** (RTK on): straight runs both ways; headland turns left and right; full circles both ways at two speeds; a few standstills of a minute (clean ZUPT data); reversing, if the encoder gives direction; then the same with the camera rig if it is fitted. Note surface and tyre pressures.

## Steps
1. Sensor: **decided 2026-11-16**, a coarse Hall pickup on the right rear wheel on input A (see "First sensor"). Count the targets and measure the rear tyre's circumference by rolling, for an expected pulses per metre; measure the rear track and the IMU's position against the right rear wheel with a tape, for the learning check later. The raw pulses-per-metre figure is edge-timed (built 2026-11-16, "First sensor").
2. Bench: the opto input stage with a signal generator into GP12; rate against frequency, timestamp jitter, also at 1–10 Hz (the Hall pickup's range). Then the pickup itself on the stage, passed by a target by hand: one clean rising edge per target, no bounce. Pulse capture and the log are built and bench-tested without the opto stage (above).
3. First drive, live and logged (program above, without reversing). Check the raw pulses per metre in the status report against the expected figure from step 1, and look at the gap pattern in the log (uneven targets).
4. Write the auto-learning as a host-and-firmware C module, per gap ("First sensor"); tune it on the logs, then run it live on the next drive. The gap-length table only if the log shows the spread is needed.
5. INSLIB step 1: the learned speed through `m.speed` (no library change). Replay with GNSS cut for 10 s, 30 s and 60 s (`external/INSLIB/tools/replay.c`): lateral constraint only, wheel speed, flow, wheel and flow.
6. The flash record and the `calib.html` section (pulses per metre).
7. INSLIB step 2: the body-velocity input with a lever arm, shared with flow.

## Verification
- Bench: measured speed within 0.1 % of the generator frequency, from 1 Hz up; edge timestamp jitter under 10 µs. Hall pickup: one rising edge per target.
- Log: `field_log.py summary` shows 0 dropped and 0 CRC errors over a full drive, and every IMU sample there.
- Field, straight: drive a measured line under RTK; pulse distance against RTK distance gives the scale and the slip.
- Field, turns: drive circles both ways under RTK; the axle-centre speed moved to the IMU point should match the GNSS velocity (antenna lever arm applied) in the turn as well as on the straight; the residual left in turns, plotted against ω_z, should have no slope (a slope means `y_w` is off, or the sign convention is).
- Learning: from a cold start with no record, drive straight then a few headlands under RTK; k, c and d should settle and match a tape measure (c positive for the right rear wheel, about half the track from the centreline, d the IMU-to-rear-axle distance) within a few centimetres, and 1 / k match the target count over the rolled circumference. Reboot: the record is applied and the channel is ready at once. Change tyre pressure: k follows, c and d stay.
- Filter: position error growth in GNSS-cut replays with and without wheel speed, compared with flow.
