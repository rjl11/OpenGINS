# Survey pole: heading and tilt from natural sway

## Context

On a survey pole the output is the pole tip, not the antenna: the ground point `ant_height_m` below the antenna along the body down axis (`firmware/app/ground_point.c`, [web-config-server.md](web-config-server.md), version 5 settings). It follows the tilt only while the INS attitude is good, a 2° gate on the down axis's 1-sigma ([inslib-nav-suite.md](inslib-nav-suite.md), output table). With one antenna the pole has no heading source of its own: the course over ground is not its heading, so INSLIB's `automotive_mode` is off on the pole platform ([gnss-um98x-config.md](gnss-um98x-config.md)). Without help the yaw converges only slowly while walking, and the ground point stays "straight down" until it does.

Two papers in `docs/survey-pole/` show how commercial and research tilt-RTK poles solve this:

- Luo, Schaufler, Carrera, Celebi, *High-Precision RTK Positioning with Calibration-Free Tilt Compensation*, FIG 2018 (`docs/survey-pole/TS04E_luo_schaufler_et_al_9407.pdf`): the Leica GS18 T. INS with GNSS position and velocity, no magnetometer, tilt compensation switched on and off by a 2° attitude-uncertainty gate, about 10 s to get there "under normal conditions with sufficient movements".
- Ai, Chen, Niu, *All-in-One Initialization of MEMS-INS for Tilt-Compensated GNSS RTK Receivers*, IEEE TIM 2026 (`docs/survey-pole/All-in-One_Initialization_of_MEMS-INS_for_Tilt-Compensated_GNSS_RTK_Receivers.pdf`): swing the pole with its tip on a fixed point. The tip doesn't move, so its specific force is gravity (eq. 14) and its velocity is zero (eq. 3); in an EKF that gives roll, pitch, gyro and accelerometer errors and the velocity in about 80 s of a set swing. Heading comes from GNSS by matching the INS and GNSS tracks (Chen et al., GPS Solutions 2020).

The user doesn't want a set swing at every start (decision 2026-11-16). The firmware uses the small sways that happen anyway while a point is observed, walking, and the magnetometer only when its readings look right. A deliberate swing still works and only makes it faster.

Status: built and host-tested 2026-11-16 (the learned pole axis, item 5, too), not yet run on a pole. Every threshold below is an estimate, not yet measured.

## Design

All on core 1, in `firmware/app/pole_aid.c/.h` (plain C, in SRAM with the filter), called from `nav_core1.c` on every IMU sample. Active only on the pole platform (in effect since boot) with `ant_height_m > 0`. The tip lever arm is the GNSS lever arm plus `(0, 0, ant_height_m)`: the pole runs along the body down axis.

### 1. Planted-tip detector

The Wuhan paper's eq. (14) without the filter: the specific force moved to the tip, `f_T = f + w x (w x l) + dw/dt x l`, stays at g in magnitude while the tip rests on the ground, however the top sways. `dw/dt` is the 20 Hz low-passed rate differenced over 10 ms. Over a 0.25 s window of 50 ms blocks the tip counts as planted while

- the mean of `||f_T| - g|` is at most `APP_POLE_TIP_ERR_MAX` (0.25 m/s²) and no block peaks above 1 m/s², and
- the INS's own tip velocity `v + R (w x l)` stays under `APP_POLE_TIP_SPEED_MAX` (0.2 m/s).

The second test is needed: a pole carried at a steady speed without turning passes the force test. Walking fails the force test anyway (a 3 cm bounce at 2 Hz is about 5 m/s²). Before the INS is ready there is no tip velocity, so nothing counts as planted.

### 2. Tip constraint (the paper's constraint, at position level)

INSLIB's local NED position input has its own lever arm (`m.local_pos`, `local_pos_leverarm_b`, `ins_fuse_local_pos()` in `external/INSLIB/src/ins.c`, chi2-gated), so the constraint needs no INSLIB change. 0.3 s after the tip is planted, core 1 latches the tip, `ins_get_position_local() + R l`, and then hands INSLIB that position every 50 ms with lever arm `l` and 1-sigma `APP_POLE_TIP_SD_M` (1 cm per axis). Only while `ins_is_ready()`: before that a local position would bootstrap the filter. The latch is renewed every 5 s, so neither it nor the yaw fit holds a stale value for long.

With the GNSS fix of the antenna in the same filter, the antenna moves on a sphere around a point that doesn't move. That makes roll, pitch and the gyro bias observable from any sway, and the yaw too when the fixes are good enough to see the antenna move (RTK: centimetres, against a sway of about 10 cm at 2 m).

Side effect, accepted: while the tip is held, INSLIB counts the local position as aiding, so the 3D solution stays up through a GNSS gap. The tip really is fixed then.

### 3. Yaw fix per planted episode (track matching)

The filter's yaw update is small-angle; with no heading source at all the start yaw can be anything. So each planted episode also gives a direct, linear yaw fix. For every GNSS fix with a horizontal 1-sigma of at most `APP_POLE_FIT_GNSS_SD_MAX_M` (3 cm: RTK), the antenna's offset from the tip as the INS sees it, `a_i = R(t_valid) (l_ant - l_tip)` (attitude from a 100 Hz ring at the fix's time of validity), is paired with the fix. The model `p_i = t + Rz(dpsi) a_i` is linear in `(t_N, t_E, cos dpsi, sin dpsi)`: a 4x4 least-squares solve. At the end of the episode (the pole lifted, or 5 s held) the fit is used when

- it has at least 10 fixes and the offsets spread at least 8 cm (else there was too little sway; not counted as a rejection),
- the residual RMS is at most 2 cm, and `|sqrt(c² + s²) - 1|` at most 0.1 (the tip really stayed put).

Then INSLIB gets one yaw measurement, the INS yaw plus `dpsi`, with the fit's 1-sigma (at least 2°), counted in `n_yaw`. One per episode, so the same data is never fused twice; INSLIB's chi2 gate still judges it. Host test: from 0°, 90°, 170° and -45° of yaw error it returns the error to within 1° (1.7 cm noise, 4° sway); a tip drifting 5 cm/s is rejected.

### 4. Magnetometer trust gate

On the pole the magnetometer reaches INSLIB only while trusted (counted as `n_mag_withheld` otherwise). Each sample is calibrated as INSLIB will see it (`M (raw - bias)`, less the INS hard-iron estimate) and turned to NED with the INS attitude, or the gyro-only filter's before the INS is ready. Per 1 s block: the worst field-strength and dip error against INSLIB's WMM, the tilt range and the mean yaw; and, once the INS yaw 1-sigma is under 5°, the heading residual against the INS. Trusted when the last 10 s (at least 8 blocks) show

- field strength within ±10 % and dip within ±5° of the WMM,
- the attitude varied: 5° of tilt or 20° of yaw (a bad calibration or a nearby disturbance shows only when the orientation changes), and
- where the INS yaw is known: heading residual mean within ±5° and SD under 3°.

Once trusted, each new block only has to pass the field and dip tests (and the residual where checked), so a pole held still keeps its trust. No WMM yet (no position): never trusted. So INSLIB's start yaw comes from the magnetometer only when it looks right; otherwise from items 2–3 and walking.

### 5. Pole axis in the IMU frame (learned mounting)

Items 1–3 assume the pole runs along the body down axis. If the TM210 sits tilted by α on the pole (the real axis is the line from the tip to the antenna), every tilted point is off by `L·sin α` sideways, in a direction that turns with the pole: 1° is 3.5 cm at 2 m. This is the error a two-face check shows (the same point with the pole turned 180°). It is learned in the background from the planted episodes, in `firmware/app/pole_mount_fit.c` (the fit, host test `firmware/tools/test_pole_mount.c`) and `firmware/app/pole_mount.c` (record, boot decision, web actions, core 0).

| Offset | Effect on the ground point | Seen from | Done |
|---|---|---|---|
| Pole axis tilt in the IMU frame (2 angles) | `L·α`, turns with the pole | a **twist of the pole about its own axis** while planted (the antenna traces a circle of radius `L·α`) | learned and applied |
| Pole length L (antenna phase centre to tip) | about 1:1 in height | the sway (the antenna moves `L·δθ`) | learned, **shown only** against `ant_height_m`: a tape measure plus the antenna's PCO is better |
| Rotation of the IMU about the pole axis | none (a pole has no forward direction) | — | not needed |
| IMU off the pole axis (antenna and tip shifted alike) | none (it cancels in tip − antenna) | — | not estimated; it only enters INSLIB's GNSS fusion |

**Model.** Per RTK fix i of episode k: `p_i = t_k + Rz(ψ_k) G_i d`, with `d` = antenna − tip in the body frame (nominally `(0, 0, −h)`), `t_k` the episode's tip, `ψ_k` its yaw correction and `G_i` the top two rows of the attitude. Horizontal rows only. The model is linear in `d`, so each episode's normal equations (with `t_k` eliminated) add up across episodes and boots, whatever axis is in use. Plain sway can't show the tilt: at a constant yaw, `R·(L α)` is a constant shift that `t_k` takes up. It takes a twist within an episode.

**On core 1** (`pole_aid.c`), per episode:
- The attitude is integrated from the gyro alone from the start of the episode (synced to the INS between episodes). While the tip constraint is fed with the wrong axis, INSLIB bends its attitude to fit it during a twist, which would hide part of the error. The rate is calibrated as INSLIB uses it, less its bias estimate.
- The sums are kept so that the yaw correction can be applied at the end (`pole_episode_t`, about 110 B). The twist (rotation about the pole axis, max − min) is kept for the status.
- An episode goes to core 0 through a queue when its yaw fit was solved with enough sway and `|scale − 1| ≤ 0.1`, whatever its RMS: once the pole twists, a mounting error raises the RMS, so the axis fit judges that itself.

**On core 0** (`pole_mount.c`, `APP_POLE_MOUNT_*`, all estimates, not yet measured):
- **Yaw per episode:** `ψ_k` is refitted (2-D Procrustes) against the current axis, twice: once against the axis before, once with the episode in.
- **Prior:** tilt σ 3°, length σ 5 cm about `ant_height_m`. Fix noise 1.5 cm, with the variance inflated 10× because 20 Hz RTK errors are correlated over about half a second.
- **Episode gate:** an episode that fits worse than 2 cm RMS is dropped (a slipped tip). Once 5 episodes are in, so is one worse than 1.5× the overall RMS (but at least 1 cm). A slip alone partly hides in the axis; against a baseline it doesn't.
- **Use:** the learned axis is used when its tilt σ is at most 0.2° (7 mm at 2 m), the tilt at most 5°, and the length within 30 cm of the antenna height.
- **Applied:** the axis `u` replaces the body down axis in the tip lever arm (`leverarm + h·u`, core 1, live) and in the output ground point, which uses `h·u` only while the attitude passes the gate. The straight-down fallback stays `(0, 0, h)` in NED, because a levelled pole's axis is the true vertical. The tilt in the status is the pole's.
- **Record:** its own `flash_rec` in the two sectors below the IMU calibration's. Saved at most every 5 minutes while new episodes come in, and at each action.
- **Modes**, as for the IMU groups on `calib.html`: auto (each boot uses the learned axis when it passes the checks, else the stored set), hold, accept, clear, factory. No stillness needed, since the tip lever arm changes without a filter restart.

Host test (`test_pole_mount.c`, the whole path through `pole_aid` with an exact rigid-body motion, RTK noise 1.2 cm):

| Case | Result |
|---|---|
| 1.5° mount, 20 episodes with ±20° twist | 1.55°, toward 60.2° (true 60°), σ 0.13°, L 1.996 m |
| The same, with the INS attitude bent by 0.5° following the twist | the same result |
| 2° mount, one turn on the point | 1.95°, toward −123° (true −120°), σ 0.16° |
| No twist | tilt stays at its prior (σ 3°); L still 1.996 m |
| A tip slipping 5 cm/s after 20 good episodes | all 6 dropped |

The INS yaw was off by −60° to +90°.

### Not done

The gyro and accelerometer scale factors (the other states in the Wuhan EKF): INSLIB has no scale states. The background calibration (`firmware/app/imu_calib*.c`, [imu-auto-calibration.md](imu-auto-calibration.md)) already learns them on the pole from poses and rotations and applies them at boot, so nothing has to be redone at each start.

## Status output

The USB status report has a `POLE` line (planted / tip held / moving, the tip force error and speed, counts of plants, tip fusions and yaw fits, the last fit, the magnetometer gate) and `/status.json` a `pole` object with the same. The learned axis (item 5) is a `CALIB pole` line (axis in use, learned tilt with σ and direction, length against the setting, episodes, twist, the last episode's fit) and a `pole` group in `/calib.json`, shown on `calib.html`. All are absent on other platforms.

## Field test

1. Pole platform, `ant_height_m` set to the pole length plus the antenna's phase-centre offset, RTK fixed.
2. Stand the pole on a point and hold it as for a measurement: `POLE planted, tip held`, tip fusions counting. After a few seconds of natural sway: a yaw fit used, `axis_sd_deg` under 2, the ground point `tilted`.
3. Lift and carry: `moving`, no tip fusions; the tip speed shows why.
4. Magnetometer: next to a car or a steel facade it should be withheld; in the open, after walking and tilting a while, trusted, with the residual against the INS small.
5. Ground-point accuracy: the same point measured at 0°, 15° and 30° tilt and from four directions; compare the spread with the Leica figure (about 8 mm at 1° to 2 cm at 30° from attitude alone, at 1.8 m).
6. Pole axis (item 5): shim the TM210 by a known 2° on the pole. Measure points as usual for a while, then turn the pole once slowly on a point. `CALIB pole` should show a learned tilt of 2° ± 0.3°, and the length within a few centimetres of `ant_height_m`. After a reboot (auto mode) or accept, repeat step 5: the spread over the four directions should drop from about 7 cm to under 1 cm.

Record with `log on` and `firmware/tools/field_log.py` for replay.
