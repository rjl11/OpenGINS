# IMU: TM210 configuration for INSLIB

## Context
The IMU is a SYD Dynamics TransducerM **TM210** (9-axis AHRS). INSLIB does its own fusion ([inslib-nav-suite.md](inslib-nav-suite.md)), so the TM210 is used only as a source of calibrated raw gyro, accelerometer and magnetometer samples. This plan covers how to configure it, how to read it on the RP2350, and how to convert its data into what INSLIB expects.

Sources: `docs/datasheets/TransducerM_TM210_Datasheet_EN_V126-R.pdf`, `docs/TransducerM_TM3xx_UserGuide_EN_V135-R1.pdf` (EasyProtocol section; the protocol is shared across TransducerM models). Vendor example code and the TM210 3D model (`TransducerM_TM210_3D-Shape_V1-1-2.stp`, for mounting) are in `external/syd_dynamics/`. Vendor Q&A: https://www.syd-dynamics.com/questions-answers/.

## Key facts
- Internal update rate 800 Hz. Output data rate (ODR) selectable: 800, 400, 200, 100, 50, 25, 10, 5, 1 Hz.
- UART: 3.3 V TTL (5 V tolerant), 2400 bps to 921600 / 1 Mbps. The vendor recommends ≥921600 for ODR ≥400 Hz. USB-C virtual COM port works at the same time.
- Power: 5 V, 80 mA. Connector PH2.0-5: RXD, TXD, 5V, GND, GND.
- **Sensor frame** (printed on the case): x forward, y left, z up (FLU). INSLIB works in **FRD** (x forward, y right, z down).
- Gyro: ±1000 °/s, noise 0.20–0.35 °/s peak-to-peak at 100 Hz, bias stability <5.5 °/h, temperature drift ±0.004 °/s/°C.
- Accelerometer: ±10 g, noise density 190 µg/√Hz, bias instability 0.05 mg. Calibrated "in units": direction is accurate, absolute magnitude is not.
- Magnetometer: ±1.3 G, internal sampling 75 Hz, unit = "one Earth field as measured at factory calibration in Denmark".
- Calibrated at 22.5 °C only (TM210-T1 variant: 5–65 °C). Outdoors on a tractor, expect temperature-driven gyro bias that INSLIB must track.

## EasyProtocol packet
`AA 55 | length (1 B) | header (4 B) | payload | CRC16 (2 B, Modbus, over length + header + payload)`

Header bit fields: CMD (7 bits), 3 bits, FROM_ID (11), TO_ID (11). The user guide calls the 3 bits "reserved, must be 0", and the newer embedded EasyProfile builds call them QoS. On the TM210 they read 0 on every packet, so accept any value and ignore it. QoS comes from the Status object ([imu-tm210-config.md](imu-tm210-config.md)).

**Raw Sensor Data object, CMD 41** (40-byte payload, 49 bytes per packet):
| Bytes | Field | Unit |
|---|---|---|
| 0–3 | timestamp, uint32 | µs since TM210 start, wraps every ~71.6 min |
| 4–15 | gyro x, y, z (float32) | rad/s |
| 16–27 | acc x, y, z (float32) | g (vendor: 1 g = 9.8158 m/s²) |
| 28–39 | mag x, y, z (float32) | Earth-field units |

The user guide ("Raw Data Display" and "Raw Sensor Data") calls this "calibrated sensor data without applying any data processing filter", with gravity part of the accelerometer measurement.

**Status object, CMD 22**: timestamp, temperature [°C], internal update rate [Hz], QoS (0–5). Request with the broadcast command `aa55080c08000016000000e0ed`. From the user guide ("Status" and "Step 2 – Heat Up"):
- **QoS:** 0 booting or restarting, 1 system fault, 2 limited (for example right after a dynamic boot), 3 basic (right after a static boot), 4 fine, 5 very good. **The technical specification applies only from QoS 4.** The status bits need firmware V4.7.5 (3) or later; earlier firmware sends 0.
- **Temperature** is measured inside the main sensor chip, and is usually above ambient because of the module's own heat.

## TM210 configuration (set by the MCU at boot)
The Pico checks these settings at every boot and provisions the TM210 if they don't match: read-modify-write of the Setting object, applied to RAM first and saved to flash only after the output has been verified. A new baud only takes effect from flash after a power cycle, so moving a TM210 from 115200 to 921600 takes one power cycle after the Pico has saved it (bench, 2026-11-13). No PC is needed. Details are in [imu-tm210-config.md](imu-tm210-config.md).

| Setting | Value | Why |
|---|---|---|
| Output objects | **Raw data only**. Status, RPY, Quaternion, Euler, Gravity, Combo (CMD 43) and CMD 42 off | INSLIB fuses itself; less traffic and parsing |
| ODR | 800 Hz (Setting bytes 18–19, in Hz; verified on the bench) | Full internal rate; INSLIB strapdown at 800 Hz |
| TM210 UART baud (the TM210 calls its port UART1; it goes to the Pico's UART0) | 921600 | (49 B raw + 21 B Status) × 800 Hz = 56 kB/s ≈ 560 kbit/s with start/stop bits, ~61 % of 921600 |
| USB output (switch bit 20) | on | ImuAssistant can debug over USB while the Pico reads UART; also the recovery path. The TM210 has no CAN bus, so the user guide's CAN settings don't apply |
| Temperature calibration (`reserved7`) | 0 | Vendor recommendation |
| Boot mode | Auto boot (3.2 s cold start) | No need for the 10–30 s static boot; INSLIB levels itself |
| Calibration commands (CalibB, forced calibration per AN002, heading fix) | never sent | They only affect the TM210's fused outputs. With raw-only output they are irrelevant, and INSLIB estimates the biases |
| Sensor fusion profile, gains, gyro/acc/mag enable checkboxes | unchanged | Do not affect raw data output (the vendor Q&A says so for the enable checkboxes) |
| Status object | output (bit 5): one CMD 22 after every CMD 41, same TM210 timestamp, 800 Hz. Not requested, because steady requests stop the Setting replies ([imu-tm210-config.md](imu-tm210-config.md)) | Temperature and QoS for logs and the web page |

Recovery: if the baud rate is unknown, the firmware scans the supported rates itself. By hand, the TM210 is recovered over its USB port (vendor Q&A); the 6-second 115200 SCAN lock in the user guide is for the TM200/TM35x, not the TM210. ImuAssistant is still used for bench work (Allan variance, debugging).

Check after configuring: the TM210 sends CMD 41 at 800 ± 20 Hz and CMD 22 Status on its UART (the Pico's UART0), nothing else. Status follows the raw packet of the same frame, so it doesn't delay the raw packet's arrival and the timestamp correction (`PACKET_TX_TIME_US`, the raw packet alone) stays.

## Firmware: reading the TM210 (core 0)
- **UART0** at 921600, RX into a DMA ring buffer (≥4 KB, ~100 ms of data). TX carries the Setting read and write at boot ([imu-tm210-config.md](imu-tm210-config.md)) and nothing after that.
- **Parser**: sync on `AA 55`, check length and CRC, accept and ignore the 3 header bits after CMD, accept CMD 41 (plus 21, 13 and 22 when requested or during configuration). Count CRC errors and lost packets for diagnostics.
- **Timestamps**:
  - Unwrap the uint32 µs timestamp to 64-bit.
  - Use TM210 timestamp differences for `strapdown_dt_sec` (the sensor's own sample spacing, no UART jitter).
  - Map TM210 time to the Pico µs clock with an offset + drift estimate: offset = minimum of (arrival time − TM210 time) over a sliding window, minus the fixed packet transmit time (~0.53 ms at 921600). GNSS (PPS) and baro are on the Pico clock, so everything shares one timebase.
  - Flag gaps (dt > 2 × 1.25 ms) and backwards steps; INSLIB's IMU-loss handling takes over on long gaps.
- Push one sample per packet onto the core 0 → core 1 queue.

## Converting to INSLIB units and frame
INSLIB wants FRD, accelerometer as **specific force** in m/s² (level and at rest: `acc ≈ (0, 0, −9.81)`, see `external/INSLIB/doc/example_imu.csv`), gyro in rad/s, magnetometer in µT.

| Sensor | TM210 (FLU) → INSLIB (FRD) |
|---|---|
| Gyro | `(gx, −gy, −gz)` rad/s |
| Accelerometer | `(−ax, ay, az) × 9.8158`. The TM210's raw object reports the **gravity vector**, not specific force (bench result 2026-11-12, below; the datasheet footnote shows the same `(0, 0, −1)` with z pointing to the sky). The firmware sets this with `APP_TM210_ACC_IS_GRAVITY 1`. |
| Magnetometer | `(mx, −my, −mz) × ~50 µT` (approx. total field in Denmark), then refine with mag calibration |

Evidence for the accelerometer sign, from another project: the AgOpenWeb-HAT IMU evaluation (https://github.com/chriskinal/AgOpenWeb-HAT, `hardware/imu-breakout/HARDWARE_IMU_BREAKOUT.md` §8.4, commit `e4bd9ef`, read 2026-11-10) logged a **TM171** beside an ST IMU and found "the same axes [X forward, Y left, Z up], its accel the opposite sign (it reports gravity), its gyro in rad/s". That points to the second form, `(−ax, ay, az) × 9.8158`. It was measured on a TM171's output, not on our TM210's raw object; our bench check (below) found the same on the TM210. The user guide's own raw-packet example ("Example of EasyProtocol", "More Examples") reads accel z = −1.00012 g with near-zero gyro, which fits too. It is a TM3xx example with an unstated orientation, so it is supporting evidence only.

Do the sign/unit conversion in the parser so all later data is plain FRD SI. Put the **physical mounting** (TM210 not aligned with the vehicle) into config as `mount_rpy_deg` and compose it into `imu_acc_misalignment` / `imu_gyr_misalignment` with `imu_mount_compose()` from `external/INSLIB/tools/imu_mount.h`.

Magnetometer: the TM210 samples it at 75 Hz but repeats it in every 800 Hz packet. Set `mag.is_valid` only when the value changes. It is fed to INSLIB from boot (user decision 2026-11-13, `APP_FEED_MAG` in `firmware/app/app_config.h`): the AHRS and the INS both use it, INSLIB applies its own World Magnetic Model for the declination, and the INS estimates the hard-iron bias. The on-vehicle calibration (step 4 below) refines it later ([inslib-nav-suite.md](inslib-nav-suite.md), "Magnetometer").

## INSLIB noise parameters
The firmware defaults (`APP_*_PSD`, `APP_*_BIAS_RW` in `firmware/app/app_config.h`) are the measured values from our own Allan run below. Settings saved from the web page are kept in flash and override them.

| Parameter | Measured (in use) | Datasheet start value | Start value derivation |
|---|---|---|---|
| `acc.Qll_diag` (PSD) | **2.1e-6** (m/s²)²/Hz | 3.5e-6 | (190 µg/√Hz × 9.81)² |
| `gyr.Qll_diag` (PSD) | **1.0e-7** (rad/s)²/Hz | 1.5e-8 | 0.2–0.35 °/s p-p at 100 Hz → σ ≈ 0.05 °/s over 50 Hz → ~1.2e-4 rad/s/√Hz |
| `gyr_bias_pred_stddev_rps_sqrts` | **7e-7** rad/s/√s, upper bound | 5e-6 (2e-6 to 1e-5) | `config_basic.yaml` value, raised for outdoor temperature swing |
| `acc_bias_pred_stddev_mps2_sqrts` | **3e-5** m/s²/√s | 3e-4 | `config_basic.yaml` value |

### Allan variance result (TM210, node ID 925, 2026-11-13/14)
- **Run:** static on the bench, logged over USB on dietpi with `tm210_allan_log.py` (below): 30 min warm-up, then 12 h from 22:50 to 10:50, QoS 5 throughout. 36.1 M raw packets, 14,041 missing (0.04 %) in about 50 bursts of up to 0.8 s in the recorded part, 1 CRC error, no reconnects. Converted with `tm210_to_imu_csv.py --average 8 --max-fill-ms 2000` into one 12.03 h segment at 100 Hz (10,606 samples, 0.03 %, interpolated). With the default 200 ms fill limit the record split into hour-long segments, too short for the bias random walk.
- **Disturbances:** the IMU temperature rose from 23.3 to 24.5 °C in the first 1.3 h, then fell slowly to 21.8 °C by morning. The x accelerometer bias settled by about 25 mm/s² over the first 2–3 h. A knock at 10:18 (11.46 h in; accelerometer spikes of 2 m/s²) shifted the x accelerometer by about 10 mm/s² for the rest of the run. Apart from that, only 11 samples lie above 6σ. The "possible motion" warnings (272) are noise peaks just above the tool's 1 °/s threshold.
- **Analysis:** `allan_variance.py --skip-start 7200 --skip-end 2100`, 9.4 h. The full 12 h gives the same white noise and gyro values, but an accelerometer bias random walk twice as high (6e-5), driven by the settling and the knock.

| | x | y | z |
|---|---|---|---|
| Gyro noise density, rad/s/√Hz (ARW) | 3.4e-4 (1.2 °/√h) | 4.0e-4 (1.4 °/√h) | 1.9e-4 (0.64 °/√h) |
| Gyro bias instability, upper bound | ≤ 2.2 °/h | ≤ 7.0 °/h | ≤ 2.4 °/h |
| Gyro bias random walk, upper bound | ≤ 2.2e-7 rad/s/√s | ≤ 6.8e-7 | ≤ 2.3e-7 |
| Accel noise density | 1.4e-3 m/s²/√Hz (143 µg/√Hz) | 1.4e-3 | 1.5e-3 |
| Accel bias instability | 0.37 mm/s² (0.037 mg) at 210 s | 0.58 mm/s² at 264 s | 0.37 mm/s² at 420 s |
| Accel bias random walk | 2.0e-5 m/s²/√s | 2.5e-5 | 1.7e-5 |

- **Gyro:** the noise is about 7× the datasheet start value as a PSD (2.6× as a density), with x and y about twice z. The pattern matches the TM171 comparison below, at about 1.5× its level. The Allan curve still falls at the longest trusted averaging time (about 3,400 s); `allan_variance.py` puts the +1/2 branch near 5,400 s, which needs about 45 h of static data. So the bias instability and bias random walk are upper bounds, and the gyro bias is more stable than this record shows.
- **Accelerometer:** the noise density is 25 % below the datasheet's 190 µg/√Hz, and the bias instability matches its 0.05 mg.
- **Bump near τ = 1–2 s** on the x and y gyro curves: a narrow line at **0.289 Hz** (period 3.46 s; 256 s FFT windows, resolution 0.004 Hz) with an amplitude of about 4.4e-4 rad/s (0.025 °/s) on x and y and none on z. A sinusoid at f peaks in the Allan deviation near τ = 0.37/f = 1.3 s. It is not the logger's 1 Hz Status request: a 10 min log with `--no-status` on 2026-11-14 (13.8 °C, the IMU in another orientation) shows the same line at the same frequency and amplitude, and the same Allan curve within a few percent. It is also unchanged between 24 °C and 22 °C in the overnight run. The cause is not known; the same amplitude on x and y in two orientations points to something inside the TM210 rather than the bench, but that is an inference. As an attitude error it is small: 4.4e-4 / (2π × 0.289) ≈ 2.4e-4 rad, 0.014° peak. It does lift the white-noise fit, which `allan_variance.py` reads at τ = 1 s, on the bump: below the bump (τ = 0.05–0.3 s) the noise density is 3.1e-4 / 3.5e-4 / 1.3e-4 rad/s/√Hz for x / y / z, a gyro PSD of about 8e-8 instead of 1.0e-7. That is within the tool's stated accuracy, so `APP_GYR_PSD` stays at 1.0e-7.
- **Not covered:** temperature. The run spans 21.7–24.6 °C indoors, so field use may need the two bias random walks raised. The TM171 was not logged.

### Second Allan run (TM210, node ID 925, 2026-11-14/15)
- **Run:** same bench and IMU, logged on dietpi with the threaded-reader version of `tm210_allan_log.py` (Status as output, no requests): 30 min warm-up, then 9.6 h from 23:01 to 08:36, stopped by hand before the planned 12 h. QoS 5 throughout. 29.1 M raw packets, **0 missing**, 0 CRC errors, 0 reconnects. Converted with `--average 8 --max-fill-ms 2000` into one 9.61 h segment at 100 Hz with nothing filled (48 bytes skipped by the converter, not checked; probably a half-written packet at the end of `raw.bin` when the logger was stopped).
- **Conditions:** 25.1–26.4 °C, about 2 °C warmer than the first run and over a narrower span. No knock.
- **Analysis:** `allan_variance.py` with the default trim (30 s start, 60 s end), 9.6 h. With `--skip-start 7200` (7.6 h) the white noise is the same and both bias random walks stay within a factor of 2 (gyro bound ≤ 3.4e-7 on the worst axis, accel 3.3e-5). The 165 "possible motion" warnings are single noise peaks just above the 1 °/s threshold, as in the first run.

| | x | y | z |
|---|---|---|---|
| Gyro noise density, rad/s/√Hz (ARW) | 3.2e-4 (1.1 °/√h) | 3.7e-4 (1.3 °/√h) | 1.4e-4 (0.48 °/√h) |
| Gyro bias instability, upper bound | ≤ 1.4 °/h | ≤ 1.9 °/h | ≤ 0.62 °/h |
| Gyro bias random walk, upper bound | ≤ 1.4e-7 rad/s/√s | ≤ 1.8e-7 | ≤ 5.9e-8 |
| Accel noise density | 1.4e-3 m/s²/√Hz | 1.4e-3 | 1.5e-3 |
| Accel bias instability | 0.44 mm/s² | 0.67 mm/s² | 0.37 mm/s² |
| Accel bias random walk | 3.2e-5 m/s²/√s | 3.2e-5 | 2.4e-5 |

- **Against the first run:** the white noise agrees within 10 % on x and y (z 25 % lower), and the accelerometer figures within the tool's factor-2 accuracy. The gyro curve again still falls at the longest trusted averaging time (3,450 s); the tool now puts the +1/2 branch near 8,600 s, which needs about 72 h of static data. So the gyro bias instability and random walk are again upper bounds, lower than the first run's (worst axis 1.8e-7 against 6.8e-7): this record has no knock or settling, and the bound is set by how far the curve reaches, not by a measured random walk.
- **Suggested values** from the tool: gyro PSD 8.6e-8, accel PSD 2.1e-6, gyro bias random walk 2e-7 (upper bound), accel 4e-5. All match the values in use (table above) within the tool's accuracy, so `firmware/app/app_config.h` is unchanged. The gyro bias random walk in use (7e-7) stays above the bench bound on purpose; its field value comes from the temperature test ([TODO.md](../TODO.md), "gyro bias vs temperature").
- **Files** (on dietpi, not in the repo): `~/allan/tm210_20261114/` holds `raw.bin`, `imu.csv`, `allan_default.txt`, `allan_skip2h.txt`, `allan.pdf`, `allan_skip2h.pdf`.

**A measured comparison (TM171, not our TM210).** The AgOpenWeb-HAT evaluation (same document, §8.4) ran a 2 h Allan test on a TM171 on 2026-11-06, at 24–26 °C:

| | Measured on their TM171 | As an INSLIB PSD | Our datasheet start value |
|---|---|---|---|
| Gyro random walk x / y / z | 0.70 / 0.92 / 0.41 °/√h | 4.1e-8 / 7.2e-8 / 1.4e-8 (rad/s)²/Hz | 1.5e-8 (≈ 0.4 °/√h) |
| Accel noise density | about 145 µg/√Hz | 2.0e-6 (m/s²)²/Hz | 3.5e-6 (190 µg/√Hz) |
| Gyro bias instability | 1.0 / 0.8 / 0.4 °/h at 650 s | — | datasheet: <5.5 °/h |
| Accel bias instability | 0.03–0.10 mg | — | datasheet: 0.05 mg |

So the x and y gyro noise may be about 2× the datasheet-derived figure, and the accelerometer a little quieter. Their note says the TM171 outputs bias-corrected, filtered data, and they logged it at 20 ms spacing, so the figures are not directly those of a TM210 raw object at 800 Hz. They are a reason to run our own Allan test before trusting the start values, not replacements for them.

**The TM171 as a second IMU (planned 2026-11-20).** A TM171 (`docs/datasheets/TransducerM_TM151_TM171_Datasheet_EN_V116-R.pdf`: up to 800 Hz, calibrated raw data output, UART and USB-C, same EasyProfile protocol) goes in the front camera unit on the tractor's chassis, read by the unit's SBC. It de-rotates the flow, serves as a degraded backup for this TM210 on the cab roof, and gives the chassis roll that separates the cab's suspension roll for the ground point. See [front-camera-unit.md](front-camera-unit.md).

## Calibration steps
1. **Allan variance**: static log at 800 Hz, constant temperature, after a 30 min warm-up → `external/INSLIB/tools/allan_variance.py` → final PSDs and bias random walks.
   - **Length: 8–12 h.** The relative error of the Allan deviation at averaging time τ is about 1/√(2(N−1)), with N = T/τ windows. The random walk terms (τ = 1–10 s) are fine after minutes. At the bias-instability minimum (about 650 s on the TM171, above) 2 h gives N ≈ 11, about ±22 %. The bias random walk sits on the rising slope beyond it (τ ≈ 1000–3000 s), which needs 8–12 h for about ±10–15 %.
   - **Temperature:** the gyro drift of ±0.004 °/s/°C is about 14 °/h per degree, more than the bias instability, so a room that cools overnight shows up as false bias random walk. Log the Status temperature at 1 Hz beside the data.
   - **QoS:** the user guide's "Step 2 – Heat Up" keeps the module still for about 5 min until QoS reaches 5, and calls that "extremely important". Start the log only once QoS is 5 and the 30 min warm-up is over, and log QoS beside the temperature.
   - **Gaps:** over USB whole packets go missing when the host doesn't read in time (bench results below). Use the TM210 timestamps and split the log at gaps, or fill single missing samples. On dietpi the cause was the host: its tty holds only about 4 KB (about 70 ms of raw plus Status data), and an fsync to the SD card took up to 1 s, so with reads and disk writes in one loop each slow fsync lost packets (236 and 1074 in two 240 s tests on 2026-11-14, matched to slow fsyncs; probably also the first run's 14,041, which was logged without read timing). Since then `tm210_allan_log.py` reads the port in its own thread; the second run lost none in 9.6 h.
   - **Size:** 40 B × 800 Hz ≈ 115 MB/h of raw payload; with a Status packet after every raw packet (since 2026-11-14) about 202 MB/h as logged, 141 MB/h before.
   - **Tools** (`firmware/tools/`, tested against a simulated TM210 on dietpi, then used for the 12 h run above):
     - `tm210_allan_log.py --port <usb port> --out <new dir>`: reads the Setting object and, if needed, writes the logging target to RAM only (raw and Status output, 800 Hz, USB on; every other field copied). After that it sends nothing to the IMU: no Status requests, which since 2026-11-14 come as output instead, because steady requests stop the TM210's Setting replies ([imu-tm210-config.md](imu-tm210-config.md)). It logs every received byte to `raw.bin` (fsync every 10 s) and Status (temperature, QoS) once per second to `status.csv`. A separate thread does nothing but read the port, so a slow disk can't make the host drop packets; `timing.csv` (one line per second) and a `GAP` line per timestamp gap in `events.log` record the read timing, to tell a host stall from a loss upstream. The `--no-status` option of the older version is gone. It marks the record start in `meta.json` once the warm-up (`--warmup-min`, default 30) is over and QoS ≥ 5, and stops `--hours` (default 12) later. If the port drops out it reopens it and writes the target again. Uses pyserial if installed, otherwise termios. Run one instance per IMU.
     - `tm210_to_imu_csv.py <dir>`: converts to INSLIB's `imu.csv` (FRD SI plus temperature) from the record start. Timestamps are unwrapped. Gaps up to `--max-fill-ms` (default 200; use 2000 for an overnight USB log) are interpolated, because `allan_variance.py` assumes a uniform period. A longer gap or a restart starts a new segment. `--average 8` writes 100 Hz block means, so a 12 h file is about 440 MB of CSV instead of 3.5 GB; the Allan deviation for τ ≥ 10 ms is unchanged.
   - **Host:** miniPi lost 684 packets in 10 min over USB, the PC 1 (bench results below). Prefer the PC or dietpi.
2. **Multi-pose calibration** with `external/INSLIB/tools/inslib_imu_calib.py`, no fixture needed. This sets the fixed errors (bias, scale, misalignment) in `imu_acc_misalignment` / `imu_gyr_misalignment` / `imu_*_fixed_bias`. It also fixes the "units, not absolute" accelerometer magnitude.
   - **Method:** imu_tk (Tedaldi, Pretto, Menegatti, ICRA 2014); the maths is in `tools/inslib_imu_tk.py`. In every still pose the accelerometer sees only gravity. The fit finds the accelerometer bias and 3×3 scale/misalignment matrix M (`corrected = M·(raw − bias)`, 9 parameters) that make the corrected magnitude equal g in every pose. Only the magnitude enters, so the poses can point anywhere. The gyro M (9 parameters) comes from the turns: integrated across each turn, the gyro has to carry one pose's gravity direction onto the next. The gyro bias comes from the initial rest.
   - **Why not the old 6-position test:** INSLIB replaced it. Its (up + down) / 2 bias is exact only when each pair of positions is exactly opposite. The tool's notes put the error from an independent 1° placement error per position at about 0.11 m/s² of accelerometer bias, more than the bias itself. It also gave no gyro scale or misalignment without a rate table.
   - **Session:** keep the unit still for 20 s first (`--init-sec`; it sets the gyro bias, a noise estimate and the stillness threshold, so it must be undisturbed and longer than any pose). Then make at least 12 poses (imu_tk's floor) of about 4 s each (`--pose-sec`), in as many different orientations as possible, picking the unit up and turning it between poses. About 200 s in all (`--duration`). The temperature is written beside the result, which is valid only near that temperature (see the gyro bias vs temperature item in `TODO.md`).
   - **With the TM210:** the live modes (`--port`, `--udp`) need INSLIB's own board protocol, which the TM210 doesn't send, so record and solve offline. Record with `tm210_allan_log.py --hours 0.1` and wait for the record start (after the warm-up and QoS 5) before the 20 s rest. Convert with `tm210_to_imu_csv.py <dir> --min-minutes 0` (the default 10-minute minimum would drop the short session) and no `--average`, so the turns are integrated at 800 Hz. Solve with `inslib_imu_calib.py --csv <dir>`. The converter writes no `mag.csv`, so this run doesn't calibrate the magnetometer; that is step 4. Not yet tried.
   - **Mounting:** the result is the sensor's own calibration. The vehicle mounting (`mount_rpy_deg`, step 3) is composed into the same misalignment matrices with `imu_mount_compose()` (above).
   - **Why it matters:** scale and misalignment errors show in turns (a 0.5 % gyro scale error is 0.9° of heading after a 180° headland turn), and accelerometer bias shows as roll and pitch error (about 0.06° per 0.01 m/s²).
3. **Mount angles** on the vehicle: `external/INSLIB/tools/inslib_frame_align.py` or measure, set `mount_rpy_deg`.
4. **Magnetometer** (on the vehicle, with the TM210 in its final place): `external/INSLIB/tools/inslib_mag_calib.py` → `mag_misalignment`, `mag_fixed_bias`. The mag is already in use before this; the calibration removes the tractor's soft-iron distortion and most of its hard-iron offset, which the INS's bias estimate otherwise has to find while turning.

## Bench checks (after conversion, FRD)
Expected specific force: `f = (g·sinθ, −g·sinφ·cosθ, −g·cosφ·cosθ)` for pitch θ, roll φ.
- Level → acc ≈ (0, 0, −9.81). Done: the raw object is the gravity vector, so the conversion in the table is `(−ax, ay, az)` (results below).
- Nose up → acc_x > 0. Right side down → acc_y < 0.
- Turn right (clockwise seen from above) → gyr_z > 0. Roll right → gyr_x > 0. Nose up → gyr_y > 0. Done: all three as expected with `(gx, −gy, −gz)` (results below).
- Rate: 800 ± 20 packets/s, CRC error rate ≈ 0 over 10 minutes with the real cable length.
- Timestamp wrap: run > 72 min and check the unwrapped time stays monotonic.
- Run INSLIB's static auto-init on the bench: roll/pitch converge to the table angle, mode reaches ATTITUDE_ONLY without GNSS.

### Bench results (2026-11-12/13, TM210 over USB on miniPi and on the Windows PC)
The TM210 (node ID 925) was read over its USB port with a Python script: on miniPi as `/dev/ttyACM0` (termios, no pyserial) on 2026-11-12, and on the Windows PC as COM13 (pyserial) on 2026-11-13. For each run the target was written to RAM only (bit 18 = 0): raw only, 800 Hz, UART + USB on, silent time 0, baud unchanged. It was acknowledged and read back. A power cycle restores the flash settings.

- **Flash settings as found:** on miniPi, switches `0x0008a85b`: 50 Hz, raw + RPY, UART on, **USB output off** (bit 20 = 0), baud 115200, silent time 1. With USB output off the TM210 still answered Setting and Status requests over USB, but streamed nothing. On the PC the next day they read `0x001ea85f`: the same plus bit 2 (mag enable), bits 17/18 and **bit 20 = 1**, which is what a save from ImuAssistant leaves. Rate, outputs, baud and silent time were unchanged.

**Accelerometer sign: the gravity vector.** Raw values (TM210 frame, g), 1 s means, held still:

| Position | Host | ax | ay | az | Reading |
|---|---|---|---|---|---|
| Flat, top face up | miniPi | −0.049 | +0.058 | **−0.997** | z up and az < 0: gravity, not specific force |
| Flat, top face up | PC | +0.022 | +0.017 | **−0.997** | same |
| X arrow raised about 33° | miniPi | **−0.541** | −0.131 | −0.831 | gravity along −x |
| X arrow raised about 36° | PC | **−0.584** | +0.015 | −0.811 | same |
| Y arrow raised about 30° | miniPi | −0.037 | **−0.494** | −0.865 | gravity along −y |
| Y arrow raised about 33° | PC | −0.021 | **−0.548** | −0.830 | same |

|a| was 0.995–1.000 in every position. After `(−ax, ay, az) × 9.8158`, nose up gives acc_x > 0 (+5.3 m/s²) and right side down gives acc_y < 0 (−4.85 m/s²), as the checks above expect. The case markings and the data agree on a right-handed FLU frame.

**Gyro sign (PC, 2026-11-13): as expected, rad/s, FLU.** Rotations by hand, 1 s means:

| Motion | Dominant axis, 1 s means | FLU sign | FRD after `(gx, −gy, −gz)` |
|---|---|---|---|
| Y side raised about 45° (about +x) | gx +0.637, back −0.653 | + | roll right → gyr_x > 0 ✓ |
| X end lowered about 45° (about +y) | gy +0.249, +0.423, back −0.631 | + | nose up → gyr_y > 0 ✓ |
| Counter-clockwise from above (about +z), about 22° | gz +0.103, +0.220, +0.059 | + | turn right → gyr_z > 0 ✓ |

The accelerometer moved the same way at each step (ay to −0.66 when the Y side went up, ax to +0.66 when the X end went down). At rest the 1 s gyro means stay within ±0.001 rad/s.

**Rate and link, 10 min static over USB:**

| Item | miniPi (2026-11-12) | PC (2026-11-13) |
|---|---|---|
| Raw packets in 599 s | 480 548, mean 802.25 per host second | 481 225, mean 803.38 per host second |
| Full seconds | 794–803 | 789–816; one second 724 (the gap below) |
| CRC errors, skipped bytes, other objects | 0, 0, 0 | 0, 0, 0 |
| TM210 timestamp steps | 479 863 × 1 250 µs, **684 × 2 500 µs** | 481 222 × 1 250 µs, 1 × 2 500 µs, **1 × 96 250 µs** |
| Backward steps | 0 | 0 |
| Status (on request) | 17.2 °C, 802 Hz, QoS 5 | 17.8 → 18.5 °C, 802 Hz, QoS 4–5 |

- **Missing samples are whole packets, and their number depends on the host,** so they are most likely lost on the TM210's USB side, not skipped by the sensor: 684 single-packet steps on miniPi (whose `dwc_otg` USB already drops bytes from the UM982's CH340), 1 on the PC. Every byte that arrived was in a valid packet, and USB packets (64 B) don't line up with 49-byte EasyProtocol packets, so the loss happens before the data reaches the USB pipe. That fits an STM32 virtual COM port that drops a packet when the host hasn't collected the previous one yet. A cause, not a measured fact.
- **The one long gap on the PC** (96.25 ms, 76 packets) has the same signature: the timestamps jump forward, nothing is corrupted. A Windows host stall is the likely cause.
- **For the firmware:** the UART has no such host side, so the UART link test ([imu-tm210-config.md](imu-tm210-config.md), "Still to do" 1) will show whether the TM210 ever skips samples by itself. Until then, the 2 500 µs steps sit exactly at the firmware's gap limit (`GAP_US` 2500, gap only if longer) and are not flagged; INSLIB gets the right `dt` from the timestamp difference either way.
- **The TM210 clock runs a few tenths of a percent fast against the host:** about 802–803 packets per host second while its own timestamps step 1 250 µs (800 Hz), and the Status object reports 802 Hz. An estimate from host-second counting, not measured precisely. The firmware's offset-and-drift mapping to the Pico clock has to absorb it.
- **Not covered:** the TM210's UART, the run with the real cable length, and the timestamp-wrap run (> 72 min).

## Vibration check
The datasheet gives no g-sensitivity or vibration rectification figure, so measure it. The method and the reference levels come from the AgOpenWeb-HAT evaluation (same document, §8.3–8.4, a TM171 on a three-cylinder tractor, 2026-11-08); the results are theirs, not measured on our hardware.

- **What they measured on the tractor:** 18 mg rms at idle (peaks at 15, 22.5 and 45 Hz) and 43 mg rms at working revs (34.9, 52.2 and 204 Hz). At those levels the TM171's fused roll shifted by at most 0.14° and its gyro by under 4 mdps.
- **What they measured under heavy shaking** (a sander, 1.5–1.8 g rms at about 80 Hz, far above cab level): the TM171's fused roll ramped to +3.2° in 100 s, because the accelerometer's mean moves (vibration rectification). The gyro shift stayed under 25 mdps.
- **What it means here:** INSLIB uses the accelerometer for leveling and the vertical channel, so an accelerometer mean shift under vibration becomes a roll and pitch error. At cab levels it looks harmless; the mounting must keep the IMU away from the harsh levels. The 204 Hz component is well below the 400 Hz Nyquist limit of our 800 Hz sampling.
- **Bench test:** fix the IMU plate rigidly to the body of a random-orbit sander, not to its pad. Sequence: tap, 60 s still, 120 s shaking, 60 s still, tap, at two or three intensities and in two orientations. The plate does not tilt, so any roll change is error.
- **Tractor test:** engine off, idle and working revs for 5 min each at rest. The true roll is constant, so the shift when the engine starts is the error.
- **Pass criteria (theirs, a reasonable start):** roll mean shift ≤ 0.2° from still to shaking, roll standard deviation ≤ 0.3° in 1 s windows, gyro mean shift ≤ 0.1 °/s. Log the INS roll and the raw accelerometer mean, and the vibration level, so runs can be compared.
