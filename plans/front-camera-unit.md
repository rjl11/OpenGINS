# Front camera unit on the tractor: SBC, OV9281 and TM171 over Ethernet

> **Status: planned** (2026-11-20). Done: the `$PAOGI` ground-point change below, and the first part of the link: W5500 socket 2 on UDP 5800 takes one `flow_link` frame per datagram into the same path as frames on USB (`firmware/app/eth.c`, `field_io_usb_flow()`), and the Pi program sends them with `flow --udp HOST[:PORT]`. Clock sync done the same day (below, "Clock sync"): with it valid, the Pico times each frame by the unit's sensor timestamp, else by arrival as on USB. Not tested with a W5500 yet. Also done the same day: a camera preview web page in `flow` (port 8080) for aiming and focusing the camera at installation, linked from the Pico's status page (`pi/flow/README.md`, "Camera preview page"); run on dietpi with the live camera. Not done: the Pico → unit settings messages, everything on the TM171 and the rangefinder. The rangefinder is a TF-Luna, bought 2026-11-20 (below, "Rangefinder: TF-Luna").

## Context
The optical flow ([optical-flow-aiding.md](optical-flow-aiding.md)) was planned with the camera close to the Pico: a PIO UART for the frames, and trigger and strobe wires for the timing. It reaches the Pico over USB on the bench today, timed by the Pi's clock (2026-11-20).

On the tractor the camera goes at the **front**, on the chassis, where it is protected and away from the dust. That puts it 2–3 m ahead of the rear axle, the tractor's pivot, with 3–5 m of cable to the cab. The TM210 IMU and the GNSS antenna stay on the **cab roof**.

So the camera travels with its single-board computer (SBC) as a front unit, which also gets:
- a **TM171 IMU** (`docs/datasheets/TransducerM_TM151_TM171_Datasheet_EN_V116-R.pdf`);
- a downward rangefinder.

The unit talks to the Pico over Ethernet through the W5500. This plan covers:
- the link,
- the TM171's three jobs: de-rotating the flow at the camera, a backup for the roof TM210, and the cab-suspension roll for the ground point,
- what the front mounting means for the flow,
- the decision to send the true ground point in `$PAOGI`.

All figures marked "estimate" are estimates, not yet measured.

## Hardware
| Part | Notes |
|---|---|
| SBC | ROCK 4SE (dietpi) for the bench; any SBC with a CSI port and Ethernet for the tractor. Builds `pi/flow/` as today |
| Camera | OV9281 (CAM-MIPI9281RAW-V2), free-running, global shutter. Behind a hooded window |
| IMU | TM171: 800 Hz maximum output rate, calibrated raw data output, UART (up to 921600 / 1 Mbps) and USB-C, 5 V at 80 mA, 40×34×12.6 mm, no IP rating (datasheet above). Same EasyProfile protocol as the TM210 ([imu-tm210-config.md](imu-tm210-config.md), "Cross-check: the TM171 in the AiO v4.5 firmware"). `firmware/tools/tm210_ep.py` already handles its Setting layout |
| Rangefinder | **TF-Luna** (Benewake, bought 2026-11-20 from Waveshare): single-point ToF lidar, 0.2–8 m, UART or I2C at 3.3 V, 5 V at ≤ 70 mA. Next to the lens, aimed at the patch the camera sees, with its own opening. Details below, "Rangefinder: TF-Luna" |
| Night light | 850 nm IR LEDs pulsed by the camera's strobe output, wired at the unit (the OV9281 is mono with no IR filter) |
| Power and data | One PoE cable from the cab: a PoE splitter or HAT at the unit, a 12 V PoE injector or switch in the cab. Sealed M12 D-coded connectors |
| Network | A small switch in the cab joins the Pico's W5500, the front unit and the AgOpenWeb tablet on 192.168.5.x |
| Housing | **Bench / field-test box designed 2026-11-25**, not yet printed: [hardware/front_unit/](../hardware/front_unit/README.md) (OpenSCAD). Vented, open lens aperture, no PoE. It holds the ROCK 4SE above the camera, the TM171 and the TF-Luna, with the TF-Luna's beam 31.5 mm from the optical axis. It has ears with M6 slots and holes for M5 screws. Rebuilt the same day from the user's caliper readings, Radxa's ROCK 4SE drawing (the RK3399 is on the underside, above the camera) and SYD's TM171 STEP models. No fan and no heat-set inserts (screws thread into the plastic). Since 2026-11-26 the camera, TM171 and TF-Luna sit on heat-staked printed pins, the camera PCB between locating corners. There is a Ø22 lens hole to turn the focus with the camera in place, and 6.5 mm between the camera and the TF-Luna for its cable. The sealed tractor housing (hooded window, PoE, M12) is still to design |

Mount the unit rigidly to the chassis: the front weight carrier or the bumper, looking down, clear of a front linkage implement, a loader and the weights.

## Link: UDP over the W5500
- **Socket:** W5500 socket 2, UDP port 5800 (`APP_ETH_FRONT_PORT`). The W5500 plan has sockets 0 and 1 for `$PAOGI` and RTCM, and 3 for AgIO's scan on 8888.
- **Frames:** one UDP datagram per message, carrying a whole `common/flow_link.h` frame with its CRC.
  - The Pico checks it with the same code as the USB path (`field_io_usb_flow()` in `firmware/app/field_io.c`).
  - UDP, not TCP: a late frame is worthless, so it shouldn't be resent.
- **Clock sync, instead of trigger and strobe wires:**
  - About once a second the Pico sends a time request and notes its `time_us_64()`. The unit answers with its `CLOCK_MONOTONIC` time, the clock V4L2 stamps frames with. The Pico notes the reply's arrival.
  - From the replies with the shortest round trip, as NTP does, the Pico keeps the offset between the clocks and its drift.
  - Expected accuracy 0.1–0.3 ms (estimate; W5500 polling over SPI adds the jitter).
  - **Done 2026-11-20.** Messages `FLOW_MSG_TIME_REQ` (Pico → unit: a sequence number) and `FLOW_MSG_TIME_RESP` (unit → Pico: the number, the unit's 64-bit receive time and its turnaround) in `common/flow_link.h`, with the Python mirror and the three tests.
    - Pico: `firmware/app/clock_sync.c` (host test `firmware/tools/test_clock_sync.c`). It keeps 32 replies, uses those within 300 µs of the shortest round trip, and fits a line through them (offset and drift; with less than 5 s of them the mean). It starts over when a reply is 50 ms off the line (the unit rebooted) and is not valid 10 s after the last reply. `eth.c` asks once the unit has sent a datagram, 4 times a second for the first 8 requests, then once a second, and reads the reply on every main-loop pass while one is due, so its arrival time isn't held up by the 1 ms poll.
    - Unit: `flow --udp` answers on the socket it sends from. The receive time is the kernel's packet timestamp (`SO_TIMESTAMPNS`, moved from CLOCK_REALTIME to CLOCK_MONOTONIC), so the wake-up delay of its thread drops out. With `--frames` it does not answer (made-up timestamps), and the Pico keeps timing by arrival.
    - Frame time: `flow_nav.c` maps `pi_time_us` (low 32 bits, unwrapped against the last reply) onto the Pico clock and adds `APP_FLOW_SYNC_TS_TO_MID_US`, the timestamp-to-exposure-middle constant (0 until measured). The V4L2 path uses buffer timestamps on CLOCK_MONOTONIC; libcamera's `SensorTimestamp` clock is still to check before the libcamera path relies on it.
    - Loopback on the ROCK 4SE (`pi/flow/tools/clock_sync_loopback.py` playing the Pico, the camera running at 100 Hz, 3 runs): shortest round trip 305–350 µs, offset at the shortest round trip 58–82 µs from the true 0. Against the Pico over the W5500: not yet measured.
    - Status: the `clock sync` line under `ETH` on USB, `sync_*` in `eth` in `/status.json`, `clock_synced` in the flow figures.
  - The free-running path in `firmware/app/flow_nav.c` takes this offset in place of its arrival-minimum guess, and `APP_FLOW_FREE_LATENCY_US` goes away. The only constant left is the unit's own timestamp-to-exposure-middle offset, which it knows from the sensor's line time and exposure.
  - No gap-pulse matching, and GP10/GP11 stay free. The wired trigger and the PIO UART remain the option for a camera next to the Pico.
- **Pico → unit:** the settings the unit needs (camera height, image direction, TM171 mounting) and a heartbeat. The time requests are the first Pico → unit message (2026-11-20). **Settings and heartbeat written 2026-11-21:** `FLOW_MSG_UNIT_CFG` (28 bytes: a sequence number, the camera height, the image direction, the TM171 mounting roll / pitch / yaw against the body, and whether flow is on), once a second whether anything changed or not, so it is also the heartbeat.
  - Pico: `settings_unit_cfg()` builds it from the web settings; the TM171 mounting is new in settings version 11 (`front_imu_roll_deg` / `_pitch_deg` / `_yaw_deg` under "Optical-flow camera", defaults `APP_FRONT_IMU_*`, 0 = level, x forward). `eth.c` sends it to the front unit over UDP (never in the same pass as a time request; `settings sent` on the `clock sync` line, `unit_cfg_sent` in `/status.json`). On USB, while flow frames arrive there, `field_io.c` prints it as a `flowcfg <hex>` line, for bench tests.
  - Unit: `flow` takes it from the UDP socket or the Pico's USB output (those lines are not copied to stdout). From the first one on it replaces `--height` and `--cam-yaw`, and the TM171 mounting turns the gyro into body axes before the de-rotation (the tilt is the TM171's less its mounting roll and pitch, small angles). A change is printed once; the 5 s line shows the last sequence number and its age, or "no settings from the Pico yet".
  - Host-tested (the example frame in the three link tests, the mounting rotation in `test_tm171`). On the bench (2026-11-21, dietpi's Pico flashed with it, `flow --usb --tm171`, over USB only): the version 10 settings record was read with the mounting at 0; the unit printed the Pico's settings once and then got one a second (0.9 s old on every 5 s line), with no `flowcfg` lines in its output, while the Pico used all 2102 camera velocities. Over UDP not yet run (no W5500 fitted), and no change of a setting tried yet.
- **Messages to add to `flow_link.h`** (with the Python mirror and the three tests, in one commit):
  - time request / response;
  - camera velocity (already de-rotated, below), with the rangefinder height: **done 2026-11-21**, `FLOW_MSG_CAM_VEL` (36 bytes: the camera's mean velocity over the frame interval in camera axes, its image-noise 1-sigma, the vertical height used with its 1-sigma, and `FLOW_F_RANGE` when that height is from the rangefinder), with the Python mirror and an example frame checked by the three tests. Sent by `flow --tm171` (step 2);
  - TM171 sample batch (8–10 samples per datagram).
- **Bandwidth:** flow 3.5 kB/s at 100 Hz. TM171 samples at 800 Hz about 30 kB/s (estimate, about 36 bytes a sample). Far below what the W5500 and 100 Mbit/s carry.

## The TM171's three jobs
### 1. De-rotating the flow at the camera
The cab sits on its suspension and the camera on the chassis, so they are not one rigid body. The cab rocks against the chassis at about 1–3 Hz (estimate), slower than the 0.1 s speed averaging can remove. De-rotating the chassis camera's flow with the roof gyro is wrong by height × (ω_cab − ω_chassis): for example 1 m × 0.1 rad/s = 0.1 m/s (estimate).

- **On the unit:** it integrates the TM171 gyro over each frame interval, in its own clock with no sync error, and removes the rotation. It sends the camera's velocity in body axes with the rangefinder height.
- **On the Pico:** `flow_nav.c` gets a path for these messages. It skips the de-rotation, keeps the lever arm (yaw is shared by cab and chassis, so the roof gyro serves for `ω × r`), the tilt gate and the averaging. **Done 2026-11-21:** `flow_nav_cam_vel()`, sharing the timing (clock sync or arrival), the gates and the 10 Hz window with `flow_nav_message()`; the unit's `height_sd_m` sets the height part of the covariance (the web page's relative error when 0). `field_io.c` routes the message from USB and UDP, and shows it on the `FLOW` status lines and in `/status.json` (`cam_vel_msgs`, `unit_height_m`, `unit_height_ranged`, `ranged`). Host-tested in `firmware/tools/test_flow_nav.c` (a turn with the camera 1 m ahead, sideways motion, a missing height).
- The de-rotation formulas stay as in `flow_nav.h`, so the unit's C++ can share them. They move into a header both sides include, or are mirrored and tested on the same cases as `firmware/tools/test_flow_nav.c`.

### 2. Backup IMU for the roof TM210
- **Path:** the unit streams TM171 samples in batches. The Pico puts them in a time base through the clock sync and into the same core 0 → core 1 IMU queue as the TM210 (`firmware/app/nav_core1.c`).
- **Switching:** core 1 already stops INSLIB 50 ms after the last IMU sample (mode NONE). When the TM210 stays lost for longer, a time to choose, core 0 switches the source. Core 1 then re-initialises with the TM171's mounting rotation and lever arms and re-aligns. Its lever arm to the antenna is about 2.5 m forward and 2 m up.
- **Limits:**
  - The antenna moves against the TM171 with the cab sway, by about 3 m × 1–2° = 5–10 cm (estimate).
  - The Ethernet path adds delay and may lose a datagram. A lost batch must show as an IMU gap, never be bridged.
  - So it is a degraded backup that keeps steering alive, not an equal of the roof IMU.
- **Not planned:** both IMUs in INSLIB at once. `nav_suite` takes one IMU.
- **Written 2026-11-21:**
  - Link: `FLOW_MSG_IMU_BATCH` (up to 8 samples, 120 bytes; `FLOW_LINK_MAX_PAYLOAD` raised from 64 to 128 for it): the first sample's unit time, a batch number (a gap is a lost batch), and per sample its offset in µs, the gyro (2⁻¹² rad/s, ±8 rad/s) and the specific force (2⁻¹⁰ m/s², ±32 m/s²) in the TM171's own axes as FRD, fixed point; clamped values and samples lost on the unit are flagged.
  - Unit: `flow --tm171` with `--udp` or `--usb` sends every 8 raw samples as a batch (100 a second), on the same link as the flow, from the TM171's thread. The specific force is the TM171's gravity output converted like the TM210's (`(-x, y, z)` × g, g = 9.80665 here).
  - Pico: `firmware/app/front_imu.c` maps the batch onto the Pico clock (the clock sync when valid, else the smallest arrival offset, as for free-running flow frames; `APP_FRONT_IMU_FREE_LATENCY_US`), turns it into body axes with the mounting, and counts lost batches. `main.c` (`imu_source_poll()`) switches core 1 to it when the TM210 has been lost for `APP_FRONT_IMU_BACKUP_AFTER_MS` (1 s) and the TM171 is live, never in the first 10 s after boot, and back once the TM210 has run for `APP_FRONT_IMU_BACKUP_BACK_MS` (5 s). On the TM171 the filter gets the antenna lever arm minus the TM171's position from the roof IMU (new settings `front_imu_lever_*`, version 12), none of the TM210's learned calibration and no magnetometer. Web setting `front_imu_backup` switches it off. Status: the `source` line under `IMU`, `imu.source` and `imu.front` in `/status.json`.
  - Host test `firmware/tools/test_front_imu.c` (timing within 12 µs free-running, the clock-sync path, the mounting, lost batches, the tilt). Not yet run against a Pico with this firmware, nor with the TM210 unplugged.

### 3. Cab-suspension roll for the ground point
Cab roll = roof roll (INS, TM210) − chassis roll (TM171).

The rear axle and the hitch, which AgOpenGPS steers by, are on the chassis; the antenna is on the cab. The ground point below the antenna is therefore:
- the antenna moved down by the cab's roll over the height above the cab's roll centre,
- then by the chassis roll over the roll centre's height above the ground.

That replaces one roll over the whole antenna height. At 3 m, 1° of cab sway taken as chassis roll puts the ground point 5 cm sideways (estimate). Pitch works the same way, fore and aft.

- **Where:** `firmware/app/ground_point.c` (today one attitude and one height, `ant_height_m`), with a new setting for the roll-centre height (to find on the bench).
- **The chassis attitude:** from the TM171's own fused roll and pitch (it outputs roll, pitch and yaw), or from its gyro and accelerometer through a small AHRS on the Pico. **Chosen: the small filter on the Pico**, from the same batches as the backup IMU, in our own FRD conversion (the TM171's Euler signs are not checked on all axes): gyro propagation pulled towards the accelerometer's level with a 1 s time constant, the accelerometer used only within 5 % of 1 g (`APP_FRONT_TILT_*`, estimates). Lateral acceleration in a long turn still biases it; to check on the tractor.
- **Written 2026-11-21:** `ground_point_cab()` in `ground_point.c` goes from the antenna to the roll centre along the cab's attitude (the roof INS) and on to the ground along the chassis roll and pitch with the cab's yaw; host-tested (a 2° cab sway moves the point by the 2 m above the roll centre only; cab and chassis on the same slope give the one-roll point; a chassis pitch leans the lower part). `best_position()` uses it on the tractor when the new setting `cab_roll_centre_m` (version 12, 0 = off) is above 0 and below the antenna height, the INS attitude passes the tilt gate and the chassis tilt is fresh; the status shows the chassis roll and pitch, `/status.json` `position.cab_roll`. The roll-centre height is still to find on the tractor.
- It reaches AgOpenGPS / AgOpenWeb through the ground point in `$PAOGI` (next section).

## Decision: `$PAOGI` sends the true ground point (2026-11-20)
- **The rule:** with `ant_height_m` above 0 (tractor), `$PAOGI` lat / lon / alt are the ground point below the antenna from `best_position()` in `firmware/app/main.c`. With `ant_height_m` 0 it is the antenna.
  - Done 2026-11-20: `paogi_in_t` carries `lat_deg` / `lon_deg` / `h_ell_m`, filled from the ground point when there is one.
  - The altitude is then the ground's height above MSL.
  - When the INS attitude fails the tilt gate, the ground point goes straight down (no roll compensation), as for the pole.
- **On the AgOpenGPS / AgOpenWeb side:**
  - Set the antenna height to 0, so their own roll compensation adds nothing.
  - Keep the antenna's forward and sideways offsets from the pivot: the point is still under the antenna.
  - The roll field still carries the true roof roll.
- **Why:** this device knows the attitude, and later the split between cab and chassis roll. One roll times the antenna height on the host can't make that split.
- **Bench checks:** with the antenna height 0 in AgOpenWeb, the shown position must not move when the unit is rolled; whatever else AgOpenWeb uses the roll for (display, a roll correction for steering) still works.

## Rangefinder: TF-Luna
Bought 2026-11-20 (Waveshare's [TF-Luna wiki](https://www.waveshare.com/wiki/TF-Luna_LiDAR_Range_Sensor)). Figures below are from Benewake's product manual, `docs/datasheets/SJ-PM-TF-Luna_A05_Product_Manual.pdf`, and its two-page datasheet, `docs/datasheets/SJ-GU-TF-Luna_A05_Datasheet.pdf`; none measured here yet. The datasheet adds: the 0.2–8 m / 0.2–2.5 m ranges hold outdoors at 90 klux too (both at 25 °C on a white board); output 1–250 Hz (500/n); ≤ 0.35 W; I2C slave up to 400 kbit/s at default address 0x10 (0x08–0x77); storage −20 to 75 °C. It gives the height as 12.5 mm where its own drawing, the manual and Waveshare's wiki show 13.5 mm over the lens rims (12 mm body); the housing uses the drawing.

What Waveshare's wiki adds to Benewake's documents (read 2026-11-25; its spec table repeats the datasheet):
- **Bench signs:** when powered, a faint red glow shows in the emitting lens seen from the front. It gets slightly warm in continuous use; Waveshare calls that normal.
- **Supply:** 3.7–5.2 V only, no 3.3 V version. The logic stays 3.3 V.
- **Pin 6 in I/O (switching) mode:** high (3.3 V) or low by distance with a hysteresis band, near and far levels; in I2C mode with switching off it is data-ready. Not used here.
- **Several units:** two side by side whose spots overlap on one object don't interfere; more than two do, and two facing each other interfere badly. Matters if a second TF-Luna (UAV, rear unit) ever looks at the same ground.
- **Host tools:** Benewake's Windows "TF host computer" program shows Dist, Strength and the valid points per second over a USB-TTL adapter; with format 0x06 (mm) it still labels the values cm. Its demo package (Pi C / Python, Pico MicroPython, Arduino, ESP32; UART and I2C reading) adds nothing to `pi/flow/src/tfluna.cpp`.
- **Pi wiring in its examples:** UART on GPIO14/15 with the serial console turned off, I2C on GPIO2/3. The ROCK 4SE's header differs ([hardware/front_unit/README.md](../hardware/front_unit/README.md), "Header wiring").

### What it is
- **Principle:** phase-shift ToF with an 850 nm VCSEL (Class 1), FoV 2° (theoretical). It ranges 500 times a second internally; at an output rate of 500/n Hz each value is the mean of the last n ranges (100 Hz default: 5 ranges, so a value describes about the last 10 ms).
- **Range:** 0.2–8 m on 90 % reflectivity, 0.2–2.5 m on 10 % (dark soil is nearer the second). Below 0.2 m (blind zone) and above 8 m the value is unreliable. Ambient light up to 70 klux.
- **Accuracy:** ±6 cm from 0.2 to 3 m, ±2 % beyond; 1 cm resolution. Repeatability at 100 Hz depends on the signal strength (Amp): σ 3 cm at Amp 100–200, 2 cm at 400, 1 cm at 1000, 0.5 cm at ≥ 2000.
- **Spot:** diameter 2 · D · tan 1° = 3.5 cm at 1 m. When the spot falls on two surfaces (stubble tips and the soil between them, a crop row edge) the value is a mix of both, not either one.
- **Electrical:** 3.7–5.2 V, ≤ 70 mA average, 150 mA peak, **no reverse-polarity or over-voltage protection**. Logic LVTTL 3.3 V. 6-pin connector: 1 +5 V, 2 RXD/SDA, 3 TXD/SCL, 4 GND, 5 interface select (open or 3.3 V = UART, GND = I2C, read at power-on), 6 data-ready in I2C mode.
- **Environment:** −10 to 60 °C operating (a winter morning on a tractor can be below that), 35 × 21.25 × 13.5 mm, < 5 g, no IP rating.

### Interface: UART or I2C on the unit
- **Both are written; on the ROCK 4SE it is I2C (2026-11-25).** The ROCK 4SE's header has two UARTs, and the second is the debug console that U-Boot also uses: a sensor streaming into it from power-on can stop the autoboot. With the TM171 on UART4, the TF-Luna goes on I2C7 and the console stays as it is. The UART path stays for a USB-serial adapter on any SBC, and for the one-time save below. Wiring: [hardware/front_unit/README.md](../hardware/front_unit/README.md), "Header wiring".
- **UART:** pin 5 left open. 115200 8N1 by default, settable from 9600 to 921600. The default 9-byte frame at 100 Hz is 900 B/s, so 115200 has room even at 250 Hz. On a USB-serial adapter it moves to any SBC unchanged. The Pico doesn't see the sensor either way: the unit reads it and puts the height in its messages.
- **I2C** (manual §6.3 and appendix III): pin 5 at GND before power-on, address 0x10, up to 400 kHz, firmware 1.0.7 or later. A register map: distance in cm only (no mm format), Amp, temperature in 0.01 °C, a tick and an error word at 0x00–0x09; mode (0x23), enable (0x25) and rate (0x26) are written at every start and read back after 100 ms, not saved. Pin 6 goes high when a new result is in the registers and low after any read, and the manual says to read on that edge, since a read during the update can tear: so pin 6 goes to a GPIO, and the reader reads the registers on each rising edge, stamped by the kernel on `CLOCK_MONOTONIC`. That stamp replaces the UART's estimate from the bytes' arrival; the sample time is still the edge less half an output interval (the averaging). A read whose tick hasn't changed is dropped.
  - **The filters can't be set over I2C:** no register holds the filter bitmap (ID 0x39). Before the sensor goes to I2C it is set up once over UART and saved (`tfluna --port … --save`: filters off, mm, 100 Hz, output on, then ID 0x11), the one exception to never saving. Saved on our sensor 2026-11-25 (firmware 3.6.0): after a soft reset the bitmap reads back `00` over UART, so it is in flash. The I2C reader can't read the bitmap back; that the saved bitmap applies in I2C mode is not yet checked on the sensor (a step in height, filters on and off).
- **Data frame** (9-byte/cm, format 0x01, default): `59 59 Dist_L Dist_H Amp_L Amp_H Temp_L Temp_H Sum`, the sum being the low 8 bits of bytes 0–7. Dist in cm; Temp in °C = raw / 8 − 256. Format 0x06 is the same frame with Dist in mm; whether that is finer than the 1 cm resolution or the same value × 10 is to see on the bench. The reader uses it.
- **Commands:** `5A Len ID payload Sum`. The device doesn't check the sum unless told to (ID 0x08), but always sends a valid one. Settings take effect at once and are lost at power-off unless saved (ID 0x11).
  - The unit sends its settings **at every start and never saves them**, as the Pico does with the receiver: frame rate (ID 0x03, 2 bytes LE, 500/n Hz only; 0 = trigger mode), output format (ID 0x05), filter bitmap (ID 0x39: bit 0 Kalman, bit 1 median, default both on), Amp threshold (ID 0x22). Read back with ID 0x3F, version with ID 0x01 (log it at start).
  - Trigger mode (rate 0, then `5A 04 04 00` per measurement) would let the unit range once per camera frame. Not planned: the free-running 100 Hz average is simpler, and a range per frame doesn't need tighter timing (below).
- **Its own filters:** the built-in Kalman and median filters add lag of unknown length. The reader turns them off (ID 0x39, bitmap 0) and filters on the unit, where the lag is known; compare both on the bench (`tfluna --device-filter`).

### Validity and filtering on the unit
- A value is invalid when Amp < 100, Amp = 65535 (overexposed) or Amp > 32768 (ambient overload, e.g. low sun straight into it), or Dist < 20 cm. Below the Amp threshold the device outputs Dist = 0 (its "dummy distance").
- Dual-frequency mode (default, ID 0x29) has no wrap-around within 65 m but can be off by tens of metres when Amp is below 100. The Amp gate above throws those out; single-frequency mode wraps at 15 m, which can't happen when looking at the ground from 1–3 m, so it is the safer choice. Bench check before switching.
- On the unit: a short median over the samples in the camera frame interval and the one before it, the valid count, and a hold of the last value for up to about 0.5 s (estimate) before the height is marked missing. With no valid range the flow keeps the fixed mounting height and the learned scale ([optical-flow-aiding.md](optical-flow-aiding.md), "Height above ground").

### Timing
- Each sample is stamped on `CLOCK_MONOTONIC` at its first byte's arrival (worked back from the read time and the bytes after it, 87 µs a byte at 115200), minus half the averaging window (5 ms at 100 Hz, estimate of the delay inside the sensor, to check against a step in height on the bench). Over a USB-serial adapter add its latency (about 1 ms, estimate; an FTDI needs its latency timer at 1 ms).
- **Done 2026-11-20 (built and host-tested on dietpi, run against a fake sensor on a pseudo-terminal; on 2026-11-25 `tfluna` ran on the real sensor through a USB-serial adapter on dietpi: setup took, 100 % ok, 97–98 Hz with no lost frames, range not yet checked against a tape measure):** `pi/flow/src/tfluna.cpp` (protocol, validity, the range per frame), `tfluna_reader.cpp` (setup at start and the read thread), `flow --lidar DEV` (range per frame in the dry-run line, the 5 s stats and the preview page, `--lidar-log`), the bench tool `tfluna`, and `tests/test_tfluna.cpp` (`pi/flow/README.md`, "TF-Luna rangefinder"). The range is not yet in a message to the Pico, nor corrected for tilt.
- **I2C done 2026-11-25 (built without warnings and host-tested on dietpi; no sensor connected):** `src/i2c_gpio.cpp` (register reads over `I2C_RDWR`, pin 6's rising edges through the GPIO uAPI v2 with a pull-down), the I2C mode in `tfluna_reader.cpp`, `flow --lidar-i2c DEV --lidar-drdy CHIP:LINE`, `tfluna --i2c … --drdy …` and `tfluna --port … --save`; tests for the register decode, the GPIO spec and the save command. Checked on dietpi: the data-ready line is requested as user `dietpi` and sees no edge with nothing on it, a missing device gives the "no answer" hint, and the option errors. Not yet run against the sensor.
- The height changes slowly next to the frame rate (the chassis pitches at a few Hz at most), so the unit takes the sample nearest to each frame's exposure middle. A few milliseconds of error don't matter.

### Height from the range
- The TF-Luna measures along its beam. The flow needs the height above the ground plane along the vertical: h = range × cos(roll) × cos(pitch) for a beam that is vertical when the chassis is level, with roll and pitch from the TM171 ([its third job](#3-cab-suspension-roll-for-the-ground-point)). The mounting tilt of the beam against the TM171 is a setting, measured once.
- Mount it so that its beam hits the middle of the camera's view: parallel to the optical axis, as close to the lens as the housings allow (a 3 cm offset is under 1° of view at 1 m). The camera then sees the ground the rangefinder measures.
- The height goes in the camera-velocity message (above, "Messages to add") as a separate field with its own valid flag, so the Pico can log it and choose between it and the setting. As built, the unit chooses: `height_m` is the height it scaled by, and `FLOW_F_RANGE` says whether it came from the range; the Pico shows it and uses its 1-sigma.

### Two 850 nm sources in one housing
- **The camera sees the TF-Luna's spot.** The OV9281 has no IR filter and the spot is 850 nm, the wavelength of the night LEDs. A bright spot that stays at the same place in the image while the ground moves past is a feature with zero flow, which pulls the speed towards zero. To check on the bench in the dark and in daylight. If it shows: mask the spot's image region in the flow (it moves along a line in the image with the height, because of the offset between beam and lens), or range between exposures in trigger mode.
- **The night LEDs and the TF-Luna:** the LEDs are pulsed, not modulated at the TF-Luna's frequency, and far below 70 klux, so no effect is expected (estimate). Check Amp and the range with the LEDs on and off.
- **Window:** the manual warns against anything transparent in front of it (glass, water). Reflections from a window in front of a ToF sensor give false short ranges. So the TF-Luna gets its own opening in the hood, not the camera's window; dust and mud on its lens lower Amp, which the status shows.

## Front mounting and the flow
- **Lever arm:** `flow_nav.c` moves the camera velocity to the IMU point. At 2.5 m ahead, a headland turn at 0.3–0.5 rad/s puts 0.75–1.25 m/s of sideways speed at the camera, and a 10 cm arm error gives 0.03–0.05 m/s (estimate). Measure the arm to about 5 cm, or learn it under RTK later.
- **Height:** at the front it changes with the chassis pitch: Δh = 2.5 m × 1° = 4.4 cm, 4–5 % of the scale at 1 m height (estimate). Lifting a heavy rear implement takes weight off the front axle and lifts the nose; standing crop changes it too.
  - The rangefinder measures the height the flow sees, frame by frame.
  - A slow scale against the RTK speed, learned like the wheel pulses per metre (`firmware/app/speed_ppm.c`), takes out what is left.
- **View:** ground ahead of the front wheels, uncrushed stubble or crop. Over standing crop the flow follows the canopy, which the rangefinder also follows.
- **Gain:** 2.5 m ahead of the pivot, the camera's sideways speed is ω × 2.5 m. That is a direct measurement of yaw rate and slip, which the body-velocity input of our INSLIB fork carries (REQ-NAV-096, [optical-flow-aiding.md](optical-flow-aiding.md), "INSLIB integration"): `flow_nav.c` moves the velocity to the IMU point with the gyro and keeps its sideways part.

## Steps
1. TM171 on the SBC: read it at 800 Hz on UART or USB, check its raw output, its delay and its Allan variance (`firmware/tools/tm210_allan_log.py`; `TODO.md`). **Reader done 2026-11-21** (`pi/flow/src/tm171*.cpp`, the bench tool `tm171`, `tests/test_tm171.cpp`): our TM171 is set up and saved to flash (raw, Status and RPY at 800 Hz on USB), 802–804 packets/s each with no CRC errors on dietpi, and boots from flash with them (checked after a replug). Its delay and Allan variance are still to measure.
2. De-rotation on the unit with the TM171 gyro, the camera-velocity message, the `flow_nav.c` path for it. Bench test over USB first (`flow --usb`, as today). The message and the Pico path are done (2026-11-21, host-tested), and so is the unit's side: `flow --tm171` de-rotates each frame pair with the TM171 gyro, scales by the range (or the set height over the TM171's tilt) and sends `FLOW_MSG_CAM_VEL` (2026-11-21, run on dietpi standing still: every frame pair converted, ±0.003 m/s at rest). Into the Pico over USB (2026-11-21, `flow --usb --tm171`, 20 s standing still): all 2054 camera velocities used, 10 Hz samples to INSLIB at 0.00 m/s; INSLIB didn't fuse them because that Pico had no GNSS and its INS wasn't running. Still to check: the signs with the camera moving, and the fusion with the INS running (GNSS fitted).
3. W5500 on the Pico ([ethernet-w5500-agopenweb.md](ethernet-w5500-agopenweb.md)), socket 2, the clock sync, flow over UDP. Code for socket 2, flow over UDP and the clock sync written 2026-11-20; not yet run against a W5500.
4. TF-Luna on the unit: a reader in `pi/flow/` (settings sent at start, the 9-byte frame, the Amp gate, timestamps), logged next to the flow (written 2026-11-20, `flow --lidar` and `tfluna`); bench checks of its accuracy over soil, its own filters on and off, and the 850 nm spot in the camera image. Then the height in the message, and the scale learned under RTK.
5. Front mount on the tractor, PoE, the lever arm measured (the bench / field-test box in [hardware/front_unit/](../hardware/front_unit/README.md) comes first: print, fit, and check the image corners and the TF-Luna against the camera); drive against RTK. Aim the camera and set the focus with `flow`'s preview page (`http://<unit>:8080/` from the tablet on the cab switch, or the "Front camera" chip on the Pico's status page).
6. TM171 batches to the Pico, the backup switching, a test with the TM210 unplugged. Written 2026-11-21 (above, "Backup IMU for the roof TM210"); the TM210-unplugged test is still to do.
7. Chassis roll and the cab-roll ground point; bench check of the roll-centre height and of AgOpenWeb with an antenna height of 0. Written 2026-11-21 (above, "Cab-suspension roll"); the roll-centre height and the AgOpenWeb check are still to do.

## Verification
- **De-rotation:** standing still with the engine running and the cab rocking, the camera velocity stays near zero. Compare the roof gyro against the TM171 gyro to see the cab sway.
- **Clock sync:** the offset's spread over minutes, and the residual of the flow speed against RTK as a function of an added delay, whose minimum should be at zero.
- **Backup:** unplug the TM210 while driving under RTK; the position error after the switch, and how long the re-alignment takes.
- **TF-Luna:** against a tape measure at 0.5, 1, 1.5 and 2 m over dry and wet soil, stubble and grass, with Amp, the valid share and σ per surface; its delay from a step in height (a board pulled away under it); the camera image in the dark with the LEDs off, to see whether the spot shows and whether the flow standing still over it reads zero.
- **Ground point:** roll the cab by hand on its suspension with the chassis still. The ground point must not move with the cab-roll correction, and moves by the roll-centre geometry without it.
