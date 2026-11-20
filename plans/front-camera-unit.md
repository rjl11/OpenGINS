# Front camera unit on the tractor: SBC, OV9281 and TM171 over Ethernet

> **Status: planned** (2026-11-20). Done: the `$PAOGI` ground-point change below, and the first part of the link: W5500 socket 2 on UDP 5800 takes one `flow_link` frame per datagram into the same path as frames on USB (`firmware/app/eth.c`, `field_io_usb_flow()`), and the Pi program sends them with `flow --udp HOST[:PORT]`. Clock sync done the same day (below, "Clock sync"): with it valid, the Pico times each frame by the unit's sensor timestamp, else by arrival as on USB. Not tested with a W5500 yet. Not done: the Pico → unit settings messages, everything on the TM171 and the rangefinder.

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
| Rangefinder | ToF lidar of the TF-Luna / TFmini-S class at 100 Hz, next to the lens, aimed at the patch the camera sees |
| Night light | 850 nm IR LEDs pulsed by the camera's strobe output, wired at the unit (the OV9281 is mono with no IR filter) |
| Power and data | One PoE cable from the cab: a PoE splitter or HAT at the unit, a 12 V PoE injector or switch in the cab. Sealed M12 D-coded connectors |
| Network | A small switch in the cab joins the Pico's W5500, the front unit and the AgOpenWeb tablet on 192.168.5.x |

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
- **Pico → unit:** the settings the unit needs (camera height, image direction, TM171 mounting) and a heartbeat. The time requests are the first Pico → unit message (2026-11-20); the settings and the heartbeat are still to add.
- **Messages to add to `flow_link.h`** (with the Python mirror and the three tests, in one commit):
  - time request / response;
  - camera velocity (already de-rotated, below), with the rangefinder height;
  - TM171 sample batch (8–10 samples per datagram).
- **Bandwidth:** flow 3.5 kB/s at 100 Hz. TM171 samples at 800 Hz about 30 kB/s (estimate, about 36 bytes a sample). Far below what the W5500 and 100 Mbit/s carry.

## The TM171's three jobs
### 1. De-rotating the flow at the camera
The cab sits on its suspension and the camera on the chassis, so they are not one rigid body. The cab rocks against the chassis at about 1–3 Hz (estimate), slower than the 0.1 s speed averaging can remove. De-rotating the chassis camera's flow with the roof gyro is wrong by height × (ω_cab − ω_chassis): for example 1 m × 0.1 rad/s = 0.1 m/s (estimate).

- **On the unit:** it integrates the TM171 gyro over each frame interval, in its own clock with no sync error, and removes the rotation. It sends the camera's velocity in body axes with the rangefinder height.
- **On the Pico:** `flow_nav.c` gets a path for these messages. It skips the de-rotation, keeps the lever arm (yaw is shared by cab and chassis, so the roof gyro serves for `ω × r`), the tilt gate and the averaging.
- The de-rotation formulas stay as in `flow_nav.h`, so the unit's C++ can share them. They move into a header both sides include, or are mirrored and tested on the same cases as `firmware/tools/test_flow_nav.c`.

### 2. Backup IMU for the roof TM210
- **Path:** the unit streams TM171 samples in batches. The Pico puts them in a time base through the clock sync and into the same core 0 → core 1 IMU queue as the TM210 (`firmware/app/nav_core1.c`).
- **Switching:** core 1 already stops INSLIB 50 ms after the last IMU sample (mode NONE). When the TM210 stays lost for longer, a time to choose, core 0 switches the source. Core 1 then re-initialises with the TM171's mounting rotation and lever arms and re-aligns. Its lever arm to the antenna is about 2.5 m forward and 2 m up.
- **Limits:**
  - The antenna moves against the TM171 with the cab sway, by about 3 m × 1–2° = 5–10 cm (estimate).
  - The Ethernet path adds delay and may lose a datagram. A lost batch must show as an IMU gap, never be bridged.
  - So it is a degraded backup that keeps steering alive, not an equal of the roof IMU.
- **Not planned:** both IMUs in INSLIB at once. `nav_suite` takes one IMU.

### 3. Cab-suspension roll for the ground point
Cab roll = roof roll (INS, TM210) − chassis roll (TM171).

The rear axle and the hitch, which AgOpenGPS steers by, are on the chassis; the antenna is on the cab. The ground point below the antenna is therefore:
- the antenna moved down by the cab's roll over the height above the cab's roll centre,
- then by the chassis roll over the roll centre's height above the ground.

That replaces one roll over the whole antenna height. At 3 m, 1° of cab sway taken as chassis roll puts the ground point 5 cm sideways (estimate). Pitch works the same way, fore and aft.

- **Where:** `firmware/app/ground_point.c` (today one attitude and one height, `ant_height_m`), with a new setting for the roll-centre height (to find on the bench).
- **The chassis attitude:** from the TM171's own fused roll and pitch (it outputs roll, pitch and yaw), or from its gyro and accelerometer through a small AHRS on the Pico.
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

## Front mounting and the flow
- **Lever arm:** `flow_nav.c` moves the camera velocity to the IMU point. At 2.5 m ahead, a headland turn at 0.3–0.5 rad/s puts 0.75–1.25 m/s of sideways speed at the camera, and a 10 cm arm error gives 0.03–0.05 m/s (estimate). Measure the arm to about 5 cm, or learn it under RTK later.
- **Height:** at the front it changes with the chassis pitch: Δh = 2.5 m × 1° = 4.4 cm, 4–5 % of the scale at 1 m height (estimate). Lifting a heavy rear implement takes weight off the front axle and lifts the nose; standing crop changes it too.
  - The rangefinder measures the height the flow sees, frame by frame.
  - A slow scale against the RTK speed, learned like the wheel pulses per metre (`firmware/app/speed_ppm.c`), takes out what is left.
- **View:** ground ahead of the front wheels, uncrushed stubble or crop. Over standing crop the flow follows the canopy, which the rangefinder also follows.
- **Gain:** 2.5 m ahead of the pivot, the camera's sideways speed is ω × 2.5 m. That is a direct measurement of yaw rate and slip, which suits INSLIB's later body-velocity input ([optical-flow-aiding.md](optical-flow-aiding.md), "INSLIB integration").

## Steps
1. TM171 on the SBC: read it at 800 Hz on UART or USB, check its raw output, its delay and its Allan variance (`firmware/tools/tm210_allan_log.py`; `TODO.md`).
2. De-rotation on the unit with the TM171 gyro, the camera-velocity message, the `flow_nav.c` path for it. Bench test over USB first (`flow --usb`, as today).
3. W5500 on the Pico ([ethernet-w5500-agopenweb.md](ethernet-w5500-agopenweb.md)), socket 2, the clock sync, flow over UDP. Code for socket 2, flow over UDP and the clock sync written 2026-11-20; not yet run against a W5500.
4. Rangefinder on the unit, height in the message; the scale learned under RTK.
5. Front mount on the tractor, PoE, the lever arm measured; drive against RTK.
6. TM171 batches to the Pico, the backup switching, a test with the TM210 unplugged.
7. Chassis roll and the cab-roll ground point; bench check of the roll-centre height and of AgOpenWeb with an antenna height of 0.

## Verification
- **De-rotation:** standing still with the engine running and the cab rocking, the camera velocity stays near zero. Compare the roof gyro against the TM171 gyro to see the cab sway.
- **Clock sync:** the offset's spread over minutes, and the residual of the flow speed against RTK as a function of an added delay, whose minimum should be at zero.
- **Backup:** unplug the TM210 while driving under RTK; the position error after the switch, and how long the re-alignment takes.
- **Ground point:** roll the cab by hand on its suspension with the chassis still. The ground point must not move with the cab-roll correction, and moves by the roll-centre geometry without it.
