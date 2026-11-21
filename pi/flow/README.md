# Optical flow on the Raspberry Pi

Runs on a Raspberry Pi with the INNO-MAKER CAM-MIPI9281RAW-V2 (OV9281, mono, global shutter) looking down. It sends the image displacement between consecutive frames to the Pico over a serial link. The Pico removes rotation with its gyro, divides by its own strobe interval and scales by height above ground before INSLIB sees anything.

- Design: [plans/optical-flow-aiding.md](../../plans/optical-flow-aiding.md)
- Link protocol: [common/flow_link.h](../../common/flow_link.h), used directly by the C++ program and mirrored in `python/flow_link.py`
- Camera manual and trigger tools: `external/CAM-OV9281RAW-V2` (submodule; the Pi does not need it, copy `ov9281_libcamera/` over)

Status: the C++ program builds against Debian 13's libcamera 0.4.0 and OpenCV 4.10, and its tests pass on dietpi with synthetic frames. On a ROCK 4SE (below) `flow --v4l2` runs on the live camera: 100 and 300 fps, 100 % valid with the camera standing still. Nothing has run on a Raspberry Pi with the camera yet, nor with the camera moving.

## Layout

| Path | What |
|---|---|
| `src/main.cpp` | Capture → flow → link loop; `--dry-run` prints, `--frames DIR` reads PNGs instead of the camera; stats on stderr every 5 s |
| `src/frame_source.hpp` | The frame, the camera options and the interface both captures implement |
| `src/v4l2_camera.cpp` | V4L2 capture (`--v4l2`) for boards libcamera can't drive (ROCK 4SE): 8-bit raw from the capture node scaled to the processing size, exposure, gain and frame rate through the sensor's controls, the line time corrected once from the measured frame period, drops counted from the buffer sequence. Always built |
| `src/camera.cpp` | libcamera capture: the 640×400 sensor mode (full view), scaled by the ISP to the processing size, fixed exposure and gain, sensor timestamps. Built only when libcamera is found |
| `src/flow_lk.cpp` | Lucas-Kanade flow that keeps its features between frames, outlier rejection against the median, undistortion, median displacement and its 1-sigma; phase correlation on the image centre when too few features are left (below) |
| `src/preview_http.cpp` | The camera preview web page on port 8080, for aiming and focusing the camera (below, "Camera preview page") |
| `src/preview_metrics.cpp` | The page's focus figure (variance of the Laplacian) and exposure figures |
| `src/interval_watch.hpp` | Marks the frame after the Pico's left-out trigger (`FLOW_F_AFTER_GAP`) and frames lost on the Pi (`FLOW_F_DROPPED`) |
| `src/serial.cpp` | Raw 8N1 serial port |
| `src/tfluna.cpp` | TF-Luna lidar protocol: data frames and command replies, the commands, the validity rules, and the buffer that picks the range at each frame. No OS calls (below, "TF-Luna rangefinder") |
| `src/tfluna_reader.cpp` | The TF-Luna on a serial port: set up at start, read in its own thread, a CSV log; used by `flow --lidar` |
| `src/tfluna_main.cpp` | `tfluna`, the TF-Luna alone for bench checks: every sample as CSV, a summary each second |
| `src/tm171.cpp` | TM171 IMU: the EasyProfile codec, its Setting target and layout guard, the clock map onto `CLOCK_MONOTONIC`, the gyro history and the de-rotation (the model of the Pico's `flow_nav.c`). No OS calls (below, "TM171 IMU") |
| `src/tm171_reader.cpp` | The TM171 on its USB port: settings checked (and fixed) at start, read in its own thread; used by `flow --tm171` |
| `src/tm171_main.cpp` | `tm171`, the TM171 alone: the setup (`--save` to flash), then one line a second |
| `rock4se/` | Camera setup on a Radxa ROCK 4SE: device-tree overlay and `setup.sh` (ISP module build, pipeline, status) |
| `tests/test_flow.cpp` | Synthetic shifts, a moving sequence, the link example frame, the interval rules, and a speed run |
| `tests/test_tfluna.cpp` | TF-Luna commands against the manual's examples, frame parsing through junk, split reads and bad sums, replies, sample times, the range buffer. Needs no OpenCV |
| `tests/test_tm171.cpp` | TM171 packets against the Python bench code and our unit's own bytes, the Setting target, the clock map, the gyro integral, the de-rotation of a pure rotation and a translation. Needs no OpenCV |
| `python/` | The first version in Python (Picamera2): same algorithm and flags, kept as the reference and for quick experiments on recorded frames. Its tests: `python3 -m unittest -v test_flow_link test_flow_lk` from `python/` |

## Setup on the Pi (Raspberry Pi OS Bookworm or later)

```bash
sudo apt install -y cmake ninja-build g++ pkg-config libopencv-dev libcamera-dev
```

1. `/boot/firmware/config.txt`: `dtoverlay=ov9281,cam0` (or `cam1`, by the CSI port used), reboot. `rpicam-hello --list-cameras` should list the 640×400 mode at 309.79 fps (RAW8).
2. Serial to the Pico: enable the UART (`raspi-config` → Interface → Serial: no login shell, hardware on). `/dev/serial0` TX is header pin 8 (GPIO14) → Pico GP8. Common ground.
3. Trigger mode (later, once the Pico triggers): copy the vendor's `ov9281_libcamera/` folder over, `source ./libcamera-timeout.sh` in the shell that runs `flow` (libcamera otherwise times out waiting for triggered frames), then `./ov9281_trigger <i2c bus> 1` (bus from `dmesg | grep ov9281`), and run with `--triggered`.

Clone this repo on the Pi without `--recursive`; it needs no submodule.

## Setup on a Radxa ROCK 4SE (RK3399, DietPi / Armbian)

The camera also runs on a ROCK 4SE's CSI connector (15-pin, same cable as a Pi), but with three differences from a Pi, all handled by `rock4se/setup.sh`:

- **No ISP driver in the kernel.** Armbian's `rockchip64` kernels (6.18 `current` and 7.3 `edge`, checked 2026-11-19) are built without `CONFIG_VIDEO_ROCKCHIP_ISP1`. `setup.sh install` builds `rkisp1` out of tree from kernel.org's source for the running kernel version, installs it under `/lib/modules/<ver>/updates/`, and must be run again after a kernel upgrade.
- **No mono input on the ISP.** `rkisp1` takes only Bayer or YUV in, so the ov9282 driver is rebuilt with its Y8/Y10 bus codes reported as RGGB. The MIPI data type (RAW8/RAW10) and the pixels are the same. With the ISP in bypass, the mainpath delivers the sensor's 8-bit pixels unchanged (`RGGB` fourcc, but grey data).
- **No libcamera.** libcamera 0.4's rkisp1 pipeline has no sensor helper for the ov9281 (`Failed to create camera sensor helper`), so `cam` and `flow`'s libcamera capture don't start. Run `flow --v4l2` instead, after `setup.sh pipeline` has routed sensor → CSI → ISP → mainpath at 640×400 raw. It finds the capture node (`rkisp1_mainpath`) and the sensor (`ov9281 …`) by name; `--video` / `--subdev` override that.

The overlay `rock4se/rk3399-rock-4se-ov9281.dts` puts the sensor on I2C4 at 0x60 (chip ID `0x9281` at registers 0x300A/B) with a 24 MHz fixed clock (the module has its own oscillator), 2 lanes at 400 MHz, into ISP0 through MIPI D-PHY RX0. `setup.sh install` adds it to `user_overlays` in `/boot/dietpiEnv.txt` (or `armbianEnv.txt`).

```bash
sh pi/flow/rock4se/setup.sh install
```

Reboot, then after every boot:

```bash
sh pi/flow/rock4se/setup.sh pipeline
```

```bash
sh pi/flow/rock4se/setup.sh status
```

Exposure, gain and frame rate are V4L2 controls on the sensor subdevice (`/dev/v4l-subdev4` here; `status` prints them): `exposure` in lines (1–1717 at the default blanking), `analogue_gain` 16–255 (16 = 1×), `vertical_blanking`. The line time at 640×400 is 7.59 µs, so frame rate = 1 / (7.59 µs × (400 + 22 + `vertical_blanking`)). Measured 2026-11-19 with `v4l2-ctl --stream-mmap`: 92.67 fps at the default blanking (1022), 100.05 fps at 917, 311.78 fps at the minimum (22), no ISP errors. Exposure must fit in the frame, so lower the exposure before you shorten the frame. `flow --v4l2` sets all three from `--fps`, `--exposure-us` and `--gain`. The driver's own line time ((640 + 816) / 200 MHz = 7.28 µs) is 4 % short in this binned mode, so `flow` measures the frame period over the first 100 frames and sets the blanking again (it prints `line time now 7.592 us`). The frame time is the ISP's frame end (`CLOCK_BOOTTIME`), not the exposure start as with libcamera.

```bash
pi/flow/build/flow --dry-run --v4l2 --fps 100 --exposure-us 2000 --gain 4
```

Measured 2026-11-19 on dietpi, camera standing still in a dark room (gain 15.9), dry run, 320×200 processing, 52 features tracked: at 100 fps (exposure 9 ms) 100.0 msg/s, 100 % valid, no frames dropped, dt 9.99–10.01 ms, flow 5.6 ms per frame; at 300 fps (exposure 2.5 ms, pinned to the A72 cores with `taskset -c 4,5`) 299.6 msg/s, 100 % valid, 4 frames dropped at startup, flow 1.75 ms per frame. The 5.6 ms at 100 fps is the CPU clock governor running the cores slowly under the light load (it was the same pinned to the A72s), not a limit.

## Build and test

From the repo root:

```bash
cmake -S pi/flow -B pi/flow/build -G Ninja
```

```bash
ninja -C pi/flow/build
```

```bash
pi/flow/build/test_flow
```

```bash
pi/flow/build/test_tfluna
```

```bash
pi/flow/build/test_tm171
```

The link's C test (the Pico side's decoder and the shared encoder):

```bash
gcc -O2 -Wall -Wextra -I common firmware/tools/test_flow_link.c -o /tmp/t -lm && /tmp/t
```

## Run

```bash
pi/flow/build/flow --dry-run
```

```bash
pi/flow/build/flow --port /dev/serial0
```

```bash
pi/flow/build/flow --dry-run --frames DIR
```

Bench test with the Pico on the Pi's USB instead of the link UART (no trigger or strobe wired; the Pico times the frames by the Pi's clock, `plans/optical-flow-aiding.md`, "Timing"). The Pico's status text comes out on stdout, the flow program's own figures on stderr:

```bash
pi/flow/build/flow --usb --port /dev/ttyACM0 --v4l2 --fps 100 --exposure-us 2000 > pico.txt
```

To the Pico's Ethernet (W5500, the front camera unit, `plans/front-camera-unit.md`): each frame is one UDP datagram to port 5800. The program answers the Pico's clock sync requests on the same socket, and once the sync is valid the Pico times the frames by their sensor timestamps instead of their arrival (not with `--frames`, whose timestamps are made up). "Ethernet fitted" must be on in the Pico's web config. `tools/clock_sync_loopback.py` plays the Pico on one host to check the responder:

```bash
pi/flow/build/flow --udp 192.168.5.124 --v4l2 --fps 100 --exposure-us 2000
```

Options: `--size WxH` (processing size, default 320×200), `--fps` (free-running rate, default 100), `--exposure-us`, `--gain`, `--camera N`, `--triggered`, `--v4l2` (with `--video DEV`, `--subdev DEV`: capture through V4L2, see the ROCK 4SE section), `--calib FILE.json`, `--corners N` (default 60), `--baud`, `--usb` and `--udp HOST[:PORT]` (above), `--threads N` (OpenCV threads, default 1: OpenCV's own default spread the flow over 2.6 cores and made it slower), `--loop` and `--realtime` (replay `--frames` endlessly from memory at `--fps`, for soak tests), `--preview` / `--no-preview` / `--preview-port N` (below), `--lidar DEV` with `--lidar-baud`, `--lidar-rate`, `--lidar-cm`, `--lidar-device-filter`, `--lidar-log FILE.csv` (below, "TF-Luna rangefinder").

## TF-Luna rangefinder

The front unit's height sensor (`plans/front-camera-unit.md`, "Rangefinder: TF-Luna"; Benewake's manual in `docs/datasheets/SJ-PM-TF-Luna_A05_Product_Manual.pdf`). Status (2026-11-20): builds on dietpi without warnings and `test_tfluna` passes. Run against a fake sensor on a pseudo-terminal (a Python script answering the setup commands and sending 100 Hz frames, every 10th too weak): setup took, 100 Hz, 90 % ok, no bad sums, the median sample spacing 9.9 ms; `flow --dry-run --frames … --lidar` gave every frame a range at 100 frames/s and the preview's `/stats.json` carried it. Not yet run against the real sensor.

**Wiring:** UART mode, so pin 5 open (or at 3.3 V) before power-on. Pin 1 to 5 V, pin 4 to ground, pin 3 (its TX) to the SBC's RX, pin 2 (its RX) to the SBC's TX, 3.3 V logic. It has no reverse-polarity protection: check pin 1 before power-on. A USB-serial adapter works the same and moves to any SBC; on an FTDI adapter set `/sys/bus/usb-serial/devices/ttyUSB0/latency_timer` to 1, or its 16 ms default delays every sample.

**Setup at every start, never saved:** the program turns the output off, asks the version, sets the rate (`--rate` / `--lidar-rate`, default 100 Hz; the sensor runs 500/n Hz), the 9-byte mm format (`--cm` for cm), its own Kalman and median filters off (`--device-filter` keeps them), and turns the output on again. Nothing is saved in the sensor, so a power cycle brings back its defaults; only the baud rate has to match (115200 unless saved otherwise, `--baud`). No answer to the version request stops it with the likely causes (pin 5, TX/RX crossed, 5 V).

**Samples:** each one is classified `ok`, `low_amp` (Amp < 100; the distance is then the sensor's dummy 0), `overload` (Amp > 32768, sunlight), `too_close` (< 0.2 m) or `too_far` (> 8 m). Its time on `CLOCK_MONOTONIC` (the frames' clock) is the first byte's arrival, worked back from the read time and the bytes after it at the line rate, less half an output interval: each output is the mean of the sensor's 500 Hz ranges over that interval. The read time is late by the thread's wake-up, and over USB by the adapter; both are estimates to check with a step in height.

**In `flow`:** `--lidar DEV` gives each frame the median of the valid samples within 2.5 lidar intervals of the frame time (25 ms at 100 Hz: later samples have mostly not arrived when the frame is processed, and each is stamped half an interval back), else the last valid one up to 0.5 s old (`held`), else none. Recorded `--frames` take the newest samples, since their times are made up. The range goes into the `--dry-run` line (`range 1.234 m n 3 amp 1500 age 2.1 ms`), the 5 s stats line (rate, % ok, mean and sd, Amp, temperature) and the preview page; `--lidar-log FILE.csv` writes every sample. With `--tm171` it is the distance the camera velocity is scaled by, and its tilt-corrected height goes to the Pico in `FLOW_MSG_CAM_VEL` (below, "TM171 IMU"); without the TM171 it is not sent.

**Alone, for bench checks** (`plans/front-camera-unit.md`, "Verification"): `tfluna` prints every sample as CSV on stdout and a summary each second on stderr (rate, % ok, mean range and its sd, mean Amp, temperature, the count of each reject).

```bash
pi/flow/build/tfluna --port /dev/ttyUSB0 --seconds 60 > board_1m.csv
```

## TM171 IMU

The front unit's IMU (`plans/front-camera-unit.md`, "The TM171's three jobs"; protocol facts in `plans/imu-tm210-config.md`). Here it does job 1: de-rotating the flow at the camera, in this clock, so the Pico gets the camera's velocity (`FLOW_MSG_CAM_VEL`) and the cab's sway against the chassis doesn't enter it.

**Setup:** at start the Setting object is read back (node ID from the reply). The target: raw (CMD 41), Status (CMD 22) and RPY (CMD 35) output at 800 Hz on USB, no other fused output, temperature calibration off, Auto boot, silent time 0, baud word 9216 (921600, for a UART later; no effect on USB). If the read-back differs, the target goes to RAM; `--tm171-save` (or `tm171 --save`) writes it to flash instead. A read-back that matches is never written: it can't tell RAM from flash (bits 17 and 18 read 0 after a power cycle). A Setting whose layout doesn't look like the known one (bit 13 = 1, bits 14–16 = 0b010) is not written to. Calibration commands are never sent.

**Times:** the TM171's own microsecond timestamps, mapped onto `CLOCK_MONOTONIC` by the smallest arrival-minus-timestamp over the last one to two seconds (the packet that waited least in USB).

**Settings from the Pico:** with `--udp` or `--usb` the Pico sends the unit's settings once a second (`FLOW_MSG_UNIT_CFG`, also its heartbeat: camera height, image direction, TM171 mounting, flow on or off; on USB as `flowcfg <hex>` lines, which are not copied to stdout). From the first one on they replace `--height` and `--cam-yaw`, and the TM171 mounting turns its gyro into body axes. A change is printed once; the 5 s line shows the last one's number and age, or "no settings from the Pico yet". Run on the bench over USB (2026-11-21): one a second, each 0.9 s old when the 5 s line was printed.

**IMU batches:** with `--tm171` and a link (`--udp` or `--usb`), every 8 raw TM171 samples go to the Pico as a `FLOW_MSG_IMU_BATCH` (100 a second, fixed point, the TM171's own axes as FRD), from the TM171's thread under the same send lock as the flow: the Pico's backup IMU and the chassis tilt for the cab-roll ground point (`firmware/app/front_imu.h`). The 5 s line shows the batches per second.

**In `flow`:** `--tm171 DEV` integrates the gyro (FRD) over each frame pair's exposure middles and removes that rotation from the displacement, with the camera axes from `--cam-yaw` (image top from the TM171's x axis, clockwise from above; the Pico's convention). The distance along the optical axis is the TF-Luna range when `--lidar` has one (the beam is parallel to the optical axis), else `--height` over cos(roll)·cos(pitch) from the TM171's RPY. The message carries the vertical height (`FLOW_F_RANGE` when it came from the range, its 1-sigma 3 cm from the manual's ±6 cm, an estimate). A frame pair the gyro doesn't cover goes as the raw displacement, which the Pico de-rotates with the roof gyro. The 5 s line counts camera velocities, ranged ones, and frames without gyro or tilt; `--dry-run` prints each velocity.

Status (2026-11-21, dietpi with the camera and our TM171 on USB): set up and **saved to flash** (switches `0x001ea87f`, 800 Hz), then raw, RPY and Status each at 802–804/s with no CRC errors; gyro noise 0.0035–0.0044 rad/s per axis at rest, roll −0.1°, pitch +0.9° on the desk. In `flow --dry-run --v4l2 --tm171 /dev/ttyACM2`, 100 frames/s, every frame pair got a camera velocity (none without gyro or tilt), at rest within ±0.003 m/s at 1 m. After a power cycle (replugged the same day) it came up from flash with the target and streamed at once: raw, RPY and Status at 800 Hz, timestamps 1250 µs apart, no CRC errors. **Into the Pico** (2026-11-21, dietpi's Pico flashed with `main` at `9bf6786`, flow switched on in its settings): `flow --usb --port /dev/ttyACM1 --v4l2 --tm171 /dev/ttyACM2` for 20 s, camera standing still over texture (100 % valid, 49 features): the Pico took all 2054 camera velocities (`used 2054/2054`, 0 CRC errors, timed by the Pi's clock) and gave INSLIB 10 Hz samples at 0.00 m/s. INSLIB used none: with no GNSS receiver on that Pico the INS never initialised (ATTITUDE_ONLY). Not yet checked: the signs under real motion (a rotation in place over texture must read zero), and the fusion with the INS running.

```bash
pi/flow/build/tm171 /dev/ttyACM2 --seconds 10
```

## Camera preview page

For mounting the front camera unit (`plans/front-camera-unit.md`): `flow` serves a read-only web page with the live image, to aim the camera and set the lens focus. It runs alongside the flow in any mode (`--dry-run`, `--usb`, `--udp`, `--frames`) and prints its address at startup (`preview on http://192.168.5.30:8080/`). During development it is on by default; `--no-preview` turns it off, `--preview-port N` moves it from 8080. A production build turns the default off with `cmake -S pi/flow -B pi/flow/build -G Ninja -DFLOW_PREVIEW_DEFAULT=OFF`, and then the page needs `--preview`.

- `/`: the image about 4 times a second, with the tracked features (green while the flow is valid, orange when not; the feature count reads "phase" when the frame's flow came from the phase-correlation fallback), a centre cross and the box the focus figure is taken over. "zoom on the focus box" shows that box at full resolution.
- `/frame.jpg`: one frame as JPEG, at the sensor's 640×400 with `--v4l2`, else at the processing size.
- `/stats.json`: the figures for the last frame `/frame.jpg` served: focus, mean grey, % black / white, exposure, gain, features, the 5 s msg/s, % valid and flow time, frames dropped, and the feature points.

Installing: aim the camera until the cross is on the patch of ground it should see and the box is clear of the bumper, wheels and weights. Then turn the lens through focus and stop where the focus figure is at its peak (the bar full; "reset peak" after moving the camera or when the light changes). The focus figure is relative, since it also grows with the ground's texture: compare it only on one scene. Check that black / white stay near 0 % (not red) and that there are features over the whole image.

The page is served by its own thread at a lower priority. The flow loop only copies a frame when the page asks for one (about 4 times a second), and does the JPEG and the figures in the page's thread. Measured 2026-11-20 on dietpi with 300 synthetic frames at 100 fps: 100 msg/s and a mean flow time of 5.4–5.9 ms with the page fetched continuously, against 5.8–6.0 ms with `--no-preview`. The Pico's status page links to it (a "Front camera" chip) once frames arrive over its Ethernet; the link works only from the 192.168.5.x side (the AgOpenWeb tablet), not over the Pico's Wi-Fi. Only in the C++ program; `python/` has no preview.

## Soak test without a camera

`python/make_test_frames.py OUT` writes 300 frames on a closed circular path (3 px per frame), so `--loop` has no jump. `tools/soak.sh` runs the flow for `DUR` seconds and logs CPU, RAM, temperature and throttling every `INT` seconds to `log.csv`, with the flow's own 5 s stats in `flow.log`:

```bash
DUR=3600 INT=60 OUT=~/soak nohup setsid sh pi/flow/tools/soak.sh pi/flow/build/flow --dry-run --frames FRAMES --loop --realtime --fps 100 > /dev/null 2>&1 < /dev/null &
```

## Phase-correlation fallback

When Lucas-Kanade keeps fewer than 15 features (smooth concrete, fine soil, dusk), `flow_lk.cpp` runs `cv::phaseCorrelate` on a square in the image centre instead. It needs texture, not corners. The idea is from Clover's `optical_flow.cpp` (CopterExpress/clover, MIT); here the square is set by angle (`phase_roi_rad`, 0.5 rad wide, 116 px at 320×200), the mean is removed before the Hanning window (otherwise the window's own shape pulls weak texture towards zero shift), and the shift is undistorted around the calibrated principal point, not the image centre. The message then carries `FLOW_F_PHASE` and `n_tracked` holds the correlation response × 1000; the Pico skips its feature-count check for these frames and counts them (`phase corr N` on the `FLOW` status line, `phase` in `/status.json`).

It is rejected when the square's contrast is below 2 grey levels (1-sigma), the response is below 0.2 (`--phase-min`), or the shift is more than a quarter of the square. Its 1-sigma is 0.1 px / response. These three numbers are estimates, not yet tuned on real ground. It sees one translation for the whole square, so yaw within a frame and repeating patterns (crop rows) can fool it, and the sensor's fixed-pattern noise may pull a weak image towards zero shift: check both on the camera before trusting it. `--no-phase` turns it off. The `flow` stats show the share of frames that used it (`N % phase corr`). C++ only; the Python reference has only the flag.

Speed on dietpi (`test_flow`, A72 core, 320×200, synthetic frames, measured 2026-11-20): 1.51 ms per phase correlation, on top of the Lucas-Kanade pass that failed first.

## Calibration

Without `--calib` the lens is taken as a 2.8 mm pinhole with no distortion, which is only right near the centre of the 148° lens. Calibrate once at the processing size (OpenCV's fisheye model with a chessboard) and write a JSON (both the C++ and the Python version read it):

```json
{"fx": 233.0, "fy": 233.0, "cx": 159.5, "cy": 99.5, "dist": [0.0, 0.0, 0.0, 0.0], "fisheye": true}
```

## Speed

Measured 2026-11-15 on dietpi (Rockchip RK3399, one thread, OpenCV 4.10, synthetic frames at 320×200), per frame:

| Version | Time |
|---|---|
| Python, first version: new features every frame, LK forward and backward, 3 levels, 21 px window | 16.6 ms |
| Python, optimized: features kept between frames, one forward pass, 2 levels, 15 px window, median outlier test | 3.2 ms |
| C++, same algorithm | 1.9 ms |

Corner detection ran 4 times in 299 frames. Of the first version's 16.6 ms, only about 1 ms was Python itself; the gain came from the algorithm, and C++ adds the rest. The first frame of a run takes about 15 ms (detection and allocation).

The C++ version on three hosts (`test_flow`'s speed run, 320×200, one thread, 3 runs each agreeing within 0.01 ms; measured 2026-11-15):

| Host | CPU | OpenCV | Per frame | Max rate |
|---|---|---|---|---|
| rtkbase (Raspberry Pi 5) | Cortex-A76, 2.4 GHz | 4.6.0 | 0.75 ms | 1330 Hz |
| dietpi (ROCK 4SE, RK3399), big core | Cortex-A72, 1.5 GHz | 4.10.0 | 1.89 ms | 530 Hz |
| dietpi, little core (pinned with `taskset`) | Cortex-A53, 1.0 GHz | 4.10.0 | 5.65 ms | 177 Hz |
| miniPi (Raspberry Pi Zero 2 W) | Cortex-A53, 1.0 GHz | 4.10.0 | 5.57 ms | 180 Hz |

**One-hour soak on miniPi (Zero 2 W), 2026-11-15 20:25–21:25,** with `tools/soak.sh`: 300 synthetic frames on a circular path replayed at 100 fps in real time (`--loop --realtime --threads 1`, dry run), no camera. 100.0 msg/s throughout (lowest 5 s window 99.7), 100 % valid; flow 6.0 ms per frame (worst 5 s mean 6.3 ms; slowest single frame 32.6 ms after the first 5 s, 157 ms at startup); the flow process 60.5 % of one core, the whole machine 16.3 %; RSS 81 MB steady, 234–240 MB RAM available, swap unchanged at 44 MB; temperature 54.8 → 59.6 °C in miniPi's closed case (up for over two days before the run, so warm to begin with), no throttling (`0x0`) at any time. New features were detected about 3 times per 100 frames on this path (10 789 in the hour), which is what lifts the mean above the 5.6 ms of the benchmark. Capture (libcamera, the ISP) was not part of it.

Max rate is 1 / time per frame for the flow alone, with nothing else on that core; capture, the copy out of the camera buffer and the serial write come on top, and the camera itself tops out at 309 fps. On a Zero 2 W the default 100 fps leaves about 45 % of one core free; a Pi 4/5 runs the sensor's full rate easily. No throttling was reported on either Raspberry Pi (`vcgencmd get_throttled` 0x0; rtkbase also runs dump1090 at about a third of one core).
