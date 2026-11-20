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
| `src/flow_lk.cpp` | Lucas-Kanade flow that keeps its features between frames, outlier rejection against the median, undistortion, median displacement and its 1-sigma |
| `src/interval_watch.hpp` | Marks the frame after the Pico's left-out trigger (`FLOW_F_AFTER_GAP`) and frames lost on the Pi (`FLOW_F_DROPPED`) |
| `src/serial.cpp` | Raw 8N1 serial port |
| `rock4se/` | Camera setup on a Radxa ROCK 4SE: device-tree overlay and `setup.sh` (ISP module build, pipeline, status) |
| `tests/test_flow.cpp` | Synthetic shifts, a moving sequence, the link example frame, the interval rules, and a speed run |
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

Options: `--size WxH` (processing size, default 320×200), `--fps` (free-running rate, default 100), `--exposure-us`, `--gain`, `--camera N`, `--triggered`, `--v4l2` (with `--video DEV`, `--subdev DEV`: capture through V4L2, see the ROCK 4SE section), `--calib FILE.json`, `--corners N` (default 60), `--baud`, `--usb` and `--udp HOST[:PORT]` (above), `--threads N` (OpenCV threads, default 1: OpenCV's own default spread the flow over 2.6 cores and made it slower), `--loop` and `--realtime` (replay `--frames` endlessly from memory at `--fps`, for soak tests).

## Soak test without a camera

`python/make_test_frames.py OUT` writes 300 frames on a closed circular path (3 px per frame), so `--loop` has no jump. `tools/soak.sh` runs the flow for `DUR` seconds and logs CPU, RAM, temperature and throttling every `INT` seconds to `log.csv`, with the flow's own 5 s stats in `flow.log`:

```bash
DUR=3600 INT=60 OUT=~/soak nohup setsid sh pi/flow/tools/soak.sh pi/flow/build/flow --dry-run --frames FRAMES --loop --realtime --fps 100 > /dev/null 2>&1 < /dev/null &
```

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
