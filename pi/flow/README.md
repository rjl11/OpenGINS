# Optical flow on the Raspberry Pi

Runs on a Raspberry Pi with the INNO-MAKER CAM-MIPI9281RAW-V2 (OV9281, mono, global shutter) looking down. It sends the image displacement between consecutive frames to the Pico over a serial link. The Pico removes rotation with its gyro, divides by its own strobe interval and scales by height above ground before INSLIB sees anything.

- Design: [plans/optical-flow-aiding.md](../../plans/optical-flow-aiding.md)
- Link protocol: [common/flow_link.h](../../common/flow_link.h), used directly by the C++ program and mirrored in `python/flow_link.py`
- Camera manual and trigger tools: `external/CAM-OV9281RAW-V2` (submodule; the Pi does not need it, copy `ov9281_libcamera/` over)

Status: the C++ program builds against Debian 13's libcamera 0.4.0 and OpenCV 4.10, and its tests pass on dietpi with synthetic frames. Nothing has run on a Pi or the camera yet.

## Layout

| Path | What |
|---|---|
| `src/main.cpp` | Capture → flow → link loop; `--dry-run` prints, `--frames DIR` reads PNGs instead of the camera; stats on stderr every 5 s |
| `src/camera.cpp` | libcamera capture: the 640×400 sensor mode (full view), scaled by the ISP to the processing size, fixed exposure and gain, sensor timestamps. Built only when libcamera is found |
| `src/flow_lk.cpp` | Lucas-Kanade flow that keeps its features between frames, outlier rejection against the median, undistortion, median displacement and its 1-sigma |
| `src/interval_watch.hpp` | Marks the frame after the Pico's left-out trigger (`FLOW_F_AFTER_GAP`) and frames lost on the Pi (`FLOW_F_DROPPED`) |
| `src/serial.cpp` | Raw 8N1 serial port |
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

Options: `--size WxH` (processing size, default 320×200), `--fps` (free-running rate, default 100), `--exposure-us`, `--gain`, `--camera N`, `--triggered`, `--calib FILE.json`, `--corners N` (default 60), `--baud`, `--threads N` (OpenCV threads, default 1: OpenCV's own default spread the flow over 2.6 cores and made it slower), `--loop` and `--realtime` (replay `--frames` endlessly from memory at `--fps`, for soak tests).

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
