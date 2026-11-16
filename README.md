# OpenGINS

**Open GNSS and Inertial Navigation System.**

OpenGINS is GNSS and IMU fusion firmware for a navigation unit meant to be universal: an agricultural unit for a tractor (AgOpenGPS / AgOpenWeb), a GNSS survey pole, and a UAV. The design is one board and one filter for all three; the platform setting picks how the sensors are used.

The unit combines a Unicore UM980 / UM982 GNSS receiver, a TM210 IMU, a BMP581 barometer and optional wheel or optical-flow speed in the [INSLIB](https://github.com/jnz/INSLIB) `nav_suite` filter. It runs on a Raspberry Pi Pico 2 W (RP2350) in bare metal. The filter runs on core 1; core 0 handles the sensor UARTs, Bluetooth and the Wi-Fi configuration page.

The firmware also reports the height above sea level (RH 2000) next to the ellipsoid height, using Lantmäteriet's SWEN17_RH2000 geoid model cut to an area around the RTK base. See [Geoid heights](#geoid-heights).

## Status

Work in progress. The firmware builds, and its parts have been tested on the bench: sensor bring-up, the filter's output, the web page, and the wheel-speed pulse capture. The optical-flow path is written and tested with synthetic frames but has not yet run with a camera. The whole system has not yet been tested end to end in a vehicle. There are no automated tests; the host-side C tests in `firmware/tools/` cover the calibration, speed and geoid code. Open items are listed in [TODO.md](TODO.md).

## Hardware

| Part | Role | Interface |
|---|---|---|
| Raspberry Pi Pico 2 W | Main MCU, Wi-Fi, Bluetooth | — |
| Unicore UM980 or UM982 | GNSS receiver, 20 Hz logs, PPS | UART1 to COM1 (commands, logs and RTCM); USB-C is COM3 for a PC |
| TM210 | IMU, 800 Hz raw data | UART0 at 921600 |
| BMP581 | Barometer, 100 Hz | I2C1 |
| Raspberry Pi with OV9281 camera | Optical flow (optional) | Link to the Pico on a PIO UART |
| Wheel or radar pulse input | Ground speed (optional) | Two speed inputs |

The GPIO assignments are all in [`firmware/app/board_pins.h`](firmware/app/board_pins.h). No other file names a pin number.

## Repository layout

| Path | Contents |
|---|---|
| `firmware/` | Pico firmware: `app/` (main loop, filter on core 1, settings, calibration, web server), `drivers/` (UART DMA, TM210, UM980/UM982, BMP581, pulse input, flow I/O, INSLIB wrapper), `tools/` (host tests, logging and decoding scripts), `link/` (linker override that puts the filter in SRAM) |
| `pi/flow/` | Optical-flow program for the Raspberry Pi (C++17, OpenCV and libcamera from the OS). `python/` is the first version, kept as a reference |
| `geoid/` | Source geoid grid SWEN17_RH2000 (Lantmäteriet, CC0) and its README |
| `common/` | Headers shared by the Pico and the Pi, such as the flow link format `flow_link.h` |
| `plans/` | Design documents, one per topic. The index is [plans/README.md](plans/README.md) |
| `docs/` | Datasheets, receiver manuals and receiver firmware builds |
| `external/` | Git submodules: INSLIB (the filter), the Bosch BMP5 driver API, and reference copies of the Adafruit BMP5 library and the OV9281 camera repository |
| `BUILDING.md` | Building, flashing, testing, and the rules for the code |
| `TODO.md` | Open work items, each linked to its plan |

## Getting the code

```bash
git clone https://github.com/rjl11/OpenGINS.git
cd OpenGINS
git submodule update --init --recursive
```

The Raspberry Pi needs no `--recursive`: the flow program does not use a submodule.

## Build

You need the Pico SDK 2.3.1 (`PICO_SDK_PATH` set), Ninja, CMake and an Arm GCC toolchain.

```bash
cmake -S firmware -B firmware/build -G Ninja
ninja -C firmware/build
```

The output is `firmware/build/app.uf2`; copy it to the BOOTSEL drive. [BUILDING.md](BUILDING.md) has the build options (antenna mode, platform, Wi-Fi), the SRAM check to run after each build, flashing and debugging over SWD, the host tests, the Raspberry Pi optical-flow program, and the geoid table for another area.

## Use

- **Web page.** The Pico joins the network named by `WIFI_STA_SSID` and prints its IP address on USB. It is also reachable as `gnss-smart.local`. If it cannot join within 30 s, it opens the access point `gnss_smart` at 192.168.4.1.
- **Status.** `/status.json` gives the live state of the sensors, the filter and the receiver. The ellipsoid height comes with `h_geoid_m` (RH 2000 height) and `geoid_model` (which geoid was used).
- **Field logs.** Send `log on` on the USB port to record every sensor, the filter output and the flow link as binary records. Decode them with `firmware/tools/field_log.py`.

## Geoid heights

The filter's height is an ellipsoid height. To get a height above sea level, the firmware subtracts the geoid height N from it. The geoid model is Lantmäteriet's SWEN17_RH2000 (open data, CC0), which converts a SWEREF 99 ellipsoid height to an RH 2000 height. The whole grid is too large for flash, so `firmware/tools/geoid_cut.py` cuts a 100 km area around the RTK base (58N 12E) into `firmware/app/geoid_grid.c` (182 × 172 values, about 61 KB of int16 data), and `firmware/app/geoid.c` interpolates it bilinearly.

- The result is an RH 2000 height only when the position is in SWEREF 99, which is the case for RTK from a base given in SWEREF 99. A standalone fix (WGS84 / ITRF) is a few decimetres off.
- Outside the cut, the firmware uses the receiver's own undulation instead, and `geoid_model` says so.
- Lantmäteriet gives an uncertainty of 8–10 mm over most of Sweden and 2–10 cm in the mountains, near Vättern, in coastal waters and further out at sea. See [geoid/README.md](geoid/README.md) for the file format and how to make the table for another base or radius.

## Licence

The code in this repository is licensed under the **GNU General Public License, version 3 or later** ([LICENSE](LICENSE)), with an additional permission for linking with the Pico SDK's `cyw43-driver` and BTstack ([LICENSE-EXCEPTION.md](LICENSE-EXCEPTION.md)).

Parts with their own licences:

| Part | Licence |
|---|---|
| `external/INSLIB` (the navigation filter, compiled into the firmware) | GNU AGPL-3.0, by its author |
| `external/BMP5_SensorAPI` (Bosch, compiled in) | BSD-3-Clause |
| `external/Adafruit_BMP5xx`, `external/CAM-OV9281RAW-V2` (reference only, not compiled) | Their own, see each repository |
| `firmware/drivers/ap_servers/` (DHCP and DNS servers for the access point) | MIT and BSD-3-Clause, see the files |
| Pico SDK, lwIP | BSD-3-Clause |
| Pico SDK `cyw43-driver` and BTstack | Raspberry Pi's licences, for use with Raspberry Pi chips only |
| `geoid/SWEN17_RH2000.txt` and the table cut from it (Lantmäteriet) | CC0 1.0 |

Because INSLIB is AGPL-3.0, a firmware binary built from this repository is also covered by the AGPL: if you distribute it, or let people use a modified version over a network (such as its web page), offer them its complete source code.

## Documentation

- [plans/README.md](plans/README.md): design and hardware decisions, with an index.
- [TODO.md](TODO.md): open work and what has been tested so far.
- [BUILDING.md](BUILDING.md): building, flashing and debugging, host tests, and the rules for the code.
