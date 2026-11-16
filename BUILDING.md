# Building, flashing and testing

This covers the Pico firmware, the host-side tests, the Raspberry Pi optical-flow program, and regenerating the geoid table. For what the unit does, see [README.md](README.md). For the design, see [plans/README.md](plans/README.md).

## Requirements

- **Pico SDK 2.3.1**, with `PICO_SDK_PATH` pointing at it. Clone it with its submodules (`git submodule update --init` inside the SDK); the CYW43439 driver, BTstack and lwIP come from there.
- **Arm GCC** (`arm-none-eabi-gcc`), tested with 14.2.
- **CMake** 3.13 or newer and **Ninja**.
- **picotool** 2.3.1 (the SDK builds it itself if it can't find one, which is slow).
- **Python 3** for `check_ram_path.py` and the tools in `firmware/tools/`. `field_log.py` and the TM210 tools use `pyserial`.
- A host C compiler (`gcc`) for the host tests.

On Linux (Debian 13 and similar):

```bash
sudo apt install cmake ninja-build gcc-arm-none-eabi libnewlib-arm-none-eabi build-essential python3 python3-serial
```

On Windows, Raspberry Pi's Pico SDK installer or the VS Code Pico extension gives the toolchain, CMake, Ninja and picotool. If the build can't find them, open a new shell: an old one may still have the `Path` and `PICO_SDK_PATH` from before the install.

## Get the code

```bash
git clone https://github.com/rjl11/OpenGINS.git
cd OpenGINS
git submodule update --init --recursive
```

The submodules are INSLIB (the filter, compiled in), Bosch's BMP5 API (compiled in), and two reference repositories that are not compiled (Adafruit's BMP5xx library and INNO-MAKER's OV9281 camera repository).

## Build the Pico firmware

```bash
cmake -S firmware -B firmware/build -G Ninja
ninja -C firmware/build
```

The outputs are `firmware/build/app.uf2` (copy to the BOOTSEL drive), `app.elf` (for SWD) and `app.dis` (disassembly, for the check below).

### Build options

Pass them on the `cmake -S` line with `-D<option>=<value>`. Change one by running cmake again with the new value; the build directory keeps the last one.

| Option | Values (default first) | Meaning |
|---|---|---|
| `GNSS_ANTENNA_MODE` | `single`, `dual` | One antenna, or two for a UM982 with GNSS heading. The web page can override it (applied at reboot) |
| `PLATFORM` | `pole`, `tractor`, `multirotor`, `fixed_wing` | When the GNSS course counts as a heading for the magnetometer calibration. `pole` also sets an antenna height of 2 m. The web page can override it (applied at reboot) |
| `WEB_MODE` | `always`, `switch`, `off` | Wi-Fi and the web page always on; only when GPIO22 is pulled to GND at boot; or built without Wi-Fi |
| `WIFI_MODE` | `auto`, `sta`, `ap` | Join a network and fall back to the open access point `gnss_smart` (192.168.4.1) if it hasn't joined within 30 s; station only; or the access point from boot |
| `WIFI_STA_SSID` | see `firmware/CMakeLists.txt` | Network to join |
| `WIFI_STA_PASSWORD` | empty (open network) | Password for that network |

Example, a tractor build for a UM982 with two antennas, joining your own network:

```bash
cmake -S firmware -B firmware/build -G Ninja -DPLATFORM=tractor -DGNSS_ANTENNA_MODE=dual -DWIFI_STA_SSID=MyNetwork -DWIFI_STA_PASSWORD=secret
```

### Check that the filter runs from SRAM

The filter on core 1 and the timestamping interrupt handlers must not run from flash: flash access stalls while the Wi-Fi driver or a settings write uses it, and that would break the 800 Hz timing. The linker override in `firmware/link/` places them in SRAM. After every build, run from the repo root:

```bash
python firmware/tools/check_ram_path.py firmware/build/app.dis
```

It must report nothing in flash that is reachable from core 1 or the ISRs. If code on core 1 starts calling a new SDK or libc function, add the file that holds it to `firmware/link/default_text_excludes.incl`.

### Build hosts with little RAM

On a board with under 1 GB of RAM (a Raspberry Pi Zero 2 W, for example), build with one job and add a swap file of about 2 GB. The SDK builds `pioasm` and `picotool` as nested CMake projects that ignore Ninja's `-j1` but follow `CMAKE_BUILD_PARALLEL_LEVEL`:

```bash
export CMAKE_BUILD_PARALLEL_LEVEL=1
cmake -S firmware -B firmware/build -G Ninja
ninja -j1 -C firmware/build
```

## Flash

### UF2 over USB

Hold BOOTSEL while plugging in the Pico 2 W, then copy `firmware/build/app.uf2` to the drive that appears. Or, with picotool:

```bash
picotool load -x firmware/build/app.uf2
```

`picotool load -f -x` is meant to reboot a running app into BOOTSEL first. On our bench it sometimes rebooted straight back into the app and the load failed; then use BOOTSEL or SWD.

### SWD with a Debug Probe

With a Raspberry Pi Debug Probe (or another CMSIS-DAP probe) and Raspberry Pi's OpenOCD fork (the `sdk-2.0.0` branch; upstream OpenOCD 0.12.0 has no RP2350 target):

```bash
openocd -f interface/cmsis-dap.cfg -f target/rp2350.cfg -c "adapter speed 5000" -c init -c "reset halt" -c "mww 0x40022000 0x0C000004" -c "flash write_image erase firmware/build/app.elf" -c "verify_image firmware/build/app.elf" -c "reset run" -c exit
```

The `mww` line holds DMA, UART0 and UART1 in reset (bits 2, 26 and 27 of the RESETS set alias at `0x40022000`). Without it, the running app's UART DMA keeps writing into SRAM while OpenOCD uses it as a flash buffer, and whole 4 KB sectors come out wrong once the IMU is streaming. Speeds from 5 to 30 MHz verified on our wiring. The write takes about 9 s whatever the speed, because the flash chip's erase and program time sets the pace.

The Pico 2 W has no SWD connector. Solder a 3-pin header to its SWCLK, GND and SWDIO pads (labelled in [docs/pico2w-pinout.svg](docs/pico2w-pinout.svg)) and connect them to the probe's "D" port: SWCLK to SC (orange), GND to GND (black), SWDIO to SD (yellow).

### Debugging

```bash
openocd -f interface/cmsis-dap.cfg -f target/rp2350.cfg -c "adapter speed 5000" -c "bindto 0.0.0.0"
```

and in another shell (`gdb-multiarch` on Debian, `arm-none-eabi-gdb` elsewhere):

```bash
gdb-multiarch firmware/build/app.elf -ex "target extended-remote :3333"
```

## Run

The firmware prints a status report once a second on its USB serial port (USB CDC, so any terminal and any baud setting). UART0 is the IMU, so there is no stdio on a UART. With Wi-Fi on, the report includes the IP address; the web page is at that address or `gnss-smart.local`. Commands typed on the USB port include `log on` for the field-test log, recorded and decoded with:

```bash
python firmware/tools/field_log.py --help
```

## Host tests

The calibration, pole, speed, ground-point, geoid and flow-link code is plain C that also builds on a PC. Each test exits with 0 when all checks pass. From the repo root (`temp/` is git-ignored):

```bash
mkdir -p temp
gcc -O2 -Wall -Wextra -I firmware/app firmware/tools/test_imu_calib_fit.c firmware/app/imu_calib_fit.c -lm -o temp/test_imu_calib_fit && temp/test_imu_calib_fit
gcc -O2 -Wall -Wextra -I firmware/app firmware/tools/test_imu_calib_acc.c firmware/app/imu_calib_acc.c firmware/app/imu_calib_lsq.c -lm -o temp/test_imu_calib_acc && temp/test_imu_calib_acc
gcc -O2 -Wall -Wextra -I firmware/app firmware/tools/test_imu_calib_mag.c firmware/app/imu_calib_mag.c firmware/app/imu_calib_lsq.c -lm -o temp/test_imu_calib_mag && temp/test_imu_calib_mag
gcc -O2 -Wall -Wextra -I firmware/app firmware/tools/test_imu_calib_rot.c firmware/app/imu_calib_rot.c firmware/app/imu_calib_lsq.c -lm -o temp/test_imu_calib_rot && temp/test_imu_calib_rot
gcc -O2 -Wall -Wextra -I firmware/app firmware/tools/test_pole_aid.c firmware/app/pole_aid.c -lm -o temp/test_pole_aid && temp/test_pole_aid
gcc -O2 -Wall -Wextra -I firmware/app firmware/tools/test_pole_mount.c firmware/app/pole_aid.c firmware/app/pole_mount_fit.c firmware/app/imu_calib_lsq.c -lm -o temp/test_pole_mount && temp/test_pole_mount
gcc -O2 -Wall -Wextra -I firmware/app firmware/tools/test_ground_point.c firmware/app/ground_point.c -lm -o temp/test_ground_point && temp/test_ground_point
gcc -O2 -Wall -Wextra -I firmware/app -I firmware/drivers/flow_io -I common firmware/app/flow_nav.c firmware/tools/test_flow_nav.c -lm -o temp/test_flow_nav && temp/test_flow_nav
gcc -O2 -Wall -Wextra -I common firmware/tools/test_flow_link.c -o temp/test_flow_link && temp/test_flow_link
gcc -O2 -Wall -Wextra -I firmware/app firmware/tools/test_geoid.c firmware/app/geoid.c firmware/app/geoid_grid.c -lm -o temp/test_geoid && temp/test_geoid geoid/SWEN17_RH2000.txt
```

The same frame is checked on the Pi side by `pi/flow/build/test_flow` and `python3 pi/flow/python/test_flow_link.py`. Change `common/flow_link.h` and both sides in one commit.

## Raspberry Pi optical-flow program

On the Pi (Raspberry Pi OS or Debian 13), with OpenCV and libcamera from the OS packages:

```bash
sudo apt install cmake ninja-build g++ libopencv-dev libcamera-dev
cmake -S pi/flow -B pi/flow/build -G Ninja
ninja -C pi/flow/build
pi/flow/build/test_flow
```

The Pi needs no submodules, so a plain `git clone` is enough there. See [pi/flow/README.md](pi/flow/README.md) for the camera, the trigger wiring and running it.

## Geoid table for another area

`firmware/app/geoid_grid.c` is generated: Lantmäteriet's SWEN17_RH2000 grid (in `geoid/`, CC0) cut to a radius around the RTK base, because the whole grid doesn't fit in flash. To make it for your own base:

```bash
python firmware/tools/geoid_cut.py geoid/SWEN17_RH2000.txt --lat <base latitude> --lon <base longitude> --radius-km 100 -o firmware/app/geoid_grid.c
```

Then rebuild, and run `test_geoid` above to check the cut against the source grid. The model only covers Sweden; outside the cut the firmware uses the receiver's own geoid undulation. See [geoid/README.md](geoid/README.md).

## Rules for the code

- **Core 1 runs only the INSLIB filter**, in a plain loop that nothing preempts. Core 0 runs the UART DMA, the sensor parsers, the barometer, Wi-Fi and Bluetooth, written as separate modules with queues between them.
- **Timestamps are taken where the data arrives** (`time_us_64()` in the ISR or DMA-complete handler), not where it is processed.
- **The filter's hot path and the timestamping ISRs live in SRAM** (`__not_in_flash_func` or the linker override); check with `check_ram_path.py` after each build.
- **Every pin, bus instance and baud rate is in `firmware/app/board_pins.h`.** No other file names a GPIO number; drivers get their pins through their config structs.
- **GP16–21 (SPI0) stay reserved** for a W5500 Ethernet option. **ADC input 3 / GP29 must never be read**: the CYW43439 shares it.
- **Drivers** are Pico SDK static libraries in `firmware/drivers/<name>/`, each with its own `CMakeLists.txt`; `firmware/drivers/bmp581` is the pattern. Vendor code is a git submodule in `external/`.
- **The UM980 / UM982 is configured by the firmware at boot.** Never lock one of its ports (`CONFIG RXTYPE` can brick the receiver). Its own firmware is updated with Unicore's UPrecise over the module's USB-C, never from the Pico.
