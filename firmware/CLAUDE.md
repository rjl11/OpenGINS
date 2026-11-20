# firmware/CLAUDE.md

The Pico firmware's modules and drivers. This loads with the root `CLAUDE.md` (architecture, core split, pins, build) whenever a file under `firmware/` is opened. Its rules still apply here.

## Application modules (`firmware/app/`)

- `main.c`: the core 0 loop and the 1 Hz status report.
- `nav_core1.c`: the filter on core 1 and its queues.
- `cpu_load.h`: header-only load meter for both cores' polling loops, timed with each core's SysTick because the M33 has no DWT cycle counter. Each loop flags its work passes, and idle passes are charged the mean of the typical idle pass. Shown as the `CPU` lines and `cpu` in `/status.json`. A FreeRTOS move would take the SysTick, so switch to its idle hook then.
- `die_temp.c`: the RP2350's on-chip temperature sensor on the internal ADC input, read with the status report. Uncalibrated, so a few degrees off. Nothing else uses the ADC, and ADC input 3 / GP29 must never be read because the CYW43439 shares it.
- `app_config.h`: mounting, noise figures, options.
- `settings.c`: web-page settings in flash, including the sensors fitted (barometer, magnetometer, optical flow, wheel pulses), each on/off and applied live. New fields are appended and `settings_load()` migrates older records by size.
- `flash_rec.c`: the two-sector flash record both use.
- `imu_calib.c` with `imu_calib_fit.c`, `imu_calib_acc.c`, `imu_calib_mag.c`, `imu_calib_rot.c` and the shared solver `imu_calib_lsq.c`: gyro bias vs temperature and scale / misalignment, the accelerometer from static poses and the magnetometer, learned in the background, applied at boot, managed on the web page `calib.html`. Host tests `firmware/tools/test_imu_calib_*.c`.
- `field_io.c`: the wheel / ground-speed inputs and the optical-flow link on core 0: status figures and the raw pulses per metre. Wheel speed is not fused yet.
- `speed_ppm.c`: that raw pulses-per-metre figure, edge-timed: input A's pulses between two rising edges over the GNSS distance between their times, runs of at least `APP_SPEED_PPM_MIN_SPAN_S`. Host test `firmware/tools/test_speed_ppm.c`.
- `flow_nav.c`: the Pi's flow to a ground speed: frame matching by the left-out trigger pulse, strobe timing, gyro de-rotation, tilt-corrected height, lever arm, 10 Hz averaging. Pushed with `nav_push_speed()` into INSLIB's speed input on core 1, a queue shared with wheel speed later. Camera height, image direction and lever arm are on the web config page, defaults in `APP_FLOW_*`. Host test `firmware/tools/test_flow_nav.c`.
- `geoid.c` with the generated `geoid_grid.c`: geoid height for `h_geoid_m` in `/status.json` and the USB status. Lantmäteriet's SWEN17_RH2000 cut to 100 km around the RTK base by `firmware/tools/geoid_cut.py` from `geoid/SWEN17_RH2000.txt` (source and format in `geoid/README.md`). An RH 2000 height only with RTK from the SWEREF 99 base; outside the grid, the receiver's undulation. Host test `firmware/tools/test_geoid.c`.
- `ground_point.c` (tractor / survey pole): with the web setting `ant_height_m` above 0, `best_position()` outputs the ground point below the antenna: the IMU point moved to the antenna by the lever arm (turned with the filter's attitude whenever it has one), then by the height along the body down axis, tilted only while the INS attitude passes `APP_GROUND_TILT_MAX_SD_DEG`, else straight down. Host test `firmware/tools/test_ground_point.c`.
- `pole_aid.c` (survey pole, core 1, in SRAM): planted-tip detector, the tip held through INSLIB's lever-armed local-position input, a linear yaw fix per planted sway, and a magnetometer trust gate. `plans/pole-tilt-aiding.md`, thresholds `APP_POLE_*`. Host test `firmware/tools/test_pole_aid.c`.
- `pole_mount.c` with `pole_mount_fit.c` (survey pole, core 0): the IMU's tilt against the pole axis, learned from the twist of planted episodes that `pole_aid` sends through a queue, kept in its own flash record, applied at boot to the tip lever arm and the ground point, managed on `calib.html`. The pole length is only checked against `ant_height_m`. `plans/pole-tilt-aiding.md` item 5, thresholds `APP_POLE_MOUNT_*`. Host test `firmware/tools/test_pole_mount.c`.
- `paogi.c`: the `$PAOGI` field rules, `paogi_fill()`, and the formatter for AgOpenGPS / AgOpenWeb (`plans/ethernet-w5500-agopenweb.md`). On the tractor platform `main.c` sends it on USB at 10 Hz between the status text, with the true ground point when the antenna height is set (else the antenna; AgOpenGPS / AgOpenWeb then run with an antenna height of 0). USB command `paogi on|off`, held while `log on`. Host test `firmware/tools/test_paogi.c`.
- `eth.c`: W5500 Ethernet on core 0 when the web setting `eth_fitted` is on, applied live with the static IP / mask / gateway.
  - `$PAOGI` to the subnet broadcast on UDP 9999 on any platform while the link is up.
  - RTCM in on UDP 2233, put back into frames by `rtcm_frame.c` (host test `firmware/tools/test_rtcm_frame.c`), passed through the one-source arbiter `rtcm_src.c` (web setting `rtcm_src`) and queued whole for COM1 with `um980_send_rtcm()` while the receiver is in RUN.
  - Flow frames from the front camera unit on UDP 5800 into `field_io_usb_flow()` (the Pi sends them with `flow --udp`). They are timed by the unit's sensor clock through the clock sync `clock_sync.c` once it is valid (time request / reply in `common/flow_link.h`, a line through the shortest round trips; host test `firmware/tools/test_clock_sync.c`), else by arrival.
  - AgIO's scan and hello on UDP 8888 answered by `aog_pgn.c` (host test `firmware/tools/test_aog_pgn.c`, also for `rtcm_src.c`).
  - The `ETH` / `RTCM` status lines and `eth` / `corrections` in `/status.json`.
- `usb_log.c`: the field-test log mode. `log on` on the USB port streams every sensor, the INSLIB output, the speed edges and the flow link as framed binary records, with printf going into TEXT records. Recorded and decoded with `firmware/tools/field_log.py`, which reconnects after a Pico reboot and gives every row a GPS time from the GNSS / TIME records (`plans/wheel-odometry-aiding.md`, "Field testing").
- `web/` with `lwipopts.h`: the Wi-Fi web server.

`inslib_replay.c` / `inslib_replay.sh` / `inslib_replay_plot.py` in `firmware/tools/` run a log through INSLIB on a host with simulated RTK GNSS; keep its filter setup in step with `nav_core1.c`.

## Drivers (`firmware/drivers/`)

- `uart_dma`: DMA ring receive, polled transmit.
- `tm210` and `um980`: a parser plus a boot check/provision state machine each; `um980` drives both the UM980 and the UM982.
- `bmp581`, `inslib` (CMake wrapper around `external/INSLIB`).
- `pulse_in`: edge timestamps from the two speed inputs.
- `flow_io`: camera trigger PWM, strobe timestamps, receive-only PIO UART for the flow link.
- `w5500`: register-level WIZnet W5500 driver, UDP sockets and the PHY link, non-blocking send.
- `ap_servers`: the AP's vendored DHCP/DNS servers.

The GPIO raw handlers each take an SDK shared-IRQ slot, so the app raises `PICO_MAX_SHARED_IRQ_HANDLERS` (a hard assert at boot otherwise). The PWM interrupt is exclusive to `flow_io`.

Each driver in `firmware/drivers/<name>/` is a Pico SDK static library with its own `CMakeLists.txt`. The firmware project pulls it in after `pico_sdk_init()` with `add_subdirectory(drivers/<name>)` and `target_link_libraries(app PRIVATE <name>)`. Vendor code is a git submodule in `external/`, compiled into the driver library from its path relative to the driver.

`firmware/drivers/bmp581` is the pattern to follow. It is thin glue around Bosch's API, with I2C timeouts so a stuck bus can't hang core 0, and an INT-pin edge timestamp that is read later by `bmp581_poll()` from the main loop. It is not thread safe and runs on core 0. Driver error codes are below -100, so they don't collide with the vendor's negative codes. It defines `BMP5_USE_FLOATING_POINT` because the RP2350 has an FPU.
