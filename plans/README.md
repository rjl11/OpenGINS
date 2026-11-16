# Plans

Planning documents for OpenGINS: system architecture, hardware selection (MCU, sensors), sensor fusion design and roadmap.

| Plan | Topic |
|---|---|
| [rp2350-feasibility.md](rp2350-feasibility.md) | Pico 2 W / RP2350 CPU budget, serial port layout, Bluetooth output |
| [esp32-p4-feasibility.md](esp32-p4-feasibility.md) | ESP32-P4 as an alternative MCU |
| [esp32-s3-feasibility.md](esp32-s3-feasibility.md) | ESP32-S3 as an alternative MCU: CPU budget, UART count, Wi-Fi/BLE, risks |
| [esp32-s3-wifi-companion.md](esp32-s3-wifi-companion.md) | Plain Pico 2 + ESP32-S3 as Wi-Fi/BLE companion: link, protocol, split of work |
| [pico2-zero2w-usb-companion.md](pico2-zero2w-usb-companion.md) | Plain Pico 2 + Pi Zero 2 W as Linux companion over USB: split of work, RTCM into COM2, stratum 1 NTP, SWD debug rig, serial over TCP to the receiver |
| [inslib-nav-suite.md](inslib-nav-suite.md) | INSLIB integration through `nav_suite`: options, epoch building, outputs |
| [imu-tm210.md](imu-tm210.md) | TM210 IMU configuration, protocol, unit/frame conversion and calibration for INSLIB |
| [imu-tm210-config.md](imu-tm210-config.md) | TM210 auto-configuration by the MCU: Setting object, boot check/provision state machine, RAM-then-flash commit |
| [imu-auto-calibration.md](imu-auto-calibration.md) | Background IMU and magnetometer calibration on core 0 for tractor, survey pole and UAV: observability per platform, prior-based fits, gates, flash record, apply at boot |
| [imu-adis16465-vs-tm210.md](imu-adis16465-vs-tm210.md) | ADIS16465-2 as an alternative IMU: datasheet comparison, system effect, firmware impact |
| [gnss-mosaic-x5-vs-um980.md](gnss-mosaic-x5-vs-um980.md) | Septentrio mosaic-X5 as an alternative GNSS receiver: datasheet comparison, system effect, firmware impact |
| [gnss-um98x-config.md](gnss-um98x-config.md) | UM980 / UM982 auto-configuration by the MCU: single/dual antenna mode (compile-time default, web override, applied at reboot), per-mode command set, boot check/provision state machine, COM1 shared by commands and RTCM, dual-antenna heading into INSLIB `m.yaw`, receiver firmware builds and what their change logs mean here |
| [rtcm-over-bluetooth.md](rtcm-over-bluetooth.md) | RTCM from SW Maps (Android NTRIP) over Bluetooth to UM980 COM1; replaces the radio |
| [optical-flow-aiding.md](optical-flow-aiding.md) | Optical-flow velocity from a global-shutter camera on a Raspberry Pi: Pi/Pico split, Pico-triggered timing, PIO UART link, height per platform, INSLIB speed input now and a body-velocity input later, camera choice |
| [wheel-odometry-aiding.md](wheel-odometry-aiding.md) | Wheel and radar ground speed on a tractor (ISO 11786 at 130 pulses/m, ISOBUS, a Hall pickup or a quadrature encoder on two inputs) into INSLIB's speed input: one rear wheel corrected with the gyro turn rate, auto-learning of pulses per metre and geometry under RTK (user only wires pulses), field tests live plus a USB log of every sensor and the flow link, input stage, pins; why the steering angle is not fused |
| [pole-tilt-aiding.md](pole-tilt-aiding.md) | Survey pole: heading and tilt from the natural sway of a planted pole (planted-tip detector, tip constraint into INSLIB, linear yaw fix per episode, magnetometer trust gate), after the Leica GS18 T and Wuhan tilt-RTK papers in `docs/survey-pole/` |
| [rtos-decision.md](rtos-decision.md) | Bare metal; FreeRTOS on core 0 only on a named trigger (re-evaluated 2026-11-16) |
| [web-config-server.md](web-config-server.md) | Web config and diagnostics: always on in development (station on `MyWiFi`), open AP `OpenGINS` later, GPIO22 switch for production; flash memory layout with the hot path in SRAM |

## On hold
These plans are not part of the current build, but they are kept for later.

| Plan | Topic |
|---|---|
| [ethernet-w5500-agopenweb.md](ethernet-w5500-agopenweb.md) | W5500 Ethernet, `$PAOGI` output for AgOpenWeb, RTCM in. SPI0 (GP16–21) stays reserved for it |
