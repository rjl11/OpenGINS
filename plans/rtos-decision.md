# RTOS decision: bare metal, FreeRTOS only on a named trigger

## Context
The firmware runs on a dual-core RP2350 ([rp2350-feasibility.md](rp2350-feasibility.md)). The filter needs deterministic 800 Hz timing, while the I/O side (UARTs, Bluetooth, Wi-Fi) is what an RTOS usually helps with. This plan decides whether to run on bare metal or on FreeRTOS, and how to keep a later move small. It was first written before the firmware existed and re-evaluated on 2026-11-16 against the built firmware and its bench figures ("Re-evaluation (2026-11-16)").

## Recommendation
Stay bare metal on the Pico SDK. Move core 0 to FreeRTOS only when one of the triggers under "When FreeRTOS pays off" is met. The first version of this plan named NTRIP over Wi-Fi or SD logging as the point to move; neither is coming to the Pico (see the re-evaluation).

## Bare-metal layout
- **Core 1:** only the INSLIB filter, in a plain loop fed by IMU, GNSS, barometer and speed samples from core 0 through `pico_util/queue` (`firmware/app/nav_core1.c`). Nothing preempts it, so the 800 Hz timing stays deterministic. How the filter is fed and read: [inslib-nav-suite.md](inslib-nav-suite.md).
- **Core 0:** UART DMA buffers, UM980 and TM210 parsers, BMP581 driver, the Bluetooth link (NMEA out, RTCM in from SW Maps; not built yet), the RTCM forwarder, the UM980 and TM210 configuration state machines ([imu-tm210-config.md](imu-tm210-config.md)), the web server, the IMU calibration, the field inputs and the USB log, all polled from one loop in `firmware/app/main.c`. RTCM and commands share UART1 TX → UM980 COM1 through one arbiter (see [rtcm-over-bluetooth.md](rtcm-over-bluetooth.md) and [gnss-um98x-config.md](gnss-um98x-config.md)). The W5500 Ethernet driver (WIZnet ioLibrary, hardware TCP/IP), with UDP `$PAOGI` output and RTCM in through the same COM1 arbiter, is on hold. It would also go on core 0 (see [ethernet-w5500-agopenweb.md](ethernet-w5500-agopenweb.md)). Wi-Fi and Bluetooth (BTstack) run in the background via `pico_cyw43_arch_lwip_threadsafe_background` (a build links only one arch library), with no RTOS needed ([web-config-server.md](web-config-server.md)).
- **Core 1 stack:** `nav_suite_update()` needs about 13 KB worst case ([inslib-nav-suite.md](inslib-nav-suite.md)). The Pico SDK's default stacks are much smaller (2 KB per core, `PICO_STACK_SIZE` / `PICO_CORE1_STACK_SIZE`). Give core 1 at least 16 KB, either by raising `PICO_CORE1_STACK_SIZE` or with `multicore_launch_core1_with_stack()` and a static buffer, and check the high-water mark on the bench.
- INSLIB has no OS dependencies and no heap, so it works either way.

## Re-evaluation (2026-11-16)
**The first triggers, and where they stand:**

| First trigger | Now |
|---|---|
| NTRIP client over Wi-Fi | Not needed: SW Maps does NTRIP on the phone ([rtcm-over-bluetooth.md](rtcm-over-bluetooth.md)). If it is ever wanted, the Zero 2 W companion owns it ([pico2-zero2w-usb-companion.md](pico2-zero2w-usb-companion.md)). |
| SD-card logging | Field logging is the USB log mode (`firmware/app/usb_log.c`): 54 kB/s, 0 bytes dropped in 60 s, also while `/status.json` was polled 5 times a second ([wheel-odometry-aiding.md](wheel-odometry-aiding.md), "Field testing"). Raw logging to SD or a USB stick belongs to the companion. |
| Web UI | Built bare metal (lwIP raw API, background arch) and bench-tested: station, AP fallback, config page, settings in flash ([web-config-server.md](web-config-server.md)). This trigger was too pessimistic. |
| Firmware updates, config shell | Firmware updates belong to the companion (SWD, picotool); configuration is the web page. |

**Headroom on core 0 (measured on the Pico 2 W, 2026-11-15, [rp2350-feasibility.md](rp2350-feasibility.md)):**
- Load 15–19 % with the USB log off, 26–27 % with it on. An idle pass takes 6–7.7 µs.
- The longest pass is about 10.5 ms, the 1 Hz pass that prints the status report and builds the status JSON. The IMU DMA ring holds about 200 ms and the core 1 IMU queue (`IMU_QUEUE_LEN` 128) 160 ms, so the margin is about 15×.
- The next large piece, Bluetooth, is BTstack in the same background arch as lwIP: event-driven callbacks, no task needed.

**What a move would cost now:**
- About 10k lines of core 0 code are written as polled, non-blocking state machines (TM210 and UM980 detect and provisioning, BMP581 re-init, the Wi-Fi join backoff, the IMU calibration, the USB log). They already cooperate in one loop; turning them into tasks gains nothing.
- Switch to `pico_cyw43_arch_lwip_sys_freertos`, with locking around the lwIP callbacks in `firmware/app/web/`.
- Move `cpu_load.h` from core 0's SysTick to the FreeRTOS idle hook.
- Check that `flash_safe_execute()` (`firmware/app/flash_rec.c`) still locks out bare-metal core 1 correctly with FreeRTOS on core 0.
- Re-run `firmware/tools/check_ram_path.py` with the port's interrupt code in the image.

**A timing risk an RTOS does not fix: saving to flash.** `flash_rec.c` pauses core 1 and masks core 0's interrupts for a sector erase plus the program. The IMU auto-calibration saves this way in the background ([imu-auto-calibration.md](imu-auto-calibration.md)), so it happens in the field, not only when someone saves on the web page. A 4 KB sector erase can take hundreds of ms at worst (estimate, not yet measured; check the figure in the flash chip's datasheet), against the IMU ring's 200 ms. Too long an erase drops IMU bytes, and a PPS edge can be missed while interrupts are masked. Measure it (the open item "a save with the IMU at 800 Hz" in `TODO.md`). If it is too long, save only while the unit is still, or less often; FreeRTOS would not change it.

**Verdict:** stay bare metal. The decision holds until a trigger below is met.

## When FreeRTOS pays off
Move core 0 to FreeRTOS when one of these is true:
- **A blocking third-party stack has to run on the Pico itself** and has only a sockets or RTOS API: TLS NTRIP without the companion, MQTT, FatFS on an SD card with long blocking writes.
- **An output needs lower latency than core 0's longest pass**, and that pass can't simply be split up, for example a low-latency link to a UAV flight controller. Try splitting the 1 Hz status pass across several loop passes first.
- **Core 0 load passes about 60 %**, or the longest pass comes within a few times of the ring margins.

## How to add it later
- Run FreeRTOS on core 0 only (`configNUMBER_OF_CORES = 1`) and keep the filter on bare-metal core 1, so the filter sees no scheduler jitter.
- Give any task that calls `nav_suite_update()` at least about 16 KB of stack (worst case about 13 KB).
- Cost: about 10–20 KB of RAM (estimate, not yet measured; 186 KB of the 520 KB SRAM is used today, [web-config-server.md](web-config-server.md), "Memory layout") and a few percent of CPU.
- Do the steps under "What a move would cost now".
- To keep the move small, keep writing core 0 code as separate non-blocking modules with queues or snapshots between them (the web handlers already read only a snapshot struct).
