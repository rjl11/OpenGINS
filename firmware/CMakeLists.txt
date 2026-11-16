# gnss_smart firmware: sensor bring-up + INSLIB nav_suite demo (Pico 2 W, bare metal).
#
# Build (PICO_SDK_PATH set; options, flashing and small build hosts in BUILDING.md):
#   cmake -S firmware -B firmware/build -G Ninja
#   ninja -C firmware/build
# Output: firmware/build/app.uf2 (and app.elf for SWD).

cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD pico2_w CACHE STRING "Board type")
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

include(pico_sdk_import.cmake)

project(gnss_smart C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_subdirectory(drivers/uart_dma)
add_subdirectory(drivers/bmp581)
add_subdirectory(drivers/tm210)
add_subdirectory(drivers/um980)
add_subdirectory(drivers/inslib)
add_subdirectory(drivers/pulse_in)
add_subdirectory(drivers/flow_io)

add_executable(app
    app/main.c
    app/die_temp.c
    app/nav_core1.c
    app/settings.c
    app/flash_rec.c
    app/imu_calib.c
    app/imu_calib_fit.c
    app/imu_calib_acc.c
    app/imu_calib_lsq.c
    app/imu_calib_mag.c
    app/imu_calib_rot.c
    app/field_io.c
    app/flow_nav.c
    app/geoid.c
    app/geoid_grid.c
    app/ground_point.c
    app/pole_aid.c
    app/pole_mount.c
    app/pole_mount_fit.c
    app/usb_log.c
)

target_include_directories(app PRIVATE app)

target_compile_options(app PRIVATE -Wall -Wextra)

# GNSS antenna mode default: single (UM980, or UM982 with the slave antenna
# off) or dual (UM982 with heading). See app/app_config.h.
set(GNSS_ANTENNA_MODE single CACHE STRING "GNSS antenna mode default: single or dual")
set_property(CACHE GNSS_ANTENNA_MODE PROPERTY STRINGS single dual)
if(GNSS_ANTENNA_MODE STREQUAL "dual")
    target_compile_definitions(app PRIVATE APP_GNSS_ANTENNA_MODE=1)
elseif(GNSS_ANTENNA_MODE STREQUAL "single")
    target_compile_definitions(app PRIVATE APP_GNSS_ANTENNA_MODE=0)
else()
    message(FATAL_ERROR "GNSS_ANTENNA_MODE must be single or dual, not '${GNSS_ANTENNA_MODE}'")
endif()

# Platform default (plans/imu-auto-calibration.md, "Gates per platform"): when
# the GNSS course over ground counts as a heading. The web page can override
# it; applied at reboot.
set(PLATFORM pole CACHE STRING "Platform default: tractor, pole, multirotor or fixed_wing")
set_property(CACHE PLATFORM PROPERTY STRINGS tractor pole multirotor fixed_wing)
if(PLATFORM STREQUAL "tractor")
    target_compile_definitions(app PRIVATE APP_PLATFORM=0)
elseif(PLATFORM STREQUAL "pole")
    target_compile_definitions(app PRIVATE APP_PLATFORM=1)
elseif(PLATFORM STREQUAL "multirotor")
    target_compile_definitions(app PRIVATE APP_PLATFORM=2)
elseif(PLATFORM STREQUAL "fixed_wing")
    target_compile_definitions(app PRIVATE APP_PLATFORM=3)
else()
    message(FATAL_ERROR "PLATFORM must be tractor, pole, multirotor or fixed_wing, not '${PLATFORM}'")
endif()

# Wi-Fi and the web server (plans/web-config-server.md, "When Wi-Fi runs").
# always: Wi-Fi and the web server from boot (the development default).
# switch: only when GPIO22 is pulled to GND at boot (production).
# off: no Wi-Fi code linked in.
set(WEB_MODE always CACHE STRING "Web server: always, switch or off")
set_property(CACHE WEB_MODE PROPERTY STRINGS always switch off)
set(WIFI_STA_SSID "MyWiFi" CACHE STRING "Wi-Fi network to join")
set(WIFI_STA_PASSWORD "" CACHE STRING "Wi-Fi password; empty for an open network")
# auto: station, open AP gnss_smart if not joined within 30 s of boot.
# sta: station only. ap: open AP from boot.
set(WIFI_MODE auto CACHE STRING "Wi-Fi network mode: auto, sta or ap")
set_property(CACHE WIFI_MODE PROPERTY STRINGS auto sta ap)
if(WIFI_MODE STREQUAL "auto")
    set(APP_WIFI_MODE_VALUE 0)
elseif(WIFI_MODE STREQUAL "sta")
    set(APP_WIFI_MODE_VALUE 1)
elseif(WIFI_MODE STREQUAL "ap")
    set(APP_WIFI_MODE_VALUE 2)
else()
    message(FATAL_ERROR "WIFI_MODE must be auto, sta or ap, not '${WIFI_MODE}'")
endif()
if(WEB_MODE STREQUAL "always" OR WEB_MODE STREQUAL "switch")
    if(WEB_MODE STREQUAL "always")
        set(APP_WEB_MODE_VALUE 1)
    else()
        set(APP_WEB_MODE_VALUE 2)
    endif()
    add_subdirectory(drivers/ap_servers)
    target_sources(app PRIVATE app/web/web.c app/web/jsonw.c app/web/config_web.c)
    target_link_libraries(app PRIVATE ap_servers)
    target_compile_definitions(app PRIVATE
        APP_WEB_MODE=${APP_WEB_MODE_VALUE}
        APP_WIFI_MODE=${APP_WIFI_MODE_VALUE}
        "APP_WIFI_STA_SSID=\"${WIFI_STA_SSID}\""
        "APP_WIFI_STA_PASSWORD=\"${WIFI_STA_PASSWORD}\""
        "CYW43_HOST_NAME=\"gnss-smart\""
    )
    target_link_libraries(app PRIVATE
        pico_cyw43_arch_lwip_threadsafe_background
        pico_lwip_http
        pico_lwip_mdns
    )
    pico_set_lwip_httpd_content(app PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/app/web/www/index.html
        ${CMAKE_CURRENT_LIST_DIR}/app/web/www/status.html
        ${CMAKE_CURRENT_LIST_DIR}/app/web/www/config.html
        ${CMAKE_CURRENT_LIST_DIR}/app/web/www/calib.html)
elseif(WEB_MODE STREQUAL "off")
    target_compile_definitions(app PRIVATE APP_WEB_MODE=0)
else()
    message(FATAL_ERROR "WEB_MODE must be always, switch or off, not '${WEB_MODE}'")
endif()

target_link_libraries(app PRIVATE
    pico_stdlib
    pico_multicore
    pico_util
    pico_sync
    pico_flash
    hardware_flash
    hardware_watchdog
    hardware_uart
    hardware_i2c
    hardware_adc
    uart_dma
    bmp581
    tm210
    um980
    inslib
    pulse_in
    flow_io
)

# Default flash (XIP) binary. A whole-image copy_to_ram build won't fit once
# the CYW43 Wi-Fi firmware and lwIP are linked in, so only the core 1 filter
# path (INSLIB, nav_core1.c and the SDK calls it makes) goes to SRAM, through
# the linker-script override in link/. The timestamping ISRs are marked
# __not_in_flash_func in their drivers. See plans/web-config-server.md,
# "Memory layout", and check with tools/check_ram_path.py.
pico_add_linker_script_override_path(app ${CMAKE_CURRENT_LIST_DIR}/link
    FILES default_text_excludes.incl default_rodata_excludes.incl)

# Debug output on USB CDC. UART0 is the TM210, so stdio must never use it.
pico_enable_stdio_usb(app 1)
pico_enable_stdio_uart(app 0)

# USB log mode (app/usb_log.h): a larger CDC transmit FIFO than the SDK's 64
# bytes, so the 50-70 kB/s field-test stream leaves in bigger pieces.
target_compile_definitions(app PRIVATE CFG_TUD_CDC_TX_BUFSIZE=2048)

# Shared IRQ handler slots: the SDK default (6 with stdio_usb) is used up by
# USB, CYW43 and the GPIO raw handlers (PPS, BMP581 INT, the speed inputs,
# the camera strobe); a hard assert at boot otherwise.
target_compile_definitions(app PRIVATE PICO_MAX_SHARED_IRQ_HANDLERS=12)

pico_add_extra_outputs(app)
