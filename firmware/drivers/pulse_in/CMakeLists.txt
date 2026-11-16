# Edge timestamps from the wheel / ground-speed inputs
# (plans/wheel-odometry-aiding.md).
#
# Usage from the firmware project, after pico_sdk_init():
#   add_subdirectory(drivers/pulse_in)
#   target_link_libraries(app PRIVATE pulse_in)

add_library(pulse_in STATIC ${CMAKE_CURRENT_LIST_DIR}/pulse_in.c)

target_include_directories(pulse_in PUBLIC ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(pulse_in PRIVATE hardware_gpio hardware_irq hardware_sync pico_time)
