# The Pico's side of the optical-flow link: camera trigger (PWM), strobe
# timestamps and the receive-only PIO UART (plans/optical-flow-aiding.md,
# common/flow_link.h).
#
# Usage from the firmware project, after pico_sdk_init():
#   add_subdirectory(drivers/flow_io)
#   target_link_libraries(app PRIVATE flow_io)

add_library(flow_io STATIC ${CMAKE_CURRENT_LIST_DIR}/flow_io.c)

pico_generate_pio_header(flow_io ${CMAKE_CURRENT_LIST_DIR}/flow_uart_rx.pio)

target_include_directories(flow_io PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/../../../common
)

target_link_libraries(flow_io PRIVATE
    hardware_clocks hardware_dma hardware_gpio hardware_irq hardware_pio hardware_pwm hardware_sync pico_time
)
