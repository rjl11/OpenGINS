# WIZnet W5500 Ethernet on SPI, UDP only (plans/ethernet-w5500-agopenweb.md).
#
# Usage from the firmware project, after pico_sdk_init():
#   add_subdirectory(drivers/w5500)
#   target_link_libraries(app PRIVATE w5500)

add_library(w5500 STATIC ${CMAKE_CURRENT_LIST_DIR}/w5500.c)

target_include_directories(w5500 PUBLIC ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(w5500 PUBLIC hardware_spi PRIVATE hardware_gpio pico_time)
