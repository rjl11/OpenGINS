# Optical flow on the Raspberry Pi (C++). OpenCV and libcamera come from the
# OS packages (libopencv-dev, libcamera-dev); see README.md.
#
#   cmake -S pi/flow -B pi/flow/build -G Ninja && ninja -C pi/flow/build
#   pi/flow/build/test_flow && pi/flow/build/test_tfluna && pi/flow/build/test_tm171
cmake_minimum_required(VERSION 3.16)
project(opengins_flow CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra)

find_package(OpenCV REQUIRED COMPONENTS core imgproc video calib3d imgcodecs)
find_package(PkgConfig)
if(PkgConfig_FOUND)
  pkg_check_modules(LIBCAMERA IMPORTED_TARGET libcamera)
endif()

# Headers shared with the Pico firmware (the link format).
set(COMMON_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../common)

# The camera preview web page (src/preview_http.hpp) starts unless
# --no-preview is given; a production build sets this OFF, and then the page
# needs --preview.
option(FLOW_PREVIEW_DEFAULT "flow serves the camera preview page unless --no-preview" ON)
find_package(Threads REQUIRED)

add_library(flow_core STATIC src/flow_lk.cpp src/preview_metrics.cpp)
target_include_directories(flow_core PUBLIC src ${COMMON_DIR})
target_link_libraries(flow_core PUBLIC ${OpenCV_LIBS})

# TF-Luna lidar (src/tfluna.hpp): the protocol without OpenCV or OS calls,
# then the serial reader for flow --lidar and the bench tool tfluna.
# The serial port (src/serial.hpp) is its own library: flow uses it for the
# Pico link, tfluna_reader for the lidar.
add_library(flow_serial STATIC src/serial.cpp)
target_include_directories(flow_serial PUBLIC src)
add_library(tfluna_core STATIC src/tfluna.cpp)
target_include_directories(tfluna_core PUBLIC src)
add_library(tfluna_reader STATIC src/tfluna_reader.cpp)
target_link_libraries(tfluna_reader PUBLIC tfluna_core flow_serial Threads::Threads)
add_executable(tfluna src/tfluna_main.cpp)
target_link_libraries(tfluna PRIVATE tfluna_reader)

# TM171 IMU (src/tm171.hpp): the EasyProfile codec, clock map, gyro history
# and de-rotation without OS calls, then its serial reader for flow --tm171
# and the bench tool tm171.
add_library(tm171_core STATIC src/tm171.cpp)
target_include_directories(tm171_core PUBLIC src)
add_library(tm171_reader STATIC src/tm171_reader.cpp)
target_link_libraries(tm171_reader PUBLIC tm171_core flow_serial Threads::Threads)
target_include_directories(tm171_reader PUBLIC ${COMMON_DIR})  # flow_link.h: the IMU batches
add_executable(tm171 src/tm171_main.cpp)
target_link_libraries(tm171 PRIVATE tm171_reader)

# V4L2 capture is always built: for boards where libcamera can't drive the
# sensor, such as the ROCK 4SE (README, "Setup on a Radxa ROCK 4SE").
add_executable(flow src/main.cpp src/udp.cpp src/v4l2_camera.cpp src/preview_http.cpp)
target_link_libraries(flow PRIVATE flow_core flow_serial tfluna_reader tm171_reader Threads::Threads)
if(FLOW_PREVIEW_DEFAULT)
  target_compile_definitions(flow PRIVATE FLOW_PREVIEW_DEFAULT=1)
else()
  target_compile_definitions(flow PRIVATE FLOW_PREVIEW_DEFAULT=0)
endif()
if(LIBCAMERA_FOUND)
  target_sources(flow PRIVATE src/camera.cpp)
  target_compile_definitions(flow PRIVATE FLOW_WITH_LIBCAMERA)
  target_link_libraries(flow PRIVATE PkgConfig::LIBCAMERA)
  message(STATUS "flow: libcamera ${LIBCAMERA_VERSION}, camera capture built")
else()
  message(WARNING "flow: libcamera not found, camera capture through V4L2 only")
endif()

add_executable(test_flow tests/test_flow.cpp)
target_link_libraries(test_flow PRIVATE flow_core)

add_executable(test_tfluna tests/test_tfluna.cpp)
target_link_libraries(test_tfluna PRIVATE tfluna_core)

add_executable(test_tm171 tests/test_tm171.cpp)
target_link_libraries(test_tm171 PRIVATE tm171_core)

enable_testing()
add_test(NAME test_flow COMMAND test_flow)
add_test(NAME test_tfluna COMMAND test_tfluna)
add_test(NAME test_tm171 COMMAND test_tm171)
