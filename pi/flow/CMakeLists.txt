# Optical flow on the Raspberry Pi (C++). OpenCV and libcamera come from the
# OS packages (libopencv-dev, libcamera-dev); see README.md.
#
#   cmake -S pi/flow -B pi/flow/build -G Ninja && ninja -C pi/flow/build
#   pi/flow/build/test_flow
cmake_minimum_required(VERSION 3.16)
project(gnss_smart_flow CXX)

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

add_library(flow_core STATIC src/flow_lk.cpp)
target_include_directories(flow_core PUBLIC src ${COMMON_DIR})
target_link_libraries(flow_core PUBLIC ${OpenCV_LIBS})

add_executable(flow src/main.cpp src/serial.cpp)
target_link_libraries(flow PRIVATE flow_core)
if(LIBCAMERA_FOUND)
  target_sources(flow PRIVATE src/camera.cpp)
  target_compile_definitions(flow PRIVATE FLOW_WITH_LIBCAMERA)
  target_link_libraries(flow PRIVATE PkgConfig::LIBCAMERA)
  message(STATUS "flow: libcamera ${LIBCAMERA_VERSION}, camera capture built")
else()
  message(WARNING "flow: libcamera not found, only --frames DIR will work")
endif()

add_executable(test_flow tests/test_flow.cpp)
target_link_libraries(test_flow PRIVATE flow_core)

enable_testing()
add_test(NAME test_flow COMMAND test_flow)
