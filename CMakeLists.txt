cmake_minimum_required(VERSION 3.17)
project(vcu C CXX)

# Common ARM/Teensy flags
set(TEENSY_COMMON_FLAGS "-mthumb -mcpu=cortex-m7 -mfloat-abi=hard -mfpu=fpv5-d16 -Wno-unused-function -Wno-tautological-compare -Wno-cpp")

# Make sure both C and C++ use the same ABI/FPU
set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} ${TEENSY_COMMON_FLAGS}"   CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TEENSY_COMMON_FLAGS}" CACHE STRING "" FORCE)

include(${CMAKE_SOURCE_DIR}/cmake_declare_and_fetch.cmake.in)
include(${CMAKE_SOURCE_DIR}/cmake/libdeps.cmake)

DeclareAndFetchNoSource(teensy_cmake_macros https://github.com/newdigate/teensy-cmake-macros.git main)
include(${teensy_cmake_macros_SOURCE_DIR}/CMakeLists.include.txt)

import_arduino_library(cores ${teensy_cores_SOURCE_DIR}/teensy4 avr debug util)

import_arduino_library_git(SPI https://github.com/PaulStoffregen/SPI.git master "")
import_arduino_library_git(SdFat https://github.com/PaulStoffregen/SdFat.git master "src" common DigitalIO ExFatLib FatLib FsLib iostream SdCard SpiDriver)
import_arduino_library_git(SD https://github.com/PaulStoffregen/SD.git Juse_Use_SdFat src)
import_arduino_library_git(Time https://github.com/PaulStoffregen/Time.git master "")
#   https://github.com/tonton81/FlexCAN_T4
# 	https://github.com/KSU-MS/ksu-fw-common
# 	https://github.com/KSU-MS/pio-git-hash-gen
# 	https://github.com/KSU-MS/ksu-ms-dbc/releases/download/105/can_lib.tar.gz

import_arduino_library_git(FlexCAN_T4 https://github.com/tonton81/FlexCAN_T4.git master "")
import_arduino_library_git(ksu_fw_common https://github.com/KSU-MS/ksu-fw-common.git main "")
fetch_and_add_url_lib(ksu_ms_dbc https://github.com/KSU-MS/ksu-ms-dbc/releases/latest/download/can_lib.tar.gz "")

add_subdirectory(src)