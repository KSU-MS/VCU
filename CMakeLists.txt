cmake_minimum_required(VERSION 3.17)
project(vcu C CXX)

# Common ARM/Teensy flags
set(TEENSY_COMMON_FLAGS "-mthumb -mcpu=cortex-m7 -mfloat-abi=hard -mfpu=fpv5-d16 -Wno-unused-function -Wno-tautological-compare -Wno-cpp")

# Make sure both C and C++ use the same ABI/FPU
set(CMAKE_C_FLAGS   "${TEENSY_COMMON_FLAGS}"   CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS "${TEENSY_COMMON_FLAGS}" CACHE STRING "" FORCE)

find_package(Python3 COMPONENTS Interpreter REQUIRED)

include(${CMAKE_SOURCE_DIR}/cmake/cmake_declare_and_fetch.cmake.in)
include(${CMAKE_SOURCE_DIR}/cmake/libdeps.cmake)

DeclareAndFetchNoSource(teensy_cmake_macros https://github.com/newdigate/teensy-cmake-macros.git main)
include(${teensy_cmake_macros_SOURCE_DIR}/CMakeLists.include.txt)

import_arduino_library(cores ${teensy_cores_SOURCE_DIR}/teensy4 avr debug util)

set(PACKAGES_JSON ${CMAKE_SOURCE_DIR}/packages.json)
set(PACKAGES_CMAKE ${CMAKE_BINARY_DIR}/generated_packages.cmake)

if(NOT EXISTS ${PACKAGES_JSON})
    message(FATAL_ERROR "packages.json not found at ${PACKAGES_JSON}")
endif()

execute_process(
    COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_SOURCE_DIR}/cmake/generate_packages_cmake.py
            ${PACKAGES_JSON}
            ${PACKAGES_CMAKE}
    RESULT_VARIABLE PACKAGES_GEN_RESULT
)

if(NOT PACKAGES_GEN_RESULT EQUAL 0)
    message(FATAL_ERROR "Failed to generate ${PACKAGES_CMAKE} from ${PACKAGES_JSON}")
endif()

include(${PACKAGES_CMAKE})

add_subdirectory(src)