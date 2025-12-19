cmake_minimum_required(VERSION 3.17)
project(vcu)

file(GLOB_RECURSE SOURCE_FILES ${CMAKE_SOURCE_DIR}/src/*.cpp ${CMAKE_SOURCE_DIR}/src/*.c)
file(GLOB_RECURSE HEADER_FILES ${CMAKE_SOURCE_DIR}/include/*.hpp ${CMAKE_SOURCE_DIR}/include/*.h)

teensy_add_library(vcu STATIC ${SOURCE_FILES} ${HEADER_FILES})

add_custom_target(installarduino
"${CMAKE_COMMAND}" -P "${CMAKE_SOURCE_DIR}/cmake/install_arduino_library.cmake")

project(firmware)

teensy_include_directories(../include)
teensy_add_executable(firmware ${SOURCE_FILES})

string(TIMESTAMP RTC_LOCALTIME "%s" UTC)
target_link_options(firmware.elf PRIVATE "-Wl,--defsym=__rtc_localtime=${RTC_LOCALTIME}")

set(FIRMWARE_VENDOR_LIBS
    vcu
    SPI
    SdFat
    SD
    Time
    ksu_fw_common
    ksu_ms_dbc
    cores
)

if(TARGET FlexCAN_T4.o)
    list(APPEND FIRMWARE_VENDOR_LIBS FlexCAN_T4)
endif()

teensy_target_link_libraries(firmware ${FIRMWARE_VENDOR_LIBS})

set(FIRMWARE_LINK_LIBS c gcc m stdc++)

if(NOT ARM_CORTEXM7_DSP_LIB)
    find_library(ARM_CORTEXM7_DSP_LIB
        NAMES arm_cortexM7lfsp_math
        PATHS
            "$ENV{HOME}/.platformio/packages/framework-arduinoteensy/cores/teensy4"
            "$ENV{HOME}/.platformio/packages/framework-arduinoteensy/libraries"
            "${teensy_cores_SOURCE_DIR}/teensy4"
        NO_DEFAULT_PATH
    )
endif()

if(ARM_CORTEXM7_DSP_LIB)
    list(APPEND FIRMWARE_LINK_LIBS ${ARM_CORTEXM7_DSP_LIB})
else()
    message(WARNING "libarm_cortexM7lfsp_math not found; skipping link step")
endif()

target_link_libraries(firmware.elf ${FIRMWARE_LINK_LIBS})
