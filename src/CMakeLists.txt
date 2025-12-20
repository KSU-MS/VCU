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

teensy_target_link_libraries(firmware vcu ${PACKAGES_DECLARED} cores)

set(FIRMWARE_LINK_LIBS c gcc m stdc++)

target_link_libraries(firmware.elf ${FIRMWARE_LINK_LIBS})
