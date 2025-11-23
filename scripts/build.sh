#!/bin/bash
if ! command -v ninja &> /dev/null
then
    echo -e "\033[1;33mNote: 'ninja' build tool not installed, using standard cmake build system instead (Install ninja for faster builds).\033[0m" >&2
    cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=cmake/toolchains/teensy41.cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
else
    cmake -S . -B build -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/toolchains/teensy41.cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
fi
ln -sf build/compile_commands.json ./compile_commands.json
cmake --build build