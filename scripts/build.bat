@echo off
rmdir /s /q build
del /q compile_commands.json 2>NUL
where ninja >nul 2>nul
if %ERRORLEVEL% NEQ 0 (
    echo Note: 'ninja' build tool not installed, using standard cmake build system instead (Install ninja for faster builds).
    cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=cmake/toolchains/teensy41.cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
) else (
    cmake -S . -B build -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/toolchains/teensy41.cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
)
move build/compile_commands.json compile_commands.json >nul
cmake --build build
