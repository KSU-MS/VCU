#pragma once
#include <stdint.h>

// Thin hardware abstraction so the control logic can be built and tested off
// the Teensy. On target these forward straight to the Arduino core
// (src/hal_arduino.cpp); natively they're backed by a simulator
// (test/sim/hal_sim.cpp) that tests can poke at.
//
// Names avoid LOW/HIGH/INPUT/OUTPUT since Arduino #defines those
namespace hal {

enum pin_dir : uint8_t { PIN_INPUT = 0, PIN_OUTPUT = 1 };

uint32_t millis();

void pin_mode(uint8_t pin, pin_dir dir);
void digital_write(uint8_t pin, bool level);

// The RTWDOG the bootloader armed for booting (4 s): watchdog_start()
// shortens it to 1 s and locks it, once the firmware is up and about to run
// its loop. From then on feed it or the board resets; VCU::tick() feeds it
// as its last act.
void watchdog_start();
void watchdog_feed();
// Was the last reset the watchdog? Read (and cleared in hardware) once at
// boot, the same answer after that.
bool reset_by_watchdog();

void log(const char *msg);
void log(long value);
inline void logln(const char *msg) {
  log(msg);
  log("\r\n");
}

} // namespace hal
