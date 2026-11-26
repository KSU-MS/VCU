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
void delay_ms(uint32_t ms);

void pin_mode(uint8_t pin, pin_dir dir);
void digital_write(uint8_t pin, bool level);

void log(const char *msg);
void log(long value);
inline void logln(const char *msg) {
  log(msg);
  log("\r\n");
}

} // namespace hal
