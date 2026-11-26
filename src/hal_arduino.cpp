#ifdef ARDUINO
#include "hal.hpp"
#include <Arduino.h>

namespace hal {

uint32_t millis() { return ::millis(); }

void pin_mode(uint8_t pin, pin_dir dir) {
  ::pinMode(pin, dir == PIN_OUTPUT ? OUTPUT : INPUT);
}

void digital_write(uint8_t pin, bool level) {
  ::digitalWrite(pin, level ? HIGH : LOW);
}

void log(const char *msg) { Serial.print(msg); }
void log(long value) { Serial.print(value); }

} // namespace hal
#endif
