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

// i.MX RT1060 RM 58.4.2: reconfigure within 128 bus clocks of the unlock,
// so no interrupt in between. Left alone if the bootloader didn't arm it
// (firmware loaded without it) or it's already locked.
void watchdog_start() {
  if (!(WDOG3_CS & WDOG_CS_EN) || !(WDOG3_CS & WDOG_CS_UPDATE))
    return;
  __disable_irq();
  WDOG3_CNT = 0xD928C520; // unlock
  while (!(WDOG3_CS & WDOG_CS_ULK))
    ;
  WDOG3_TOVAL = 32768; // 1 s of LPO clock
  // UPDATE = 0: nothing can change or stop it now until the next reset
  WDOG3_CS = WDOG_CS_EN | WDOG_CS_CLK(1) | WDOG_CS_CMD32EN;
  while (!(WDOG3_CS & WDOG_CS_RCS))
    ;
  __enable_irq();
}

// The 32-bit refresh sequence (the bootloader sets CS[CMD32EN]), i.MX RT1060
// RM 58.4.4. One store, so nothing can interleave with it.
void watchdog_feed() { WDOG3_CNT = 0xB480A602; }

bool reset_by_watchdog() {
  static int cached = -1;
  if (cached < 0) {
    cached = (SRC_SRSR & SRC_SRSR_WDOG3_RST_B) != 0;
    SRC_SRSR = SRC_SRSR_WDOG3_RST_B; // write 1 to clear, for the next boot
  }
  return cached;
}

void log(const char *msg) { Serial.print(msg); }
void log(long value) { Serial.print(value); }

} // namespace hal
#endif
