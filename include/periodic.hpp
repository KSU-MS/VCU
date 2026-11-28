#pragma once
#include <stdint.h>

// Drop-in for Metro(interval, true) that takes the time as an argument
// instead of calling millis(), so it can run off a simulated clock
class Periodic {
private:
  uint32_t interval_ms;
  uint32_t last_ms = 0;

public:
  explicit Periodic(uint32_t interval_ms) : interval_ms(interval_ms) {}

  inline bool check(uint32_t now_ms) {
    if (now_ms - last_ms >= interval_ms) {
      last_ms = now_ms;
      return true;
    }
    return false;
  }

  inline void reset(uint32_t now_ms) { last_ms = now_ms; }
};
