#pragma once
#include <stdint.h>

// When a message that safety decisions rest on was last heard
struct Freshness {
  bool heard_ever = false;
  uint32_t at_ms = 0;

  void heard(uint32_t now) {
    heard_ever = true;
    at_ms = now;
  }
  bool fresh(uint32_t now, uint32_t timeout_ms) const {
    return heard_ever && now - at_ms <= timeout_ms;
  }
};
