#pragma once
#include <stdint.h>

enum state {
  STARTUP = 0, // This is a catch for when the VCU is powering on
  TRACTIVE_SYSTEM_DISABLED = 1,    // GLV is on, but not TSV
  TRACTIVE_SYSTEM_PRECHARGING = 2, // TSV is comin'
  TRACTIVE_SYSTEM_ENERGIZED = 3,   // TSV is up, but RTD button isn't pressed
  TRACTIVE_SYSTEM_ENABLED = 4,     // Enable everything required to go fast
  READY_TO_DRIVE = 5,              // Try not to hit a curb plz
};

class VCU {
private:
  state current_state;

  uint16_t bool_code; // Encode all the possible gateing factors into a uint16_t
  uint16_t error_code; // So that we can get "error codes" when transitions fail

public:
  inline void init_state_machine() { this->current_state = STARTUP; }

  inline state get_current_state() { return this->current_state; }

  bool set_state(state target_state);

  inline uint16_t get_bool_code() { return this->bool_code; };
  inline uint16_t get_error_code() { return this->error_code; };
};
