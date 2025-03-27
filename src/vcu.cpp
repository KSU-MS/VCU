#include "vcu.hpp"

bool VCU::try_ts_energized() { return false; }

bool VCU::try_ts_enabled() { return false; }

bool VCU::ts_safe() { return false; }

bool VCU::set_state(state target_state) {
  switch (current_state) {
  case STARTUP: // This is just a catch for evil starts
    if (target_state == TRACTIVE_SYSTEM_DISABLED) {
      this->current_state = TRACTIVE_SYSTEM_DISABLED;
      return true;
    } else {
      this->error_code = this->bool_code;
      this->current_state = STARTUP;
      return false;
    }
    break;

  case TRACTIVE_SYSTEM_DISABLED:
    return false;
    break;

  case TRACTIVE_SYSTEM_PRECHARGING:
    return false;
    break;

  case TRACTIVE_SYSTEM_ENERGIZED:
    return false;
    break;

  case TRACTIVE_SYSTEM_ENABLED:
    if (target_state == READY_TO_DRIVE) {
      // Check to make sure we can actually enter READY_TO_DRIVE

      return true;
    } else {
      this->error_code = this->bool_code;

      // Disable everything and go back to TRACTIVE_SYSTEM_DISABLED

      this->current_state = TRACTIVE_SYSTEM_DISABLED;
      return false;
    }
    break;

  case READY_TO_DRIVE: // We want to be able to leave no matter what
    this->error_code = this->bool_code;

    // Disable everything

    this->current_state = TRACTIVE_SYSTEM_DISABLED;
    return true;
    break;

  default:
    this->error_code = this->bool_code;
    this->current_state = TRACTIVE_SYSTEM_DISABLED;
    return false;
    break;
  }
}
