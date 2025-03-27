#include "main.hpp"

void setup() {
  consol.logln("Booting...");

  vcu.init_state_machine();

  consol.logln("Booted");
}

void loop() {
  switch (vcu.get_current_state()) {
  case STARTUP:
    if (vcu.set_state(TRACTIVE_SYSTEM_DISABLED)) {
      consol.logln("Tractive system disabled, waiting for TS voltage");
    } else {
      consol.log("Failed to boot, ERROR: ");
      consol.logln(vcu.get_error_code());
    }
    break;

  case TRACTIVE_SYSTEM_DISABLED:
    if (vcu.try_ts_energized()) {
      if (vcu.set_state(TRACTIVE_SYSTEM_PRECHARGING)) {
        consol.logln("Entering TRACTIVE_SYSTEM_PRECHARGING");
        consol.logln("Trying to precharge...");
      } else {
        consol.log("Failed to enter TRACTIVE_SYSTEM_PRECHARGING, ERROR: ");
        consol.logln(vcu.get_error_code());
      }
    };
    break;

  case TRACTIVE_SYSTEM_PRECHARGING:
    if (vcu.set_state(TRACTIVE_SYSTEM_ENERGIZED)) {
      consol.logln("Tractive system energized, waiting on driver...");
    } else {
      consol.log("Failed to enter TRACTIVE_SYSTEM_ENERGIZED, ERROR: ");
      consol.logln(vcu.get_error_code());
    }
    break;

  case TRACTIVE_SYSTEM_ENERGIZED:
    if (vcu.try_ts_enabled()) {
      if (vcu.set_state(TRACTIVE_SYSTEM_ENABLED)) {
        consol.logln("Entering TRACTIVE_SYSTEM_ENABLED");
      } else {
        consol.log("Failed to enter TRACTIVE_SYSTEM_ENABLED, ERROR: ");
        consol.logln(vcu.get_error_code());
      }
    }
    break;

  case TRACTIVE_SYSTEM_ENABLED:
    if (vcu.set_state(READY_TO_DRIVE)) {
      consol.logln("Ready to Rip");
    } else {
      consol.log("Failed to enter READY_TO_DRIVE, ERROR: ");
      consol.logln(vcu.get_error_code());
    }
    break;

  case READY_TO_DRIVE:
    if (vcu.ts_safe()) {
      // inverter.send_torque(pedals.get_requested_torque());
    } else {
      consol.log("Something isn't safe, ERROR: ");
      consol.logln(vcu.get_error_code());
    }
    break;
  }
}
