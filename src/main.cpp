#include "main.hpp"

void setup() {
  consol.logln("Booting...");

  vcu.init_state_machine();
}

void loop() {
  switch (vcu.get_current_state()) {
  case STARTUP:
    consol.logln("Booted");
    vcu.set_state(TRACTIVE_SYSTEM_DISABLED);
    break;

  case TRACTIVE_SYSTEM_DISABLED:
    consol.logln("Tractive system off");
    break;

  case TRACTIVE_SYSTEM_PRECHARGING:
    consol.logln("Trying to precharge...");
    break;

  case TRACTIVE_SYSTEM_ENERGIZED:
    consol.logln("Tractive system energized");
    break;

  case TRACTIVE_SYSTEM_ENABLED:
    consol.logln("Trying to enter READY_TO_DRIVE...");
    break;

  case READY_TO_DRIVE:

    break;
  }
}
