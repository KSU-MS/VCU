#include "main.hpp"

void setup() {
  consol.logln("Booting...");

  vcu.init_state_machine();

  consol.logln("Booted");
}

void loop() {
  //
  //// ADC Stage
  apps1.update();
  apps2.update();
  bse.update();
  vcu.pedals->update_travel(apps1.value.in, apps2.value.in, bse.value.in);

  //
  //// CAN Stage
  if (vcu.acc_can->check_controller_message()) {
    can_message msg_in = vcu.acc_can->get_controller_message();

    switch (msg_in.id) {
    case CAN_ID_ACU_SHUTDOWN_STATUS:
      vcu.accumulator->update_acu_status(msg_in.buf.val, msg_in.length);
      vcu.daq_can->send_controller_message(msg_in);
      break;

    case CAN_ID_PRECHARGE_STATUS:
      vcu.accumulator->update_precharge_status(msg_in.buf.val, msg_in.length);
      vcu.daq_can->send_controller_message(msg_in);
      break;

      // We forward everything to the DAQ bus for loggin n telemetry, but I
      // should move this logic to the eveLogger so that it doesn't make
      // duplicate messages on it and frees up some power on this fella
    default:
      vcu.daq_can->send_controller_message(msg_in);
      break;
    }
  }

  if (vcu.inv_can->check_controller_message()) {
    can_message msg_in = vcu.inv_can->get_controller_message();

    switch (msg_in.id) {
    case CAN_ID_DASH_BUTTONS:
      vcu.update_dash_buttons(msg_in.buf.val, msg_in.length);
      vcu.daq_can->send_controller_message(msg_in);
      break;

    default:
      vcu.daq_can->send_controller_message(msg_in);
      break;
    }
  }

  //
  //// State machine Stage
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
