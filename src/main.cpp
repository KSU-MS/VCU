#include "main.hpp"

void setup() {
  consol.logln("Booting...");

  vcu.init_state_machine();

  // TODO: Get rid of these evil arduino calls for the buzzer
  pinMode(BUZZER, OUTPUT);

  consol.logln("Booted");
}

uint8_t last_state = 9;

void loop() {
  //
  //// ADC Stage
  apps1.update();
  apps2.update();
  bse.update();
  vcu.pedals->update_travel(apps1.value.in, apps2.value.in, bse.value.in);

  vsense_bspd.update();
  vcu.update_bspd(vsense_bspd.value.in, 0, 0);

  // consol.log("apps1: ");
  // consol.log(vcu.pedals->get_apps1_travel());
  // consol.log("\t");
  // consol.logln(apps1.value.in);
  // consol.log("apps2: ");
  // consol.log(vcu.pedals->get_apps2_travel());
  // consol.log("\t");
  // consol.logln(apps2.value.in);
  // consol.log("apps: ");
  // consol.logln(vcu.pedals->get_travel());
  // consol.log("brake: ");
  // consol.logln(vcu.pedals->get_brake_travel());
  // consol.log("\t");
  // consol.logln(bse.value.in);

  //
  //// CAN Stage
  // vcu.send_firmware_status_message();
  // vcu.send_status_message();
  // vcu.send_pedal_message();

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

    // We foward this to the inverter bus for the dash
    case CAN_ID_MSGID_0X6B3:
      vcu.inv_can->send_controller_message(msg_in);
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
    vcu.inverter->ping(); // Get the inverter prepped

    consol.logln(vcu.pedals->get_brake_travel());
    consol.logln(vcu.get_rtd_fella());

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
    vcu.inverter->ping(); // Keep the inverter prepped

    digitalWrite(BUZZER, vcu.get_buzzer_state());
    delay(1000); // BUG: Get rid of this aids arduino call

    if (vcu.set_state(READY_TO_DRIVE)) {
      consol.logln("Ready to Rip");
    } else {
      consol.log("Failed to enter READY_TO_DRIVE, ERROR: ");
      consol.logln(vcu.get_error_code());
    }
    break;

  case READY_TO_DRIVE:
    if (vcu.ts_safe()) {
      vcu.inverter->command_torque(vcu.pedals->get_torque_request(
          vcu.pedals->get_travel(), vcu.inverter->get_torque_limit()));
    } else {
      consol.log("Something isn't safe, ERROR: ");
      consol.logln(vcu.get_error_code());
    }
    break;
  }
}
