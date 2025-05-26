#include "main.hpp"
#include "car.h"

void setup() {
  consol.logln("Booting...");

  vcu.init_state_machine();

  // TODO: Get rid of these evil arduino calls for the buzzer
  pinMode(BUZZER, OUTPUT);

  consol.logln("Booted");

  vcu.accumulator->set_charge_limit(15);
  vcu.accumulator->set_discharge_limit(200);
}

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
  // consol.log(vcu.pedals->get_brake_travel());
  // consol.log("\t");
  // consol.logln(bse.value.in);

  //
  //// CAN Stage
  if (timer_1s.check()) {
    vcu.send_firmware_status_message();
    vcu.send_status_message();
    vcu.accumulator->send_bms_current_limit();
  }

  if (timer_20hz.check()) {
    vcu.send_pedal_travel_message();
    vcu.send_pedal_raw_message(vcu.pedals->get_apps1_raw(),
                               vcu.pedals->get_apps2_raw(),
                               vcu.pedals->get_brake_raw());

    print_message(&kms_can, CAN_ID_VCU_PEDALS_TRAVEL, &std_out_wrap);
    print_message(&kms_can, CAN_ID_VCU_PEDAL_READINGS, &std_out_wrap);
  }

  if (vcu.acc_can->check_controller_message()) {
    can_message msg_in = vcu.acc_can->get_controller_message();
    vcu.inv_can->send_controller_message(msg_in);
    // vcu.daq_can->send_controller_message(msg_in);

    switch (msg_in.id) {
    case CAN_ID_ACU_SHUTDOWN_STATUS:
      vcu.accumulator->update_acu_status(msg_in.buf.val, msg_in.length);
      break;

    case CAN_ID_PRECHARGE_STATUS:
      vcu.accumulator->update_precharge_status(msg_in.buf.val, msg_in.length);
      break;

      // NOTE: Commented out for now as we are already fowarding everything from
      // the accumulator bus to the inverter bus

      // We foward this to the inverter bus for the dash

      // case CAN_ID_MSGID_0X6B3:
      //   vcu.inv_can->send_controller_message(msg_in);
      //   break;
    }
  }

  if (vcu.inv_can->check_controller_message()) {
    can_message msg_in = vcu.inv_can->get_controller_message();
    // vcu.daq_can->send_controller_message(msg_in);

    switch (msg_in.id) {
    case CAN_ID_DASH_BUTTONS:
      vcu.update_dash_buttons(msg_in.buf.val, msg_in.length);
      break;

    case CAN_ID_M166_CURRENT_INFO:
      vcu.inverter->update_bus_current(msg_in.buf.val, msg_in.length);
      break;

    case CAN_ID_M167_VOLTAGE_INFO:
      vcu.inverter->update_bus_voltage(msg_in.buf.val, msg_in.length);
      break;

    case CAN_ID_VCU_SET_PARAMETER:
      vcu.set_parameter(msg_in.buf.val, msg_in.length);
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
    if (timer_20hz.check())
      vcu.inverter->ping();

    if (vcu.ts_safe()) {
      if (vcu.set_state(TRACTIVE_SYSTEM_PRECHARGING)) {
        consol.logln("Entering TRACTIVE_SYSTEM_PRECHARGING");
        consol.logln("TCU is trying to precharge...");
      } else {
        consol.log("Failed to enter TRACTIVE_SYSTEM_PRECHARGING, ERROR: ");
        consol.logln(vcu.get_error_code());
      }
    };
    break;

  case TRACTIVE_SYSTEM_PRECHARGING:
    if (timer_20hz.check())
      vcu.inverter->ping();

    if (vcu.set_state(TRACTIVE_SYSTEM_ENERGIZED)) {
      consol.logln("Tractive system energized, waiting on driver...");
    } else {
      consol.log("Failed to enter TRACTIVE_SYSTEM_ENERGIZED, ERROR: ");
      consol.logln(vcu.get_error_code());
    }
    break;

  case TRACTIVE_SYSTEM_ENERGIZED:
    if (timer_20hz.check())
      vcu.inverter->ping();

    // try_ts_enabled is just looking for the brake and RTD button
    if (vcu.try_ts_enabled()) {
      if (vcu.set_state(TRACTIVE_SYSTEM_ENABLED)) {
        consol.logln("Entering TRACTIVE_SYSTEM_ENABLED");
      } else {
        consol.log("Failed to enter TRACTIVE_SYSTEM_ENABLED, ERROR: ");
        consol.logln(vcu.get_error_code());
      }
    }

    // Catch for if we unlatch
    if (!vcu.ts_safe()) {
      consol.log("Something isn't safe, leaving ENERGIZED, ERROR: ");
      consol.logln(vcu.get_error_code());
      vcu.set_state(TRACTIVE_SYSTEM_DISABLED);
    }
    break;

  case TRACTIVE_SYSTEM_ENABLED:
    if (timer_20hz.check())
      vcu.inverter->ping();

    digitalWrite(BUZZER, vcu.get_buzzer_state());

    delay(2215); // BUG: Get rid of this aids arduino call

    if (vcu.set_state(READY_TO_DRIVE)) {
      consol.logln("Ready to Rip");

      digitalWrite(BUZZER, vcu.get_buzzer_state());
    } else {
      consol.log("Failed to enter READY_TO_DRIVE, ERROR: ");
      consol.logln(vcu.get_error_code());

      digitalWrite(BUZZER, vcu.get_buzzer_state());
    }
    break;

  case READY_TO_DRIVE:
    if (vcu.ts_safe()) {
      if (timer_200hz.check()) {
        vcu.inverter->command_torque(vcu.pedals->get_torque_request(
            vcu.pedals->get_travel(), vcu.inverter->get_torque_limit()));
      }

      // if (timer_10hz.check()) {
      //   vcu.inverter->send_clear_faults();
      // }
    } else {
      consol.log("Something isn't safe, leaving RTD, ERROR: ");
      consol.logln(vcu.get_error_code());
      vcu.set_state(TRACTIVE_SYSTEM_DISABLED);
    }
    break;
  }
}
