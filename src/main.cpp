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

  //
  //// CAN Stage
  vcu.update_acc_can();
  vcu.update_inv_can();

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
      if (vcu.set_state(TRACTIVE_SYSTEM_ENERGIZED)) {
        consol.logln("Entering TRACTIVE_SYSTEM_PRECHARGING");
        consol.logln("TCU is trying to precharge...");
      } else {
        consol.log("Failed to enter TRACTIVE_SYSTEM_PRECHARGING, ERROR: ");
        consol.logln(vcu.get_error_code());
      }
    };
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
    buzzer_timer.reset();

    if (vcu.set_state(READY_TO_DRIVE) && buzzer_timer.check()) {
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

      // NOTE: I don't think this works right now...
      // if (timer_10hz.check()) {
      //   vcu.inverter->send_clear_faults();
      // }
    } else {
      consol.log("Something isn't safe, leaving RTD, ERROR: ");
      consol.logln(vcu.get_error_code());
      vcu.set_state(TRACTIVE_SYSTEM_DISABLED);
    }
    break;

  case LAUNCH_WAIT:
    if (vcu.set_state(LAUNCH)) {
      // TODO: I think this should just be wating for some confirmation from the
      // driver or something idk go figure it out nerd

    } else {
      consol.log("Aborting launch, ERROR: ");
      consol.logln(vcu.get_error_code());
      vcu.set_state(READY_TO_DRIVE);
    }
    break;

  case LAUNCH:
    if (vcu.get_launch_state()) {
      // TODO: Get the launch logic goin

    } else {
      consol.log("Exiting launch");
      vcu.set_state(READY_TO_DRIVE);
    }
    break;
  }
}
