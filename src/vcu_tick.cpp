// Everything loop() used to do after reading the ADCs, moved here so it can be
// driven by a simulated clock and CAN bus in tests. main.cpp now just reads
// the sensors and calls vcu.tick(millis()).
#include "hal.hpp"
#include "parameters.hpp"
#include "vcu.hpp"

void VCU::begin() {
  init_state_machine();

  hal::pin_mode(BUZZER, hal::PIN_OUTPUT);

  inverter->set_power_limit_kw(POWER_LIMIT_KW);

  // Pump fellas
  hal::pin_mode(LOWSIDE1, hal::PIN_OUTPUT);
  hal::pin_mode(LOWSIDE2, hal::PIN_OUTPUT);
  hal::digital_write(LOWSIDE1, false);
  hal::digital_write(LOWSIDE2, false);
}

void VCU::tick(uint32_t now) {
  //
  //// CAN Stage
  update_acc_can();
  update_inv_can();

  if (timer_1s.check(now)) {
    send_firmware_status_message();
    send_status_message();
  }

  if (timer_20hz.check(now)) {
    send_pedal_travel_message();
    send_pedal_raw_message(pedals->get_apps1_raw(), pedals->get_apps2_raw(),
                           pedals->get_brake_raw());
    send_steering_raw_message(pedals->get_steering_raw());

    send_power_tracking_message();

    hal::log("\n\rraw_apps1: ");
    hal::log(pedals->get_apps1_raw());
    hal::log("\n\rraw_apps2: ");
    hal::log(pedals->get_apps2_raw());
    hal::log("\n\rraw_brake: ");
    hal::log(pedals->get_brake_raw());
    hal::log("\n\rraw_steering angle: ");
    hal::log(long(pedals->get_steering_raw()));

    send_brake_pressure_message(pedals->get_brake_raw());
  }

  //
  //// Math Stage
  accumulator->calculate_energy_consumed_wh(now);
  inverter->calculate_motor_distance_M(now);

  //
  //// State machine Stage
  switch (get_current_state()) {
  case STARTUP:
    if (set_state(TRACTIVE_SYSTEM_DISABLED)) {
      hal::logln("Tractive system disabled, waiting for TS voltage");
    } else {
      hal::log("Failed to boot, ERROR: ");
      hal::log(long(get_error_code()));
      hal::logln("");
    }
    break;

  case TRACTIVE_SYSTEM_DISABLED:
    if (timer_10hz.check(now))
      inverter->ping();

    if (ts_safe()) {
      if (set_state(TRACTIVE_SYSTEM_ENERGIZED)) {
        hal::logln("Entering TRACTIVE_SYSTEM_ENERGIZED");
        hal::logln("Car is waiting on driver...");
      } else {
        hal::log("Failed to enter TRACTIVE_SYSTEM_PRECHARGING, ERROR: ");
        hal::log(long(get_error_code()));
        hal::logln("");
      }
    };
    break;

  case TRACTIVE_SYSTEM_ENERGIZED:
    if (timer_10hz.check(now))
      inverter->ping();

    // try_ts_enabled is just looking for the brake and RTD button
    if (try_ts_enabled()) {
      if (set_state(TRACTIVE_SYSTEM_ENABLED)) {
        hal::logln("Entering TRACTIVE_SYSTEM_ENABLED");
        hal::logln("Car is preping to Rip");
      } else {
        hal::log("Failed to enter TRACTIVE_SYSTEM_ENABLED, ERROR: ");
        hal::log(long(get_error_code()));
        hal::logln("");
      }
    }

    // Catch for if we unlatch
    if (!ts_safe()) {
      hal::log("Something isn't safe, leaving ENERGIZED, ERROR: ");
      hal::log(long(get_error_code()));
      hal::logln("");
      set_state(TRACTIVE_SYSTEM_DISABLED);
    }
    break;

  case TRACTIVE_SYSTEM_ENABLED:
    if (timer_10hz.check(now))
      inverter->ping();

    inverter->set_current_limits(INVERTER_CHARGE_LIMIT,
                                 INVERTER_DISCHARGE_LIMIT);

    inverter->send_clear_faults();

    hal::digital_write(BUZZER, get_buzzer_state());
    hal::delay_ms(2151);

    if (set_state(READY_TO_DRIVE)) {
      hal::logln("Ready to Rip");

      hal::digital_write(BUZZER, get_buzzer_state());
    } else {
      hal::log("Failed to enter READY_TO_DRIVE, ERROR: ");
      hal::log(long(get_error_code()));
      hal::logln("");

      hal::digital_write(BUZZER, get_buzzer_state());
    }
    break;

  case READY_TO_DRIVE:
    if (ts_safe()) {
      if (timer_200hz.check(now)) {
        inverter->command_torque(pedals->get_torque_request(
            pedals->get_travel(), inverter->get_torque_limit()));
      }

      if (timer_10hz_2.check(now)) {
        inverter->set_current_limits(
            INVERTER_CHARGE_LIMIT, inverter->get_instant_current_limit(
                                       accumulator->get_pack_voltage()));
      }
    } else {
      hal::log("Something isn't safe, leaving RTD, ERROR: ");
      hal::log(long(get_error_code()));
      hal::logln("");
      set_state(TRACTIVE_SYSTEM_DISABLED);
    }
    break;

  case LAUNCH_WAIT:
    if (set_state(LAUNCH)) {
      // TODO: I think this should just be wating for some confirmation from the
      // driver or something idk go figure it out nerd

    } else {
      hal::log("Aborting launch, ERROR: ");
      hal::log(long(get_error_code()));
      hal::logln("");
      set_state(READY_TO_DRIVE);
    }
    break;

  case LAUNCH:
    if (get_launch_state()) {
      // TODO: Get the launch logic goin

    } else {
      hal::log("Exiting launch");
      set_state(READY_TO_DRIVE);
    }
    break;
  }
}
