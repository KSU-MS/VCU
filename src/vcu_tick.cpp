// Everything loop() used to do after reading the ADCs, moved here so it can be
// driven by a simulated clock and CAN bus in tests. main.cpp now just reads
// the sensors and calls vcu.tick(millis()).
#include "hal.hpp"
#include "parameters.hpp"
#include "vcu.hpp"
#include <ota.hpp>

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
  now_ms = now;

  //
  //// CAN Stage
  update_acc_can();
  update_inv_can();
  update_daq_can(now);
  if (updater)
    updater->poll(now);

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
  // Track when we got into the current state, for states that wait on time
  bool entered_state = current_state != state_last_tick;
  if (entered_state) {
    state_entered_ms = now;
    state_last_tick = current_state;
  }

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

    // Don't let the car go live while someone is reflashing it
    if (updater && updater->programming())
      break;

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
      drop_out();
    }
    break;

  case TRACTIVE_SYSTEM_ENABLED:
    if (timer_10hz.check(now))
      inverter->ping();

    // Prep the inverter and start the buzzer once, on the way in
    if (entered_state) {
      inverter->set_current_limits(INVERTER_CHARGE_LIMIT,
                                   INVERTER_DISCHARGE_LIMIT);

      inverter->send_clear_faults();

      hal::digital_write(BUZZER, get_buzzer_state());
    }

    // The buzzer used to be a blocking delay(), which stopped CAN being read
    // and the inverter being pinged, so a fault during it went unnoticed
    // until after we'd already gone RTD. Keep checking while it plays.
    if (!ts_safe()) {
      hal::log("Something isn't safe, leaving ENABLED, ERROR: ");
      hal::log(long(get_error_code()));
      hal::logln("");
      drop_out();
      break;
    }

    if (now - state_entered_ms >= RTD_BUZZER_DURATION_MS) {
      if (set_state(READY_TO_DRIVE)) {
        hal::logln("Ready to Rip");
      } else {
        hal::log("Failed to enter READY_TO_DRIVE, ERROR: ");
        hal::log(long(get_error_code()));
        hal::logln("");
      }

      hal::digital_write(BUZZER, get_buzzer_state());
    }
    break;

  case READY_TO_DRIVE:
    if (ts_safe()) {
      inverter->regulate_power(now);
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
      drop_out();
    }
    break;

  case TRACTIVE_SYSTEM_FAULT:
    if (timer_10hz.check(now))
      inverter->ping();

    // Over once the fault has cleared, or the voltage is gone; ready to
    // drive then needs the driver all over again
    if (fault_code() == 0 || !hv_maybe_present()) {
      hal::logln("Fault over, tractive system disabled");
      current_state = TRACTIVE_SYSTEM_DISABLED;
      hal::digital_write(LOWSIDE1, false);
      hal::digital_write(LOWSIDE2, false);
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
