#include "state_machine.hpp"
#include "data_handler.hpp"
#include "parameters.hpp"
#include <car.h>
#include <logger.hpp>

extern Logger consol;

StateMachine::StateMachine(Inverter *inverter,
                           std::array<Parameter, 25> *params,
                           VehicleData *vehicle_data)
    : vehicle_data(vehicle_data), inverter(inverter), params(params) {}

bool StateMachine::try_ts_enabled() {
  bool rtd_pressed = vehicle_data->driver.rtd_button_pressed;
  return rtd_pressed &&
         (vehicle_data->pedals.brake_travel > MINIMUM_BRAKE_FOR_RTD);
}

bool StateMachine::ts_safe() {
  if (vehicle_data->accumulator.precharge_state == PRECHARGE_OK_STATE &&
      vehicle_data->accumulator.bms_ok_hs &&
      vehicle_data->accumulator.imd_ok_hs &&
      vehicle_data->inverter.bus_voltage > TRACTIVE_SYSTEM_MINIMUM_VOLTAGE) {
    return true;
  } else {
    set_state(StateMachineData::state::TRACTIVE_SYSTEM_DISABLED);
    return false;
  }
}

bool StateMachine::set_state(StateMachineData::state target_state) {
  switch (vehicle_data->state_machine.current_state) {

  // This is just a catch for evil starts
  case StateMachineData::state::STARTUP:
    if (target_state == StateMachineData::state::TRACTIVE_SYSTEM_DISABLED) {
      vehicle_data->state_machine.current_state =
          StateMachineData::state::TRACTIVE_SYSTEM_DISABLED;

      digitalWrite(LOWSIDE1, LOW);
      digitalWrite(LOWSIDE2, LOW);

      return true;
    } else {
      vehicle_data->state_machine.error_code =
          vehicle_data->state_machine.bool_code;
      vehicle_data->state_machine.current_state =
          StateMachineData::state::STARTUP;
      return false;
    }
    break;

  case StateMachineData::state::TRACTIVE_SYSTEM_DISABLED:
    if (target_state == StateMachineData::state::TRACTIVE_SYSTEM_ENERGIZED &&
        ts_safe()) {
      vehicle_data->state_machine.current_state =
          StateMachineData::state::TRACTIVE_SYSTEM_ENERGIZED;

      digitalWrite(LOWSIDE1, HIGH);
      digitalWrite(LOWSIDE2, HIGH);

      return true;
    } else {
      vehicle_data->state_machine.error_code =
          vehicle_data->state_machine.bool_code;
      vehicle_data->state_machine.current_state =
          StateMachineData::state::TRACTIVE_SYSTEM_DISABLED;

      digitalWrite(LOWSIDE1, LOW);
      digitalWrite(LOWSIDE2, LOW);

      return false;
    }
    break;

  case StateMachineData::state::TRACTIVE_SYSTEM_ENERGIZED:
    if (target_state == StateMachineData::state::TRACTIVE_SYSTEM_ENABLED &&
        ts_safe()) {
      vehicle_data->state_machine.current_state =
          StateMachineData::state::TRACTIVE_SYSTEM_ENABLED;

      vehicle_data->state_machine.buzzer_active = true;

      digitalWrite(LOWSIDE1, HIGH);
      digitalWrite(LOWSIDE2, HIGH);

      // Get the inverter prepped
      inverter->set_inverter_enable(true);

      return true;
    } else {
      vehicle_data->state_machine.error_code =
          vehicle_data->state_machine.bool_code;
      vehicle_data->state_machine.current_state =
          StateMachineData::state::TRACTIVE_SYSTEM_DISABLED;

      digitalWrite(LOWSIDE1, LOW);
      digitalWrite(LOWSIDE2, LOW);

      return false;
    }
    break;

  case StateMachineData::state::TRACTIVE_SYSTEM_ENABLED:
    if ((target_state == StateMachineData::state::READY_TO_DRIVE_TORQUE ||
         target_state == StateMachineData::state::READY_TO_DRIVE_SPEED) &&
        ts_safe()) {
      vehicle_data->state_machine.current_state = target_state;
      if (target_state == StateMachineData::state::READY_TO_DRIVE_TORQUE) {
        inverter->set_command_mode_to_torque();
      } else if (target_state ==
                 StateMachineData::state::READY_TO_DRIVE_SPEED) {
        inverter->set_command_mode_to_speed();
      }

      vehicle_data->state_machine.buzzer_active = false;

      digitalWrite(LOWSIDE1, HIGH);
      digitalWrite(LOWSIDE2, HIGH);

      // TODO: Make this torque limit easier to configure
      inverter->set_inverter_enable(true);

      // not implemented yet
      // inverter->set_torque_limit(MAX_TORQUE_LIMIT_NM);
      // inverter->set_speed_limit(SOFT_MOTOR_RPM_LIMIT);

      return true;
    } else {
      vehicle_data->state_machine.buzzer_active = false;

      vehicle_data->state_machine.error_code =
          vehicle_data->state_machine.bool_code;
      vehicle_data->state_machine.current_state =
          StateMachineData::state::TRACTIVE_SYSTEM_DISABLED;

      digitalWrite(LOWSIDE1, LOW);
      digitalWrite(LOWSIDE2, LOW);

      return false;
    }
    break;

  case StateMachineData::state::READY_TO_DRIVE_TORQUE: // We want to be able to
                                                       // leave no matter what
    inverter->set_inverter_enable(false);

    vehicle_data->state_machine.buzzer_active = false;

    digitalWrite(LOWSIDE1, LOW);
    digitalWrite(LOWSIDE2, LOW);

    vehicle_data->state_machine.current_state =
        StateMachineData::state::TRACTIVE_SYSTEM_DISABLED;
    return true;
    break;
  case StateMachineData::state::READY_TO_DRIVE_SPEED: // We want to be able to
                                                      // leave no matter what
    inverter->set_inverter_enable(false);

    vehicle_data->state_machine.buzzer_active = false;

    digitalWrite(LOWSIDE1, LOW);
    digitalWrite(LOWSIDE2, LOW);

    vehicle_data->state_machine.current_state =
        StateMachineData::state::TRACTIVE_SYSTEM_DISABLED;
    return true;
    break;

  case StateMachineData::state::LAUNCH_WAIT:
    if ((target_state == StateMachineData::state::READY_TO_DRIVE_TORQUE ||
         target_state == StateMachineData::state::READY_TO_DRIVE_SPEED) &&
        ts_safe()) {

      // TODO: Figure out what needs to get turned off
      vehicle_data->state_machine.current_state = target_state;
    } else if (target_state == StateMachineData::state::LAUNCH && ts_safe()) {

      // TODO: Get some pre-lim logic goin
      vehicle_data->state_machine.current_state = target_state;
    } else {
      inverter->set_inverter_enable(false);

      vehicle_data->state_machine.buzzer_active = false;

      vehicle_data->state_machine.current_state =
          StateMachineData::state::TRACTIVE_SYSTEM_DISABLED;
      return false;
    }
    break;

  case StateMachineData::state::LAUNCH:
    if ((target_state == StateMachineData::state::READY_TO_DRIVE_TORQUE ||
         target_state == StateMachineData::state::READY_TO_DRIVE_SPEED) &&
        ts_safe() && !vehicle_data->state_machine.bool_code) {
    } else {
      vehicle_data->state_machine.current_state =
          StateMachineData::state::TRACTIVE_SYSTEM_DISABLED;
    }
    break;

  default:
    inverter->set_inverter_enable(false);

    vehicle_data->state_machine.buzzer_active = false;

    vehicle_data->state_machine.current_state =
        StateMachineData::state::TRACTIVE_SYSTEM_DISABLED;
    return false;
    break;
  }
}

void StateMachine::state_machine_main_loop() {
  switch (vehicle_data->state_machine.current_state) {
  case StateMachineData::state::STARTUP:
    if (set_state(StateMachineData::state::TRACTIVE_SYSTEM_DISABLED)) {
      consol.logln("Tractive system disabled, waiting for TS voltage");
    } else {
      consol.log("Failed to boot, ERROR: ");
      consol.logln(vehicle_data->state_machine.error_code);
    }
    break;

  case StateMachineData::state::TRACTIVE_SYSTEM_DISABLED:
    if (ts_safe()) {
      if (set_state(StateMachineData::state::TRACTIVE_SYSTEM_ENERGIZED)) {
        consol.logln("Entering TRACTIVE_SYSTEM_ENERGIZED");
        consol.logln("Car is waiting on driver...");
      } else {
        consol.log("Failed to enter TRACTIVE_SYSTEM_PRECHARGING, ERROR: ");
        consol.logln(vehicle_data->state_machine.error_code);
      }
    };
    break;

  case StateMachineData::state::TRACTIVE_SYSTEM_ENERGIZED:
    if (try_ts_enabled()) {
      if (set_state(StateMachineData::state::TRACTIVE_SYSTEM_ENABLED)) {
        consol.logln("Entering TRACTIVE_SYSTEM_ENABLED");
        consol.logln("Car is preping to Rip");
      } else {
        consol.log("Failed to enter TRACTIVE_SYSTEM_ENABLED, ERROR: ");
        consol.logln(vehicle_data->state_machine.error_code);
      }
    }

    if (!ts_safe()) {
      consol.log("Something isn't safe, leaving ENERGIZED, ERROR: ");
      consol.logln(vehicle_data->state_machine.error_code);
      set_state(StateMachineData::state::TRACTIVE_SYSTEM_DISABLED);
    }
    break;

  case StateMachineData::state::TRACTIVE_SYSTEM_ENABLED:
    inverter->set_current_limits(
        this->params->at(CURRENT_CHARGE_LIMIT_ID).value,
        this->params->at(CURRENT_DISCHARGE_LIMIT_ID).value);

    digitalWrite(BUZZER, vehicle_data->state_machine.buzzer_active);
    delay(2151);
    if (this->params->at(INVERTER_CONTROL_MODE_TORQUE_ID).value) {
      if (set_state(StateMachineData::state::READY_TO_DRIVE_TORQUE)) {
        consol.logln("Ready to Rip");

        digitalWrite(BUZZER, vehicle_data->state_machine.buzzer_active);
      } else {
        consol.log("Failed to enter READY_TO_DRIVE, ERROR: ");
        consol.logln(vehicle_data->state_machine.error_code);

        digitalWrite(BUZZER, vehicle_data->state_machine.buzzer_active);
      }
    } else {
      if (set_state(StateMachineData::state::READY_TO_DRIVE_SPEED)) {
        consol.logln("Ready to Rip");

        digitalWrite(BUZZER, vehicle_data->state_machine.buzzer_active);
      } else {
        consol.log("Failed to enter READY_TO_DRIVE, ERROR: ");
        consol.logln(vehicle_data->state_machine.error_code);

        digitalWrite(BUZZER, vehicle_data->state_machine.buzzer_active);
      }
    }
    break;

  case StateMachineData::state::READY_TO_DRIVE_TORQUE:
    if (ts_safe()) {
      inverter->command_torque(vehicle_data->pedals.throttle_travel *
                               this->params->at(MAX_TORQUE_ID).value);
    } else {
      consol.log("Something isn't safe, leaving RTD, ERROR: ");
      consol.logln(vehicle_data->state_machine.error_code);
      set_state(StateMachineData::state::TRACTIVE_SYSTEM_DISABLED);
    }
    break;

  case StateMachineData::state::READY_TO_DRIVE_SPEED:
    if (ts_safe()) {
      inverter->command_speed(vehicle_data->pedals.throttle_travel *
                              this->params->at(SOFT_RPM_LIMIT_ID).value);
    } else {
      consol.log("Something isn't safe, leaving RTD, ERROR: ");
      consol.logln(vehicle_data->state_machine.error_code);
      set_state(StateMachineData::state::TRACTIVE_SYSTEM_DISABLED);
    case StateMachineData::state::LAUNCH_WAIT:
      if (set_state(StateMachineData::state::LAUNCH)) {
      } else {
        consol.log("Aborting launch, ERROR: ");
        consol.logln(vehicle_data->state_machine.error_code);
        set_state(StateMachineData::state::READY_TO_DRIVE_TORQUE);
      }
      break;

    case StateMachineData::state::LAUNCH:
      if (vehicle_data->state_machine.bool_code) {
      } else {
        consol.log("Exiting launch");
        set_state(StateMachineData::state::READY_TO_DRIVE_TORQUE);
      }
      break;
    }
  }
}

void StateMachine::send_status_message() {
  DataHandler::send_vcu_status_message(
      vehicle_data->state_machine.bspd_brake_high,
      vehicle_data->state_machine.bspd_current_high,
      vehicle_data->state_machine.bspd_ok_hs,
      vehicle_data->accumulator.bms_ok_hs, vehicle_data->accumulator.imd_ok_hs,
      vehicle_data->state_machine.buzzer_active,
      inverter->get_inverter_enable(), this->params->at(MAX_TORQUE_ID).value,
      vehicle_data->state_machine.bool_code,
      static_cast<int>(vehicle_data->state_machine.current_state));
}

void StateMachine::send_firmware_status_message() {
  // TODO: Abstract this arduino call
  DataHandler::send_vcu_firmware_status_message(millis() / 1000, AUTO_VERSION,
                                                FW_PROJECT_IS_DIRTY,
                                                FW_PROJECT_IS_MAIN_OR_MASTER);
}

void StateMachine::send_power_tracking_message() {
  DataHandler::send_vcu_power_tracking_message(
      uint32_t(vehicle_data->inverter.motor_distance_m),
      vehicle_data->accumulator.consumed_power_wh);
}

void StateMachine::state_machine_200hz_loop() {}

void StateMachine::state_machine_10hz_loop() {
  this->send_power_tracking_message();
}

void StateMachine::state_machine_1hz_loop() {
  this->send_firmware_status_message();
  this->send_status_message();
}