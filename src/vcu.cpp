#include "vcu.hpp"
#include "car.h"
#include "data_handler.hpp"
#include "parameters.hpp"
#include <array>
#include <logger.hpp>

extern Logger consol;

VCU::VCU(Pedals *pedals, Inverter *inverter, Accumulator *accumulator,
         std::array<parameter, 25> *params, VehicleData *vehicle_data) {
  this->pedals = pedals;
  this->inverter = inverter;
  this->accumulator = accumulator;
  this->params = params;
  this->vehicle_data = vehicle_data;
}

bool VCU::try_ts_enabled() {
  bool rtd_pressed =
      vehicle_data ? vehicle_data->driver.rtd_button_pressed : false;
  if (rtd_pressed && (pedals->get_brake_travel() > MINIMUM_BRAKE_FOR_RTD)) {
    return true;
  } else {
    return false;
  }
}

bool VCU::ts_safe() {
  if (accumulator->get_precharge_state() == PRECHARGE_OK_STATE &&
      accumulator->get_bms_ok_hs() && accumulator->get_imd_ok_hs() &&
      inverter->get_bus_voltage() > TRACTIVE_SYSTEM_MINIMUM_VOLTAGE) {
    return true;
  } else {
    set_state(TRACTIVE_SYSTEM_DISABLED);
    return false;
  }
}

bool VCU::set_state(state target_state) {
  switch (current_state) {

  // This is just a catch for evil starts
  case STARTUP:
    if (target_state == TRACTIVE_SYSTEM_DISABLED) {
      current_state = TRACTIVE_SYSTEM_DISABLED;

      digitalWrite(LOWSIDE1, LOW);
      digitalWrite(LOWSIDE2, LOW);

      return true;
    } else {
      error_code = bool_code;
      current_state = STARTUP;
      return false;
    }
    break;

  case TRACTIVE_SYSTEM_DISABLED:
    if (target_state == TRACTIVE_SYSTEM_ENERGIZED && ts_safe()) {
      current_state = TRACTIVE_SYSTEM_ENERGIZED;

      digitalWrite(LOWSIDE1, HIGH);
      digitalWrite(LOWSIDE2, HIGH);

      return true;
    } else {
      error_code = bool_code;
      current_state = TRACTIVE_SYSTEM_DISABLED;

      digitalWrite(LOWSIDE1, LOW);
      digitalWrite(LOWSIDE2, LOW);

      return false;
    }
    break;

  case TRACTIVE_SYSTEM_ENERGIZED:
    if (target_state == TRACTIVE_SYSTEM_ENABLED && ts_safe()) {
      current_state = TRACTIVE_SYSTEM_ENABLED;

      buzzer_active = true;

      digitalWrite(LOWSIDE1, HIGH);
      digitalWrite(LOWSIDE2, HIGH);

      // Get the inverter prepped
      inverter->set_inverter_enable(true);

      return true;
    } else {
      error_code = bool_code;
      current_state = TRACTIVE_SYSTEM_DISABLED;

      digitalWrite(LOWSIDE1, LOW);
      digitalWrite(LOWSIDE2, LOW);

      return false;
    }
    break;

  case TRACTIVE_SYSTEM_ENABLED:
    if (target_state == READY_TO_DRIVE && ts_safe()) {
      current_state = READY_TO_DRIVE;

      buzzer_active = false;

      digitalWrite(LOWSIDE1, HIGH);
      digitalWrite(LOWSIDE2, HIGH);

      // TODO: Make this torque limit easier to configure
      inverter->set_inverter_enable(true);

      // not implemented yet
      // inverter->set_torque_limit(MAX_TORQUE_LIMIT_NM);
      // inverter->set_speed_limit(SOFT_MOTOR_RPM_LIMIT);

      return true;
    } else {
      buzzer_active = false;

      error_code = bool_code;
      current_state = TRACTIVE_SYSTEM_DISABLED;

      digitalWrite(LOWSIDE1, LOW);
      digitalWrite(LOWSIDE2, LOW);

      return false;
    }
    break;

  case READY_TO_DRIVE: // We want to be able to leave no matter what
    inverter->set_inverter_enable(false);

    buzzer_active = false;

    digitalWrite(LOWSIDE1, LOW);
    digitalWrite(LOWSIDE2, LOW);

    this->current_state = TRACTIVE_SYSTEM_DISABLED;
    return true;
    break;

  case LAUNCH_WAIT:
    if (target_state == READY_TO_DRIVE && ts_safe()) {

      // TODO: Figure out what needs to get turned off
      this->current_state = READY_TO_DRIVE;
    } else if (target_state == LAUNCH && ts_safe()) {

      // TODO: Get some pre-lim logic goin
      this->current_state = LAUNCH;
    } else {
      inverter->set_inverter_enable(false);

      buzzer_active = false;

      this->current_state = TRACTIVE_SYSTEM_DISABLED;
      return false;
    }
    break;

  case LAUNCH:
    if (target_state == READY_TO_DRIVE && ts_safe() && !launch_state) {
    } else {
      this->current_state = TRACTIVE_SYSTEM_DISABLED;
    }
    break;

  default:
    inverter->set_inverter_enable(false);

    buzzer_active = false;

    this->current_state = TRACTIVE_SYSTEM_DISABLED;
    return false;
    break;
  }
}

void VCU::set_parameter(uint8_t target_parameter, uint32_t parameter_value) {
  // set the parameter in the list
  this->params->at(target_parameter).parameter_value =
      double(parameter_value) /
      double(this->params->at(target_parameter).scale);
}

void VCU::handle_state_machine() {
  switch (current_state) {
  case STARTUP:
    if (set_state(TRACTIVE_SYSTEM_DISABLED)) {
      consol.logln("Tractive system disabled, waiting for TS voltage");
    } else {
      consol.log("Failed to boot, ERROR: ");
      consol.logln(get_error_code());
    }
    break;

  case TRACTIVE_SYSTEM_DISABLED:
    if (timer_inverter_ping != nullptr && timer_inverter_ping()) {
      inverter->ping();
    }

    if (ts_safe()) {
      if (set_state(TRACTIVE_SYSTEM_ENERGIZED)) {
        consol.logln("Entering TRACTIVE_SYSTEM_ENERGIZED");
        consol.logln("Car is waiting on driver...");
      } else {
        consol.log("Failed to enter TRACTIVE_SYSTEM_PRECHARGING, ERROR: ");
        consol.logln(get_error_code());
      }
    };
    break;

  case TRACTIVE_SYSTEM_ENERGIZED:
    if (timer_inverter_ping != nullptr && timer_inverter_ping()) {
      inverter->ping();
    }

    if (try_ts_enabled())
      if (timer_inverter_ping != nullptr && timer_inverter_ping()) {
        inverter->ping();
      }
    {
      if (set_state(TRACTIVE_SYSTEM_ENABLED)) {
        consol.logln("Entering TRACTIVE_SYSTEM_ENABLED");
        consol.logln("Car is preping to Rip");
      } else {
        consol.log("Failed to enter TRACTIVE_SYSTEM_ENABLED, ERROR: ");
        consol.logln(get_error_code());
      }
    }

    if (!ts_safe()) {
      consol.log("Something isn't safe, leaving ENERGIZED, ERROR: ");
      consol.logln(get_error_code());
      set_state(TRACTIVE_SYSTEM_DISABLED);
    }
    break;

  case TRACTIVE_SYSTEM_ENABLED:
    if (timer_inverter_ping != nullptr && timer_inverter_ping()) {
      inverter->ping();
    }

    inverter->set_current_limits(
        static_cast<uint16_t>((*params)[CURRENT_CHARGE_LIMIT].parameter_value),
        static_cast<uint16_t>(
            (*params)[CURRENT_DISCHARGE_LIMIT].parameter_value));

    digitalWrite(BUZZER, get_buzzer_state());
    delay(2151);

    if (set_state(READY_TO_DRIVE)) {
      consol.logln("Ready to Rip");

      digitalWrite(BUZZER, get_buzzer_state());
    } else {
      consol.log("Failed to enter READY_TO_DRIVE, ERROR: ");
      consol.logln(get_error_code());

      digitalWrite(BUZZER, get_buzzer_state());
    }
    break;

  case READY_TO_DRIVE:
    if (ts_safe()) {
      if (timer_inverter_command != nullptr && timer_inverter_command()) {
        inverter->command_torque(
            pedals->get_torque_request(pedals->get_throttle_travel(),
                                       (*params)[MAX_TORQUE].parameter_value));
      }

      if (timer_current_limit != nullptr && timer_current_limit()) {
        inverter->set_current_limits(
            static_cast<uint16_t>(
                (*params)[CURRENT_CHARGE_LIMIT].parameter_value),
            inverter->get_instant_current_limit(
                accumulator->get_pack_voltage()));

        if (reset_timer_current_limit) {
          reset_timer_current_limit();
        }
      }
    } else {
      consol.log("Something isn't safe, leaving RTD, ERROR: ");
      consol.logln(get_error_code());
      set_state(TRACTIVE_SYSTEM_DISABLED);
    }
    break;

  case LAUNCH_WAIT:
    if (set_state(LAUNCH)) {
    } else {
      consol.log("Aborting launch, ERROR: ");
      consol.logln(get_error_code());
      set_state(READY_TO_DRIVE);
    }
    break;

  case LAUNCH:
    if (get_launch_state()) {
    } else {
      consol.log("Exiting launch");
      set_state(READY_TO_DRIVE);
    }
    break;
  }
}

void VCU::send_status_message() {
  data_handler::send_vcu_status_message(
      bspd_brake_high, bspd_current_high, bspd_ok_hs,
      accumulator->get_bms_ok_hs(), accumulator->get_imd_ok_hs(), buzzer_active,
      inverter->get_inverter_enable(),
      this->params->at(MAX_TORQUE).parameter_value, torque_mode,
      static_cast<int>(current_state));
}

void VCU::send_firmware_status_message() {
  // TODO: Abstract this arduino call
  data_handler::send_vcu_firmware_status_message(millis() / 1000, AUTO_VERSION,
                                                 FW_PROJECT_IS_DIRTY,
                                                 FW_PROJECT_IS_MAIN_OR_MASTER);
}

void VCU::send_power_tracking_message() {
  data_handler::send_vcu_power_tracking_message(
      uint32_t(inverter->get_motor_distance_M()),
      accumulator->get_consumed_wh());
}

// void VCU::send_launch_control_status_message() {
//   encode_can_0x0cb_vcu_launchcontrol_elapsed_time(dbc, 0);
//   encode_can_0x0cb_vcu_launchcontrol_outputtorqueco(dbc, 0);
//   encode_can_0x0cb_vcu_launchcontrol_state(dbc, launch_state);
//   encode_can_0x0cb_vcu_launchcontrol_type(dbc, launch_mode);
//
//   can_message out_msg;
//   out_msg.id = CAN_ID_VCU_LAUNCHCONTROL_DIAGDATA;
//   out_msg.length =
//       pack_message(dbc, CAN_ID_VCU_LAUNCHCONTROL_DIAGDATA, &out_msg.buf.val);
//
//   data_handler::send_inv_and_daq(out_msg);
// }
