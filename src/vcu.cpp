#include "vcu.hpp"
#include "car.h"

VCU::VCU(PEDALS *pedals, CM200 *inverter, ACCUMULATOR *accumulator,
         can_obj_car_h_t *dbc, canMan *acc_can, canMan *inv_can,
         canMan *daq_can, bool (*timer_status_message)(),
         bool (*timer_pedal_message)(), bool (*timer_RTD_buzzer)()) {
  this->pedals = pedals;
  this->inverter = inverter;
  this->accumulator = accumulator;

  this->dbc = dbc;
  this->acc_can = acc_can;
  this->inv_can = inv_can;
  this->daq_can = daq_can;

  this->timer_status_message = timer_status_message;
  this->timer_pedal_message = timer_pedal_message;
  this->timer_RTD_buzzer = timer_RTD_buzzer;
}

bool VCU::try_ts_energized() {
  switch (accumulator->get_precharge_state()) {
  case 0: // TCU is not trying to precharge
    return false;
    break;

  case 1: // TCU is actively trying to precharge
    return true;
    break;

  case 2: // TCU has precharged
    return true;
    break;

  default: // TCU is doing some fucked shit
    return false;
    break;
  }
}

bool VCU::try_ts_enabled() {
  if (RTD_button_pressed && (pedals->get_brake_travel() > 0.3)) {
    return true;
  } else {
    return false;
  }
}

bool VCU::ts_safe() {
  if (accumulator->get_precharge_state() == 2 && accumulator->get_bms_ok_hs() &&
      accumulator->get_imd_ok_hs()) {
    return true;
  } else {
    return false;
  }
}

bool VCU::set_state(state target_state) {
  switch (current_state) {
  case STARTUP: // This is just a catch for evil starts
    if (target_state == TRACTIVE_SYSTEM_DISABLED) {
      current_state = TRACTIVE_SYSTEM_DISABLED;
      return true;
    } else {
      error_code = bool_code;
      current_state = STARTUP;
      return false;
    }
    break;

  case TRACTIVE_SYSTEM_DISABLED:
    if (target_state == TRACTIVE_SYSTEM_PRECHARGING) {
      current_state = TRACTIVE_SYSTEM_PRECHARGING;
      return true;
    } else {
      error_code = bool_code;
      current_state = TRACTIVE_SYSTEM_DISABLED;
      return false;
    }
    break;

  case TRACTIVE_SYSTEM_PRECHARGING:
    if (target_state == TRACTIVE_SYSTEM_ENERGIZED && ts_safe()) {
      current_state = TRACTIVE_SYSTEM_ENERGIZED;
      return true;
    } else {
      error_code = bool_code;
      current_state = TRACTIVE_SYSTEM_DISABLED;
      return false;
    }
    break;

  case TRACTIVE_SYSTEM_ENERGIZED:
    if (target_state == TRACTIVE_SYSTEM_ENABLED && ts_safe()) {
      current_state = TRACTIVE_SYSTEM_ENABLED;

      buzzer_active = true;

      // Get the inverter prepped
      inverter->set_inverter_enable(true);
      inverter->set_torque_limit(0);

      return true;
    } else {
      error_code = bool_code;
      current_state = TRACTIVE_SYSTEM_DISABLED;
      return false;
    }
    break;

  case TRACTIVE_SYSTEM_ENABLED:
    if (target_state == READY_TO_DRIVE) {
      current_state = READY_TO_DRIVE;

      buzzer_active = false;

      // TODO: Make this torque limit easier to configure
      inverter->set_inverter_enable(true);
      inverter->set_torque_limit(120);

      return true;
    } else {
      buzzer_active = false;

      error_code = bool_code;
      current_state = TRACTIVE_SYSTEM_DISABLED;
      return false;
    }
    break;

  case READY_TO_DRIVE: // We want to be able to leave no matter what
    error_code = bool_code;

    inverter->set_inverter_enable(false);
    buzzer_active = false;

    this->current_state = TRACTIVE_SYSTEM_DISABLED;
    return true;
    break;

  default:
    this->error_code = this->bool_code;
    this->current_state = TRACTIVE_SYSTEM_DISABLED;
    return false;
    break;
  }
}

void VCU::update_bspd(uint16_t raw_relay, uint16_t raw_current,
                      uint16_t raw_brake) {
  if (raw_relay > 500)
    bspd_ok_hs = true;
  else
    bspd_ok_hs = false;

  // TODO: Make the bspd_brake_high and bspd_current_high real
  bspd_brake_high = true;
  bspd_current_high = true;
}

void VCU::update_dash_buttons(uint64_t msg, uint8_t length) {
  unpack_message(dbc, CAN_ID_DASH_BUTTONS, msg, length, 0);

  uint8_t button_val;
  decode_can_0x0eb_dash_button3status(dbc, &button_val);

  RTD_button_pressed = button_val;
}

void VCU::set_parameter(uint64_t msg, uint8_t length) {
  unpack_message(dbc, CAN_ID_VCU_SET_PARAMETER, msg, length, 0);

  uint8_t target_parameter;
  uint32_t parameter_value;
  decode_can_0x0d6_vcu_target_parameter(dbc, &target_parameter);
  decode_can_0x0d6_vcu_parameter_value(dbc, &parameter_value);

  switch (parameter(target_parameter)) {
  case POWER_LIMIT:
    inverter->set_power_limit(parameter_value);
    break;

  case TORQUE_LIMIT:
    inverter->set_torque_limit(parameter_value);
    break;

  case SPEED_MODE:
    // Not real yet
    break;

  case SPEED_LIMIT:
    // Not real yet
    break;

  case BMS_CHARGE_LIMIT:
    accumulator->set_charge_limit(parameter_value);
    break;

  case BMS_DISCHARGE_LIMIT:
    accumulator->set_discharge_limit(parameter_value);
    break;

  case LAUNCH_MODE:
    // Not real yet
    break;

  case TRACTION_MODE:
    // Not real yet
    break;

  default:
    break;
  }
}

void VCU::send_pedal_message() {
  if (timer_pedal_message) {
    encode_can_0x0cc_vcu_apps1_travel(dbc, pedals->get_apps1_travel());
    encode_can_0x0cc_vcu_apps2_travel(dbc, pedals->get_apps2_travel());
    encode_can_0x0cc_vcu_bse1_travel(dbc, pedals->get_brake_travel());

    // Init and pack the message
    can_message out_msg;
    out_msg.id = CAN_ID_VCU_PEDALS_TRAVEL;
    out_msg.length =
        pack_message(dbc, CAN_ID_VCU_PEDALS_TRAVEL, &out_msg.buf.val);

    daq_can->send_controller_message(out_msg);
    inv_can->send_controller_message(out_msg);
  }
}

void VCU::send_status_message() {
  // if (timer_status_message) {
  encode_can_0x0c3_VCU_ACCEL_BRAKE_IMPLAUSIBLE(
      dbc, pedals->get_apps_bse_fault_ok_low());
  encode_can_0x0c3_VCU_ACCEL_IMPLAUSIBLE(dbc, pedals->get_apps_fault_ok_low());
  encode_can_0x0c3_VCU_BRAKE_IMPLAUSIBLE(dbc, pedals->get_bse_fault_ok_low());
  encode_can_0x0c3_VCU_BRAKE_ACTIVE(dbc,
                                    bool(pedals->get_brake_travel() > 0.3));
  encode_can_0x0c3_VCU_BSPD_BRAKE_HIGH(dbc, bspd_brake_high);
  encode_can_0x0c3_VCU_BSPD_CURRENT_HIGH(dbc, bspd_current_high);
  encode_can_0x0c3_VCU_BSPD_OK_HIGH(dbc, bspd_ok_hs);
  encode_can_0x0c3_VCU_BMS_OK_HIGH(dbc, accumulator->get_bms_ok_hs());
  encode_can_0x0c3_VCU_IMD_OK_HIGH(dbc, accumulator->get_imd_ok_hs());
  encode_can_0x0c3_VCU_SHUTDOWN_B_OK_HIGH(dbc, 0.0); // What
  encode_can_0x0c3_VCU_SHUTDOWN_C_OK_HIGH(dbc, 0.0); // The
  encode_can_0x0c3_VCU_SHUTDOWN_D_OK_HIGH(dbc, 0.0); // Fuck
  encode_can_0x0c3_VCU_SHUTDOWN_E_OK_HIGH(dbc, 0.0); // Are these????
  encode_can_0x0c3_VCU_SOFTWARE_OK_HIGH(dbc, true);  // later
  encode_can_0x0c3_VCU_ACTIVATE_BUZZER(dbc, buzzer_active);
  encode_can_0x0c3_VCU_SOFTWARE_OK(dbc, true);           // later
  encode_can_0x0c3_VCU_DISTANCE_TRAVELLED(dbc, 0.0);     // later
  encode_can_0x0c3_VCU_ENERGY_METER_PRESENT(dbc, false); // later
  encode_can_0x0c3_VCU_INVERTER_POWERED(dbc, inverter->get_inverter_enable());
  encode_can_0x0c3_VCU_LAUNCH_CONTROL_ACTIVE(dbc, 0); // later
  encode_can_0x0c3_VCU_MAX_TORQUE(dbc, inverter->get_torque_limit());
  encode_can_0x0c3_VCU_TORQUE_MODE(dbc, torque_mode);
  encode_can_0x0c3_VCU_STATEMACHINE_STATE(dbc, current_state);

  // Init and pack the message
  can_message out_msg;
  out_msg.id = CAN_ID_VCU_STATUS;
  out_msg.length = pack_message(dbc, CAN_ID_VCU_STATUS, &out_msg.buf.val);

  daq_can->send_controller_message(out_msg);
  inv_can->send_controller_message(out_msg);
  // }
}

// These values are provided by the python script ran by the lib_dep
// https://github.com/KSU-MS/pio-git-hash-gen
#ifndef AUTO_VERSION
#warning "AUTO_VERSION was not defined by the generator!"
#define AUTO_VERSION 0xdeadbeef
#endif

#ifndef FW_PROJECT_IS_DIRTY
#warning "FW_PROJECT_IS_DIRTY was not defined by the generator!"
#define FW_PROJECT_IS_DIRTY 1
#endif

#ifndef FW_PROJECT_IS_MAIN_OR_MASTER
#warning "FW_PROJECT_IS_MAIN_OR_MASTER was not defined by the generator!"
#define FW_PROJECT_IS_MAIN_OR_MASTER 0
#endif

void VCU::send_firmware_status_message() {
  if (timer_status_message) {
    Serial.println("fella");

    // TODO: Abstract this arduino call
    encode_can_0x0c8_vcu_on_time_seconds(dbc, millis() / 1000);
    encode_can_0x0c8_vcu_fw_version(dbc, AUTO_VERSION);
    encode_can_0x0c8_vcu_project_is_dirty(dbc, FW_PROJECT_IS_DIRTY);
    encode_can_0x0c8_vcu_project_on_main(dbc, FW_PROJECT_IS_MAIN_OR_MASTER);

    can_message out_msg;
    out_msg.id = CAN_ID_VCU_FIRMWARE_VERSION;
    out_msg.length =
        pack_message(dbc, CAN_ID_VCU_FIRMWARE_VERSION, &out_msg.buf.val);

    daq_can->send_controller_message(out_msg);
    inv_can->send_controller_message(out_msg);
  }
}
