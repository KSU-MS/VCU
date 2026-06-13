#include "vcu.hpp"
#include "car.h"
#include "core_pins.h"
#include "parameters.hpp"

VCU::VCU(Pedals *pedals, Inverter *inverter, Accumulator *accumulator,
         can_obj_car_h_t *dbc, canMan *acc_can, canMan *inv_can) {
  this->pedals = pedals;
  this->inverter = inverter;
  this->accumulator = accumulator;

  this->dbc = dbc;
  this->acc_can = acc_can;
  this->inv_can = inv_can;
}

bool VCU::try_ts_enabled() {
  if (RTD_button_pressed &&
      (pedals->get_brake_travel() > MINIMUM_BRAKE_FOR_RTD)) {
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
      inverter->set_torque_limit(0);

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
      inverter->set_torque_limit(MAX_TORQUE_LIMIT_NM);

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
    inverter->set_torque_limit(0);

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
      inverter->set_torque_limit(0);

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
    inverter->set_torque_limit(0);

    buzzer_active = false;

    this->current_state = TRACTIVE_SYSTEM_DISABLED;
    return false;
    break;
  }
}

void VCU::update_bspd(uint16_t raw_relay, uint16_t raw_brake,
                      uint16_t raw_current) {
  if (raw_relay > 800)
    bspd_ok_hs = false;
  else
    bspd_ok_hs = true;

  if (raw_brake > 800)
    bspd_brake_high = false;
  else
    bspd_brake_high = true;

  if (raw_current > 800)
    bspd_current_high = false;
  else
    bspd_current_high = true;
}

void VCU::update_dash_buttons(uint64_t msg, uint8_t length) {
  unpack_message(dbc, CAN_ID_DASH_BUTTONS, msg, length, 0);

  uint8_t button_val;
  decode_can_0x0eb_dash_button5status(dbc, &button_val);

  RTD_button_pressed = button_val;
}

void VCU::set_parameter(uint64_t msg, uint8_t length) {
  unpack_message(dbc, CAN_ID_VCU_SET_PARAMETER, msg, length, 0);

  uint8_t target_parameter;
  uint32_t parameter_value;
  decode_can_0x0c9_vcu_target_parameter(dbc, &target_parameter);
  decode_can_0x0c9_vcu_parameter_value(dbc, &parameter_value);

  switch (parameter(target_parameter)) {
  case POWER_LIMIT:
    // inverter->set_power_limit_kw(parameter_value);
    break;

  case TORQUE_LIMIT:
    // inverter->set_torque_limit(parameter_value);
    break;

  case SPEED_MODE:
    // Not real yet
    break;

  case SPEED_LIMIT:
    // Not real yet
    break;

  case INV_DISCHARGE_LIMIT:
    // inverter->set_current_limits(INVERTER_CHARGE_LIMIT, parameter_value);
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

//
//// CAN stage
void VCU::update_acc_can() {
  if (acc_can->check_controller_message()) {
    can_message msg_in = acc_can->get_controller_message();
    inv_can->send_controller_message(msg_in);

    switch (msg_in.id) {
    case CAN_ID_ACU_SHUTDOWN_STATUS:
      accumulator->update_acu_status(msg_in.buf.val, msg_in.length);
      break;

    case CAN_ID_PRECHARGE_STATUS:
      accumulator->update_precharge_status(msg_in.buf.val, msg_in.length);
      break;

    case CAN_ID_MSGID_0X6B1:
      accumulator->update_pack_power(msg_in.buf.val, msg_in.length);
      break;

    // We foward this to the inverter bus for the dash
    case CAN_ID_MSGID_0X6B3:
      inv_can->send_controller_message(msg_in);
      break;

    default:
      break;
    }
  }
}

void VCU::update_inv_can() {
  if (inv_can->check_controller_message()) {
    can_message msg_in = inv_can->get_controller_message();

    switch (msg_in.id) {
    case CAN_ID_DASH_BUTTONS:
      update_dash_buttons(msg_in.buf.val, msg_in.length);
      break;

    case CAN_ID_M165_MOTOR_POSITION_INFO:
      inverter->update_motor_feedback(msg_in.buf.val, msg_in.length);
      break;

    case CAN_ID_M166_CURRENT_INFO:
      inverter->update_bus_current(msg_in.buf.val, msg_in.length);
      break;

    case CAN_ID_M167_VOLTAGE_INFO:
      inverter->update_bus_voltage(msg_in.buf.val, msg_in.length);
      acc_can->send_controller_message(msg_in); // Forward this for precharge
      break;

    case CAN_ID_VCU_SET_PARAMETER:
      set_parameter(msg_in.buf.val, msg_in.length);
      break;

    default:
      break;
    }
  }
}

void VCU::send_pedal_travel_message() {
  encode_can_0x0c4_vcu_apps1_travel(dbc, pedals->get_apps1_travel() * 100);
  encode_can_0x0c4_vcu_apps2_travel(dbc, pedals->get_apps2_travel() * 100);
  encode_can_0x0c4_vcu_bse_travel(dbc, pedals->get_brake_travel() * 100);

  // Init and pack the message
  can_message out_msg;
  out_msg.id = CAN_ID_VCU_PEDALS_TRAVEL;
  out_msg.length =
      pack_message(dbc, CAN_ID_VCU_PEDALS_TRAVEL, &out_msg.buf.val);

  inv_can->send_controller_message(out_msg);
}

void VCU::send_pedal_raw_message(uint16_t raw_apps1, uint16_t raw_apps2,
                                 uint16_t raw_brake) {
  encode_can_0x0c5_apps_1_raw(dbc, raw_apps1);
  encode_can_0x0c5_apps_2_raw(dbc, raw_apps2);
  encode_can_0x0c5_bse_raw(dbc, raw_brake);

  can_message out_msg;
  out_msg.id = CAN_ID_VCU_PEDAL_READINGS;
  out_msg.length = pack_message(dbc, out_msg.id, &out_msg.buf.val);

  inv_can->send_controller_message(out_msg);
}

void VCU::send_brake_pressure_message(uint16_t raw_brake) {
  uint16_t kpa = 0;
  if (raw_brake < 102) {
    kpa = 0;
  } else if (raw_brake > 922) {
    kpa = 10342;
  } else {
    kpa =
        uint16_t(double(raw_brake - 102) * double((double)1500 / (double)820));
  }

  encode_can_0x384_pressure_delta(dbc, 0.0);
  encode_can_0x384_pressure_kPa(dbc, kpa);
  encode_can_0x384_an1_uint12(dbc, raw_brake);

  can_message out_msg;
  out_msg.id = CAN_ID_AN1_FRONT_BRAKEPRESSURE;
  out_msg.length = pack_message(dbc, out_msg.id, &out_msg.buf.val);

  inv_can->send_controller_message(out_msg);
}

void VCU::send_status_message() {
  encode_can_0x0c3_vcu_accel_brake_implausible(
      dbc, pedals->get_apps_bse_fault_ok_low());
  encode_can_0x0c3_vcu_accel_implausible(dbc, pedals->get_apps_fault_ok_low());
  encode_can_0x0c3_vcu_brake_implausible(dbc, pedals->get_bse_fault_ok_low());
  encode_can_0x0c3_vcu_brake_active(
      dbc, bool(pedals->get_brake_travel() > MINIMUM_BRAKE_FOR_RTD));
  encode_can_0x0c3_vcu_bspd_brake_high(dbc, bspd_brake_high);
  encode_can_0x0c3_vcu_bspd_current_high(dbc, bspd_current_high);
  encode_can_0x0c3_vcu_bspd_ok_high(dbc, bspd_ok_hs);
  encode_can_0x0c3_vcu_bms_ok_high(dbc, accumulator->get_bms_ok_hs());
  encode_can_0x0c3_vcu_imd_ok_high(dbc, accumulator->get_imd_ok_hs());
  encode_can_0x0c3_vcu_max_torque(dbc, inverter->get_torque_limit());
  encode_can_0x0c3_vcu_statemachine_state(dbc, current_state);

  // Init and pack the message
  can_message out_msg;
  out_msg.id = CAN_ID_VCU_STATUS;
  out_msg.length = pack_message(dbc, out_msg.id, &out_msg.buf.val);

  inv_can->send_controller_message(out_msg);
  acc_can->send_controller_message(out_msg);
}

void VCU::send_firmware_status_message() {
  // TODO: Abstract this arduino call
  encode_can_0x3bc_board_on_time_seconds(dbc, millis() / 1000);
  encode_can_0x3bc_firmware_version(dbc, AUTO_VERSION);
  encode_can_0x3bc_firmware_is_dirty(dbc, FW_PROJECT_IS_DIRTY);
  encode_can_0x3bc_firmware_on_main(dbc, FW_PROJECT_IS_MAIN_OR_MASTER);

  can_message out_msg;
  out_msg.id = CAN_ID_VCU_BOARD_DATA;
  out_msg.length = pack_message(dbc, out_msg.id, &out_msg.buf.val);

  inv_can->send_controller_message(out_msg);
}

void VCU::send_power_tracking_message() {
  encode_can_0x0cb_vcu_lifetime_distance(dbc, inverter->get_motor_distance_M());
  encode_can_0x0cb_vcu_lifetime_ontime(dbc, accumulator->get_consumed_wh());

  can_message out_msg;
  out_msg.id = CAN_ID_VCU_LIFETIME_DISTANCE_AND_ONTIME;
  out_msg.length = pack_message(dbc, out_msg.id, &out_msg.buf.val);

  inv_can->send_controller_message(out_msg);
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
//   inv_can->send_controller_message(out_msg);
//   daq_can->send_controller_message(out_msg);
// }
