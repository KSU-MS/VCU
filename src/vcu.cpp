#include "vcu.hpp"
#include "can_tools.hpp"
#include "car.h"
#include "hal.hpp"
#include "parameters.hpp"
#include <cmath>
#include <cstdint>
#include <ota.hpp>

VCU::VCU(Pedals *pedals, Drivetrain *drivetrain, Accumulator *accumulator,
         can_obj_car_h_t *dbc, canMan *acc_can, canMan *inv_can,
         canMan *daq_can) {
  this->pedals = pedals;
  this->drivetrain = drivetrain;
  this->accumulator = accumulator;

  this->dbc = dbc;
  this->acc_can = acc_can;
  this->inv_can = inv_can;
  this->daq_can = daq_can;
}

bool VCU::try_ts_enabled() {
  if (RTD_button_pressed &&
      (pedals->get_brake_travel() > MINIMUM_BRAKE_FOR_RTD)) {
    return true;
  } else {
    return false;
  }
}

bool VCU::safety_data_fresh() {
  return acu_status_heard.fresh(now_ms, SAFETY_DATA_TIMEOUT_MS) &&
         precharge_heard.fresh(now_ms, SAFETY_DATA_TIMEOUT_MS) &&
         drivetrain->all_heard(now_ms, SAFETY_DATA_TIMEOUT_MS);
}

bool VCU::ts_safe() {
  return fault_code() == 0 &&
         accumulator->get_precharge_state() == PRECHARGE_OK_STATE &&
         drivetrain->min_bus_voltage() > TRACTIVE_SYSTEM_MINIMUM_VOLTAGE;
}

uint8_t VCU::fault_code() {
  uint8_t code = 0;
  if (!acu_status_heard.fresh(now_ms, SAFETY_DATA_TIMEOUT_MS) ||
      !precharge_heard.fresh(now_ms, SAFETY_DATA_TIMEOUT_MS))
    code |= FAULT_ACU_SILENT;
  if (!drivetrain->all_heard(now_ms, SAFETY_DATA_TIMEOUT_MS))
    code |= FAULT_INVERTER_SILENT;
  if (!accumulator->get_imd_ok_hs())
    code |= FAULT_IMD;
  if (!accumulator->get_bms_ok_hs())
    code |= FAULT_BMS;
  return code;
}

bool VCU::hv_maybe_present() {
  return !drivetrain->all_heard(now_ms, SAFETY_DATA_TIMEOUT_MS) ||
         drivetrain->max_bus_voltage() >= HIGH_VOLTAGE_PRESENT;
}

void VCU::drop_out() {
  drivetrain->set_enable(false);
  drivetrain->set_torque_limits(false);
  buzzer_active = false;
  hal::digital_write(BUZZER, false);

  if (fault_code() && hv_maybe_present()) {
    // Not TRACTIVE_SYSTEM_DISABLED: that would say the car isn't live. Keep
    // the pumps going, there may be voltage and a hot inverter.
    current_state = TRACTIVE_SYSTEM_FAULT;
    hal::log("FAULT with high voltage possibly present, code ");
    hal::log(long(fault_code()));
    hal::logln("");
    return;
  }

  // The driver switched the tractive system off, or precharge was undone
  current_state = TRACTIVE_SYSTEM_DISABLED;
  hal::digital_write(LOWSIDE1, false);
  hal::digital_write(LOWSIDE2, false);
}

bool VCU::set_state(state target_state) {
  switch (current_state) {

  // This is just a catch for evil starts
  case STARTUP:
    if (target_state == TRACTIVE_SYSTEM_DISABLED) {
      current_state = TRACTIVE_SYSTEM_DISABLED;

      hal::digital_write(LOWSIDE1, false);
      hal::digital_write(LOWSIDE2, false);

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

      hal::digital_write(LOWSIDE1, true);
      hal::digital_write(LOWSIDE2, true);

      return true;
    } else {
      error_code = bool_code;
      current_state = TRACTIVE_SYSTEM_DISABLED;

      hal::digital_write(LOWSIDE1, false);
      hal::digital_write(LOWSIDE2, false);

      return false;
    }
    break;

  case TRACTIVE_SYSTEM_ENERGIZED:
    if (target_state == TRACTIVE_SYSTEM_ENABLED && ts_safe()) {
      current_state = TRACTIVE_SYSTEM_ENABLED;

      buzzer_active = true;

      hal::digital_write(LOWSIDE1, true);
      hal::digital_write(LOWSIDE2, true);

      // Get the inverter prepped
      drivetrain->set_enable(true);
      drivetrain->set_torque_limits(false);

      return true;
    } else {
      error_code = bool_code;
      current_state = TRACTIVE_SYSTEM_DISABLED;

      hal::digital_write(LOWSIDE1, false);
      hal::digital_write(LOWSIDE2, false);

      return false;
    }
    break;

  case TRACTIVE_SYSTEM_ENABLED:
    if (target_state == READY_TO_DRIVE && ts_safe()) {
      current_state = READY_TO_DRIVE;

      buzzer_active = false;

      hal::digital_write(LOWSIDE1, true);
      hal::digital_write(LOWSIDE2, true);

      // TODO: Make this torque limit easier to configure
      drivetrain->set_enable(true);
      drivetrain->set_torque_limits(true);

      return true;
    } else {
      buzzer_active = false;

      // Entering ENABLED turned the inverter on, undo that on the way out
      drivetrain->set_enable(false);
      drivetrain->set_torque_limits(false);

      error_code = bool_code;
      current_state = TRACTIVE_SYSTEM_DISABLED;

      hal::digital_write(LOWSIDE1, false);
      hal::digital_write(LOWSIDE2, false);

      return false;
    }
    break;

  case READY_TO_DRIVE: // We want to be able to leave no matter what
    drivetrain->set_enable(false);
    drivetrain->set_torque_limits(false);

    buzzer_active = false;

    hal::digital_write(LOWSIDE1, false);
    hal::digital_write(LOWSIDE2, false);

    this->current_state = TRACTIVE_SYSTEM_DISABLED;
    return true;
    break;

  default:
    drivetrain->set_enable(false);
    drivetrain->set_torque_limits(false);

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

  // Traction control on/off, on the press
  uint8_t tc_button;
  decode_can_0x0eb_dash_button6status(dbc, &tc_button);
  if (tc_button && !tc_button_was_down)
    traction.toggle();
  tc_button_was_down = tc_button;
}

void VCU::set_parameter(uint64_t msg, uint8_t length) {
  unpack_message(dbc, CAN_ID_VCU_SET_PARAMETER, msg, length, 0);

  uint8_t target_parameter;
  uint32_t parameter_value;
  decode_can_0x0c9_vcu_target_parameter(dbc, &target_parameter);
  decode_can_0x0c9_vcu_parameter_value(dbc, &parameter_value);

  switch (parameter(target_parameter)) {
  case POWER_LIMIT:
    set_power_limit(parameter_value);
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

  case TRACTION_MODE:
    traction.select(uint8_t(parameter_value));
    break;

  default:
    break;
  }
}

//
//// CAN stage
//
// Each bus is drained of everything that arrived since the last loop, so the
// safety data (IMD/BMS/precharge, inverter voltage) is never more than one
// loop old. Bounded by the queue size so a flooded bus can't stall the loop.
void VCU::update_acc_can() {
  for (size_t i = 0;
       i < CAN_RX_QUEUE_SIZE && acc_can->check_controller_message(); i++) {
    can_message msg_in = acc_can->get_controller_message();
    daq_can->send_controller_message(msg_in);

    switch (msg_in.id) {
    case CAN_ID_ACU_SHUTDOWN_STATUS:
      acu_status_heard.heard(now_ms);
      accumulator->update_acu_status(msg_in.buf.val, msg_in.length);
      break;

    case CAN_ID_PRECHARGE_STATUS:
      precharge_heard.heard(now_ms);
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
      update_vehicle_sensors(msg_in);
      break;
    }
  }
}

void VCU::update_inv_can() {
  for (size_t i = 0;
       i < CAN_RX_QUEUE_SIZE && inv_can->check_controller_message(); i++) {
    can_message msg_in = inv_can->get_controller_message();
    daq_can->send_controller_message(msg_in);

    // The inverters' own frames first: they decode them, we see what changed
    const Drivetrain::Heard heard = drivetrain->on_frame(msg_in, now_ms);
    if (heard.feedback != Inverter::NOT_MINE) {
      Inverter &inv = drivetrain->inverter(heard.index);
      // Traction control works off the rear axle's motor, if there is one
      if (&inv == drivetrain->rear_axle()) {
        if (heard.feedback & Inverter::MOTOR_SPEED)
          traction.motor_rpm(inv.get_motor_rpm(), now_ms);
        if (heard.feedback & Inverter::MOTOR_TORQUE)
          traction.motor_torque(float(inv.get_motor_torque()));
      }
      // The ACU's precharge reads the Cascadia's voltage frame
      if ((heard.feedback & Inverter::DC_VOLTAGE) &&
          drivetrain->drive(heard.index).inverter ==
              vehicle::InverterKind::CASCADIA_PM100)
        acc_can->send_controller_message(msg_in);
      continue;
    }

    switch (msg_in.id) {
    case CAN_ID_DASH_BUTTONS:
      update_dash_buttons(msg_in.buf.val, msg_in.length);
      break;

    case CAN_ID_VCU_SET_PARAMETER:
      set_parameter(msg_in.buf.val, msg_in.length);
      break;

    default:
      update_vehicle_sensors(msg_in);
      break;
    }
  }
}

// The DAQ bus is mostly outbound, the only thing we listen for on it is a
// tester doing a firmware update.
void VCU::update_daq_can(uint32_t now) {
  for (size_t i = 0;
       i < CAN_RX_QUEUE_SIZE && daq_can->check_controller_message(); i++) {
    can_message msg_in = daq_can->get_controller_message();

    if (updater && ota::Service::wants(msg_in))
      updater->on_frame(msg_in, now);
    else if (msg_in.id == CAN_ID_PI_TORQUE_COMMAND)
      pi.on_frame(msg_in.buf.byte, msg_in.length, now);
    else
      update_vehicle_sensors(msg_in);
  }
}

void VCU::update_vehicle_sensors(const can_message &msg) {
  switch (msg.id) {
  case CAN_ID_VECTORNAV_VELOCITY: {
    unpack_message(dbc, msg.id, msg.buf.val, msg.length, 0);
    double north, east;
    decode_can_0x1f7_vn_v_n(dbc, &north);
    decode_can_0x1f7_vn_v_e(dbc, &east);
    traction.vectornav_velocity(float(north), float(east), now_ms);
    vn_speed = std::sqrt(north * north + east * east);
    vn_heard.heard(now_ms);
    break;
  }
  case CAN_ID_VECTORNAV_ACCELERATION: {
    unpack_message(dbc, msg.id, msg.buf.val, msg.length, 0);
    double forward;
    decode_can_0x1f8_vn_accel_x(dbc, &forward);
    traction.vectornav_accel(float(forward));
    vn_accel = forward;
    break;
  }
  case CAN_ID_AN1_FL_WHEELSPEED:
  case CAN_ID_AN1_FR_WHEELSPEED:
  case CAN_ID_AN1_RL_WHEELSPEED:
  case CAN_ID_AN1_RR_WHEELSPEED: {
    unpack_message(dbc, msg.id, msg.buf.val, msg.length, 0);
    uint16_t rpm = 0;
    // The four share a layout; each has its own decoder
    switch (msg.id) {
    case CAN_ID_AN1_FL_WHEELSPEED: decode_can_0x38e_wheel_rpm(dbc, &rpm); break;
    case CAN_ID_AN1_FR_WHEELSPEED: decode_can_0x38f_wheel_rpm(dbc, &rpm); break;
    case CAN_ID_AN1_RL_WHEELSPEED: decode_can_0x390_wheel_rpm(dbc, &rpm); break;
    default: decode_can_0x391_wheel_rpm(dbc, &rpm); break;
    }
    traction.wheel_rpm(int(msg.id - CAN_ID_AN1_FL_WHEELSPEED), float(rpm));
    break;
  }
  default:
    break;
  }
}

bool VCU::safe_to_program() {
  return (current_state == STARTUP ||
          current_state == TRACTIVE_SYSTEM_DISABLED) &&
         !drivetrain->any_enabled();
}

bool VCU::set_power_limit(uint32_t kw) {
  const bool parked = current_state != TRACTIVE_SYSTEM_ENABLED &&
                      current_state != READY_TO_DRIVE &&
                      !drivetrain->any_enabled();
  const double ceiling = drivetrain->peak_power_kw();
  if (!parked || kw > ceiling) {
    hal::log(parked ? "Power limit refused, over the motors' peak kW: "
                    : "Power limit refused, not parked: ");
    hal::log(long(kw));
    hal::logln("");
    return false;
  }
  const double limit = kw ? double(kw) : double(POWER_LIMIT_KW);
  drivetrain->set_power_limit_kw(limit);
  hal::log("Power limit kW: ");
  hal::log(long(limit));
  hal::logln(limit > POWER_LIMIT_KW ? " (OVER THE RULES' LIMIT: testing only)"
                                    : "");
  return true;
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

  daq_can->send_controller_message(out_msg);
}

void VCU::send_pedal_raw_message(uint16_t raw_apps1, uint16_t raw_apps2,
                                 uint16_t raw_brake) {
  encode_can_0x0c5_apps_1_raw(dbc, raw_apps1);
  encode_can_0x0c5_apps_2_raw(dbc, raw_apps2);
  encode_can_0x0c5_bse_raw(dbc, raw_brake);

  can_message out_msg;
  out_msg.id = CAN_ID_VCU_PEDAL_READINGS;
  out_msg.length = pack_message(dbc, out_msg.id, &out_msg.buf.val);

  daq_can->send_controller_message(out_msg);
}
void VCU::send_steering_raw_message(uint16_t raw_steering) {
  encode_can_0x383_an1_uint12(dbc, raw_steering);

  can_message out_msg;
  out_msg.id = CAN_ID_AN1_STEERINGPOT;
  out_msg.length = pack_message(dbc, out_msg.id, &out_msg.buf.val);

  daq_can->send_controller_message(out_msg);
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

  daq_can->send_controller_message(out_msg);
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
  encode_can_0x0c3_vcu_max_torque(dbc, drivetrain->torque_limit_nm());
  encode_can_0x0c3_vcu_statemachine_state(dbc, current_state);

  // Init and pack the message
  can_message out_msg;
  out_msg.id = CAN_ID_VCU_STATUS;
  out_msg.length = pack_message(dbc, out_msg.id, &out_msg.buf.val);

  inv_can->send_controller_message(out_msg);
  acc_can->send_controller_message(out_msg);
  daq_can->send_controller_message(out_msg);
}

void VCU::send_firmware_status_message() {
  // TODO: Abstract this arduino call
  encode_can_0x3bc_board_on_time_seconds(dbc, hal::millis() / 1000);
  encode_can_0x3bc_firmware_version(dbc, AUTO_VERSION);
  encode_can_0x3bc_firmware_is_dirty(dbc, FW_PROJECT_IS_DIRTY);
  encode_can_0x3bc_firmware_on_main(dbc, FW_PROJECT_IS_MAIN_OR_MASTER);

  can_message out_msg;
  out_msg.id = CAN_ID_VCU_BOARD_DATA;
  out_msg.length = pack_message(dbc, out_msg.id, &out_msg.buf.val);

  daq_can->send_controller_message(out_msg);
}

void VCU::send_power_tracking_message() {
  encode_can_0x0cb_vcu_lifetime_distance(dbc, drivetrain->distance_m());
  encode_can_0x0cb_vcu_lifetime_ontime(dbc, accumulator->get_consumed_wh());

  can_message out_msg;
  out_msg.id = CAN_ID_VCU_LIFETIME_DISTANCE_AND_ONTIME;
  out_msg.length = pack_message(dbc, out_msg.id, &out_msg.buf.val);

  daq_can->send_controller_message(out_msg);
}

void VCU::send_pi_status_message() {
  // The DBC's encoders zero anything out of range, so keep it in range
  auto nm = [](double v) { return v < 0 ? 0 : v > 6553.4 ? 6553.4 : v; };
  const double ceiling = pi.ceiling();
  encode_can_0x0cd_vcu_pi_mode(dbc, pi.get_mode());
  encode_can_0x0cd_vcu_pi_reason(dbc, pi.get_reason());
  encode_can_0x0cd_vcu_pi_rejected_frames(dbc, pi.rejected_frames());
  encode_can_0x0cd_vcu_pi_driver_torque(dbc, nm(driver_nm));
  encode_can_0x0cd_vcu_pi_torque_sent(dbc, nm(torque_nm));
  encode_can_0x0cd_vcu_pi_ceiling(dbc, ceiling < 0 ? 6553.5 : nm(ceiling));

  can_message out_msg;
  out_msg.id = CAN_ID_VCU_PI_STATUS;
  out_msg.length = pack_message(dbc, out_msg.id, &out_msg.buf.val);

  daq_can->send_controller_message(out_msg);
}

void VCU::send_traction_control_message() {
  // The DBC's encoders zero anything out of range, so keep it in range
  auto clamp = [](double v, double lo, double hi) {
    return !(v == v) ? 0 : v < lo ? lo : (v > hi ? hi : v); // NaN: 0
  };
  const uint8_t reasons = traction.reasons();
  encode_can_0x0cc_vcu_tc_selected(dbc, traction.selected());
  encode_can_0x0cc_vcu_tc_active(dbc, traction.active());
  encode_can_0x0cc_vcu_tc_no_vectornav(
      dbc, bool(reasons & TractionControl::TC_NO_VECTORNAV));
  encode_can_0x0cc_vcu_tc_no_motor_speed(
      dbc, bool(reasons & TractionControl::TC_NO_MOTOR_SPEED));
  encode_can_0x0cc_vcu_tc_gear_ratio_fault(
      dbc, bool(reasons & TractionControl::TC_GEAR_RATIO));
  encode_can_0x0cc_vcu_tc_waiting_for_lift(
      dbc, bool(reasons & TractionControl::TC_WAITING_FOR_LIFT));
  encode_can_0x0cc_vcu_tc_slip(dbc, clamp(traction.slip(), -32.7, 32.7));
  encode_can_0x0cc_vcu_tc_speed(dbc, clamp(traction.speed(), 0, 655.3));
  encode_can_0x0cc_vcu_tc_scale(dbc, clamp(traction.scale(), 0, 2.55));
  encode_can_0x0cc_vcu_tc_grip(dbc, clamp(traction.grip(), 0, 5.1));

  can_message out_msg;
  out_msg.id = CAN_ID_VCU_TRACTION_CONTROL;
  out_msg.length = pack_message(dbc, out_msg.id, &out_msg.buf.val);

  daq_can->send_controller_message(out_msg);
}
