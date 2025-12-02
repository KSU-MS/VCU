#include "inverter.hpp"
#include "car.h"
#include "data_handler.hpp"
#include "parameters.hpp"

Inverter::Inverter(bool spin_direction, std::array<parameter, 25> *params) {
  this->spin_forward = spin_direction;

  this->params = params;

  this->ping();

  this->set_inv_parameter(
      Motor_Overspeed_EEPROM_RPM,
      static_cast<uint32_t>(params->at(MAX_RPM_LIMIT).parameter_value));
  this->set_inv_parameter(
      Max_Speed_EEPROM_RPM,
      static_cast<uint32_t>(params->at(SOFT_RPM_LIMIT).parameter_value));
  this->set_inv_parameter(
      Break_Speed_EEPROM_RPM,
      static_cast<uint32_t>(params->at(BRAKE_SPEED_LIMIT).parameter_value));
  // this->set_inv_parameter(Speed_Rate_Limit_EEPROM_RPM_per_s,
  // SPEED_RATE_LIMIT_RPM_PER_S);

  this->torquepid =
      QuickPID(&torque_over_nm, &torque_adjustment, 0,
               double(torque_kp_x100) / 100, double(torque_ki_x100) / 100,
               double(torque_kd_x100) / 100, QuickPID::Action::direct);
  this->torquepid.SetSampleTimeUs(5000);
}

void Inverter::set_current_limits(uint16_t charge_limit,
                                  uint16_t discharge_limit) {
  encode_can_0x202_BMS_Max_Charge_Current(&kms_can, charge_limit);
  encode_can_0x202_BMS_Max_Discharge_Current(&kms_can, discharge_limit);

  can_message out_msg;
  out_msg.id = CAN_ID_BMS_CURRENT_LIMIT;
  out_msg.length =
      pack_message(&kms_can, CAN_ID_BMS_CURRENT_LIMIT, &out_msg.buf.val);

  inv_can.send_controller_message(out_msg);
  daq_can.send_controller_message(out_msg);
}

void Inverter::update_bus_current(uint64_t msg_in, uint8_t length) {
  unpack_message(&kms_can, CAN_ID_M166_CURRENT_INFO, msg_in, length, 0);

  decode_can_0x0a6_INV_DC_Bus_Current(&kms_can, &bus_current);
}

void Inverter::update_bus_voltage(uint64_t msg_in, uint8_t length) {
  unpack_message(&kms_can, CAN_ID_M167_VOLTAGE_INFO, msg_in, length, 0);

  decode_can_0x0a7_INV_DC_Bus_Voltage(&kms_can, &bus_voltage);
}

void Inverter::calculate_power_output() {
  // power output is calculated as bus_current * bus_voltage
  power_over_w = (bus_current * bus_voltage);
}

void Inverter::update_motor_feedback(uint64_t msg_in, uint8_t length) {
  unpack_message(&kms_can, CAN_ID_M165_MOTOR_POSITION_INFO, msg_in, length, 0);

  decode_can_0x0a5_INV_Motor_Speed(&kms_can, &motor_rpm);
}

void Inverter::calculate_motor_distance_M(uint32_t time_msec) {
  uint32_t time_elaped_msec = time_msec - time_last_msec;

  double velocity_Msec =
      (double(motor_rpm) / 60 / GEAR_RATIO) * WHEEL_CIRCUMFRANCE_M;

  distance_M += (double(time_elaped_msec) / 1000) * velocity_Msec;

  time_last_msec = time_msec;
}

void Inverter::ping() {
  encode_can_0x0c0_VCU_INV_Torque_Command(&kms_can, 0.0);
  encode_can_0x0c0_VCU_INV_Torque_Limit_Command(&kms_can, 0.0);
  encode_can_0x0c0_VCU_INV_Speed_Command(&kms_can, 0);
  encode_can_0x0c0_VCU_INV_Speed_Mode_Enable(&kms_can, 0);
  encode_can_0x0c0_VCU_INV_Direction_Command(&kms_can, spin_forward);
  encode_can_0x0c0_VCU_INV_Inverter_Discharge(&kms_can, inverter_discharge);
  encode_can_0x0c0_VCU_INV_Inverter_Enable(&kms_can, inverter_enable);

  can_message out_msg;
  out_msg.id = CAN_ID_M192_COMMAND_MESSAGE;
  out_msg.length =
      pack_message(&kms_can, CAN_ID_M192_COMMAND_MESSAGE, &out_msg.buf.val);

  inv_can.send_controller_message(out_msg);
  daq_can.send_controller_message(out_msg);
}

void Inverter::send_clear_faults() {
  encode_can_0x0c1_VCU_INV_Parameter_Address(&kms_can, 20);
  encode_can_0x0c1_VCU_INV_Parameter_RW_Command(&kms_can, 1);
  encode_can_0x0c1_VCU_INV_Parameter_Data(&kms_can, 0);

  can_message out_msg;
  out_msg.length =
      pack_message(&kms_can, CAN_ID_M192_COMMAND_MESSAGE, &out_msg.buf.val);

  inv_can.send_controller_message(out_msg);
  daq_can.send_controller_message(out_msg);
}

void Inverter::command_torque(double torque_request) {
  double torque_target = torque_request;

  // EV. 1.4.4 A violation is defined as using more than the specified maximum
  // power OR exceeding the maximum voltage EITHER: a. Continuously for 100 ms
  // or more b. After a moving average over 500 ms is applied

  // electronic rev limiter

  // https://www.desmos.com/calculator/j8kydktjry

  // calculate excess power output
  double power_over_w = std::max(
      0.0, power_output -
               (((*params)[POWER_LIMIT].parameter_value / 10.0) * 1000.0));

  if (power_over_w > 1e-6) {
    // calculate excess torque output from motor speed and excess power
    torque_over_nm =
        power_over_w / std::max(1e-6, (motor_rpm / 60.0) * 2.0 * M_PI);

    // cap torque adjustment to 10% of max torque
    double torque_adjustment_capped =
        std::min(torque_over_nm, (*params)[MAX_TORQUE].parameter_value * 0.1);

    // ensure torque subtracted is not negative
    torque_adjustment_capped = std::max(0.0, torque_adjustment_capped);

    // adjust torque target
    torque_target -= torque_adjustment_capped;

    // ensure torque target is not negative
    torque_target = std::max(0.0, torque_target);
  } else {
    torque_over_nm = 0.0;
  }
  // else
  // {
  //   torque_I *= 0.98;
  //   // torque_target -= torque_kp * torque_over_nm + torque_I;
  // }
  // speed limiter
  // if (angular_vel_over_rad_s > 0)
  // {
  //   torque_over_nm += power_over_w / angular_vel_over_rad_s;
  // }
  // else
  // {
  //   speed_I *= 0.98;
  // }
  // double speed_err = speed_kp * angular_vel_over_rad_s + speed_I;
  // torque_target -= speed_err;

  // if (motor_rpm >= speed_limit && torque_target > 0)
  // {
  //   speed_I += speed_ki * ((motor_rpm - speed_limit) / speed_limit) * dt_s;
  //   torque_target *= 1 - (speed_kp * ((motor_rpm - speed_limit) /
  //   speed_limit) + speed_I);
  // }
  // else
  // {
  //   speed_I *= 0.98;
  // }

  encode_can_0x0c0_VCU_INV_Torque_Command(
      &kms_can, torque_target); // torque command to INV

  // all of this should really be handled elsewhere, we call this function on a
  // 200hz interval

  // encode_can_0x0c0_VCU_INV_Torque_Limit_Command(dbc,
  // (*params)[MAX_TORQUE].parameter_value);
  // encode_can_0x0c0_VCU_INV_Speed_Command(dbc, 0);
  // encode_can_0x0c0_VCU_INV_Speed_Mode_Enable(dbc, 0);
  // encode_can_0x0c0_VCU_INV_Direction_Command(dbc, spin_forward);
  // encode_can_0x0c0_VCU_INV_Inverter_Discharge(dbc, inverter_discharge);
  // encode_can_0x0c0_VCU_INV_Inverter_Enable(dbc, inverter_enable);

  can_message out_msg;
  out_msg.id = CAN_ID_M192_COMMAND_MESSAGE;
  out_msg.length =
      pack_message(&kms_can, CAN_ID_M192_COMMAND_MESSAGE, &out_msg.buf.val);

  inv_can.send_controller_message(out_msg);
  daq_can.send_controller_message(out_msg);
}

void Inverter::command_speed(int16_t speed_request) // unused
{
  encode_can_0x0c0_VCU_INV_Torque_Command(&kms_can, 0.0);
  encode_can_0x0c0_VCU_INV_Torque_Limit_Command(
      &kms_can, (*params)[MAX_TORQUE].parameter_value);
  encode_can_0x0c0_VCU_INV_Speed_Command(&kms_can, speed_request);
  encode_can_0x0c0_VCU_INV_Speed_Mode_Enable(&kms_can, speed_mode);
  encode_can_0x0c0_VCU_INV_Direction_Command(&kms_can, spin_forward);
  encode_can_0x0c0_VCU_INV_Inverter_Discharge(&kms_can, inverter_discharge);
  encode_can_0x0c0_VCU_INV_Inverter_Enable(&kms_can, inverter_enable);

  can_message out_msg;
  out_msg.id = CAN_ID_M192_COMMAND_MESSAGE;
  out_msg.length =
      pack_message(&kms_can, CAN_ID_M192_COMMAND_MESSAGE, &out_msg.buf.val);

  inv_can.send_controller_message(out_msg);
  daq_can.send_controller_message(out_msg);
}

void Inverter::set_inv_parameter(uint16_t param_address, uint32_t param_data) {
  encode_can_0x0c1_VCU_INV_Parameter_Address(&kms_can, param_address);
  encode_can_0x0c1_VCU_INV_Parameter_RW_Command(&kms_can, 1); // write
  encode_can_0x0c1_VCU_INV_Parameter_Data(&kms_can, param_data);

  can_message out_msg;
  out_msg.id = CAN_ID_M192_COMMAND_MESSAGE;
  out_msg.length =
      pack_message(&kms_can, CAN_ID_M192_COMMAND_MESSAGE, &out_msg.buf.val);

  inv_can.send_controller_message(out_msg);
  daq_can.send_controller_message(out_msg);
}
