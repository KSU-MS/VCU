#include "inverter.hpp"
#include "car.h"

Inverter::Inverter(bool (*timer_mc_kick)(), bool (*timer_current_limit)(),
                   bool (*timer_motor_controller_send)(), bool spin_direction,
                   canMan *can, can_obj_car_h_t *dbc,
                   float over_power_decay_factor) {
  this->timer_mc_kick = timer_mc_kick;
  this->timer_current_limit = timer_current_limit;
  this->timer_motor_controller_send = timer_motor_controller_send;

  this->spin_forward = spin_direction;

  this->over_power_decay_factor = over_power_decay_factor;

  this->can = can;
  this->dbc = dbc;

  this->ping();
}

void Inverter::update_bus_current(uint64_t msg_in, uint8_t length) {
  unpack_message(dbc, CAN_ID_M166_CURRENT_INFO, msg_in, length, 0);

  decode_can_0x0a6_D4_DC_Bus_Current(dbc, &bus_current);
}

void Inverter::update_bus_voltage(uint64_t msg_in, uint8_t length) {
  unpack_message(dbc, CAN_ID_M167_VOLTAGE_INFO, msg_in, length, 0);

  decode_can_0x0a7_D1_DC_Bus_Voltage(dbc, &bus_voltage);
}

void Inverter::ping() {
  encode_can_0x0c0_Torque_Command(dbc, 0.0);
  encode_can_0x0c0_Torque_Limit_Command(dbc, 0.0);
  encode_can_0x0c0_Speed_Command(dbc, 0.0);
  encode_can_0x0c0_Speed_Mode_Enable(dbc, 0.0);
  encode_can_0x0c0_Inverter_Discharge(dbc, 0.0);
  encode_can_0x0c0_Direction_Command(dbc, spin_forward);
  encode_can_0x0c0_Inverter_Enable(dbc, inverter_enable);

  can_message out_msg;
  out_msg.id = CAN_ID_M192_COMMAND_MESSAGE;
  out_msg.length =
      pack_message(dbc, CAN_ID_M192_COMMAND_MESSAGE, &out_msg.buf.val);

  can->send_controller_message(out_msg);
}

void Inverter::send_clear_faults() {
  encode_can_0x0c1_D1_Parameter_Address_Command(dbc, 20);
  encode_can_0x0c1_D2_Read_Write_Command(dbc, 1);
  encode_can_0x0c1_D3_Data_Command(dbc, 0);

  can_message out_msg;
  out_msg.length =
      pack_message(dbc, CAN_ID_M192_COMMAND_MESSAGE, &out_msg.buf.val);

  can->send_controller_message(out_msg);
}

void Inverter::command_torque(double torque_request) {
  double torque_target = torque_request;

  // TODO: Get rid of these arduino calls
  // We apply an exponential decay function to the torque_request if we go over
  // whatever power_limit is set, this could probably be optimized
  // if ((bus_voltage * bus_current) >= power_limit && !over_power) {
  //   over_power_event = millis();
  //   over_power = true;
  // } else if ((bus_voltage * bus_current) < power_limit) {
  //   over_power = false;
  // }

  // if (over_power) {
  // torque_target =
  //     torque_target *
  //     pow(2.71828182846, (over_power_decay_factor *
  //                         ((millis() - over_power_event) / 1000.0)));
  //
  //   torque_target = 0;
  // }

  encode_can_0x0c0_Torque_Command(dbc, torque_target);
  encode_can_0x0c0_Torque_Limit_Command(dbc, torque_limit);
  encode_can_0x0c0_Speed_Command(dbc, 0.0);
  encode_can_0x0c0_Speed_Mode_Enable(dbc, 0.0);
  encode_can_0x0c0_Inverter_Discharge(dbc, 0.0);
  encode_can_0x0c0_Direction_Command(dbc, spin_forward);
  encode_can_0x0c0_Inverter_Enable(dbc, inverter_enable);

  can_message out_msg;
  out_msg.id = CAN_ID_M192_COMMAND_MESSAGE;
  out_msg.length =
      pack_message(dbc, CAN_ID_M192_COMMAND_MESSAGE, &out_msg.buf.val);

  can->send_controller_message(out_msg);
}

void Inverter::command_speed(int16_t speed_request) {
  encode_can_0x0c0_Torque_Command(dbc, 0.0);
  encode_can_0x0c0_Torque_Limit_Command(dbc, torque_limit);
  encode_can_0x0c0_Speed_Command(dbc, speed_request);
  encode_can_0x0c0_Speed_Mode_Enable(dbc, speed_mode);
  encode_can_0x0c0_Inverter_Discharge(dbc, inverter_discharge);
  encode_can_0x0c0_Direction_Command(dbc, spin_forward);
  encode_can_0x0c0_Inverter_Enable(dbc, inverter_enable);

  can_message out_msg;
  out_msg.id = CAN_ID_M192_COMMAND_MESSAGE;
  out_msg.length =
      pack_message(dbc, CAN_ID_M192_COMMAND_MESSAGE, &out_msg.buf.val);

  can->send_controller_message(out_msg);
}
