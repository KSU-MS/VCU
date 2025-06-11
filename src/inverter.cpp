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

void Inverter::set_current_limits(uint16_t charge_limit,
                                  uint16_t discharge_limit) {
  encode_can_0x202_BMS_Max_Charge_Current(dbc, charge_limit);
  encode_can_0x202_BMS_Max_Discharge_Current(dbc, discharge_limit);

  can_message out_msg;
  out_msg.id = CAN_ID_BMS_CURRENT_LIMIT;
  out_msg.length =
      pack_message(dbc, CAN_ID_BMS_CURRENT_LIMIT, &out_msg.buf.val);

  can->send_controller_message(out_msg);
}

void Inverter::update_bus_current(uint64_t msg_in, uint8_t length) {
  unpack_message(dbc, CAN_ID_M166_CURRENT_INFO, msg_in, length, 0);

  decode_can_0x0a6_INV_DC_Bus_Current(dbc, &bus_current);
}

void Inverter::update_bus_voltage(uint64_t msg_in, uint8_t length) {
  unpack_message(dbc, CAN_ID_M167_VOLTAGE_INFO, msg_in, length, 0);

  decode_can_0x0a7_INV_DC_Bus_Voltage(dbc, &bus_voltage);
}

void Inverter::ping() {
  encode_can_0x0c0_VCU_INV_Torque_Command(dbc, 0.0);
  encode_can_0x0c0_VCU_INV_Torque_Limit_Command(dbc, 0.0);
  encode_can_0x0c0_VCU_INV_Speed_Command(dbc, 0);
  encode_can_0x0c0_VCU_INV_Speed_Mode_Enable(dbc, 0);
  encode_can_0x0c0_VCU_INV_Direction_Command(dbc, spin_forward);
  encode_can_0x0c0_VCU_INV_Inverter_Discharge(dbc, inverter_discharge);
  encode_can_0x0c0_VCU_INV_Inverter_Enable(dbc, inverter_enable);

  can_message out_msg;
  out_msg.id = CAN_ID_M192_COMMAND_MESSAGE;
  out_msg.length =
      pack_message(dbc, CAN_ID_M192_COMMAND_MESSAGE, &out_msg.buf.val);

  can->send_controller_message(out_msg);
}

void Inverter::send_clear_faults() {
  encode_can_0x0c1_VCU_INV_Parameter_Address(dbc, 20);
  encode_can_0x0c1_VCU_INV_Parameter_RW_Command(dbc, 1);
  encode_can_0x0c1_VCU_INV_Parameter_Data(dbc, 0);

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
  //  over_power_event_epoch = millis();
  //  over_power = true;
  //} else if ((bus_voltage * bus_current) < power_limit) {
  //  over_power = false;
  //}

  // if (over_power) {
  //   torque_target =
  //       torque_target *
  //       pow(2.71828182846, (over_power_decay_factor *
  //                           ((millis() - over_power_event_epoch) / 1000.0)));
  //
  //   torque_target = 0;
  // }

  // Clamp to make sure this can only de-rate torque, just in case
  // if (torque_target > torque_request)
  //  torque_target = torque_request;

  encode_can_0x0c0_VCU_INV_Torque_Command(dbc, torque_target);
  encode_can_0x0c0_VCU_INV_Torque_Limit_Command(dbc, torque_limit);
  encode_can_0x0c0_VCU_INV_Speed_Command(dbc, 0);
  encode_can_0x0c0_VCU_INV_Speed_Mode_Enable(dbc, 0);
  encode_can_0x0c0_VCU_INV_Direction_Command(dbc, spin_forward);
  encode_can_0x0c0_VCU_INV_Inverter_Discharge(dbc, inverter_discharge);
  encode_can_0x0c0_VCU_INV_Inverter_Enable(dbc, inverter_enable);

  can_message out_msg;
  out_msg.id = CAN_ID_M192_COMMAND_MESSAGE;
  out_msg.length =
      pack_message(dbc, CAN_ID_M192_COMMAND_MESSAGE, &out_msg.buf.val);

  can->send_controller_message(out_msg);
}

void Inverter::command_speed(int16_t speed_request) {
  encode_can_0x0c0_VCU_INV_Torque_Command(dbc, 0.0);
  encode_can_0x0c0_VCU_INV_Torque_Limit_Command(dbc, torque_limit);
  encode_can_0x0c0_VCU_INV_Speed_Command(dbc, speed_request);
  encode_can_0x0c0_VCU_INV_Speed_Mode_Enable(dbc, speed_mode);
  encode_can_0x0c0_VCU_INV_Direction_Command(dbc, spin_forward);
  encode_can_0x0c0_VCU_INV_Inverter_Discharge(dbc, inverter_discharge);
  encode_can_0x0c0_VCU_INV_Inverter_Enable(dbc, inverter_enable);

  can_message out_msg;
  out_msg.id = CAN_ID_M192_COMMAND_MESSAGE;
  out_msg.length =
      pack_message(dbc, CAN_ID_M192_COMMAND_MESSAGE, &out_msg.buf.val);

  can->send_controller_message(out_msg);
}
