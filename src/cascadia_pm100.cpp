#include "cascadia_pm100.hpp"
#include "parameters.hpp"

#if VEHICLE_HAS_CASCADIA

CascadiaPM100::CascadiaPM100(bool spin_direction, canMan *can, canMan *daq,
                             can_obj_car_h_t *dbc,
                             float over_power_decay_factor)
    : Inverter(spin_direction, GEAR_RATIO),
      over_power_decay_factor(over_power_decay_factor), can(can), daq(daq),
      dbc(dbc) {
  ping();
}

uint8_t CascadiaPM100::on_frame(const can_message &msg) {
  switch (msg.id) {
  case CAN_ID_M165_MOTOR_POSITION_INFO:
    unpack_message(dbc, msg.id, msg.buf.val, msg.length, 0);
    decode_can_0x0a5_INV_Motor_Speed(dbc, &motor_rpm);
    return MOTOR_SPEED;

  case CAN_ID_M166_CURRENT_INFO: {
    unpack_message(dbc, msg.id, msg.buf.val, msg.length, 0);
    double amps;
    decode_can_0x0a6_INV_DC_Bus_Current(dbc, &amps);
    new_dc_current(amps);
    return DC_CURRENT;
  }

  case CAN_ID_M167_VOLTAGE_INFO:
    unpack_message(dbc, msg.id, msg.buf.val, msg.length, 0);
    decode_can_0x0a7_INV_DC_Bus_Voltage(dbc, &bus_voltage);
    return DC_VOLTAGE;

  case CAN_ID_M172_TORQUE_AND_TIMER_INFO:
    unpack_message(dbc, msg.id, msg.buf.val, msg.length, 0);
    decode_can_0x0ac_INV_Torque_Feedback(dbc, &motor_torque);
    return MOTOR_TORQUE;

  default:
    return NOT_MINE;
  }
}

void CascadiaPM100::set_current_limits(uint16_t charge_limit,
                                  uint16_t discharge_limit) {
  encode_can_0x202_BMS_Max_Charge_Current(dbc, charge_limit);
  encode_can_0x202_BMS_Max_Discharge_Current(dbc, discharge_limit);

  can_message out_msg;
  out_msg.id = CAN_ID_BMS_CURRENT_LIMIT;
  out_msg.length =
      pack_message(dbc, CAN_ID_BMS_CURRENT_LIMIT, &out_msg.buf.val);

  can->send_controller_message(out_msg);
  daq->send_controller_message(out_msg);
}

void CascadiaPM100::ping() {
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
  daq->send_controller_message(out_msg);
}

void CascadiaPM100::send_clear_faults() {
  encode_can_0x0c1_VCU_INV_Parameter_Address(dbc, 0x0014);
  encode_can_0x0c1_VCU_INV_Parameter_RW_Command(dbc, 1);
  encode_can_0x0c1_VCU_INV_Parameter_Data(dbc, 0);

  can_message out_msg;
  out_msg.id = CAN_ID_M193_READ_WRITE_PARAM_COMMAND;
  out_msg.length = pack_message(dbc, out_msg.id, &out_msg.buf.val);

  can->send_controller_message(out_msg);
  daq->send_controller_message(out_msg);
}

void CascadiaPM100::send_torque(double torque_target) {
  encode_can_0x0c0_VCU_INV_Torque_Command(dbc, torque_target);
  encode_can_0x0c0_VCU_INV_Torque_Limit_Command(dbc, torque_limit_nm);
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
  daq->send_controller_message(out_msg);
}

void CascadiaPM100::command_speed(int16_t speed_request) {
  encode_can_0x0c0_VCU_INV_Torque_Command(dbc, 0.0);
  encode_can_0x0c0_VCU_INV_Torque_Limit_Command(dbc, torque_limit_nm);
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
  daq->send_controller_message(out_msg);
}

#endif // VEHICLE_HAS_CASCADIA
