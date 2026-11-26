#include "sim_car.hpp"

namespace sim {

void Car::send(Bus &b, uint32_t id) {
  can_message msg;
  msg.id = id;
  msg.length = pack_message(&other_boards, id, &msg.buf.val);
  b.inject(msg);
}

void Car::acu_status(bool imd_ok, bool bms_ok) {
  encode_can_0x06a_acu_imd_relay_state(&other_boards, imd_ok);
  encode_can_0x06a_acu_bms_relay_state(&other_boards, bms_ok);
  send(acc(), CAN_ID_ACU_SHUTDOWN_STATUS);
}

void Car::precharge_state(uint8_t state) {
  encode_can_0x069_precharge_state(&other_boards, state);
  send(acc(), CAN_ID_PRECHARGE_STATUS);
}

void Car::pack_power(double volts, double amps) {
  encode_can_0x6b1_Pack_Summed_Voltage(&other_boards, volts);
  encode_can_0x6b1_Pack_Current(&other_boards, amps);
  send(acc(), CAN_ID_MSGID_0X6B1);
}

void Car::inverter_bus_voltage(double volts) {
  encode_can_0x0a7_INV_DC_Bus_Voltage(&other_boards, volts);
  send(inv(), CAN_ID_M167_VOLTAGE_INFO);
}

void Car::inverter_motor_rpm(int16_t rpm) {
  encode_can_0x0a5_INV_Motor_Speed(&other_boards, rpm);
  send(inv(), CAN_ID_M165_MOTOR_POSITION_INFO);
}

void Car::dash_rtd_button(bool pressed) {
  encode_can_0x0eb_dash_button5status(&other_boards, pressed);
  send(inv(), CAN_ID_DASH_BUTTONS);
}

void Car::set_parameter(uint8_t target, uint32_t value) {
  encode_can_0x0c9_vcu_target_parameter(&other_boards, target);
  encode_can_0x0c9_vcu_parameter_value(&other_boards, value);
  send(inv(), CAN_ID_VCU_SET_PARAMETER);
}

void Car::drive_to_rtd() {
  run_for(10); // STARTUP -> TS_DISABLED
  make_ts_healthy();
  run_for(10); // -> TS_ENERGIZED
  set_brake(0.5);
  dash_rtd_button(true);
  run_until(READY_TO_DRIVE, 3000); // -> TS_ENABLED, buzzer, -> READY_TO_DRIVE
  dash_rtd_button(false);
  set_brake(0);
  run_for(10);
}

bool Car::last_inverter_command(InverterCommand *out) {
  const can_message *m = inv().last_sent(CAN_ID_M192_COMMAND_MESSAGE);
  if (!m)
    return false;

  can_obj_car_h_t rx{};
  unpack_message(&rx, m->id, m->buf.val, m->length, 0);

  uint8_t enable, direction;
  decode_can_0x0c0_VCU_INV_Torque_Command(&rx, &out->torque_nm);
  decode_can_0x0c0_VCU_INV_Torque_Limit_Command(&rx, &out->torque_limit_nm);
  decode_can_0x0c0_VCU_INV_Inverter_Enable(&rx, &enable);
  decode_can_0x0c0_VCU_INV_Direction_Command(&rx, &direction);
  out->enable = enable;
  out->direction = direction;
  return true;
}

int Car::last_reported_state() {
  const can_message *m = daq().last_sent(CAN_ID_VCU_STATUS);
  if (!m)
    return -1;

  can_obj_car_h_t rx{};
  unpack_message(&rx, m->id, m->buf.val, m->length, 0);

  uint8_t state;
  decode_can_0x0c3_vcu_statemachine_state(&rx, &state);
  return state;
}

int Car::last_discharge_limit() {
  const can_message *m = inv().last_sent(CAN_ID_BMS_CURRENT_LIMIT);
  if (!m)
    return -1;

  can_obj_car_h_t rx{};
  unpack_message(&rx, m->id, m->buf.val, m->length, 0);

  uint16_t limit;
  decode_can_0x202_BMS_Max_Discharge_Current(&rx, &limit);
  return limit;
}

} // namespace sim
