#include "accumulator.hpp"
#include "car.h"

Accumulator::Accumulator(can_obj_car_h_t *dbc, canMan *can,
                         bool (*can_message_check)()) {
  this->dbc = dbc;
  this->can = can;
  this->can_message_check = can_message_check;
}

void Accumulator::update_acu_status(uint64_t msg, uint8_t length) {
  unpack_message(dbc, CAN_ID_ACU_SHUTDOWN_STATUS, msg, length, 0);

  uint8_t imd_relay_val, bms_relay_val;

  decode_can_0x258_acu_imd_relay_state(dbc, &imd_relay_val);
  decode_can_0x258_acu_bms_relay_state(dbc, &bms_relay_val);

  imd_ok_hs = bool(imd_relay_val);
  bms_ok_hs = bool(bms_relay_val);
}

void Accumulator::update_precharge_status(uint64_t msg, uint8_t length) {
  unpack_message(dbc, CAN_ID_PRECHARGE_STATUS, msg, length, 0);

  decode_can_0x069_precharge_state(dbc, &precharge_state);
}

void Accumulator::update_pack_power(uint64_t msg, uint8_t length) {
  unpack_message(dbc, CAN_ID_MSGID_0X6B1, msg, length, 0);

  decode_can_0x6b1_Pack_Summed_Voltage(dbc, &pack_current);
  decode_can_0x6b1_Pack_Current(dbc, &pack_current);
}

void Accumulator::calculate_energy_consumed_wh(uint32_t time_msec) {
  uint32_t time_elaped_msec = time_msec - time_last_msec;

  consumed_power_wh += ((double(time_elaped_msec) / 1000) / 3600) *
                       (pack_current * pack_voltage);
}
