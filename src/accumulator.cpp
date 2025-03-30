#include "accumulator.hpp"

ACCUMULATOR::ACCUMULATOR(can_obj_car_h_t *dbc, canMan *can,
                         bool (*can_message_check)()) {
  this->dbc = dbc;
  this->can = can;
  this->can_message_check = can_message_check;
}

void ACCUMULATOR::send_bms_current_limit() {
  encode_can_0x202_D2_Max_Charge_Current(dbc, charge_limit);
  encode_can_0x202_D1_Max_Discharge_Current(dbc, discharge_limit);

  can_message out_msg;
  out_msg.id = CAN_ID_BMS_CURRENT_LIMIT;
  out_msg.length =
      pack_message(dbc, CAN_ID_BMS_CURRENT_LIMIT, &out_msg.buf.val);

  can->send_controller_message(out_msg);
}

void ACCUMULATOR::update_acu_status(uint64_t msg, uint8_t length) {
  unpack_message(dbc, CAN_ID_ACU_SHUTDOWN_STATUS, msg, length, 0);

  uint8_t imd_gpio_val, bms_gpio_val;

  decode_can_0x258_acu_imd_gpio_state(dbc, &imd_gpio_val);
  decode_can_0x258_acu_bms_gpio_state(dbc, &bms_gpio_val);

  imd_ok_hs = bool(imd_gpio_val);
  bms_ok_hs = bool(bms_gpio_val);
}

void ACCUMULATOR::update_precharge_status(uint64_t msg, uint8_t length) {
  unpack_message(dbc, CAN_ID_PRECHARGE_STATUS, msg, length, 0);

  decode_can_0x069_precharge_state(dbc, &precharge_state);
}
