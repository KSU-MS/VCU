#pragma once

#include <can_tools.hpp>
#include <car.h>

class ACCUMULATOR {
private:
  canMan *can;
  can_obj_car_h_t *dbc;

  uint16_t charge_limit;
  uint16_t discharge_limit;

  uint8_t precharge_state = 0;

  bool imd_ok_hs = false;
  bool bms_ok_hs = false;

  bool (*can_message_check)();

public:
  ACCUMULATOR(can_obj_car_h_t *dbc, canMan *acc_can,
              bool (*can_message_check)());

  // HACK: If you want the VCU to try to enter RTD regardless of acc state...
  inline uint16_t get_charge_limit() { return this->charge_limit; }
  inline uint16_t get_discharge_limit() { return this->discharge_limit; }
  inline uint8_t get_precharge_state() { return this->precharge_state; }
  inline bool get_imd_ok_hs() { return this->imd_ok_hs; }
  inline bool get_bms_ok_hs() { return this->bms_ok_hs; }

  void set_charge_limit(uint16_t limit) { this->charge_limit = limit; };
  void set_discharge_limit(uint16_t limit) { this->discharge_limit = limit; };

  void send_bms_current_limit();
  void update_acu_status(uint64_t msg, uint8_t length);
  void update_precharge_status(uint64_t msg, uint8_t length);
};
