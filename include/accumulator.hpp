#pragma once

#include <can_tools.hpp>
#include <car.h>

class Accumulator {
private:
  canMan *can;
  can_obj_car_h_t *dbc;

  uint8_t precharge_state = 0;

  bool imd_ok_hs = false;
  bool bms_ok_hs = false;

  bool (*can_message_check)();

  double pack_voltage = 0;
  double pack_current = 0;

public:
  Accumulator(can_obj_car_h_t *dbc, canMan *acc_can,
              bool (*can_message_check)());

  inline uint8_t get_precharge_state() { return this->precharge_state; }
  inline bool get_imd_ok_hs() { return imd_ok_hs; }
  inline bool get_bms_ok_hs() { return bms_ok_hs; }

  void update_acu_status(uint64_t msg, uint8_t length);
  void update_precharge_status(uint64_t msg, uint8_t length);

  void update_instant_power(uint64_t msg, uint8_t length);

  void calculate_effecincy(void);
};
