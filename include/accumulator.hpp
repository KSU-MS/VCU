#pragma once

#include <array>
#include <can_tools.hpp>
#include <car.h>
#include <parameters.hpp>

class Accumulator {
private:
  uint32_t time_last_msec = 0;

  std::array<parameter, 25> *params;

  canMan *can;
  can_obj_car_h_t *dbc;

  uint8_t precharge_state = 0;

  bool imd_ok_hs = false;
  bool bms_ok_hs = false;

  bool (*can_message_check)();

  double pack_voltage = 0;
  double pack_current = 0;

  double consumed_power_wh = 0;

public:
  Accumulator(std::array<parameter, 25> *params);

  inline uint8_t get_precharge_state() { return this->precharge_state; }
  inline bool get_imd_ok_hs() { return imd_ok_hs; }
  inline bool get_bms_ok_hs() { return bms_ok_hs; }

  void update_acu_status(uint64_t msg, uint8_t length);
  void update_precharge_status(uint64_t msg, uint8_t length);
  void update_pack_power(uint64_t msg, uint8_t length);

  inline double get_pack_voltage() { return pack_voltage; }
  inline double get_pack_current() { return pack_current; }
  inline double get_consumed_wh() { return consumed_power_wh; }

  void calculate_energy_consumed_wh(uint32_t time_msec);
};
