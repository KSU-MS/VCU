#pragma once

#include <array>

#include "data.hpp"
#include <parameters.hpp>

class Accumulator {
private:
  std::array<parameter, 25> *params;

  VehicleData *vehicle_data;

public:
  Accumulator(std::array<parameter, 25> *params, VehicleData *vehicle_data);

  inline uint8_t get_precharge_state() {
    return vehicle_data ? vehicle_data->accumulator.precharge_state : 0;
  }
  inline bool get_imd_ok_hs() {
    return vehicle_data ? vehicle_data->accumulator.imd_ok_hs : false;
  }
  inline bool get_bms_ok_hs() {
    return vehicle_data ? vehicle_data->accumulator.bms_ok_hs : false;
  }

  inline double get_pack_voltage() {
    return vehicle_data ? vehicle_data->accumulator.pack_voltage : 0.0;
  }
  inline double get_pack_current() {
    return vehicle_data ? vehicle_data->accumulator.pack_current : 0.0;
  }
  inline double get_consumed_wh() {
    return vehicle_data ? vehicle_data->accumulator.consumed_power_wh : 0.0;
  }

  void calculate_energy_consumed_wh(uint32_t time_msec);
};
