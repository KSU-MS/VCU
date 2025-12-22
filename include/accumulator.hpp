#pragma once

#include <array>

#include "data.hpp"
#include "parameters.hpp"

class Accumulator {
private:
  std::array<Parameter, 25> *params;
  VehicleData *vehicle_data = nullptr;

public:
  Accumulator(std::array<Parameter, 25> *params, VehicleData *vehicle_data);

  void calculate_energy_consumed_wh(uint32_t time_msec);

  void accumulator_main_loop();
  void accumulator_200hz_loop();
  void accumulator_10hz_loop();
};
