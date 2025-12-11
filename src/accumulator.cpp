#include "accumulator.hpp"

Accumulator::Accumulator(std::array<parameter, 25> *params,
                         VehicleData *vehicle_data) {
  this->params = params;
  this->vehicle_data = vehicle_data;
}

void Accumulator::calculate_energy_consumed_wh(uint32_t time_msec) {
  if (vehicle_data == nullptr) {
    return;
  }

  auto &acc_data = vehicle_data->accumulator;
  uint32_t time_elaped_msec =
      time_msec - acc_data.last_energy_calc_timestamp_ms;

  acc_data.consumed_power_wh += ((double(time_elaped_msec) / 1000) / 3600) *
                                (acc_data.pack_current * acc_data.pack_voltage);

  acc_data.last_energy_calc_timestamp_ms = time_msec;
}