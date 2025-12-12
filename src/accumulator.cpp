#include "accumulator.hpp"
#include "core_pins.h"

Accumulator::Accumulator(std::array<parameter, 25> *params,
                         VehicleData *vehicle_data) {
  this->params = params;
  this->vehicle_data = vehicle_data;
}

void Accumulator::calculate_energy_consumed_wh(uint32_t time_msec) {
  if (vehicle_data == nullptr) {
    return;
  }

  uint32_t time_elaped_msec =
      time_msec - vehicle_data->accumulator.last_energy_calc_timestamp_ms;

  vehicle_data->accumulator.consumed_power_wh +=
      ((double(time_elaped_msec) / 1000) / 3600) *
      (vehicle_data->accumulator.pack_current *
       vehicle_data->accumulator.pack_voltage);

  vehicle_data->accumulator.last_energy_calc_timestamp_ms = time_msec;
}

void Accumulator::accumulator_main_loop() {
  if (vehicle_data == nullptr) {
    return;
  }

  this->calculate_energy_consumed_wh(millis());
}

void Accumulator::accumulator_200hz_loop() {

}

void Accumulator::accumulator_10hz_loop() {

}