#pragma once

#include "data.hpp"
#include "parameters.hpp"
#include <adc.hpp>

class Pedals {

private:
  // Pots
  adc apps1 = adc(mcp, ADC_CS, ADC_ACCEL_1_CHANNEL, 0.980483996877);
  adc apps2 = adc(mcp, ADC_CS, ADC_ACCEL_2_CHANNEL, 0.980483996877);
  adc bse = adc(mcp, ADC_CS, ADC_BSE_CHANNEL, 0.980483996877);
  adc vsense_bspd = adc(avr, BSPD_SENSE);

  VehicleData *vehicle_data;

  float brake_ratio;
  uint16_t brake_start;
  uint16_t bse_low_fault;
  uint16_t bse_high_fault;

  uint16_t apps_low_fault;

  float apps1_ratio;
  uint16_t apps1_start;

  float apps2_ratio;
  uint16_t apps2_start;

public:
  enum class PedalFaults {
    APPS_FAULT = 0,
    BSE_FAULT = 1,
    APPS_BSE_FAULT = 2,
    APPS_FAULT_AND_BSE_FAULT = 3,
    APPS_FAULT_AND_APPS_BSE_FAULT = 4,
    BSE_FAULT_AND_APPS_BSE_FAULT = 5,
    APPS_FAULT_AND_BSE_FAULT_AND_APPS_BSE_FAULT = 6,
    NO_FAULT = 7,
    UNKNOWN_FAULT = 8,
  };
  Pedals(uint16_t bse_low_fault, uint16_t brake_start, uint16_t brake_end,
         uint16_t bse_high_fault, uint16_t apps_low_fault, uint16_t apps1_start,
         uint16_t apps1_end, uint16_t apps2_start, uint16_t apps2_end,
         VehicleData *vehicle_data) {

    this->brake_ratio = 1 / float(brake_end - brake_start);
    this->brake_start = brake_start;
    this->bse_low_fault = bse_low_fault;
    this->bse_high_fault = bse_high_fault;

    this->apps_low_fault = apps_low_fault;

    apps1_ratio = 1 / float(apps1_end - apps1_start);
    this->apps1_start = apps1_start;

    apps2_ratio = 1 / float(apps2_end - apps2_start);
    this->apps2_start = apps2_start;

    this->vehicle_data = vehicle_data;
  };

  // TODO: Make the release and apps_bse values configurable
  void update_travel(uint16_t raw_apps1, uint16_t raw_apps2,
                     uint16_t raw_brake);

  void check_hard_faults();

  void pedal_main_loop();
  void pedal_200hz_loop();
  void pedal_10hz_loop();
};