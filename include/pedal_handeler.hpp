#pragma once

#include <can_tools.hpp>
#include <car.h>
#include <cmath>
#include <stdint.h>

#include "adc.hpp"
#include "data.hpp"
#include "parameters.hpp"

class Pedals {

private:
  // Pots
  adc apps1 = adc(mcp, ADC_CS, ADC_ACCEL_1_CHANNEL, 0.980483996877);
  adc apps2 = adc(mcp, ADC_CS, ADC_ACCEL_2_CHANNEL, 0.980483996877);
  adc bse = adc(mcp, ADC_CS, ADC_BSE_CHANNEL, 0.980483996877);
  adc vsense_bspd = adc(avr, BSPD_SENSE);

  VehicleData *vehicle_data = nullptr;

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

  // TODO: Add additional pedal maps with diffrent curves?
  inline double get_torque_request(double throttle_travel, double max_torque) {
    return throttle_travel * max_torque;
  };

  // TODO: Make the release and apps_bse values configurable
  void update_travel(uint16_t raw_apps1, uint16_t raw_apps2,
                     uint16_t raw_brake);

  inline bool get_bse_fault_ok_low() const {
    return vehicle_data ? vehicle_data->pedals.bse_fault : false;
  }
  inline bool get_apps_fault_ok_low() const {
    return vehicle_data ? vehicle_data->pedals.apps_fault : false;
  }
  inline bool get_apps_bse_fault_ok_low() const {
    return vehicle_data ? vehicle_data->pedals.apps_bse_fault : false;
  }

  inline uint16_t get_apps1_raw() const {
    return vehicle_data ? vehicle_data->pedals.raw_apps1 : 0;
  }
  inline uint16_t get_apps2_raw() const {
    return vehicle_data ? vehicle_data->pedals.raw_apps2 : 0;
  }
  inline uint16_t get_brake_raw() const {
    return vehicle_data ? vehicle_data->pedals.raw_brake : 0;
  }
  inline double get_apps1_travel() const {
    return vehicle_data ? vehicle_data->pedals.apps1_travel : 0.0;
  }
  inline double get_apps2_travel() const {
    return vehicle_data ? vehicle_data->pedals.apps2_travel : 0.0;
  }
  inline double get_brake_travel() const {
    return vehicle_data ? vehicle_data->pedals.brake_travel : 0.0;
  }
  inline double get_throttle_travel() const {
    return vehicle_data ? vehicle_data->pedals.throttle_travel : 0.0;
  }

  void pedal_200hz_loop();

  void pedal_10hz_loop();

  void check_hard_faults();
};

// Yes, this works, but the "combined faults" values (e.g.,
// APPS_FAULT_AND_BSE_FAULT, etc.) are not actually used by the logic of
// get_pedal_faults(). Instead, it just bitwise-or's the base faults together
// and returns the combined bits as a PedalFaults value.

// In C++ enum/bitflags, this is fine if you are just checking (faults &
// SOME_FAULT), but the "combined" names are unused and not necessary unless you
// want to explicitly check for exactly those combinations. The current
// get_pedal_faults() function is valid and will return a bit pattern with each
// fault it sees set.

// If you want to clean it up a bit, you can drop the combined names, or just
// clarify your intended usage/documenting below.

enum PedalFaults {
  NO_FAULT = 0,
  APPS_FAULT = 1 << 1,
  BSE_FAULT = 1 << 2,
  APPS_BSE_FAULT = 1 << 3,
  // The following are not needed for bitmask use; you can determine
  // combinations by bitwise-or
  APPS_FAULT_AND_BSE_FAULT = APPS_FAULT | BSE_FAULT,
  APPS_FAULT_AND_APPS_BSE_FAULT = APPS_FAULT | APPS_BSE_FAULT,
  BSE_FAULT_AND_APPS_BSE_FAULT = BSE_FAULT | APPS_BSE_FAULT,
  APPS_FAULT_AND_BSE_FAULT_AND_APPS_BSE_FAULT =
      APPS_FAULT | BSE_FAULT | APPS_BSE_FAULT,
};

static PedalFaults get_pedal_faults(bool apps_fault, bool bse_fault,
                                    bool apps_bse_fault) {
  int faults = NO_FAULT;
  if (apps_fault) {
    faults |= APPS_FAULT;
  }
  if (bse_fault) {
    faults |= BSE_FAULT;
  }
  if (apps_bse_fault) {
    faults |= APPS_BSE_FAULT;
  }
  return static_cast<PedalFaults>(faults);
}