#pragma once

#include <stdint.h>

class PEDALS {
private:
  float brake_ratio;
  uint16_t brake_start;
  uint16_t bse_low_fault;
  uint16_t bse_high_fault;

  float apps1_ratio;
  uint16_t apps1_start;

  float apps2_ratio;
  uint16_t apps2_start;

  double brake_travel;
  double apps1_travel;
  double apps2_travel;
  double travel;

  // EV.4.7
  bool bse_fault;      // BSE is reading too high or too low
  bool apps_fault;     // Not reading within 10% of each other
  bool apps_bse_fault; // Screenshot (too much brake and gas (before BSPD))

public:
  PEDALS(uint16_t bse_low_fault, uint16_t brake_start, uint16_t brake_end,
         uint16_t bse_high_fault, uint16_t apps1_start, uint16_t apps1_end,
         uint16_t apps2_start, uint16_t apps2_end) {

    this->brake_ratio = 1 / float(brake_end - brake_start);
    this->brake_start = brake_start;
    this->bse_low_fault = bse_low_fault;
    this->bse_high_fault = bse_high_fault;

    apps1_ratio = 1 / float(apps1_end - apps1_start);
    this->apps1_start = apps1_start;

    apps2_ratio = 1 / float(apps2_end - apps2_start);
    this->apps2_start = apps2_start;
  };

  // TODO: Add additional pedal maps with diffrent curves?
  double get_torque_request(double apps_travel, double max_torque) {
    return apps_travel * max_torque;
  };

  // TODO: Make the release and apps_bse values confgiurable
  void update_travel(uint16_t raw_apps1, uint16_t raw_apps2,
                     uint16_t raw_brake) {
    // Get the pedal percentage in its throw from 0 to 1
    apps1_travel = (raw_apps1 - apps1_start) * apps1_ratio;
    apps2_travel = (raw_apps2 - apps2_start) * apps2_ratio;

    // T.4.3.4
    // BSE check to make sure its not shorting
    if (raw_brake > bse_high_fault || raw_brake < bse_low_fault) {
      bse_fault = true;
    } else {
      bse_fault = false;
      brake_travel = (raw_brake - brake_start) * brake_ratio;
    }

    // Reset if the pedals are released
    if (this->apps1_travel < 0.1 && this->apps2_travel < 0.1) {
      apps_fault = false;
      apps_bse_fault = false;
    }

    // T.4.2.4
    // Check that there is no apps related faults
    if (apps_fault == false && bse_fault == false && apps_bse_fault == false) {

      // Check that the pedals are reading within 10%
      if ((fabs(apps1_travel - apps2_travel) < 0.1)) {

        // Check that the driver isn't using both pedals at once
        if ((apps1_travel * apps2_travel) / 2 > 0.3 && brake_travel > 0.3) {
          travel = (apps1_travel * apps2_travel) / 2;
        } else {
          apps_bse_fault = true;
          travel = 0;
        }
      } else {
        apps_fault = true;
        travel = 0;
      }
    } else {
      travel = 0;
    }
  }

  inline bool get_bse_fault_ok_low() { return bse_fault; }
  inline bool get_apps_fault_ok_low() { return apps_fault; }
  inline bool get_apps_bse_fault_ok_low() { return apps_bse_fault; }

  inline double get_apps1_travel() { return apps1_travel; }
  inline double get_apps2_travel() { return apps2_travel; }
  inline double get_brake_travel() { return brake_travel; }
  inline double get_travel() { return travel; }
};
