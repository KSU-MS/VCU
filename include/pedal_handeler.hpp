#pragma once

#ifndef pedal_handeler_hpp
#define pedal_handeler_hpp

#include <cmath.h>
#include <stdint.h>

class pedals {
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

  // EV.4.7
  bool bse_fault;      // BSE is reading too high or too low
  bool apps_fault;     // Not reading within 10% of each other
  bool apps_bse_fault; // Screenshot (too much brake and gas (before BSPD))

public:
  pedals(uint16_t brake_start, uint16_t brake_end, uint16_t bse_low_fault,
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

  inline double get_apps_travel(uint16_t raw_apps1, uint16_t raw_apps2) {
    // Get the pedal percentage in its throw
    apps1_travel = (raw_apps1 - apps1_start) * apps1_ratio;
    apps2_travel = (raw_apps2 - apps2_start) * apps2_ratio;

    // T.4.2.4
    // check that the pedals are reading within 10% of each other, and that
    // there are no faults active
    if ((fabs(this->apps1_travel - this->apps2_travel) < 0.1) &&
        apps_fault == false && bse_fault == false && apps_bse_fault == false) {
      apps_fault = false;

      // We average the two here to give a more consistent/linear pedal feel
      return (apps1_travel * apps2_travel) / 2;
    }

    // Reset if pedals are released
    else if (this->apps1_travel < 0.1 && this->apps2_travel < 0.1) {
      apps_fault = false;
      return 0;
    }

    // Set fault high if the pedals are not within 10% or released
    else {
      apps_fault = true;
      return 0; // return 0
    }
  }

  // T.4.3.4
  // BSE check to make sure its not shorting
  double get_bse1_travel(uint16_t raw) {
    if (raw > bse_high_fault || raw < bse_low_fault) {
      bse_fault = true;
      return 0;
    } else {
      bse_fault = false;
      return (raw - brake_start) * this->brake_ratio;
    }
  }

  bool apps_is_chill(double app1, double app2) {};

  bool apps_bse_is_chill();

  inline bool get_bse_fault_ok_low() { return bse_fault; }
  inline bool get_apps_fault_ok_low() { return apps_fault; }
  inline bool get_apps_bse_fault_ok_low() { return apps_bse_fault; }
};

#endif
