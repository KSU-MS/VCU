#pragma once

#include <cmath>
#include <stdint.h>

class Pedals {
private:
  uint16_t raw_apps1 = 0;
  uint16_t raw_apps2 = 0;
  uint16_t raw_brake = 0;
  uint16_t raw_steering = 0;

  float brake_ratio;
  uint16_t brake_start;
  uint16_t bse_low_fault;
  uint16_t bse_high_fault;

  uint16_t apps_low_fault;

  float apps1_ratio;
  uint16_t apps1_start;
  uint16_t apps1_high_fault;

  float apps2_ratio;
  uint16_t apps2_start;
  uint16_t apps2_high_fault;

  double brake_travel = 0;
  double apps1_travel = 0;
  double apps2_travel = 0;
  double travel = 0;

  // EV.4.7
  bool bse_fault = false;      // BSE is reading too high or too low
  bool apps_fault = false;     // Latched: an APPS broken, or implausible >100 ms
  bool apps_bse_fault = false; // Screenshot (too much brake and gas (before BSPD))

  // The two APPS disagreeing right now, and since when
  bool apps_disagree = false;
  uint32_t disagree_since = 0;

public:
  // T.4.2.4: more than 10 points of travel apart is an implausibility.
  // T.4.2.5: if it lasts more than 100 ms, power to the motor must be cut.
  static constexpr double APPS_IMPLAUSIBLE_TRAVEL = 0.10;
  static constexpr uint32_t APPS_IMPLAUSIBLE_MS = 100;

public:
  // Each sensor: a fault below `low_fault` or above `high_fault` (open or
  // shorted), travel 0..1 from `start` to `end`
  Pedals(uint16_t bse_low_fault, uint16_t brake_start, uint16_t brake_end,
         uint16_t bse_high_fault, uint16_t apps_low_fault, uint16_t apps1_start,
         uint16_t apps1_end, uint16_t apps1_high_fault, uint16_t apps2_start,
         uint16_t apps2_end, uint16_t apps2_high_fault) {

    this->brake_ratio = 1 / float(brake_end - brake_start);
    this->brake_start = brake_start;
    this->bse_low_fault = bse_low_fault;
    this->bse_high_fault = bse_high_fault;

    this->apps_low_fault = apps_low_fault;

    apps1_ratio = 1 / float(apps1_end - apps1_start);
    this->apps1_start = apps1_start;
    this->apps1_high_fault = apps1_high_fault;

    apps2_ratio = 1 / float(apps2_end - apps2_start);
    this->apps2_start = apps2_start;
    this->apps2_high_fault = apps2_high_fault;
  };

  // TODO: Add additional pedal maps with diffrent curves?
  double get_torque_request(double apps_travel, double max_torque) {
    return apps_travel * max_torque;
  };

  // TODO: Make the release and apps_bse values confgiurable
  void update_travel(uint16_t raw_apps1, uint16_t raw_apps2,
                     uint16_t raw_brake, uint32_t now_ms) {
    this->raw_apps1 = raw_apps1;
    this->raw_apps2 = raw_apps2;
    this->raw_brake = raw_brake;

    // T.4.2.10, T.4.3.4: a reading outside the sensor's range means it's
    // open or shorted. Its travel then reads 0, not whatever it said last:
    // a stale value used to clear the fault below and become the torque
    // request with the driver's foot off the pedal.
    const bool apps1_ok =
        raw_apps1 >= apps_low_fault && raw_apps1 <= apps1_high_fault;
    const bool apps2_ok =
        raw_apps2 >= apps_low_fault && raw_apps2 <= apps2_high_fault;
    bse_fault = raw_brake < bse_low_fault || raw_brake > bse_high_fault;

    apps1_travel = apps1_ok ? clamp01((raw_apps1 - apps1_start) * apps1_ratio) : 0;
    apps2_travel = apps2_ok ? clamp01((raw_apps2 - apps2_start) * apps2_ratio) : 0;
    brake_travel = bse_fault ? 0 : clamp01((raw_brake - brake_start) * brake_ratio);

    // Reset if the pedals are released, and both sensors work
    if (apps1_ok && apps2_ok && apps1_travel < 0.1 && apps2_travel < 0.1) {
      apps_fault = false;
      apps_bse_fault = false;
    }
    if (!apps1_ok || !apps2_ok)
      apps_fault = true;

    // T.4.2.4, T.4.2.5: the APPS disagreeing cuts throttle for as long as
    // it lasts (stricter than the rule, so a glitch costs a dropout, not a
    // fault); lasting more than 100 ms latches it until the pedal's released
    const bool disagreed = apps_disagree;
    apps_disagree = std::fabs(apps1_travel - apps2_travel) > APPS_IMPLAUSIBLE_TRAVEL;
    if (apps_disagree && !disagreed)
      disagree_since = now_ms;
    if (apps_disagree && now_ms - disagree_since > APPS_IMPLAUSIBLE_MS)
      apps_fault = true;

    if (apps_fault || apps_disagree || bse_fault || apps_bse_fault) {
      travel = 0;
    } else {
      travel = apps1_travel;

      // Check that the driver isn't using both pedals at once
      if ((travel > 0.3) && (brake_travel > 0.3)) {
        apps_bse_fault = true;
        travel = 0;
      }
    }
  }
  static double clamp01(double x) { return x < 0 ? 0 : x > 1 ? 1 : x; }

  void update_steering(uint16_t raw_steering) {
    this->raw_steering = raw_steering;
  }

  inline bool get_bse_fault_ok_low() { return bse_fault; }
  // Throttle is cut for either; only the latched one needs a pedal release
  inline bool get_apps_fault_ok_low() { return apps_fault || apps_disagree; }
  inline bool get_apps_bse_fault_ok_low() { return apps_bse_fault; }

  inline uint16_t get_apps1_raw() { return raw_apps1; }
  inline uint16_t get_apps2_raw() { return raw_apps2; }
  inline uint16_t get_brake_raw() { return raw_brake; }
  inline double get_apps1_travel() { return apps1_travel; }
  inline double get_apps2_travel() { return apps2_travel; }
  inline double get_brake_travel() { return brake_travel; }
  inline double get_travel() { return travel; }
  inline double get_steering_raw() { return raw_steering; }
};
