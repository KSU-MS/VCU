#pragma once
#include <stdint.h>

// This enum has the modes for "Launch" control and "Slip" control, after the
// car gets going its expecting to transition into slip control to keep goin
enum traction_mode {
  LC_DRIVERCONTROL, // Driver throttle directly after timer for launch
  LC_MU_ESTIMATE,   // Uses estimated friction coefficent for launch
  TC_LOOKUP,        // Polynomial/Lookup table for slip
  TC_PID,           // PID controller for slip
  TC_LINEAR,        // Linear equation based controller for slip
};

class TractionController {
private:
  traction_mode current_mode;

  uint32_t start_time_ms = 0;
  uint32_t current_time_ms = 0;
  uint16_t elapsed_time_ms = 0;
  double driver_torque_request_nm = 0; // Driver allowed torque
  double lc_torque_request_nm = 0;     // Controller target
  double output_torque_command_nm = 0; // Final smoothed output

  double maximum_rate_of_change = 100;

  // LC_LOOKUP vars
  const float cal_5 = -0.000000000000085; // Fifth
  const float cal_4 = 0.000000000114956;  // Fourth
  const float cal_3 = 0.000000015913376;  // Third
  const float cal_2 = 0.000011808754927;  // Second
  const float cal_1 = 0.093415288604319;  // First order of poly
  const float cal_intercept = 10.361085973494500;

  // LC_PID vars
  const double tire_slip_low = 0.05;
  const double tire_slip_high = 0.2;
  double k_p = 4.0;
  double k_i = 2.0;
  double k_d = 1.0;
  const double output_min = -1.0; // Minimum output of the PID controller
  const double output_max = 0;    // Max output of the PID controller
  double input, setpoint, output;
  // AutoPID pid;

  // LC_LINEAR vars
  double linear_m; // Slope of the linear equation
  double linear_b; // Intercept of the linear equation

public:
  inline void update_time(uint32_t sys_time) {
    this->current_time_ms = sys_time;
  };
  double get_torque_target();
};
