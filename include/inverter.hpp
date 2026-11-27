#pragma once

#include <can_tools.hpp>
#include <car.h>
#include <cmath>
class Inverter {
private:
  uint32_t time_last_msec = 0;

  bool spin_forward = false;
  bool inverter_enable = false;
  bool inverter_discharge = false;

  bool speed_mode = false;
  int16_t speed_request = 0;
  int16_t speed_limit = 0;
  double torque_request = 0;
  double torque_limit_nm = 0;

  int16_t motor_rpm = 0;
  uint16_t motor_temp = 0;
  double distance_M = 0;

  bool over_power = false;
  uint32_t over_power_event_epoch = 0;
  float over_power_decay_factor;
  double bus_voltage = 0;
  double bus_current = 0;
  uint16_t power_limit_kw = 0;

  // Closed loop on measured power (regulate_power): a trim on the power the
  // open-loop torque cap allows
  double power_trim = 1.0;
  bool power_sample_new = false;
  bool power_sample_ever = false;
  uint32_t power_sample_ms = 0;

  bool (*timer_mc_kick)() = nullptr;
  bool (*timer_current_limit)() = nullptr;
  bool (*timer_motor_controller_send)() = nullptr;

  canMan *can;
  canMan *daq;
  can_obj_car_h_t *dbc;

public:
  Inverter(bool spin_direction, canMan *can, canMan *daq, can_obj_car_h_t *dbc,
           float over_power_decay_factor);

  inline uint8_t get_torque_limit() { return uint8_t(torque_limit_nm); }
  inline bool get_inverter_enable() { return inverter_enable; }
  inline double get_bus_voltage() { return bus_voltage; }
  inline int16_t get_motor_rpm() { return motor_rpm; }
  inline double get_bus_current() { return bus_current; }
  inline uint32_t get_motor_distance_M() { return distance_M; }
  uint16_t get_instant_current_limit(float voltage) {
    // No pack voltage yet (or garbage): allow no discharge current rather than
    // dividing by zero. Float->int of inf/NaN is UB, and on the Teensy it
    // saturates, which would mean no limit at all
    if (!(voltage > 0))
      return 0;

    float limit = (power_limit_kw * 1000) / voltage;
    if (limit >= UINT16_MAX)
      return UINT16_MAX;
    return uint16_t(limit);
  }

  inline void set_torque_limit(double limit) { torque_limit_nm = limit; }
  inline void set_speed_limit(uint16_t limit) { speed_limit = limit; }
  inline void set_power_limit_kw(uint16_t limit) { power_limit_kw = limit; }
  inline void set_inverter_enable(bool enable) { inverter_enable = enable; }
  void set_current_limits(uint16_t charge_limit, uint16_t discharge_limit);

  void update_bus_current(uint64_t msg_in, uint8_t length);
  void update_bus_voltage(uint64_t msg_in, uint8_t length);
  void update_motor_feedback(uint64_t msg_in, uint8_t length);

  void calculate_motor_distance_M(uint32_t time_msec);

  void ping();
  void send_clear_faults();
  void command_torque(double torque_request);

  // EV.3.3.1 limits the power the Energy Meter measures, V x I at the
  // accumulator, not shaft power. command_torque caps torque at the power
  // limit x 0.90 assumed drivetrain efficiency / motor speed; this trims that
  // cap on the inverter's measured DC bus V x I: each new sample over
  // POWER_LIMIT_TARGET of the limit scales it down so the next one lands on
  // it, and below the target it creeps back up (to 1/0.90, the full limit
  // at the shaft, if the drivetrain turns out lossless). Without fresh
  // samples it never allows more than the open-loop cap. Call every loop.
  void regulate_power(uint32_t now_ms);
  inline double get_power_trim() { return power_trim; }
  inline double get_measured_power_w() { return bus_voltage * bus_current; }
  void command_speed(int16_t speed_request);
};
