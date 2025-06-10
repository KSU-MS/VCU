#pragma once

#include <can_tools.hpp>
#include <car.h>

class Inverter {
private:
  bool spin_forward = true;
  bool inverter_enable = false;
  bool inverter_discharge = false;

  bool speed_mode = false;
  int16_t speed_request = 0;
  int16_t speed_limit = 0;
  double torque_request = 0;
  double torque_limit = 0;

  uint16_t motor_rpm;
  uint16_t motor_temp;

  bool over_power;
  uint32_t over_power_event_epoch;
  float over_power_decay_factor;
  double bus_voltage;
  double bus_current;
  uint16_t power_limit;

  bool (*timer_mc_kick)();
  bool (*timer_current_limit)();
  bool (*timer_motor_controller_send)();

  canMan *can;
  can_obj_car_h_t *dbc;

public:
  Inverter(bool (*timer_mc_kick)(), bool (*timer_current_limit)(),
           bool (*timer_motor_controller_send)(), bool spin_direction,
           canMan *can, can_obj_car_h_t *dbc, float over_power_decay_factor);

  inline uint8_t get_torque_limit() { return uint8_t(torque_limit); }
  inline bool get_inverter_enable() { return inverter_enable; }
  inline double get_bus_voltage() { return bus_voltage; }
  inline double get_bus_current() { return bus_current; }

  inline void set_power_limit(double limit) { power_limit = limit; }
  inline void set_torque_limit(double limit) { torque_limit = limit; }
  inline void set_speed_limit(uint16_t limit) { speed_limit = limit; }
  inline void set_inverter_enable(bool enable) { inverter_enable = enable; }
  void set_current_limits(uint16_t charge_limit, uint16_t discharge_limit);

  void update_bus_current(uint64_t msg_in, uint8_t length);
  void update_bus_voltage(uint64_t msg_in, uint8_t length);

  void ping();
  void send_clear_faults();
  void command_torque(double torque_request);
  void command_speed(int16_t speed_request);
};
