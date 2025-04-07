#pragma once

#include <can_tools.hpp>
#include <car.h>

class CM200 {
private:
  bool spin_forward = true;
  bool inverter_enable = false;
  bool inverter_discharge = false;

  bool speed_mode = false;
  int16_t speed_request = 0;
  double torque_request = 0;
  double torque_limit = 0;

  uint16_t motor_rpm;
  uint16_t motor_temp;

  bool over_power;
  uint32_t over_power_event;
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
  CM200(bool (*timer_mc_kick)(), bool (*timer_current_limit)(),
        bool (*timer_motor_controller_send)(), bool spin_direction, canMan *can,
        can_obj_car_h_t *dbc, float over_power_decay_factor);

  inline uint8_t get_torque_limit() { return uint8_t(torque_limit); }
  inline bool get_inverter_enable() { return inverter_enable; }

  inline void set_power_limit(double limit) { power_limit = limit; }
  inline void set_torque_limit(double limit) { torque_limit = limit; }
  inline void set_inverter_enable(bool enable) { inverter_enable = enable; }

  void update_bus_current(uint64_t msg_in, uint8_t length);
  void update_bus_voltage(uint64_t msg_in, uint8_t length);
  void ping();
  void command_torque(double torque_request);
  void command_speed(int16_t speed_request);
};
