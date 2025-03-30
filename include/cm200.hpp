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
  uint16_t bus_voltage;

  bool (*timer_mc_kick)();
  bool (*timer_current_limit)();
  bool (*timer_motor_controller_send)();

  void disable();

  canMan *can;

  can_obj_car_h_t *dbc;

public:
  CM200(bool (*timer_mc_kick)(), bool (*timer_current_limit)(),
        bool (*timer_motor_controller_send)(), bool spin_direction, canMan *can,
        can_obj_car_h_t *dbc);

  inline uint8_t get_torque_limit() { return uint8_t(torque_limit); }
  inline bool get_inverter_enable() { return inverter_enable; }

  void ping();
  void command_torque(double torque_request);
  void command_speed(int16_t speed_request);
};
