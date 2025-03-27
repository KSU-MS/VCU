#pragma once

#include "can_handle.hpp"
#include "main.hpp"
#include <stdint.h>

// TODO: Add more timer methods and make this more flexi
#ifdef ARDUINO
#include <Metro.h>

Metro timer_20hz = Metro(50, 1);
Metro timer_2s = Metro(2000, 1);
Metro timer_200hz = Metro(5, 1);
Metro timer_100hz = Metro(10, 1);
#endif

enum timer_method {
  metro,
};

class Inverter {
private:
  bool spin_forward = true;
  bool inverter_enable = false;
  uint16_t motor_rpm;
  uint16_t motor_temp;
  uint16_t bus_voltage;

  bool *timer_mc_kick();
  bool *timer_current_limit();
  bool *timer_inverter_enable();
  bool *timer_motor_controller_send();

  void disable();

public:
  Inverter(timer_method clock, bool spin_direction);

  void ping();
};
