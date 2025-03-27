#include "inverter.hpp"
#include "can_handle.h"

Inverter::Inverter(timer_method clock, bool spin_direction) {
  switch (clock) {
  case metro:
#ifdef ARDUINO
    this->timer_mc_kick() = timer_20hz.check();
    this->timer_current_limit() = timer_100hz.check();
    this->timer_inverter_enable() = timer_2s.check();
    this->timer_motor_controller_send() = timer_200hz.check();
#endif
    break;
  };

  this->spin_forward = spin_direction;

  this->inverter_enable = true;
  this->ping();
}

void Inverter::ping() {
  if (this->timer_mc_kick()) {
    inv_can.send_controller_message(pack_inverter_power_command(
        0, 0, this->spin_forward, 0, 0, this->inverter_enable, 0));
  }
}
