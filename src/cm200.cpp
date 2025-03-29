#include "cm200.hpp"

cm200::cm200(bool (*timer_mc_kick)(), bool (*timer_current_limit)(),
             bool (*timer_inverter_enable)(),
             bool (*timer_motor_controller_send)(), bool spin_direction,
             canMan *can) {
  this->timer_mc_kick = timer_mc_kick;
  this->timer_current_limit = timer_current_limit;
  this->timer_inverter_enable = timer_inverter_enable;
  this->timer_motor_controller_send = timer_motor_controller_send;

  this->spin_forward = spin_direction;

  this->can = can;

  this->inverter_enable = true;
  this->ping();
}

void cm200::ping() {
  if (timer_mc_kick()) {
    can->send_controller_message(pack_inverter_power_command(
        0, 0, this->spin_forward, 0, 0, this->inverter_enable, 0));
  }
}
