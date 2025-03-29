#include "cm200.hpp"

cm200::cm200(bool (*timer_mc_kick)(), bool (*timer_current_limit)(),
             bool (*timer_inverter_enable)(),
             bool (*timer_motor_controller_send)(), bool spin_direction,
             canMan *can, can_obj_car_h_t *dbc) {
  this->timer_mc_kick = timer_mc_kick;
  this->timer_current_limit = timer_current_limit;
  this->timer_inverter_enable = timer_inverter_enable;
  this->timer_motor_controller_send = timer_motor_controller_send;

  this->spin_forward = spin_direction;

  this->can = can;
  this->dbc = dbc;

  this->inverter_enable = true;
  this->ping();
}

void cm200::ping() {
  if (timer_mc_kick()) {
    encode_can_0x0c0_Torque_Command(dbc, 0.0);
    encode_can_0x0c0_Torque_Limit_Command(dbc, 0.0);
    encode_can_0x0c0_Speed_Command(dbc, 0.0);
    encode_can_0x0c0_Speed_Mode_Enable(dbc, 0.0);
    encode_can_0x0c0_Inverter_Discharge(dbc, 0.0);
    encode_can_0x0c0_Direction_Command(dbc, spin_forward);
    encode_can_0x0c0_Inverter_Enable(dbc, inverter_enable);

    can_message out_msg;
    out_msg.id = CAN_ID_M192_COMMAND_MESSAGE;
    out_msg.length =
        pack_message(dbc, CAN_ID_M192_COMMAND_MESSAGE, &out_msg.buf.val);

    can->send_controller_message(out_msg);
  }
}
