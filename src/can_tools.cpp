#include "can_tools.hpp"
#include "settings.hpp"

void can_man::setup_can(can_controller target_can_controller, int target_baud) {
  switch (target_can_controller) {
  case TEENSY_CAN1:
    init_can1(target_baud);

    this->controller_has_new_msg = can1_has_new_msg;
    this->send_controller_new_msg = can1_send_msg;
    break;

  case TEENSY_CAN2:
    init_can2(target_baud);

    this->controller_has_new_msg = can2_has_new_msg;
    this->send_controller_new_msg = can2_send_msg;
    break;

  case TEENSY_CAN3:
    init_can3(target_baud);

    this->controller_has_new_msg = can3_has_new_msg;
    this->send_controller_new_msg = can2_send_msg;
    break;
  }
}

// TODO: This makes me cry but template functions here make me want to die so
#ifdef TEENSYDUINO
FlexCAN_T4<CAN1, RX_SIZE_1024, TX_SIZE_1024> can1;
FlexCAN_T4<CAN2, RX_SIZE_1024, TX_SIZE_1024> can2;
FlexCAN_T4<CAN3, RX_SIZE_1024, TX_SIZE_1024> can3;

void init_can1(int target_baud) {
  can1.begin();
  can1.setBaudRate(target_baud);
}
void init_can2(int target_baud) {
  can2.begin();
  can2.setBaudRate(target_baud);
}
void init_can3(int target_baud) {
  can3.begin();
  can3.setBaudRate(target_baud);
}

bool can1_has_new_msg(can_message *message_out) {
  if (can1.read(msg1)) {
    message_out->id = msg1.id;
    message_out->length = msg1.len;
    memcpy(message_out->buffer, msg1.buf, msg1.len);

    return true;
  } else {
    return false;
  }
}
bool can2_has_new_msg(can_message *message_out) {
  if (can2.read(msg2)) {
    message_out->id = msg2.id;
    message_out->length = msg2.len;
    memcpy(message_out->buffer, msg2.buf, msg2.len);

    return true;
  } else {
    return false;
  }
}
bool can3_has_new_msg(can_message *message_out) {
  if (can3.read(msg3)) {
    message_out->id = msg3.id;
    message_out->length = msg3.len;
    memcpy(message_out->buffer, msg3.buf, msg3.len);

    return true;
  } else {
    return false;
  }
}

void can1_send_msg(can_message *message_out) {
  msg1.id = message_out->id;
  msg1.len = message_out->length;
  memcpy(msg1.buf, message_out->buffer, message_out->length);

  can1.write(msg1);
}
void can2_send_msg(can_message *message_out) {
  msg2.id = message_out->id;
  msg2.len = message_out->length;
  memcpy(msg2.buf, message_out->buffer, message_out->length);

  can2.write(msg2);
}
void can3_send_msg(can_message *message_out) {
  msg3.id = message_out->id;
  msg3.len = message_out->length;
  memcpy(msg3.buf, message_out->buffer, message_out->length);

  can3.write(msg3);
}
#endif
