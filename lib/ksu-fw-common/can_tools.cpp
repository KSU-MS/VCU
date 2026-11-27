#include "can_tools.hpp"

canMan::canMan(can_controller target_can_controller, int target_baud) {
  switch (target_can_controller) {
  case TEENSY_CAN1:
    init_flexcan_1(target_baud);

    this->controller_has_new_msg = flexcan_1_has_new_msg;
    this->send_controller_new_msg = flexcan_1_send_msg;
    break;

  case TEENSY_CAN2:
    init_flexcan_2(target_baud);

    this->controller_has_new_msg = flexcan_2_has_new_msg;
    this->send_controller_new_msg = flexcan_2_send_msg;
    break;

  case TEENSY_CAN3:
    init_flexcan_3(target_baud);

    this->controller_has_new_msg = flexcan_3_has_new_msg;
    this->send_controller_new_msg = flexcan_3_send_msg;
    break;
  }
}

// TODO: This makes me cry but template functions here make me want to die so
#if defined(__IMXRT1062__)
FlexCAN_T4<CAN1, RX_SIZE_1024, TX_SIZE_1024> flexcan_1;
FlexCAN_T4<CAN2, RX_SIZE_1024, TX_SIZE_1024> flexcan_2;
FlexCAN_T4<CAN3, RX_SIZE_1024, TX_SIZE_1024> flexcan_3;

CAN_message_t msg1;
CAN_message_t msg2;
CAN_message_t msg3;

void init_flexcan_1(int target_baud) {
  flexcan_1.begin();
  flexcan_1.setBaudRate(target_baud);
}
void init_flexcan_2(int target_baud) {
  flexcan_2.begin();
  flexcan_2.setBaudRate(target_baud);
}
void init_flexcan_3(int target_baud) {
  flexcan_3.begin();
  flexcan_3.setBaudRate(target_baud);
}

bool flexcan_1_has_new_msg(can_message *message_out) {
  if (flexcan_1.read(msg1)) {
    message_out->id = msg1.id;
    message_out->length = msg1.len;
    message_out->is_extended = msg1.flags.extended;

    if (msg1.len <= 8)
      memcpy(message_out->buf.byte, msg1.buf, msg1.len);

    return true;
  } else {
    return false;
  }
}
bool flexcan_2_has_new_msg(can_message *message_out) {
  if (flexcan_2.read(msg2)) {
    message_out->id = msg2.id;
    message_out->length = msg2.len;
    message_out->is_extended = msg2.flags.extended;

    if (msg2.len <= 8)
      memcpy(message_out->buf.byte, msg2.buf, msg2.len);

    return true;
  } else {
    return false;
  }
}
bool flexcan_3_has_new_msg(can_message *message_out) {
  if (flexcan_3.read(msg3)) {
    message_out->id = msg3.id;
    message_out->length = msg3.len;
    message_out->is_extended = msg3.flags.extended;

    if (msg3.len <= 8)
      memcpy(message_out->buf.byte, msg3.buf, msg3.len);

    return true;
  } else {
    return false;
  }
}

void flexcan_1_send_msg(can_message *message_out) {
  msg1.id = message_out->id;
  msg1.len = message_out->length;
  msg1.flags.extended = message_out->is_extended;
  memcpy(msg1.buf, message_out->buf.byte, message_out->length);

  flexcan_1.write(msg1);
}
void flexcan_2_send_msg(can_message *message_out) {
  msg2.id = message_out->id;
  msg2.len = message_out->length;
  msg2.flags.extended = message_out->is_extended;
  memcpy(msg2.buf, message_out->buf.byte, message_out->length);

  flexcan_2.write(msg2);
}
void flexcan_3_send_msg(can_message *message_out) {
  msg3.id = message_out->id;
  msg3.len = message_out->length;
  msg3.flags.extended = message_out->is_extended;
  memcpy(msg3.buf, message_out->buf.byte, message_out->length);

  flexcan_3.write(msg3);
}
#endif
