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

// Filled by the mailbox interrupts, see CanRxQueue
CanRxQueue<CAN_RX_QUEUE_SIZE> rx_queue_1;
CanRxQueue<CAN_RX_QUEUE_SIZE> rx_queue_2;
CanRxQueue<CAN_RX_QUEUE_SIZE> rx_queue_3;

static void queue_frame(CanRxQueue<CAN_RX_QUEUE_SIZE> &queue,
                        const CAN_message_t &msg) {
  can_message out;
  out.id = msg.id;
  out.length = msg.len <= 8 ? msg.len : 8;
  out.is_extended = msg.flags.extended;
  memcpy(out.buf.byte, msg.buf, out.length);
  queue.push(out);
}

// Interrupt context
static void flexcan_1_on_receive(const CAN_message_t &msg) {
  queue_frame(rx_queue_1, msg);
}
static void flexcan_2_on_receive(const CAN_message_t &msg) {
  queue_frame(rx_queue_2, msg);
}
static void flexcan_3_on_receive(const CAN_message_t &msg) {
  queue_frame(rx_queue_3, msg);
}

// Mailbox interrupts also drain FlexCAN_T4's transmit queue: in polling mode
// a frame sent while all 8 TX mailboxes were busy was queued and never sent,
// since only the TX-complete interrupt or events() sends queued frames.
void init_flexcan_1(int target_baud) {
  flexcan_1.begin();
  flexcan_1.setBaudRate(target_baud);
  flexcan_1.onReceive(flexcan_1_on_receive);
  flexcan_1.enableMBInterrupts();
}
void init_flexcan_2(int target_baud) {
  flexcan_2.begin();
  flexcan_2.setBaudRate(target_baud);
  flexcan_2.onReceive(flexcan_2_on_receive);
  flexcan_2.enableMBInterrupts();
}
void init_flexcan_3(int target_baud) {
  flexcan_3.begin();
  flexcan_3.setBaudRate(target_baud);
  flexcan_3.onReceive(flexcan_3_on_receive);
  flexcan_3.enableMBInterrupts();
}

bool flexcan_1_has_new_msg(can_message *message_out) {
  return rx_queue_1.pop(message_out);
}
bool flexcan_2_has_new_msg(can_message *message_out) {
  return rx_queue_2.pop(message_out);
}
bool flexcan_3_has_new_msg(can_message *message_out) {
  return rx_queue_3.pop(message_out);
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
