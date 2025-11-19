#include "data_handler.hpp"

void start_comms(void) {
  // TODO: Maybe use mailboxes and interupts off the mailboxes to handle the ID
  // filtering? IMO we are not performace/reasource constrained and doing
  // interupts off specific messages doesn't really seem like something we need
  // to do...

  // acc_can.enableMBInterrupts(); acc_can.setMaxMB(NUM_TX_MAILBOXES +
  // NUM_RX_MAILBOXES);
  //
  // for (int i = 0; i < NUM_RX_MAILBOXES; i++) {
  //   acc_can.setMB((FLEXCAN_MAILBOX)i, RX, EXT);
  // }
  //
  // for (int i = NUM_RX_MAILBOXES; i < (NUM_TX_MAILBOXES + NUM_RX_MAILBOXES);
  //      i++) {
  //   acc_can.setMB((FLEXCAN_MAILBOX)i, TX, EXT);
  // }
  //
  // acc_can.setMBFilter(REJECT_ALL);
  //
  // acc_can.onReceive(MB0, some_function_here?);
  // acc_can.setMBUserFilter(MB0, 0x00, 0xFF);
  // acc_can.mailboxStatus();
}

void process_acc_message(void) {
  can_message msg_in = acc_can.get_controller_message();
  daq_can.send_controller_message(msg_in);

  switch (msg_in.id) {
  case CAN_ID_ACU_SHUTDOWN_STATUS:
    // accumulator.update_acu_status(msg_in.buf.val, msg_in.length);
    break;

  case CAN_ID_PRECHARGE_STATUS:
    // accumulator.update_precharge_status(msg_in.buf.val, msg_in.length);
    break;

  case CAN_ID_MSGID_0X6B1:
    // accumulator.update_pack_power(msg_in.buf.val, msg_in.length);
    break;

  case CAN_ID_MSGID_0X6B3:
    inv_can.send_controller_message(msg_in);
    break;

  default:
    break;
  }
}

void process_inv_message(void) {
  can_message msg_in = inv_can.get_controller_message();
  daq_can.send_controller_message(msg_in);

  switch (msg_in.id) {
  case CAN_ID_DASH_BUTTONS:
    // vcu.update_dash_buttons(msg_in.buf.val, msg_in.length);
    break;

  case CAN_ID_M165_MOTOR_POSITION_INFO:
    // vcu.inverter->update_motor_feedback(msg_in.buf.val, msg_in.length);
    break;

  case CAN_ID_M166_CURRENT_INFO:
    // vcu.inverter->update_bus_current(msg_in.buf.val, msg_in.length);
    break;

  case CAN_ID_M167_VOLTAGE_INFO:
    // vcu.inverter->update_bus_voltage(msg_in.buf.val, msg_in.length);
    acc_can.send_controller_message(msg_in);
    break;

  default:
    break;
  }
}

void process_daq_message(void) {
  can_message msg_in = daq_can.get_controller_message();

  switch (msg_in.id) {
  // case CAN_ID_VCU_SET_PARAMETER:
  //   vcu.set_parameter(msg_in.buf.val, msg_in.length);
  //   break;
  case CAN_ID_M193_READ_WRITE_PARAM_COMMAND:
    inv_can.send_controller_message(msg_in);
    break;

  default:
    break;
  }
}
