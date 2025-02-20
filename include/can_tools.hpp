#pragma once
#include <stdint.h>

enum can_controller { TEENSY_CAN1 = 0, TEENSY_CAN2 = 1, TEENSY_CAN3 = 2 };

struct can_message {
  uint16_t id;
  uint8_t length;
  uint8_t *buffer;
};

// TODO: Rework this
class can_man {
private:
  bool (*controller_has_new_msg)(can_message *);
  void (*send_controller_new_msg)(can_message *);

  can_message controller_message;

public:
  void setup_can(can_controller target_controller, int target_baud);

  inline bool check_controller_message() {
    return this->controller_has_new_msg(&controller_message);
  }

  inline can_message get_controller_message() {
    return this->controller_message;
  }

  inline void send_controller_message(can_message out_msg) {
    this->send_controller_new_msg(&out_msg);
  }
};

#ifdef TEENSYDUINO
#include <FlexCAN_T4.h>

class flexcan_controller {
private:
  // void *controller;
  CAN_message_t msg;

public:
  template <typename CAN_DEVICE>
  void init(CAN_DEVICE &target_controller, int target_baud);

  bool has_new_message(can_message *message);
  void send_message(can_message *message);
};

CAN_message_t msg1;
CAN_message_t msg2;
CAN_message_t msg3;

void init_can1(int target_baud);
void init_can2(int target_baud);
void init_can3(int target_baud);

bool can1_has_new_msg(can_message *message_out);
bool can2_has_new_msg(can_message *message_out);
bool can3_has_new_msg(can_message *message_out);

void can1_send_msg(can_message *message_out);
void can2_send_msg(can_message *message_out);
void can3_send_msg(can_message *message_out);
#endif
