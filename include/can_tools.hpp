#pragma once
#include <stdint.h>

// TODO: Add in the MCP2515 CAN fella
enum can_controller { TEENSY_CAN1 = 0, TEENSY_CAN2 = 1, TEENSY_CAN3 = 2 };

// TODO: Maybe add more of the CAN message settings to this fella
struct can_message {
  uint16_t id;
  uint8_t length;
  uint8_t *buffer;
};

// TODO: Make this a template function type beat
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

// TODO: Sadge
#ifdef TEENSYDUINO
#include <FlexCAN_T4.h>

void init_flexcan_1(int target_baud);
void init_flexcan_2(int target_baud);
void init_flexcan_3(int target_baud);

bool flexcan_1_has_new_msg(can_message *message_out);
bool flexcan_2_has_new_msg(can_message *message_out);
bool flexcan_3_has_new_msg(can_message *message_out);

void flexcan_1_send_msg(can_message *message_out);
void flexcan_2_send_msg(can_message *message_out);
void flexcan_3_send_msg(can_message *message_out);
#endif
