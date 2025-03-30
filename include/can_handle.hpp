#pragma once
#ifndef can_handle_hpp
#define can_handle_hpp

// TODO: I need to find all the random scatered messages the KS6e firmware was
// messing with, and figure out what it was doing with them, so some features
// maybe borked until I figure it out

void unpack_dash_buttons_message(uint64_t msg, uint8_t length, uint32_t time,
                                 bool *button_1, bool *button_2, bool *button_3,
                                 bool *button_4, bool *button_5,
                                 bool *button_6) {

  unpack_message(&kms_can, CAN_ID_DASH_BUTTONS, msg, length, time);

  uint8_t val_1, val_2, val_3, val_4, val_5, val_6;

  decode_can_0x0eb_dash_button1status(&kms_can, &val_1);
  decode_can_0x0eb_dash_button2status(&kms_can, &val_2);
  decode_can_0x0eb_dash_button3status(&kms_can, &val_3);
  decode_can_0x0eb_dash_button4status(&kms_can, &val_4);
  decode_can_0x0eb_dash_button5status(&kms_can, &val_5);
  decode_can_0x0eb_dash_button6status(&kms_can, &val_6);

  *button_1 = val_1;
  *button_2 = val_2;
  *button_3 = val_3;
  *button_4 = val_4;
  *button_5 = val_5;
  *button_6 = val_6;
}

#endif // can_handle_hpp
