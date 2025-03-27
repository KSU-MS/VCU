#pragma once

// TODO: I need to find all the random scatered messages the KS6e firmware was
// messing with, and figure out what it was doing with them, so some features
// maybe borked until I figure it out

// NOTE: Currently this file has functions to
// Unpack
//  ACU_SHUTDOWN_STATUS
//  PRECHARGE_STATUS
//  DASH_BUTTONS
// Pack
//  VCU_PEDAL_READINGS
//  VCU_PEDALS_TRAVEL
//  VCU_STATUS
//  VCU_BOARD_READINGS_ONE
//  VCU_BOARD_READINGS_TWO
//  VCU_FIRMWARE_VERSION
//  BMS_CURRENT_LIMIT
//  M192_COMMAND_MESSAGE

#include "car.h"
#include <can_tools.hpp>

can_obj_car_h_t kms_can;

void unpack_acu_shutdown_message(uint64_t msg, uint8_t length, uint32_t time,
                                 bool *imd_ok_hs, bool *bms_ok_hs) {
  // Decode the message
  unpack_message(&kms_can, CAN_ID_ACU_SHUTDOWN_STATUS, msg, length, time);

  // Grab what we want
  uint8_t imd_gpio_val, bms_gpio_val;

  decode_can_0x258_acu_imd_gpio_state(&kms_can, &imd_gpio_val);
  decode_can_0x258_acu_bms_gpio_state(&kms_can, &bms_gpio_val);

  *imd_ok_hs = imd_gpio_val;
  *bms_ok_hs = bms_gpio_val;
}

void unpack_tcu_precharge_message(uint64_t msg, uint8_t length, uint32_t time,
                                  uint8_t *precharge_state) {

  unpack_message(&kms_can, CAN_ID_PRECHARGE_STATUS, msg, length, time);

  decode_can_0x069_precharge_state(&kms_can, precharge_state);
}

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

can_message pack_pedal_voltage_message(double apps1_voltage,
                                       double apps2_voltage,
                                       double bse_voltage) {
  // Encode all the message feilds
  encode_can_0x0c4_APPS1(&kms_can, apps1_voltage);
  encode_can_0x0c4_APPS2(&kms_can, apps2_voltage);
  encode_can_0x0c4_BSE1(&kms_can, bse_voltage);

  // Init and pack the message
  can_message out_msg;
  out_msg.id = CAN_ID_VCU_PEDAL_READINGS;
  out_msg.length =
      pack_message(&kms_can, CAN_ID_VCU_PEDAL_READINGS, &out_msg.buf.val);

  return out_msg;
}

can_message pack_pedal_travel_message(double apps1_travel, double apps2_travel,
                                      double bse_travel) {

  encode_can_0x0cc_vcu_apps1_travel(&kms_can, apps1_travel);
  encode_can_0x0cc_vcu_apps2_travel(&kms_can, apps2_travel);
  encode_can_0x0cc_vcu_bse1_travel(&kms_can, bse_travel);

  // Init and pack the message
  can_message out_msg;
  out_msg.id = CAN_ID_VCU_PEDALS_TRAVEL;
  out_msg.length =
      pack_message(&kms_can, CAN_ID_VCU_PEDALS_TRAVEL, &out_msg.buf.val);

  return out_msg;
}

// Oh god
can_message pack_vcu_status_message(
    bool accel_brake_implausible, bool brake_implausible,
    bool accel_implausible, bool brake_active, bool bspd_brake_high,
    bool bspd_current_high, bool bspd_ok_hs, bool bms_ok_hs, bool imd_ok_hs,
    bool shutdown_b_ok_hs, bool shutdown_c_ok_hs, bool shutdown_d_ok_hs,
    bool shutdown_e_ok_hs, bool software_ok_hs, bool buzzer_active,
    bool software_ok, uint16_t distance_travelled, bool energy_meter_present,
    bool inverter_powered, bool launch_control_active, uint8_t max_torque,
    uint8_t torque_mode, uint8_t vcu_state) {

  encode_can_0x0c3_VCU_ACCEL_BRAKE_IMPLAUSIBLE(&kms_can,
                                               accel_brake_implausible);
  encode_can_0x0c3_VCU_ACCEL_IMPLAUSIBLE(&kms_can, brake_implausible);
  encode_can_0x0c3_VCU_BRAKE_IMPLAUSIBLE(&kms_can, accel_implausible);
  encode_can_0x0c3_VCU_BRAKE_ACTIVE(&kms_can, brake_active);
  encode_can_0x0c3_VCU_BSPD_BRAKE_HIGH(&kms_can, bspd_brake_high);
  encode_can_0x0c3_VCU_BSPD_CURRENT_HIGH(&kms_can, bspd_current_high);
  encode_can_0x0c3_VCU_BSPD_OK_HIGH(&kms_can, bspd_ok_hs);
  encode_can_0x0c3_VCU_BMS_OK_HIGH(&kms_can, bms_ok_hs);
  encode_can_0x0c3_VCU_IMD_OK_HIGH(&kms_can, imd_ok_hs);
  encode_can_0x0c3_VCU_SHUTDOWN_B_OK_HIGH(&kms_can, shutdown_b_ok_hs);
  encode_can_0x0c3_VCU_SHUTDOWN_C_OK_HIGH(&kms_can, shutdown_c_ok_hs);
  encode_can_0x0c3_VCU_SHUTDOWN_D_OK_HIGH(&kms_can, shutdown_d_ok_hs);
  encode_can_0x0c3_VCU_SHUTDOWN_E_OK_HIGH(&kms_can, shutdown_e_ok_hs);
  encode_can_0x0c3_VCU_SOFTWARE_OK_HIGH(&kms_can, software_ok_hs);
  encode_can_0x0c3_VCU_ACTIVATE_BUZZER(&kms_can, buzzer_active);
  encode_can_0x0c3_VCU_SOFTWARE_OK(&kms_can, software_ok);
  encode_can_0x0c3_VCU_DISTANCE_TRAVELLED(&kms_can, distance_travelled);
  encode_can_0x0c3_VCU_ENERGY_METER_PRESENT(&kms_can, energy_meter_present);
  encode_can_0x0c3_VCU_INVERTER_POWERED(&kms_can, inverter_powered);
  encode_can_0x0c3_VCU_LAUNCH_CONTROL_ACTIVE(&kms_can, launch_control_active);
  encode_can_0x0c3_VCU_MAX_TORQUE(&kms_can, max_torque);
  encode_can_0x0c3_VCU_TORQUE_MODE(&kms_can, torque_mode);
  encode_can_0x0c3_VCU_STATEMACHINE_STATE(&kms_can, vcu_state);

  // Init and pack the message
  can_message out_msg;
  out_msg.id = CAN_ID_VCU_STATUS;
  out_msg.length = pack_message(&kms_can, CAN_ID_VCU_STATUS, &out_msg.buf.val);

  return out_msg;
}

can_message pack_voltage_readings_one_message(uint16_t glv_voltage,
                                              uint16_t glv_current,
                                              uint16_t v5_voltage,
                                              uint16_t bspd_voltage) {

  encode_can_0x0c9_VCU_GLV_VOLTAGE(&kms_can, glv_voltage);
  encode_can_0x0c9_VCU_GLV_CURRENT(&kms_can, glv_current);
  encode_can_0x0c9_VCU_5V_VOLTAGE(&kms_can, v5_voltage);
  encode_can_0x0c9_VCU_BSPD_VOLTAGE(&kms_can, bspd_voltage);

  can_message out_msg;
  out_msg.id = CAN_ID_VCU_BOARD_READINGS_ONE;
  out_msg.length =
      pack_message(&kms_can, CAN_ID_VCU_BOARD_READINGS_ONE, &out_msg.buf.val);

  return out_msg;
}

can_message pack_voltage_readings_two_message(uint16_t sdc_voltage,
                                              uint16_t sdc_current,
                                              uint16_t ain9_voltage,
                                              uint16_t ain10_voltage) {

  encode_can_0x0ca_VCU_SDC_VOLTAGE(&kms_can, sdc_voltage);
  encode_can_0x0ca_VCU_SDC_CURRENT(&kms_can, sdc_current);
  encode_can_0x0ca_VCU_AIN9_VOLTAGE(&kms_can, ain9_voltage);
  encode_can_0x0ca_VCU_AIN10_VOLTAGE(&kms_can, ain10_voltage);

  can_message out_msg;
  out_msg.id = CAN_ID_VCU_BOARD_READINGS_TWO;
  out_msg.length =
      pack_message(&kms_can, CAN_ID_VCU_BOARD_READINGS_TWO, &out_msg.buf.val);

  return out_msg;
}

can_message pack_bms_current_limit(uint16_t charge_limit,
                                   uint16_t discharge_limit) {

  encode_can_0x202_D2_Max_Charge_Current(&kms_can, charge_limit);
  encode_can_0x202_D1_Max_Discharge_Current(&kms_can, discharge_limit);

  can_message out_msg;
  out_msg.id = CAN_ID_BMS_CURRENT_LIMIT;
  out_msg.length =
      pack_message(&kms_can, CAN_ID_BMS_CURRENT_LIMIT, &out_msg.buf.val);

  return out_msg;
}

can_message pack_inverter_power_command(double torque_request,
                                        double torque_limit, bool direction,
                                        uint16_t speed_request, bool speed_mode,
                                        bool inverter_enable,
                                        bool inverter_discharge) {

  encode_can_0x0c0_Torque_Command(&kms_can, torque_request);
  encode_can_0x0c0_Torque_Limit_Command(&kms_can, torque_limit);
  encode_can_0x0c0_Direction_Command(&kms_can, direction);
  encode_can_0x0c0_Speed_Command(&kms_can, speed_request);
  encode_can_0x0c0_Speed_Mode_Enable(&kms_can, speed_mode);
  encode_can_0x0c0_Inverter_Enable(&kms_can, inverter_enable);
  encode_can_0x0c0_Inverter_Discharge(&kms_can, inverter_discharge);

  can_message out_msg;
  out_msg.id = CAN_ID_M192_COMMAND_MESSAGE;
  out_msg.length =
      pack_message(&kms_can, CAN_ID_M192_COMMAND_MESSAGE, &out_msg.buf.val);

  return out_msg;
}

// These values are provided by the python script ran by the lib_dep
// https://github.com/KSU-MS/pio-git-hash-gen
#ifndef AUTO_VERSION
#warning "AUTO_VERSION was not defined by the generator!"
#define AUTO_VERSION 0xdeadbeef
#endif

#ifndef FW_PROJECT_IS_DIRTY
#warning "FW_PROJECT_IS_DIRTY was not defined by the generator!"
#define FW_PROJECT_IS_DIRTY 1
#endif

#ifndef FW_PROJECT_IS_MAIN_OR_MASTER
#warning "FW_PROJECT_IS_MAIN_OR_MASTER was not defined by the generator!"
#define FW_PROJECT_IS_MAIN_OR_MASTER 0
#endif

can_message pack_status_message(uint16_t time) {
  encode_can_0x0c8_vcu_on_time_seconds(&kms_can, time);
  encode_can_0x0c8_vcu_fw_version(&kms_can, AUTO_VERSION);
  encode_can_0x0c8_vcu_project_is_dirty(&kms_can, FW_PROJECT_IS_DIRTY);
  encode_can_0x0c8_vcu_project_on_main(&kms_can, FW_PROJECT_IS_MAIN_OR_MASTER);

  can_message out_msg;
  out_msg.id = CAN_ID_VCU_FIRMWARE_VERSION;
  out_msg.length =
      pack_message(&kms_can, CAN_ID_VCU_FIRMWARE_VERSION, &out_msg.buf.val);

  return out_msg;
}
