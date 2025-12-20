#include "data_handler.hpp"
#include "car.h"
#include "data.hpp"

// wierd static member initialization so we can just call the static functions
// from anywhere
DataHandler *DataHandler::active_instance = nullptr;

DataHandler::DataHandler(std::array<parameter, 25> *params,
                         VehicleData *vehicle_data)
    : params(params), vehicle_data(vehicle_data) {
  active_instance = this;
}

void DataHandler::start_comms(void) {
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

void DataHandler::process_acc_message(void) {
  if (!acc_can.check_controller_message()) {
    return;
  }

  can_message msg_in = acc_can.get_controller_message();
  daq_can.send_controller_message(msg_in);

  switch (msg_in.id) {
  case CAN_ID_ACU_SHUTDOWN_STATUS: {
    if (vehicle_data != nullptr) {
      unpack_message(&kms_can, CAN_ID_ACU_SHUTDOWN_STATUS, msg_in.buf.val,
                     msg_in.length, 0);

      uint8_t imd_relay_val = 0;
      uint8_t bms_relay_val = 0;
      decode_can_0x258_acu_imd_relay_state(&kms_can, &imd_relay_val);
      decode_can_0x258_acu_bms_relay_state(&kms_can, &bms_relay_val);

      vehicle_data->accumulator.imd_ok_hs = bool(imd_relay_val);
      vehicle_data->accumulator.bms_ok_hs = bool(bms_relay_val);
    }
    break;
  }

  case CAN_ID_PRECHARGE_STATUS: {
    if (vehicle_data != nullptr) {
      unpack_message(&kms_can, CAN_ID_PRECHARGE_STATUS, msg_in.buf.val,
                     msg_in.length, 0);

      decode_can_0x069_precharge_state(
          &kms_can, &vehicle_data->accumulator.precharge_state);
    }
    break;
  }

  case CAN_ID_MSGID_0X6B1: {
    if (vehicle_data != nullptr) {
      unpack_message(&kms_can, CAN_ID_MSGID_0X6B1, msg_in.buf.val,
                     msg_in.length, 0);

      decode_can_0x6b1_Pack_Summed_Voltage(
          &kms_can, &vehicle_data->accumulator.pack_voltage);
      decode_can_0x6b1_Pack_Current(&kms_can,
                                    &vehicle_data->accumulator.pack_current);
    }
    break;
  }

  case CAN_ID_MSGID_0X6B3:
    inv_can.send_controller_message(msg_in);
    break;

  default:
    break;
  }
}

void DataHandler::process_inv_message(void) {
  if (!inv_can.check_controller_message()) {
    return;
  }

  can_message msg_in = inv_can.get_controller_message();
  daq_can.send_controller_message(msg_in);

  switch (msg_in.id) {
  case CAN_ID_DASH_BUTTONS: {
    if (vehicle_data != nullptr) {
      unpack_message(&kms_can, CAN_ID_DASH_BUTTONS, msg_in.buf.val,
                     msg_in.length, 0);

      uint8_t button_val = 0;
      decode_can_0x0eb_dash_button3status(&kms_can, &button_val);
      vehicle_data->driver.rtd_button_pressed = static_cast<bool>(button_val);
    }
    break;
  }

  case CAN_ID_M165_MOTOR_POSITION_INFO: {
    if (vehicle_data != nullptr) {
      unpack_message(&kms_can, CAN_ID_M165_MOTOR_POSITION_INFO, msg_in.buf.val,
                     msg_in.length, 0);

      decode_can_0x0a5_INV_Motor_Speed(&kms_can,
                                       &vehicle_data->inverter.motor_rpm);
    }
    break;
  }

  case CAN_ID_M166_CURRENT_INFO: {
    if (vehicle_data != nullptr) {
      unpack_message(&kms_can, CAN_ID_M166_CURRENT_INFO, msg_in.buf.val,
                     msg_in.length, 0);

      decode_can_0x0a6_INV_DC_Bus_Current(&kms_can,
                                          &vehicle_data->inverter.bus_current);
    }
    acc_can.send_controller_message(msg_in);
    break;
  }

  case CAN_ID_M167_VOLTAGE_INFO: {
    if (vehicle_data != nullptr) {
      unpack_message(&kms_can, CAN_ID_M167_VOLTAGE_INFO, msg_in.buf.val,
                     msg_in.length, 0);

      decode_can_0x0a7_INV_DC_Bus_Voltage(&kms_can,
                                          &vehicle_data->inverter.bus_voltage);
    }
    acc_can.send_controller_message(msg_in);
    break;
  }

  default:
    break;
  }
}

void DataHandler::process_daq_message(void) {
  if (!daq_can.check_controller_message()) {
    return;
  }

  can_message msg_in = daq_can.get_controller_message();

  switch (msg_in.id) {
  case CAN_ID_VCU_SET_PARAMETER: {
    if (params != nullptr) {
      unpack_message(&kms_can, CAN_ID_VCU_SET_PARAMETER, msg_in.buf.val,
                     msg_in.length, 0);

      uint8_t target_parameter;
      uint32_t parameter_value;
      decode_can_0x0d6_vcu_target_parameter(&kms_can, &target_parameter);
      decode_can_0x0d6_vcu_parameter_value(&kms_can, &parameter_value);
      (*params)[target_parameter].parameter_value = parameter_value;
    }
    break;
  }
  case CAN_ID_M193_READ_WRITE_PARAM_COMMAND:
    inv_can.send_controller_message(msg_in);
    break;

  default:
    break;
  }
}

void DataHandler::send_acc(const can_message &msg) {
  if (active_instance == nullptr) {
    return;
  }

  active_instance->acc_can.send_controller_message(msg);
}

void DataHandler::send_inv(const can_message &msg) {
  if (active_instance == nullptr) {
    return;
  }

  active_instance->inv_can.send_controller_message(msg);
}

void DataHandler::send_daq(const can_message &msg) {
  if (active_instance == nullptr) {
    return;
  }

  active_instance->daq_can.send_controller_message(msg);
}

// Inverter encoding methods
void DataHandler::send_inverter_ping(bool spin_forward, bool inverter_enable,
                                     bool inverter_discharge) {
  if (active_instance == nullptr) {
    return;
  }

  encode_can_0x0c0_VCU_INV_Torque_Command(&active_instance->kms_can, 0.0);
  encode_can_0x0c0_VCU_INV_Torque_Limit_Command(&active_instance->kms_can, 0.0);
  encode_can_0x0c0_VCU_INV_Speed_Command(&active_instance->kms_can, 0);
  encode_can_0x0c0_VCU_INV_Speed_Mode_Enable(&active_instance->kms_can, 0);
  encode_can_0x0c0_VCU_INV_Direction_Command(&active_instance->kms_can,
                                             spin_forward);
  encode_can_0x0c0_VCU_INV_Inverter_Discharge(&active_instance->kms_can,
                                              inverter_discharge);
  encode_can_0x0c0_VCU_INV_Inverter_Enable(&active_instance->kms_can,
                                           inverter_enable);

  can_message out_msg;
  out_msg.id = CAN_ID_M192_COMMAND_MESSAGE;
  out_msg.length = pack_message(&active_instance->kms_can,
                                CAN_ID_M192_COMMAND_MESSAGE, &out_msg.buf.val);

  active_instance->send_inv(out_msg);
  active_instance->send_daq(out_msg);
}

void DataHandler::send_inverter_torque_command(double torque_target) {
  if (active_instance == nullptr) {
    return;
  }

  encode_can_0x0c0_VCU_INV_Torque_Command(&active_instance->kms_can,
                                          torque_target);

  can_message out_msg;
  out_msg.id = CAN_ID_M192_COMMAND_MESSAGE;
  out_msg.length = pack_message(&active_instance->kms_can,
                                CAN_ID_M192_COMMAND_MESSAGE, &out_msg.buf.val);

  active_instance->send_inv(out_msg);
  active_instance->send_daq(out_msg);
}

void DataHandler::send_inverter_speed_command(
    int16_t speed_request, bool spin_forward, bool speed_mode,
    bool inverter_enable, bool inverter_discharge, double max_torque) {
  if (active_instance == nullptr) {
    return;
  }

  encode_can_0x0c0_VCU_INV_Torque_Command(&active_instance->kms_can, 0.0);
  encode_can_0x0c0_VCU_INV_Torque_Limit_Command(&active_instance->kms_can,
                                                max_torque);
  encode_can_0x0c0_VCU_INV_Speed_Command(&active_instance->kms_can,
                                         speed_request);
  encode_can_0x0c0_VCU_INV_Speed_Mode_Enable(&active_instance->kms_can,
                                             speed_mode);
  encode_can_0x0c0_VCU_INV_Direction_Command(&active_instance->kms_can,
                                             spin_forward);
  encode_can_0x0c0_VCU_INV_Inverter_Discharge(&active_instance->kms_can,
                                              inverter_discharge);
  encode_can_0x0c0_VCU_INV_Inverter_Enable(&active_instance->kms_can,
                                           inverter_enable);

  can_message out_msg;
  out_msg.id = CAN_ID_M192_COMMAND_MESSAGE;
  out_msg.length = pack_message(&active_instance->kms_can,
                                CAN_ID_M192_COMMAND_MESSAGE, &out_msg.buf.val);

  active_instance->send_inv(out_msg);
  active_instance->send_daq(out_msg);
}

void DataHandler::send_inverter_current_limits(uint16_t charge_limit,
                                               uint16_t discharge_limit) {
  if (active_instance == nullptr) {
    return;
  }

  encode_can_0x202_BMS_Max_Charge_Current(&active_instance->kms_can,
                                          charge_limit);
  encode_can_0x202_BMS_Max_Discharge_Current(&active_instance->kms_can,
                                             discharge_limit);

  can_message out_msg;
  out_msg.id = CAN_ID_BMS_CURRENT_LIMIT;
  out_msg.length = pack_message(&active_instance->kms_can,
                                CAN_ID_BMS_CURRENT_LIMIT, &out_msg.buf.val);

  active_instance->send_inv(out_msg);
  active_instance->send_daq(out_msg);
}

void DataHandler::send_inverter_parameter(uint16_t param_address,
                                          uint32_t param_data) {
  if (active_instance == nullptr) {
    return;
  }

  encode_can_0x0c1_VCU_INV_Parameter_Address(&active_instance->kms_can,
                                             param_address);
  encode_can_0x0c1_VCU_INV_Parameter_RW_Command(&active_instance->kms_can,
                                                1); // write
  encode_can_0x0c1_VCU_INV_Parameter_Data(&active_instance->kms_can,
                                          param_data);

  can_message out_msg;
  out_msg.id = CAN_ID_M192_COMMAND_MESSAGE;
  out_msg.length = pack_message(&active_instance->kms_can,
                                CAN_ID_M192_COMMAND_MESSAGE, &out_msg.buf.val);

  active_instance->send_inv(out_msg);
  active_instance->send_daq(out_msg);
}

void DataHandler::send_inverter_clear_faults() {
  if (active_instance == nullptr) {
    return;
  }

  encode_can_0x0c1_VCU_INV_Parameter_Address(&active_instance->kms_can, 20);
  encode_can_0x0c1_VCU_INV_Parameter_RW_Command(&active_instance->kms_can, 1);
  encode_can_0x0c1_VCU_INV_Parameter_Data(&active_instance->kms_can, 0);

  can_message out_msg;
  out_msg.id = CAN_ID_M192_COMMAND_MESSAGE;
  out_msg.length = pack_message(&active_instance->kms_can,
                                CAN_ID_M192_COMMAND_MESSAGE, &out_msg.buf.val);

  active_instance->send_inv(out_msg);
  active_instance->send_daq(out_msg);
}

// VCU encoding methods
void DataHandler::send_vcu_status_message(
    bool bspd_brake_high, bool bspd_current_high, bool bspd_ok_hs,
    bool bms_ok_hs, bool imd_ok_hs, bool buzzer_active, bool inverter_enable,
    double max_torque, uint8_t torque_mode, int current_state) {
  if (active_instance == nullptr) {
    return;
  }

  encode_can_0x0c3_VCU_BSPD_BRAKE_HIGH(&active_instance->kms_can,
                                       bspd_brake_high);
  encode_can_0x0c3_VCU_BSPD_CURRENT_HIGH(&active_instance->kms_can,
                                         bspd_current_high);
  encode_can_0x0c3_VCU_BSPD_OK_HIGH(&active_instance->kms_can, bspd_ok_hs);
  encode_can_0x0c3_VCU_BMS_OK_HIGH(&active_instance->kms_can, bms_ok_hs);
  encode_can_0x0c3_VCU_IMD_OK_HIGH(&active_instance->kms_can, imd_ok_hs);
  encode_can_0x0c3_VCU_SHUTDOWN_B_OK_HIGH(&active_instance->kms_can, 0.0);
  encode_can_0x0c3_VCU_SHUTDOWN_C_OK_HIGH(&active_instance->kms_can, 0.0);
  encode_can_0x0c3_VCU_SHUTDOWN_D_OK_HIGH(&active_instance->kms_can, 0.0);
  encode_can_0x0c3_VCU_SHUTDOWN_E_OK_HIGH(&active_instance->kms_can, 0.0);
  encode_can_0x0c3_VCU_SOFTWARE_OK_HIGH(&active_instance->kms_can, true);
  encode_can_0x0c3_VCU_ACTIVATE_BUZZER(&active_instance->kms_can,
                                       buzzer_active);
  encode_can_0x0c3_VCU_SOFTWARE_OK(&active_instance->kms_can, true);
  encode_can_0x0c3_VCU_DISTANCE_TRAVELLED(&active_instance->kms_can, 0.0);
  encode_can_0x0c3_VCU_ENERGY_METER_PRESENT(&active_instance->kms_can, false);
  encode_can_0x0c3_VCU_INVERTER_POWERED(&active_instance->kms_can,
                                        inverter_enable);
  encode_can_0x0c3_VCU_LAUNCH_CONTROL_ACTIVE(&active_instance->kms_can, 0);
  encode_can_0x0c3_VCU_MAX_TORQUE(&active_instance->kms_can, max_torque);
  encode_can_0x0c3_VCU_TORQUE_MODE(&active_instance->kms_can, torque_mode);
  encode_can_0x0c3_VCU_STATEMACHINE_STATE(&active_instance->kms_can,
                                          current_state);

  can_message out_msg;
  out_msg.id = CAN_ID_VCU_STATUS;
  out_msg.length = pack_message(&active_instance->kms_can, CAN_ID_VCU_STATUS,
                                &out_msg.buf.val);

  active_instance->send_inv(out_msg);
  active_instance->send_daq(out_msg);
}

void DataHandler::send_vcu_firmware_status_message(uint32_t on_time_seconds,
                                                   uint32_t fw_version,
                                                   bool project_is_dirty,
                                                   bool project_on_main) {
  if (active_instance == nullptr) {
    return;
  }

  encode_can_0x0c8_vcu_on_time_seconds(&active_instance->kms_can,
                                       on_time_seconds);
  encode_can_0x0c8_vcu_fw_version(&active_instance->kms_can, fw_version);
  encode_can_0x0c8_vcu_project_is_dirty(&active_instance->kms_can,
                                        project_is_dirty);
  encode_can_0x0c8_vcu_project_on_main(&active_instance->kms_can,
                                       project_on_main);

  can_message out_msg;
  out_msg.id = CAN_ID_VCU_FIRMWARE_VERSION;
  out_msg.length = pack_message(&active_instance->kms_can,
                                CAN_ID_VCU_FIRMWARE_VERSION, &out_msg.buf.val);

  active_instance->send_inv(out_msg);
  active_instance->send_daq(out_msg);
}

void DataHandler::send_vcu_power_tracking_message(uint32_t lifetime_distance,
                                                  double lifetime_ontime) {
  if (active_instance == nullptr) {
    return;
  }

  encode_can_0x0d0_vcu_lifetime_distance(&active_instance->kms_can,
                                         lifetime_distance);
  encode_can_0x0d0_vcu_lifetime_ontime(&active_instance->kms_can,
                                       lifetime_ontime);

  can_message out_msg;
  out_msg.id = CAN_ID_VCU_LIFETIME_DISTANCE_AND_ONTIME;
  out_msg.length =
      pack_message(&active_instance->kms_can,
                   CAN_ID_VCU_LIFETIME_DISTANCE_AND_ONTIME, &out_msg.buf.val);

  active_instance->send_inv(out_msg);
  active_instance->send_daq(out_msg);
}

// Pedal encoding methods
void DataHandler::send_pedal_travel_message(double apps1_travel,
                                            double apps2_travel,
                                            double brake_travel) {
  if (active_instance == nullptr) {
    return;
  }

  encode_can_0x0cc_vcu_apps1_travel(&active_instance->kms_can, apps1_travel);
  encode_can_0x0cc_vcu_apps2_travel(&active_instance->kms_can, apps2_travel);
  encode_can_0x0cc_vcu_bse1_travel(&active_instance->kms_can, brake_travel);

  can_message out_msg;
  out_msg.id = CAN_ID_VCU_PEDALS_TRAVEL;
  out_msg.length = pack_message(&active_instance->kms_can,
                                CAN_ID_VCU_PEDALS_TRAVEL, &out_msg.buf.val);

  active_instance->send_inv(out_msg);
  active_instance->send_daq(out_msg);
}

void DataHandler::send_pedal_raw_message(uint16_t raw_apps1, uint16_t raw_apps2,
                                         uint16_t raw_brake) {
  if (active_instance == nullptr) {
    return;
  }
  encode_can_0x0c4_APPS1(&active_instance->kms_can, raw_apps1);
  encode_can_0x0c4_APPS2(&active_instance->kms_can, raw_apps2);
  encode_can_0x0c4_BSE1(&active_instance->kms_can, raw_brake);

  can_message out_msg;
  out_msg.id = CAN_ID_VCU_PEDAL_READINGS;
  out_msg.length = pack_message(&active_instance->kms_can,
                                CAN_ID_VCU_PEDAL_READINGS, &out_msg.buf.val);

  active_instance->send_inv(out_msg);
  active_instance->send_daq(out_msg);
}

void DataHandler::data_handler_main_loop() {

  active_instance->process_acc_message();
  active_instance->process_inv_message();
  active_instance->process_daq_message();
}

void DataHandler::data_handler_200hz_loop() {}

void DataHandler::data_handler_10hz_loop() {}