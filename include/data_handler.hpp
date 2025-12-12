#pragma once
#include "parameters.hpp"
#include <FlexCAN_T4.h>
#include <can_tools.hpp>
#include <car.h>
#include <cstdint>

class StateMachine;
class Accumulator;
class Inverter;
struct VehicleData;

class data_handler {
private:
  static data_handler *active_instance;

  canMan acc_can = canMan(TEENSY_CAN1, ACCUMULATOR_CAN_BAUD_RATE);
  canMan inv_can = canMan(TEENSY_CAN2, INVERTER_CAN_BAUD_RATE);
  canMan daq_can = canMan(TEENSY_CAN3, DAQ_CAN_BAUD_RATE);
  can_obj_car_h_t kms_can;
  StateMachine *state_machine = nullptr;
  VehicleData *vehicle_data = nullptr;

  void send_acc_impl(const can_message &msg);
  void send_inv_impl(const can_message &msg);
  void send_daq_impl(const can_message &msg);

public:
  data_handler(StateMachine *state_machine, VehicleData *vehicle_data);
  void start_comms(void);
  void process_acc_message(void);
  void process_inv_message(void);
  void process_daq_message(void);
  void send_1hz_messages(void);
  void send_20hz_messages(void);
  void send_200hz_messages(void);

  static void send_acc(const can_message &msg);
  static void send_inv(const can_message &msg);
  static void send_daq(const can_message &msg);
  static void send_inv_and_daq(const can_message &msg);

  // Inverter encoding methods
  static void send_inverter_ping(bool spin_forward, bool inverter_enable,
                                 bool inverter_discharge);
  static void send_inverter_torque_command(double torque_target);
  static void send_inverter_speed_command(int16_t speed_request,
                                          bool spin_forward, bool speed_mode,
                                          bool inverter_enable,
                                          bool inverter_discharge,
                                          double max_torque);
  static void send_inverter_current_limits(uint16_t charge_limit,
                                           uint16_t discharge_limit);
  static void send_inverter_parameter(uint16_t param_address,
                                      uint32_t param_data);
  static void send_inverter_clear_faults();

  // VCU encoding methods
  static void send_vcu_status_message(bool bspd_brake_high,
                                      bool bspd_current_high, bool bspd_ok_hs,
                                      bool bms_ok_hs, bool imd_ok_hs,
                                      bool buzzer_active, bool inverter_enable,
                                      double max_torque, uint8_t torque_mode,
                                      int current_state);
  static void send_vcu_firmware_status_message(uint32_t on_time_seconds,
                                               uint32_t fw_version,
                                               bool project_is_dirty,
                                               bool project_on_main);
  static void send_vcu_power_tracking_message(uint32_t lifetime_distance,
                                              double lifetime_ontime);

  // Pedal encoding methods
  static void send_pedal_travel_message(double apps1_travel,
                                        double apps2_travel,
                                        double brake_travel);
  static void send_pedal_raw_message(uint16_t raw_apps1, uint16_t raw_apps2,
                                     uint16_t raw_brake);

  static void data_handler_main_loop();
  static void data_handler_200hz_loop();
  static void data_handler_10hz_loop();
};
