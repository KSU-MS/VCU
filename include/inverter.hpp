#pragma once

#include <FlexCAN_T4.h>
#include <car.h>

#include "Metro.h"
#include "QuickPID.h"
#include "data_handler.hpp"
#include "vehicle.hpp"

// Inverter gets special treatment in the state machine because its a black box
// that requires specific configuration and communication.

class Inverter {
private:
  DataHandler *data_handler;

  double torque_adjustment = 0;

  const int torque_kp_x100 = 300; // tune this variable
  const int torque_ki_x100 = 1;   // tune this variable
  const int torque_kd_x100 = 1;
  const int speed_kp_x100 = 2; // tune this variable
  const int speed_ki_x100 = 0; // tune this variable

  uint32_t power_output_w = 0;

  double torque_over_nm = 0.0;
  uint32_t power_over_w = 0;
  double angular_vel_over_rad_s = 0.0;

  QuickPID torquepid;

  bool (*timer_mc_kick)();
  bool (*timer_current_limit)();
  bool (*timer_motor_controller_send)();

  Metro *timer_overpower_decay;

  std::array<Parameter, 25> *params;
  VehicleData *vehicle_data;

  enum inv_param_address : uint16_t {
    Motor_Overspeed_EEPROM_RPM = 0x006F,
    Max_Speed_EEPROM_RPM = 0x0080,
    Break_Speed_EEPROM_RPM = 0x007F,
    Speed_Rate_Limit_EEPROM_RPM_per_s = 0x00A9,
  };

public:
  Inverter(bool spin_direction, std::array<Parameter, 25> *params,
           VehicleData *vehicle_data);

  uint16_t get_instant_current_limit(float voltage) {
    return static_cast<uint16_t>((this->params->at(POWER_LIMIT_ID).value) /
                                 voltage);
  }
  inline void calculate_pid_loop() { torquepid.Compute(); }
  inline void set_pid_parameters(uint32_t kp_x100, uint32_t ki_x100,
                                 uint32_t kd_x100) {
    torquepid.SetTunings(double(kp_x100) / 100.0, double(ki_x100) / 100.0,
                         double(kd_x100) / 100.0);
  }
  void set_inverter_current_limits(uint16_t charge_limit,
                                   uint16_t discharge_limit);

  void calculate_motor_distance_M(uint32_t time_msec);
  void calculate_power_output();

  void ping();
  void send_clear_faults();
  void request_torque(double torque_request);
  void command_torque();

  void set_inv_parameter(uint16_t param_address, uint32_t param_data);
  void read_inv_parameter(uint16_t param_address);

  void inverter_main_loop();
  void inverter_200hz_loop();
  void inverter_10hz_loop();
  void inverter_1hz_loop();
};
