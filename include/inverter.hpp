#pragma once

#include "parameters.hpp"

#include <FlexCAN_T4.h>
#include <car.h>

#include "Metro.h"
#include "QuickPID.h"
#include "data.hpp"
#include "data_handler.hpp"

// Inverter gets special treatment in the state machine because its a black box
// that requires specific configuration and communication.

class Inverter {
private:
  DataHandler *data_handler;

  bool spin_forward = true;
  bool inverter_enable = false;
  bool inverter_discharge = false;

  bool speed_mode = false;
  double torque_adjustment = 0;

  const int torque_kp_x100 = 300; // tune this variable
  const int torque_ki_x100 = 1;   // tune this variable
  const int torque_kd_x100 = 1;
  const int speed_kp_x100 = 2; // tune this variable
  const int speed_ki_x100 = 0; // tune this variable

  double power_output_w = 0.0;

  double torque_over_nm = 0.0;
  double power_over_w = 0.0;
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

  inline bool get_inverter_enable() { return inverter_enable; }

  uint16_t get_instant_current_limit(float voltage) {
    return static_cast<uint16_t>(
        (as<double>(params->at(POWER_LIMIT_ID)) * 1000.0) / voltage);
  }
  inline void set_inverter_enable(bool enable) { inverter_enable = enable; }
  inline void calculate_pid_loop() { torquepid.Compute(); }
  inline void set_pid_parameters(int kp, int ki, int kd) {
    torquepid.SetTunings(double(kp) / 100.0, double(ki) / 100.0,
                         double(kd) / 100.0);
  }
  void set_current_limits(uint16_t charge_limit, uint16_t discharge_limit);

  void calculate_motor_distance_M(uint32_t time_msec);
  void calculate_power_output();

  void ping();
  void send_clear_faults();
  void command_torque(double torque_request);
  void command_speed(int16_t speed_request);
  void set_command_mode_to_torque();
  void set_command_mode_to_speed();

  void set_inv_parameter(uint16_t param_address, uint32_t param_data);
  void read_inv_parameter(uint16_t param_address);

  void inverter_main_loop();
  void inverter_200hz_loop();
  void inverter_10hz_loop();
  void inverter_1hz_loop();
};
