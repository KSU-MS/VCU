#pragma once

#include <can_tools.hpp>
#include <parameters.hpp>
#include <car.h>
#include <Metro.h>
#include <QuickPID.h>

class Inverter
{
private:
  uint32_t time_last_msec = 0;

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

  double torque_over_nm = 0.0;
  double power_over_w = 0.0;

  double angular_vel_over_rad_s = 0.0;

  QuickPID torquepid;

  int16_t motor_rpm;
  uint16_t motor_temp;
  double distance_M;

  double bus_voltage;
  double bus_current;
  double power_output;

  bool (*timer_mc_kick)();
  bool (*timer_current_limit)();
  bool (*timer_motor_controller_send)();

  Metro *timer_overpower_decay;

  std::array<parameter, 25> *params;

  canMan *inv_can;
  canMan *daq_can;
  can_obj_car_h_t *dbc;

  enum inv_param_address : uint16_t
  {
    Motor_Overspeed_EEPROM_RPM = 0x006F,
    Max_Speed_EEPROM_RPM = 0x0080,
    Break_Speed_EEPROM_RPM = 0x007F,
    Speed_Rate_Limit_EEPROM_RPM_per_s = 0x00A9,
  };

public:
  Inverter(bool (*timer_mc_kick)(), bool (*timer_current_limit)(),
           bool (*timer_motor_controller_send)(), bool spin_direction, std::array<parameter, 25> *params,
           canMan *can, canMan *daq_can, can_obj_car_h_t *dbc);

  inline bool get_inverter_enable() { return inverter_enable; }
  inline double get_bus_voltage() { return bus_voltage; }
  inline double get_bus_current() { return bus_current; }
  inline double get_power_output_kw() { return (power_output); }
  inline double get_motor_distance_M() { return distance_M; }
  uint16_t get_instant_current_limit(float voltage)
  {
    return static_cast<uint16_t>(((*params)[POWER_LIMIT].parameter_value * 1000.0) / voltage);
  }

  inline void set_torque_limit(double limit)
  {
    (*params)[MAX_TORQUE].parameter_value = static_cast<uint64_t>(limit * 10.0);
  }
  inline void set_soft_speed_limit(uint16_t limit) { (*params)[SOFT_RPM_LIMIT].parameter_value = static_cast<double>(limit); }
  inline void set_hard_speed_limit(uint16_t limit) { (*params)[MAX_RPM_LIMIT].parameter_value = static_cast<double>(limit); }
  inline void set_power_limit_kw(uint16_t limit) { (*params)[POWER_LIMIT].parameter_value = static_cast<double>(limit) * 10.0; }
  inline void set_inverter_enable(bool enable) { inverter_enable = enable; }
  inline void calculate_pid_loop() { torquepid.Compute(); }
  inline void set_pid_parameters(int kp, int ki, int kd) { torquepid.SetTunings(double(kp) / 100.0, double(ki) / 100.0, double(kd) / 100.0); }
  inline int get_pid_kp() { return torque_kp_x100; }
  inline int get_pid_ki() { return torque_ki_x100; }
  inline int get_pid_kd() { return torque_kd_x100; }
  void set_current_limits(uint16_t charge_limit, uint16_t discharge_limit);

  void update_bus_current(uint64_t msg_in, uint8_t length);
  void update_bus_voltage(uint64_t msg_in, uint8_t length);
  void update_motor_feedback(uint64_t msg_in, uint8_t length);

  void calculate_motor_distance_M(uint32_t time_msec);
  void calculate_power_output();

  void ping();
  void send_clear_faults();
  void command_torque(double torque_request);
  void command_speed(int16_t speed_request);

  void set_inv_parameter(uint16_t param_address, uint32_t param_data);
  void read_inv_parameter(uint16_t param_address);
};
