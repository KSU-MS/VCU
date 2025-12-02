#pragma once

<<<<<<< HEAD
#include <can_tools.hpp>
#include <car.h>
=======
#include <Metro.h>
#include <QuickPID.h>
#include <can_tools.hpp>
#include <car.h>
#include <parameters.hpp>
>>>>>>> d343a79 (cmake build toolchain init)

class Inverter {
private:
  uint32_t time_last_msec = 0;

  bool spin_forward = true;
  bool inverter_enable = false;
  bool inverter_discharge = false;

  bool speed_mode = false;
  int16_t speed_request = 0;
  int16_t speed_limit = 0;
  double torque_request = 0;
  double torque_limit_nm = 0;

  int16_t motor_rpm;
  uint16_t motor_temp;
  double distance_M;

  bool over_power;
  uint32_t over_power_event_epoch;
  float over_power_decay_factor;
  double bus_voltage;
  double bus_current;
  uint16_t power_limit_kw;

  bool (*timer_mc_kick)();
  bool (*timer_current_limit)();
  bool (*timer_motor_controller_send)();

  canMan *can;
  canMan *daq_can;
  can_obj_car_h_t *dbc;

<<<<<<< HEAD
public:
  Inverter(bool (*timer_mc_kick)(), bool (*timer_current_limit)(),
           bool (*timer_motor_controller_send)(), bool spin_direction,
           canMan *can, canMan *daq_can, can_obj_car_h_t *dbc,
           float over_power_decay_factor);
=======
  enum inv_param_address : uint16_t {
    Motor_Overspeed_EEPROM_RPM = 0x006F,
    Max_Speed_EEPROM_RPM = 0x0080,
    Break_Speed_EEPROM_RPM = 0x007F,
    Speed_Rate_Limit_EEPROM_RPM_per_s = 0x00A9,
  };

public:
  Inverter(bool (*timer_mc_kick)(), bool (*timer_current_limit)(),
           bool (*timer_motor_controller_send)(), bool spin_direction,
           std::array<parameter, 25> *params, canMan *can, canMan *daq_can,
           can_obj_car_h_t *dbc, float over_power_decay_factor);
>>>>>>> d343a79 (cmake build toolchain init)

  inline uint8_t get_torque_limit() { return uint8_t(torque_limit_nm); }
  inline bool get_inverter_enable() { return inverter_enable; }
  inline double get_bus_voltage() { return bus_voltage; }
  inline double get_bus_current() { return bus_current; }
<<<<<<< HEAD
  inline uint32_t get_motor_distance_M() { return distance_M; }
  uint16_t get_instant_current_limit(float voltage) {
    return ((power_limit_kw * 1000) / voltage);
  }

  inline void set_torque_limit(double limit) { torque_limit_nm = limit; }
  inline void set_speed_limit(uint16_t limit) { speed_limit = limit; }
  inline void set_power_limit_kw(uint16_t limit) { power_limit_kw = limit; }
  inline void set_inverter_enable(bool enable) { inverter_enable = enable; }
=======
  inline double get_power_output_kw() { return (power_output); }
  inline double get_motor_distance_M() { return distance_M; }
  uint16_t get_instant_current_limit(float voltage) {
    return static_cast<uint16_t>(
        ((*params)[POWER_LIMIT].parameter_value * 1000.0) / voltage);
  }

  inline void set_torque_limit(double limit) {
    (*params)[MAX_TORQUE].parameter_value = static_cast<uint64_t>(limit * 10.0);
  }
  inline void set_speed_limit(uint16_t limit) { speed_limit = limit; }
  inline void set_power_limit_kw(uint16_t limit_kw) {
    (*params)[POWER_LIMIT].parameter_value =
        static_cast<uint64_t>(limit_kw * 10);
  }
  inline void set_inverter_enable(bool enable) { inverter_enable = enable; }
  inline void calculate_pid_loop() { torquepid.Compute(); }
  inline void set_pid_parameters(int kp, int ki, int kd) {
    torquepid.SetTunings(double(kp) / 100.0, double(ki) / 100.0,
                         double(kd) / 100.0);
  }
  inline int get_pid_kp() { return torque_kp_x100; }
  inline int get_pid_ki() { return torque_ki_x100; }
  inline int get_pid_kd() { return torque_kd_x100; }
>>>>>>> d343a79 (cmake build toolchain init)
  void set_current_limits(uint16_t charge_limit, uint16_t discharge_limit);

  void update_bus_current(uint64_t msg_in, uint8_t length);
  void update_bus_voltage(uint64_t msg_in, uint8_t length);
  void update_motor_feedback(uint64_t msg_in, uint8_t length);

  void calculate_motor_distance_M(uint32_t time_msec);

  void ping();
  void send_clear_faults();
  void command_torque(double torque_request);
  void command_speed(int16_t speed_request);
};
