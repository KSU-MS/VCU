#pragma once
#include <can_tools.hpp>
#include <car.h>

#include "data.hpp"
#include "inverter.hpp"
#include "parameters.hpp"
#include "traction_control.hpp"
#include <array>

enum state {
  STARTUP = 0,                   // VCU is powering on
  TRACTIVE_SYSTEM_DISABLED = 1,  // GLV is on, but not TSV
  TRACTIVE_SYSTEM_ENERGIZED = 2, // TSV is up, but RTD button isn't pressed
  TRACTIVE_SYSTEM_ENABLED = 3,   // Enable everything required to go fast
  READY_TO_DRIVE = 4,            // Try not to hit a curb plz
  LAUNCH_WAIT = 5,               // Make sure everything is chill for launch
  LAUNCH = 6,                    // Accelerate, but faster
};

class StateMachine {
private:
  VehicleData *vehicle_data;

public:
  Inverter *inverter;
  TractionController *tc;
  std::array<parameter, 25> *params;

  StateMachine(Inverter *inverter, std::array<parameter, 25> *params,
               VehicleData *vehicle_data);

  inline void init_state_machine() {
    vehicle_data->state_machine.current_state =
        StateMachineData::state::STARTUP;
  }

  bool set_state(state target_state);
  bool try_ts_energized();
  bool try_ts_enabled();
  bool ts_safe();

  void set_parameter(uint8_t target_parameter, uint32_t parameter_value);
  void update_bspd(uint16_t raw_relay, uint16_t raw_current,
                   uint16_t raw_brake);

  void send_status_message();
  void send_firmware_status_message();
  void send_launch_control_status_message();

  void send_power_tracking_message();

  // TODO: Add additional pedal maps with diffrent curves?
  inline double get_torque_request(double throttle_travel, double max_torque) {
    return throttle_travel * max_torque;
  };

  void state_machine_main_loop();
  void state_machine_200hz_loop();
  void state_machine_10hz_loop();
  void state_machine_1hz_loop();
};
