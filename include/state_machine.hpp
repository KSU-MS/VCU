#pragma once
#include <can_tools.hpp>
#include <car.h>

#include "inverter.hpp"
#include "traction_control.hpp"
#include "vehicle.hpp"
#include <array>

class StateMachine {
private:
  VehicleData *vehicle_data;

public:
  Inverter *inverter;
  TractionController *tc;
  std::array<Parameter, 25> *params;

  StateMachine(Inverter *inverter, std::array<Parameter, 25> *params,
               VehicleData *vehicle_data);

  inline void init_state_machine() {
    vehicle_data->state_machine.current_state =
        StateMachineData::state::STARTUP;
  }

  bool set_state(StateMachineData::state target_state);
  bool try_ts_energized();
  bool try_ts_enabled();
  bool ts_safe();

  void update_bspd(uint16_t raw_relay, uint16_t raw_current,
                   uint16_t raw_brake);

  void send_status_message();
  void send_firmware_status_message();
  void send_launch_control_status_message();

  void send_power_tracking_message();

  // TODO: Add additional pedal maps with diffrent curves?
  inline double get_inverter_request(double throttle_travel,
                                     double max_torque) {
    return throttle_travel * max_torque;
  };

  void state_machine_main_loop();
  void state_machine_200hz_loop();
  void state_machine_10hz_loop();
  void state_machine_1hz_loop();
};
