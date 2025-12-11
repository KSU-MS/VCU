#pragma once
#include <can_tools.hpp>
#include <car.h>

#include "accumulator.hpp"
#include "data.hpp"
#include "inverter.hpp"
#include "parameters.hpp"
#include "pedal_handeler.hpp"
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

class VCU {
private:
  state current_state = STARTUP;
  uint8_t torque_mode = 0; // Legacy

  // TODO: Make the codes real and document some common failures
  uint16_t bool_code = 0;  // Encode all the possible gateing factors into a val
  uint16_t error_code = 0; // This gives us error codes when transitions fail
  bool buzzer_active = false;
  bool bspd_ok_hs = false;
  bool bspd_brake_high = false;
  bool bspd_current_high = false;

  bool (*timer_status_message)();
  bool (*timer_pedal_message)();
  bool (*timer_RTD_buzzer)();
  bool (*timer_inverter_ping)();
  bool (*timer_inverter_command)();
  bool (*timer_current_limit)();
  void (*reset_timer_current_limit)();

  bool launch_state;
  uint8_t launch_mode = 0;

  VehicleData *vehicle_data;

public:
  Pedals *pedals;
  Inverter *inverter;
  Accumulator *accumulator;
  TractionController *tc;
  std::array<parameter, 25> *params;

  VCU(Pedals *pedals, Inverter *inverter, Accumulator *accumulator,
      std::array<parameter, 25> *params, VehicleData *vehicle_data);

  inline void init_state_machine() { this->current_state = STARTUP; }

  inline state get_current_state() { return this->current_state; }
  inline uint16_t get_bool_code() { return this->bool_code; }
  inline uint16_t get_error_code() { return this->error_code; }
  inline bool get_launch_state() { return this->launch_state; }
  inline bool get_buzzer_state() { return this->buzzer_active; }
  inline bool get_bspd_ok_hs() { return this->bspd_ok_hs; }
  inline bool get_rtd_fella() {
    return vehicle_data ? vehicle_data->driver.rtd_button_pressed : false;
  }

  void handle_state_machine();

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
};
