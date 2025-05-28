#pragma once
#include <can_tools.hpp>
#include <car.h>

#include "accumulator.hpp"
#include "inverter.hpp"
#include "pedal_handeler.hpp"
#include "traction_control.hpp"

enum state {
  STARTUP = 0,                   // VCU is powering on
  TRACTIVE_SYSTEM_DISABLED = 1,  // GLV is on, but not TSV
  TRACTIVE_SYSTEM_ENERGIZED = 2, // TSV is up, but RTD button isn't pressed
  TRACTIVE_SYSTEM_ENABLED = 3,   // Enable everything required to go fast
  READY_TO_DRIVE = 4,            // Try not to hit a curb plz
  LAUNCH_WAIT = 5,               // Make sure everything is chill for launch
  LAUNCH = 6,                    // Accelerate, but faster
};

// TODO: Make this neatly documented somewhere in the readme or something, and
// add some guys to adjust the pedal settings and maybe use a negative value for
// persistant save or not? Anyways add more shit to this
enum parameter {
  POWER_LIMIT = 0,         // In KW
  TORQUE_LIMIT = 1,        // In Nm
  SPEED_MODE = 2,          // Boolean
  SPEED_LIMIT = 3,         // In some speed unit?
  INV_DISCHARGE_LIMIT = 4, // Need to find unit for this
  LAUNCH_MODE = 5,         // Look at traction_control.hpp for mode defs
  TRACTION_MODE = 6,       // Look at traction_control.hpp for mode defs
};

class VCU {
private:
  state current_state = STARTUP;
  uint8_t torque_mode = 0; // Legacy

  // TODO: Make the codes real and document some common failures
  uint16_t bool_code = 0;  // Encode all the possible gateing factors into a val
  uint16_t error_code = 0; // This gives us error codes when transitions fail
  bool buzzer_active = false;
  bool RTD_button_pressed = false;
  bool bspd_ok_hs = false;
  bool bspd_brake_high = false;
  bool bspd_current_high = false;

  bool (*timer_status_message)();
  bool (*timer_pedal_message)();
  bool (*timer_RTD_buzzer)();

  bool launch_state;
  uint8_t launch_mode = 0;

public:
  Pedals *pedals;
  Inverter *inverter;
  Accumulator *accumulator;
  TractionController *tc;

  can_obj_car_h_t *dbc;
  canMan *acc_can;
  canMan *inv_can;
  canMan *daq_can;

  VCU(Pedals *pedals, Inverter *inverter, Accumulator *accumulator,
      can_obj_car_h_t *dbc, canMan *acc_can, canMan *inv_can, canMan *daq_can,
      bool (*timer_status_message)(), bool (*timer_pedal_message)());

  inline void init_state_machine() { this->current_state = STARTUP; }

  inline state get_current_state() { return this->current_state; }
  inline uint16_t get_bool_code() { return this->bool_code; }
  inline uint16_t get_error_code() { return this->error_code; }
  inline bool get_launch_state() { return this->launch_state; }
  inline bool get_buzzer_state() { return this->buzzer_active; }
  inline bool get_bspd_ok_hs() { return this->bspd_ok_hs; }
  inline bool get_rtd_fella() { return this->RTD_button_pressed; }

  bool set_state(state target_state);
  bool try_ts_energized();
  bool try_ts_enabled();
  bool ts_safe();

  void update_acc_can();
  void update_inv_can();

  void set_parameter(uint64_t msg, uint8_t length);
  void update_dash_buttons(uint64_t msg, uint8_t length);
  void update_bspd(uint16_t raw_relay, uint16_t raw_current,
                   uint16_t raw_brake);

  void send_status_message();
  void send_pedal_raw_message(uint16_t raw_apps1, uint16_t raw_apps2,
                              uint16_t raw_brake);
  void send_pedal_travel_message();
  void send_firmware_status_message();
  void send_launch_control_status_message();
};
