#pragma once
#include <can_tools.hpp>
#include <car.h>
#include <cstdint>

#include "accumulator.hpp"
#include "drivetrain.hpp"
#include "freshness.hpp"
#include "inverter.hpp"
#include "pedal_handeler.hpp"
#include "periodic.hpp"
#include "pi_supervisor.hpp"
#include "traction_control.hpp"

namespace ota {
class Service;
}

enum state {
  STARTUP = 0,                   // VCU is powering on
  TRACTIVE_SYSTEM_DISABLED = 1,  // GLV is on, but not TSV
  TRACTIVE_SYSTEM_ENERGIZED = 2, // TSV is up, but RTD button isn't pressed
  TRACTIVE_SYSTEM_ENABLED = 3,   // Enable everything required to go fast
  READY_TO_DRIVE = 4,            // Try not to hit a curb plz
  // 5 and 6 were LAUNCH_WAIT and LAUNCH, stubs that never ran. A launch is
  // now traction control from a standstill (traction_control.hpp)
  // Something's wrong (IMD or BMS tripped, the ACU or inverter gone quiet)
  // and there may still be high voltage: no torque, no ready to drive, and
  // fault_code() says why. Until it clears or the voltage is gone.
  TRACTIVE_SYSTEM_FAULT = 7,
};

// fault_code() bits: why the tractive system can't be trusted
enum fault : uint8_t {
  FAULT_ACU_SILENT = 1 << 0,      // ACU status or precharge not heard lately
  FAULT_INVERTER_SILENT = 1 << 1, // inverter voltage not heard lately
  FAULT_IMD = 1 << 2,
  FAULT_BMS = 1 << 3,
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
  LAUNCH_MODE = 5,         // Retired: launches are TRACTION_MODE's job
  TRACTION_MODE = 6,       // TractionControl::mode: 0 off, 1 slip, 2 grip map
};

class VCU {
private:
  state current_state = STARTUP;

  // The ACU's and inverter's status, as of when it was last heard: ts_safe()
  // only trusts it while it keeps arriving (SAFETY_DATA_TIMEOUT_MS)
  uint32_t now_ms = 0;
  Freshness acu_status_heard;
  Freshness precharge_heard;
  uint8_t torque_mode = 0; // Legacy

  // TODO: Make the codes real and document some common failures
  uint16_t bool_code = 0;  // Encode all the possible gateing factors into a val
  uint16_t error_code = 0; // This gives us error codes when transitions fail
  bool buzzer_active = false;
  bool RTD_button_pressed = false;
  bool bspd_ok_hs = false;
  bool bspd_brake_high = false;
  bool bspd_current_high = false;

  bool tc_button_was_down = false;
  double driver_nm = 0; // the pedal map's request this pass
  // VectorNav's speed and forward acceleration, for the split by axle load
  Freshness vn_heard;
  double vn_speed = 0, vn_accel = 0;
  double torque_nm = 0; // after traction control and the Pi's ceiling

  Periodic timer_1s = Periodic(1000);    // Used for VCU status message
  Periodic timer_10hz = Periodic(100);   // Used for inverter ping
  Periodic timer_10hz_2 = Periodic(100); // Used for inverter current limit
  Periodic timer_20hz = Periodic(50);    // Used for pedal/DAQ messages
  Periodic timer_200hz = Periodic(5);    // Used for inverter command message
  Periodic timer_100hz = Periodic(10);   // Used for the Pi's status frame

  state state_last_tick = STARTUP; // To spot transitions in tick()
  uint32_t state_entered_ms = 0;

  ota::Service *updater = nullptr; // Firmware update over the DAQ bus

public:
  Pedals *pedals;
  Drivetrain *drivetrain;
  Accumulator *accumulator;
  TractionControl traction;
  PiSupervisor pi;

  can_obj_car_h_t *dbc;
  canMan *acc_can;
  canMan *inv_can;
  canMan *daq_can;

  VCU(Pedals *pedals, Drivetrain *drivetrain, Accumulator *accumulator,
      can_obj_car_h_t *dbc, canMan *acc_can, canMan *inv_can, canMan *daq_can);

  inline void init_state_machine() { this->current_state = STARTUP; }

  // Pin setup + defaults, called once from setup()
  void begin();
  // One pass of the main loop (CAN, periodic messages, state machine), with
  // the sensor readings already pushed into pedals/update_bspd
  void tick(uint32_t now);

  inline state get_current_state() { return this->current_state; }
  inline uint16_t get_bool_code() { return this->bool_code; }
  inline uint16_t get_error_code() { return this->error_code; }
  inline bool get_buzzer_state() { return this->buzzer_active; }
  inline bool get_bspd_ok_hs() { return this->bspd_ok_hs; }
  inline bool get_rtd_fella() { return this->RTD_button_pressed; }

  bool set_state(state target_state);
  bool try_ts_energized();
  bool try_ts_enabled();
  // Everything's fine for the tractive system to be (or go) live. No side
  // effects.
  bool ts_safe();
  // The ACU and inverter are still talking: their last status is recent
  // enough to go on
  bool safety_data_fresh();
  // FAULT_* bits for what's wrong now, 0 if nothing is
  uint8_t fault_code();
  // There may be high voltage outside the accumulator: the inverter says so,
  // or isn't saying anything
  bool hv_maybe_present();
  // Leave a live state because ts_safe() failed: to TRACTIVE_SYSTEM_FAULT if
  // that's a fault and there may still be high voltage, else back to
  // TRACTIVE_SYSTEM_DISABLED
  void drop_out();

  void update_acc_can();
  void update_inv_can();
  // Frames traction control needs that could come on any bus (VectorNav,
  // wheel speed nodes)
  void update_vehicle_sensors(const can_message &msg);
  void update_daq_can(uint32_t now);

  // OTA is optional, the VCU runs the same without it
  inline void attach_updater(ota::Service *service) { updater = service; }
  // Parked: no tractive system and the inverter can't make torque
  bool safe_to_program();

  void set_parameter(uint64_t msg, uint8_t length);
  void update_dash_buttons(uint64_t msg, uint8_t length);
  void update_bspd(uint16_t raw_relay, uint16_t raw_current,
                   uint16_t raw_brake);

  void send_status_message();
  void send_pedal_raw_message(uint16_t raw_apps1, uint16_t raw_apps2,
                              uint16_t raw_brake);
  void send_steering_raw_message(uint16_t raw_steering);
  void send_pedal_travel_message();
  void send_brake_pressure_message(uint16_t raw_brake);
  void send_firmware_status_message();
  void send_traction_control_message();
  void send_pi_status_message();

  void send_power_tracking_message();
};
