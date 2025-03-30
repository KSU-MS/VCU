#include "vcu.hpp"
#include "core_pins.h"

VCU::VCU(PEDALS *pedals, CM200 *inverter, ACCUMULATOR *accumulator,
         can_obj_car_h_t *dbc, canMan *acc_can, canMan *inv_can,
         canMan *daq_can, bool (*timer_status_message)()) {
  this->pedals = pedals;
  this->inverter = inverter;
  this->accumulator = accumulator;

  this->acc_can = acc_can;
  this->inv_can = inv_can;
  this->daq_can = daq_can;

  this->dbc = dbc;

  this->timer_status_message = timer_status_message;
}

bool VCU::try_ts_energized() { return false; }

bool VCU::try_ts_enabled() { return false; }

bool VCU::ts_safe() { return false; }

bool VCU::set_state(state target_state) {
  switch (current_state) {
  case STARTUP: // This is just a catch for evil starts
    if (target_state == TRACTIVE_SYSTEM_DISABLED) {
      this->current_state = TRACTIVE_SYSTEM_DISABLED;
      return true;
    } else {
      this->error_code = this->bool_code;
      this->current_state = STARTUP;
      return false;
    }
    break;

  case TRACTIVE_SYSTEM_DISABLED:
    return false;
    break;

  case TRACTIVE_SYSTEM_PRECHARGING:
    return false;
    break;

  case TRACTIVE_SYSTEM_ENERGIZED:
    return false;
    break;

  case TRACTIVE_SYSTEM_ENABLED:
    if (target_state == READY_TO_DRIVE) {
      // Check to make sure we can actually enter READY_TO_DRIVE

      return true;
    } else {
      this->error_code = this->bool_code;

      // Disable everything and go back to TRACTIVE_SYSTEM_DISABLED

      this->current_state = TRACTIVE_SYSTEM_DISABLED;
      return false;
    }
    break;

  case READY_TO_DRIVE: // We want to be able to leave no matter what
    this->error_code = this->bool_code;

    // Disable everything

    this->current_state = TRACTIVE_SYSTEM_DISABLED;
    return true;
    break;

  default:
    this->error_code = this->bool_code;
    this->current_state = TRACTIVE_SYSTEM_DISABLED;
    return false;
    break;
  }
}

void VCU::send_pedal_message() {
  encode_can_0x0cc_vcu_apps1_travel(dbc, pedals->get_apps1_travel());
  encode_can_0x0cc_vcu_apps2_travel(dbc, pedals->get_apps2_travel());
  encode_can_0x0cc_vcu_bse1_travel(dbc, pedals->get_brake_travel());

  // Init and pack the message
  can_message out_msg;
  out_msg.id = CAN_ID_VCU_PEDALS_TRAVEL;
  out_msg.length =
      pack_message(dbc, CAN_ID_VCU_PEDALS_TRAVEL, &out_msg.buf.val);

  daq_can->send_controller_message(out_msg);
}

void VCU::send_status_message() {
  encode_can_0x0c3_VCU_ACCEL_BRAKE_IMPLAUSIBLE(
      dbc, pedals->get_apps_bse_fault_ok_low());
  encode_can_0x0c3_VCU_ACCEL_IMPLAUSIBLE(dbc, pedals->get_apps_fault_ok_low());
  encode_can_0x0c3_VCU_BRAKE_IMPLAUSIBLE(dbc, pedals->get_bse_fault_ok_low());
  encode_can_0x0c3_VCU_BRAKE_ACTIVE(dbc,
                                    bool(pedals->get_brake_travel() > 0.3));
  encode_can_0x0c3_VCU_BSPD_BRAKE_HIGH(dbc, bspd_brake_high);
  encode_can_0x0c3_VCU_BSPD_CURRENT_HIGH(dbc, bspd_current_high);
  encode_can_0x0c3_VCU_BSPD_OK_HIGH(dbc, is_bspd_chill());
  encode_can_0x0c3_VCU_BMS_OK_HIGH(dbc, accumulator->get_bms_ok_hs());
  encode_can_0x0c3_VCU_IMD_OK_HIGH(dbc, accumulator->get_imd_ok_hs());
  encode_can_0x0c3_VCU_SHUTDOWN_B_OK_HIGH(dbc, 0.0); // What
  encode_can_0x0c3_VCU_SHUTDOWN_C_OK_HIGH(dbc, 0.0); // The
  encode_can_0x0c3_VCU_SHUTDOWN_D_OK_HIGH(dbc, 0.0); // Fuck
  encode_can_0x0c3_VCU_SHUTDOWN_E_OK_HIGH(dbc, 0.0); // Are these????
  encode_can_0x0c3_VCU_SOFTWARE_OK_HIGH(dbc, true);
  encode_can_0x0c3_VCU_ACTIVATE_BUZZER(dbc, buzzer_active);
  encode_can_0x0c3_VCU_SOFTWARE_OK(dbc, true);
  encode_can_0x0c3_VCU_DISTANCE_TRAVELLED(dbc, 0.0);
  encode_can_0x0c3_VCU_ENERGY_METER_PRESENT(dbc, false);
  encode_can_0x0c3_VCU_INVERTER_POWERED(dbc, inverter->get_inverter_enable());
  encode_can_0x0c3_VCU_LAUNCH_CONTROL_ACTIVE(dbc, 0);
  encode_can_0x0c3_VCU_MAX_TORQUE(dbc, inverter->get_torque_limit());
  encode_can_0x0c3_VCU_TORQUE_MODE(dbc, torque_mode);
  encode_can_0x0c3_VCU_STATEMACHINE_STATE(dbc, current_state);

  // Init and pack the message
  can_message out_msg;
  out_msg.id = CAN_ID_VCU_STATUS;
  out_msg.length = pack_message(dbc, CAN_ID_VCU_STATUS, &out_msg.buf.val);

  daq_can->send_controller_message(out_msg);
}

// These values are provided by the python script ran by the lib_dep
// https://github.com/KSU-MS/pio-git-hash-gen
#ifndef AUTO_VERSION
#warning "AUTO_VERSION was not defined by the generator!"
#define AUTO_VERSION 0xdeadbeef
#endif

#ifndef FW_PROJECT_IS_DIRTY
#warning "FW_PROJECT_IS_DIRTY was not defined by the generator!"
#define FW_PROJECT_IS_DIRTY 1
#endif

#ifndef FW_PROJECT_IS_MAIN_OR_MASTER
#warning "FW_PROJECT_IS_MAIN_OR_MASTER was not defined by the generator!"
#define FW_PROJECT_IS_MAIN_OR_MASTER 0
#endif

void VCU::send_firmware_status_message() {
  if (timer_status_message) {
    // BUG: Get rid of this arduino call for time
    encode_can_0x0c8_vcu_on_time_seconds(dbc, millis() / 1000);
    encode_can_0x0c8_vcu_fw_version(dbc, AUTO_VERSION);
    encode_can_0x0c8_vcu_project_is_dirty(dbc, FW_PROJECT_IS_DIRTY);
    encode_can_0x0c8_vcu_project_on_main(dbc, FW_PROJECT_IS_MAIN_OR_MASTER);

    can_message out_msg;
    out_msg.id = CAN_ID_VCU_FIRMWARE_VERSION;
    out_msg.length =
        pack_message(dbc, CAN_ID_VCU_FIRMWARE_VERSION, &out_msg.buf.val);

    daq_can->send_controller_message(out_msg);
  }
}
