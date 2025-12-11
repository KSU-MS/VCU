#pragma once

#include <cstdint>

struct AccumulatorData {
  uint8_t precharge_state = 0;
  bool imd_ok_hs = false;
  bool bms_ok_hs = false;
  double pack_voltage = 0.0;
  double pack_current = 0.0;
  double consumed_power_wh = 0.0;
  uint32_t last_energy_calc_timestamp_ms = 0;
};

struct InverterData {
  double bus_voltage = 0.0;
  double bus_current = 0.0;
  int16_t motor_rpm = 0;
  double motor_distance_m = 0.0;
  double power_output_w = 0.0;
  uint32_t last_distance_calc_timestamp_ms = 0;
};

struct DriverInterfaceData {
  bool rtd_button_pressed = false;
};

struct PedalData {
  uint16_t raw_apps1 = 0;
  uint16_t raw_apps2 = 0;
  uint16_t raw_brake = 0;
  uint16_t raw_bspd = 0;

  double brake_travel = 0.0;
  double apps1_travel = 0.0;
  double apps2_travel = 0.0;
  double throttle_travel = 0.0;

  bool bspd_ok_hs =
      false; // BSPD (Brake system pressure detector) high side fault
  bool bse_fault = false;      // BSE (Brake sensor encoder) fault
                               // outside of valid range (brake sensor encoder)
  bool apps_fault = false;     // APPS (Accelerator pedal position sensor) fault
                               // (outside of valid range)
  bool apps_bse_fault = false; // APPS and BSE simultaneous input fault

  // TODO: implement these faults
  bool bspd_brake_high = false;   // BSPD (Brake system plausability device)
                                  // brake high side fault (not real)
  bool bspd_current_high = false; // BSPD (Brake system plausability device)
                                  // current high side fault (not real)
};

struct VehicleData {
  AccumulatorData accumulator;
  InverterData inverter;
  DriverInterfaceData driver;
  PedalData pedals;
};
