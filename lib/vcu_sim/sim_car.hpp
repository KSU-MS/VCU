#pragma once
// A whole VCU wired up exactly like main.hpp does it, sitting on the simulated
// buses, plus helpers that play the part of the other boards on the car (ACU,
// inverter, dash) by packing real DBC frames.
#include <car.h>

#include <accumulator.hpp>
#include <inverter.hpp>
#include <parameters.hpp>
#include <pedal_handeler.hpp>
#include <vcu.hpp>

#include "sim.hpp"

namespace sim {

// Constructed before the canMan members so the buses are clean by the time
// canMan's constructor "initializes" them
struct ResetFirst {
  ResetFirst() { sim::reset(); }
};

struct Car : ResetFirst {
  can_obj_car_h_t dbc{};

  canMan acc_can{TEENSY_CAN1, ACCUMULATOR_CAN_BAUD_RATE};
  canMan inv_can{TEENSY_CAN2, INVERTER_CAN_BAUD_RATE};
  canMan daq_can{TEENSY_CAN3, DAQ_CAN_BAUD_RATE};

  Pedals pedals{MIN_BRAKE_PEDAL,           START_BRAKE_PEDAL,
                END_BRAKE_PEDAL,           MAX_BRAKE_PEDAL,
                MIN_APPS_PEDAL,            START_ACCELERATOR_PEDAL_1,
                END_ACCELERATOR_PEDAL_1,   START_ACCELERATOR_PEDAL_2,
                END_ACCELERATOR_PEDAL_2};

  Inverter inverter{SPIN_DIR, &inv_can, &daq_can, &dbc, -0.69314718056};
  Accumulator accumulator{&dbc, &acc_can};
  VCU vcu{&pedals, &inverter, &accumulator, &dbc, &acc_can, &inv_can, &daq_can};

  // Raw ADC counts fed to the pedals every tick, defaulting to released
  // pedals that are inside their valid ranges
  uint16_t apps1_raw = START_ACCELERATOR_PEDAL_1;
  uint16_t apps2_raw = START_ACCELERATOR_PEDAL_2;
  uint16_t brake_raw = START_BRAKE_PEDAL;
  uint16_t steering_raw = 2048;

  Car() { vcu.begin(); }

  Bus &acc() { return bus(TEENSY_CAN1); }
  Bus &inv() { return bus(TEENSY_CAN2); }
  Bus &daq() { return bus(TEENSY_CAN3); }

  // One pass of loop(), same order as main.cpp
  void tick() {
    pedals.update_travel(apps1_raw, apps2_raw, brake_raw);
    pedals.update_steering(steering_raw);
    vcu.update_bspd(0, 0, 0);
    vcu.tick(sim::now());
  }

  // Run the loop at 1 kHz for `ms` of simulated time
  void run_for(uint32_t ms) {
    for (uint32_t i = 0; i < ms; i++) {
      tick();
      sim::advance(1);
    }
  }

  //
  //// Pedals, as fractions of travel (0..1)
  void set_throttle(double travel) {
    apps1_raw = uint16_t(START_ACCELERATOR_PEDAL_1 +
                         travel * (END_ACCELERATOR_PEDAL_1 -
                                   START_ACCELERATOR_PEDAL_1));
    apps2_raw = uint16_t(START_ACCELERATOR_PEDAL_2 +
                         travel * (END_ACCELERATOR_PEDAL_2 -
                                   START_ACCELERATOR_PEDAL_2));
  }
  void set_brake(double travel) {
    brake_raw = uint16_t(START_BRAKE_PEDAL +
                         travel * (END_BRAKE_PEDAL - START_BRAKE_PEDAL));
  }

  //
  //// Other boards on the car
  void acu_status(bool imd_ok, bool bms_ok);
  void precharge_state(uint8_t state);
  void pack_power(double volts, double amps);
  void inverter_bus_voltage(double volts);
  void inverter_motor_rpm(int16_t rpm);
  void dash_rtd_button(bool pressed);
  void set_parameter(uint8_t target, uint32_t value);

  // Everything the rest of the car needs to see for the VCU to let TS go live
  void make_ts_healthy() {
    acu_status(true, true);
    precharge_state(PRECHARGE_OK_STATE);
    pack_power(450, 0);
    inverter_bus_voltage(450);
  }

  // Walk the car from boot to READY_TO_DRIVE the way a driver would
  void drive_to_rtd();

  //
  //// Decoded views of what the VCU put on the wire
  struct InverterCommand {
    double torque_nm;
    double torque_limit_nm;
    bool enable;
    bool direction;
  };
  // Most recent 0x0C0 on the inverter bus, false if none sent yet
  bool last_inverter_command(InverterCommand *out);
  // State reported in the most recent 0x0C3 status frame on the DAQ bus
  int last_reported_state();

private:
  can_obj_car_h_t other_boards{}; // separate DBC state, like a real node
  void send(Bus &b, uint32_t id);
};

} // namespace sim
