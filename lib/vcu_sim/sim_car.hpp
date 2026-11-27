#pragma once
// A whole VCU wired up exactly like main.hpp does it, sitting on the simulated
// buses, plus helpers that play the part of the other boards on the car (ACU,
// inverter, dash) by packing real DBC frames.
#include <car.h>

#include <accumulator.hpp>
#include <cascadia_pm100.hpp>
#include <vehicle.hpp>
#include <ota.hpp>
#include <parameters.hpp>
#include <pedal_handeler.hpp>
#include <vcu.hpp>

#include "sim.hpp"
#include "sim_ota.hpp"

namespace sim {

#if VEHICLE_HAS_CASCADIA
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
                END_ACCELERATOR_PEDAL_1,   MAX_ACCELERATOR_PEDAL_1,
                START_ACCELERATOR_PEDAL_2, END_ACCELERATOR_PEDAL_2,
                MAX_ACCELERATOR_PEDAL_2};

  CascadiaPM100 inverter{SPIN_DIR, &inv_can, &daq_can, &dbc, -0.69314718056};
  Inverter *const inverters[1] = {&inverter};
  Drivetrain drivetrain{inverters, vehicle::THIS.drives,
                        vehicle::THIS.drive_count};
  Accumulator accumulator{&dbc, &acc_can};
  VCU vcu{&pedals, &drivetrain, &accumulator, &dbc, &acc_can, &inv_can, &daq_can};

  // Firmware update, on the DAQ bus like on the car. The whole 8 MB flash
  // chip, which starts out as a fresh USB flash: slot A, version 11111111.
  NorFlash chip{8 * 1024 * 1024};
  Board board{chip};
  ota::Service updater{board, board.staging(), &daq_can};

  // Raw ADC counts fed to the pedals every tick, defaulting to released
  // pedals that are inside their valid ranges
  uint16_t apps1_raw = START_ACCELERATOR_PEDAL_1;
  uint16_t apps2_raw = START_ACCELERATOR_PEDAL_2;
  uint16_t brake_raw = START_BRAKE_PEDAL;
  uint16_t steering_raw = 2048;

  Car() {
    board.install(boot::SLOT_A, std::vector<uint8_t>(1000, 0x5A), 0x11111111);
    board.power_on();
    board.safe = [this] { return vcu.safe_to_program(); };
    vcu.attach_updater(&updater);
    vcu.begin();
  }

  Bus &acc() { return bus(TEENSY_CAN1); }
  Bus &inv() { return bus(TEENSY_CAN2); }
  Bus &daq() { return bus(TEENSY_CAN3); }

  // Every state change seen at the end of a tick, with the time it happened
  struct Transition {
    uint32_t at_ms;
    state to;
  };
  std::vector<Transition> transitions;

  // One pass of loop(), same order as main.cpp
  void tick() {
    repeat_status();
    pedals.update_travel(apps1_raw, apps2_raw, brake_raw, sim::now());
    pedals.update_steering(steering_raw);
    vcu.update_bspd(0, 0, 0);
    vcu.tick(sim::now());

    state s = vcu.get_current_state();
    if (transitions.empty() || transitions.back().to != s)
      transitions.push_back({sim::now(), s});
  }

  // Run the loop at 1 kHz for `ms` of simulated time
  void run_for(uint32_t ms) {
    for (uint32_t i = 0; i < ms; i++) {
      tick();
      sim::advance(1);
    }
  }

  // Run until the VCU reaches `target`, giving up after `timeout_ms`
  bool run_until(state target, uint32_t timeout_ms) {
    uint32_t deadline = sim::now() + timeout_ms;
    while (sim::now() < deadline) {
      tick();
      sim::advance(1);
      if (vcu.get_current_state() == target)
        return true;
    }
    return false;
  }

  bool ever_reached(state s) const {
    for (const auto &t : transitions)
      if (t.to == s)
        return true;
    return false;
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
  void inverter_bus_current(double amps);
  void dash_rtd_button(bool pressed);
  void dash_tc_button(bool pressed);
  void inverter_torque_feedback(double nm);
  // On the DAQ bus, where the logger and the wheel speed nodes are; the VCU
  // takes them from any bus
  void vectornav(double north, double east, double accel_forward);
  void wheel_rpm(uint16_t fl, uint16_t fr, uint16_t rl, uint16_t rr);
  // A frame from the Pi on the DAQ bus, as is (pi_frame::encode makes a
  // good one)
  void pi_frame(const uint8_t *data, uint8_t len = 8);
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
  // Discharge limit in the most recent 0x202 on the inverter bus, -1 if none
  int last_discharge_limit();

  //
  //// A laptop on the DAQ bus doing UDS over ISO-TP. Deliberately its own
  //// ISO-TP implementation, not the firmware's, so the two check each other.

  // Send one UDS request and wait for the response. Empty on timeout.
  std::vector<uint8_t> uds(const std::vector<uint8_t> &request,
                           uint32_t timeout_ms = 2000);

  // The whole update sequence, like tools/ota/flash.py does it. Returns the
  // first step that failed, or nullptr if the image was applied.
  const char *flash(const std::vector<uint8_t> &image);

  // Like the real boards, the ACU and inverter repeat their status every
  // 100 ms once a test has set it. Clear these to see what the VCU does when
  // one dies or its CAN wire breaks.
  bool acu_talking = true;
  bool inverter_talking = true;

private:
  can_obj_car_h_t other_boards{}; // separate DBC state, like a real node
  void send(Bus &b, uint32_t id);

  std::vector<uint32_t> repeating; // status messages sent at least once
  uint32_t last_repeat_ms = 0;
  void repeat_status();

  void tester_send(const std::vector<uint8_t> &request, size_t *cursor,
                   uint32_t timeout_ms);
  bool tester_wait_frame(size_t *cursor, can_message *out, uint32_t timeout_ms);
  void tester_inject(const uint8_t *bytes, size_t len);
};

#endif // VEHICLE_HAS_CASCADIA

// The development signing key, keys/ota-dev.key. Its public half is what
// the native builds trust (scripts/ota_key.py), like the bench firmware.
extern const uint8_t DEV_SIGNING_SEED[32];

// An OTA image in the same format tools/ota/image.py makes, signed with
// `seed` (the dev key unless a test wants a stranger's). Built for slot B
// unless said otherwise: a fresh board runs A, so that's where its first
// update goes.
std::vector<uint8_t> make_image(
    const std::vector<uint8_t> &firmware, uint32_t version,
    const char *target = ota::IMAGE_TARGET,
    uint32_t load_address = boot::slot_firmware_address(boot::SLOT_B),
    const uint8_t *seed = DEV_SIGNING_SEED);

} // namespace sim
