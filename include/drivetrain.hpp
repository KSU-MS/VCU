#pragma once
// Every motor's inverter, as one thing the VCU drives (docs/vehicles.md).
// On rwd_emrax that's the one Cascadia and nothing changes; on the hub
// motor cars, several.
//
// The torque request is in the drivetrain's own units, max_request() at
// full pedal: N m at the motor on rwd_emrax (as it always was, so traction
// control and the Pi's ceiling are unchanged), N m at the wheels, all of
// them together, on the hub cars.
//
// The 80 kW limit is shared out by wheel-torque capability, each inverter
// keeping to its share on its own measured power, so the total stays
// under the limit; the pack's current limits are shared the same way.
#include <stddef.h>
#include <stdint.h>

#include "freshness.hpp"
#include "inverter.hpp"
#include "torque_split.hpp"
#include "vehicle.hpp"

class Drivetrain {
public:
  static constexpr size_t MAX_DRIVES = 4;

  // inverters[i] drives drives[i]
  Drivetrain(Inverter *const *inverters, const vehicle::Drive *drives,
             size_t count);

  // A frame from the inverters' bus. Which inverter it was, if any
  struct Heard {
    uint8_t feedback; // Inverter::feedback bits, NOT_MINE if nobody's
    size_t index;
  };
  Heard on_frame(const can_message &msg, uint32_t now_ms);

  // Each inverter's DC voltage heard within timeout_ms
  bool all_heard(uint32_t now_ms, uint32_t timeout_ms) const;
  // The lowest DC voltage any of them reports (all precharged?) and the
  // highest (any of it live?)
  double min_bus_voltage();
  double max_bus_voltage();

  void set_enable(bool enable);
  bool any_enabled();
  // Torque limits on (each motor's own) or off
  void set_torque_limits(bool on);
  // For the status frame: the first drive's
  uint8_t torque_limit_nm();
  void set_power_limit_kw(double total_kw);
  // The pack's discharge limit as power over its voltage, each inverter its
  // share (in ready to drive)
  void set_current_limits(uint16_t charge_limit, float pack_voltage);
  // A fixed pack discharge limit, amps, shared the same way
  void set_current_limit_amps(uint16_t charge_limit, uint16_t discharge_limit);

  void ping();
  void send_clear_faults();
  void regulate_power(uint32_t now_ms);

  // Full pedal, in the drivetrain's units (above)
  double max_request();
  // The driver's (or traction control's, or the Pi's) request, in those
  // units, out to the motors: on the hub cars split by torque_split
  void command(double request);
  // For the split by axle load: speed (m/s) and acceleration (m/s^2)
  void set_motion(double vx, double ax);
  void set_split(torque_split::Mode mode, double front_bias);
  // What the last command put on each drive's wheel(s), N m (hub cars)
  double wheel_torque(size_t i) const { return wheel[i]; }

  // The rear axle's motor, if the car has one (traction control's input)
  Inverter *rear_axle();
  // The odometer: the first drive's
  void calculate_distance(uint32_t now_ms);
  uint32_t distance_m();

  size_t count() const { return n; }
  Inverter &inverter(size_t i) { return *inverters[i]; }
  const vehicle::Drive &drive(size_t i) const { return drives[i]; }

private:
  // Each drive's wheel-torque capability as a share of the total
  double share(size_t i) const;
  double torque_limit_for(size_t i) const;
  // What drive i can put on its wheel(s) now: its torque limit, or its
  // motor's torque curve at its speed if that's less
  double wheel_cap(size_t i);

  Inverter *inverters[MAX_DRIVES];
  const vehicle::Drive *drives;
  size_t n;
  Freshness voltage_heard[MAX_DRIVES];
  double wheel[MAX_DRIVES] = {};
  double vx = 0, ax = 0;
  torque_split::Mode split_mode = torque_split::Mode::LOAD;
  double front_bias = 0.45;
};
