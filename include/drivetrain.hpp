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
// The 80 kW limit: on rwd_emrax, the Cascadia's own loop on its measured
// power, as it always was. On the hub cars, one loop on all the inverters'
// measured power together (power_loop.hpp); when it binds, every motor's
// torque is scaled by the same factor, so the split stands and the power
// goes wherever the split puts the torque. Each inverter's DC current limit
// (the pack's discharge limit, or the power limit over the pack voltage)
// follows its share of the commanded power, so the inverters themselves
// also hold the total.
#include <stddef.h>
#include <stdint.h>

#include "freshness.hpp"
#include "inverter.hpp"
#include "power_loop.hpp"
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
  // The total, for all the motors together
  void set_power_limit_kw(double total_kw);
  double power_limit_kw() const { return limit_kw; }
  // The motors' peak powers added up: the most the power limit may be set
  // to. POWER_LIMIT_KW if any motor's isn't known
  double peak_power_kw() const;
  // On the hub cars: the share of the open-loop power cap the total's loop
  // allows now (rwd_emrax: the Cascadia's get_power_trim)
  double power_trim() const { return total_power.trim(); }
  // The power limit over the pack voltage, each inverter its share (in ready
  // to drive)
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
  // Each drive's share of the power the last command asked for, mixed with
  // its capability share so none is ever left with nothing
  double power_share(size_t i) const;
  double torque_limit_for(size_t i) const;
  // What drive i can put on its wheel(s) now: its torque limit, or its
  // motor's torque curve at its speed if that's less
  double wheel_cap(size_t i);

  Inverter *inverters[MAX_DRIVES];
  const vehicle::Drive *drives;
  size_t n;
  Freshness voltage_heard[MAX_DRIVES];
  double wheel[MAX_DRIVES] = {};
  double motor_nm[MAX_DRIVES] = {}; // what the last command sent each motor
  double limit_kw = 0;
  // The hub cars' loop on their total DC power, and when each inverter's
  // DC current was last heard (all of them fresh, or it's flying blind)
  PowerLoop total_power;
  Freshness dc_heard[MAX_DRIVES];
  bool dc_new = false;
  double vx = 0, ax = 0;
  torque_split::Mode split_mode = torque_split::Mode::LOAD;
  double front_bias = 0.45;
};
