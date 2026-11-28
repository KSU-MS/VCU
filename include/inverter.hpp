#pragma once
// What the VCU needs from a motor's inverter, whichever one it is. The
// motor-independent part lives here: the torque clamps, the 80 kW power
// limit (open loop on motor speed, trimmed on measured DC power), the
// current-limit arithmetic and the odometer. The wire protocol is a
// subclass's: CascadiaPM100 (cascadia_pm100.hpp) on the car today; DTI and
// AMK for the hub motor cars (docs/vehicles.md).
#include <can_tools.hpp>
#include <cmath>
#include <cstdint>

#include "power_loop.hpp"

class Inverter {
public:
  // `spin_forward`: the direction command for positive torque.
  // `gear_ratio`: motor revs per wheel rev, for the odometer
  Inverter(bool spin_forward, double gear_ratio);
  virtual ~Inverter() = default;

  // What a frame from the inverter told us (on_frame's result, bits)
  enum feedback : uint8_t {
    NOT_MINE = 0,
    MOTOR_SPEED = 1 << 0,
    DC_CURRENT = 1 << 1,
    DC_VOLTAGE = 1 << 2,
    MOTOR_TORQUE = 1 << 3,
    STATUS = 1 << 4, // anything else of its own: it's alive
  };
  // A frame from the inverter's bus: decode it if it's this inverter's
  virtual uint8_t on_frame(const can_message &msg) = 0;

  // Keep it alive but making no torque (while not ready to drive)
  virtual void ping() = 0;
  virtual void send_clear_faults() = 0;
  // Pack current limits, A. For inverters that take them; others ignore it
  virtual void set_current_limits(uint16_t charge_limit,
                                  uint16_t discharge_limit) {
    (void)charge_limit;
    (void)discharge_limit;
  }

  // The torque the driver (or traction control) asks for: clamped to the
  // torque limit and the power limit, zero unless enabled and sane, then
  // sent (send_torque). Drive only, no regen yet
  void command_torque(double torque_request);

  inline uint8_t get_torque_limit() { return uint8_t(torque_limit_nm); }
  inline double get_torque_limit_nm() { return torque_limit_nm; }
  inline bool get_inverter_enable() { return inverter_enable; }
  inline double get_bus_voltage() { return bus_voltage; }
  inline int16_t get_motor_rpm() { return motor_rpm; }
  inline double get_bus_current() { return bus_current; }
  inline double get_motor_torque() { return motor_torque; }
  inline uint32_t get_motor_distance_M() { return distance_M; }
  uint16_t get_instant_current_limit(float voltage) {
    // No pack voltage yet (or garbage): allow no discharge current rather than
    // dividing by zero. Float->int of inf/NaN is UB, and on the Teensy it
    // saturates, which would mean no limit at all
    if (!(voltage > 0))
      return 0;

    float limit = (power_limit_kw * 1000) / voltage;
    if (limit >= UINT16_MAX)
      return UINT16_MAX;
    return uint16_t(limit);
  }

  inline void set_torque_limit(double limit) { torque_limit_nm = limit; }
  inline void set_speed_limit(uint16_t limit) { speed_limit = limit; }
  inline void set_power_limit_kw(double limit) { power_limit_kw = limit; }
  inline double get_power_limit_kw() { return power_limit_kw; }
  inline void set_inverter_enable(bool enable) { inverter_enable = enable; }

  void calculate_motor_distance_M(uint32_t time_msec);

  // The power loop (power_loop.hpp) on this inverter's own measured DC
  // power: on rwd_emrax, where it's the only one. Call every loop. (The hub
  // motor cars run one loop on all their inverters instead, in Drivetrain.)
  void regulate_power(uint32_t now_ms);
  inline double get_power_trim() { return power.trim(); }
  inline double get_measured_power_w() { return bus_voltage * bus_current; }

protected:
  // Put the command on the wire: `torque_nm` already clamped (0 when
  // disabled), with the enable, direction and torque limit as they are now
  virtual void send_torque(double torque_nm) = 0;

  // Feedback, filled in by on_frame. A new DC current sample is what
  // regulate_power works from
  void new_dc_current(double amps) {
    bus_current = amps;
    power_sample_new = true;
  }

  bool spin_forward = false;
  bool inverter_enable = false;
  bool inverter_discharge = false;

  bool speed_mode = false;
  int16_t speed_limit = 0;
  double torque_limit_nm = 0;

  int16_t motor_rpm = 0;
  double motor_torque = 0;
  double bus_voltage = 0;
  double bus_current = 0;

private:
  double gear_ratio;
  uint32_t time_last_msec = 0;
  double distance_M = 0;

  double power_limit_kw = 0;

  // Closed loop on measured power (regulate_power): a trim on the power the
  // open-loop torque cap allows
  PowerLoop power;
  bool power_sample_new = false;
};
