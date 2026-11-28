#pragma once
// Traction control: kmslap's launch/traction controller (lib/vcu_control,
// vendored from python-data-analysis) between the pedal map and the
// inverter. It only ever takes torque away: what it returns is never more
// than the driver asked for.
//
// On or off by the driver (a dash button) or over CAN (vcu_set_parameter
// TRACTION_MODE). A change only takes effect off the throttle, so the torque
// never steps mid-corner. Without fresh VectorNav speed and motor speed
// there's no slip to control: it steps aside (driver torque straight
// through) and comes back by itself once they're back and the driver lifts.
// docs/traction-control.md.
#include <stddef.h>
#include <stdint.h>
#include <vcu_control.h>

class TractionControl {
public:
  // What the driver picked. The values are vcu_mode's.
  enum mode : uint8_t {
    OFF = VCU_MODE_PASSTHROUGH,
    SLIP = VCU_MODE_SLIP_PI,         // slip target, scaling the driver's torque
    GRIP_MAP = VCU_MODE_MU_ADAPTIVE, // learned grip map + slip trim
  };

  // Why it isn't controlling while on (status frame bits)
  enum reason : uint8_t {
    TC_NO_VECTORNAV = 1 << 0,   // no ground speed
    TC_NO_MOTOR_SPEED = 1 << 1, // no rear wheel speed
    TC_GEAR_RATIO = 1 << 2,     // the controller's own check: final drive is wrong
                             // in the config, or a sensor is. Until re-selected
    TC_WAITING_FOR_LIFT = 1 << 3, // a change waits for the driver to lift
  };

  TractionControl();
  // Room for the controller's state (vcu_state_size(), tested)
  static constexpr size_t STATE_BYTES = 512;

  // Inputs, as CAN frames arrive
  void wheel_rpm(int wheel, float rpm); // VCU_FL..VCU_RR
  void vectornav_velocity(float north, float east, uint32_t now_ms);
  void vectornav_accel(float forward);
  void motor_rpm(float rpm, uint32_t now_ms);
  void motor_torque(float nm);

  // The driver's or the pits' choice. select() takes any mode; toggle()
  // goes between OFF and the last mode that wasn't (SLIP to begin with).
  void select(uint8_t requested);
  void toggle();

  // Every pass of the loop. Returns the torque to command: driver_nm, or
  // less while it's controlling.
  double step(uint32_t now_ms, double driver_nm);

  mode selected() const { return requested; }
  mode active() const { return running; } // OFF while stepped aside
  uint8_t reasons() const;
  float slip() const { return out.slip; }
  float speed() const { return out.v_est; }
  float scale() const { return running == OFF ? 1.0f : out.scale; }
  float grip() const { return out.learned.mu; }

private:
  void restart(uint32_t now_ms, mode m);
  bool inputs_fresh(uint32_t now_ms) const;

  // The controller's state: its size is only known to vcu_control.cpp
  alignas(8) uint8_t state[STATE_BYTES];
  vcu_inputs_t in{};
  vcu_outputs_t out{};
  vcu_learned_t learned{};

  mode requested = OFF;
  mode last_on = SLIP;
  mode running = OFF;
  bool started = false;
  bool lift_needed = false;
  bool gear_fault = false;
  uint32_t last_now_ms = 0;

  bool vn_heard = false;
  uint32_t vn_at_ms = 0;
  bool motor_heard = false;
  uint32_t motor_at_ms = 0;
};
