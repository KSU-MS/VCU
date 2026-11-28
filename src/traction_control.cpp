#include "traction_control.hpp"
#include "car_model.hpp"
#include "parameters.hpp"
#include <math.h>

namespace {

// kmslap's FirmwareController.config() for this car (python-data-analysis
// 40e0100), the values its accel sim swept and checked, except what the VCU
// knows better: final drive, tire radius and torque limit come from
// parameters.hpp, and wheel speeds come over CAN (ws_teeth = 0).
vcu_config_t config(uint8_t mode) {
  vcu_config_t c{};
  c.mode = mode;

  c.mass = float(car_model::MASS_KG);
  c.cg_height = float(car_model::CG_HEIGHT_M);
  c.wheelbase = float(car_model::WHEELBASE_M);
  c.cg_to_front = float(car_model::CG_TO_FRONT_M);
  c.tire_radius = float(WHEEL_CIRCUMFRANCE_M / 6.283185307179586);
  c.final_drive = float(GEAR_RATIO);
  c.driveline_eff = 0.95f;
  // Wheels and driveline 0.21 (kmslap), plus the EMRAX 228's rotor through
  // the final drive (kmslap's 0.5889 is for its 3.0)
  c.rear_inertia = float(0.21 + 0.0421 * GEAR_RATIO * GEAR_RATIO);
  c.cla = float(car_model::CLA);
  c.cda = float(car_model::CDA);
  c.rho = float(car_model::RHO);
  c.crr = float(car_model::CRR);
  c.aero_front = float(car_model::AERO_FRONT);
  c.torque_peak = float(MAX_TORQUE_LIMIT_NM);

  c.ws_teeth = 0;
  c.ws_min_window_us = 10000;
  c.ws_timeout_us = TC_INPUT_TIMEOUT_MS * 1000;
  c.ws_valid_speed = 3.0f;
  c.vn_vel_gain = 0.2f;
  c.ws_vel_gain = 0.05f;
  c.accel_filter_tau = 0.02f;

  c.radius_cal_speed = 5.0f;
  c.radius_cal_rate = 1.0f;
  c.radius_tolerance = 0.05f;
  c.coast_torque = 2.0f;
  c.accel_bias_rate = 2.0f;

  c.control_period_us = 5000;
  c.slip_target = 0.10f;
  c.slip_speed_floor = 1.0f;
  // Swept per mode in kmslap with CAN rates, latency and inverter lag
  if (mode == VCU_MODE_MU_ADAPTIVE) {
    c.kp = 0.1f;
    c.ki = 0.3f;
    c.recover_gain = 1.0f;
  } else {
    c.kp = 0.05f;
    c.ki = 0.3f;
    c.recover_gain = 3.0f;
  }
  c.scale_max = 1.5f;
  c.mu_init = 1.2f;
  c.mu_min = 0.6f;
  c.mu_max = 2.5f;
  c.mu_adapt_rate = 1.0f;
  c.mu_transfer_rate = 1.0f;
  c.torque_track_tol = 5.0f;

  c.ratio_check_revs = 2.0f;
  c.ratio_tolerance = 0.08f;
  c.ratio_fail_count = 2;
  return c;
}

} // namespace

TractionControl::TractionControl() {}

void TractionControl::wheel_rpm(int wheel, float rpm) {
  if (wheel < 0 || wheel > 3 || !isfinite(rpm))
    return;
  in.ws_rpm[wheel] = rpm;
  in.ws_rpm_new[wheel] = 1;
}

void TractionControl::vectornav_velocity(float north, float east,
                                         uint32_t now_ms) {
  if (!isfinite(north) || !isfinite(east))
    return;
  // Ground speed. The car doesn't reverse, and in a corner it's the speed
  // along the path the rear tires slip against, near enough
  in.vn_vel_x = sqrtf(north * north + east * east);
  in.vn_new = 1;
  vn_heard = true;
  vn_at_ms = now_ms;
}

void TractionControl::vectornav_accel(float forward) {
  if (isfinite(forward))
    in.vn_accel_x = forward;
}

void TractionControl::motor_rpm(float rpm, uint32_t now_ms) {
  if (!isfinite(rpm))
    return;
  in.motor_rpm = rpm;
  in.motor_rpm_new = 1;
  motor_heard = true;
  motor_at_ms = now_ms;
}

void TractionControl::motor_torque(float nm) {
  if (!isfinite(nm))
    return;
  in.motor_torque_fb = nm;
  in.motor_torque_new = 1;
}

void TractionControl::select(uint8_t m) {
  if (m != OFF && m != SLIP && m != GRIP_MAP)
    return;
  requested = mode(m);
  if (requested != OFF)
    last_on = requested;
  gear_fault = false; // a fresh choice gets a fresh check
}

void TractionControl::toggle() { select(requested == OFF ? last_on : OFF); }

bool TractionControl::inputs_fresh(uint32_t now_ms) const {
  return vn_heard && now_ms - vn_at_ms <= TC_INPUT_TIMEOUT_MS &&
         motor_heard && now_ms - motor_at_ms <= TC_INPUT_TIMEOUT_MS;
}

uint8_t TractionControl::reasons() const {
  if (requested == OFF)
    return 0;
  uint8_t r = 0;
  if (!vn_heard || last_now_ms - vn_at_ms > TC_INPUT_TIMEOUT_MS)
    r |= TC_NO_VECTORNAV;
  if (!motor_heard || last_now_ms - motor_at_ms > TC_INPUT_TIMEOUT_MS)
    r |= TC_NO_MOTOR_SPEED;
  if (gear_fault)
    r |= TC_GEAR_RATIO;
  if (lift_needed)
    r |= TC_WAITING_FOR_LIFT;
  return r;
}

void TractionControl::restart(uint32_t now_ms, mode m) {
  // What it learned so far (radii, accel bias, grip) carries over
  if (started)
    learned = out.learned;
  if (vcu_state_size() > sizeof(state)) {
    running = OFF; // can't happen with the vendored controller (tested)
    return;
  }
  const vcu_config_t c = config(m);
  vcu_init(state, &c, started ? &learned : nullptr, now_ms * 1000u);
  out.faults = 0; // the old controller's, not this one's
  started = true;
  running = m;
}

double TractionControl::step(uint32_t now_ms, double driver_nm) {
  last_now_ms = now_ms;
  if (!started)
    restart(now_ms, OFF);

  // The controlling one's check, not the passthrough one's that runs while
  // it's off (it would latch again straight after being chosen)
  if (running != OFF && (out.faults & VCU_FAULT_GEAR_RATIO))
    gear_fault = true; // the controller fell back to passthrough itself

  const mode want =
      (requested != OFF && inputs_fresh(now_ms) && !gear_fault) ? requested
                                                                : OFF;
  const bool off_throttle = !(driver_nm > 0);
  lift_needed = false;
  if (want != running) {
    if (want == OFF && requested != OFF) {
      // Lost what it needs: it can't control blind, so the driver gets
      // their torque back at once, like a car without TC
      restart(now_ms, OFF);
    } else if (off_throttle) {
      restart(now_ms, want);
    } else {
      lift_needed = true;
    }
  }

  in.now_us = now_ms * 1000u;
  in.driver_torque_nm = isfinite(driver_nm) ? float(driver_nm) : 0.0f;
  vcu_step(state, &in, &out);
  in.motor_rpm_new = in.motor_torque_new = in.vn_new = 0;
  for (auto &n : in.ws_rpm_new)
    n = 0;

  if (running == OFF || !isfinite(out.torque_cmd_nm))
    return driver_nm;
  // Only ever less than the driver asked for, whatever the controller says
  const double cmd = out.torque_cmd_nm;
  if (cmd < 0)
    return 0;
  return cmd < driver_nm ? cmd : driver_nm;
}
