#include "inverter.hpp"
#include "parameters.hpp"
#include <cmath>
#include <cstdint>

Inverter::Inverter(bool spin_forward, double gear_ratio)
    : spin_forward(spin_forward), gear_ratio(gear_ratio) {}

void Inverter::regulate_power(uint32_t now_ms) {
  // 1/0.90 undoes the open-loop efficiency guess: the full limit at the
  // shaft, the most it could ever be with a lossless drivetrain
  constexpr double max_trim = 1.0 / 0.90;
  constexpr double min_trim = 0.1;
  // Back up from a cut to the full limit in about two seconds
  constexpr double recover_per_ms = (max_trim - min_trim) / 2000.0;
  // Samples older than this don't count
  constexpr uint32_t stale_ms = 100;

  const double target_w = power_limit_kw * 1000.0 * POWER_LIMIT_TARGET;

  if (power_sample_new) {
    power_sample_new = false;
    const uint32_t since = power_sample_ever ? now_ms - power_sample_ms : 0;
    power_sample_ever = true;
    power_sample_ms = now_ms;

    const double measured_w = get_measured_power_w();
    if (std::isfinite(measured_w) && measured_w > target_w) {
      // Torque cap ~ trim, so this puts the next sample on the target. Only
      // per new sample: the measurement lags, so cutting again every loop
      // until it catches up would overshoot
      power_trim *= target_w / measured_w;
    } else if (std::isfinite(measured_w)) {
      power_trim += recover_per_ms * since;
    }
  }

  if (!power_sample_ever || now_ms - power_sample_ms > stale_ms) {
    // Flying blind: no more than the open-loop cap
    if (power_trim > 1.0)
      power_trim = 1.0;
  }

  if (power_trim > max_trim)
    power_trim = max_trim;
  if (power_trim < min_trim || !std::isfinite(power_trim))
    power_trim = min_trim;
}

void Inverter::calculate_motor_distance_M(uint32_t time_msec) {
  uint32_t time_elaped_msec = time_msec - time_last_msec;

  // Lifetime distance is an odometer, so reversing counts too. A negative
  // total would also be UB when get_motor_distance_M casts it to uint32_t
  double velocity_Msec =
      (std::abs(double(motor_rpm)) / 60 / gear_ratio) * WHEEL_CIRCUMFRANCE_M;

  distance_M += (double(time_elaped_msec) / 1000) * velocity_Msec;

  time_last_msec = time_msec;
}

void Inverter::command_torque(double torque_request) {

  // TODO: Get rid of these arduino calls
  // We apply an exponential decay function to the torque_request if we go over
  // whatever power_limit is set, this could probably be optimized
  // if ((bus_voltage * bus_current) >= power_limit && !over_power) {
  //  over_power_event_epoch = millis();
  //  over_power = true;
  //} else if ((bus_voltage * bus_current) < power_limit) {
  //  over_power = false;
  //}

  // if (over_power) {
  //   torque_target =
  //       torque_target *
  //       pow(2.71828182846, (over_power_decay_factor *
  //                           ((millis() - over_power_event_epoch) / 1000.0)));
  //
  //   torque_target = 0;
  // }

  // Clamp to make sure this can only de-rate torque, just in case
  // if (torque_target > torque_request)
  //  torque_target = torque_request;

  // open loop power limit algo with a efficency factor
  // torque  = power * efficency / angular velcity

  constexpr double pi = 3.141592653589793;
  constexpr double efficiency = 0.90; // this the drivetrain efficiency value

  const double angular_velocity =
      std::abs(static_cast<double>(motor_rpm)) * 2.0 * pi / 60;

  const double power_limit_w = static_cast<double>(power_limit_kw) * 1000.0;

  double allowed_torque = torque_limit_nm;

  // avoiding div by zero
  if (angular_velocity > 0.0) {
    const double power_limited_torque =
        (power_limit_w * efficiency * power_trim) / angular_velocity;

    if (power_limited_torque < allowed_torque) {
      allowed_torque = power_limited_torque;
    }
  }

  double torque_target = 0.0;

  if (inverter_enable && power_limit_kw > 0 && std::isfinite(torque_request) &&
      std::isfinite(allowed_torque) && allowed_torque > 0.0 &&
      torque_request > 0.0) {

    torque_target = torque_request;

    if (torque_target > allowed_torque) {
      torque_target = allowed_torque;
    }
  }

  send_torque(torque_target);
}
