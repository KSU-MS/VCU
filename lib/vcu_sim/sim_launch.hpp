#pragma once
// A straight-line launch for the traction control tests: the car, its
// driven rear wheels and tires, and the sensors that report them on CAN at
// about their real rates (inverter 100 Hz, wheel speed nodes 100 Hz,
// VectorNav 50 Hz). Much simpler than kmslap's model, which is where the
// controller is tuned: this is to check the VCU wires it up right (units,
// IDs, which way the torque goes), not to tune it.
#include <cmath>

#include "sim_car.hpp"

namespace sim {

struct Launch {
  // The surface: peak friction coefficient
  double mu = 1.0;
  // Sensors on (a test can turn one off to see what TC does without it)
  bool vectornav_on = true;
  bool wheel_speeds_on = true;
  // What the rear wheel speed nodes report, as a multiple of the truth: a
  // wrong final drive in the VCU looks the same to the ratio check
  double rear_wheel_sensor_gain = 1.0;

  // State
  double v = 0;      // m/s
  double omega = 0;  // rear wheel rad/s
  double torque = 0; // Nm the motor is making
  double accel = 0;  // m/s^2

  // Car, close to kmslap's
  static constexpr double mass = 285.0;
  static constexpr double g = 9.81;
  static constexpr double cg_height = 0.2751;
  static constexpr double wheelbase = 1.61544;
  static constexpr double cg_to_front = 0.80772;
  // At the axle: wheels and driveline (kmslap), plus the EMRAX 228's rotor
  // through the final drive
  static constexpr double rear_inertia = 0.21 + 0.0421 * GEAR_RATIO * GEAR_RATIO;
  static constexpr double driveline_eff = 0.95;
  static constexpr double cda = 1.3, rho = 1.226;
  static constexpr double inverter_lag_s = 0.010;
  double radius() const { return WHEEL_CIRCUMFRANCE_M / (2 * M_PI); }

  // Longitudinal slip, with the same 1 m/s floor the controller uses
  double slip() const {
    return (omega * radius() - v) / std::max(v, 1.0);
  }
  double motor_rpm() const { return omega * GEAR_RATIO * 60 / (2 * M_PI); }

  // Pacejka-ish: peaks at slip ~0.11 at mu x the rear load
  double tire_force(double s, double fz) const {
    return mu * fz * std::sin(1.9 * std::atan(10.0 * s));
  }

  // Run the car and the VCU together for `ms`, at 1 kHz for the VCU
  void run(Car &car, uint32_t ms) {
    for (uint32_t i = 0; i < ms; i++) {
      car.tick();
      Car::InverterCommand cmd{};
      const double want =
          car.last_inverter_command(&cmd) && cmd.enable ? cmd.torque_nm : 0.0;

      constexpr int substeps = 50; // the tire is stiff
      const double dt = 0.001 / substeps;
      for (int k = 0; k < substeps; k++) {
        torque += (want - torque) * dt / inverter_lag_s;
        const double fz = mass * g * cg_to_front / wheelbase +
                          mass * accel * cg_height / wheelbase;
        const double fx = tire_force(slip(), fz);
        const double wheel_torque = torque * GEAR_RATIO * driveline_eff;
        omega += (wheel_torque - fx * radius()) / rear_inertia * dt;
        if (omega < 0)
          omega = 0;
        accel = (fx - 0.5 * rho * cda * v * v) / mass;
        v += accel * dt;
        if (v < 0)
          v = 0;
      }

      const uint32_t t = sim::now();
      if (t % 10 == 0) {
        car.inverter_motor_rpm(int16_t(std::lround(motor_rpm())));
        car.inverter_torque_feedback(torque);
        if (wheel_speeds_on) {
          const uint16_t front = rpm(v / radius());
          const uint16_t rear = rpm(omega * rear_wheel_sensor_gain);
          car.wheel_rpm(front, front, rear, rear);
        }
      }
      if (t % 20 == 0 && vectornav_on)
        car.vectornav(v, 0, accel);
      sim::advance(1);

      peak_slip = std::max(peak_slip, slip());
    }
  }

  double peak_slip = 0;

private:
  static uint16_t rpm(double rad_s) {
    return uint16_t(std::lround(rad_s * 60 / (2 * M_PI)));
  }
};

} // namespace sim
