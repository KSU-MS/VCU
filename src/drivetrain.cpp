#include "drivetrain.hpp"
#include "parameters.hpp"

#include <cmath>

using vehicle::InverterKind;
using vehicle::Position;

Drivetrain::Drivetrain(Inverter *const *inv, const vehicle::Drive *drives,
                       size_t count)
    : drives(drives), n(count < MAX_DRIVES ? count : MAX_DRIVES) {
  for (size_t i = 0; i < MAX_DRIVES; i++)
    inverters[i] = i < n ? inv[i] : nullptr;
}

Drivetrain::Heard Drivetrain::on_frame(const can_message &msg,
                                       uint32_t now_ms) {
  for (size_t i = 0; i < n; i++) {
    const uint8_t got = inverters[i]->on_frame(msg);
    if (got == Inverter::NOT_MINE)
      continue;
    if (got & Inverter::DC_VOLTAGE)
      voltage_heard[i].heard(now_ms);
    return {got, i};
  }
  return {Inverter::NOT_MINE, 0};
}

bool Drivetrain::all_heard(uint32_t now_ms, uint32_t timeout_ms) const {
  for (size_t i = 0; i < n; i++)
    if (!voltage_heard[i].fresh(now_ms, timeout_ms))
      return false;
  return n > 0;
}

double Drivetrain::min_bus_voltage() {
  double v = n ? inverters[0]->get_bus_voltage() : 0;
  for (size_t i = 1; i < n; i++)
    if (inverters[i]->get_bus_voltage() < v)
      v = inverters[i]->get_bus_voltage();
  return v;
}

double Drivetrain::max_bus_voltage() {
  double v = n ? inverters[0]->get_bus_voltage() : 0;
  for (size_t i = 1; i < n; i++)
    if (inverters[i]->get_bus_voltage() > v)
      v = inverters[i]->get_bus_voltage();
  return v;
}

void Drivetrain::set_enable(bool enable) {
  for (size_t i = 0; i < n; i++)
    inverters[i]->set_inverter_enable(enable);
}

bool Drivetrain::any_enabled() {
  for (size_t i = 0; i < n; i++)
    if (inverters[i]->get_inverter_enable())
      return true;
  return false;
}

double Drivetrain::torque_limit_for(size_t i) const {
  // The Cascadia's has always been MAX_TORQUE_LIMIT_NM (the EMRAX's
  // datasheet says 230); the hub motors' is their peak
  return drives[i].inverter == InverterKind::CASCADIA_PM100
             ? MAX_TORQUE_LIMIT_NM
             : drives[i].motor->peak_torque_nm;
}

void Drivetrain::set_torque_limits(bool on) {
  for (size_t i = 0; i < n; i++)
    inverters[i]->set_torque_limit(on ? torque_limit_for(i) : 0);
}

uint8_t Drivetrain::torque_limit_nm() {
  return n ? inverters[0]->get_torque_limit() : 0;
}

double Drivetrain::share(size_t i) const {
  auto capability = [this](size_t j) {
    return torque_limit_for(j) * drives[j].gear_ratio * drives[j].gear_eff;
  };
  double total = 0;
  for (size_t j = 0; j < n; j++)
    total += capability(j);
  return total > 0 ? capability(i) / total : 0;
}

double Drivetrain::wheel_cap(size_t i) {
  // The torque limit, or less at speed: the motor's torque curve, as kmslap
  // has it, so full pedal asks for what the motors can actually give
  const double motor_nm =
      std::fmin(inverters[i]->get_torque_limit_nm(),
                motors::max_torque_nm(*drives[i].motor,
                                      inverters[i]->get_motor_rpm()));
  return motor_nm * drives[i].gear_ratio * drives[i].gear_eff;
}

void Drivetrain::set_motion(double vx_ms, double ax_ms2) {
  vx = vx_ms;
  ax = ax_ms2;
}

void Drivetrain::set_split(torque_split::Mode mode, double bias) {
  split_mode = mode;
  front_bias = bias;
}

void Drivetrain::set_power_limit_kw(double total_kw) {
  for (size_t i = 0; i < n; i++)
    inverters[i]->set_power_limit_kw(total_kw * share(i));
}

void Drivetrain::set_current_limits(uint16_t charge_limit,
                                    float pack_voltage) {
  // Each inverter's discharge limit is its power share over the pack voltage
  for (size_t i = 0; i < n; i++)
    inverters[i]->set_current_limits(
        charge_limit, inverters[i]->get_instant_current_limit(pack_voltage));
}

void Drivetrain::set_current_limit_amps(uint16_t charge_limit,
                                        uint16_t discharge_limit) {
  for (size_t i = 0; i < n; i++)
    inverters[i]->set_current_limits(
        charge_limit, uint16_t(discharge_limit * share(i)));
}

void Drivetrain::ping() {
  for (size_t i = 0; i < n; i++)
    inverters[i]->ping();
}

void Drivetrain::send_clear_faults() {
  for (size_t i = 0; i < n; i++)
    inverters[i]->send_clear_faults();
}

void Drivetrain::regulate_power(uint32_t now_ms) {
  for (size_t i = 0; i < n; i++)
    inverters[i]->regulate_power(now_ms);
}

double Drivetrain::max_request() {
  if (n == 1)
    return inverters[0]->get_torque_limit();
  double total = 0;
  for (size_t i = 0; i < n; i++)
    total += wheel_cap(i);
  return total;
}

void Drivetrain::command(double request) {
  if (n == 1) {
    inverters[0]->command_torque(request);
    return;
  }
  double cap[MAX_DRIVES];
  for (size_t i = 0; i < n; i++)
    cap[i] = wheel_cap(i);
  const double front =
      torque_split::front_share(split_mode, front_bias, vx, ax);
  torque_split::split(request, front, drives, cap, n, wheel);
  // Wheel torque back to the motor's; the inverter still applies its own
  // torque and power limits on top
  for (size_t i = 0; i < n; i++)
    inverters[i]->command_torque(
        wheel[i] / (drives[i].gear_ratio * drives[i].gear_eff));
}

Inverter *Drivetrain::rear_axle() {
  for (size_t i = 0; i < n; i++)
    if (drives[i].position == Position::REAR_AXLE)
      return inverters[i];
  return nullptr;
}

void Drivetrain::calculate_distance(uint32_t now_ms) {
  if (n)
    inverters[0]->calculate_motor_distance_M(now_ms);
}

uint32_t Drivetrain::distance_m() {
  return n ? inverters[0]->get_motor_distance_M() : 0;
}
