#include "drivetrain.hpp"
#include "parameters.hpp"

#include <cmath>

using vehicle::InverterKind;
using vehicle::Position;

namespace {
// rad/s per rpm (not PI: Arduino defines that)
constexpr double RAD_S_PER_RPM = 2 * 3.141592653589793 / 60;
// The open-loop cap's guess at motor and inverter efficiency, as Inverter's
constexpr double EFFICIENCY = 0.90;
} // namespace

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
    if (got & Inverter::DC_CURRENT) {
      dc_heard[i].heard(now_ms);
      dc_new = true;
    }
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
  for (size_t i = 0; i < n; i++) {
    inverters[i]->set_torque_limit(on ? torque_limit_for(i) : 0);
    // Regen on the hub motors only: the Cascadia's never been set up for
    // it, so rwd_emrax (and the EMRAX on the awd cars) never regenerates
    const bool can_regen = drives[i].inverter != InverterKind::CASCADIA_PM100;
    inverters[i]->set_regen_limit(on && can_regen ? torque_limit_for(i) : 0);
  }
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

double Drivetrain::regen_wheel_cap(size_t i) {
  // The same curve, generating; through the gears the other way
  const double motor_nm =
      std::fmin(inverters[i]->get_regen_limit_nm(),
                motors::max_torque_nm(*drives[i].motor,
                                      inverters[i]->get_motor_rpm()));
  return motor_nm * drives[i].gear_ratio / drives[i].gear_eff;
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
  limit_kw = total_kw;
  // On the hub cars each inverter's own open-loop cap is set to the total,
  // so it never binds before the total's loop does
  for (size_t i = 0; i < n; i++)
    inverters[i]->set_power_limit_kw(total_kw);
}

double Drivetrain::peak_power_kw() const {
  double total = 0;
  for (size_t i = 0; i < n; i++) {
    if (!(drives[i].motor->peak_power_kw > 0))
      return POWER_LIMIT_KW;
    total += drives[i].motor->peak_power_kw;
  }
  return total > POWER_LIMIT_KW ? total : POWER_LIMIT_KW;
}

double Drivetrain::power_share(size_t i) const {
  // A quarter by capability, so a motor asked for nothing now still has
  // some current when it's asked for more, before the next update
  constexpr double by_capability = 0.25;
  double asked[MAX_DRIVES], total = 0;
  for (size_t j = 0; j < n; j++) {
    const double omega =
        std::fabs(double(inverters[j]->get_motor_rpm())) * RAD_S_PER_RPM;
    asked[j] = std::fabs(motor_nm[j]) * omega; // driving or regenerating
    total += asked[j];
  }
  if (!(total > 0))
    return share(i);
  return by_capability * share(i) + (1 - by_capability) * asked[i] / total;
}

void Drivetrain::set_current_limits(uint16_t charge_limit,
                                    float pack_voltage) {
  // The power limit over the pack voltage (get_instant_current_limit), each
  // inverter its share: rwd_emrax's one gets all of it. And the hub cars'
  // charge limit, the BMS's, shared the same way
  const double total =
      n ? inverters[0]->get_instant_current_limit(pack_voltage) : 0;
  for (size_t i = 0; i < n; i++) {
    if (n == 1) {
      inverters[i]->set_current_limits(charge_limit, uint16_t(total));
      continue;
    }
    const double s = power_share(i);
    inverters[i]->set_current_limits(uint16_t(std::floor(charge_limit_a * s)),
                                     uint16_t(std::floor(total * s)));
  }
}

void Drivetrain::set_regen_limits(double charge_a, double pack_voltage,
                                  double bus_v_max) {
  charge_limit_a = charge_a > 0 && charge_a < UINT16_MAX ? charge_a : 0;
  regen_pack_v = pack_voltage > 0 ? pack_voltage : 0;
  regen_v_max = bus_v_max > 0 && bus_v_max < REGEN_BUS_V_MAX
                    ? bus_v_max
                    : REGEN_BUS_V_MAX;
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
  if (n == 1) {
    inverters[0]->regulate_power(now_ms);
    return;
  }
  // The total, from each inverter's latest V x I: a sample only when they
  // all have a fresh one, or the loop flies blind (the open-loop cap). Half
  // the loop's own staleness each, so it's blind within 1.5 x STALE_MS of
  // one going quiet
  bool all_fresh = true;
  double measured_w = 0;
  for (size_t i = 0; i < n; i++) {
    all_fresh &= dc_heard[i].fresh(now_ms, PowerLoop::STALE_MS / 2);
    measured_w += inverters[i]->get_measured_power_w();
  }
  total_power.update(now_ms, limit_kw * 1000.0, dc_new && all_fresh,
                     measured_w);
  dc_new = false;
}

double Drivetrain::max_request() {
  if (n == 1)
    return inverters[0]->get_torque_limit();
  double total = 0;
  for (size_t i = 0; i < n; i++)
    total += wheel_cap(i);
  return total;
}

double Drivetrain::max_regen_request() {
  double total = 0;
  for (size_t i = 0; i < n; i++)
    total += regen_wheel_cap(i);
  return total;
}

void Drivetrain::command(double request, uint32_t now_ms) {
  if (n == 1) {
    inverters[0]->command_torque(request);
    return;
  }
  const bool regen = request < 0;
  double cap[MAX_DRIVES];
  for (size_t i = 0; i < n; i++)
    cap[i] = regen ? regen_wheel_cap(i) : wheel_cap(i);
  // By axle load either way: braking moves it to the front
  const double front =
      torque_split::front_share(split_mode, front_bias, vx, ax);
  torque_split::split(regen ? -request : request, front, drives, cap, n,
                      wheel);

  if (regen) {
    for (size_t i = 0; i < n; i++) {
      wheel[i] = -wheel[i];
      motor_nm[i] = wheel[i] * drives[i].gear_eff / drives[i].gear_ratio;
    }
    limit_regen(now_ms);
    for (size_t i = 0; i < n; i++)
      inverters[i]->command_torque(motor_nm[i]);
    last_command_ms = now_ms;
    return;
  }
  // Not regenerating: the slip control starts afresh next time
  for (size_t i = 0; i < n; i++)
    slip_keep[i] = 1;
  last_command_ms = now_ms;

  // Wheel torque back to the motor's, and the power it all takes at the
  // motors' speeds
  double mech_w = 0;
  for (size_t i = 0; i < n; i++) {
    motor_nm[i] = wheel[i] / (drives[i].gear_ratio * drives[i].gear_eff);
    const double omega =
        std::fabs(double(inverters[i]->get_motor_rpm())) * RAD_S_PER_RPM;
    mech_w += motor_nm[i] > 0 ? motor_nm[i] * omega : 0;
  }
  // Over what the power limit allows (the open-loop cap, trimmed on the
  // measured total): every motor scaled back alike, the split kept
  const double allowed_w =
      limit_kw * 1000.0 * EFFICIENCY * total_power.trim();
  const double scale =
      !(limit_kw > 0) ? 0 : (mech_w > allowed_w ? allowed_w / mech_w : 1);
  // The inverter still applies its own torque limit on top
  for (size_t i = 0; i < n; i++) {
    motor_nm[i] *= scale;
    wheel[i] *= scale;
    inverters[i]->command_torque(motor_nm[i]);
  }
}

void Drivetrain::limit_regen(uint32_t now_ms) {
  auto clamp01 = [](double x) { return x < 0 ? 0 : (x > 1 ? 1 : x); };

  // The power it all puts back into the pack, against what the BMS lets it
  // take: every motor alike
  double mech_w = 0;
  for (size_t i = 0; i < n; i++)
    mech_w += -motor_nm[i] *
              std::fabs(double(inverters[i]->get_motor_rpm())) * RAD_S_PER_RPM;
  const double elec_w = mech_w * REGEN_EFFICIENCY;
  const double charge_w = charge_limit_a * regen_pack_v;
  double scale = elec_w > charge_w ? charge_w / elec_w : 1;
  // EV.3.3.2: fading out before the DC bus gets near 600 V
  scale *= clamp01((regen_v_max - max_bus_voltage()) /
                   (REGEN_BUS_V_MAX - REGEN_BUS_V_FADE));

  // Each wheel: its regen backs off as it turns slower than the car
  // (kmslap RegenPolicy.slip_scale), filtered
  const double dt_s = (now_ms - last_command_ms) / 1000.0;
  const double follow =
      dt_s / REGEN_SLIP_TAU_S < 1 ? dt_s / REGEN_SLIP_TAU_S : 1;
  for (size_t i = 0; i < n; i++) {
    double target = 1;
    if (vx > REGEN_MIN_SPEED_MS) {
      const double wheel_ms =
          std::fabs(double(inverters[i]->get_motor_rpm())) /
          drives[i].gear_ratio * WHEEL_CIRCUMFRANCE_M / 60;
      const double slip = (wheel_ms - vx) / vx;
      target = clamp01(1 - (-slip - REGEN_SLIP_TARGET) / REGEN_SLIP_BAND);
    }
    slip_keep[i] += follow * (target - slip_keep[i]);
    motor_nm[i] *= scale * slip_keep[i];
    wheel[i] *= scale * slip_keep[i];
  }
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
