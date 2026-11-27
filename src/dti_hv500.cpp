#include "dti_hv500.hpp"
#include <cmath>

namespace {

int16_t be16(const uint8_t *d) { return int16_t(d[0] << 8 | d[1]); }
int32_t be32(const uint8_t *d) {
  return int32_t(uint32_t(d[0]) << 24 | uint32_t(d[1]) << 16 |
                 uint32_t(d[2]) << 8 | d[3]);
}

} // namespace

DtiHv500::DtiHv500(uint8_t node, const MotorSpec &motor, double gear_ratio,
                   canMan *can, canMan *daq)
    : Inverter(true, gear_ratio), node_id(node), motor(motor), can(can),
      daq(daq) {
  // The most AC current it may ever use: the motor's peak, in A peak
  send_i16(SET_MAX_AC_CURRENT, std::sqrt(2.0) * motor.peak_torque_nm /
                                   motor.kt_nm_per_arms,
           10, 2);
  ping();
}

double DtiHv500::current_for_torque(double nm) const {
  return std::sqrt(2.0) * nm / motor.kt_nm_per_arms;
}

void DtiHv500::send(packet p, const uint8_t *data, uint8_t len) {
  can_message out;
  out.id = id(p);
  out.is_extended = true;
  out.length = len;
  for (uint8_t i = 0; i < 8; i++)
    out.buf.byte[i] = i < len ? data[i] : 0;
  can->send_controller_message(out);
  daq->send_controller_message(out);
}

void DtiHv500::send_i16(packet p, double value, double scale, uint8_t len) {
  double raw = std::round(value * scale);
  if (!(raw > INT16_MIN)) // and NaN
    raw = raw < 0 ? INT16_MIN : 0;
  if (raw > INT16_MAX)
    raw = INT16_MAX;
  const int16_t v = int16_t(raw);
  const uint8_t d[8] = {uint8_t(uint16_t(v) >> 8), uint8_t(v)};
  send(p, d, len);
}

uint8_t DtiHv500::on_frame(const can_message &msg) {
  // Its feedback is extended: a standard frame with the same number is
  // someone else's
  if (!msg.is_extended || (msg.id & 0xFF) != node_id || msg.length < 8)
    return NOT_MINE;
  const uint8_t *d = msg.buf.byte;
  switch (msg.id >> 8) {
  case ERPM_DUTY_VOLTAGE: {
    const double rpm = double(be32(d)) / motor.pole_pairs;
    motor_rpm = int16_t(rpm > INT16_MAX ? INT16_MAX
                        : rpm < INT16_MIN ? INT16_MIN
                                          : rpm);
    bus_voltage = be16(d + 6);
    return MOTOR_SPEED | DC_VOLTAGE;
  }
  case AC_DC_CURRENT:
    // Torque from the AC current, the inverse of current_for_torque
    motor_torque = be16(d) * 0.1 * motor.kt_nm_per_arms / std::sqrt(2.0);
    new_dc_current(be16(d + 2) * 0.1);
    return DC_CURRENT | MOTOR_TORQUE;
  case TEMPERATURES:
    temp_controller = be16(d) * 0.1;
    temp_motor = be16(d + 2) * 0.1;
    fault = d[4];
    return STATUS;
  default:
    return NOT_MINE;
  }
}

void DtiHv500::ping() {
  const uint8_t off[2] = {0, 0};
  send(SET_DRIVE_ENABLE, off, 2);
  send_i16(SET_AC_CURRENT, 0, 10, 8);
}

void DtiHv500::set_current_limits(uint16_t, uint16_t discharge_limit) {
  send_i16(SET_MAX_DC_CURRENT, discharge_limit, 10, 2);
}

void DtiHv500::send_torque(double torque_nm) {
  const uint8_t enable[2] = {uint8_t(inverter_enable ? 1 : 0), 0};
  send(SET_DRIVE_ENABLE, enable, 2);
  // Sending SetAcCurrent is also what puts it in current control
  send_i16(SET_AC_CURRENT, inverter_enable ? current_for_torque(torque_nm) : 0,
           10, 8);
}
