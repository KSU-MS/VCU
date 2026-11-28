#pragma once
// DTI HV500 (F-SiC) driving a hub motor, in current mode, over DTI's CAN
// map v2.5: extended IDs, packet << 8 | node, big endian.
//
// Packed here rather than with the DBC's generated code: dbcc names its
// functions after each node's ID (encode_can_0x101_..., 0x102_, ...), so a
// class that works for any node can't use them. test/4wd_dti checks these
// frames against the generated decoders for every node.
//
// Torque is sent as AC current, in A peak ("peak, not RMS": DTI's CAN
// manual v2.5, Set AC current 0x01), so I_pk = sqrt(2) x torque / kt, with
// kt in N m per A rms.
#include "inverter.hpp"
#include "motors.hpp"

class DtiHv500 : public Inverter {
public:
  DtiHv500(uint8_t node, const MotorSpec &motor, double gear_ratio,
           canMan *can, canMan *daq);

  uint8_t on_frame(const can_message &msg) override;
  // Drive disabled, no current: also what keeps it from timing out
  void ping() override;
  // DTI's map has no clear-faults command: a fault clears when its cause has
  // gone and drive enable is cycled, which ping() does
  void send_clear_faults() override {}
  // SetMaxDcCurrent: the share of the pack's discharge limit it gets, and
  // SetMaxDcBrakeCurrent its share of the charge limit (regen), sent
  // negative. (DTI's manual gives 0x0B's unit as % in its table but A in
  // its example: A, decided)
  void set_current_limits(uint16_t charge_limit,
                          uint16_t discharge_limit) override;

  // The AC current a torque takes, A peak (above)
  double current_for_torque(double nm) const;

  double controller_temp_c() const { return temp_controller; }
  double motor_temp_c() const { return temp_motor; }
  uint8_t fault_code() const { return fault; }
  uint8_t node() const { return node_id; }

  // DTI CAN map v2.5 packet numbers
  enum packet : uint8_t {
    SET_AC_CURRENT = 0x01,
    SET_MAX_AC_CURRENT = 0x08,
    SET_MAX_AC_BRAKE_CURRENT = 0x09,
    SET_MAX_DC_CURRENT = 0x0A,
    SET_MAX_DC_BRAKE_CURRENT = 0x0B,
    SET_DRIVE_ENABLE = 0x0C,
    ERPM_DUTY_VOLTAGE = 0x20,
    AC_DC_CURRENT = 0x21,
    TEMPERATURES = 0x22,
  };
  uint32_t id(packet p) const { return uint32_t(p) << 8 | node_id; }

protected:
  void send_torque(double torque_nm) override;

private:
  void send(packet p, const uint8_t *data, uint8_t len);
  void send_i16(packet p, double value, double scale, uint8_t len);

  uint8_t node_id;
  const MotorSpec &motor;
  canMan *can;
  canMan *daq;

  double temp_controller = 0;
  double temp_motor = 0;
  uint8_t fault = 0;
};
