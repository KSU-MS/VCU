#pragma once
// Cascadia Motion PM100DX, driving the EMRAX 228 on the car today: M192
// torque commands, M165/M166/M167/M172 feedback, the BMS current limit
// frame. On the inverter bus, mirrored to the DAQ bus.
#include <car.h>

#include "inverter.hpp"

class CascadiaPM100 : public Inverter {
public:
  CascadiaPM100(bool spin_direction, canMan *can, canMan *daq,
                can_obj_car_h_t *dbc, float over_power_decay_factor);

  uint8_t on_frame(const can_message &msg) override;
  void ping() override;
  void send_clear_faults() override;
  void set_current_limits(uint16_t charge_limit,
                          uint16_t discharge_limit) override;

  void command_speed(int16_t speed_request);

protected:
  void send_torque(double torque_nm) override;

private:
  // Unused since the exponential decay was replaced by regulate_power;
  // kept so the constructor (and main.hpp) stay as they were
  float over_power_decay_factor;

  canMan *can;
  canMan *daq;
  can_obj_car_h_t *dbc;
};
