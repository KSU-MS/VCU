#pragma once
#ifndef cm200_hpp
#define cm200_hpp

class cm200 {
private:
  bool spin_forward = true;
  bool inverter_enable = false;
  uint16_t motor_rpm;
  uint16_t motor_temp;
  uint16_t bus_voltage;

  bool (*timer_mc_kick)();
  bool (*timer_current_limit)();
  bool (*timer_inverter_enable)();
  bool (*timer_motor_controller_send)();

  void disable();

  canMan *can;

public:
  cm200(bool (*timer_mc_kick)(), bool (*timer_current_limit)(),
        bool (*timer_inverter_enable)(), bool (*timer_motor_controller_send)(),
        bool spin_direction, canMan *can);

  void ping();
};

#endif // cm200_hpp
