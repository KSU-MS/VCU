#pragma once

// ksu-fw-common
#include <adc.hpp>
#include <can_tools.hpp>
#include <logger.hpp>

// CAN lib stuff
#include <car.h>
can_obj_car_h_t kms_can;

// Local
#include "accumulator.hpp"
#include "data.hpp"
#include "data_handler.hpp"
#include "inverter.hpp"
#include "parameters.hpp"
#include "pedal_handeler.hpp"
#include "state_machine.hpp"

#ifdef ARDUINO
#include <Arduino.h>

// TODO: Make the timer stuff into its own utility, maybe a part of the logger?
#include <Metro.h>

Metro timer_1s = Metro(1000, true);    // Used for VCU status message
Metro timer_2hz = Metro(500, true);    // Used for ACU and Precharge messages
Metro timer_10hz = Metro(100, true);   // Used for VCU pedals message
Metro timer_10hz_2 = Metro(100, true); // A naming convention so good I made 2
Metro timer_20hz = Metro(50, true);    // Used for inverter timeout
Metro timer_100hz = Metro(10, true);   // Used for inverter current limit
Metro timer_200hz = Metro(5, true);    // Used for inverter command message

Metro buzzer_timer = Metro(2215, false);

bool wrapped_1s() {
  if (timer_1s.check()) {
    return true;
    timer_1s.reset();
  } else {
    return false;
  }
}

bool wrapped_2hz() {
  if (timer_2hz.check()) {
    return true;
    timer_2hz.reset();
  } else {
    return false;
  }
}

bool wrapped_10hz() {
  if (timer_10hz.check()) {
    return true;
    timer_10hz.reset();
  } else {
    return false;
  }
}

bool wrapped_20hz() {
  if (timer_20hz.check()) {
    return true;
    timer_20hz.reset();
  } else {
    return false;
  }
}

bool wrapped_100hz() {
  if (timer_100hz.check()) {
    return true;
    timer_100hz.reset();
  } else {
    return false;
  }
}

bool wrapped_200hz() {
  if (timer_200hz.check()) {
    return true;
    timer_200hz.reset();
  } else {
    return false;
  }
}

bool wrapped_10hz_2() {
  if (timer_10hz_2.check()) {
    return true;
  } else {
    return false;
  }
}

void reset_wrapped_10hz_2() { timer_10hz_2.reset(); }
#endif

std::array<parameter, 25> params = {
    parameter{MAX_TORQUE_LIMIT_NM_x10 / 10.0, 10, "TORQUE_LIMIT_NM_x10"},
    parameter{SOFT_MOTOR_RPM_LIMIT_x10 / 10.0, 10, "SOFT_MOTOR_RPM_LIMIT_x10"},
    parameter{MAX_MOTOR_RPM_LIMIT_x10 / 10.0, 10, "MAX_MOTOR_RPM_LIMIT_x10"},
    parameter{BRAKE_SPEED_RPM_x10 / 10.0, 10, "BRAKE_SPEED_RPM_x10"},
    parameter{POWER_LIMIT_KW_x10 / 10.0, 10, "POWER_LIMIT_KW_x10"},
    parameter{INVERTER_CHARGE_LIMIT, 1, "INVERTER_CHARGE_LIMIT"},
    parameter{INVERTER_DISCHARGE_LIMIT, 1, "INVERTER_DISCHARGE_LIMIT"},
};

VehicleData vehicle_data;

//
//// Comms
// loggers
Logger consol(serial);
// FILE std_out_wrap;

// Critical components
Pedals pedals(MIN_BRAKE_PEDAL, START_BRAKE_PEDAL, END_BRAKE_PEDAL,
              MAX_BRAKE_PEDAL, MIN_APPS_PEDAL, START_ACCELERATOR_PEDAL_1,
              END_ACCELERATOR_PEDAL_1, START_ACCELERATOR_PEDAL_2,
              END_ACCELERATOR_PEDAL_2, &vehicle_data);

Inverter inverter(false, &params, &vehicle_data);

Accumulator accumulator(&params, &vehicle_data);

StateMachine state_machine(&inverter, &params, &vehicle_data);

data_handler data_handler(&params, &vehicle_data);

//
//// Gizmos
// Aditional ADC chanels

// unused
// adc steering_angle(mcp, static_cast<uint8_t>(ADC_CS),
//                    static_cast<uint8_t>(ADC_STEERING_CHANNEL));

// // Voltage / Current sense lines
// adc vsense_sdc(avr, VSENSE_SDC);
// adc isense_sdc(avr, ISENSE_SDC);
// adc vsense_12v(avr, VSENSE_GLV);
// adc isense_12v(avr, ISENSE_GLV);
// adc vsense_5v(avr, VSENSE_5V);
