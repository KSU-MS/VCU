#pragma once
#ifndef main_hpp
#define main_hpp

// ksu-fw-common
#include <adc.hpp>
#include <can_tools.hpp>

// CAN lib stuff
#include <car.h>
can_obj_car_h_t kms_can;

// Local
#include "cm200.hpp"
#include "logger.hpp"
#include "pin_defs.hpp"
#include "vcu.hpp"

#ifdef ARDUINO
#include <Arduino.h>

// TODO: Make the timer stuff into its own utility, maybe a part of the logger?
#include <Metro.h>

Metro timer_20hz = Metro(50, 1);
Metro timer_2s = Metro(2000, 1);
Metro timer_200hz = Metro(5, 1);
Metro timer_100hz = Metro(10, 1);

inline bool wrapped_20hz() { return bool(timer_20hz.check()); }
inline bool wrapped_2s() { return bool(timer_2s.check()); }
inline bool wrapped_200hz() { return bool(timer_200hz.check()); }
inline bool wrapped_100hz() { return bool(timer_100hz.check()); }
#endif

Logger consol(serial);

// canMan daq_can(TEENSY_CAN3, DAQ_CAN_BAUD_RATE);
// canMan inv_can(TEENSY_CAN2, INVERTER_CAN_BAUD_RATE);
// canMan acc_can(TEENSY_CAN1, ACCUMULATOR_CAN_BAUD_RATE);

// Critical components
VCU vcu;
// cm200 fella(&wrapped_20hz, &wrapped_2s, &wrapped_100hz, &wrapped_200hz, true,
//             &inv_can);
// Accumulator accumulator();

// Pots
adc pedal_3v(mcp, ADC_ACCEL_1_CHANNEL);
adc pedal_5v(mcp, ADC_ACCEL_2_CHANNEL);
adc brake_pedal(mcp, ADC_BSE_CHANNEL);
adc steering_angle(mcp, ADC_STEERING_CHANNEL);

adc pots[] = {pedal_3v, pedal_5v, brake_pedal, steering_angle};

// Voltage / Current sense lines
adc vsense_bspd(avr, BSPD_SENSE);
adc vsense_sdc(avr, VSENSE_SDC);
adc isense_sdc(avr, ISENSE_SDC);
adc vsense_12v(avr, VSENSE_GLV);
adc isense_12v(avr, ISENSE_GLV);
adc vsense_5v(avr, VSENSE_5V);

adc sense_lines[] = {vsense_bspd, vsense_sdc, isense_sdc,
                     vsense_12v,  isense_12v, vsense_5v};

#endif // main_hpp
