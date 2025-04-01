#pragma once

// ksu-fw-common
#include <adc.hpp>
#include <can_tools.hpp>

// CAN lib stuff
#include <car.h>
can_obj_car_h_t kms_can;

// Local
#include "accumulator.hpp"
#include "cm200.hpp"
#include "logger.hpp"
#include "pedal_handeler.hpp"
#include "pin_defs.hpp"
#include "vcu.hpp"

#ifdef ARDUINO
#include <Arduino.h>

// TODO: Make the timer stuff into its own utility, maybe a part of the logger?
#include <Metro.h>

Metro timer_1s = Metro(1000, 1);  // Used for VCU status message
Metro timer_2hz = Metro(500, 1);  // Used for ACU and Precharge messages
Metro timer_10hz = Metro(100, 1); // Used for VCU pedals message
Metro timer_20hz = Metro(50, 1);  // Used for inverter timeout
Metro timer_100hz = Metro(10, 1); // Used for inverter current limit
Metro timer_200hz = Metro(5, 1);  // Used for inverter command message

inline bool wrapped_1s() { return bool(timer_1s.check()); }
inline bool wrapped_2hz() { return bool(timer_2hz.check()); }
inline bool wrapped_10hz() { return bool(timer_10hz.check()); }
inline bool wrapped_20hz() { return bool(timer_20hz.check()); }
inline bool wrapped_200hz() { return bool(timer_200hz.check()); }
inline bool wrapped_100hz() { return bool(timer_100hz.check()); }
#endif

//
//// Comms
// loggers
Logger consol(serial);

// CAN controllers
canMan daq_can(TEENSY_CAN3, DAQ_CAN_BAUD_RATE);
canMan inv_can(TEENSY_CAN2, INVERTER_CAN_BAUD_RATE);
canMan acc_can(TEENSY_CAN1, ACCUMULATOR_CAN_BAUD_RATE);

// Pots
adc apps1(mcp, ADC_CS, ADC_ACCEL_1_CHANNEL);
adc apps2(mcp, ADC_CS, ADC_ACCEL_2_CHANNEL);
adc bse(mcp, ADC_CS, ADC_BSE_CHANNEL);

//
//// Critical components
PEDALS pedals(MIN_BRAKE_PEDAL, START_BRAKE_PEDAL, END_BRAKE_PEDAL,
              MAX_BRAKE_PEDAL, START_ACCELERATOR_PEDAL_1,
              END_ACCELERATOR_PEDAL_1, START_ACCELERATOR_PEDAL_2,
              END_ACCELERATOR_PEDAL_2);
CM200 inverter(&wrapped_20hz, &wrapped_100hz, &wrapped_200hz, true, &inv_can,
               &kms_can);
ACCUMULATOR accumulator(&kms_can, &acc_can, &wrapped_2hz);
VCU vcu(&pedals, &inverter, &accumulator, &kms_can, &acc_can, &inv_can,
        &daq_can, &wrapped_1s, &wrapped_10hz);

//
//// Gizmos
// Aditional ADC chanels
adc steering_angle(mcp, ADC_CS, ADC_STEERING_CHANNEL);

// Voltage / Current sense lines
adc vsense_bspd(avr, BSPD_SENSE);
adc vsense_sdc(avr, VSENSE_SDC);
adc isense_sdc(avr, ISENSE_SDC);
adc vsense_12v(avr, VSENSE_GLV);
adc isense_12v(avr, ISENSE_GLV);
adc vsense_5v(avr, VSENSE_5V);

adc sense_lines[] = {vsense_bspd, vsense_sdc, isense_sdc,
                     vsense_12v,  isense_12v, vsense_5v};
