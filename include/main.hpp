#pragma once

#ifdef ARDUINO
#include <Arduino.h>
#endif

#include "Pin_Defs.hpp"
#include "can_tools.hpp"
#include "logger.hpp"
#include "settings.hpp"
#include "state_machine.hpp"

#include <KSUCAN.hpp>
#include <adc.hpp>

VCU vcu;

Logger consol(serial);

canMan daq_can(TEENSY_CAN3, DAQ_CAN_BAUD_RATE);
canMan inv_can(TEENSY_CAN2, INVERTER_CAN_BAUD_RATE);
canMan acc_can(TEENSY_CAN1, ACCUMULATOR_CAN_BAUD_RATE);

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
