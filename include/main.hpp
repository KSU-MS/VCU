#pragma once

// ksu-fw-common
#include <adc_new.hpp>
#include <can_tools.hpp>
#include <logger.hpp>

// CAN lib stuff
#include <car.h>
can_obj_car_h_t kms_can;

// Local
#include "accumulator.hpp"
#include "inverter.hpp"
#include "parameters.hpp"
#include "pedal_handeler.hpp"
#include "vcu.hpp"

#include <Arduino.h>

// The loop timers now live in VCU (see Periodic in periodic.hpp)

//
//// Comms
// loggers
Logger consol(serial);
// FILE std_out_wrap;

// CAN controllers
canMan acc_can(TEENSY_CAN1, ACCUMULATOR_CAN_BAUD_RATE);
canMan inv_can(TEENSY_CAN2, INVERTER_CAN_BAUD_RATE);
canMan daq_can(TEENSY_CAN3, DAQ_CAN_BAUD_RATE);

// Pots
adc apps1(ads, ADC_CS, (uint8_t)ADC_ACCEL_1_CHANNEL);
adc apps2(ads_tech, ADC_CS, (uint8_t)ADC_ACCEL_2_CHANNEL);

// Front brake pressure
adc bse(avr, A10);

//
//// Critical components
Pedals pedals(MIN_BRAKE_PEDAL, START_BRAKE_PEDAL, END_BRAKE_PEDAL,
              MAX_BRAKE_PEDAL, MIN_APPS_PEDAL, START_ACCELERATOR_PEDAL_1,
              END_ACCELERATOR_PEDAL_1, START_ACCELERATOR_PEDAL_2,
              END_ACCELERATOR_PEDAL_2);

Inverter inverter(SPIN_DIR, &inv_can, &daq_can, &kms_can, -0.69314718056);

Accumulator accumulator(&kms_can, &acc_can);

VCU vcu(&pedals, &inverter, &accumulator, &kms_can, &acc_can, &inv_can,
        &daq_can);

// Firmware update over the DAQ bus, see docs/ota.md
#include "ota_teensy.hpp"
#include <ota.hpp>
TeensyFlash ota_flash;
TeensyBoard ota_board(ota_flash, vcu);
ota::Service updater(ota_board, ota_board.staging(), &daq_can);

//
//// Gizmos
// Aditional ADC chanels
adc steering_angle(ads, static_cast<uint8_t>(ADC_CS),
                   static_cast<uint8_t>(ADC_STEERING_CHANNEL));

// Voltage / Current sense lines
adc vsense_bspd(avr, BSPD_SENSE);
adc vsense_bspd_bse(avr, BSPD_BSE_SENSE);
adc vsense_bspd_hall(avr, BSPD_HALL_SENSE);

#ifdef VCU_REV4
adc vsense_sdc(avr, VSENSE_SDC);
adc isense_sdc(avr, ISENSE_SDC);
adc vsense_12v(avr, VSENSE_GLV);
adc isense_12v(avr, ISENSE_GLV);
adc vsense_5v(avr, VSENSE_5V);
#endif
