#pragma once

//
//// Teensy pins
#define ADC_CS 10

#define WSFL 28 // Digital input pullup
#define WSFR 29 // Digital input pullup

#define BSPD_SENSE 16 // Analog signal
#define ISENSE_SDC 20 // Analog signal
#define ISENSE_GLV 38 // Analog signal
#define VSENSE_SDC 39 // Analog signal
#define VSENSE_5V 40  // Analog signal
#define VSENSE_GLV 41 // Analog signal
#define A9 27         // Analog signal
#define A10 26        // Analog signal

#define BUZZER 4   // Output
#define LOWSIDE1 5 // Output
#define LOWSIDE2 6 // Output

//
//// MCP pins
#define ADC_ACCEL_1_CHANNEL 0
#define ADC_ACCEL_2_CHANNEL 1
#define ADC_BSE_CHANNEL 2
#define ADC_STEERING_CHANNEL 3

//
//// Default CAN settings
#define ACCUMULATOR_CAN_BAUD_RATE 500000
#define INVERTER_CAN_BAUD_RATE 500000
#define DAQ_CAN_BAUD_RATE 1000000

//
//// Default SPI settings
#define DEFAULT_SPI_SPEED 1000000

//
//// Hardcoded pedal values
#define MIN_BRAKE_PEDAL 2400
#define START_BRAKE_PEDAL 2600
#define END_BRAKE_PEDAL 3200
#define MAX_BRAKE_PEDAL 3400

#define START_ACCELERATOR_PEDAL_1 600
#define END_ACCELERATOR_PEDAL_1 1900

#define START_ACCELERATOR_PEDAL_2 2870
#define END_ACCELERATOR_PEDAL_2 3500
