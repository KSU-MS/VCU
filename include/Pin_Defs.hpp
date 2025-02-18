#pragma once

//
//// Teensy pins
#define CS_ADC 10

#define WSFL 28 // Digital input pullup
#define WSFR 29 // Digital input pullup

#define BSPDSENSE 16  // Analog signal
#define ISENSE_SDC 20 // Analog signal
#define ISENSE_GLV 38 // Analog signal
#define VSENSE_SDC 39 // Analog signal
#define VSENSE_5V 40  // Analog signal
#define VSENSE_GLV 41 // Analog signal
#define A9 27         // Analog signal
#define A10 26        // Analog signal

const int analog_pin_list[] = {VSENSE_SDC, ISENSE_SDC, BSPDSENSE, VSENSE_GLV,
                               ISENSE_GLV, VSENSE_5V,  A9,        A10};
#define BUZZER 4   // Output
#define LOWSIDE1 5 // Output
#define LOWSIDE2 6 // Output

//
//// MCP pins
#define ADC_STEERING_CHANNEL 3
#define ADC_BRAKE_1_CHANNEL 2
#define ADC_ACCEL_1_CHANNEL 1
#define ADC_ACCEL_2_CHANNEL 0

//
//// Other config things
#define DEFAULT_SPI_SPEED 1000000
