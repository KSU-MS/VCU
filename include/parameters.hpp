#pragma once
#include <cstdint>

//
// Some car values
#define MAX_TORQUE_LIMIT_NM 190
#define POWER_LIMIT_W 80000
#define SOFT_MOTOR_RPM_LIMIT 6000
#define MAX_MOTOR_RPM_LIMIT 7000
#define BRAKE_SPEED_RPM 5000
#define SPEED_RATE_LIMIT_RPM_PER_S 1000

#define INVERTER_CONTROL_MODE_TORQUE 1

#define INVERTER_TORQUE_KP_X100 300
#define INVERTER_TORQUE_KI_X100 1
#define INVERTER_TORQUE_KD_X100 0

#define TRACTIVE_SYSTEM_MINIMUM_VOLTAGE 400
#define PRECHARGE_OK_STATE 2
#define MINIMUM_BRAKE_FOR_RTD 0.3

#define INVERTER_CHARGE_LIMIT_A 60
#define INVERTER_DISCHARGE_LIMIT_A 600

#define WHEEL_CIRCUMFRANCE_M 1.435608
#define GEAR_RATIO 4.18

//
//// Hardcoded pedal values
#define MIN_BRAKE_PEDAL 2000
#define START_BRAKE_PEDAL 2840
#define END_BRAKE_PEDAL 3341
#define MAX_BRAKE_PEDAL 4000

#define MIN_APPS_PEDAL 100

#define START_ACCELERATOR_PEDAL_1 2862
#define END_ACCELERATOR_PEDAL_1 3484

#define START_ACCELERATOR_PEDAL_2 585
#define END_ACCELERATOR_PEDAL_2 1816

//
//// Default CAN settings
#define ACCUMULATOR_CAN_BAUD_RATE 500000
#define INVERTER_CAN_BAUD_RATE 500000
#define DAQ_CAN_BAUD_RATE 1000000

//
//// Teensy pins
// #define ADC_CS 10

#define WSFL 28 // Digital input pullup
#define WSFR 29 // Digital input pullup

#define BSPD_SENSE 16 // Analog signal
#define ISENSE_SDC 20 // Analog signal
#define ISENSE_GLV 38 // Analog signal
#define VSENSE_SDC 39 // Analog signal
#define VSENSE_5V 40  // Analog signal
#define VSENSE_GLV 41 // Analog signal
// #define A9 27         // Analog signal
// #define A10 26        // Analog signal

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
//// Default SPI settings
#define DEFAULT_SPI_SPEED 1000000

//
//// GIT status message defs
// These values are provided by the python script ran by the lib_dep
// https://github.com/KSU-MS/pio-git-hash-gen
#ifndef AUTO_VERSION
#warning "AUTO_VERSION was not defined by the generator!"
#define AUTO_VERSION 0xdeadbeef
#endif

#ifndef FW_PROJECT_IS_DIRTY
#warning "FW_PROJECT_IS_DIRTY was not defined by the generator!"
#define FW_PROJECT_IS_DIRTY 1
#endif

#ifndef FW_PROJECT_IS_MAIN_OR_MASTER
#warning "FW_PROJECT_IS_MAIN_OR_MASTER was not defined by the generator!"
#define FW_PROJECT_IS_MAIN_OR_MASTER 0
#endif

enum parameterId : uint8_t {
  MAX_TORQUE_ID = 0,
  SOFT_RPM_LIMIT_ID = 1,
  MAX_RPM_LIMIT_ID = 2,
  BRAKE_SPEED_LIMIT_ID = 3,
  POWER_LIMIT_ID = 4,
  CURRENT_CHARGE_LIMIT_ID = 5,
  CURRENT_DISCHARGE_LIMIT_ID = 6,
  INSTANT_CURRENT_LIMIT_ID = 7,
  INVERTER_TORQUE_KP_X100_ID = 8,
  INVERTER_TORQUE_KI_X100_ID = 9,
  INVERTER_TORQUE_KD_X100_ID = 10,
  INVERTER_CONTROL_MODE_TORQUE_ID = 11,
};

struct Parameter {
  uint32_t value;
  const char *name;
};
