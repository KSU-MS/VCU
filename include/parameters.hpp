#pragma once

//
//// Some car values
#define MAX_TORQUE_LIMIT_NM 240
#define POWER_LIMIT_KW 80
// regulate_power() aims for this much of it, measured: margin for the
// Energy Meter's own error and the loop's overshoot (EV.3.3.1, EV.3.4.1)
#define POWER_LIMIT_TARGET 0.97
#define SPIN_DIR 0

#define TRACTIVE_SYSTEM_MINIMUM_VOLTAGE 400
#define PRECHARGE_OK_STATE 2
#define MINIMUM_BRAKE_FOR_RTD 0.3
#define RTD_BUZZER_DURATION_MS 2151
// The ACU's shutdown and precharge status and the inverter's bus voltage
// must each have arrived within this long, or the tractive system counts as
// unsafe: a dead ACU or a broken CAN wire mustn't leave the car live on the
// last thing it said. Check it's several of the boards' real send periods.
#define SAFETY_DATA_TIMEOUT_MS 500
// At or above this the tractive system counts as live (T.9.1.1's high
// voltage, where the Ready to Move light flashes, EV.6.2.7)
#define HIGH_VOLTAGE_PRESENT 60

#define INVERTER_CHARGE_LIMIT 60
#define INVERTER_DISCHARGE_LIMIT 600

#define WHEEL_CIRCUMFRANCE_M 1.435608
// Motor revs per wheel rev. Traction control's slip depends on it, and its
// gear ratio check (motor rpm against the rear wheel speed nodes) turns TC
// off if the car doesn't match. kmslap's car model says 3.0: check which
#define GEAR_RATIO 4.18

//
//// Cars with more than one motor (docs/vehicles.md): how the driver's
//// wheel torque is shared out (torque_split.hpp). By axle load, or fixed
#define TORQUE_SPLIT_BY_LOAD 1
#define TORQUE_SPLIT_FRONT_BIAS 0.45 // fixed: the front's share (kmslap's)

//
//// Regen (docs/regen.md). Which cars have it, and how much brake travel
//// is regen only, is vehicle::THIS.regen_brake_travel
// Brake travel before regen starts: the sensor's noise at rest
#define REGEN_BRAKE_DEADBAND 0.03
// EV.3.3.3: no regen at 5 km/h or less; full from 8 (kmslap's
// regen_min_speed / regen_fade_speed), in between in proportion
#define REGEN_MIN_SPEED_MS (5.0 / 3.6)
#define REGEN_FULL_SPEED_MS (8.0 / 3.6)
// EV.3.3.2 caps any voltage at 600 V, and regen raises the pack's: it
// fades out between these, on the highest inverter DC voltage
#define REGEN_BUS_V_FADE 585.0
#define REGEN_BUS_V_MAX 595.0
// Motor and inverter generating, for the power the pack takes (kmslap's
// motor_eff x regen_inverter_eff)
#define REGEN_EFFICIENCY 0.90
// The BMS's charge current limit (Pack_CCL) older than this: no regen
#define BMS_LIMIT_TIMEOUT_MS 1000
// Per-wheel slip control on regen (kmslap RegenPolicy.slip_scale): a
// wheel's regen backs off as its slip goes past the target, to none a band
// beyond it, filtered
#define REGEN_SLIP_TARGET 0.08
#define REGEN_SLIP_BAND 0.06
#define REGEN_SLIP_TAU_S 0.02

//
//// Traction control (docs/traction-control.md)
// What it starts as: off until it's been checked on the car
#define TC_DEFAULT_MODE 0
// VectorNav speed and motor speed older than this: no slip to control, so
// it steps aside. Several of their send periods
#define TC_INPUT_TIMEOUT_MS 200
// Its status frame is vcu_traction_control (0x0CC) on the DAQ bus

//
//// The Pi's torque ceiling (docs/pi-supervisor.md, pi_supervisor.hpp)
// Its command is pi_torque_command (0x010, a low ID so it wins
// arbitration) on the DAQ bus, the VCU's status for it vcu_pi_status (0x0CD)
// The Pi sends at 100 Hz: this is three frames
#define PI_COMMAND_TIMEOUT_MS 30
// Clean frames for this long before the Pi gets control (back)
#define PI_HEALTHY_MS 500
// Leaving the Pi's control mid-throttle, the torque climbs back this fast
// from its last ceiling: 0 to full in about a quarter of a second
#define PI_FALLBACK_RAMP_NM_PER_S 1000

//
//// Hardcoded pedal values
// Raw counts. MIN/MAX are fault thresholds: a reading outside them means the
// sensor is open or shorted (T.4.2.10, T.4.3.4), so they sit between the
// calibrated travel and what a short would read.
#define MIN_BRAKE_PEDAL 25
#define START_BRAKE_PEDAL 130
#define END_BRAKE_PEDAL 922
// A10's divider reads a short to 5 V as ~995 counts and a short to GLV is
// clamped at ~1023 (docs/hardware.md), so this has to sit below 995. It was
// 1025, which a 10-bit ADC can't reach. Check full press on the car stays
// under it.
#define MAX_BRAKE_PEDAL 960

#define MIN_APPS_PEDAL 100

// ADS1018 counts, 1 mV each at the +-2.048 V range the APPS channels use
// (adc_new.hpp): a short to the sensor supply reads full scale, 2047. The
// fault thresholds are ~10% of the travel above full press.
#define START_ACCELERATOR_PEDAL_1 670
#define END_ACCELERATOR_PEDAL_1 1100
#define MAX_ACCELERATOR_PEDAL_1 1150

#define START_ACCELERATOR_PEDAL_2 450
#define END_ACCELERATOR_PEDAL_2 710
#define MAX_ACCELERATOR_PEDAL_2 740

//
//// Default CAN settings
#define ACCUMULATOR_CAN_BAUD_RATE 500000
#define INVERTER_CAN_BAUD_RATE 500000
#define DAQ_CAN_BAUD_RATE 1000000

//
//// Teensy pins
#define ADC_CS 10

#define WSFL 28 // Digital input pullup
#define WSFR 29 // Digital input pullup

#define BSPD_SENSE A2      // Analog signal
#define BSPD_BSE_SENSE A7  // Analog signal
#define BSPD_HALL_SENSE A6 // Analog signal

#ifdef VCU_REV4
#define ISENSE_SDC 20 // Analog signal
#define ISENSE_GLV 38 // Analog signal
#define VSENSE_SDC 39 // Analog signal
#define VSENSE_5V 40  // Analog signal
#define VSENSE_GLV 41 // Analog signal
#endif

#define BUZZER 9   // Output
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
