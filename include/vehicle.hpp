#pragma once
#include <SD.h>
#include <array>
#include <cstdint>

// ── Car constants
// ─────────────────────────────────────────────────────────────

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

// Hardcoded pedal values
#define MIN_BRAKE_PEDAL 2000
#define START_BRAKE_PEDAL 2840
#define END_BRAKE_PEDAL 3341
#define MAX_BRAKE_PEDAL 4000

#define MIN_APPS_PEDAL 100

#define START_ACCELERATOR_PEDAL_1 2862
#define END_ACCELERATOR_PEDAL_1 3484

#define START_ACCELERATOR_PEDAL_2 585
#define END_ACCELERATOR_PEDAL_2 1816

// Default CAN settings
#define ACCUMULATOR_CAN_BAUD_RATE 500000
#define INVERTER_CAN_BAUD_RATE 500000
#define DAQ_CAN_BAUD_RATE 1000000

// Teensy pins
#define WSFL 28 // Digital input pullup
#define WSFR 29 // Digital input pullup

#define BSPD_SENSE 16 // Analog signal
#define ISENSE_SDC 20 // Analog signal
#define ISENSE_GLV 38 // Analog signal
#define VSENSE_SDC 39 // Analog signal
#define VSENSE_5V 40  // Analog signal
#define VSENSE_GLV 41 // Analog signal

#define BUZZER 4   // Output
#define LOWSIDE1 5 // Output
#define LOWSIDE2 6 // Output

// MCP pins
#define ADC_ACCEL_1_CHANNEL 0
#define ADC_ACCEL_2_CHANNEL 1
#define ADC_BSE_CHANNEL 2
#define ADC_STEERING_CHANNEL 3

// Default SPI settings
#define DEFAULT_SPI_SPEED 1000000

// GIT status message defs
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

// ── Parameter table
// ───────────────────────────────────────────────────────────

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

// ── Vehicle data structs
// ──────────────────────────────────────────────────────

struct AccumulatorData {
  uint8_t precharge_state = 0;
  bool imd_ok_hs = false;
  bool bms_ok_hs = false;
  double pack_voltage = 0.0;
  double pack_current = 0.0;
  double consumed_power_wh = 0.0;
  uint32_t last_energy_calc_timestamp_ms = 0;
};

struct InverterData {
  double torque_target_nm = 0.0;
  double bus_voltage = 0.0;
  double bus_current = 0.0;
  int16_t motor_rpm = 0;
  double motor_distance_m = 0.0;
  double power_output_w = 0.0;
  uint32_t last_distance_calc_timestamp_ms = 0;
  bool spin_forward = true;
  bool inverter_enable = false;
  bool inverter_discharge = false;
};

struct DriverInterfaceData {
  bool rtd_button_pressed = false;
};

struct PedalData {
  uint16_t raw_apps1 = 0;
  uint16_t raw_apps2 = 0;
  uint16_t raw_brake = 0;
  uint16_t raw_bspd = 0;

  double brake_travel = 0.0;
  double apps1_travel = 0.0;
  double apps2_travel = 0.0;
  double throttle_travel = 0.0;

  bool bspd_ok_hs =
      false; // BSPD (Brake system pressure detector) high side fault
  bool bse_fault = false;      // BSE (Brake sensor encoder) fault
  bool apps_fault = false;     // APPS (Accelerator pedal position sensor) fault
  bool apps_bse_fault = false; // APPS and BSE simultaneous input fault

  // TODO: implement these faults
  bool bspd_brake_high = false;   // BSPD brake high side fault (not real)
  bool bspd_current_high = false; // BSPD current high side fault (not real)
};

struct StateMachineData {
  enum state {
    STARTUP = 0,
    TRACTIVE_SYSTEM_DISABLED = 1,
    TRACTIVE_SYSTEM_ENERGIZED = 2,
    TRACTIVE_SYSTEM_ENABLED = 3,
    READY_TO_DRIVE = 4,
  } current_state;
  uint16_t bool_code = 0;
  uint16_t error_code = 0;
  bool buzzer_active = false;
};

struct VehicleData {
  AccumulatorData accumulator;
  InverterData inverter;
  DriverInterfaceData driver;
  PedalData pedals;
  StateMachineData state_machine;
};

// ── ParameterStore
// ────────────────────────────────────────────────────────────
//
// Loads and saves the params array to/from a plain-text INI-style file on the
// Teensy 4.1 built-in SD card slot.
//
// File format (params.cfg):
//   # comment lines are ignored
//   MAX_TORQUE_LIMIT_NM=190
//   POWER_LIMIT_W=80000
//   ...
//
// On first boot (no file present) the compiled-in defaults are written out so
// the file can be edited offline and placed back on the card.

class ParameterStore {
public:
  static constexpr uint8_t SD_CS_PIN = BUILTIN_SDCARD;
  static constexpr const char *PARAMS_FILE = "params.cfg";

  // Initialise the SD card. Returns true on success.
  static bool begin();

  // Load parameters from PARAMS_FILE into `params`.
  // If the file does not exist the current values are saved as defaults.
  // Returns true if values were loaded from disk.
  template <std::size_t N> static bool load(std::array<Parameter, N> &params);

  // Persist the current values of `params` to PARAMS_FILE.
  // Returns true on success.
  template <std::size_t N>
  static bool save(const std::array<Parameter, N> &params);

private:
  static bool sd_ready;

  template <std::size_t N>
  static void apply_line(char *line, std::array<Parameter, N> &params);
};

template <std::size_t N>
bool ParameterStore::load(std::array<Parameter, N> &params) {
  if (!sd_ready) {
    return false;
  }

  if (!SD.exists(PARAMS_FILE)) {
    save(params);
    return false;
  }

  File f = SD.open(PARAMS_FILE, FILE_READ);
  if (!f) {
    return false;
  }

  char line[64];
  uint8_t pos = 0;

  while (f.available()) {
    char c = static_cast<char>(f.read());
    if (c == '\r') {
      continue;
    }
    if (c == '\n' || pos == sizeof(line) - 1) {
      line[pos] = '\0';
      pos = 0;
      if (line[0] != '#' && line[0] != '\0') {
        apply_line(line, params);
      }
    } else {
      line[pos++] = c;
    }
  }
  if (pos > 0) {
    line[pos] = '\0';
    if (line[0] != '#') {
      apply_line(line, params);
    }
  }

  f.close();
  return true;
}

template <std::size_t N>
bool ParameterStore::save(const std::array<Parameter, N> &params) {
  if (!sd_ready) {
    return false;
  }

  if (SD.exists(PARAMS_FILE)) {
    SD.remove(PARAMS_FILE);
  }

  File f = SD.open(PARAMS_FILE, FILE_WRITE);
  if (!f) {
    return false;
  }

  f.println("# VCU parameter file - edit values then reboot");
  f.println("# Format: NAME=VALUE  (uint32 decimal)");
  f.println("#");

  for (const auto &p : params) {
    if (p.name == nullptr) {
      continue;
    }
    f.print(p.name);
    f.print('=');
    f.println(p.value);
  }

  f.close();
  return true;
}

template <std::size_t N>
void ParameterStore::apply_line(char *line, std::array<Parameter, N> &params) {
  char *eq = line;
  while (*eq != '\0' && *eq != '=') {
    ++eq;
  }
  if (*eq != '=') {
    return;
  }

  *eq = '\0';
  const char *key = line;
  uint32_t value = static_cast<uint32_t>(strtoul(eq + 1, nullptr, 10));

  for (auto &p : params) {
    if (p.name != nullptr && strcmp(p.name, key) == 0) {
      p.value = value;
      return;
    }
  }
}
