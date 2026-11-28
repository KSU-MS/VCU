#pragma once
// Which car the firmware is for (docs/vehicles.md). The five drivetrains
// kmslap simulates, named the same, with the same inverters as
// ksu-ms-dbc's gen_tool/vehicles.json. platformio.ini picks one with
// -D VEHICLE_<NAME>, along with that car's CAN library (can_lib_<name>).
// The node IDs are vehicles.json's: AMK uses 1, 2, 5, 6, since node n's
// Actual Values 2 sits at n + 2 and would collide.
//
// rwd_emrax builds; 4wd_dti builds natively (tests) while its VCU side is
// written; the others need their inverters first.
#include <stddef.h>
#include <stdint.h>

#include "motors.hpp"
#include "parameters.hpp"

namespace vehicle {

// Where a motor drives: one hub each, or the rear axle through the chain
// and diff
enum class Position : uint8_t { FL, FR, RL, RR, REAR_AXLE };
enum class InverterKind : uint8_t { CASCADIA_PM100, DTI_HV500, AMK_KW26 };

struct Drive {
  Position position;
  InverterKind inverter;
  const MotorSpec *motor;
  double gear_ratio; // motor revs per wheel rev
  double gear_eff;   // wheel torque = motor torque x gear_ratio x gear_eff
  uint8_t node;      // the inverter's CAN node ID (DTI, AMK); 0 for Cascadia
};

struct Vehicle {
  const char *name;
  const Drive *drives;
  size_t drive_count;
  // The brake pedal travel that's regen only, before the hydraulics
  // (T.3.2.4a: up to 0.9), as a fraction. 0: no regen. Needs a brake
  // pedal travel sensor and a pedal box with that free travel: today's car
  // measures pressure, so it has none
  double regen_brake_travel;
};

// kmslap's Car.hub_ratio and hub_gear_eff: ESTIMATES until the hub
// gearbox is chosen; driveline_eff for the EMRAX's chain
constexpr double HUB_RATIO = 13.0;
constexpr double HUB_GEAR_EFF = 0.97;
constexpr double CHAIN_EFF = 0.95;

constexpr Drive RWD_EMRAX_DRIVES[] = {
    {Position::REAR_AXLE, InverterKind::CASCADIA_PM100, &motors::EMRAX_228_HV,
     GEAR_RATIO, CHAIN_EFF, 0},
};
constexpr Drive AWD_EMRAX_DTI_DRIVES[] = {
    {Position::REAR_AXLE, InverterKind::CASCADIA_PM100, &motors::EMRAX_228_HV,
     GEAR_RATIO, CHAIN_EFF, 0},
    {Position::FL, InverterKind::DTI_HV500, &motors::DTI_FMOT,
     HUB_RATIO, HUB_GEAR_EFF, 1},
    {Position::FR, InverterKind::DTI_HV500, &motors::DTI_FMOT,
     HUB_RATIO, HUB_GEAR_EFF, 2},
};
constexpr Drive AWD_EMRAX_AMK_DRIVES[] = {
    {Position::REAR_AXLE, InverterKind::CASCADIA_PM100, &motors::EMRAX_228_HV,
     GEAR_RATIO, CHAIN_EFF, 0},
    {Position::FL, InverterKind::AMK_KW26, &motors::AMK_DD5,
     HUB_RATIO, HUB_GEAR_EFF, 1},
    {Position::FR, InverterKind::AMK_KW26, &motors::AMK_DD5,
     HUB_RATIO, HUB_GEAR_EFF, 2},
};
constexpr Drive FOUR_WD_DTI_DRIVES[] = {
    {Position::FL, InverterKind::DTI_HV500, &motors::DTI_FMOT,
     HUB_RATIO, HUB_GEAR_EFF, 1},
    {Position::FR, InverterKind::DTI_HV500, &motors::DTI_FMOT,
     HUB_RATIO, HUB_GEAR_EFF, 2},
    {Position::RL, InverterKind::DTI_HV500, &motors::DTI_FMOT,
     HUB_RATIO, HUB_GEAR_EFF, 3},
    {Position::RR, InverterKind::DTI_HV500, &motors::DTI_FMOT,
     HUB_RATIO, HUB_GEAR_EFF, 4},
};
constexpr Drive FOUR_WD_AMK_DRIVES[] = {
    {Position::FL, InverterKind::AMK_KW26, &motors::AMK_DD5,
     HUB_RATIO, HUB_GEAR_EFF, 1},
    {Position::FR, InverterKind::AMK_KW26, &motors::AMK_DD5,
     HUB_RATIO, HUB_GEAR_EFF, 2},
    {Position::RL, InverterKind::AMK_KW26, &motors::AMK_DD5,
     HUB_RATIO, HUB_GEAR_EFF, 5},
    {Position::RR, InverterKind::AMK_KW26, &motors::AMK_DD5,
     HUB_RATIO, HUB_GEAR_EFF, 6},
};

template <size_t N>
constexpr Vehicle make(const char *name, const Drive (&drives)[N],
                       double regen_brake_travel) {
  return {name, drives, N, regen_brake_travel};
}

constexpr Vehicle ALL[] = {
    // The hub cars: kmslap's 0.9 (laptime.run's default with hub motors)
    make("rwd_emrax", RWD_EMRAX_DRIVES, 0.0),
    make("awd_emrax_dti", AWD_EMRAX_DTI_DRIVES, 0.9),
    make("awd_emrax_amk", AWD_EMRAX_AMK_DRIVES, 0.9),
    make("4wd_dti", FOUR_WD_DTI_DRIVES, 0.9),
    make("4wd_amk", FOUR_WD_AMK_DRIVES, 0.9),
};

// The one this build is for, and which inverter code it needs
#if defined(VEHICLE_RWD_EMRAX)
static constexpr const Vehicle &THIS = ALL[0];
#define VEHICLE_HAS_CASCADIA 1
#elif defined(VEHICLE_4WD_DTI)
static constexpr const Vehicle &THIS = ALL[3];
#define VEHICLE_HAS_DTI 1
#elif defined(VEHICLE_AWD_EMRAX_DTI) || defined(VEHICLE_AWD_EMRAX_AMK) ||      \
    defined(VEHICLE_4WD_AMK)
#error "only rwd_emrax and 4wd_dti build so far: this car's inverters aren't written yet (docs/vehicles.md)"
#else
#error "no vehicle chosen: platformio.ini sets -D VEHICLE_<NAME>"
#endif

} // namespace vehicle
