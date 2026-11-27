#pragma once
// The motors the cars can have, from kmslap's data
// (python-data-analysis datasheets/*.toml, kmslap/car/motor.py), so the VCU
// and the simulation agree. 0 where kmslap has no number and nothing here
// needs one yet.

struct MotorSpec {
  const char *model;
  double peak_torque_nm;
  double nominal_torque_nm; // AMK's torque units are 0.1 % of this (MN)
  double max_rpm;
  int pole_pairs;
  double kt_nm_per_arms; // torque constant, N m per A rms
};

namespace motors {

// The car today, through the chain. kmslap: PEAK_TORQUE, Motor('228HV')
constexpr MotorSpec EMRAX_228_HV{"EMRAX 228 HV", 230.0, 0.0, 6500.0, 10, 0.0};

// DTI F-MOT hub motor (datasheets/dti_fmot.toml)
constexpr MotorSpec DTI_FMOT{"DTI F-MOT", 31.6, 11.1, 20000.0, 4, 0.492};

// AMK DD5-14-10-POW hub motor (datasheets/amk_dd5.toml). Pole pairs and kt
// aren't in kmslap's data; its inverter takes torque in % of MN, not current
constexpr MotorSpec AMK_DD5{"AMK DD5-14-10-POW", 21.0, 9.8, 20000.0, 0, 0.0};

} // namespace motors
