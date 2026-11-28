#pragma once
// What the VCU believes about the car, for what needs a model of it
// (traction control's controller, the torque split by axle load): kmslap's
// numbers (python-data-analysis, FirmwareController.config, Car)
namespace car_model {

constexpr double MASS_KG = 285.0;
constexpr double G = 9.81;
constexpr double CG_HEIGHT_M = 0.2751;
constexpr double WHEELBASE_M = 1.61544;
constexpr double CG_TO_FRONT_M = 0.80772; // front axle to CG
// Static weight on the front axle, as a fraction
constexpr double FRONT_WEIGHT = 1.0 - CG_TO_FRONT_M / WHEELBASE_M;
constexpr double CLA = 1.69;        // lift coefficient x area, m^2
constexpr double CDA = 1.3;         // drag coefficient x area, m^2
constexpr double RHO = 1.226;       // air, kg/m^3
constexpr double CRR = 0.015;       // rolling resistance
constexpr double AERO_FRONT = 0.45; // share of downforce on the front axle

} // namespace car_model
