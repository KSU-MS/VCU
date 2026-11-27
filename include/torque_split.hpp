#pragma once
// The driver's wheel torque shared out to the motors on a car with more
// than one: kmslap's TorqueController "fixed" and "load" modes, without the
// yaw control (python-data-analysis kmslap/vcu/torque_control.py).
//
// The front axle gets its share (a fixed bias, or its share of the load:
// weight, load transfer from the acceleration, downforce), capped at what
// its motors can do, the rear the rest, capped likewise, and the front
// takes back what the rear couldn't. Within an axle, each motor by its
// capability. The total is never more than the ask.
#include <stddef.h>
#include <stdint.h>

#include "vehicle.hpp"

namespace torque_split {

enum class Mode : uint8_t { FIXED, LOAD };

// The front axle's share of the total, 0..1. vx m/s, ax m/s^2 (forward)
double front_share(Mode mode, double front_bias, double vx, double ax);

// ask_nm: the total wheel torque asked for (drive only, >= 0).
// cap_nm[i]: what drive i can put on its wheel(s) now. Writes each drive's
// wheel torque to wheel_nm[i]
void split(double ask_nm, double front, const vehicle::Drive *drives,
           const double *cap_nm, size_t n, double *wheel_nm);

} // namespace torque_split
