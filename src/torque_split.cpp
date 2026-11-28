#include "torque_split.hpp"
#include "car_model.hpp"
#include <cmath>

using vehicle::Position;

namespace torque_split {

namespace {

bool is_front(Position p) { return p == Position::FL || p == Position::FR; }

double clamp(double x, double lo, double hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}

} // namespace

double front_share(Mode mode, double front_bias, double vx, double ax) {
  using namespace car_model;
  if (mode == Mode::FIXED)
    return clamp(front_bias, 0, 1);
  const double down = 0.5 * RHO * vx * vx * CLA;
  const double front = MASS_KG * G * FRONT_WEIGHT -
                       MASS_KG * ax * CG_HEIGHT_M / WHEELBASE_M +
                       down * AERO_FRONT;
  const double total = MASS_KG * G + down;
  return clamp(front / total, 0, 1);
}

void split(double ask_nm, double front, const vehicle::Drive *drives,
           const double *cap_nm, size_t n, double *wheel_nm) {
  if (!(ask_nm > 0))
    ask_nm = 0;
  double cap_f = 0, cap_r = 0;
  for (size_t i = 0; i < n; i++)
    (is_front(drives[i].position) ? cap_f : cap_r) += cap_nm[i];

  double t_f = std::fmin(front * ask_nm, cap_f);
  const double t_r = std::fmin(ask_nm - t_f, cap_r);
  t_f = std::fmin(ask_nm - t_r, cap_f);

  for (size_t i = 0; i < n; i++) {
    const bool f = is_front(drives[i].position);
    const double axle = f ? t_f : t_r, cap = f ? cap_f : cap_r;
    wheel_nm[i] = cap > 0 ? axle * cap_nm[i] / cap : 0;
  }
}

} // namespace torque_split
