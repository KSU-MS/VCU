// Sharing the driver's wheel torque between motors (torque_split.hpp):
// kmslap's fixed and load modes, on the hub cars' drive tables. Pure
// arithmetic, so it runs in the default build.
#include <sim_test_main.hpp>

#include <car_model.hpp>
#include <cmath>
#include <random>
#include <torque_split.hpp>
#include <vehicle.hpp>

using namespace torque_split;
using vehicle::FOUR_WD_DTI_DRIVES;
using vehicle::AWD_EMRAX_DTI_DRIVES;

namespace {

// DTI F-MOT peak at the wheel: 31.6 N m x 13 x 0.97
const double HUB_CAP = 31.6 * 13.0 * 0.97;

double sum(const double *x, size_t n) {
  double t = 0;
  for (size_t i = 0; i < n; i++)
    t += x[i];
  return t;
}

} // namespace

TEST_CASE("fixed: the front gets its bias, left and right alike") {
  const double cap[4] = {HUB_CAP, HUB_CAP, HUB_CAP, HUB_CAP};
  double wheel[4];
  split(1000, front_share(Mode::FIXED, 0.45, 0, 0), FOUR_WD_DTI_DRIVES, cap,
        4, wheel);
  CHECK(wheel[0] == doctest::Approx(225));
  CHECK(wheel[1] == doctest::Approx(225));
  CHECK(wheel[2] == doctest::Approx(275));
  CHECK(wheel[3] == doctest::Approx(275));
}

TEST_CASE("load: the static weight standing still, more to the rear "
          "accelerating, and downforce shifts it to its own balance") {
  CHECK(front_share(Mode::LOAD, 0, 0, 0) ==
        doctest::Approx(car_model::FRONT_WEIGHT));
  // 1 g forward: m g h / L of the weight moves back
  const double transfer = car_model::CG_HEIGHT_M / car_model::WHEELBASE_M;
  CHECK(front_share(Mode::LOAD, 0, 0, car_model::G) ==
        doctest::Approx(car_model::FRONT_WEIGHT - transfer));
  // Braking moves it forward
  CHECK(front_share(Mode::LOAD, 0, 0, -car_model::G) >
        car_model::FRONT_WEIGHT);
  // Very fast, downforce dominates: towards its 45 % on the front
  const double fast = front_share(Mode::LOAD, 0, 200, 0);
  CHECK(fast < car_model::FRONT_WEIGHT);
  CHECK(fast > car_model::AERO_FRONT);
}

TEST_CASE("an axle at its limit hands the rest to the other one") {
  const double cap[4] = {HUB_CAP, HUB_CAP, HUB_CAP, HUB_CAP};
  double wheel[4];
  // All of it asked of the front: it can take 2 x HUB_CAP, the rear the rest
  split(3 * HUB_CAP, 1.0, FOUR_WD_DTI_DRIVES, cap, 4, wheel);
  CHECK(wheel[0] == doctest::Approx(HUB_CAP));
  CHECK(wheel[1] == doctest::Approx(HUB_CAP));
  CHECK(wheel[2] + wheel[3] == doctest::Approx(HUB_CAP));
}

TEST_CASE("more than every motor can do: each at its limit, no more") {
  const double cap[4] = {HUB_CAP, HUB_CAP, HUB_CAP, HUB_CAP};
  double wheel[4];
  split(10 * HUB_CAP, 0.5, FOUR_WD_DTI_DRIVES, cap, 4, wheel);
  for (double w : wheel)
    CHECK(w == doctest::Approx(HUB_CAP));
}

TEST_CASE("the EMRAX and the front hubs share by what each can do") {
  // EMRAX: 240 N m x 4.18 x 0.95 at the rear axle; two hubs at the front
  const double emrax = 240 * 4.18 * 0.95;
  const double cap[3] = {emrax, HUB_CAP, HUB_CAP};
  double wheel[3];
  split(1000, 0.3, AWD_EMRAX_DTI_DRIVES, cap, 3, wheel);
  CHECK(wheel[0] == doctest::Approx(700));
  CHECK(wheel[1] == doctest::Approx(150));
  CHECK(wheel[2] == doctest::Approx(150));
}

TEST_CASE("[EV.3.1.3] the motors' total is never more than the ask, nor "
          "any motor over its limit (x5000 random cases)") {
  std::mt19937 rng(3);
  std::uniform_real_distribution<double> u(0, 1);
  int checked = 0;
  for (int k = 0; k < 5000; k++) {
    double cap[4];
    for (double &c : cap)
      c = u(rng) < 0.1 ? 0 : u(rng) * HUB_CAP; // sometimes a motor's out
    const double ask = u(rng) < 0.05 ? -50 : u(rng) * 5 * HUB_CAP;
    const Mode mode = u(rng) < 0.5 ? Mode::FIXED : Mode::LOAD;
    const double front =
        front_share(mode, u(rng), u(rng) * 40, (u(rng) - 0.5) * 30);
    double wheel[4];
    split(ask, front, FOUR_WD_DTI_DRIVES, cap, 4, wheel);
    CHECK(sum(wheel, 4) <= std::fmax(ask, 0) + 1e-9);
    for (int i = 0; i < 4; i++) {
      CHECK(wheel[i] >= 0);
      CHECK(wheel[i] <= cap[i] + 1e-9);
    }
    // and all of it when the motors can
    if (ask > 0 && ask <= sum(cap, 4))
      CHECK(sum(wheel, 4) == doctest::Approx(ask));
    checked++;
  }
  CHECK(checked == 5000);
}
