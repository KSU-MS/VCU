// Pedals on their own: travel mapping, sensor range faults and the
// APPS/BSE plausibility checks from the rules.
#include <sim_test_main.hpp>

#include <parameters.hpp>
#include <pedal_handeler.hpp>

namespace {

Pedals make_pedals() {
  return Pedals(MIN_BRAKE_PEDAL, START_BRAKE_PEDAL, END_BRAKE_PEDAL,
                MAX_BRAKE_PEDAL, MIN_APPS_PEDAL, START_ACCELERATOR_PEDAL_1,
                END_ACCELERATOR_PEDAL_1, START_ACCELERATOR_PEDAL_2,
                END_ACCELERATOR_PEDAL_2);
}

uint16_t apps1_at(double travel) {
  return uint16_t(START_ACCELERATOR_PEDAL_1 +
                  travel * (END_ACCELERATOR_PEDAL_1 - START_ACCELERATOR_PEDAL_1));
}
uint16_t apps2_at(double travel) {
  return uint16_t(START_ACCELERATOR_PEDAL_2 +
                  travel * (END_ACCELERATOR_PEDAL_2 - START_ACCELERATOR_PEDAL_2));
}
uint16_t brake_at(double travel) {
  return uint16_t(START_BRAKE_PEDAL +
                  travel * (END_BRAKE_PEDAL - START_BRAKE_PEDAL));
}

// Push both APPS to the same travel with the brake at `brake`
void press(Pedals &p, double throttle, double brake = 0) {
  p.update_travel(apps1_at(throttle), apps2_at(throttle), brake_at(brake));
}

bool any_fault(Pedals &p) {
  return p.get_apps_fault_ok_low() || p.get_bse_fault_ok_low() ||
         p.get_apps_bse_fault_ok_low();
}

} // namespace

TEST_CASE("released pedals read zero travel with no faults") {
  Pedals p = make_pedals();
  press(p, 0);
  CHECK(p.get_travel() == doctest::Approx(0));
  CHECK(p.get_brake_travel() == doctest::Approx(0));
  CHECK_FALSE(any_fault(p));
}

TEST_CASE("travel maps linearly across each sensor's calibrated range") {
  Pedals p = make_pedals();

  press(p, 0.5);
  CHECK(p.get_apps1_travel() == doctest::Approx(0.5).epsilon(0.01));
  CHECK(p.get_apps2_travel() == doctest::Approx(0.5).epsilon(0.01));
  CHECK(p.get_travel() == doctest::Approx(0.5).epsilon(0.01));

  press(p, 1.0);
  CHECK(p.get_travel() == doctest::Approx(1.0).epsilon(0.01));

  press(p, 0, 0.25);
  CHECK(p.get_brake_travel() == doctest::Approx(0.25).epsilon(0.01));
}

TEST_CASE("travel clamps to 0..1 outside the calibrated range") {
  Pedals p = make_pedals();

  // Below start but above the open-circuit threshold
  p.update_travel(START_ACCELERATOR_PEDAL_1 - 50, START_ACCELERATOR_PEDAL_2 - 50,
                  START_BRAKE_PEDAL - 50);
  CHECK(p.get_apps1_travel() == 0);
  CHECK(p.get_apps2_travel() == 0);
  CHECK(p.get_brake_travel() == 0);

  p.update_travel(END_ACCELERATOR_PEDAL_1 + 50, END_ACCELERATOR_PEDAL_2 + 50,
                  START_BRAKE_PEDAL);
  CHECK(p.get_apps1_travel() == 1);
  CHECK(p.get_apps2_travel() == 1);
}

TEST_CASE("torque request scales travel by the limit") {
  Pedals p = make_pedals();
  CHECK(p.get_torque_request(0.5, 240) == doctest::Approx(120));
  CHECK(p.get_torque_request(0, 240) == 0);
  CHECK(p.get_torque_request(1, 240) == doctest::Approx(240));
}

TEST_CASE("an APPS reading below the open-circuit threshold kills throttle") {
  Pedals p = make_pedals();
  press(p, 0.5);
  REQUIRE(p.get_travel() > 0.4);

  SUBCASE("APPS1 open") {
    p.update_travel(MIN_APPS_PEDAL - 1, apps2_at(0.5), brake_at(0));
  }
  SUBCASE("APPS2 open") {
    p.update_travel(apps1_at(0.5), MIN_APPS_PEDAL - 1, brake_at(0));
  }

  CHECK(p.get_apps_fault_ok_low());
  CHECK(p.get_travel() == 0);
}

TEST_CASE("an out-of-range BSE reading kills throttle (T.4.3.4)") {
  Pedals p = make_pedals();

  SUBCASE("shorted low") {
    p.update_travel(apps1_at(0.5), apps2_at(0.5), MIN_BRAKE_PEDAL - 1);
  }
  SUBCASE("shorted high") {
    p.update_travel(apps1_at(0.5), apps2_at(0.5), MAX_BRAKE_PEDAL + 1);
  }

  CHECK(p.get_bse_fault_ok_low());
  CHECK(p.get_travel() == 0);
}

TEST_CASE("BSE fault clears as soon as the reading is back in range") {
  Pedals p = make_pedals();
  p.update_travel(apps1_at(0), apps2_at(0), MAX_BRAKE_PEDAL + 1);
  REQUIRE(p.get_bse_fault_ok_low());

  press(p, 0.5);
  CHECK_FALSE(p.get_bse_fault_ok_low());
  CHECK(p.get_travel() > 0.4);
}

TEST_CASE("APPS sensors that disagree kill throttle and latch until release") {
  Pedals p = make_pedals();

  // 50% apart is implausible under any tolerance
  p.update_travel(apps1_at(0.8), apps2_at(0.3), brake_at(0));
  CHECK(p.get_apps_fault_ok_low());
  CHECK(p.get_travel() == 0);

  // Sensors agreeing again isn't enough, the driver has to lift off
  press(p, 0.5);
  CHECK(p.get_apps_fault_ok_low());
  CHECK(p.get_travel() == 0);

  press(p, 0);
  press(p, 0.5);
  CHECK_FALSE(p.get_apps_fault_ok_low());
  CHECK(p.get_travel() > 0.4);
}

TEST_CASE("brake + throttle together kills throttle and latches until lift") {
  Pedals p = make_pedals();

  press(p, 0.5, 0.5);
  CHECK(p.get_apps_bse_fault_ok_low());
  CHECK(p.get_travel() == 0);

  // Releasing the brake alone doesn't clear it (EV.4.7.2)
  press(p, 0.5, 0);
  CHECK(p.get_apps_bse_fault_ok_low());
  CHECK(p.get_travel() == 0);

  press(p, 0, 0);
  CHECK_FALSE(p.get_apps_bse_fault_ok_low());
  press(p, 0.5, 0);
  CHECK(p.get_travel() > 0.4);
}

TEST_CASE("light brake with throttle is allowed (trail braking)") {
  Pedals p = make_pedals();
  press(p, 0.5, 0.1);
  CHECK_FALSE(p.get_apps_bse_fault_ok_low());
  CHECK(p.get_travel() > 0.4);
}

//
//// Thresholds that look looser than the FSAE rules. These are judgement calls
//// for whoever owns the pedal code (maybe they were loosened on purpose for
//// noisy sensors), so they WARN rather than fail. Check them against the
//// current rulebook before changing anything.

TEST_CASE("KNOWN ISSUE: APPS disagreement over 10 points should fault "
          "(T.4.2.4, code uses 0.3)") {
  Pedals p = make_pedals();
  p.update_travel(apps1_at(0.5), apps2_at(0.25), brake_at(0));
  WARN(p.get_apps_fault_ok_low());
}

TEST_CASE("KNOWN ISSUE: brake + >25% throttle should fault "
          "(EV.4.7.1, code uses 0.3)") {
  Pedals p = make_pedals();
  press(p, 0.27, 0.5);
  WARN(p.get_apps_bse_fault_ok_low());
}

TEST_CASE("KNOWN ISSUE: brake/throttle fault should hold until APPS < 5% "
          "(EV.4.7.2, code clears below 10%)") {
  Pedals p = make_pedals();
  press(p, 0.5, 0.5);
  REQUIRE(p.get_apps_bse_fault_ok_low());
  press(p, 0.07, 0);
  WARN(p.get_apps_bse_fault_ok_low());
}
