// Pedals on their own: travel mapping, sensor range faults and the
// APPS/BSE plausibility checks from the rules.
#include <sim_test_main.hpp>

#include <parameters.hpp>
#include <pedal_handeler.hpp>
#include <vector>

namespace {

Pedals make_pedals() {
  return Pedals(MIN_BRAKE_PEDAL, START_BRAKE_PEDAL, END_BRAKE_PEDAL,
                MAX_BRAKE_PEDAL, MIN_APPS_PEDAL, START_ACCELERATOR_PEDAL_1,
                END_ACCELERATOR_PEDAL_1, MAX_ACCELERATOR_PEDAL_1,
                START_ACCELERATOR_PEDAL_2, END_ACCELERATOR_PEDAL_2,
                MAX_ACCELERATOR_PEDAL_2);
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

  // Past full press, but short of the fault thresholds
  p.update_travel((END_ACCELERATOR_PEDAL_1 + MAX_ACCELERATOR_PEDAL_1) / 2,
                  (END_ACCELERATOR_PEDAL_2 + MAX_ACCELERATOR_PEDAL_2) / 2,
                  START_BRAKE_PEDAL);
  CHECK_FALSE(p.get_apps_fault_ok_low());
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
//// Sensor faults: open, shorted, unplugged (T.4.2.10, T.4.3.4)
////
//// What a fault looks like at the ADC depends on the board. An unplugged
//// sensor reads 0 with a pull-down on its input (the rules want one, so an
//// open circuit can be detected), a short to ground reads 0, a short to the
//// sensor supply reads full scale (2047 on the ADS1018 for the APPS; ~995
//// through A10's divider for the brake, ~1023 shorted to GLV).

namespace {
constexpr uint16_t ADS_FULL_SCALE = 2047;
const uint16_t BRAKE_SHORTED_TO_5V = uint16_t(5.0 * 12.0 / 18.7 / 3.3 * 1023);
constexpr uint16_t BRAKE_SHORTED_TO_GLV = 1023;

struct Fault {
  const char *name;
  uint16_t raw;
};

std::vector<Fault> apps_faults(uint16_t high_fault) {
  return {{"unplugged (pulled down)", 0},
          {"shorted to ground", 0},
          {"shorted to the sensor supply", ADS_FULL_SCALE},
          {"one count under the range", MIN_APPS_PEDAL - 1},
          {"one count over the range", uint16_t(high_fault + 1)}};
}
} // namespace

TEST_CASE("every way an APPS can fail kills throttle, at any pedal position") {
  for (int sensor : {1, 2}) {
    for (const Fault &f :
         apps_faults(sensor == 1 ? MAX_ACCELERATOR_PEDAL_1 : MAX_ACCELERATOR_PEDAL_2)) {
      for (double pedal : {0.0, 0.08, 0.5, 1.0}) {
        CAPTURE(sensor);
        CAPTURE(f.name);
        CAPTURE(pedal);
        Pedals p = make_pedals();
        press(p, pedal);
        uint16_t a1 = apps1_at(pedal), a2 = apps2_at(pedal);
        (sensor == 1 ? a1 : a2) = f.raw;
        p.update_travel(a1, a2, brake_at(0));

        CHECK(p.get_apps_fault_ok_low());
        CHECK(p.get_travel() == 0);
        // and the broken sensor doesn't keep reporting its last good value
        CHECK((sensor == 1 ? p.get_apps1_travel() : p.get_apps2_travel()) == 0);
      }
    }
  }
}

TEST_CASE("an APPS unplugged at light throttle can't drive the car once released") {
  // The old code kept a broken sensor's last travel, and the "pedals
  // released" check then cleared the fault with it: APPS1 unplugged at 8%,
  // foot off, and the torque request was the stale 8% with no fault shown
  Pedals p = make_pedals();
  press(p, 0.08);
  p.update_travel(0, apps2_at(0.08), brake_at(0)); // APPS1 unplugged
  p.update_travel(0, apps2_at(0), brake_at(0));    // foot off
  CHECK(p.get_apps_fault_ok_low());
  CHECK(p.get_travel() == 0);

  for (double pedal : {0.0, 0.3, 1.0}) {
    p.update_travel(0, apps2_at(pedal), brake_at(0));
    CHECK(p.get_apps_fault_ok_low());
    CHECK(p.get_travel() == 0);
  }
}

TEST_CASE("an APPS fault clears once the sensor is back and the pedal released") {
  Pedals p = make_pedals();
  p.update_travel(apps1_at(0.5), ADS_FULL_SCALE, brake_at(0));
  REQUIRE(p.get_apps_fault_ok_low());

  // Sensor back, but still on the pedal
  press(p, 0.5);
  CHECK(p.get_apps_fault_ok_low());
  CHECK(p.get_travel() == 0);

  press(p, 0);
  CHECK_FALSE(p.get_apps_fault_ok_low());
  press(p, 0.5);
  CHECK(p.get_travel() > 0.4);
}

TEST_CASE("both APPS failing the same way is still a fault") {
  Pedals p = make_pedals();
  SUBCASE("both unplugged") { p.update_travel(0, 0, brake_at(0)); }
  SUBCASE("both shorted to the supply") {
    // They'd agree perfectly at 100% travel if only the clamped travel was
    // looked at
    p.update_travel(ADS_FULL_SCALE, ADS_FULL_SCALE, brake_at(0));
  }
  CHECK(p.get_apps_fault_ok_low());
  CHECK(p.get_travel() == 0);
}

TEST_CASE("APPS1 and APPS2 shorted together never make torque") {
  // Both inputs read the same voltage. The two sensors have different
  // transfer functions, so whatever that voltage is, they either disagree,
  // read out of range, or both read 0 travel.
  for (uint32_t raw = 0; raw <= ADS_FULL_SCALE; raw++) {
    CAPTURE(raw);
    Pedals p = make_pedals();
    p.update_travel(uint16_t(raw), uint16_t(raw), brake_at(0));
    CHECK(p.get_travel() == 0);
  }
}

TEST_CASE("an APPS stuck at one reading is caught when the pedal moves") {
  // Frozen ADC channel, or a sensor whose linkage has come off
  Pedals p = make_pedals();
  press(p, 0.1);
  const uint16_t stuck = apps1_at(0.1);
  p.update_travel(stuck, apps2_at(1.0), brake_at(0));
  CHECK(p.get_apps_fault_ok_low());
  CHECK(p.get_travel() == 0);
}

TEST_CASE("every way the brake sensor can fail is a fault, and reads no brake") {
  const Fault faults[] = {
      {"unplugged (pulled down)", 0},
      {"shorted to ground", 0},
      {"shorted to 5 V", BRAKE_SHORTED_TO_5V},
      {"shorted to GLV (clamped)", BRAKE_SHORTED_TO_GLV},
      {"one count under the range", MIN_BRAKE_PEDAL - 1},
      {"one count over the range", MAX_BRAKE_PEDAL + 1},
  };
  for (const Fault &f : faults) {
    for (double brake : {0.0, 0.6}) {
      CAPTURE(f.name);
      CAPTURE(brake);
      Pedals p = make_pedals();
      press(p, 0.5, brake);
      p.update_travel(apps1_at(0.5), apps2_at(0.5), f.raw);
      CHECK(p.get_bse_fault_ok_low());
      CHECK(p.get_travel() == 0);
      // Not the last good reading: that's what lets the car into ready to
      // drive, and what the brake plausibility check looks at
      CHECK(p.get_brake_travel() == 0);
    }
  }
}

TEST_CASE("a hard brake press isn't mistaken for a short") {
  Pedals p = make_pedals();
  press(p, 0, 1.0);
  CHECK_FALSE(p.get_bse_fault_ok_low());
  CHECK(p.get_brake_travel() == doctest::Approx(1.0));
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

