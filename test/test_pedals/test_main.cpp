// Pedals on their own: travel mapping, sensor faults and the plausibility
// checks. Tests cite the rule they check in their name, e.g. [EV.4.7.2]:
// the requirements are in docs/requirements.md (FSAE Rules 2027).
#include <sim_test_main.hpp>

#include <parameters.hpp>
#include <pedal_handeler.hpp>
#include <cstdlib>
#include <string>
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

// The pedals are read every loop; each update here is 1 ms later
uint32_t clock_ms = 0;
void update(Pedals &p, uint16_t apps1, uint16_t apps2, uint16_t brake) {
  p.update_travel(apps1, apps2, brake, clock_ms++);
}

// The same reading every millisecond for `ms`
void hold(Pedals &p, uint16_t apps1, uint16_t apps2, uint16_t brake, uint32_t ms) {
  for (uint32_t i = 0; i < ms; i++)
    update(p, apps1, apps2, brake);
}

// Push both APPS to the same travel with the brake at `brake`
void press(Pedals &p, double throttle, double brake = 0) {
  update(p, apps1_at(throttle), apps2_at(throttle), brake_at(brake));
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

TEST_CASE("[T.4.2.3] the two APPS have different transfer functions, well apart (x91: 10-100% travel)") {
  // Different offsets and gradients
  CHECK(START_ACCELERATOR_PEDAL_1 != START_ACCELERATOR_PEDAL_2);
  CHECK(END_ACCELERATOR_PEDAL_1 - START_ACCELERATOR_PEDAL_1 !=
        END_ACCELERATOR_PEDAL_2 - START_ACCELERATOR_PEDAL_2);

  // Above 10% travel, the two outputs at least 10% apart. Read as 10% of
  // the ADC's full scale, since both share it.
  for (int i = 10; i <= 100; i++) {
    const double t = i / 100.0;
    CAPTURE(t);
    CHECK(std::abs(int(apps1_at(t)) - int(apps2_at(t))) >= 0.10 * 2047);
  }

  // An open circuit, pulled down to 0 counts, is out of range: a failure,
  // and no throttle
  CHECK(0 < MIN_APPS_PEDAL);
  Pedals p = make_pedals();
  update(p, 0, apps2_at(0), brake_at(0));
  CHECK(p.get_travel() == 0);
  CHECK(p.get_apps_fault_ok_low());
}

TEST_CASE("[T.4.2.10] travel clamps to 0..1 outside the calibrated range, short of a fault") {
  Pedals p = make_pedals();

  // Below start but above the open-circuit threshold
  update(p, START_ACCELERATOR_PEDAL_1 - 50, START_ACCELERATOR_PEDAL_2 - 50,
                  START_BRAKE_PEDAL - 50);
  CHECK(p.get_apps1_travel() == 0);
  CHECK(p.get_apps2_travel() == 0);
  CHECK(p.get_brake_travel() == 0);

  // Past full press, but short of the fault thresholds
  update(p, (END_ACCELERATOR_PEDAL_1 + MAX_ACCELERATOR_PEDAL_1) / 2,
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

TEST_CASE("[T.4.2.10] an APPS reading below the open-circuit threshold kills throttle") {
  Pedals p = make_pedals();
  press(p, 0.5);
  REQUIRE(p.get_travel() > 0.4);

  SUBCASE("APPS1 open") {
    update(p, MIN_APPS_PEDAL - 1, apps2_at(0.5), brake_at(0));
  }
  SUBCASE("APPS2 open") {
    update(p, apps1_at(0.5), MIN_APPS_PEDAL - 1, brake_at(0));
  }

  CHECK(p.get_apps_fault_ok_low());
  CHECK(p.get_travel() == 0);
}

TEST_CASE("[T.4.3.3][T.4.3.4] an out-of-range BSE reading kills throttle") {
  Pedals p = make_pedals();

  SUBCASE("shorted low") {
    update(p, apps1_at(0.5), apps2_at(0.5), MIN_BRAKE_PEDAL - 1);
  }
  SUBCASE("shorted high") {
    update(p, apps1_at(0.5), apps2_at(0.5), MAX_BRAKE_PEDAL + 1);
  }

  CHECK(p.get_bse_fault_ok_low());
  CHECK(p.get_travel() == 0);
}

TEST_CASE("BSE fault clears as soon as the reading is back in range") {
  Pedals p = make_pedals();
  update(p, apps1_at(0), apps2_at(0), MAX_BRAKE_PEDAL + 1);
  REQUIRE(p.get_bse_fault_ok_low());

  press(p, 0.5);
  CHECK_FALSE(p.get_bse_fault_ok_low());
  CHECK(p.get_travel() > 0.4);
}

TEST_CASE("[T.4.2.4][T.4.2.5] APPS sensors that disagree kill throttle and latch until release") {
  Pedals p = make_pedals();

  // 50% apart, for longer than T.4.2.5's 100 ms
  hold(p, apps1_at(0.8), apps2_at(0.3), brake_at(0), 150);
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

//
//// APPS plausibility (T.4.2.4, T.4.2.5): more than 10 points apart is an
//// implausibility; lasting over 100 ms, power must be cut. Here throttle is
//// cut the moment they disagree, and a disagreement over 100 ms latches
//// until the pedal is released.

namespace {
// APPS1 at `travel`, APPS2 `points` of travel lower
void apart(Pedals &p, double travel, double points, uint32_t ms = 1) {
  hold(p, apps1_at(travel), apps2_at(travel - points), brake_at(0), ms);
}
} // namespace

TEST_CASE("[T.4.2.4] the 10 point line") {
  Pedals p = make_pedals();
  apart(p, 0.6, 0.09, 200);
  CHECK_FALSE(p.get_apps_fault_ok_low());
  CHECK(p.get_travel() > 0.5);

  apart(p, 0.6, 0.11);
  CHECK(p.get_apps_fault_ok_low());
  CHECK(p.get_travel() == 0); // the moment they're too far apart
}

TEST_CASE("[T.4.2.5] a disagreement shorter than 100 ms cuts throttle only while it lasts") {
  // A glitch, or the two sensors' filters catching up after a stab of
  // throttle: no latched fault, no need to lift off
  Pedals p = make_pedals();
  press(p, 0.5);
  apart(p, 0.5, 0.2, 80);
  CHECK(p.get_travel() == 0);

  press(p, 0.5);
  CHECK_FALSE(p.get_apps_fault_ok_low());
  CHECK(p.get_travel() > 0.4);
}

TEST_CASE("[T.4.2.5] a disagreement over 100 ms latches until the pedal is released") {
  Pedals p = make_pedals();
  // Each update is 1 ms: the first disagreeing reading is t0, 100 ms later
  // is still within the rule, the reading after that isn't
  apart(p, 0.5, 0.2, 101);
  press(p, 0.5);
  CHECK_FALSE(p.get_apps_fault_ok_low()); // exactly 100 ms: not latched

  apart(p, 0.5, 0.2, 102);
  press(p, 0.5);
  CHECK(p.get_apps_fault_ok_low());
  CHECK(p.get_travel() == 0);

  press(p, 0);
  press(p, 0.5);
  CHECK_FALSE(p.get_apps_fault_ok_low());
  CHECK(p.get_travel() > 0.4);
}

TEST_CASE("[T.4.2.5] disagreements that come and go don't add up (x10)") {
  // Each one is timed from its own start
  Pedals p = make_pedals();
  for (int i = 0; i < 10; i++) {
    apart(p, 0.5, 0.2, 60);
    press(p, 0.5);
  }
  CHECK_FALSE(p.get_apps_fault_ok_low());
  CHECK(p.get_travel() > 0.4);
}

TEST_CASE("[T.4.2.4][T.4.2.9] a sensor drifting out of calibration is caught as it crosses 10 points") {
  Pedals p = make_pedals();
  bool cut = false;
  for (int step = 0; step <= 30 && !cut; step++) {
    double drift = step * 0.01;
    hold(p, apps1_at(0.5), apps2_at(0.5 - drift), brake_at(0), 20);
    cut = p.get_travel() == 0;
    CHECK(cut == (drift > 0.10 + 1e-9));
  }
  CHECK(cut);
}

TEST_CASE("[EV.4.7.1][EV.4.7.2] brake + throttle together kills throttle and latches until lift") {
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

//
//// Brake plausibility (EV.4.7): brakes actuated and APPS over 25% cuts power
//// until the APPS is under 5%, whatever the brake does.

TEST_CASE("[EV.4.7.1] a foot resting on the brake, short of where it bites, is fine") {
  Pedals p = make_pedals();
  press(p, 0.5, Pedals::BRAKE_ACTUATED_TRAVEL - 0.02);
  CHECK_FALSE(p.get_apps_bse_fault_ok_low());
  CHECK(p.get_travel() > 0.4);

  press(p, 0.5, Pedals::BRAKE_ACTUATED_TRAVEL + 0.02);
  CHECK(p.get_apps_bse_fault_ok_low());
  CHECK(p.get_travel() == 0);
}

TEST_CASE("[EV.4.7.1] the 25% line") {
  Pedals p = make_pedals();
  press(p, 0.24, 0.5);
  CHECK_FALSE(p.get_apps_bse_fault_ok_low());
  CHECK(p.get_travel() > 0.2);

  press(p, 0.26, 0.5);
  CHECK(p.get_apps_bse_fault_ok_low());
  CHECK(p.get_travel() == 0);
}

TEST_CASE("[EV.4.7.2] held until the APPS is under 5%, whatever the brake does") {
  Pedals p = make_pedals();
  press(p, 0.5, 0.5);
  REQUIRE(p.get_apps_bse_fault_ok_low());

  press(p, 0.5, 0); // brake off
  CHECK(p.get_apps_bse_fault_ok_low());
  press(p, 0.06, 0);
  CHECK(p.get_apps_bse_fault_ok_low());
  CHECK(p.get_travel() == 0);

  press(p, 0.04, 0);
  CHECK_FALSE(p.get_apps_bse_fault_ok_low());
  press(p, 0.5, 0);
  CHECK(p.get_travel() > 0.4);
}

TEST_CASE("[EV.4.7.1] either pedal first") {
  Pedals p = make_pedals();
  SUBCASE("on the throttle, then the brake") {
    press(p, 0.5);
    press(p, 0.5, 0.5);
  }
  SUBCASE("on the brake, then the throttle") {
    press(p, 0, 0.5);
    press(p, 0.5, 0.5);
  }
  CHECK(p.get_apps_bse_fault_ok_low());
  CHECK(p.get_travel() == 0);
}

TEST_CASE("[EV.4.7.1] either APPS over 25% trips it") {
  // Within the 10 points they're allowed to differ, the higher one counts
  Pedals p = make_pedals();
  SUBCASE("APPS1 higher") {
    update(p, apps1_at(0.27), apps2_at(0.20), brake_at(0.5));
  }
  SUBCASE("APPS2 higher") {
    update(p, apps1_at(0.20), apps2_at(0.27), brake_at(0.5));
  }
  CHECK(p.get_apps_bse_fault_ok_low());
  CHECK(p.get_travel() == 0);
}

TEST_CASE("the same rule releases an APPS fault: under 5%") {
  Pedals p = make_pedals();
  hold(p, apps1_at(0.8), apps2_at(0.3), brake_at(0), 150);
  REQUIRE(p.get_apps_fault_ok_low());
  press(p, 0.06);
  CHECK(p.get_apps_fault_ok_low());
  press(p, 0.04);
  CHECK_FALSE(p.get_apps_fault_ok_low());
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

// One SUBCASE per combination, each named, so each shows up on its own in
// the test report (doctest's version of pytest's parametrize)
std::string at_pedal(double pedal) {
  return " at " + std::to_string(int(pedal * 100)) + "%";
}

TEST_CASE("[T.4.2.9][T.4.2.10] every way an APPS can fail kills throttle, at any pedal position") {
  for (int sensor : {1, 2}) {
    for (const Fault &f :
         apps_faults(sensor == 1 ? MAX_ACCELERATOR_PEDAL_1 : MAX_ACCELERATOR_PEDAL_2)) {
      for (double pedal : {0.0, 0.08, 0.5, 1.0}) {
        SUBCASE(("APPS" + std::to_string(sensor) + " " + f.name + at_pedal(pedal)).c_str()) {
        Pedals p = make_pedals();
        press(p, pedal);
        uint16_t a1 = apps1_at(pedal), a2 = apps2_at(pedal);
        (sensor == 1 ? a1 : a2) = f.raw;
        update(p, a1, a2, brake_at(0));

        CHECK(p.get_apps_fault_ok_low());
        CHECK(p.get_travel() == 0);
        // and the broken sensor doesn't keep reporting its last good value
        CHECK((sensor == 1 ? p.get_apps1_travel() : p.get_apps2_travel()) == 0);
        }
      }
    }
  }
}

TEST_CASE("[T.4.2.9] an APPS unplugged at light throttle can't drive the car once released") {
  // The old code kept a broken sensor's last travel, and the "pedals
  // released" check then cleared the fault with it: APPS1 unplugged at 8%,
  // foot off, and the torque request was the stale 8% with no fault shown
  Pedals p = make_pedals();
  press(p, 0.08);
  update(p, 0, apps2_at(0.08), brake_at(0)); // APPS1 unplugged
  update(p, 0, apps2_at(0), brake_at(0));    // foot off
  CHECK(p.get_apps_fault_ok_low());
  CHECK(p.get_travel() == 0);

  for (double pedal : {0.0, 0.3, 1.0}) {
    update(p, 0, apps2_at(pedal), brake_at(0));
    CHECK(p.get_apps_fault_ok_low());
    CHECK(p.get_travel() == 0);
  }
}

TEST_CASE("an APPS fault clears once the sensor is back and the pedal released") {
  Pedals p = make_pedals();
  update(p, apps1_at(0.5), ADS_FULL_SCALE, brake_at(0));
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

TEST_CASE("[T.4.2.9][T.4.2.10] both APPS failing the same way is still a fault") {
  Pedals p = make_pedals();
  SUBCASE("both unplugged") { update(p, 0, 0, brake_at(0)); }
  SUBCASE("both shorted to the supply") {
    // They'd agree perfectly at 100% travel if only the clamped travel was
    // looked at
    update(p, ADS_FULL_SCALE, ADS_FULL_SCALE, brake_at(0));
  }
  CHECK(p.get_apps_fault_ok_low());
  CHECK(p.get_travel() == 0);
}

TEST_CASE("[T.4.2.3][T.4.2.9] APPS1 and APPS2 shorted together never make torque (x2048: every ADC value)") {
  // Both inputs read the same voltage. The two sensors have different
  // transfer functions, so whatever that voltage is, they either disagree,
  // read out of range, or both read 0 travel.
  for (uint32_t raw = 0; raw <= ADS_FULL_SCALE; raw++) {
    CAPTURE(raw);
    Pedals p = make_pedals();
    update(p, uint16_t(raw), uint16_t(raw), brake_at(0));
    CHECK(p.get_travel() == 0);
  }
}

TEST_CASE("[T.4.2.9] an APPS stuck at one reading is caught when the pedal moves") {
  // Frozen ADC channel, or a sensor whose linkage has come off
  Pedals p = make_pedals();
  press(p, 0.1);
  const uint16_t stuck = apps1_at(0.1);
  update(p, stuck, apps2_at(1.0), brake_at(0));
  CHECK(p.get_apps_fault_ok_low());
  CHECK(p.get_travel() == 0);
}

TEST_CASE("[T.4.3.3][T.4.3.4] every way the brake sensor can fail is a fault, and reads no brake") {
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
      SUBCASE((std::string(f.name) + (brake > 0 ? " while braking" : " with the brake off")).c_str()) {
      Pedals p = make_pedals();
      press(p, 0.5, brake);
      update(p, apps1_at(0.5), apps2_at(0.5), f.raw);
      CHECK(p.get_bse_fault_ok_low());
      CHECK(p.get_travel() == 0);
      // Not the last good reading: that's what lets the car into ready to
      // drive, and what the brake plausibility check looks at
      CHECK(p.get_brake_travel() == 0);
      }
    }
  }
}

TEST_CASE("[T.4.3.4] a hard brake press isn't mistaken for a short") {
  Pedals p = make_pedals();
  press(p, 0, 1.0);
  CHECK_FALSE(p.get_bse_fault_ok_low());
  CHECK(p.get_brake_travel() == doctest::Approx(1.0));
}
