// Traction control on the VCU: kmslap's controller (lib/vcu_control) wired
// between the pedal map and the inverter (traction_control.hpp). The
// controller itself is tuned in kmslap's sim; these check the VCU end: the
// frames it reads, the torque path, the on/off rules and what happens when
// its inputs go away. docs/traction-control.md.
#include <sim_test_main.hpp>

#include <algorithm>
#include <cmath>
#include <random>
#include <sim_car.hpp>
#include <sim_launch.hpp>
#include <traction_control.hpp>

using sim::Car;
using sim::Launch;

namespace {

// The 0x0CC status frame (tools/can/vcu_local.dbc), decoded
struct TcStatus {
  int selected, active, reasons;
  double slip, speed, scale, grip;
};

bool last_tc_status(TcStatus *out) {
  const can_message *m =
      sim::bus(TEENSY_CAN3).last_sent(CAN_ID_VCU_TRACTION_CONTROL);
  if (!m)
    return false;
  const uint8_t *d = m->buf.byte;
  out->selected = d[0] & 3;
  out->active = (d[0] >> 2) & 3;
  out->reasons = d[1];
  out->slip = int16_t(d[2] | d[3] << 8) * 0.001;
  out->speed = uint16_t(d[4] | d[5] << 8) * 0.01;
  out->scale = d[6] * 0.01;
  out->grip = d[7] * 0.02;
  return true;
}

TcStatus tc_status() {
  TcStatus s{};
  REQUIRE(last_tc_status(&s));
  return s;
}

bool has_reason(int reason) { return (tc_status().reasons & reason) != 0; }

double commanded_torque(Car &car) {
  Car::InverterCommand cmd{};
  REQUIRE(car.last_inverter_command(&cmd));
  return cmd.torque_nm;
}

// Ready to drive and rolling forward a little, sensors talking
void ready(Car &car, Launch &launch) {
  car.drive_to_rtd();
  REQUIRE(car.vcu.get_current_state() == READY_TO_DRIVE);
  launch.run(car, 300);
}

// Full throttle from a standstill for `ms`, the pedal going down over
// 100 ms like kmslap's driver (KS5e drivers: 60-100 ms). Slip over the run
// after the first `settle_ms`: the peak and the mean.
//
// The first few hundred ms aren't checked: with the VCU's sensor rates both
// modes overshoot there, the tires spin briefly before the controller
// catches them. Tuning that is kmslap's job (docs/traction-control.md).
struct LaunchResult {
  double speed, peak_slip, mean_slip;
};

LaunchResult full_throttle(Car &car, Launch &launch, uint32_t ms,
                           uint32_t settle_ms = 1000) {
  for (int i = 1; i <= 10; i++) {
    car.set_throttle(i / 10.0);
    launch.run(car, 10);
  }
  launch.run(car, settle_ms - 100);
  launch.peak_slip = 0;
  double sum = 0;
  for (uint32_t i = settle_ms; i < ms; i++) {
    launch.run(car, 1);
    sum += launch.slip();
  }
  return {launch.v, launch.peak_slip, sum / (ms - settle_ms)};
}

} // namespace

TEST_CASE("the controller's state fits the room the VCU gives it") {
  CHECK(vcu_state_size() <= TractionControl::STATE_BYTES);
}

TEST_CASE("traction control is off out of the box: full throttle spins the "
          "rear tires") {
  Car car;
  Launch launch;
  ready(car, launch);
  const auto run = full_throttle(car, launch, 2000);
  CHECK(tc_status().selected == TractionControl::OFF);
  CHECK(tc_status().active == TractionControl::OFF);
  CHECK(run.mean_slip > 0.5);
}

TEST_CASE("traction control holds slip near its 10% target on a full-throttle "
          "launch") {
  // Named per grip: a test each
  for (double mu : {0.7, 1.0, 1.4}) {
    SUBCASE(("grip " + std::to_string(mu).substr(0, 3)).c_str()) {
      Car off_car;
      Launch off;
      off.mu = mu;
      ready(off_car, off);
      const auto without = full_throttle(off_car, off, 3000);

      Car car;
      Launch launch;
      launch.mu = mu;
      car.set_parameter(TRACTION_MODE, TractionControl::SLIP);
      ready(car, launch);
      const auto with = full_throttle(car, launch, 3000);

      CHECK(tc_status().active == TractionControl::SLIP);
      CHECK(with.mean_slip > 0.05);
      CHECK(with.mean_slip < 0.15);
      CHECK(with.peak_slip < 0.2);
      // and that's quicker than spinning the tires
      CHECK(with.speed > without.speed);
    }
  }
}

TEST_CASE("the grip map mode holds slip near its target too") {
  Car car;
  Launch launch;
  car.set_parameter(TRACTION_MODE, TractionControl::GRIP_MAP);
  ready(car, launch);
  const auto run = full_throttle(car, launch, 3000);
  CHECK(tc_status().active == TractionControl::GRIP_MAP);
  CHECK(run.mean_slip > 0.03);
  CHECK(run.mean_slip < 0.15);
  CHECK(run.peak_slip < 0.2);
}

TEST_CASE("traction control never lets through more torque than the driver "
          "asked for (x2000 random inputs)") {
  std::mt19937 rng(1);
  std::uniform_real_distribution<float> u(0, 1);
  TractionControl tc;
  tc.select(TractionControl::SLIP);
  int checked = 0;
  for (uint32_t ms = 0; ms < 2000; ms++) {
    // Sensors saying anything at all, including nonsense
    tc.motor_rpm(u(rng) * 8000 - 1000, ms);
    tc.motor_torque(u(rng) * 300);
    tc.vectornav_velocity(u(rng) * 40 - 5, u(rng) * 10 - 5, ms);
    tc.vectornav_accel(u(rng) * 40 - 20);
    for (int w = 0; w < 4; w++)
      tc.wheel_rpm(w, u(rng) * 2000);
    if (ms % 97 == 0)
      tc.select(uint8_t(u(rng) * 3));
    const double driver = (ms % 300 < 30) ? 0.0 : u(rng) * 240;
    const double out = tc.step(ms, driver);
    CHECK(out >= 0);
    CHECK(out <= driver);
    checked++;
  }
  CHECK(checked == 2000);
}

TEST_CASE("the dash button turns traction control on and off, once the "
          "driver lifts") {
  Car car;
  Launch launch;
  ready(car, launch);
  car.set_throttle(0.5);
  launch.run(car, 200);

  car.dash_tc_button(true);
  launch.run(car, 100);
  car.dash_tc_button(false);
  launch.run(car, 100);
  // Chosen, but not mid-throttle: the torque mustn't step
  CHECK(tc_status().selected == TractionControl::SLIP);
  CHECK(tc_status().active == TractionControl::OFF);
  CHECK(has_reason(TractionControl::TC_WAITING_FOR_LIFT));

  car.set_throttle(0);
  launch.run(car, 100);
  CHECK(tc_status().active == TractionControl::SLIP);
  CHECK(tc_status().reasons == 0);

  // Off again: same rule
  car.set_throttle(0.5);
  launch.run(car, 100);
  car.dash_tc_button(true);
  launch.run(car, 100);
  car.dash_tc_button(false);
  launch.run(car, 100);
  CHECK(tc_status().selected == TractionControl::OFF);
  CHECK(tc_status().active == TractionControl::SLIP);
  car.set_throttle(0);
  launch.run(car, 100);
  CHECK(tc_status().active == TractionControl::OFF);
}

TEST_CASE("holding the dash button toggles once") {
  Car car;
  Launch launch;
  ready(car, launch);
  car.dash_tc_button(true);
  launch.run(car, 1000); // the dash repeats its frame; still one press
  car.dash_tc_button(false);
  launch.run(car, 100);
  CHECK(tc_status().selected == TractionControl::SLIP);
}

TEST_CASE("TRACTION_MODE over CAN picks the mode; a value that isn't one is "
          "ignored") {
  Car car;
  Launch launch;
  ready(car, launch);
  car.set_parameter(TRACTION_MODE, TractionControl::GRIP_MAP);
  launch.run(car, 100);
  CHECK(tc_status().selected == TractionControl::GRIP_MAP);
  car.set_parameter(TRACTION_MODE, 7);
  launch.run(car, 100);
  CHECK(tc_status().selected == TractionControl::GRIP_MAP);
  car.set_parameter(TRACTION_MODE, TractionControl::OFF);
  launch.run(car, 100);
  CHECK(tc_status().selected == TractionControl::OFF);
  // The dash button goes back to the last mode that was on
  car.dash_tc_button(true);
  launch.run(car, 100);
  CHECK(tc_status().selected == TractionControl::GRIP_MAP);
}

TEST_CASE("without VectorNav it steps aside at once, and comes back after a "
          "lift once it's back") {
  Car car;
  Launch launch;
  car.set_parameter(TRACTION_MODE, TractionControl::SLIP);
  ready(car, launch);
  car.set_throttle(1.0);
  launch.run(car, 1000);
  REQUIRE(tc_status().active == TractionControl::SLIP);
  REQUIRE(commanded_torque(car) < 200); // it's cutting

  launch.vectornav_on = false;
  launch.run(car, TC_INPUT_TIMEOUT_MS + 60);
  CHECK(tc_status().active == TractionControl::OFF);
  CHECK(has_reason(TractionControl::TC_NO_VECTORNAV));
  // The driver's torque, like a car without TC (the power limit aside)
  CHECK(commanded_torque(car) ==
        doctest::Approx(car.inverter.get_torque_limit()).epsilon(0.01));

  launch.vectornav_on = true;
  launch.run(car, 200);
  CHECK(tc_status().active == TractionControl::OFF); // still on throttle
  CHECK(has_reason(TractionControl::TC_WAITING_FOR_LIFT));
  car.set_throttle(0);
  launch.run(car, 100);
  CHECK(tc_status().active == TractionControl::SLIP);
}

TEST_CASE("without motor speed it steps aside too") {
  Car car;
  Launch launch;
  car.set_parameter(TRACTION_MODE, TractionControl::SLIP);
  car.drive_to_rtd();
  // No launch.run: the inverter never says how fast the motor turns
  car.set_throttle(1.0);
  car.run_for(500);
  CHECK(tc_status().active == TractionControl::OFF);
  CHECK(has_reason(TractionControl::TC_NO_MOTOR_SPEED));
  CHECK(has_reason(TractionControl::TC_NO_VECTORNAV));
}

TEST_CASE("a final drive that doesn't match the car turns it off until it's "
          "chosen again") {
  Car car;
  Launch launch;
  // The rear wheels turn 30% faster than GEAR_RATIO says they should for
  // the motor's speed: the gear ratio in parameters.hpp is wrong
  launch.rear_wheel_sensor_gain = 1.3;
  car.set_parameter(TRACTION_MODE, TractionControl::SLIP);
  ready(car, launch);
  car.set_throttle(0.6);
  launch.run(car, 3000);
  CHECK(tc_status().active == TractionControl::OFF);
  CHECK(has_reason(TractionControl::TC_GEAR_RATIO));

  // Choosing it again is a fresh check: back on at once, and off again
  // once the new check has seen enough wheel revs (on this car)
  car.set_throttle(0);
  launch.run(car, 50);
  car.set_parameter(TRACTION_MODE, TractionControl::SLIP);
  uint32_t on_ms = 0, off_again_ms = 0;
  for (uint32_t ms = 1; ms <= 2000 && !off_again_ms; ms++) {
    launch.run(car, 1);
    const auto active = car.vcu.traction.active();
    if (!on_ms && active == TractionControl::SLIP)
      on_ms = ms;
    if (on_ms && active == TractionControl::OFF)
      off_again_ms = ms;
  }
  CHECK(on_ms > 0);
  CHECK(on_ms <= 5);
  CHECK(off_again_ms > on_ms);
  CHECK((car.vcu.traction.reasons() & TractionControl::TC_GEAR_RATIO) != 0);
}

TEST_CASE("the right final drive passes the ratio check") {
  Car car;
  Launch launch;
  car.set_parameter(TRACTION_MODE, TractionControl::SLIP);
  ready(car, launch);
  car.set_throttle(0.6);
  launch.run(car, 3000);
  CHECK(launch.v > 10); // far enough for several checks
  CHECK(tc_status().active == TractionControl::SLIP);
  CHECK(tc_status().reasons == 0);
}

TEST_CASE("the status frame goes out on the DAQ bus at 20 Hz") {
  Car car;
  Launch launch;
  car.set_parameter(TRACTION_MODE, TractionControl::SLIP);
  ready(car, launch);
  const size_t before =
      sim::bus(TEENSY_CAN3).sent(CAN_ID_VCU_TRACTION_CONTROL).size();
  launch.run(car, 1000);
  const size_t sent =
      sim::bus(TEENSY_CAN3).sent(CAN_ID_VCU_TRACTION_CONTROL).size() - before;
  CHECK(sent >= 19);
  CHECK(sent <= 21);
  // Its numbers are the car's
  car.set_throttle(1.0);
  launch.run(car, 1500);
  CHECK(tc_status().speed == doctest::Approx(launch.v).epsilon(0.1));
  CHECK(tc_status().scale < 1.0);
}

TEST_CASE("outside ready to drive it passes nothing through") {
  Car car;
  Launch launch;
  car.set_parameter(TRACTION_MODE, TractionControl::SLIP);
  car.run_for(10);
  car.make_ts_healthy();
  car.set_throttle(1.0);
  launch.run(car, 1000);
  CHECK(car.vcu.get_current_state() != READY_TO_DRIVE);
  CHECK(launch.torque == doctest::Approx(0));
}

