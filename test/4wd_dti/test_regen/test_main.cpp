// Regen on the 4wd_dti car (docs/regen.md): the brake pedal's regen travel,
// the speed rule, the split under braking, the pack's limits and each
// wheel's slip.
#include <sim_test_main.hpp>

#include <cmath>
#include <sim_car.hpp>
#include <vehicle.hpp>

using sim::Car;

namespace {

const double REGEN_TRAVEL = vehicle::THIS.regen_brake_travel;

// Motor rpm with the wheel rolling at `ms`
double hub_rpm_at(double ms) {
  return ms * 60 / WHEEL_CIRCUMFRANCE_M * vehicle::HUB_RATIO;
}

// The car rolling at `ms` (and decelerating at `ax`, m/s^2), every hub's
// wheel too, for `ms_run`: VectorNav every 10 ms, like the real one
void roll(Car &car, double ms, double ax, uint32_t ms_run) {
  for (size_t i = 0; i < Car::HUBS; i++)
    car.hub_rpm(i, hub_rpm_at(ms));
  for (uint32_t t = 0; t < ms_run; t += 10) {
    car.vectornav(ms, 0, ax);
    car.run_for(10);
  }
}

// Ready to drive with the BMS saying how much charge it'll take
void ready(Car &car, double charge_a = 200) {
  car.bms_charge_limit(charge_a, 600);
  car.drive_to_rtd();
}

double total_motor_nm(Car &car) {
  double t = 0;
  for (size_t i = 0; i < Car::HUBS; i++)
    t += car.last_hub_torque(i);
  return t;
}

// The most each hub can take back at this speed, N m at the motor
double curve_at(double ms) {
  return motors::max_torque_nm(motors::DTI_FMOT, hub_rpm_at(ms));
}

// The brake travel that asks for `fraction` of the most regen
double brake_for(double fraction) {
  return REGEN_BRAKE_DEADBAND + fraction * (REGEN_TRAVEL - REGEN_BRAKE_DEADBAND);
}

} // namespace

TEST_CASE("[T.3.2.4] the brake's regen travel: regen rising to the most over "
          "it, and staying at the most once the hydraulics take over") {
  Car car;
  ready(car, 400); // enough for all of it: ~126 kW into the pack at 20 m/s
  REQUIRE(REGEN_TRAVEL == doctest::Approx(0.9));
  roll(car, 20, 0, 100);
  CHECK(total_motor_nm(car) == doctest::Approx(0).epsilon(0.001));

  car.set_brake(brake_for(0.5));
  roll(car, 20, 0, 100);
  CHECK(total_motor_nm(car) ==
        doctest::Approx(-0.5 * 4 * curve_at(20)).epsilon(0.02));
  for (size_t i = 0; i < Car::HUBS; i++)
    CHECK(car.last_hub_torque(i) < 0);

  car.set_hydraulic_brake(0.5); // past the regen travel
  roll(car, 20, 0, 100);
  CHECK(total_motor_nm(car) ==
        doctest::Approx(-4 * curve_at(20)).epsilon(0.02));
  for (size_t i = 0; i < Car::HUBS; i++)
    CHECK(car.last_hub_torque(i) >= -motors::DTI_FMOT.peak_torque_nm - 0.05);
}

TEST_CASE("the brake overrides the throttle: pressed at all, no drive torque") {
  Car car;
  ready(car);
  car.set_throttle(0.5);
  roll(car, 15, 0, 100);
  REQUIRE(total_motor_nm(car) > 0);
  car.set_brake(brake_for(0.2));
  roll(car, 15, 0, 100);
  for (size_t i = 0; i < Car::HUBS; i++)
    CHECK(car.last_hub_torque(i) < 0);
  // Off the brake: the throttle's back at once, the regen travel isn't the
  // mechanical brakes (EV.4.7.1 doesn't latch)
  car.set_brake(0);
  roll(car, 15, 0, 100);
  CHECK(total_motor_nm(car) > 0);
}

TEST_CASE("[EV.4.7.1] the hydraulics and the throttle together still cut "
          "power until the throttle's released") {
  Car car;
  ready(car);
  car.set_throttle(0.5);
  car.set_hydraulic_brake(0.5);
  roll(car, 15, 0, 100);
  CHECK(car.pedals.get_apps_bse_fault_ok_low());
  car.set_brake(0);
  roll(car, 15, 0, 100);
  CHECK(total_motor_nm(car) == 0); // latched until the throttle's under 5 %
  car.set_throttle(0);
  roll(car, 15, 0, 50);
  car.set_throttle(0.5);
  roll(car, 15, 0, 100);
  CHECK(total_motor_nm(car) > 0);
}

TEST_CASE("[EV.10.6.2] ready to drive needs the hydraulics, not just the regen "
          "travel") {
  Car car;
  car.run_for(10);
  car.make_ts_healthy();
  car.run_for(10);
  car.set_brake(REGEN_TRAVEL); // all of the regen travel, no hydraulics
  car.dash_rtd_button(true);
  car.run_for(3000);
  CHECK(car.vcu.get_current_state() != READY_TO_DRIVE);
  car.set_hydraulic_brake(0.5);
  CHECK(car.run_until(READY_TO_DRIVE, 3000));
}

TEST_CASE("[EV.3.3.3] no regen at 5 km/h or less, fading in to 8 km/h") {
  Car car;
  ready(car);
  car.set_brake(brake_for(1.0));
  roll(car, 4.9 / 3.6, 0, 100);
  CHECK(total_motor_nm(car) == 0);
  roll(car, 5.0 / 3.6, 0, 100);
  CHECK(total_motor_nm(car) == 0);
  // Half way through the fade: half of it
  roll(car, 6.5 / 3.6, 0, 100);
  CHECK(total_motor_nm(car) ==
        doctest::Approx(-0.5 * 4 * curve_at(6.5 / 3.6)).epsilon(0.02));
  roll(car, 8.0 / 3.6, 0, 100);
  CHECK(total_motor_nm(car) ==
        doctest::Approx(-4 * curve_at(8.0 / 3.6)).epsilon(0.02));
}

TEST_CASE("[EV.3.3.3] no speed, no regen: VectorNav quiet") {
  Car car;
  ready(car);
  car.set_brake(brake_for(1.0));
  for (size_t i = 0; i < Car::HUBS; i++)
    car.hub_rpm(i, hub_rpm_at(20));
  car.run_for(500); // the hubs turning, but no VectorNav
  CHECK(total_motor_nm(car) == 0);
}

TEST_CASE("braking moves the load forward: the front hubs take more") {
  Car car;
  ready(car);
  car.set_brake(brake_for(0.4));
  roll(car, 20, -12.0, 200);
  const double front = car.last_hub_torque(0) + car.last_hub_torque(1);
  const double rear = car.last_hub_torque(2) + car.last_hub_torque(3);
  CHECK(front < 1.3 * rear); // both negative: the front's bigger
  CHECK(car.last_hub_torque(0) == doctest::Approx(car.last_hub_torque(1)));
}

TEST_CASE("held to the BMS's charge limit, and none without it") {
  // 50 A at 450 V: 22.5 kW into the pack at most
  Car car;
  ready(car, 50);
  car.set_hydraulic_brake(0.5);
  roll(car, 20, 0, 200);
  const double omega = hub_rpm_at(20) * 2 * M_PI / 60;
  const double into_pack_w = -total_motor_nm(car) * omega * REGEN_EFFICIENCY;
  CHECK(into_pack_w <= 50 * 450.0 + 50);
  CHECK(into_pack_w > 0.95 * 50 * 450.0);

  // The BMS's charge limit not heard for a second: none
  Car quiet;
  quiet.drive_to_rtd(); // never sent
  quiet.set_hydraulic_brake(0.5);
  roll(quiet, 20, 0, 200);
  CHECK(total_motor_nm(quiet) == 0);
}

TEST_CASE("each inverter's DC brake current limit is its share of the BMS's, "
          "sent negative") {
  Car car;
  ready(car, 120);
  car.set_brake(brake_for(0.5));
  roll(car, 20, 0, 300);
  double sum = 0;
  for (size_t i = 0; i < Car::HUBS; i++) {
    const uint32_t id = (0x0Bu << 8) | vehicle::THIS.drives[i].node;
    const can_message *m = car.inv().last_sent(id);
    REQUIRE(m);
    const double amps =
        int16_t(uint16_t(m->buf.byte[0] << 8 | m->buf.byte[1])) * 0.1;
    CHECK(amps <= 0);
    sum += -amps;
  }
  CHECK(sum <= 120);
  CHECK(sum > 120 - 4); // whole amps each, rounded down
}

TEST_CASE("[EV.3.3.2] regen fades out before the DC bus gets to 600 V") {
  Car car;
  ready(car);
  car.set_hydraulic_brake(0.5);
  roll(car, 20, 0, 100);
  const double full = total_motor_nm(car);
  REQUIRE(full < 0);
  for (size_t i = 0; i < Car::HUBS; i++)
    car.hub_bus_voltage(i, (REGEN_BUS_V_FADE + REGEN_BUS_V_MAX) / 2);
  roll(car, 20, 0, 100);
  CHECK(total_motor_nm(car) == doctest::Approx(full / 2).epsilon(0.02));
  car.hub_bus_voltage(2, REGEN_BUS_V_MAX); // any one of them
  roll(car, 20, 0, 100);
  CHECK(total_motor_nm(car) == 0);
}

TEST_CASE("the BMS's maximum pack voltage, if lower, is where it stops") {
  Car car;
  car.bms_charge_limit(200, 560);
  car.drive_to_rtd();
  car.set_hydraulic_brake(0.5);
  for (size_t i = 0; i < Car::HUBS; i++)
    car.hub_bus_voltage(i, 560);
  roll(car, 20, 0, 100);
  CHECK(total_motor_nm(car) == 0);
}

TEST_CASE("a wheel locking up: its regen backs off, the others' doesn't") {
  Car car;
  ready(car);
  car.set_brake(brake_for(0.5));
  roll(car, 20, 0, 100);
  const double before = car.last_hub_torque(1);
  // fl turning 20 % slower than the car: past the 8 % target + 6 % band
  car.hub_rpm(0, hub_rpm_at(20 * 0.8));
  for (int t = 0; t < 10; t++) {
    car.vectornav(20, 0, 0);
    car.run_for(10);
  }
  CHECK(std::fabs(car.last_hub_torque(0)) < 0.5);
  CHECK(car.last_hub_torque(1) == doctest::Approx(before).epsilon(0.02));
  // Rolling with the car again: back
  car.hub_rpm(0, hub_rpm_at(20));
  roll(car, 20, 0, 100);
  CHECK(car.last_hub_torque(0) == doctest::Approx(before).epsilon(0.02));
}

TEST_CASE("[EV.10.6.1] no regen outside ready to drive") {
  Car car;
  car.bms_charge_limit(200, 600);
  car.run_for(10);
  car.make_ts_healthy();
  car.set_brake(brake_for(1.0));
  roll(car, 20, 0, 500);
  CHECK(car.vcu.get_current_state() != READY_TO_DRIVE);
  for (size_t i = 0; i < Car::HUBS; i++) {
    Car::HubCommand c{};
    REQUIRE(car.last_hub_command(i, &c));
    CHECK(c.ac_current_a == 0);
  }
}
