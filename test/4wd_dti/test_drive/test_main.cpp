// The 4wd_dti car in the sim, end to end: four DTI HV500s on the inverter
// bus, the VCU taking it to ready to drive, the driver's wheel torque split
// over the hubs by axle load, 80 kW across all of them, and any one
// inverter going quiet faulting the car (docs/vehicles.md).
#include <sim_test_main.hpp>

#include <algorithm>
#include <car_model.hpp>
#include <cmath>
#include <sim_car.hpp>
#include <sim_power.hpp>
#include <vehicle.hpp>

using sim::Car;

namespace {

const double HUB_PEAK = motors::DTI_FMOT.peak_torque_nm;
const double WHEEL_CAP = HUB_PEAK * vehicle::HUB_RATIO * vehicle::HUB_GEAR_EFF;

double hub_torque(Car &car, size_t i) { return car.last_hub_torque(i); }

double total_wheel_torque(Car &car) {
  double t = 0;
  for (size_t i = 0; i < Car::HUBS; i++)
    t += hub_torque(car, i) * vehicle::HUB_RATIO * vehicle::HUB_GEAR_EFF;
  return t;
}

} // namespace

TEST_CASE("[EV.10.6.2] ready to drive with all four inverters up") {
  Car car;
  car.drive_to_rtd();
  CHECK(car.vcu.get_current_state() == READY_TO_DRIVE);
  for (size_t i = 0; i < Car::HUBS; i++) {
    Car::HubCommand c{};
    REQUIRE(car.last_hub_command(i, &c));
    CHECK(c.enable);
  }
}

TEST_CASE("before ready to drive every hub sits disabled at no current") {
  Car car;
  car.run_for(10);
  car.make_ts_healthy();
  car.set_throttle(1.0);
  car.run_for(500);
  CHECK(car.vcu.get_current_state() != READY_TO_DRIVE);
  for (size_t i = 0; i < Car::HUBS; i++) {
    Car::HubCommand c{};
    REQUIRE(car.last_hub_command(i, &c));
    CHECK_FALSE(c.enable);
    CHECK(c.ac_current_a == 0);
  }
}

TEST_CASE("[EV.10.6.1] the throttle's wheel torque, split by axle load "
          "standing still: the static weight, left and right alike") {
  Car car;
  car.drive_to_rtd();
  car.set_throttle(0.5);
  car.run_for(50);
  // Half of what all four can do at the wheels
  CHECK(total_wheel_torque(car) ==
        doctest::Approx(0.5 * 4 * WHEEL_CAP).epsilon(0.01));
  const double front = hub_torque(car, 0) + hub_torque(car, 1);
  const double all = front + hub_torque(car, 2) + hub_torque(car, 3);
  CHECK(front / all ==
        doctest::Approx(car_model::FRONT_WEIGHT).epsilon(0.01));
  CHECK(hub_torque(car, 0) == doctest::Approx(hub_torque(car, 1)));
  CHECK(hub_torque(car, 2) == doctest::Approx(hub_torque(car, 3)));
}

TEST_CASE("accelerating hard, the rear hubs get more") {
  Car car;
  car.drive_to_rtd();
  car.set_throttle(0.5);
  for (int i = 0; i < 10; i++) {
    car.vectornav(10, 0, 8.0); // 10 m/s, 8 m/s^2 forward
    car.run_for(10);
  }
  CHECK(hub_torque(car, 2) > 1.2 * hub_torque(car, 0));
  // still all of the ask, just moved back
  CHECK(total_wheel_torque(car) ==
        doctest::Approx(0.5 * 4 * WHEEL_CAP).epsilon(0.01));
}

TEST_CASE("full throttle: every hub at its peak, no more") {
  Car car;
  car.drive_to_rtd();
  car.set_throttle(1.0);
  car.run_for(50);
  for (size_t i = 0; i < Car::HUBS; i++)
    CHECK(hub_torque(car, i) <= HUB_PEAK + 0.05);
  CHECK(total_wheel_torque(car) ==
        doctest::Approx(4 * WHEEL_CAP).epsilon(0.01));
}

TEST_CASE("at speed, full throttle is the torque curve: the pedal scales "
          "what the motors can give there, and nothing at top speed") {
  Car car;
  car.drive_to_rtd();
  // 15000 rpm: 35 kW / omega each, under the torque limit
  const double curve = motors::max_torque_nm(motors::DTI_FMOT, 15000);
  REQUIRE(curve < HUB_PEAK);
  for (size_t i = 0; i < Car::HUBS; i++)
    car.hub_rpm(i, 15000);
  car.set_throttle(0.5);
  car.run_for(50);
  const double wheel_curve = curve * vehicle::HUB_RATIO * vehicle::HUB_GEAR_EFF;
  CHECK(total_wheel_torque(car) ==
        doctest::Approx(0.5 * 4 * wheel_curve).epsilon(0.01));

  for (size_t i = 0; i < Car::HUBS; i++)
    car.hub_rpm(i, motors::DTI_FMOT.max_rpm);
  car.set_throttle(1.0);
  car.run_for(50);
  for (size_t i = 0; i < Car::HUBS; i++)
    CHECK(hub_torque(car, i) == 0);
}

TEST_CASE("one hub at its top speed: the others make up its share") {
  Car car;
  car.drive_to_rtd();
  car.hub_rpm(0, motors::DTI_FMOT.max_rpm); // fl spinning up
  car.set_throttle(0.3);
  car.run_for(50);
  CHECK(hub_torque(car, 0) == 0);
  CHECK(total_wheel_torque(car) ==
        doctest::Approx(0.3 * 3 * WHEEL_CAP).epsilon(0.01));
}

TEST_CASE("[EV.3.3.1][EV.3.4.1] 80 kW shared four ways, each hub on its own "
          "measured power") {
  Car car;
  CHECK(car.drivetrain.inverter(0).get_power_limit_kw() ==
        doctest::Approx(20));
  car.drive_to_rtd();
  // 15000 rpm at the motor, 90 % efficient: each inverter's DC current is
  // what its torque takes, which is what its power limit regulates on
  const double omega = 15000 * 2 * M_PI / 60, volts = 450, eff = 0.90;
  for (size_t i = 0; i < Car::HUBS; i++)
    car.hub_rpm(i, 15000);
  car.set_throttle(1.0);
  std::vector<double> watts;
  for (int ms = 0; ms < 3000; ms++) {
    double total_w = 0;
    for (size_t i = 0; i < Car::HUBS; i++) {
      const double w = hub_torque(car, i) * omega / eff;
      car.hub_dc_current(i, w / volts);
      total_w += w;
    }
    watts.push_back(total_w);
    car.run_for(1);
  }
  sim::no_violation(watts); // EV.3.4.1's test of it
  double settled = 0;
  for (size_t t = watts.size() - 1000; t < watts.size(); t++)
    settled += watts[t] / 1000;
  CHECK(settled / 1000 > 0.9 * POWER_LIMIT_KW); // and it uses the budget
}

TEST_CASE("one hub's inverter going quiet faults the car: no torque anywhere") {
  Car car;
  car.drive_to_rtd();
  car.set_throttle(0.5);
  car.run_for(100);
  car.hub_talking[2] = false; // rl
  car.run_for(SAFETY_DATA_TIMEOUT_MS + 200);
  CHECK(car.vcu.get_current_state() == TRACTIVE_SYSTEM_FAULT);
  CHECK((car.vcu.fault_code() & FAULT_INVERTER_SILENT) != 0);
  for (size_t i = 0; i < Car::HUBS; i++) {
    Car::HubCommand c{};
    REQUIRE(car.last_hub_command(i, &c));
    CHECK_FALSE(c.enable);
    CHECK(c.ac_current_a == 0);
  }
}

TEST_CASE("every inverter has to stay precharged") {
  Car car;
  car.run_for(10);
  car.make_ts_healthy();
  car.run_for(200);
  REQUIRE(car.vcu.get_current_state() == TRACTIVE_SYSTEM_ENERGIZED);
  car.hub_bus_voltage(3, TRACTIVE_SYSTEM_MINIMUM_VOLTAGE - 50); // rr drops
  car.run_for(300);
  CHECK(car.vcu.get_current_state() != TRACTIVE_SYSTEM_ENERGIZED);
  CHECK(car.drivetrain.min_bus_voltage() < TRACTIVE_SYSTEM_MINIMUM_VOLTAGE);
}

TEST_CASE("[T.4.2.5] an APPS implausibility cuts all four") {
  Car car;
  car.drive_to_rtd();
  car.set_throttle(0.5);
  car.run_for(50);
  car.apps2_raw = START_ACCELERATOR_PEDAL_2; // apps1 at 50 %, apps2 at 0
  car.run_for(200);
  for (size_t i = 0; i < Car::HUBS; i++)
    CHECK(hub_torque(car, i) == doctest::Approx(0));
}
