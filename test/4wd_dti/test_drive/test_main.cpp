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

namespace {

// Each hub's inverter reporting the DC current its torque takes at `rpm`
// (90 % efficient, 450 V), every ms for `ms`; `each_ms` (if any) runs first
// each ms, to move the car around. The total electrical power every ms, and
// each hub's averaged over the last second
struct PowerRun {
  std::vector<double> watts;
  double hub_settled_w[Car::HUBS] = {};
  double settled_w() const {
    double s = 0;
    for (size_t t = watts.size() - 1000; t < watts.size(); t++)
      s += watts[t] / 1000;
    return s;
  }
};

template <typename F>
PowerRun run_on_power(Car &car, const double (&rpm)[Car::HUBS], int ms,
                      F each_ms) {
  const double volts = 450, eff = 0.90;
  for (size_t i = 0; i < Car::HUBS; i++)
    car.hub_rpm(i, rpm[i]);
  PowerRun run;
  for (int t = 0; t < ms; t++) {
    each_ms(t);
    double total_w = 0;
    for (size_t i = 0; i < Car::HUBS; i++) {
      const double w = hub_torque(car, i) * rpm[i] * 2 * M_PI / 60 / eff;
      car.hub_dc_current(i, w / volts);
      total_w += w;
      if (t >= ms - 1000)
        run.hub_settled_w[i] += w / 1000;
    }
    run.watts.push_back(total_w);
    car.run_for(1);
  }
  return run;
}

PowerRun run_on_power(Car &car, const double (&rpm)[Car::HUBS], int ms) {
  return run_on_power(car, rpm, ms, [](int) {});
}

// The DC current limit the VCU last sent hub i's inverter (0x0A), A
double hub_dc_limit(Car &car, size_t i) {
  const uint32_t id = (0x0Au << 8) | vehicle::THIS.drives[i].node;
  const can_message *m = car.inv().last_sent(id);
  REQUIRE(m);
  return int16_t(uint16_t(m->buf.byte[0] << 8 | m->buf.byte[1])) * 0.1;
}

} // namespace

TEST_CASE("[EV.3.3.1][EV.3.4.1] 80 kW across all four, on their measured "
          "total") {
  Car car;
  CHECK(car.drivetrain.power_limit_kw() == POWER_LIMIT_KW);
  car.drive_to_rtd();
  car.set_throttle(1.0);
  const PowerRun run = run_on_power(car, {15000, 15000, 15000, 15000}, 3000);
  sim::no_violation(run.watts); // EV.3.4.1's test of it
  CHECK(run.settled_w() / 1000 > 0.9 * POWER_LIMIT_KW); // and it uses it
}

TEST_CASE("[EV.3.3.1][EV.3.4.1] the power goes where the split puts the "
          "torque: loaded rear hubs past a quarter each, the total held") {
  Car car;
  car.drive_to_rtd();
  car.set_throttle(0.6);
  // Part throttle (at full every motor is at its cap whatever the split),
  // accelerating hard: the load split puts most of the torque on the rear
  const PowerRun run = run_on_power(
      car, {15000, 15000, 15000, 15000}, 3000, [&](int t) {
        if (t % 10 == 0)
          car.vectornav(20, 0, 9.0);
      });
  sim::no_violation(run.watts);
  CHECK(run.settled_w() / 1000 > 0.9 * POWER_LIMIT_KW);
  // A fixed share would hold each hub to 20 kW
  CHECK(run.hub_settled_w[2] / 1000 > 22);
  CHECK(run.hub_settled_w[3] / 1000 > 22);
  CHECK(run.hub_settled_w[0] < run.hub_settled_w[2]);
  // The rear inverters' own DC limits moved with it, and add up to the
  // limit over the pack voltage (no more, whole amps each)
  CHECK(hub_dc_limit(car, 2) > 1.2 * hub_dc_limit(car, 0));
  double sum = 0;
  for (size_t i = 0; i < Car::HUBS; i++)
    sum += hub_dc_limit(car, i);
  CHECK(sum <= POWER_LIMIT_KW * 1000.0 / 450 + 0.5);
  // whole amps each, rounded down
  CHECK(sum > POWER_LIMIT_KW * 1000.0 / 450 - 5);
}

TEST_CASE("[EV.3.4.1] the total held while the split swings back and forth, "
          "the hubs at different speeds") {
  Car car;
  car.drive_to_rtd();
  car.set_throttle(1.0);
  // Accelerating hard, then not, every quarter second; the fronts slower
  const PowerRun run = run_on_power(
      car, {11000, 11000, 16000, 16000}, 4000, [&](int t) {
        if (t % 10 == 0)
          car.vectornav(20, 0, (t / 250) % 2 ? 9.0 : 0.0);
      });
  sim::no_violation(run.watts);
}

TEST_CASE("[EV.3.3.1] one inverter's DC current going quiet: back to the "
          "open-loop cap until it's heard again") {
  Car car;
  car.drive_to_rtd();
  car.set_throttle(1.0);
  // Settle with a drivetrain more efficient than the open loop assumes, so
  // the trim has climbed over 1
  for (size_t i = 0; i < Car::HUBS; i++)
    car.hub_rpm(i, 15000);
  for (int t = 0; t < 2000; t++) {
    for (size_t i = 0; i < Car::HUBS; i++)
      car.hub_dc_current(i, 0); // seeing no power at all
    car.run_for(1);
  }
  CHECK(car.drivetrain.power_trim() > 1.0);
  // fl stops reporting (not long enough to fault the car)
  car.hub_talking[0] = false;
  car.run_for(PowerLoop::STALE_MS + 50);
  car.run_for(PowerLoop::STALE_MS * 3 / 2 + 20);
  CHECK(car.drivetrain.power_trim() <= 1.0);
}

TEST_CASE("[EV.3.3.1] power limit override: parked only, up to the motors' "
          "140 kW, back to 80 kW at power up") {
  {
    Car car;
    car.run_for(10);
    CHECK(car.drivetrain.peak_power_kw() == doctest::Approx(4 * 35.0));
    car.set_parameter(POWER_LIMIT, 141); // over the motors' sum
    car.run_for(10);
    CHECK(car.vcu.power_limit_kw() == POWER_LIMIT_KW);
    car.set_parameter(POWER_LIMIT, 120);
    car.run_for(10);
    CHECK(car.vcu.power_limit_kw() == 120);
    car.run_for(1100); // the status frame (1 Hz) says so
    CHECK(car.last_reported_power_limit() == 120);
    CHECK(sim::log_output().find("OVER THE RULES") != std::string::npos);

    // And it's used: the total settles past 80 kW
    car.drive_to_rtd();
    car.set_throttle(1.0);
    const PowerRun run =
        run_on_power(car, {15000, 15000, 15000, 15000}, 3000);
    CHECK(run.settled_w() / 1000 > 0.9 * 120);
    CHECK(run.settled_w() / 1000 < 120);

    // Not while it could drive
    car.set_parameter(POWER_LIMIT, 0);
    car.run_for(10);
    CHECK(car.vcu.power_limit_kw() == 120);
  }
  Car fresh; // a power cycle
  CHECK(fresh.vcu.power_limit_kw() == POWER_LIMIT_KW);
  fresh.run_for(1100);
  CHECK(fresh.last_reported_power_limit() == POWER_LIMIT_KW);
  fresh.run_for(10);
  fresh.set_parameter(POWER_LIMIT, 100);
  fresh.run_for(10);
  fresh.set_parameter(POWER_LIMIT, 0); // 0: back to the rules' limit
  fresh.run_for(10);
  CHECK(fresh.vcu.power_limit_kw() == POWER_LIMIT_KW);
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
