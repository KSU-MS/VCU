// Inverter and Accumulator maths: torque/power limiting, current limits and
// the energy/distance integrators.
#include <sim_test_main.hpp>

#include <cmath>
#include <cstring>
#include <limits>
#include <new>
#include <sim_car.hpp>
#include <utility>

using sim::Car;

namespace {

// Command torque straight through the Inverter and decode what it sent
double commanded_torque(Car &car, double request) {
  car.inverter.command_torque(request);
  Car::InverterCommand cmd;
  REQUIRE(car.last_inverter_command(&cmd));
  return cmd.torque_nm;
}

void enable(Car &car) {
  car.inverter.set_power_limit_kw(POWER_LIMIT_KW);
  car.inverter.set_torque_limit(MAX_TORQUE_LIMIT_NM);
  car.inverter.set_inverter_enable(true);
}

} // namespace

//
//// Torque command

TEST_CASE("torque passes through below the limits") {
  Car car;
  enable(car);
  CHECK(commanded_torque(car, 100) == doctest::Approx(100));
}

TEST_CASE("torque is clamped to the torque limit") {
  Car car;
  enable(car);
  car.inverter.set_torque_limit(50);
  CHECK(commanded_torque(car, 100) == doctest::Approx(50));
}

TEST_CASE("torque is zero unless enabled with a sane request") {
  Car car;
  enable(car);

  SUBCASE("inverter disabled") {
    car.inverter.set_inverter_enable(false);
    CHECK(commanded_torque(car, 100) == 0);
  }
  SUBCASE("negative request") { CHECK(commanded_torque(car, -50) == 0); }
  SUBCASE("NaN request") {
    CHECK(commanded_torque(car, std::numeric_limits<double>::quiet_NaN()) == 0);
  }
  SUBCASE("infinite request") {
    CHECK(commanded_torque(car, std::numeric_limits<double>::infinity()) == 0);
  }
  SUBCASE("no power budget") {
    car.inverter.set_power_limit_kw(0);
    CHECK(commanded_torque(car, 100) == 0);
  }
}

TEST_CASE("power limit caps torque by motor speed, either direction") {
  Car car;
  enable(car);

  int16_t rpm = 0;
  SUBCASE("forward") { rpm = 5000; }
  SUBCASE("reverse") { rpm = -5000; }

  car.inverter_motor_rpm(rpm);
  car.run_for(5); // let the VCU read the feedback frame

  const double omega = std::abs(rpm) * 2 * M_PI / 60;
  const double expected = POWER_LIMIT_KW * 1000.0 * 0.90 / omega;
  CHECK(commanded_torque(car, MAX_TORQUE_LIMIT_NM) ==
        doctest::Approx(expected).epsilon(0.01));
}

//
//// Current limit

TEST_CASE("instant current limit is power over voltage") {
  Car car;
  car.inverter.set_power_limit_kw(80);
  CHECK(car.inverter.get_instant_current_limit(400) == 200);
}

TEST_CASE("instant current limit is safe without a pack voltage") {
  // Before the BMS's 0x6B1 arrives pack voltage is 0, and READY_TO_DRIVE
  // only gates on the inverter's bus voltage, so this does get called with 0
  Car car;
  car.inverter.set_power_limit_kw(80);

  SUBCASE("zero") { CHECK(car.inverter.get_instant_current_limit(0) == 0); }
  SUBCASE("negative") { CHECK(car.inverter.get_instant_current_limit(-5) == 0); }
  SUBCASE("NaN") {
    CHECK(car.inverter.get_instant_current_limit(
              std::numeric_limits<float>::quiet_NaN()) == 0);
  }
  SUBCASE("tiny") {
    // 80 kW / 0.5 V doesn't fit in a uint16_t
    CHECK(car.inverter.get_instant_current_limit(0.5) == UINT16_MAX);
  }
}

TEST_CASE("ready to drive without BMS voltage sends a zero discharge limit") {
  Car car;
  car.run_for(10);
  car.acu_status(true, true);
  car.precharge_state(PRECHARGE_OK_STATE);
  car.inverter_bus_voltage(450); // but no pack_power()
  car.run_for(10);
  car.set_brake(0.5);
  car.dash_rtd_button(true);
  REQUIRE(car.run_until(READY_TO_DRIVE, 3000));

  car.run_for(200);
  CHECK(car.last_discharge_limit() == 0);
}

//
//// Initial state
//
// On the Teensy these objects are globals, so they start zeroed whatever
// their constructors do. Anywhere else (heap, stack) they don't, and the
// SIL process caught Inverter starting with a garbage distance. Build them
// on top of junk memory to make sure nothing depends on that.

template <typename T, typename... Args> T *on_junk(void *buf, Args &&...args) {
  // Volatile, or GCC's lifetime DSE drops the fill since the object's
  // lifetime only starts at the placement new
  volatile unsigned char *junk = static_cast<unsigned char *>(buf);
  for (size_t i = 0; i < sizeof(T); i++)
    junk[i] = 0xAB;
  return new (buf) T(std::forward<Args>(args)...);
}

TEST_CASE("a fresh Inverter doesn't depend on zeroed memory") {
  Car car; // for the buses and dbc
  alignas(Inverter) unsigned char buf[sizeof(Inverter)];
  Inverter *inv =
      on_junk<Inverter>(buf, false, &car.inv_can, &car.daq_can, &car.dbc, -0.69f);

  CHECK(inv->get_bus_voltage() == 0);
  CHECK(inv->get_bus_current() == 0);
  CHECK(inv->get_motor_distance_M() == 0);
  CHECK(inv->get_torque_limit() == 0);
  CHECK_FALSE(inv->get_inverter_enable());

  // No feedback yet, so no distance should accumulate either
  inv->calculate_motor_distance_M(0);
  inv->calculate_motor_distance_M(10000);
  CHECK(inv->get_motor_distance_M() == 0);
  inv->~Inverter();
}

TEST_CASE("a fresh Pedals doesn't depend on zeroed memory") {
  alignas(Pedals) unsigned char buf[sizeof(Pedals)];
  Pedals *p = on_junk<Pedals>(buf, MIN_BRAKE_PEDAL, START_BRAKE_PEDAL,
                              END_BRAKE_PEDAL, MAX_BRAKE_PEDAL, MIN_APPS_PEDAL,
                              START_ACCELERATOR_PEDAL_1, END_ACCELERATOR_PEDAL_1,
                              MAX_ACCELERATOR_PEDAL_1, START_ACCELERATOR_PEDAL_2,
                              END_ACCELERATOR_PEDAL_2, MAX_ACCELERATOR_PEDAL_2);

  // Before any reading: no throttle, no faults
  CHECK(p->get_travel() == 0);
  CHECK(p->get_brake_travel() == 0);
  CHECK_FALSE(p->get_apps_fault_ok_low());
  CHECK_FALSE(p->get_apps_bse_fault_ok_low());

  // An open APPS1 on the very first reading mustn't use a junk APPS1 travel
  p->update_travel(MIN_APPS_PEDAL - 1, START_ACCELERATOR_PEDAL_2, START_BRAKE_PEDAL);
  CHECK(p->get_travel() == 0);
  CHECK(p->get_apps1_travel() == 0);
  p->~Pedals();
}

//
//// Integrators

TEST_CASE("energy consumed integrates pack power over time") {
  Car car;
  car.pack_power(400, 100); // 40 kW
  car.run_for(5);

  // 36 s at 40 kW = 400 Wh
  uint32_t t0 = sim::now();
  car.accumulator.calculate_energy_consumed_wh(t0);
  uint32_t before = car.accumulator.get_consumed_wh();
  car.accumulator.calculate_energy_consumed_wh(t0 + 36000);
  CHECK(car.accumulator.get_consumed_wh() - before == doctest::Approx(400).epsilon(0.01));
}

TEST_CASE("distance integrates wheel speed from motor rpm") {
  Car car;
  // rpm that gives exactly 10 m/s at the wheel
  const double rpm = 10.0 / WHEEL_CIRCUMFRANCE_M * GEAR_RATIO * 60;
  car.inverter_motor_rpm(int16_t(std::round(rpm)));
  car.run_for(5);

  uint32_t t0 = sim::now();
  car.inverter.calculate_motor_distance_M(t0);
  uint32_t before = car.inverter.get_motor_distance_M();
  car.inverter.calculate_motor_distance_M(t0 + 100000); // 100 s
  CHECK(car.inverter.get_motor_distance_M() - before ==
        doctest::Approx(1000).epsilon(0.01));
}

TEST_CASE("lifetime distance counts reversing too, and never goes negative") {
  // It's an odometer; a negative double -> uint32_t is UB on the Teensy too
  Car car;
  const double rpm = 10.0 / WHEEL_CIRCUMFRANCE_M * GEAR_RATIO * 60;
  car.inverter_motor_rpm(-int16_t(std::round(rpm)));
  car.run_for(5);

  uint32_t t0 = sim::now();
  car.inverter.calculate_motor_distance_M(t0);
  car.inverter.calculate_motor_distance_M(t0 + 10000); // 10 s backwards
  CHECK(car.inverter.get_motor_distance_M() == doctest::Approx(100).epsilon(0.02));
}
