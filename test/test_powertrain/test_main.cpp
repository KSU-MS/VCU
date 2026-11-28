// Inverter and Accumulator maths: torque/power limiting, current limits and
// the energy/distance integrators.
#include <sim_test_main.hpp>

#include <algorithm>
#include <string>
#include <cmath>
#include <cstring>
#include <limits>
#include <new>
#include <sim_car.hpp>
#include <sim_power.hpp>
#include <utility>

using sim::Car;
using sim::no_violation;

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

TEST_CASE("[EV.3.3.1] power limit caps torque by motor speed, either direction") {
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

TEST_CASE("[EV.3.3.1] without measured power, shaft power stops at 80 kW x 0.90") {
  // The open-loop cap, which is all there is until the inverter reports its
  // DC current (regulate_power): it needs the drivetrain to lose <= 10%
  Car car;
  enable(car);
  for (int16_t rpm : {500, 1000, 2000, 3500, 5000, 7500, 10000, -5000}) {
    CAPTURE(rpm);
    car.inverter_motor_rpm(rpm);
    car.run_for(5);
    const double omega = std::abs(rpm) * 2 * M_PI / 60;
    const double shaft_kw = commanded_torque(car, MAX_TORQUE_LIMIT_NM) * omega / 1000;
    CHECK(shaft_kw <= POWER_LIMIT_KW * 0.90 + 0.01);
  }
  CHECK(POWER_LIMIT_KW <= 80);
}

//
//// Closed loop on measured power (EV.3.3.1, EV.3.4.1): a drivetrain model
//// the VCU commands, reporting its DC current like the inverter does

namespace {
struct PowerTrace {
  std::vector<double> watts; // electrical, every ms
  double settled_kw;         // mean over the last second
};

// The car flat out on a long straight: the motor at `rpm`, torque following
// the command with a 20 ms lag, electrical power = shaft / efficiency,
// reported (DC bus V and I) every 10 ms
PowerTrace flat_out(Car &car, double efficiency, int16_t rpm, uint32_t ms,
                    bool report_current = true) {
  const double volts = 450, omega = rpm * 2 * M_PI / 60;
  double torque = 0;
  PowerTrace trace{};
  car.set_throttle(1.0);
  for (uint32_t t = 0; t < ms; t++) {
    car.run_for(1);
    Car::InverterCommand cmd;
    if (car.last_inverter_command(&cmd))
      torque += (cmd.torque_nm - torque) / 20;
    const double watts = torque * omega / efficiency;
    trace.watts.push_back(watts);
    if (t % 10 == 0) {
      car.inverter_motor_rpm(rpm);
      car.inverter_bus_voltage(volts);
      if (report_current)
        car.inverter_bus_current(watts / volts);
    }
  }
  double sum = 0;
  for (uint32_t t = ms - 1000; t < ms; t++)
    sum += trace.watts[t];
  trace.settled_kw = sum / 1000 / 1000;
  return trace;
}

} // namespace

TEST_CASE("[EV.3.3.1][EV.3.4.1] measured power stays under 80 kW, whatever the drivetrain loses") {
  // One SUBCASE each, named, so each shows up on its own in the report
  for (double efficiency : {0.70, 0.80, 0.85, 0.90, 0.95, 0.99}) {
   SUBCASE(("drivetrain " + std::to_string(int(efficiency * 100)) + "% efficient").c_str()) {
    Car car;
    car.drive_to_rtd();
    PowerTrace p = flat_out(car, efficiency, 6000, 5000);
    no_violation(p.watts);
    // and uses the budget: within a few % of the target, not 72 kW
    CHECK(p.settled_kw >= POWER_LIMIT_KW * POWER_LIMIT_TARGET * 0.97);
    CHECK(p.settled_kw <= POWER_LIMIT_KW);
   }
  }
}

TEST_CASE("[EV.3.3.1][EV.3.4.1] coming onto the power limit from below (x15: 1000-8000 rpm)") {
  // Accelerating through the speed where 240 Nm reaches 80 kW: torque
  // limited, then power limited, with the trim having crept up meanwhile
  Car car;
  car.drive_to_rtd();
  std::vector<double> all;
  for (int16_t rpm = 1000; rpm <= 8000; rpm += 500) {
    PowerTrace p = flat_out(car, 0.85, rpm, 300);
    all.insert(all.end(), p.watts.begin(), p.watts.end());
  }
  no_violation(all);
}

TEST_CASE("[EV.3.3.1] measured power going stale falls back to the open-loop cap") {
  Car car;
  car.drive_to_rtd();
  flat_out(car, 0.99, 6000, 3000); // a lossless-looking drivetrain: trim > 1
  REQUIRE(car.inverter.get_power_trim() > 1.0);
  PowerTrace p = flat_out(car, 0.99, 6000, 2000, /*report_current=*/false);
  CHECK(car.inverter.get_power_trim() <= 1.0);
  CHECK(p.settled_kw * 0.99 <= POWER_LIMIT_KW * 0.90 + 0.1); // shaft power
}

TEST_CASE("[EV.3.3.1] power limit override over CAN: parked only, up to the "
          "EMRAX's 100 kW, 80 kW again at power up") {
  {
    Car car;
    car.run_for(10);
    CHECK(car.drivetrain.peak_power_kw() == 100);
    car.set_parameter(POWER_LIMIT, 101);
    car.run_for(10);
    CHECK(car.vcu.power_limit_kw() == POWER_LIMIT_KW);
    car.set_parameter(POWER_LIMIT, 95);
    car.run_for(10);
    CHECK(car.vcu.power_limit_kw() == 95);
    CHECK(car.inverter.get_power_limit_kw() == 95);
    car.run_for(1100); // the status frame (1 Hz) says so
    CHECK(car.last_reported_power_limit() == 95);

    // The loop holds the new limit instead
    car.drive_to_rtd();
    PowerTrace p = flat_out(car, 0.90, 6000, 5000);
    CHECK(p.settled_kw > 95 * POWER_LIMIT_TARGET * 0.97);
    CHECK(p.settled_kw <= 95);

    // Not while it could drive
    car.set_parameter(POWER_LIMIT, 0);
    car.run_for(10);
    CHECK(car.vcu.power_limit_kw() == 95);
  }
  Car fresh; // a power cycle
  CHECK(fresh.vcu.power_limit_kw() == POWER_LIMIT_KW);
  fresh.run_for(1100);
  CHECK(fresh.last_reported_power_limit() == POWER_LIMIT_KW);
}

TEST_CASE("[EV.3.3.3] never asks for regen, at any speed (x28: 7 speeds x 4 requests)") {
  // No regen at all, so none between 0 and 5 km/h either
  Car car;
  enable(car);
  for (int16_t rpm : {0, 10, 50, 100, 300, 1000, 5000}) {
    car.inverter_motor_rpm(rpm);
    car.run_for(5);
    for (double request : {-240.0, -1.0, -1e-9, 0.0}) {
      CAPTURE(rpm);
      CAPTURE(request);
      CHECK(commanded_torque(car, request) == 0);
    }
  }
}

//
//// Current limit

TEST_CASE("[EV.3.3.3] rwd_emrax has no regen: the brake at speed asks for "
          "nothing, the throttle still works off it") {
  Car car;
  car.bms_charge_limit(200, 600);
  car.drive_to_rtd();
  REQUIRE(vehicle::THIS.regen_brake_travel == 0);
  CHECK(car.pedals.regen_fraction() == 0);
  car.inverter_motor_rpm(3000);
  for (double brake : {0.2, 0.5, 0.9}) {
    car.set_brake(brake);
    for (int t = 0; t < 10; t++) {
      car.vectornav(20, 0, -8);
      car.run_for(10);
    }
    Car::InverterCommand cmd;
    REQUIRE(car.last_inverter_command(&cmd));
    CHECK(cmd.torque_nm >= 0);
  }
}

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
  alignas(CascadiaPM100) unsigned char buf[sizeof(CascadiaPM100)];
  Inverter *inv = on_junk<CascadiaPM100>(buf, false, &car.inv_can,
                                         &car.daq_can, &car.dbc, -0.69f);

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
  p->update_travel(MIN_APPS_PEDAL - 1, START_ACCELERATOR_PEDAL_2, START_BRAKE_PEDAL, 0);
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
