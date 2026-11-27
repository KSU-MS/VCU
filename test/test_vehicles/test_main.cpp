// The cars the firmware can be built for (vehicle.hpp): the same five
// drivetrains, inverters and node IDs as kmslap and ksu-ms-dbc's
// gen_tool/vehicles.json, and the build picking one.
#include <sim_test_main.hpp>

#include <cstring>
#include <set>
#include <sim_car.hpp>
#include <string>
#include <vehicle.hpp>

using namespace vehicle;

TEST_CASE("the five vehicles, named as in kmslap and ksu-ms-dbc") {
  const char *names[] = {"rwd_emrax", "awd_emrax_dti", "awd_emrax_amk",
                         "4wd_dti", "4wd_amk"};
  REQUIRE(sizeof ALL / sizeof ALL[0] == 5);
  for (size_t i = 0; i < 5; i++)
    CHECK(std::string(ALL[i].name) == names[i]);
}

TEST_CASE("these tests are built for rwd_emrax, the car today") {
  CHECK(std::string(THIS.name) == "rwd_emrax");
  REQUIRE(THIS.drive_count == 1);
  CHECK(THIS.drives[0].position == Position::REAR_AXLE);
  CHECK(THIS.drives[0].inverter == InverterKind::CASCADIA_PM100);
  CHECK(THIS.drives[0].gear_ratio == GEAR_RATIO);
  CHECK(THIS.drives[0].motor == &motors::EMRAX_228_HV);
}

TEST_CASE("the VCU says which vehicle it is at boot") {
  sim::Car car;
  CHECK(sim::log_output().find("Vehicle: rwd_emrax") != std::string::npos);
}

TEST_CASE("every vehicle drives each wheel once, with motors it can use") {
  for (const auto &v : ALL) {
    SUBCASE(v.name) {
      std::set<Position> seen;
      bool rear_axle = false, rear_hubs = false;
      for (size_t i = 0; i < v.drive_count; i++) {
        const Drive &d = v.drives[i];
        CHECK(seen.insert(d.position).second); // no position twice
        CHECK(d.motor->peak_torque_nm > 0);
        CHECK(d.motor->max_rpm > 0);
        CHECK(d.gear_ratio > 1);
        if (d.position == Position::REAR_AXLE)
          rear_axle = true;
        if (d.position == Position::RL || d.position == Position::RR)
          rear_hubs = true;
        // The EMRAX is on the Cascadia, the hubs on DTI or AMK
        CHECK((d.position == Position::REAR_AXLE) ==
              (d.inverter == InverterKind::CASCADIA_PM100));
      }
      CHECK(rear_axle != rear_hubs); // the rear is driven, one way
    }
  }
}

TEST_CASE("hub inverters' node IDs are vehicles.json's") {
  // DTI 1-4 by position; AMK 1, 2, 5, 6 (node n's Actual Values 2 is at
  // n + 2, so 3 and 4 would collide with 1 and 2)
  const uint8_t dti[] = {1, 2, 3, 4}, amk[] = {1, 2, 5, 6};
  for (const auto &v : ALL) {
    SUBCASE(v.name) {
      for (size_t i = 0; i < v.drive_count; i++) {
        const Drive &d = v.drives[i];
        const int pos = int(d.position);
        if (d.inverter == InverterKind::DTI_HV500)
          CHECK(d.node == dti[pos]);
        else if (d.inverter == InverterKind::AMK_KW26)
          CHECK(d.node == amk[pos]);
        else
          CHECK(d.node == 0);
      }
    }
  }
}

TEST_CASE("the 4wd cars have a hub at every wheel, the awd ones at the front") {
  for (const auto &v : ALL) {
    const bool four = std::strncmp(v.name, "4wd", 3) == 0;
    const bool awd = std::strncmp(v.name, "awd", 3) == 0;
    if (four)
      CHECK(v.drive_count == 4);
    if (awd)
      CHECK(v.drive_count == 3);
  }
}

TEST_CASE("a hub motor's torque curve is kmslap's: peak torque to peak power, "
          "none at top speed") {
  const MotorSpec &m = motors::DTI_FMOT;
  CHECK(motors::max_torque_nm(m, 0) == m.peak_torque_nm);
  CHECK(motors::max_torque_nm(m, 5000) == m.peak_torque_nm);
  // 35 kW at 15000 rpm: 22.3 N m, either way round
  const double omega = 15000 * 2 * 3.141592653589793 / 60;
  CHECK(motors::max_torque_nm(m, 15000) == doctest::Approx(35000 / omega));
  CHECK(motors::max_torque_nm(m, -15000) == doctest::Approx(35000 / omega));
  CHECK(motors::max_torque_nm(m, 20000) == 0);
  CHECK(motors::max_torque_nm(m, 25000) == 0);
  // The EMRAX has no curve here: the Cascadia's path is unchanged
  CHECK(motors::max_torque_nm(motors::EMRAX_228_HV, 4000) == 230.0);
}
