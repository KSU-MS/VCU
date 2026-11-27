// The whole VCU on the simulated car: state transitions, what it commands the
// inverter to do, and how it reacts when the rest of the car goes bad.
#include <sim_test_main.hpp>

#include <cmath>
#include <sim_car.hpp>

using sim::Car;

namespace {

size_t count_sent(sim::Bus &b, uint32_t id) { return b.sent(id).size(); }

} // namespace

//
//// Startup and gating

TEST_CASE("boots into TRACTIVE_SYSTEM_DISABLED with the pumps off") {
  Car car;
  car.run_for(10);
  CHECK(car.vcu.get_current_state() == TRACTIVE_SYSTEM_DISABLED);
  CHECK(sim::pin_is_output(LOWSIDE1));
  CHECK(sim::pin_is_output(LOWSIDE2));
  CHECK_FALSE(sim::pin(LOWSIDE1));
  CHECK_FALSE(sim::pin(LOWSIDE2));
  CHECK_FALSE(car.inverter.get_inverter_enable());
}

TEST_CASE("brings all three CAN buses up at the configured baud rates") {
  Car car;
  CHECK(car.acc().baud == ACCUMULATOR_CAN_BAUD_RATE);
  CHECK(car.inv().baud == INVERTER_CAN_BAUD_RATE);
  CHECK(car.daq().baud == DAQ_CAN_BAUD_RATE);
}

TEST_CASE("energizes once ACU, precharge and bus voltage are all good") {
  Car car;
  car.run_for(10);
  car.make_ts_healthy();
  car.run_for(10);
  CHECK(car.vcu.get_current_state() == TRACTIVE_SYSTEM_ENERGIZED);
  CHECK(sim::pin(LOWSIDE1));
  CHECK(sim::pin(LOWSIDE2));
}

TEST_CASE("stays disabled while any TS gate is missing") {
  Car car;
  car.run_for(10);

  bool imd = true, bms = true;
  uint8_t precharge = PRECHARGE_OK_STATE;
  double bus_v = 450;

  SUBCASE("IMD not ok") { imd = false; }
  SUBCASE("BMS not ok") { bms = false; }
  SUBCASE("precharge not done") { precharge = PRECHARGE_OK_STATE - 1; }
  SUBCASE("bus voltage too low") { bus_v = TRACTIVE_SYSTEM_MINIMUM_VOLTAGE - 10; }

  car.acu_status(imd, bms);
  car.precharge_state(precharge);
  car.pack_power(450, 0);
  car.inverter_bus_voltage(bus_v);

  car.run_for(100);
  CHECK(car.vcu.get_current_state() == TRACTIVE_SYSTEM_DISABLED);
  CHECK_FALSE(car.ever_reached(TRACTIVE_SYSTEM_ENERGIZED));
}

TEST_CASE("needs both the RTD button and the brake to go ready to drive") {
  Car car;
  car.run_for(10);
  car.make_ts_healthy();
  car.run_for(10);
  REQUIRE(car.vcu.get_current_state() == TRACTIVE_SYSTEM_ENERGIZED);

  SUBCASE("RTD without brake") {
    car.dash_rtd_button(true);
    car.run_for(3000);
    CHECK(car.vcu.get_current_state() == TRACTIVE_SYSTEM_ENERGIZED);
  }
  SUBCASE("brake without RTD") {
    car.set_brake(0.8);
    car.run_for(3000);
    CHECK(car.vcu.get_current_state() == TRACTIVE_SYSTEM_ENERGIZED);
  }
  SUBCASE("brake below the RTD threshold") {
    car.set_brake(MINIMUM_BRAKE_FOR_RTD - 0.1);
    car.dash_rtd_button(true);
    car.run_for(3000);
    CHECK(car.vcu.get_current_state() == TRACTIVE_SYSTEM_ENERGIZED);
  }
  SUBCASE("both") {
    car.set_brake(0.8);
    car.dash_rtd_button(true);
    CHECK(car.run_until(READY_TO_DRIVE, 3000));
  }
}

TEST_CASE("sounds the buzzer for ~2 s before ready to drive") {
  Car car;
  car.run_for(10);
  car.make_ts_healthy();
  car.run_for(10);
  car.set_brake(0.8);
  car.dash_rtd_button(true);

  REQUIRE(car.run_until(TRACTIVE_SYSTEM_ENABLED, 100));
  uint32_t buzzer_start = sim::now();
  car.tick();
  CHECK(sim::pin(BUZZER));

  REQUIRE(car.run_until(READY_TO_DRIVE, 5000));
  CHECK(sim::now() - buzzer_start >= 2000);
  CHECK(sim::now() - buzzer_start <= 2500);
  CHECK_FALSE(sim::pin(BUZZER));
}

TEST_CASE("keeps the loop running during the buzzer") {
  // The inverter expects the 0x0C0 command stream to keep coming, and CAN
  // has to keep being read so a fault during the buzzer is caught
  Car car;
  car.run_for(10);
  car.make_ts_healthy();
  car.run_for(10);
  car.set_brake(0.8);
  car.dash_rtd_button(true);
  REQUIRE(car.run_until(TRACTIVE_SYSTEM_ENABLED, 100));

  size_t before = count_sent(car.inv(), CAN_ID_M192_COMMAND_MESSAGE);
  REQUIRE(car.run_until(READY_TO_DRIVE, 5000));
  size_t pings = count_sent(car.inv(), CAN_ID_M192_COMMAND_MESSAGE) - before;
  CHECK(pings >= 15); // ~21 at 10 Hz over 2.15 s
}

TEST_CASE("a fault during the buzzer never reaches ready to drive") {
  Car car;
  car.run_for(10);
  car.make_ts_healthy();
  car.run_for(10);
  car.set_brake(0.8);
  car.dash_rtd_button(true);
  REQUIRE(car.run_until(TRACTIVE_SYSTEM_ENABLED, 100));

  car.run_for(1000);
  car.acu_status(false, true); // IMD trips a second into the buzzer
  car.run_for(3000);

  CHECK_FALSE(car.ever_reached(READY_TO_DRIVE));
  CHECK(car.vcu.get_current_state() == TRACTIVE_SYSTEM_DISABLED);
  CHECK_FALSE(sim::pin(BUZZER));

  // Entering ENABLED turned the inverter on, so backing out has to turn it off
  CHECK_FALSE(car.inverter.get_inverter_enable());
  Car::InverterCommand cmd;
  REQUIRE(car.last_inverter_command(&cmd));
  CHECK_FALSE(cmd.enable);
}

//
//// Driving

TEST_CASE("ready to drive enables the inverter with the full torque limit") {
  Car car;
  car.drive_to_rtd();
  REQUIRE(car.vcu.get_current_state() == READY_TO_DRIVE);

  CHECK(car.inverter.get_inverter_enable());
  CHECK(car.inverter.get_torque_limit() == MAX_TORQUE_LIMIT_NM);

  Car::InverterCommand cmd;
  REQUIRE(car.last_inverter_command(&cmd));
  CHECK(cmd.enable);
  CHECK(cmd.torque_limit_nm == doctest::Approx(MAX_TORQUE_LIMIT_NM));
  CHECK(cmd.torque_nm == doctest::Approx(0)); // pedal released
}

TEST_CASE("throttle maps to torque once ready to drive") {
  Car car;
  car.drive_to_rtd();
  car.set_throttle(0.5);
  car.run_for(20);

  Car::InverterCommand cmd;
  REQUIRE(car.last_inverter_command(&cmd));
  CHECK(cmd.torque_nm ==
        doctest::Approx(0.5 * MAX_TORQUE_LIMIT_NM).epsilon(0.02));
}

TEST_CASE("torque is power limited at speed") {
  Car car;
  car.drive_to_rtd();

  const int16_t rpm = 6000;
  car.inverter_motor_rpm(rpm);
  car.set_throttle(1.0);
  car.run_for(20);

  // Open loop: torque = P * efficiency / omega
  const double omega = rpm * 2 * M_PI / 60;
  const double expected = POWER_LIMIT_KW * 1000.0 * 0.90 / omega;
  REQUIRE(expected < MAX_TORQUE_LIMIT_NM); // otherwise this proves nothing

  Car::InverterCommand cmd;
  REQUIRE(car.last_inverter_command(&cmd));
  CHECK(cmd.torque_nm == doctest::Approx(expected).epsilon(0.02));
}

TEST_CASE("throttle does nothing before ready to drive") {
  Car car;
  car.run_for(10);
  car.make_ts_healthy();
  car.set_throttle(0.8);
  car.run_for(500);
  REQUIRE(car.vcu.get_current_state() == TRACTIVE_SYSTEM_ENERGIZED);

  for (const auto &m : car.inv().sent(CAN_ID_M192_COMMAND_MESSAGE)) {
    can_obj_car_h_t rx{};
    unpack_message(&rx, m.id, m.buf.val, m.length, 0);
    double torque;
    uint8_t enable;
    decode_can_0x0c0_VCU_INV_Torque_Command(&rx, &torque);
    decode_can_0x0c0_VCU_INV_Inverter_Enable(&rx, &enable);
    CHECK(torque == 0);
    CHECK(enable == 0);
  }
}

TEST_CASE("a pedal fault zeroes torque but stays ready to drive") {
  Car car;
  car.drive_to_rtd();
  car.set_throttle(0.5);
  car.run_for(20);

  SUBCASE("brake and throttle together") { car.set_brake(0.8); }
  SUBCASE("APPS disagree") {
    car.apps2_raw = START_ACCELERATOR_PEDAL_2; // APPS2 reads released
  }
  SUBCASE("BSE shorted") { car.brake_raw = MAX_BRAKE_PEDAL + 10; }

  car.run_for(20);
  CHECK(car.vcu.get_current_state() == READY_TO_DRIVE);
  Car::InverterCommand cmd;
  REQUIRE(car.last_inverter_command(&cmd));
  CHECK(cmd.torque_nm == 0);
}

//
//// Faults while live

TEST_CASE("drops out of ready to drive when TS goes unsafe") {
  Car car;
  car.drive_to_rtd();
  car.set_throttle(0.5);
  car.run_for(20);
  REQUIRE(car.vcu.get_current_state() == READY_TO_DRIVE);

  SUBCASE("IMD trips") { car.acu_status(false, true); }
  SUBCASE("BMS trips") { car.acu_status(true, false); }
  SUBCASE("precharge drops") { car.precharge_state(0); }
  SUBCASE("bus voltage sags") {
    car.inverter_bus_voltage(TRACTIVE_SYSTEM_MINIMUM_VOLTAGE - 50);
  }

  car.run_for(20);
  CHECK(car.vcu.get_current_state() == TRACTIVE_SYSTEM_DISABLED);
  CHECK_FALSE(car.inverter.get_inverter_enable());
  CHECK_FALSE(sim::pin(LOWSIDE1));

  // And whatever it sends the inverter from here on is a disabled ping
  car.run_for(200);
  Car::InverterCommand cmd;
  REQUIRE(car.last_inverter_command(&cmd));
  CHECK_FALSE(cmd.enable);
  CHECK(cmd.torque_nm == 0);
}

TEST_CASE("drops out of energized when TS goes unsafe") {
  Car car;
  car.run_for(10);
  car.make_ts_healthy();
  car.run_for(10);
  REQUIRE(car.vcu.get_current_state() == TRACTIVE_SYSTEM_ENERGIZED);

  car.acu_status(false, true);
  car.run_for(20);
  CHECK(car.vcu.get_current_state() == TRACTIVE_SYSTEM_DISABLED);
  CHECK_FALSE(sim::pin(LOWSIDE1));
}

TEST_CASE("recovers to energized once the fault clears") {
  Car car;
  car.drive_to_rtd();
  car.acu_status(false, true);
  car.run_for(20);
  REQUIRE(car.vcu.get_current_state() == TRACTIVE_SYSTEM_DISABLED);

  car.acu_status(true, true);
  car.run_for(20);
  CHECK(car.vcu.get_current_state() == TRACTIVE_SYSTEM_ENERGIZED);
  // ...but has to go through RTD again before driving
  CHECK_FALSE(car.inverter.get_inverter_enable());
}

//
//// What goes on the wire

TEST_CASE("pings the inverter at 10 Hz while waiting") {
  Car car;
  car.run_for(10);
  size_t before = count_sent(car.inv(), CAN_ID_M192_COMMAND_MESSAGE);
  car.run_for(1000);
  size_t pings = count_sent(car.inv(), CAN_ID_M192_COMMAND_MESSAGE) - before;
  CHECK(pings >= 9);
  CHECK(pings <= 11);
}

TEST_CASE("commands torque at 200 Hz when ready to drive") {
  Car car;
  car.drive_to_rtd();
  size_t before = count_sent(car.inv(), CAN_ID_M192_COMMAND_MESSAGE);
  car.run_for(1000);
  size_t cmds = count_sent(car.inv(), CAN_ID_M192_COMMAND_MESSAGE) - before;
  CHECK(cmds >= 195);
  CHECK(cmds <= 205);
}

TEST_CASE("reports its state at 1 Hz on every bus") {
  Car car;
  car.run_for(10);
  car.make_ts_healthy();
  car.run_for(2000);

  CHECK(car.last_reported_state() == TRACTIVE_SYSTEM_ENERGIZED);
  CHECK(count_sent(car.daq(), CAN_ID_VCU_STATUS) >= 2);
  CHECK(count_sent(car.inv(), CAN_ID_VCU_STATUS) >= 2);
  CHECK(count_sent(car.acc(), CAN_ID_VCU_STATUS) >= 2);
}

TEST_CASE("mirrors ACU and inverter traffic to DAQ and forwards between buses") {
  Car car;
  car.run_for(10);
  car.acu_status(true, true);
  car.inverter_bus_voltage(450);

  can_message dash_info;
  dash_info.id = CAN_ID_MSGID_0X6B3;
  car.acc().inject(dash_info);

  car.run_for(10);

  // Everything heard on ACC/INV shows up on DAQ for logging
  CHECK(count_sent(car.daq(), CAN_ID_ACU_SHUTDOWN_STATUS) == 1);
  CHECK(count_sent(car.daq(), CAN_ID_M167_VOLTAGE_INFO) == 1);
  // 0x6B3 goes to the inverter bus for the dash
  CHECK(count_sent(car.inv(), CAN_ID_MSGID_0X6B3) == 1);
  // Inverter bus voltage goes to the ACU for precharge
  CHECK(count_sent(car.acc(), CAN_ID_M167_VOLTAGE_INFO) == 1);
}

TEST_CASE("every frame that arrived since the last loop is handled") {
  // update_acc_can/update_inv_can used to read one frame per bus per loop,
  // so a burst of chatter delayed safety data behind it, loop by loop
  Car car;
  car.drive_to_rtd();
  for (int i = 0; i < 50; i++)
    car.pack_power(450, 0);
  car.acu_status(false, true); // IMD trips, queued behind the chatter
  car.run_for(1);
  CHECK(car.vcu.get_current_state() == TRACTIVE_SYSTEM_DISABLED);
}

TEST_CASE("a burst of frames between loops isn't lost") {
  // With FlexCAN_T4 polled, a 5th standard-ID frame within one loop was lost
  Car car;
  car.drive_to_rtd();
  for (int i = 0; i < 200; i++)
    car.pack_power(450, 0);
  car.run_for(1);
  CHECK(car.acc().rx_dropped() == 0);
  // all 200 forwarded to the DAQ bus
  CHECK(count_sent(car.daq(), CAN_ID_MSGID_0X6B1) >= 200);
}

TEST_CASE("a flooded bus drops new frames, not the loop") {
  // More than the queue holds between two loops: the overflow is counted
  // and dropped, and the VCU keeps running on what it has
  Car car;
  car.drive_to_rtd();
  for (size_t i = 0; i < CAN_RX_QUEUE_SIZE + 10; i++)
    car.pack_power(450, 0);
  CHECK(car.acc().rx_dropped() == 10);
  car.run_for(2);
  CHECK(car.acc().rx_waiting() == 0);
  CHECK(car.vcu.get_current_state() == READY_TO_DRIVE);
}

//
//// Known issues: behaviour that looks wrong but needs a call from whoever
//// owns this code. These WARN instead of failing.

TEST_CASE("KNOWN ISSUE: should leave ready to drive if the ACU goes silent") {
  // ts_safe() trusts the last IMD/BMS/precharge values forever. If the ACU
  // board dies or its CAN line breaks, the car stays live.
  Car car;
  car.drive_to_rtd();
  car.run_for(2000); // no ACU frames at all for 2 s
  WARN(car.vcu.get_current_state() != READY_TO_DRIVE);
}


