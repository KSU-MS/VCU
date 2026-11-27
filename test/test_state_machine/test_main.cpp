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

TEST_CASE("[EV.10.6.2] needs both the RTD button and the brake to go ready to drive") {
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

TEST_CASE("[EV.10.6.2] no ready to drive without the tractive system active") {
  // Brake held and the button pressed, but TS never came up
  Car car;
  car.run_for(10);
  car.set_brake(0.8);
  car.dash_rtd_button(true);
  car.run_for(3000);
  CHECK(car.vcu.get_current_state() == TRACTIVE_SYSTEM_DISABLED);
  CHECK_FALSE(car.ever_reached(READY_TO_DRIVE));
}

TEST_CASE("[EV.10.7.2] sounds the buzzer for 1-3 s before ready to drive") {
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

  // Continuously, every loop, until ready to drive
  bool gap = false;
  while (car.vcu.get_current_state() != READY_TO_DRIVE && sim::now() - buzzer_start < 5000) {
    car.run_for(1);
    if (car.vcu.get_current_state() != READY_TO_DRIVE && !sim::pin(BUZZER))
      gap = true;
  }
  REQUIRE(car.vcu.get_current_state() == READY_TO_DRIVE);
  CHECK_FALSE(gap);
  CHECK(sim::now() - buzzer_start >= 1000); // the rule
  CHECK(sim::now() - buzzer_start <= 3000);
  CHECK(sim::now() - buzzer_start >= 2000); // RTD_BUZZER_DURATION_MS
  CHECK(sim::now() - buzzer_start <= 2500);
  CHECK_FALSE(sim::pin(BUZZER));
}

TEST_CASE("[EV.10.7.3] the buzzer only ever sounds on the way into ready to drive") {
  // A whole session: power up, TS on, a TS fault and recovery, ready to
  // drive, driving, pedal faults, TS dropping out while driving
  Car car;
  uint32_t buzzing_outside_rtd_ms = 0;
  auto run = [&](uint32_t ms) {
    for (uint32_t i = 0; i < ms; i++) {
      car.run_for(1);
      if (sim::pin(BUZZER) && car.vcu.get_current_state() != TRACTIVE_SYSTEM_ENABLED)
        buzzing_outside_rtd_ms++;
    }
  };
  run(100);
  car.make_ts_healthy();
  run(100);
  car.acu_status(false, true); // IMD trips
  run(100);
  car.make_ts_healthy();
  run(100);
  car.set_brake(0.8);
  car.dash_rtd_button(true);
  run(3000);
  REQUIRE(car.vcu.get_current_state() == READY_TO_DRIVE);
  car.dash_rtd_button(false);
  car.set_brake(0);
  car.set_throttle(0.5);
  run(500);
  car.apps2_raw = 0;
  run(200);
  car.set_throttle(0);
  car.acu_status(true, false); // BMS trips while live
  run(500);
  CHECK(buzzing_outside_rtd_ms == 0);
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
  CHECK(car.vcu.get_current_state() == TRACTIVE_SYSTEM_FAULT); // still live
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

TEST_CASE("[EV.10.6.1] throttle maps to torque once ready to drive") {
  Car car;
  car.drive_to_rtd();
  car.set_throttle(0.5);
  car.run_for(20);

  Car::InverterCommand cmd;
  REQUIRE(car.last_inverter_command(&cmd));
  CHECK(cmd.torque_nm ==
        doctest::Approx(0.5 * MAX_TORQUE_LIMIT_NM).epsilon(0.02));
}

TEST_CASE("[T.4.2.9][T.4.2.5] an APPS unplugged at light throttle sends no torque once released") {
  Car car;
  car.drive_to_rtd();
  car.set_throttle(0.08);
  car.run_for(20);
  car.apps1_raw = 0; // APPS1 connector falls off
  car.run_for(20);
  car.set_throttle(0); // foot off; APPS1 still unplugged
  car.apps1_raw = 0;
  car.run_for(50);

  Car::InverterCommand cmd;
  REQUIRE(car.last_inverter_command(&cmd));
  CHECK(cmd.torque_nm == 0);
  CHECK(car.pedals.get_apps_fault_ok_low());
}

TEST_CASE("[T.4.3.3][EV.10.6.2] a broken brake sensor doesn't count as the brake for ready to drive") {
  // The brake is pressed, then its sensor shorts: the last good reading
  // mustn't be what lets the car go live
  Car car;
  car.run_for(10);
  car.make_ts_healthy();
  car.run_for(10);
  REQUIRE(car.vcu.get_current_state() == TRACTIVE_SYSTEM_ENERGIZED);
  car.set_brake(0.8);
  car.run_for(10);
  car.brake_raw = 1023; // shorted to GLV
  car.dash_rtd_button(true);
  car.run_for(3000);
  CHECK_FALSE(car.ever_reached(TRACTIVE_SYSTEM_ENABLED));
  CHECK_FALSE(car.ever_reached(READY_TO_DRIVE));
}

TEST_CASE("[EV.3.3.1] torque is power limited at speed") {
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

TEST_CASE("[EV.10.6.1] throttle does nothing before ready to drive") {
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

TEST_CASE("[T.4.2.5][T.4.3.3][EV.4.7.2] a pedal fault zeroes torque but stays ready to drive") {
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

TEST_CASE("a fault while live goes to the fault state, not disabled") {
  // The ACU tripping doesn't make the high voltage go away: the VCU mustn't
  // claim the tractive system is disabled while it may still be live
  Car car;
  car.drive_to_rtd();
  car.set_throttle(0.5);
  car.run_for(20);
  REQUIRE(car.vcu.get_current_state() == READY_TO_DRIVE);

  uint8_t expected = 0;
  SUBCASE("IMD trips") {
    car.acu_status(false, true);
    expected = FAULT_IMD;
  }
  SUBCASE("BMS trips") {
    car.acu_status(true, false);
    expected = FAULT_BMS;
  }

  car.run_for(20);
  CHECK(car.vcu.get_current_state() == TRACTIVE_SYSTEM_FAULT);
  CHECK(car.vcu.fault_code() == expected);
  CHECK_FALSE(car.inverter.get_inverter_enable());
  CHECK(sim::pin(LOWSIDE1)); // pumps stay on while it may be live

  // And whatever it sends the inverter from here on is a disabled ping
  car.run_for(200);
  Car::InverterCommand cmd;
  REQUIRE(car.last_inverter_command(&cmd));
  CHECK_FALSE(cmd.enable);
  CHECK(cmd.torque_nm == 0);
}

TEST_CASE("an ordinary shutdown goes back to disabled") {
  // No fault: the driver switched the tractive system off
  Car car;
  car.drive_to_rtd();
  SUBCASE("precharge undone") { car.precharge_state(0); }
  SUBCASE("bus voltage falling") {
    car.inverter_bus_voltage(TRACTIVE_SYSTEM_MINIMUM_VOLTAGE - 50);
  }
  car.run_for(20);
  CHECK(car.vcu.get_current_state() == TRACTIVE_SYSTEM_DISABLED);
  CHECK(car.vcu.fault_code() == 0);
  CHECK_FALSE(car.inverter.get_inverter_enable());
  CHECK_FALSE(sim::pin(LOWSIDE1));
}

TEST_CASE("a fault in energized goes to the fault state too") {
  Car car;
  car.run_for(10);
  car.make_ts_healthy();
  car.run_for(10);
  REQUIRE(car.vcu.get_current_state() == TRACTIVE_SYSTEM_ENERGIZED);

  car.acu_status(false, true);
  car.run_for(20);
  CHECK(car.vcu.get_current_state() == TRACTIVE_SYSTEM_FAULT);
}

TEST_CASE("no ready to drive from the fault state, whatever the driver does") {
  Car car;
  car.drive_to_rtd();
  car.acu_status(false, true);
  car.run_for(20);
  REQUIRE(car.vcu.get_current_state() == TRACTIVE_SYSTEM_FAULT);
  const size_t transitions = car.transitions.size();

  car.set_brake(0.8);
  car.dash_rtd_button(true);
  car.set_throttle(0.5);
  car.run_for(5000);
  CHECK(car.vcu.get_current_state() == TRACTIVE_SYSTEM_FAULT);
  CHECK(car.transitions.size() == transitions);
  Car::InverterCommand cmd;
  REQUIRE(car.last_inverter_command(&cmd));
  CHECK(cmd.torque_nm == 0);
  CHECK_FALSE(cmd.enable);
}

TEST_CASE("the fault clears: disabled, then energized, then it needs the driver") {
  Car car;
  car.drive_to_rtd();
  car.acu_status(false, true);
  car.run_for(20);
  REQUIRE(car.vcu.get_current_state() == TRACTIVE_SYSTEM_FAULT);

  car.acu_status(true, true);
  car.run_for(20);
  CHECK(car.vcu.get_current_state() == TRACTIVE_SYSTEM_ENERGIZED);
  CHECK(car.ever_reached(TRACTIVE_SYSTEM_DISABLED));
  // ...but has to go through RTD again before driving
  CHECK_FALSE(car.inverter.get_inverter_enable());
}

TEST_CASE("the voltage gone ends the fault state, even if the fault hasn't") {
  // IMD still tripped, but the accumulator's disconnected and the inverter
  // reads the bus discharged: not live any more
  Car car;
  car.drive_to_rtd();
  car.acu_status(false, true);
  car.run_for(20);
  REQUIRE(car.vcu.get_current_state() == TRACTIVE_SYSTEM_FAULT);

  car.inverter_bus_voltage(HIGH_VOLTAGE_PRESENT + 10);
  car.run_for(200);
  CHECK(car.vcu.get_current_state() == TRACTIVE_SYSTEM_FAULT);

  car.inverter_bus_voltage(HIGH_VOLTAGE_PRESENT - 10);
  car.run_for(200);
  CHECK(car.vcu.get_current_state() == TRACTIVE_SYSTEM_DISABLED);
  CHECK(car.vcu.fault_code() == FAULT_IMD); // and still says why
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
  CHECK(car.vcu.get_current_state() == TRACTIVE_SYSTEM_FAULT);
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
//// The other boards going quiet: a dead ACU or inverter, or a broken CAN
//// wire, mustn't leave the car live on the last thing they said

namespace {
// Run until the VCU leaves `from`, return how long that took (or `limit`)
uint32_t ms_to_leave(Car &car, state from, uint32_t limit) {
  for (uint32_t t = 0; t < limit; t++) {
    if (car.vcu.get_current_state() != from)
      return t;
    car.run_for(1);
  }
  return limit;
}
} // namespace

TEST_CASE("the ACU going silent drops out of ready to drive") {
  Car car;
  car.drive_to_rtd();
  car.set_throttle(0.5);
  car.run_for(200);
  REQUIRE(car.vcu.get_current_state() == READY_TO_DRIVE);

  uint8_t expected = 0;
  SUBCASE("ACU") {
    car.acu_talking = false;
    expected = FAULT_ACU_SILENT;
  }
  SUBCASE("inverter") {
    // Can't tell whether there's still voltage: that counts as maybe live
    car.inverter_talking = false;
    expected = FAULT_INVERTER_SILENT;
  }

  // Its last status was up to 100 ms old already
  CHECK(ms_to_leave(car, READY_TO_DRIVE, 2000) <= SAFETY_DATA_TIMEOUT_MS + 2);
  CHECK(car.vcu.get_current_state() == TRACTIVE_SYSTEM_FAULT);
  CHECK(car.vcu.fault_code() == expected);
  CHECK_FALSE(car.vcu.safety_data_fresh());

  car.run_for(50);
  Car::InverterCommand cmd;
  REQUIRE(car.last_inverter_command(&cmd));
  CHECK_FALSE(cmd.enable);
  CHECK(cmd.torque_nm == 0);
}

TEST_CASE("a gap shorter than the timeout is ridden through") {
  Car car;
  car.drive_to_rtd();
  const size_t transitions = car.transitions.size();
  car.acu_talking = false;
  car.inverter_talking = false;
  car.run_for(SAFETY_DATA_TIMEOUT_MS - 150); // plus up to 100 ms already
  car.acu_talking = true;
  car.inverter_talking = true;
  car.run_for(500);
  CHECK(car.vcu.get_current_state() == READY_TO_DRIVE);
  CHECK(car.transitions.size() == transitions); // never left
}

TEST_CASE("back from silence: energized again, but ready to drive needs the driver") {
  Car car;
  car.drive_to_rtd();
  car.acu_talking = false;
  car.run_for(1000);
  REQUIRE(car.vcu.get_current_state() == TRACTIVE_SYSTEM_FAULT);

  car.acu_talking = true;
  car.set_throttle(0.5);
  car.run_for(1000);
  CHECK(car.vcu.get_current_state() == TRACTIVE_SYSTEM_ENERGIZED);
  Car::InverterCommand cmd;
  REQUIRE(car.last_inverter_command(&cmd));
  CHECK(cmd.torque_nm == 0);
}

TEST_CASE("never heard from the ACU: the tractive system can't energize") {
  // Nothing at all on the ACU bus since boot, but the inverter reports
  // voltage: the ACU's defaults mustn't count as its say-so
  Car car;
  car.run_for(10);
  car.inverter_bus_voltage(450);
  car.run_for(2000);
  CHECK_FALSE(car.ever_reached(TRACTIVE_SYSTEM_ENERGIZED));
}

//
//// Known issues: behaviour that looks wrong but needs a call from whoever
//// owns this code. These WARN instead of failing.



