// The Pi as the controls brain, the VCU as the safety authority
// (docs/pi-supervisor.md, pi_supervisor.hpp): the Pi's torque ceiling, how
// every frame is checked, the fallback when the Pi goes wrong and the
// handover back to it.
#include <sim_test_main.hpp>

#include <cmath>
#include <cstring>
#include <random>
#include <sim_car.hpp>

using sim::Car;

namespace {

// The Pi, sending its command at 100 Hz while `talking`
struct FakePi {
  double ceiling_nm = 240;
  bool engage = true;
  bool talking = true;
  uint8_t counter = 0;

  void send(Car &car) {
    uint8_t d[8];
    pi_frame::encode(d, ceiling_nm, engage, counter++);
    car.pi_frame(d);
  }

  // Run the car for `ms`, the Pi sending every 10 ms
  void run(Car &car, uint32_t ms) {
    for (uint32_t i = 0; i < ms; i++) {
      if (talking && sim::now() % 10 == 0)
        send(car);
      car.run_for(1);
    }
  }
};

// The vcu_pi_status frame, decoded with the DBC's code
struct PiStatus {
  int mode, reason, rejected;
  double driver_nm, sent_nm, ceiling_nm; // ceiling < 0: none
};

PiStatus pi_status() {
  const can_message *m = sim::bus(TEENSY_CAN3).last_sent(CAN_ID_VCU_PI_STATUS);
  REQUIRE(m != nullptr);
  can_obj_car_h_t rx{};
  unpack_message(&rx, m->id, m->buf.val, m->length, 0);
  uint8_t mode, reason, rejected;
  PiStatus s{};
  decode_can_0x0cd_vcu_pi_mode(&rx, &mode);
  decode_can_0x0cd_vcu_pi_reason(&rx, &reason);
  decode_can_0x0cd_vcu_pi_rejected_frames(&rx, &rejected);
  decode_can_0x0cd_vcu_pi_driver_torque(&rx, &s.driver_nm);
  decode_can_0x0cd_vcu_pi_torque_sent(&rx, &s.sent_nm);
  decode_can_0x0cd_vcu_pi_ceiling(&rx, &s.ceiling_nm);
  s.mode = mode;
  s.reason = reason;
  s.rejected = rejected;
  if (s.ceiling_nm > 6553)
    s.ceiling_nm = -1; // 0xFFFF: none
  return s;
}

double commanded_torque(Car &car) {
  Car::InverterCommand cmd{};
  REQUIRE(car.last_inverter_command(&cmd));
  return cmd.torque_nm;
}

// Ready to drive, with the Pi healthy and in control
void pi_in_control(Car &car, FakePi &pi) {
  car.drive_to_rtd();
  REQUIRE(car.vcu.get_current_state() == READY_TO_DRIVE);
  pi.run(car, PI_HEALTHY_MS + 100);
  REQUIRE(car.vcu.pi.get_mode() == PiSupervisor::PI_CONTROL);
}

} // namespace

TEST_CASE("the Pi's frame checksum is CRC-8 SAE J1850") {
  // Its standard check value: "123456789" -> 0x4B. The first two bytes
  // go in as the ID, little endian
  const uint8_t rest[] = {'3', '4', '5', '6', '7', '8', '9'};
  CHECK(pi_frame::crc8('1' | '2' << 8, rest, sizeof rest) == 0x4B);
}

TEST_CASE("pi_frame::encode makes the frame the DBC describes") {
  uint8_t d[8];
  pi_frame::encode(d, 123.4, true, 9);
  uint64_t raw = 0;
  for (int i = 7; i >= 0; i--)
    raw = raw << 8 | d[i];
  can_obj_car_h_t rx{};
  unpack_message(&rx, CAN_ID_PI_TORQUE_COMMAND, raw, 8, 0);
  double ceiling;
  uint8_t engage, counter, checksum;
  decode_can_0x010_pi_torque_ceiling(&rx, &ceiling);
  decode_can_0x010_pi_engage(&rx, &engage);
  decode_can_0x010_pi_counter(&rx, &counter);
  decode_can_0x010_pi_checksum(&rx, &checksum);
  CHECK(ceiling == doctest::Approx(123.4));
  CHECK(engage == 1);
  CHECK(counter == 9);
  CHECK(checksum == pi_frame::crc8(CAN_ID_PI_TORQUE_COMMAND, d, 7));
}

TEST_CASE("with no Pi the VCU drives exactly as before") {
  Car car;
  car.drive_to_rtd();
  car.set_throttle(0.5);
  car.run_for(100);
  CHECK(commanded_torque(car) ==
        doctest::Approx(0.5 * MAX_TORQUE_LIMIT_NM).epsilon(0.01));
  CHECK(pi_status().mode == PiSupervisor::LOCAL_CONTROL);
  CHECK(pi_status().reason == PiSupervisor::PI_SILENT);
  CHECK(pi_status().ceiling_nm < 0);
}

TEST_CASE("a healthy Pi gets control once it's been healthy long enough and "
          "the driver lifts") {
  Car car;
  FakePi pi;
  pi.ceiling_nm = 50;
  car.drive_to_rtd();
  car.set_throttle(0.8);
  pi.run(car, PI_HEALTHY_MS - 100);
  CHECK(pi_status().mode == PiSupervisor::LOCAL_CONTROL);
  CHECK(pi_status().reason == PiSupervisor::PI_SETTLING);

  pi.run(car, 200);
  // Healthy for long enough, but the torque mustn't step under the driver
  CHECK(pi_status().mode == PiSupervisor::LOCAL_CONTROL);
  CHECK(pi_status().reason == PiSupervisor::PI_WAITING_FOR_LIFT);
  CHECK(commanded_torque(car) == doctest::Approx(0.8 * MAX_TORQUE_LIMIT_NM)
                                     .epsilon(0.01));

  car.set_throttle(0);
  pi.run(car, 20);
  CHECK(car.vcu.pi.get_mode() == PiSupervisor::PI_CONTROL);
  car.set_throttle(1.0);
  pi.run(car, 50);
  CHECK(commanded_torque(car) == doctest::Approx(50).epsilon(0.01));
  CHECK(pi_status().mode == PiSupervisor::PI_CONTROL);
  CHECK(pi_status().ceiling_nm == doctest::Approx(50));
  CHECK(pi_status().driver_nm ==
        doctest::Approx(MAX_TORQUE_LIMIT_NM).epsilon(0.01));
}

TEST_CASE("the Pi's ceiling only ever takes torque away") {
  Car car;
  FakePi pi;
  pi_in_control(car, pi);
  car.set_throttle(0.3);
  pi.run(car, 50);
  CHECK(commanded_torque(car) ==
        doctest::Approx(0.3 * MAX_TORQUE_LIMIT_NM).epsilon(0.01));
}

TEST_CASE("the Pi going quiet hands control back within 30 ms, the torque "
          "ramping up from its ceiling") {
  Car car;
  FakePi pi;
  pi.ceiling_nm = 40;
  pi_in_control(car, pi);
  car.set_throttle(1.0);
  pi.run(car, 50);
  REQUIRE(commanded_torque(car) == doctest::Approx(40).epsilon(0.01));

  pi.talking = false;
  uint32_t local_after = 0;
  double last = 40, worst_step = 0;
  for (uint32_t ms = 1; ms <= 500; ms++) {
    pi.run(car, 1);
    if (!local_after &&
        car.vcu.pi.get_mode() == PiSupervisor::LOCAL_CONTROL)
      local_after = ms;
    const double now = commanded_torque(car);
    CHECK(now >= last - 0.1); // only up
    worst_step = std::max(worst_step, now - last);
    last = now;
  }
  CHECK(local_after > 0);
  CHECK(local_after <= PI_COMMAND_TIMEOUT_MS + 10);
  // No step bigger than the ramp allows between two 200 Hz commands
  CHECK(worst_step <= PI_FALLBACK_RAMP_NM_PER_S * 0.005 + 1);
  CHECK(last == doctest::Approx(MAX_TORQUE_LIMIT_NM).epsilon(0.01));
  CHECK(pi_status().reason == PiSupervisor::PI_SILENT);
}

TEST_CASE("a bad frame from the Pi hands control back at once") {
  struct Case {
    const char *name;
    void (*spoil)(uint8_t *d, uint8_t *len);
    int reason;
  };
  const Case cases[] = {
      {"checksum wrong", [](uint8_t *d, uint8_t *) { d[7] ^= 1; },
       PiSupervisor::PI_BAD_CHECKSUM},
      {"same counter again",
       [](uint8_t *d, uint8_t *) {
         d[6] = uint8_t((d[6] - 1) & 0x0F);
         d[7] = pi_frame::crc8(CAN_ID_PI_TORQUE_COMMAND, d, 7);
       },
       PiSupervisor::PI_BAD_COUNTER},
      {"counter jumped 5",
       [](uint8_t *d, uint8_t *) {
         d[6] = uint8_t((d[6] + 4) & 0x0F);
         d[7] = pi_frame::crc8(CAN_ID_PI_TORQUE_COMMAND, d, 7);
       },
       PiSupervisor::PI_BAD_COUNTER},
      {"reserved bits set",
       [](uint8_t *d, uint8_t *) {
         d[4] = 0x80;
         d[7] = pi_frame::crc8(CAN_ID_PI_TORQUE_COMMAND, d, 7);
       },
       PiSupervisor::PI_BAD_VALUE},
      {"ceiling over the torque limit",
       [](uint8_t *d, uint8_t *) {
         const uint16_t raw = (MAX_TORQUE_LIMIT_NM + 1) * 10;
         d[0] = uint8_t(raw);
         d[1] = uint8_t(raw >> 8);
         d[7] = pi_frame::crc8(CAN_ID_PI_TORQUE_COMMAND, d, 7);
       },
       PiSupervisor::PI_BAD_VALUE},
      {"short frame", [](uint8_t *, uint8_t *len) { *len = 6; },
       PiSupervisor::PI_BAD_VALUE},
  };
  for (const auto &c : cases) {
    SUBCASE(c.name) {
      Car car;
      FakePi pi;
      pi.ceiling_nm = 60;
      pi_in_control(car, pi);
      car.set_throttle(1.0);
      pi.run(car, 25); // between two frames

      uint8_t d[8];
      uint8_t len = 8;
      pi_frame::encode(d, pi.ceiling_nm, true, pi.counter++);
      c.spoil(d, &len);
      car.pi_frame(d, len);
      car.run_for(1);
      CHECK(car.vcu.pi.get_mode() == PiSupervisor::LOCAL_CONTROL);
      CHECK(car.vcu.pi.get_reason() == c.reason);
      CHECK(car.vcu.pi.rejected_frames() == 1);
    }
  }
}

TEST_CASE("losing up to two frames in a row is fine, three is too many") {
  Car car;
  FakePi pi;
  pi_in_control(car, pi);
  pi.counter += 2; // two never arrived
  pi.run(car, 30);
  CHECK(car.vcu.pi.get_mode() == PiSupervisor::PI_CONTROL);
  CHECK(car.vcu.pi.rejected_frames() == 0);
  pi.counter += 3;
  pi.run(car, 30);
  CHECK(car.vcu.pi.get_mode() == PiSupervisor::LOCAL_CONTROL);
  CHECK(car.vcu.pi.rejected_frames() == 1);
  // the frames after it count from there: settling again
  CHECK(car.vcu.pi.get_reason() == PiSupervisor::PI_SETTLING);
}

TEST_CASE("the counter wraps from 15 to 0") {
  Car car;
  FakePi pi;
  pi_in_control(car, pi);
  pi.run(car, 400); // 40 frames, the counter goes round twice
  CHECK(car.vcu.pi.get_mode() == PiSupervisor::PI_CONTROL);
  CHECK(car.vcu.pi.rejected_frames() == 0);
}

TEST_CASE("the Pi letting go hands control back") {
  Car car;
  FakePi pi;
  pi_in_control(car, pi);
  pi.engage = false;
  pi.run(car, 20);
  CHECK(car.vcu.pi.get_mode() == PiSupervisor::LOCAL_CONTROL);
  CHECK(car.vcu.pi.get_reason() == PiSupervisor::PI_NOT_REQUESTED);
}

TEST_CASE("after a fallback the Pi has to earn control back") {
  Car car;
  FakePi pi;
  pi.ceiling_nm = 60;
  pi_in_control(car, pi);
  car.set_throttle(1.0);
  uint8_t d[8];
  pi_frame::encode(d, 60, true, pi.counter++);
  d[7] ^= 0xFF;
  car.pi_frame(d);
  pi.run(car, 100);
  REQUIRE(car.vcu.pi.get_mode() == PiSupervisor::LOCAL_CONTROL);

  // Good frames again: settling first, then waiting for a lift
  CHECK(car.vcu.pi.get_reason() == PiSupervisor::PI_SETTLING);
  pi.run(car, PI_HEALTHY_MS);
  CHECK(car.vcu.pi.get_reason() == PiSupervisor::PI_WAITING_FOR_LIFT);
  car.set_throttle(0);
  pi.run(car, 20);
  CHECK(car.vcu.pi.get_mode() == PiSupervisor::PI_CONTROL);
}

TEST_CASE("[EV.3.1.3] nothing the Pi sends can raise the torque above the driver's "
          "request (x3000 ms of random frames)") {
  Car car;
  FakePi pi;
  pi_in_control(car, pi);
  std::mt19937 rng(7);
  std::uniform_int_distribution<int> byte(0, 255);
  std::uniform_real_distribution<double> u(0, 1);
  int checked = 0;
  double throttle = 0, before = 0;
  uint32_t changed_ms = 0;
  for (uint32_t ms = 0; ms < 3000; ms++) {
    if (ms % 200 == 0) {
      before = throttle;
      throttle = u(rng) < 0.2 ? 0 : u(rng);
      changed_ms = ms;
      car.set_throttle(throttle);
    }
    const double r = u(rng);
    if (r < 0.08) {
      pi.ceiling_nm = u(rng) * 300;
      pi.engage = u(rng) < 0.9;
      pi.send(car); // a good frame, any ceiling (over the limit: rejected)
    } else if (r < 0.1) {
      uint8_t d[8];
      for (auto &b : d)
        b = uint8_t(byte(rng));
      car.pi_frame(d, uint8_t(byte(rng) % 9)); // rubbish
    }
    car.run_for(1);
    // What the driver asked for, either side of a throttle change (the
    // command is sent at 200 Hz, so it can be one behind)
    const double asked =
        (ms - changed_ms < 10 ? std::max(throttle, before) : throttle) *
        MAX_TORQUE_LIMIT_NM;
    CHECK(commanded_torque(car) <= asked + 0.5);
    checked++;
  }
  CHECK(checked == 3000);
}

TEST_CASE("the status frame goes out on the DAQ bus at 100 Hz") {
  Car car;
  FakePi pi;
  car.drive_to_rtd();
  const size_t before = sim::bus(TEENSY_CAN3).sent(CAN_ID_VCU_PI_STATUS).size();
  pi.run(car, 1000);
  const size_t sent =
      sim::bus(TEENSY_CAN3).sent(CAN_ID_VCU_PI_STATUS).size() - before;
  CHECK(sent >= 99);
  CHECK(sent <= 101);
}

TEST_CASE("frames with the Pi's ID on another bus are ignored") {
  Car car;
  car.drive_to_rtd();
  for (int i = 0; i < 100; i++) {
    uint8_t d[8];
    pi_frame::encode(d, 10, true, uint8_t(i));
    can_message msg;
    msg.id = CAN_ID_PI_TORQUE_COMMAND;
    std::memcpy(msg.buf.byte, d, 8);
    car.inv().inject(msg);
    car.run_for(10);
  }
  CHECK(car.vcu.pi.get_mode() == PiSupervisor::LOCAL_CONTROL);
  CHECK(car.vcu.pi.get_reason() == PiSupervisor::PI_SILENT);
}
