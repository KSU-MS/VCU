// The DTI HV500 inverter on its own (dti_hv500.hpp): the frames it sends,
// the feedback it decodes, and torque as AC current. Its frames against the
// DBC's own decoders, for every node, are test/4wd_dti's.
#include <sim_test_main.hpp>

#include <cmath>
#include <dti_hv500.hpp>
#include <motors.hpp>
#include <sim_car.hpp>

using sim::Car;

namespace {

constexpr uint8_t NODE = 3;

struct Bench {
  Car car; // for the buses
  DtiHv500 dti{NODE, motors::DTI_FMOT, 13.0, &car.inv_can, &car.daq_can};

  const can_message *last(DtiHv500::packet p) {
    return sim::bus(TEENSY_CAN2).last_sent(dti.id(p));
  }
  int16_t last_i16(DtiHv500::packet p) {
    const can_message *m = last(p);
    REQUIRE(m != nullptr);
    return int16_t(m->buf.byte[0] << 8 | m->buf.byte[1]);
  }
  void feedback(DtiHv500::packet p, const uint8_t (&d)[8]) {
    can_message m;
    m.id = dti.id(p);
    m.is_extended = true;
    for (int i = 0; i < 8; i++)
      m.buf.byte[i] = d[i];
    dti.on_frame(m);
  }
};

void enable(DtiHv500 &dti) {
  dti.set_power_limit_kw(80);
  dti.set_torque_limit(motors::DTI_FMOT.peak_torque_nm);
  dti.set_inverter_enable(true);
}

} // namespace

TEST_CASE("its frames are extended, packet << 8 | node") {
  Bench b;
  CHECK(b.dti.id(DtiHv500::SET_AC_CURRENT) == 0x103);
  CHECK(b.dti.id(DtiHv500::SET_DRIVE_ENABLE) == 0xC03);
  const can_message *m = b.last(DtiHv500::SET_DRIVE_ENABLE);
  REQUIRE(m != nullptr);
  CHECK(m->is_extended);
  CHECK(m->length == 2);
}

TEST_CASE("at start it caps the AC current at the motor's peak, and sits "
          "disabled at no current") {
  Bench b;
  // 31.6 N m / 0.492 N m/A rms = 64.2 A rms = 90.8 A peak, in 0.1 A
  CHECK(b.last_i16(DtiHv500::SET_MAX_AC_CURRENT) == 908);
  CHECK(b.last(DtiHv500::SET_DRIVE_ENABLE)->buf.byte[0] == 0);
  CHECK(b.last_i16(DtiHv500::SET_AC_CURRENT) == 0);
}

TEST_CASE("torque goes out as AC current, A peak (the assumption)") {
  Bench b;
  enable(b.dti);
  b.dti.command_torque(10);
  // sqrt(2) x 10 / 0.492 = 28.74 A peak
  CHECK(b.last_i16(DtiHv500::SET_AC_CURRENT) == 287);
  CHECK(b.last(DtiHv500::SET_DRIVE_ENABLE)->buf.byte[0] == 1);
  CHECK(b.dti.current_for_torque(10) == doctest::Approx(28.74).epsilon(0.001));
}

TEST_CASE("never more than the torque limit, and nothing when disabled") {
  Bench b;
  enable(b.dti);
  b.dti.command_torque(100);
  CHECK(b.last_i16(DtiHv500::SET_AC_CURRENT) == 908);
  b.dti.set_inverter_enable(false);
  b.dti.command_torque(10);
  CHECK(b.last_i16(DtiHv500::SET_AC_CURRENT) == 0);
  CHECK(b.last(DtiHv500::SET_DRIVE_ENABLE)->buf.byte[0] == 0);
}

TEST_CASE("feedback: speed from ERPM and pole pairs, DC voltage and current, "
          "torque from AC current") {
  Bench b;
  // 40000 ERPM / 4 pole pairs = 10000 rpm; 600 V
  b.feedback(DtiHv500::ERPM_DUTY_VOLTAGE,
             {0x00, 0x00, 0x9C, 0x40, 0x00, 0x00, 0x02, 0x58});
  CHECK(b.dti.get_motor_rpm() == 10000);
  CHECK(b.dti.get_bus_voltage() == doctest::Approx(600));
  // 28.7 A peak AC, -12.5 A DC (regen)
  b.feedback(DtiHv500::AC_DC_CURRENT,
             {0x01, 0x1F, 0xFF, 0x83, 0x00, 0x00, 0x00, 0x00});
  CHECK(b.dti.get_bus_current() == doctest::Approx(-12.5));
  CHECK(b.dti.get_motor_torque() == doctest::Approx(9.98).epsilon(0.01));
  // 45.5 C controller, 80.0 C motor, fault 3
  b.feedback(DtiHv500::TEMPERATURES,
             {0x01, 0xC7, 0x03, 0x20, 0x03, 0x00, 0x00, 0x00});
  CHECK(b.dti.controller_temp_c() == doctest::Approx(45.5));
  CHECK(b.dti.motor_temp_c() == doctest::Approx(80.0));
  CHECK(b.dti.fault_code() == 3);
}

TEST_CASE("another node's frames, and standard frames with its numbers, "
          "aren't its") {
  Bench b;
  can_message m;
  m.id = (DtiHv500::ERPM_DUTY_VOLTAGE << 8) | (NODE + 1);
  m.is_extended = true;
  CHECK(b.dti.on_frame(m) == Inverter::NOT_MINE);
  m.id = b.dti.id(DtiHv500::ERPM_DUTY_VOLTAGE);
  m.is_extended = false;
  CHECK(b.dti.on_frame(m) == Inverter::NOT_MINE);
  m.is_extended = true;
  CHECK(b.dti.on_frame(m) != Inverter::NOT_MINE);
}

TEST_CASE("its power limit works on the hub motor's speed") {
  Bench b;
  enable(b.dti);
  b.dti.set_power_limit_kw(20); // its share of 80 kW
  // 10000 rpm: 20 kW x 0.90 / 1047 rad/s = 17.2 N m
  b.feedback(DtiHv500::ERPM_DUTY_VOLTAGE,
             {0x00, 0x00, 0x9C, 0x40, 0x00, 0x00, 0x02, 0x58});
  b.dti.command_torque(30);
  CHECK(b.last_i16(DtiHv500::SET_AC_CURRENT) ==
        std::lround(10 * b.dti.current_for_torque(20000 * 0.9 /
                                                  (10000 * 2 * M_PI / 60))));
}
