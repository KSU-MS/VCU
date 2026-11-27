// The DTI HV500's frames (dti_hv500.cpp packs them by hand) against the
// 4wd_dti DBC's generated decoders and encoders, for every node: the DBC
// stays what defines them.
#include <sim_test_main.hpp>

#include <cmath>
#include <car.h>
#include <dti_hv500.hpp>
#include <motors.hpp>
#include <sim.hpp>
#include <vehicle.hpp>

namespace {

struct Bench {
  canMan inv{TEENSY_CAN2, INVERTER_CAN_BAUD_RATE};
  canMan daq{TEENSY_CAN3, DAQ_CAN_BAUD_RATE};
};

const can_message *last(uint32_t id) {
  return sim::bus(TEENSY_CAN2).last_sent(id);
}

} // namespace

TEST_CASE("these suites are built for 4wd_dti") {
  CHECK(std::string(vehicle::THIS.name) == "4wd_dti");
}

// The DBC's per-node functions, in node order
struct NodeFns {
  uint32_t set_ac_current, set_drive_enable, set_max_ac_current;
  int (*decode_ac_current)(const can_obj_car_h_t *, double *);
  int (*decode_drive_enable)(const can_obj_car_h_t *, uint8_t *);
  int (*decode_max_ac_current)(const can_obj_car_h_t *, double *);
  uint32_t erpm, ac_dc;
  int (*encode_erpm)(can_obj_car_h_t *, int32_t);
  int (*encode_voltage)(can_obj_car_h_t *, int16_t);
  int (*encode_ac)(can_obj_car_h_t *, double);
  int (*encode_dc)(can_obj_car_h_t *, double);
};

#define NODE(n, fl)                                                            \
  NodeFns {                                                                    \
    CAN_ID_INV_##fl##_SETACCURRENT, CAN_ID_INV_##fl##_SETDRIVEENABLE,          \
        CAN_ID_INV_##fl##_SETMAXACCURRENT,                                     \
        decode_can_0x10##n##_CMD_TargetAcCurrent,                              \
        decode_can_0xc0##n##_CMD_DriveEnable,                                  \
        decode_can_0x80##n##_CMD_MaxAcCurrent,                                 \
        CAN_ID_INV_##fl##_ERPM_DUTY_VOLTAGE, CAN_ID_INV_##fl##_AC_DC_CURRENT,  \
        encode_can_0x200##n##_Actual_ERPM,                                     \
        encode_can_0x200##n##_Actual_InputVoltage,                             \
        encode_can_0x210##n##_Actual_ACCurrent,                                \
        encode_can_0x210##n##_Actual_DCCurrent                                 \
  }

TEST_CASE("every node's frames match the DBC (x4 nodes)") {
  const NodeFns nodes[] = {NODE(1, FL), NODE(2, FR), NODE(3, RL),
                           NODE(4, RR)};
  int checked = 0;
  for (uint8_t node = 1; node <= 4; node++) {
    const NodeFns &f = nodes[node - 1];
    sim::reset();
    Bench b;
    DtiHv500 dti{node, motors::DTI_FMOT, vehicle::HUB_RATIO, &b.inv, &b.daq};
    CHECK(dti.id(DtiHv500::SET_AC_CURRENT) == f.set_ac_current);
    CHECK(dti.id(DtiHv500::SET_DRIVE_ENABLE) == f.set_drive_enable);
    CHECK(dti.id(DtiHv500::SET_MAX_AC_CURRENT) == f.set_max_ac_current);

    can_obj_car_h_t rx{};
    double amps;
    uint8_t enable;
    const can_message *m = last(f.set_max_ac_current);
    REQUIRE(m != nullptr);
    unpack_message(&rx, m->id, m->buf.val, m->length, 0);
    f.decode_max_ac_current(&rx, &amps);
    CHECK(amps == doctest::Approx(
                      dti.current_for_torque(motors::DTI_FMOT.peak_torque_nm))
                      .epsilon(0.002));

    dti.set_power_limit_kw(80);
    dti.set_torque_limit(motors::DTI_FMOT.peak_torque_nm);
    dti.set_inverter_enable(true);
    dti.command_torque(12.5);
    m = last(f.set_ac_current);
    REQUIRE(m != nullptr);
    CHECK(m->is_extended);
    unpack_message(&rx, m->id, m->buf.val, m->length, 0);
    f.decode_ac_current(&rx, &amps);
    CHECK(amps == doctest::Approx(dti.current_for_torque(12.5)).epsilon(0.002));
    m = last(f.set_drive_enable);
    REQUIRE(m != nullptr);
    unpack_message(&rx, m->id, m->buf.val, m->length, 0);
    f.decode_drive_enable(&rx, &enable);
    CHECK(enable == 1);

    // Feedback the DBC encodes, decoded by the class
    can_obj_car_h_t tx{};
    f.encode_erpm(&tx, 4 * 9000); // 9000 rpm at 4 pole pairs
    f.encode_voltage(&tx, 550);
    can_message fb;
    fb.id = f.erpm;
    fb.is_extended = true;
    fb.length = pack_message(&tx, f.erpm, &fb.buf.val);
    dti.on_frame(fb);
    CHECK(dti.get_motor_rpm() == 9000);
    CHECK(dti.get_bus_voltage() == doctest::Approx(550));
    f.encode_ac(&tx, 40.0);
    f.encode_dc(&tx, -7.5);
    fb.id = f.ac_dc;
    fb.length = pack_message(&tx, f.ac_dc, &fb.buf.val);
    dti.on_frame(fb);
    CHECK(dti.get_bus_current() == doctest::Approx(-7.5));
    CHECK(dti.get_motor_torque() ==
          doctest::Approx(40.0 / dti.current_for_torque(1.0)));
    checked++;
  }
  CHECK(checked == 4);
}
