#include "sim_car.hpp"
#include <algorithm>
#include <monocypher-ed25519.h>
#include <string.h>

namespace sim {

//
//// Tester

void Car::tester_inject(const uint8_t *bytes, size_t len) {
  can_message m;
  m.id = ota::REQUEST_ID;
  m.length = 8;
  memset(m.buf.byte, 0xAA, 8);
  memcpy(m.buf.byte, bytes, len);
  daq().inject(m);
}

// Next frame from the VCU on the response ID after `cursor`, running the
// car until one shows up
bool Car::tester_wait_frame(size_t *cursor, can_message *out,
                            uint32_t timeout_ms) {
  uint32_t deadline = sim::now() + timeout_ms;
  for (;;) {
    auto &tx = daq().tx;
    for (; *cursor < tx.size(); (*cursor)++) {
      if (tx[*cursor].id == ota::RESPONSE_ID) {
        *out = tx[(*cursor)++];
        return true;
      }
    }
    if (sim::now() >= deadline)
      return false;
    tick();
    sim::advance(1);
  }
}

void Car::tester_send(const std::vector<uint8_t> &req, size_t *cursor,
                      uint32_t timeout_ms) {
  if (req.size() <= 7) {
    uint8_t sf[8] = {uint8_t(req.size())};
    memcpy(sf + 1, req.data(), req.size());
    tester_inject(sf, req.size() + 1);
    return;
  }

  uint8_t ff[8] = {uint8_t(0x10 | (req.size() >> 8)), uint8_t(req.size())};
  memcpy(ff + 2, req.data(), 6);
  tester_inject(ff, 8);

  size_t pos = 6;
  uint8_t sn = 1;
  while (pos < req.size()) {
    // Wait for flow control, then send as many CFs as it allows
    can_message fc;
    if (!tester_wait_frame(cursor, &fc, timeout_ms))
      return;
    if ((fc.buf.byte[0] >> 4) != 0x3 || (fc.buf.byte[0] & 0xF) != 0)
      return;
    uint8_t block_size = fc.buf.byte[1];

    for (int n = 0; pos < req.size() && (block_size == 0 || n < block_size);
         n++) {
      uint8_t cf[8] = {uint8_t(0x20 | sn)};
      size_t chunk = req.size() - pos < 7 ? req.size() - pos : 7;
      memcpy(cf + 1, req.data() + pos, chunk);
      tester_inject(cf, chunk + 1);
      pos += chunk;
      sn = (sn + 1) & 0xF;
    }
  }
}

std::vector<uint8_t> Car::uds(const std::vector<uint8_t> &req,
                              uint32_t timeout_ms) {
  size_t cursor = daq().tx.size();
  tester_send(req, &cursor, timeout_ms);

  can_message m;
  if (!tester_wait_frame(&cursor, &m, timeout_ms))
    return {};

  const uint8_t *b = m.buf.byte;
  if ((b[0] >> 4) == 0x0) {
    return std::vector<uint8_t>(b + 1, b + 1 + (b[0] & 0xF));
  }
  if ((b[0] >> 4) != 0x1)
    return {};

  // Multi-frame response: ask for the rest
  size_t len = (size_t(b[0] & 0xF) << 8) | b[1];
  std::vector<uint8_t> out(b + 2, b + 8);
  uint8_t fc[3] = {0x30, 0, 0};
  tester_inject(fc, 3);
  while (out.size() < len) {
    if (!tester_wait_frame(&cursor, &m, timeout_ms))
      return {};
    size_t chunk = len - out.size() < 7 ? len - out.size() : 7;
    out.insert(out.end(), m.buf.byte + 1, m.buf.byte + 1 + chunk);
  }
  return out;
}

const char *Car::flash(const std::vector<uint8_t> &image) {
  auto ok = [](const std::vector<uint8_t> &r, uint8_t sid) {
    return !r.empty() && r[0] == sid + 0x40;
  };

  if (!ok(uds({0x10, 0x02}), 0x10))
    return "programming session";

  uint32_t n = image.size();
  std::vector<uint8_t> req = {0x34, 0x00, 0x44, 0, 0, 0, 0,
                              uint8_t(n >> 24), uint8_t(n >> 16),
                              uint8_t(n >> 8), uint8_t(n)};
  auto r = uds(req);
  if (!ok(r, 0x34))
    return "request download";
  size_t block_data = ((size_t(r[2]) << 8) | r[3]) - 2;

  uint8_t counter = 1;
  for (size_t off = 0; off < image.size(); off += block_data) {
    size_t chunk = std::min(block_data, image.size() - off);
    std::vector<uint8_t> td = {0x36, counter};
    td.insert(td.end(), image.begin() + off, image.begin() + off + chunk);
    if (!ok(uds(td), 0x36))
      return "transfer data";
    counter++;
  }

  if (!ok(uds({0x37}), 0x37))
    return "transfer exit";

  r = uds({0x31, 0x01, 0xFF, 0x01});
  if (!ok(r, 0x31) || r.size() < 5 || r[4] != 0)
    return "check image";

  if (!ok(uds({0x11, 0x01}), 0x11))
    return "ecu reset";
  run_for(10); // let the reset happen after the response goes out
  return nullptr;
}

const uint8_t DEV_SIGNING_SEED[32] = {
    0xe2, 0x8e, 0x3e, 0xf6, 0x00, 0x5f, 0xfa, 0xce, 0x36, 0xdb, 0xcb,
    0x4d, 0x5e, 0xc9, 0xdb, 0xf8, 0x36, 0x03, 0xa0, 0x25, 0x86, 0x06,
    0x38, 0x1c, 0x3d, 0x7c, 0xcd, 0x9e, 0x25, 0xc5, 0x7b, 0xe6};

std::vector<uint8_t> make_image(const std::vector<uint8_t> &firmware,
                                uint32_t version, const char *target,
                                uint32_t load_address, const uint8_t *seed) {
  ota::ImageHeader h{};
  memcpy(h.magic, ota::IMAGE_MAGIC, 4);
  h.header_version = ota::IMAGE_HEADER_VERSION;
  h.header_size = ota::IMAGE_HEADER_SIZE;
  h.image_size = firmware.size();
  h.image_crc32 = ota::crc32(firmware.data(), firmware.size());
  h.fw_version = version;
  strncpy(h.target, target, ota::IMAGE_TARGET_LEN);
  h.load_address = load_address;
  h.header_crc32 = ota::crc32(reinterpret_cast<const uint8_t *>(&h),
                              offsetof(ota::ImageHeader, header_crc32));

  std::vector<uint8_t> out(ota::IMAGE_FIRMWARE_OFFSET, 0xFF);
  memcpy(out.data(), &h, sizeof(h));
  out.insert(out.end(), firmware.begin(), firmware.end());

  // Ed25519 over SHA-512(header || firmware)
  uint8_t digest[64];
  crypto_sha512_ctx sha;
  crypto_sha512_init(&sha);
  crypto_sha512_update(&sha, out.data(), sizeof(h));
  crypto_sha512_update(&sha, firmware.data(), firmware.size());
  crypto_sha512_final(&sha, digest);

  uint8_t seed_copy[32], secret[64], pub[32]; // key_pair wipes the seed
  memcpy(seed_copy, seed, sizeof(seed_copy));
  crypto_ed25519_key_pair(secret, pub, seed_copy);
  crypto_ed25519_sign(out.data() + ota::IMAGE_SIGNATURE_OFFSET, secret, digest,
                      sizeof(digest));
  return out;
}

void Car::send(Bus &b, uint32_t id) {
  can_message msg;
  msg.id = id;
  msg.length = pack_message(&other_boards, id, &msg.buf.val);
  b.inject(msg);

  switch (id) {
  case CAN_ID_ACU_SHUTDOWN_STATUS:
  case CAN_ID_PRECHARGE_STATUS:
  case CAN_ID_MSGID_0X6B1:
  case CAN_ID_M167_VOLTAGE_INFO:
    if (std::find(repeating.begin(), repeating.end(), id) == repeating.end())
      repeating.push_back(id);
    break;
  }
}

void Car::repeat_status() {
  if (sim::now() - last_repeat_ms < 100)
    return;
  last_repeat_ms = sim::now();
  for (uint32_t id : std::vector<uint32_t>(repeating)) {
    if (id == CAN_ID_M167_VOLTAGE_INFO) {
      if (inverter_talking)
        send(inv(), id);
    } else if (acu_talking) {
      send(acc(), id);
    }
  }
}

void Car::acu_status(bool imd_ok, bool bms_ok) {
  encode_can_0x06a_acu_imd_relay_state(&other_boards, imd_ok);
  encode_can_0x06a_acu_bms_relay_state(&other_boards, bms_ok);
  send(acc(), CAN_ID_ACU_SHUTDOWN_STATUS);
}

void Car::precharge_state(uint8_t state) {
  encode_can_0x069_precharge_state(&other_boards, state);
  send(acc(), CAN_ID_PRECHARGE_STATUS);
}

void Car::pack_power(double volts, double amps) {
  encode_can_0x6b1_Pack_Summed_Voltage(&other_boards, volts);
  encode_can_0x6b1_Pack_Current(&other_boards, amps);
  send(acc(), CAN_ID_MSGID_0X6B1);
}

void Car::inverter_bus_voltage(double volts) {
  encode_can_0x0a7_INV_DC_Bus_Voltage(&other_boards, volts);
  send(inv(), CAN_ID_M167_VOLTAGE_INFO);
}

void Car::inverter_motor_rpm(int16_t rpm) {
  encode_can_0x0a5_INV_Motor_Speed(&other_boards, rpm);
  send(inv(), CAN_ID_M165_MOTOR_POSITION_INFO);
}

void Car::dash_rtd_button(bool pressed) {
  encode_can_0x0eb_dash_button5status(&other_boards, pressed);
  send(inv(), CAN_ID_DASH_BUTTONS);
}

void Car::set_parameter(uint8_t target, uint32_t value) {
  encode_can_0x0c9_vcu_target_parameter(&other_boards, target);
  encode_can_0x0c9_vcu_parameter_value(&other_boards, value);
  send(inv(), CAN_ID_VCU_SET_PARAMETER);
}

void Car::drive_to_rtd() {
  run_for(10); // STARTUP -> TS_DISABLED
  make_ts_healthy();
  run_for(10); // -> TS_ENERGIZED
  set_brake(0.5);
  dash_rtd_button(true);
  run_until(READY_TO_DRIVE, 3000); // -> TS_ENABLED, buzzer, -> READY_TO_DRIVE
  dash_rtd_button(false);
  set_brake(0);
  run_for(10);
}

bool Car::last_inverter_command(InverterCommand *out) {
  const can_message *m = inv().last_sent(CAN_ID_M192_COMMAND_MESSAGE);
  if (!m)
    return false;

  can_obj_car_h_t rx{};
  unpack_message(&rx, m->id, m->buf.val, m->length, 0);

  uint8_t enable, direction;
  decode_can_0x0c0_VCU_INV_Torque_Command(&rx, &out->torque_nm);
  decode_can_0x0c0_VCU_INV_Torque_Limit_Command(&rx, &out->torque_limit_nm);
  decode_can_0x0c0_VCU_INV_Inverter_Enable(&rx, &enable);
  decode_can_0x0c0_VCU_INV_Direction_Command(&rx, &direction);
  out->enable = enable;
  out->direction = direction;
  return true;
}

int Car::last_reported_state() {
  const can_message *m = daq().last_sent(CAN_ID_VCU_STATUS);
  if (!m)
    return -1;

  can_obj_car_h_t rx{};
  unpack_message(&rx, m->id, m->buf.val, m->length, 0);

  uint8_t state;
  decode_can_0x0c3_vcu_statemachine_state(&rx, &state);
  return state;
}

int Car::last_discharge_limit() {
  const can_message *m = inv().last_sent(CAN_ID_BMS_CURRENT_LIMIT);
  if (!m)
    return -1;

  can_obj_car_h_t rx{};
  unpack_message(&rx, m->id, m->buf.val, m->length, 0);

  uint16_t limit;
  decode_can_0x202_BMS_Max_Discharge_Current(&rx, &limit);
  return limit;
}

} // namespace sim
