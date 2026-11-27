#include "pi_supervisor.hpp"
#include "parameters.hpp"
#include <car.h>

namespace pi_frame {

uint8_t crc8(uint32_t id, const uint8_t *data, size_t len) {
  // SAE J1850: poly 0x1D, init 0xFF, final xor 0xFF (AUTOSAR's CRC8). The
  // ID goes in so a frame meant for another ID can't pass
  uint8_t crc = 0xFF;
  auto feed = [&crc](uint8_t byte) {
    crc ^= byte;
    for (int i = 0; i < 8; i++)
      crc = uint8_t(crc & 0x80 ? (crc << 1) ^ 0x1D : crc << 1);
  };
  feed(uint8_t(id));
  feed(uint8_t(id >> 8));
  for (size_t i = 0; i < len; i++)
    feed(data[i]);
  return uint8_t(crc ^ 0xFF);
}

void encode(uint8_t out[8], double ceiling_nm, bool engage, uint8_t counter) {
  const double raw = ceiling_nm * 10 + 0.5;
  const uint16_t c = raw <= 0 ? 0 : raw >= 65535 ? 65535 : uint16_t(raw);
  out[0] = uint8_t(c);
  out[1] = uint8_t(c >> 8);
  out[2] = engage ? 1 : 0;
  out[3] = out[4] = out[5] = 0;
  out[6] = counter & 0x0F;
  out[7] = crc8(CAN_ID_PI_TORQUE_COMMAND, out, 7);
}

} // namespace pi_frame

void PiSupervisor::reject(reason r) {
  why = r;
  healthy_run = false; // drops control in step(), and restarts the clock
  if (rejected < 255)
    rejected++;
}

void PiSupervisor::on_frame(const uint8_t *d, uint8_t len, uint32_t now_ms) {
  if (len != 8)
    return reject(PI_BAD_VALUE);
  if (pi_frame::crc8(CAN_ID_PI_TORQUE_COMMAND, d, 7) != d[7])
    return reject(PI_BAD_CHECKSUM);

  // Up to two frames lost is fine; the same counter again (stuck, or a
  // repeat) or a jump isn't. Either way the next one is judged from here
  const uint8_t counter = d[6] & 0x0F;
  const uint8_t ahead = uint8_t((counter - last_counter) & 0x0F);
  const bool counter_ok = !have_counter || (ahead >= 1 && ahead <= 3);
  have_counter = true;
  last_counter = counter;

  const uint16_t raw = uint16_t(d[0] | d[1] << 8);
  if ((d[2] & ~1u) || d[3] || d[4] || d[5] || (d[6] & 0xF0) ||
      raw > MAX_TORQUE_LIMIT_NM * 10)
    return reject(PI_BAD_VALUE);
  if (!counter_ok)
    return reject(PI_BAD_COUNTER);

  heard = true;
  last_valid_ms = now_ms;
  pi_ceiling = raw * 0.1;
  engage = d[2] & 1;
  if (!healthy_run) {
    healthy_run = true;
    healthy_since_ms = now_ms;
  }
}

bool PiSupervisor::healthy(uint32_t now_ms) const {
  return heard && healthy_run && engage &&
         now_ms - last_valid_ms <= PI_COMMAND_TIMEOUT_MS;
}

double PiSupervisor::step(uint32_t now_ms, double local_nm, bool off_throttle) {
  const uint32_t dt_ms = now_ms - last_step_ms;
  last_step_ms = now_ms;
  if (!(local_nm > 0))
    local_nm = 0;

  if (current == PI_CONTROL && !healthy(now_ms)) {
    current = LOCAL_CONTROL;
    ramping = true;
    ramp_nm = pi_ceiling;
  }

  if (current == LOCAL_CONTROL) {
    if (!heard || now_ms - last_valid_ms > PI_COMMAND_TIMEOUT_MS)
      why = PI_SILENT;
    else if (!healthy_run)
      ; // still the reason it was rejected
    else if (!engage)
      why = PI_NOT_REQUESTED;
    else if (now_ms - healthy_since_ms < PI_HEALTHY_MS)
      why = PI_SETTLING;
    else if (!off_throttle)
      why = PI_WAITING_FOR_LIFT;
    else {
      current = PI_CONTROL;
      why = PI_OK;
      ramping = false;
    }
  }

  if (current == PI_CONTROL)
    return local_nm < pi_ceiling ? local_nm : pi_ceiling;

  if (ramping) {
    ramp_nm += PI_FALLBACK_RAMP_NM_PER_S * dt_ms / 1000.0;
    // Done once it's caught up with anything the driver could ask for, or
    // the driver has lifted and there's no step left to soften
    if (off_throttle || ramp_nm >= MAX_TORQUE_LIMIT_NM)
      ramping = false;
    else if (local_nm > ramp_nm)
      return ramp_nm;
  }
  return local_nm;
}

double PiSupervisor::ceiling() const {
  if (current == PI_CONTROL)
    return pi_ceiling;
  return ramping ? ramp_nm : -1;
}
