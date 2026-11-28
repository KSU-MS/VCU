#pragma once
// The Raspberry Pi as the controls brain, the VCU as the safety authority
// (docs/pi-supervisor.md). The Pi sends a torque *ceiling*; the VCU commands
// min(what it would have sent anyway, the ceiling). So a broken Pi can cost
// torque but never add any.
//
// Every frame from the Pi is checked (length, CRC, rolling counter, values).
// A bad one, or none for PI_COMMAND_TIMEOUT_MS, drops the VCU to local
// control at once, ramping the torque back up from the Pi's last ceiling
// rather than stepping. Control goes back to the Pi only after it's been
// healthy for PI_HEALTHY_MS and the driver has lifted.
#include <stddef.h>
#include <stdint.h>

// The Pi's command frame (CAN_ID_PI_TORQUE_COMMAND), little endian:
//   0-1  torque ceiling, 0.1 Nm
//   2    bit 0: the Pi wants control
//   3-5  reserved, 0
//   6    rolling counter, low 4 bits (high bits 0)
//   7    CRC-8 SAE J1850 over the frame ID (2 bytes, LE) and bytes 0-6
namespace pi_frame {
uint8_t crc8(uint32_t id, const uint8_t *data, size_t len);
// Fill in a command frame: for the sim, tests and tools
void encode(uint8_t out[8], double ceiling_nm, bool engage, uint8_t counter);
} // namespace pi_frame

class PiSupervisor {
public:
  enum mode : uint8_t { LOCAL_CONTROL = 0, PI_CONTROL = 1 };

  // Why it's local (status frame). The latest problem, kept until fixed
  enum reason : uint8_t {
    PI_OK = 0,
    PI_SILENT = 1,         // nothing heard, or not for PI_COMMAND_TIMEOUT_MS
    PI_NOT_REQUESTED = 2,  // the Pi isn't asking for control
    PI_BAD_CHECKSUM = 3,
    PI_BAD_COUNTER = 4,    // repeated, out of order or too many lost
    PI_BAD_VALUE = 5,      // wrong length, reserved bits set, ceiling out of range
    PI_SETTLING = 6,       // healthy, but not for PI_HEALTHY_MS yet
    PI_WAITING_FOR_LIFT = 7,
  };

  // A frame with the Pi's command ID arrived
  void on_frame(const uint8_t *data, uint8_t len, uint32_t now_ms);

  // Every pass of the loop. `local_nm` is what the VCU would command
  // without the Pi (after its own traction control); returns what to send.
  double step(uint32_t now_ms, double local_nm, bool off_throttle);

  mode get_mode() const { return current; }
  reason get_reason() const { return why; }
  uint8_t rejected_frames() const { return rejected; }
  // The ceiling in force now: the Pi's, or the fallback ramp's. Negative
  // for none
  double ceiling() const;

private:
  void reject(reason r);
  bool healthy(uint32_t now_ms) const;

  mode current = LOCAL_CONTROL;
  reason why = PI_SILENT;

  bool heard = false;
  uint32_t last_valid_ms = 0;
  bool healthy_run = false; // valid frames since healthy_since, none bad
  uint32_t healthy_since_ms = 0;
  bool have_counter = false;
  uint8_t last_counter = 0;
  double pi_ceiling = 0;
  bool engage = false;
  uint8_t rejected = 0;

  // Fallback: the ceiling climbs from the Pi's last one to nothing
  bool ramping = false;
  double ramp_nm = 0;
  uint32_t last_step_ms = 0;
};
