#pragma once
// Everything the OTA code needs from the board. On the VCU that's AbBoard
// (ab_board.hpp) over the Teensy's flash (src/ota_teensy.cpp); lib/vcu_sim
// runs the same AbBoard over simulated NOR flash.
#include <stddef.h>
#include <stdint.h>

namespace ota {

// Where a new image is downloaded: the A/B slot that isn't running. Offsets
// are relative to the start of the slot.
//
// NOR flash rules, which the simulator enforces: erase sets a whole sector
// to 0xFF, program can only clear bits, so anything not erased first comes
// out corrupted.
class StagingFlash {
public:
  virtual ~StagingFlash() = default;

  virtual size_t size() const = 0;
  virtual size_t sector_size() const = 0;

  virtual bool erase_sector(uint32_t offset) = 0;
  virtual bool program(uint32_t offset, const uint8_t *data, size_t len) = 0;
  virtual void read(uint32_t offset, uint8_t *out, size_t len) const = 0;
};

class Platform {
public:
  virtual ~Platform() = default;

  // Is it OK to start reprogramming right now? On the VCU: tractive system
  // off and the inverter disabled.
  virtual bool safe_to_program() = 0;

  // Can an update be downloaded at all? Not while the running firmware is
  // itself an unconfirmed update: its fallback is in the slot we'd overwrite.
  virtual bool can_update() = 0;

  // Boot the verified image in the staging slot next, as a trial, and reboot.
  // Doesn't return on hardware.
  virtual void activate_and_reboot() = 0;

  // Plain reboot, for ECUReset without a pending image
  virtual void reboot() = 0;

  // The running firmware works: keep booting it. Until this is called after
  // an update, the next reset goes back to the previous firmware.
  virtual bool confirm() = 0;

  // Version of the firmware that's running now, for ReadDataByIdentifier
  virtual uint32_t running_version() = 0;

  // Where a downloaded image's firmware must be linked to run (the staging
  // slot's). Images linked anywhere else are refused (WRONG_LOAD_ADDRESS).
  virtual uint32_t image_load_address() = 0;

  // For the boot status DID: running slot, active slot, trial slot (0xFF for
  // none), trial boots so far
  virtual void boot_status(uint8_t out[4]) = 0;
};

} // namespace ota
