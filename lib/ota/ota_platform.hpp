#pragma once
// Everything the OTA code needs from the board. The Teensy implementation is
// src/ota_teensy.cpp (on top of FlasherX); lib/vcu_sim has a simulated one
// that enforces NOR flash rules so the logic above can be tested properly.
#include <stddef.h>
#include <stdint.h>

namespace ota {

// The staging area a new image is downloaded into before it's applied.
// Offsets are relative to the start of the staging area.
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

  // Copy `size` bytes starting at staging offset `staging_offset` over the
  // running firmware and reboot into it. Doesn't return on hardware; the
  // image has already been verified by the time this is called.
  virtual void apply_and_reboot(uint32_t staging_offset, uint32_t size) = 0;

  // Plain reboot, for ECUReset without a pending image
  virtual void reboot() = 0;

  // Version of the firmware that's running now, for ReadDataByIdentifier
  virtual uint32_t running_version() = 0;

  // Where a downloaded image's firmware must be linked to run. Images
  // linked anywhere else are refused (WRONG_LOAD_ADDRESS).
  virtual uint32_t image_load_address() = 0;
};

} // namespace ota
