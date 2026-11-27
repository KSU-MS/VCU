#pragma once
// Simulated flash and board for the OTA code.
#include <boot.hpp>
#include <functional>
#include <ota_platform.hpp>
#include <vector>

namespace sim {

// NOR flash: erase sets a sector to 0xFF, program can only clear bits. A
// program over bits that weren't erased silently corrupts them, exactly like
// the real chip; `unerased_writes` counts how often that happened.
//
// Serves as a staging area (ota::StagingFlash) or as the whole chip
// (boot::Flash); the methods are the same.
class NorFlash : public ota::StagingFlash, public boot::Flash {
public:
  explicit NorFlash(size_t size, size_t sector = 4096)
      : bytes(size, 0xFF), sector(sector) {}

  size_t size() const override { return bytes.size(); }
  size_t sector_size() const override { return sector; }

  bool erase_sector(uint32_t offset) override;
  bool program(uint32_t offset, const uint8_t *data, size_t len) override;
  void read(uint32_t offset, uint8_t *out, size_t len) const override;

  std::vector<uint8_t> bytes;
  size_t sector;

  size_t erases = 0;
  size_t unerased_writes = 0;

  // Fault injection: fail every program/erase once this many bytes have
  // been programmed (power lost mid-write: bytes before it landed, the rest
  // didn't)
  size_t fail_after_bytes = SIZE_MAX;
  size_t bytes_programmed = 0;

  // Power lost during the next erase: the sector is left half erased
  bool interrupt_next_erase = false;
};

// The "board": whatever firmware it's running, and what happened to it
class Board : public ota::Platform {
public:
  explicit Board(NorFlash &staging) : staging(staging) {}

  std::function<bool()> safe = [] { return true; };

  bool safe_to_program() override { return safe(); }
  void apply_and_reboot(uint32_t staging_offset, uint32_t size) override;
  void reboot() override { reboots++; }
  uint32_t running_version() override { return version; }
  uint32_t image_load_address() override { return load_address; }

  NorFlash &staging;

  // What's in program flash, and the version it reports
  std::vector<uint8_t> firmware;
  uint32_t version = 0x11111111;
  uint32_t load_address = 0x60000000;

  size_t reboots = 0;
  size_t applies = 0;
};

} // namespace sim
