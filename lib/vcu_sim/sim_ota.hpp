#pragma once
// Simulated flash and board for the OTA code.
#include <ab_board.hpp>
#include <boot.hpp>
#include <functional>
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

// The board: the same A/B OTA code as the VCU, on simulated flash. A reboot
// runs the bootloader's policy (boot::decide) and carries on as whatever
// firmware it picked, in place: the VCU's own state isn't reset, the tests
// only look at which firmware is running.
class Board : public ota::AbBoard {
public:
  explicit Board(NorFlash &chip) : ota::AbBoard(chip, boot::SLOT_A, 0), chip(chip) {}

  std::function<bool()> safe = [] { return true; };

  bool safe_to_program() override { return safe(); }
  uint8_t reset_cause() override { return last_reset_cause; }
  uint8_t last_reset_cause = RESET_CAUSE_OTHER;
  void activate_and_reboot() override;
  void reboot() override;

  // Put a signed image of `firmware` built for `slot` there, like a USB
  // flash or an earlier update would have
  void install(uint8_t slot, const std::vector<uint8_t> &firmware,
               uint32_t version);

  // What the bootloader does on reset
  void power_on();

  // Carry on as this firmware without running the bootloader (the SIL
  // making a fresh Car for a firmware that has already booted)
  void resume(uint8_t slot, uint32_t fw_version) { set_running(slot, fw_version); }

  // The firmware in the slot that's running
  std::vector<uint8_t> firmware() const;

  NorFlash &chip;
  bool trial = false;  // running an update that isn't confirmed yet
  bool bricked = false; // the bootloader found nothing to boot

  size_t reboots = 0;
  size_t activations = 0;
};

} // namespace sim
