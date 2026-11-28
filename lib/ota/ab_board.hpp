#pragma once
// The OTA platform for A/B slots (lib/vcu_boot/boot.hpp), on any flash: the
// update goes into the slot that isn't running, ECUReset asks the bootloader
// to try it once, and the tester confirms it once it answers. What's left to
// the board is the flash itself, rebooting, and whether the car is parked.
#include <boot.hpp>

#include "ota_platform.hpp"

namespace ota {

// One slot of the flash as a staging area
class SlotStaging : public StagingFlash {
public:
  SlotStaging(boot::Flash &flash, uint8_t slot)
      : flash(flash), base(boot::SLOT_OFFSET[slot]) {}

  void set_slot(uint8_t slot) { base = boot::SLOT_OFFSET[slot]; }

  size_t size() const override { return boot::SLOT_SIZE; }
  size_t sector_size() const override { return boot::SECTOR_SIZE; }
  bool erase_sector(uint32_t offset) override;
  bool program(uint32_t offset, const uint8_t *data, size_t len) override;
  void read(uint32_t offset, uint8_t *out, size_t len) const override;

private:
  boot::Flash &flash;
  uint32_t base;
};

class AbBoard : public Platform {
public:
  AbBoard(boot::Flash &flash, uint8_t running_slot, uint32_t version)
      : flash(flash), running(running_slot), version(version),
        staging_slot(flash, boot::other_slot(running_slot)) {}

  // Where the update is downloaded to
  StagingFlash &staging() { return staging_slot; }
  uint8_t running_slot() const { return running; }

  bool can_update() override;
  void activate_and_reboot() override;
  bool confirm() override;
  uint32_t running_version() override { return version; }
  uint32_t image_load_address() override;
  void boot_status(uint8_t out[4]) override;

  // Still up to the board: safe_to_program(), reboot()

protected:
  boot::Flash &flash;
  uint8_t running;
  uint32_t version;

  // For a simulated board that "reboots" in place
  void set_running(uint8_t slot, uint32_t fw_version) {
    running = slot;
    version = fw_version;
    staging_slot.set_slot(boot::other_slot(slot));
  }

private:
  SlotStaging staging_slot;
};

} // namespace ota
