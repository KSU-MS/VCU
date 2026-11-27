#pragma once
// A/B firmware slots, and the state that says which one to boot.
//
// The 8 MB flash, as offsets from 0x60000000:
//
//   0x000000  bootloader (64 KB): FlexSPI config, IVT, code
//   0x010000  boot state: an append-only journal over two 4 KB sectors
//   0x020000  slot A: a signed image (ota_image.hpp), firmware at +4 KB
//   0x200000  slot B: same
//   0x3E0000  free
//   0x7C0000  the Teensy core's EEPROM emulation, never touched
//
// An update is written to the slot that isn't running, then booted once as a
// trial. If the new firmware doesn't confirm itself before the next reset,
// the bootloader goes back to the old slot. Nothing is ever copied over the
// running firmware, so losing power at any point leaves something bootable.
#include <stddef.h>
#include <stdint.h>

#include "ota_image.hpp"

namespace boot {

constexpr uint32_t FLASH_BASE = 0x60000000;
constexpr uint32_t SECTOR_SIZE = 4096;

constexpr uint32_t BOOTLOADER_OFFSET = 0x000000;
constexpr uint32_t BOOTLOADER_SIZE = 0x010000;
constexpr uint32_t STATE_OFFSET = 0x010000;
constexpr uint32_t STATE_SECTORS = 2;
constexpr uint32_t SLOT_SIZE = 0x1E0000;
constexpr uint32_t EEPROM_OFFSET = 0x7C0000;

constexpr uint8_t SLOT_A = 0;
constexpr uint8_t SLOT_B = 1;
constexpr uint8_t NO_SLOT = 0xFF;
constexpr uint32_t SLOT_OFFSET[2] = {0x020000, 0x200000};

static_assert(STATE_OFFSET >= BOOTLOADER_OFFSET + BOOTLOADER_SIZE, "layout");
static_assert(SLOT_OFFSET[0] >= STATE_OFFSET + STATE_SECTORS * SECTOR_SIZE, "layout");
static_assert(SLOT_OFFSET[1] >= SLOT_OFFSET[0] + SLOT_SIZE, "layout");
static_assert(SLOT_OFFSET[1] + SLOT_SIZE <= EEPROM_OFFSET, "layout");

constexpr uint8_t other_slot(uint8_t slot) { return slot == SLOT_A ? SLOT_B : SLOT_A; }

// Where a slot's firmware runs from, which its image must be linked for
constexpr uint32_t slot_firmware_address(uint8_t slot) {
  return FLASH_BASE + SLOT_OFFSET[slot] + ota::IMAGE_FIRMWARE_OFFSET;
}

// The most firmware a slot holds
constexpr size_t SLOT_FIRMWARE_MAX = SLOT_SIZE - ota::IMAGE_FIRMWARE_OFFSET;

// The flash chip, addressed by offset from FLASH_BASE. NOR rules: erase sets
// a whole sector to 0xFF, program can only clear bits.
class Flash {
public:
  virtual void read(uint32_t offset, uint8_t *out, size_t len) const = 0;
  virtual bool erase_sector(uint32_t offset) = 0;
  virtual bool program(uint32_t offset, const uint8_t *data, size_t len) = 0;

protected:
  ~Flash() = default;
};

// Full check of what's in a slot: header, linked for that slot, CRC and
// signature against `public_key`
ota::ImageError verify_slot(const Flash &flash, uint8_t slot,
                            const uint8_t public_key[32],
                            ota::ImageHeader *out = nullptr);

//
//// Boot state

// How many times a trial slot gets booted before it counts as failed
constexpr uint8_t TRIAL_BOOTS = 1;

// One journal entry. Written once into erased flash, never modified; the
// valid one with the highest sequence number is the current state.
struct BootRecord {
  uint32_t magic;
  uint32_t sequence;
  uint8_t active;   // the slot known to work
  uint8_t trial;    // a slot being tried, or NO_SLOT
  uint8_t attempts; // times the trial slot has been booted
  uint8_t reserved0;
  uint32_t reserved[4];
  uint32_t crc; // over the 28 bytes before this
};
static_assert(sizeof(BootRecord) == 32, "record layout");

constexpr uint32_t RECORD_MAGIC = 0x5453424B; // "KBST"
constexpr uint32_t RECORDS_PER_SECTOR = SECTOR_SIZE / sizeof(BootRecord);

// Losing power while a record is written leaves a partly-programmed slot
// that fails its CRC and is skipped, so the state is always either the old
// one or the new one. A sector fills up after 128 writes, then the other
// one is erased and used; the full one keeps the latest state until the
// first record lands in the new one.
class BootState {
public:
  explicit BootState(Flash &flash) : flash(flash) {}

  // Read the journal. False if there's no valid record (a board just
  // flashed over USB): current() then says slot A, no trial.
  bool load();

  const BootRecord &current() const { return latest; }
  bool has_record() const { return found; }

  // Append a new state. False if the flash wouldn't take it.
  bool write(uint8_t active, uint8_t trial, uint8_t attempts);

private:
  Flash &flash;
  BootRecord latest{};
  bool found = false;
  uint32_t sector = 0;    // where `latest` is (or where the next write goes)
  uint32_t next_free = 0; // record index in that sector

  uint32_t record_offset(uint32_t sector, uint32_t index) const {
    return STATE_OFFSET + sector * SECTOR_SIZE + index * sizeof(BootRecord);
  }
};

//
//// What to boot

struct BootDecision {
  uint8_t slot = NO_SLOT; // NO_SLOT: nothing bootable
  bool trial = false;     // it's an unconfirmed update

  // A new state to record before booting it
  bool write = false;
  uint8_t active = SLOT_A;
  uint8_t trial_slot = NO_SLOT;
  uint8_t attempts = 0;
};

// The bootloader's whole policy, given the current state and which slots
// hold a good image:
//
//   - a trial slot gets TRIAL_BOOTS boots; after that (or if it's not
//     valid), it's abandoned and the active slot boots: rollback
//   - the active slot boots if it's valid, else the other one if that is
//     (and becomes active), else nothing
BootDecision decide(const BootRecord &state, bool has_record,
                    const bool valid[2]);

//
//// What the running firmware does

// After downloading and verifying an update into `new_slot`: boot it once
// as a trial on the next reset
bool start_trial(BootState &state, uint8_t running_slot, uint8_t new_slot);

// The running firmware is good: make it the active slot. Fine to call when
// it already is.
bool confirm(BootState &state, uint8_t running_slot);

// Running an update that hasn't been confirmed yet. The other slot holds the
// fallback, so it mustn't be overwritten by another update until then.
bool running_trial(const BootState &state, uint8_t running_slot);

} // namespace boot
