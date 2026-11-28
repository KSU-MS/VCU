#include "boot.hpp"
#include <string.h>

namespace boot {

//
//// Slots

namespace {
struct SlotImage : ota::ImageReader {
  const Flash &flash;
  uint32_t base;
  SlotImage(const Flash &flash, uint32_t base) : flash(flash), base(base) {}
  void read(uint32_t offset, uint8_t *out, size_t len) const override {
    flash.read(base + offset, out, len);
  }
};
} // namespace

ota::ImageError verify_slot(const Flash &flash, uint8_t slot,
                            const uint8_t public_key[32],
                            ota::ImageHeader *out) {
  if (slot != SLOT_A && slot != SLOT_B)
    return ota::ImageError::TOO_SHORT;

  // The header says how big the image is; verify_image checks it's sane
  ota::ImageHeader h;
  flash.read(SLOT_OFFSET[slot], reinterpret_cast<uint8_t *>(&h), sizeof(h));
  size_t total = ota::IMAGE_FIRMWARE_OFFSET +
                 (h.image_size <= SLOT_FIRMWARE_MAX ? h.image_size : 0);

  ota::ImageHeader checked;
  ota::ImageError err =
      ota::verify_image(SlotImage(flash, SLOT_OFFSET[slot]), total,
                        SLOT_FIRMWARE_MAX, slot_firmware_address(slot),
                        public_key, &checked);
  if (err == ota::ImageError::OK && out)
    *out = checked;
  return err;
}

//
//// Journal

namespace {

uint32_t record_crc(const BootRecord &r) {
  return ota::crc32(reinterpret_cast<const uint8_t *>(&r),
                    offsetof(BootRecord, crc));
}

bool valid_slot(uint8_t s) { return s == SLOT_A || s == SLOT_B; }

bool record_ok(const BootRecord &r) {
  return r.magic == RECORD_MAGIC && r.crc == record_crc(r) &&
         valid_slot(r.active) && (r.trial == NO_SLOT || valid_slot(r.trial));
}

bool erased(const BootRecord &r) {
  const uint8_t *p = reinterpret_cast<const uint8_t *>(&r);
  for (size_t i = 0; i < sizeof(r); i++)
    if (p[i] != 0xFF)
      return false;
  return true;
}

BootRecord defaults() {
  BootRecord r{};
  r.active = SLOT_A;
  r.trial = NO_SLOT;
  return r;
}

} // namespace

bool BootState::load() {
  latest = defaults();
  found = false;
  sector = 0;
  next_free = 0;

  // Where the next record could go in each sector: after the last slot
  // that isn't blank (a torn record can't be written over)
  uint32_t free_in[STATE_SECTORS] = {};

  for (uint32_t s = 0; s < STATE_SECTORS; s++) {
    for (uint32_t i = 0; i < RECORDS_PER_SECTOR; i++) {
      BootRecord r;
      flash.read(record_offset(s, i), reinterpret_cast<uint8_t *>(&r),
                 sizeof(r));
      if (erased(r))
        continue;
      free_in[s] = i + 1;
      if (record_ok(r) && (!found || r.sequence > latest.sequence)) {
        latest = r;
        found = true;
        sector = s;
      }
    }
  }
  next_free = free_in[sector];
  return found;
}

bool BootState::write(uint8_t active, uint8_t trial, uint8_t attempts) {
  if (!valid_slot(active) || (trial != NO_SLOT && !valid_slot(trial)))
    return false;

  BootRecord r{};
  r.magic = RECORD_MAGIC;
  r.sequence = found ? latest.sequence + 1 : 1;
  r.active = active;
  r.trial = trial;
  r.attempts = attempts;
  r.crc = record_crc(r);

  if (next_free >= RECORDS_PER_SECTOR) {
    // This sector is full: start over in the other one. Until the record
    // below lands, the full sector still holds the latest state.
    uint32_t other = (sector + 1) % STATE_SECTORS;
    if (!flash.erase_sector(STATE_OFFSET + other * SECTOR_SIZE))
      return false;
    sector = other;
    next_free = 0;
  }

  const uint32_t at = record_offset(sector, next_free);
  // Whatever happens, this slot has been used now
  next_free++;
  if (!flash.program(at, reinterpret_cast<const uint8_t *>(&r), sizeof(r)))
    return false;

  BootRecord back;
  flash.read(at, reinterpret_cast<uint8_t *>(&back), sizeof(back));
  if (memcmp(&back, &r, sizeof(r)) != 0)
    return false;

  latest = r;
  found = true;
  return true;
}

//
//// Policy

BootDecision decide(const BootRecord &state, bool has_record,
                    const bool valid[2]) {
  BootDecision d;
  uint8_t active = has_record ? state.active : SLOT_A;

  if (has_record && state.trial != NO_SLOT) {
    if (state.attempts < TRIAL_BOOTS && valid[state.trial]) {
      // Try the update. Count the boot first, so a crash or reset in it
      // still counts.
      d.slot = state.trial;
      d.trial = true;
      d.write = true;
      d.active = active;
      d.trial_slot = state.trial;
      d.attempts = state.attempts + 1;
      return d;
    }
    // Out of tries, or it's gone bad: roll back
    d.write = true;
    d.active = active;
    d.trial_slot = NO_SLOT;
    d.attempts = 0;
  }

  if (valid[active]) {
    d.slot = active;
    return d;
  }

  // The active slot is broken. Fall back to the other one if it's good.
  uint8_t other = other_slot(active);
  if (valid[other]) {
    d.slot = other;
    d.write = true;
    d.active = other;
    d.trial_slot = NO_SLOT;
    d.attempts = 0;
    return d;
  }

  d.slot = NO_SLOT;
  return d;
}

//
//// Running firmware

bool start_trial(BootState &state, uint8_t running_slot, uint8_t new_slot) {
  if (running_slot == new_slot)
    return false;
  return state.write(running_slot, new_slot, 0);
}

bool confirm(BootState &state, uint8_t running_slot) {
  const BootRecord &r = state.current();
  if (state.has_record() && r.active == running_slot && r.trial == NO_SLOT)
    return true;
  return state.write(running_slot, NO_SLOT, 0);
}

bool running_trial(const BootState &state, uint8_t running_slot) {
  const BootRecord &r = state.current();
  return state.has_record() && r.trial == running_slot;
}

} // namespace boot
