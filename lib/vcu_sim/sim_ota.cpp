#include "sim_ota.hpp"
#include "sim_car.hpp"
#include <ota_image.hpp>
#include <string.h>

namespace sim {

bool NorFlash::erase_sector(uint32_t offset) {
  if (offset % sector != 0 || offset >= bytes.size())
    return false;
  if (bytes_programmed >= fail_after_bytes)
    return false;
  if (interrupt_next_erase) {
    interrupt_next_erase = false;
    memset(bytes.data() + offset, 0xFF, sector / 2);
    return false;
  }
  memset(bytes.data() + offset, 0xFF, sector);
  erases++;
  return true;
}

bool NorFlash::program(uint32_t offset, const uint8_t *data, size_t len) {
  if (offset + len > bytes.size())
    return false;
  for (size_t i = 0; i < len; i++) {
    if (bytes_programmed >= fail_after_bytes)
      return false;
    uint8_t &cell = bytes[offset + i];
    if ((cell & data[i]) != data[i])
      unerased_writes++;
    cell &= data[i];
    bytes_programmed++;
  }
  return true;
}

void NorFlash::read(uint32_t offset, uint8_t *out, size_t len) const {
  memcpy(out, bytes.data() + offset, len);
}

void Board::activate_and_reboot() {
  activations++;
  ota::AbBoard::activate_and_reboot();
}

void Board::reboot() {
  reboots++;
  power_on();
}

void Board::install(uint8_t slot, const std::vector<uint8_t> &firmware,
                    uint32_t version) {
  std::vector<uint8_t> img = make_image(firmware, version, ota::IMAGE_TARGET,
                                        boot::slot_firmware_address(slot));
  for (uint32_t off = 0; off < img.size(); off += boot::SECTOR_SIZE)
    chip.erase_sector(boot::SLOT_OFFSET[slot] + off);
  chip.program(boot::SLOT_OFFSET[slot], img.data(), img.size());
}

void Board::power_on() {
  boot::BootState state(chip);
  state.load();
  bool valid[2];
  ota::ImageHeader header[2];
  for (uint8_t s : {boot::SLOT_A, boot::SLOT_B})
    valid[s] = boot::verify_slot(chip, s, ota::IMAGE_PUBLIC_KEY, &header[s]) ==
               ota::ImageError::OK;

  boot::BootDecision d = boot::decide(state.current(), state.has_record(), valid);
  if (d.write)
    state.write(d.active, d.trial_slot, d.attempts);

  bricked = d.slot == boot::NO_SLOT;
  if (bricked)
    return;
  trial = d.trial;
  set_running(d.slot, header[d.slot].fw_version);
}

std::vector<uint8_t> Board::firmware() const {
  ota::ImageHeader h;
  chip.read(boot::SLOT_OFFSET[running], reinterpret_cast<uint8_t *>(&h), sizeof(h));
  const auto start = chip.bytes.begin() + boot::SLOT_OFFSET[running] +
                     ota::IMAGE_FIRMWARE_OFFSET;
  return std::vector<uint8_t>(start, start + h.image_size);
}

} // namespace sim
