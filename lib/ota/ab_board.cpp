#include "ab_board.hpp"

namespace ota {

bool SlotStaging::erase_sector(uint32_t offset) {
  if (offset % boot::SECTOR_SIZE || offset >= boot::SLOT_SIZE)
    return false;
  return flash.erase_sector(base + offset);
}

bool SlotStaging::program(uint32_t offset, const uint8_t *data, size_t len) {
  if (offset > boot::SLOT_SIZE || len > boot::SLOT_SIZE - offset)
    return false;
  return flash.program(base + offset, data, len);
}

void SlotStaging::read(uint32_t offset, uint8_t *out, size_t len) const {
  flash.read(base + offset, out, len);
}

// The boot state is re-read every time: the bootloader and earlier calls
// may have changed it since

bool AbBoard::can_update() {
  boot::BootState state(flash);
  state.load();
  return !boot::running_trial(state, running);
}

void AbBoard::activate_and_reboot() {
  boot::BootState state(flash);
  state.load();
  // If this doesn't land, the reboot just comes back to us
  boot::start_trial(state, running, boot::other_slot(running));
  reboot();
}

bool AbBoard::confirm() {
  boot::BootState state(flash);
  state.load();
  return boot::confirm(state, running);
}

uint32_t AbBoard::image_load_address() {
  return boot::slot_firmware_address(boot::other_slot(running));
}

void AbBoard::boot_status(uint8_t out[4]) {
  boot::BootState state(flash);
  state.load();
  out[0] = running;
  out[1] = state.current().active;
  out[2] = state.current().trial;
  out[3] = state.current().attempts;
}

} // namespace ota
