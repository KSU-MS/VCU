#include "sim_ota.hpp"
#include <ota_image.hpp>
#include <string.h>

namespace sim {

bool NorFlash::erase_sector(uint32_t offset) {
  if (offset % sector != 0 || offset >= bytes.size())
    return false;
  if (bytes_programmed >= fail_after_bytes)
    return false;
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

void Board::apply_and_reboot(uint32_t staging_offset, uint32_t size) {
  firmware.assign(staging.bytes.begin() + staging_offset,
                  staging.bytes.begin() + staging_offset + size);

  // The new firmware would report its own version once it boots
  ota::ImageHeader h;
  memcpy(&h, staging.bytes.data(), sizeof(h));
  version = h.fw_version;

  applies++;
  reboots++;
}

} // namespace sim
