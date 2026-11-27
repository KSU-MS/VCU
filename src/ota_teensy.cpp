// The Teensy 4.1 side of the firmware update: its flash chip through the
// Teensy core's own erase/program routines, and rebooting.
//
// THIS FILE HAS NOT BEEN RUN ON HARDWARE YET. See docs/ota.md for what to
// check on the bench.
#if defined(__IMXRT1062__)
#include "ota_teensy.hpp"
#include "parameters.hpp"
#include <Arduino.h>
#include <string.h>

// cores/teensy4/eeprom.c. Interrupts are off while each one runs and the
// chip is busy, and back on after.
extern "C" {
void eepromemu_flash_write(void *addr, const void *data, uint32_t len);
void eepromemu_flash_erase_sector(void *addr);
}

void TeensyFlash::read(uint32_t offset, uint8_t *out, size_t len) const {
  // Flash is memory mapped. The core's flash routines invalidate the cache
  // for what they touch, so this sees what was actually written.
  memcpy(out, reinterpret_cast<const void *>(boot::FLASH_BASE + offset), len);
}

bool TeensyFlash::erase_sector(uint32_t offset) {
  // Never the bootloader, never the core's EEPROM emulation
  if (offset % boot::SECTOR_SIZE || offset < boot::BOOTLOADER_SIZE ||
      offset >= boot::EEPROM_OFFSET)
    return false;
  eepromemu_flash_erase_sector(reinterpret_cast<void *>(boot::FLASH_BASE + offset));
  return true;
}

bool TeensyFlash::program(uint32_t offset, const uint8_t *data, size_t len) {
  if (offset < boot::BOOTLOADER_SIZE || offset >= boot::EEPROM_OFFSET ||
      len > boot::EEPROM_OFFSET - offset)
    return false;
  // A page program wraps at the 256 byte page boundary, so never cross one
  while (len) {
    size_t n = 256 - (offset % 256);
    if (n > len)
      n = len;
    eepromemu_flash_write(reinterpret_cast<void *>(boot::FLASH_BASE + offset),
                          data, n);
    offset += n;
    data += n;
    len -= n;
  }
  return true;
}

TeensyBoard::TeensyBoard(TeensyFlash &flash, VCU &vcu)
    : ota::AbBoard(flash, VCU_SLOT, AUTO_VERSION), vcu(vcu) {}

bool TeensyBoard::safe_to_program() { return vcu.safe_to_program(); }

void TeensyBoard::reboot() { SCB_AIRCR = 0x05FA0004; }

#endif
