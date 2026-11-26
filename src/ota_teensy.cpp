// The Teensy 4.1 side of the firmware update: where the image is staged, and
// how it's copied over the running firmware. Everything else (transport,
// protocol, verification) is in lib/ota and tested in the simulator.
//
// THIS FILE HAS NOT BEEN RUN ON HARDWARE YET. See docs/ota.md for what to
// check on the bench.
#if defined(__IMXRT1062__)
#include "ota_teensy.hpp"
#include "parameters.hpp"
#include <string.h>

extern "C" {
#include <FlashTxx.h>
}

// 1 MB staging area, 4 MB into the 8 MB flash: far above the firmware
// (~100 KB today) and well clear of the top of flash, where the Teensy keeps
// EEPROM emulation and the restore program
static constexpr uint32_t STAGING_ADDR = FLASH_BASE_ADDR + 0x400000;
static constexpr uint32_t STAGING_SIZE = 0x100000;

size_t TeensyStagingFlash::size() const { return STAGING_SIZE; }
size_t TeensyStagingFlash::sector_size() const { return FLASH_SECTOR_SIZE; }

bool TeensyStagingFlash::erase_sector(uint32_t offset) {
  if (offset % FLASH_SECTOR_SIZE || offset >= STAGING_SIZE)
    return false;
  return flash_erase_block(STAGING_ADDR + offset, FLASH_SECTOR_SIZE) == 0;
}

// The flash takes aligned 32 bit writes, from a buffer in RAM. Blocks don't
// have to be a multiple of 4 long, so merge each word with what's already
// there; re-programming bytes with their current value is a no-op on NOR.
bool TeensyStagingFlash::program(uint32_t offset, const uint8_t *data,
                                 size_t len) {
  if (offset + len > STAGING_SIZE)
    return false;

  uint32_t addr = STAGING_ADDR + offset;
  const uint32_t end = addr + len;
  while (addr < end) {
    const uint32_t word_addr = addr & ~3u;
    uint32_t word;
    memcpy(&word, (const void *)word_addr, 4);

    uint8_t *w = reinterpret_cast<uint8_t *>(&word);
    for (uint32_t a = addr; a < word_addr + 4 && a < end; a++)
      w[a - word_addr] = *data++;

    eepromemu_flash_write((void *)word_addr, &word, 4);
    addr = word_addr + 4;
  }
  return true;
}

void TeensyStagingFlash::read(uint32_t offset, uint8_t *out, size_t len) const {
  // Flash is memory mapped. The core's flash write/erase routines invalidate
  // the cache for what they touch, so this sees what was actually written.
  memcpy(out, (const void *)(STAGING_ADDR + offset), len);
}

bool TeensyBoard::safe_to_program() { return vcu.safe_to_program(); }

void TeensyBoard::apply_and_reboot(uint32_t staging_offset, uint32_t size) {
  // Point of no return. flash_move runs from RAM with interrupts off, erases
  // and rewrites the firmware sector by sector, then reboots. Losing power
  // in the next second or so leaves a board that needs a USB reflash.
  __disable_irq();
  flash_move(FLASH_BASE_ADDR, STAGING_ADDR + staging_offset, size);
}

void TeensyBoard::reboot() { REBOOT; }

uint32_t TeensyBoard::running_version() { return AUTO_VERSION; }

#endif
