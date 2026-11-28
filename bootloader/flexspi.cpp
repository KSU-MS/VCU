// The flash chip through FlexSPI IP commands, the same way the Teensy core
// does it (cores/teensy4/eeprom.c): LUT entry 15 is (re)programmed for each
// command, data goes through the IP TX FIFO 8 bytes at a time, then status
// is polled until the chip isn't busy.
//
// While the chip erases or programs it can't be read, and the code runs from
// it, so everything that issues a command and waits runs from ITCM (copied
// there by startup.c). Interrupts are off for the whole bootloader.
#include "flexspi.hpp"

#include <string.h>

#define RAMFUNC __attribute__((section(".ramfunc"), noinline, long_call))

namespace {

constexpr uint32_t FLEXSPI = 0x402A8000;
constexpr uint32_t MCR0 = 0x000, INTR = 0x014, LUTKEY = 0x018, LUTCR = 0x01C,
                   IPCR0 = 0x0A0, IPCR1 = 0x0A4, IPCMD = 0x0B0,
                   IPRXFCR = 0x0B8, IPTXFCR = 0x0BC, RFDR0 = 0x100,
                   TFDR0 = 0x180, LUT60 = 0x2F0;
constexpr uint32_t INTR_IPCMDDONE = 1u << 0, INTR_IPTXWE = 1u << 6;

// LUT instructions
constexpr uint32_t CMD_SDR = 0x01, RADDR_SDR = 0x02, WRITE_SDR = 0x08,
                   READ_SDR = 0x09, PADS1 = 0x00, PADS4 = 0x02;
constexpr uint32_t lut0(uint32_t op, uint32_t pads, uint32_t operand) {
  return (op << 10) | (pads << 8) | operand;
}
constexpr uint32_t lut1(uint32_t op, uint32_t pads, uint32_t operand) {
  return lut0(op, pads, operand) << 16;
}

__attribute__((always_inline)) inline volatile uint32_t &reg(uint32_t off) {
  return *reinterpret_cast<volatile uint32_t *>(FLEXSPI + off);
}

__attribute__((always_inline)) inline void set_lut(uint32_t l0, uint32_t l1) {
  reg(LUTKEY) = 0x5AF05AF0;
  reg(LUTCR) = 1u << 1; // unlock
  reg(LUT60) = l0;
  reg(LUT60 + 4) = l1;
  reg(LUT60 + 8) = 0;
  reg(LUT60 + 12) = 0;
}

__attribute__((always_inline)) inline void run(uint32_t addr, uint32_t size) {
  reg(IPCR0) = addr;
  reg(IPCR1) = (15u << 16) | size; // sequence 15 = LUT 60..63
  reg(IPCMD) = 1;
}

__attribute__((always_inline)) inline void wait_done() {
  while (!(reg(INTR) & INTR_IPCMDDONE))
    ;
  reg(INTR) = INTR_IPCMDDONE;
}

RAMFUNC void write_enable() {
  set_lut(lut0(CMD_SDR, PADS1, 0x06), 0);
  run(0, 0);
  wait_done();
}

RAMFUNC void wait_ready() {
  set_lut(lut0(CMD_SDR, PADS1, 0x05) | lut1(READ_SDR, PADS1, 1), 0);
  uint8_t status;
  do {
    reg(IPRXFCR) = 1; // clear RX FIFO
    run(0, 1);
    wait_done();
    status = uint8_t(reg(RFDR0));
  } while (status & 1);
  // Throw away anything the AHB read buffer prefetched from before
  reg(MCR0) |= 1;
  while (reg(MCR0) & 1)
    ;
}

RAMFUNC void erase_4k(uint32_t addr) {
  write_enable();
  set_lut(lut0(CMD_SDR, PADS1, 0x20) | lut1(RADDR_SDR, PADS1, 24), 0);
  run(addr, 0);
  wait_done();
  wait_ready();
}

// Up to one page (256 bytes), not crossing a page boundary
RAMFUNC void program_page(uint32_t addr, const uint8_t *data, uint32_t len) {
  write_enable();
  set_lut(lut0(CMD_SDR, PADS1, 0x32) | lut1(RADDR_SDR, PADS1, 24),
          lut0(WRITE_SDR, PADS4, 1));
  reg(IPTXFCR) = 1; // clear TX FIFO
  run(addr, len);
  uint32_t sent = 0;
  uint32_t intr;
  while (!((intr = reg(INTR)) & INTR_IPCMDDONE)) {
    if (intr & INTR_IPTXWE) {
      uint32_t words[2] = {0xFFFFFFFF, 0xFFFFFFFF};
      uint8_t *bytes = reinterpret_cast<uint8_t *>(words);
      for (uint32_t i = 0; i < 8 && sent < len; i++)
        bytes[i] = data[sent++];
      reg(TFDR0) = words[0];
      reg(TFDR0 + 4) = words[1];
      reg(INTR) = INTR_IPTXWE;
    }
  }
  reg(INTR) = INTR_IPCMDDONE | INTR_IPTXWE;
  wait_ready();
}

// The data cache may hold what the flash said before we changed it
void dcache_invalidate(uint32_t addr, uint32_t len) {
  volatile uint32_t &DCIMVAC = *reinterpret_cast<volatile uint32_t *>(0xE000EF5C);
  __asm__ volatile("dsb" ::: "memory");
  for (uint32_t a = addr & ~31u; a < addr + len; a += 32)
    DCIMVAC = a;
  __asm__ volatile("dsb\nisb" ::: "memory");
}

} // namespace

void FlexSpiFlash::read(uint32_t offset, uint8_t *out, size_t len) const {
  memcpy(out, reinterpret_cast<const void *>(boot::FLASH_BASE + offset), len);
}

bool FlexSpiFlash::erase_sector(uint32_t offset) {
  if (offset % boot::SECTOR_SIZE != 0 || offset < boot::BOOTLOADER_SIZE)
    return false; // never erase ourselves
  erase_4k(offset);
  dcache_invalidate(boot::FLASH_BASE + offset, boot::SECTOR_SIZE);
  return true;
}

bool FlexSpiFlash::program(uint32_t offset, const uint8_t *data, size_t len) {
  if (offset < boot::BOOTLOADER_SIZE)
    return false;
  // The data may be in flash itself, which can't be read while programming
  uint8_t page[256];
  const uint32_t start = offset;
  while (len) {
    uint32_t n = 256 - (offset % 256);
    if (n > len)
      n = len;
    memcpy(page, data, n);
    program_page(offset, page, n);
    offset += n;
    data += n;
    len -= n;
  }
  dcache_invalidate(boot::FLASH_BASE + start, offset - start);
  return true;
}
