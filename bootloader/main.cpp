// The VCU's bootloader: pick a firmware slot and start it.
//
// Every reset: read the boot state, verify both slots (CRC, linked for that
// slot, Ed25519 signature against the key built in here), let boot::decide()
// pick one (trial update, rollback, fallback), record that, and jump to the
// slot firmware's own entry point. The policy and everything it touches is
// in lib/vcu_boot and tested natively; this file is only the glue.
#include <boot.hpp>

#include "flexspi.hpp"

namespace {

// Same as the boot ROM: the firmware's IVT is 4 KB into it, and its second
// word is the entry point (the Teensy core's ResetHandler)
uint32_t entry_point(uint8_t slot) {
  const uint32_t ivt = boot::slot_firmware_address(slot) + 0x1000;
  return reinterpret_cast<const volatile uint32_t *>(ivt)[1];
}

bool in_slot(uint8_t slot, uint32_t addr) {
  const uint32_t start = boot::slot_firmware_address(slot);
  return addr >= start && addr < start + boot::SLOT_FIRMWARE_MAX;
}

// The RTWDOG (WDOG3), i.MX RT1060 RM chapter 58, clocked from the 32.768 kHz
// crystal. Armed here for the boot window, 4 s: the firmware's startup takes
// over a second before its loop runs (Teensy's USB start-up delay, then
// Serial.begin() waiting up to 2 s for a terminal). CS[UPDATE] = 1 lets the
// firmware reconfigure it once, which it does when it's up: 1 s and locked
// (hal::watchdog_start). Firmware that never gets that far is reset after
// 4 s, and an update on trial that does rolls back.
void arm_watchdog() {
  volatile uint32_t &CS = *reinterpret_cast<volatile uint32_t *>(0x400BC000);
  volatile uint32_t &CNT = *reinterpret_cast<volatile uint32_t *>(0x400BC004);
  volatile uint32_t &TOVAL = *reinterpret_cast<volatile uint32_t *>(0x400BC008);
  constexpr uint32_t CS_UPDATE = 1u << 5, CS_EN = 1u << 7,
                     CS_CLK_LPO = 1u << 8, CS_RCS = 1u << 10,
                     CS_ULK = 1u << 11, CS_PRES = 1u << 12,
                     CS_CMD32EN = 1u << 13;

  CNT = 0xD928C520; // unlock (whatever the boot ROM left it as)
  while (!(CS & CS_ULK))
    ;
  TOVAL = 4 * 128; // 4 s of LPO clock / 256
  CS = CS_UPDATE | CS_EN | CS_CLK_LPO | CS_PRES | CS_CMD32EN;
  while (!(CS & CS_RCS))
    ;
}

[[noreturn]] void start(uint32_t entry) {
  // As close to reset as we can leave it: interrupts back on (nothing has an
  // interrupt enabled), and the firmware's ResetHandler sets its own stack,
  // VTOR, FlexRAM layout and clocks
  __asm__ volatile("dsb\nisb\ncpsie i\nbx %0" ::"r"(entry));
  __builtin_unreachable();
}

[[noreturn]] void nothing_to_boot() {
  // Both slots are bad. Wait for someone with a USB cable: the Teensy's own
  // bootloader chip still works whatever's in flash.
  for (;;)
    __asm__ volatile("wfi");
}

} // namespace

int main() {
  FlexSpiFlash flash;
  boot::BootState state(flash);
  state.load();

  const bool valid[2] = {
      boot::verify_slot(flash, boot::SLOT_A, ota::IMAGE_PUBLIC_KEY) ==
          ota::ImageError::OK,
      boot::verify_slot(flash, boot::SLOT_B, ota::IMAGE_PUBLIC_KEY) ==
          ota::ImageError::OK,
  };

  boot::BootDecision d = boot::decide(state.current(), state.has_record(), valid);
  if (d.write)
    state.write(d.active, d.trial_slot, d.attempts); // boot anyway if it fails

  if (d.slot == boot::NO_SLOT)
    nothing_to_boot();
  const uint32_t entry = entry_point(d.slot);
  if (!in_slot(d.slot, entry & ~1u))
    nothing_to_boot();
  arm_watchdog();
  start(entry);
}
