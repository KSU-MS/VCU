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
  start(entry);
}
