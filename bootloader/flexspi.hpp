#pragma once
// The Teensy 4.1's flash chip for the bootloader (boot::Flash on FlexSPI)
#include <boot.hpp>

class FlexSpiFlash : public boot::Flash {
public:
  void read(uint32_t offset, uint8_t *out, size_t len) const override;
  // Refuse to touch the bootloader's own 64 KB
  bool erase_sector(uint32_t offset) override;
  bool program(uint32_t offset, const uint8_t *data, size_t len) override;
};
