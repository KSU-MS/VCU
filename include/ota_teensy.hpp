#pragma once
// Teensy 4.1 side of the firmware update (src/ota_teensy.cpp): the flash
// chip, and the A/B board on top of it. Everything else (transport,
// protocol, signatures, which slot boots) is in lib/ota and lib/vcu_boot.
#include <ab_board.hpp>

#include "vcu.hpp"

// Which slot this build runs from: platformio.ini, -DVCU_SLOT=0 (A) or 1 (B)
#ifndef VCU_SLOT
#error "VCU_SLOT isn't set: the firmware must be built for an A/B slot"
#endif

class TeensyFlash : public boot::Flash {
public:
  void read(uint32_t offset, uint8_t *out, size_t len) const override;
  bool erase_sector(uint32_t offset) override;
  bool program(uint32_t offset, const uint8_t *data, size_t len) override;
};

class TeensyBoard : public ota::AbBoard {
public:
  TeensyBoard(TeensyFlash &flash, VCU &vcu);

  bool safe_to_program() override;
  void reboot() override;

private:
  VCU &vcu;
};
