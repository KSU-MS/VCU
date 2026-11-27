#pragma once
// Teensy 4.1 implementations of lib/ota's hardware hooks (src/ota_teensy.cpp)
#include <ota_platform.hpp>

#include "vcu.hpp"

class TeensyStagingFlash : public ota::StagingFlash {
public:
  size_t size() const override;
  size_t sector_size() const override;
  bool erase_sector(uint32_t offset) override;
  bool program(uint32_t offset, const uint8_t *data, size_t len) override;
  void read(uint32_t offset, uint8_t *out, size_t len) const override;
};

class TeensyBoard : public ota::Platform {
public:
  explicit TeensyBoard(VCU &vcu) : vcu(vcu) {}

  bool safe_to_program() override;
  void apply_and_reboot(uint32_t staging_offset, uint32_t size) override;
  void reboot() override;
  uint32_t running_version() override;
  // Built as a whole-flash image, so it can only be replaced by one
  uint32_t image_load_address() override { return 0x60000000; }

private:
  VCU &vcu;
};
