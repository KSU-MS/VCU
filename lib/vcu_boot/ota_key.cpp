#include "ota_image.hpp"

// scripts/ota_key.py defines this from the public key file the build uses
#ifndef VCU_OTA_PUBLIC_KEY
#error "VCU_OTA_PUBLIC_KEY isn't defined: add pre:scripts/ota_key.py to extra_scripts"
#endif

namespace ota {
const uint8_t IMAGE_PUBLIC_KEY[32] = {VCU_OTA_PUBLIC_KEY};
} // namespace ota
