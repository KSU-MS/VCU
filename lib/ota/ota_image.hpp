#pragma once
// What gets sent over the wire: a 64 byte header followed by the raw Teensy
// flash image (what teensy_loader would write starting at 0x60000000).
// tools/ota/image.py builds these, and the layout here must match it.
#include <stddef.h>
#include <stdint.h>

namespace ota {

// Standard CRC-32 (IEEE 802.3, same as zlib.crc32), chainable:
// crc32(b, n2, crc32(a, n1)) == crc32(a + b)
uint32_t crc32(const uint8_t *data, size_t len, uint32_t crc = 0);

constexpr char IMAGE_MAGIC[4] = {'K', 'V', 'C', 'U'};
constexpr uint16_t IMAGE_HEADER_VERSION = 1;
constexpr size_t IMAGE_HEADER_SIZE = 64;
constexpr size_t IMAGE_TARGET_LEN = 16;

// Which board an image is for, so a dash image can't land on the VCU
constexpr char IMAGE_TARGET[IMAGE_TARGET_LEN] = "teensy41-vcu";

// All fields little-endian
struct ImageHeader {
  char magic[4];
  uint16_t header_version;
  uint16_t header_size;
  uint32_t image_size;  // bytes after the header
  uint32_t image_crc32; // over those bytes
  uint32_t fw_version;  // AUTO_VERSION (git hash) of the image
  char target[IMAGE_TARGET_LEN];
  uint32_t flags; // bit 0: built from a dirty tree
  uint8_t reserved[20];
  uint32_t header_crc32; // over the 60 bytes before this
};
static_assert(sizeof(ImageHeader) == IMAGE_HEADER_SIZE, "header layout");

enum class ImageError : uint8_t {
  OK = 0,
  TOO_SHORT,
  BAD_MAGIC,
  BAD_HEADER_VERSION,
  BAD_HEADER_CRC,
  WRONG_TARGET,
  SIZE_MISMATCH,
  TOO_BIG,
  BAD_IMAGE_CRC,
};

const char *image_error_name(ImageError e);

// Checks everything that can be checked from the header alone. `total_size`
// is how many bytes were received (header + image), `max_image_size` is how
// big an image the board can hold.
ImageError parse_header(const uint8_t *data, size_t total_size,
                        size_t max_image_size, ImageHeader *out);

} // namespace ota
