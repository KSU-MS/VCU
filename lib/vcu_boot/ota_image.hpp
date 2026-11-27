#pragma once
// A signed firmware image, as sent over the wire and as stored in flash:
//
//   0x0000  header, 64 bytes (ImageHeader)
//   0x0040  Ed25519 signature over SHA-512(header || firmware), 64 bytes
//   0x0080  0xFF up to IMAGE_FIRMWARE_OFFSET
//   0x1000  firmware: image_size bytes, linked to run at load_address
//
// The whole thing is written to flash byte for byte, so the firmware ends up
// at the address it was linked for when the image starts load_address - 4 KB.
// tools/ota/image.py builds these, and the layout here must match it.
#include <stddef.h>
#include <stdint.h>

namespace ota {

// Standard CRC-32 (IEEE 802.3, same as zlib.crc32), chainable:
// crc32(b, n2, crc32(a, n1)) == crc32(a + b)
uint32_t crc32(const uint8_t *data, size_t len, uint32_t crc = 0);

constexpr char IMAGE_MAGIC[4] = {'K', 'V', 'C', 'U'};
constexpr uint16_t IMAGE_HEADER_VERSION = 2;
constexpr size_t IMAGE_HEADER_SIZE = 64;
constexpr size_t IMAGE_SIGNATURE_OFFSET = IMAGE_HEADER_SIZE;
constexpr size_t IMAGE_SIGNATURE_SIZE = 64;
constexpr size_t IMAGE_FIRMWARE_OFFSET = 0x1000; // one flash sector
constexpr size_t IMAGE_TARGET_LEN = 16;

// Which board an image is for, so a dash image can't land on the VCU
constexpr char IMAGE_TARGET[IMAGE_TARGET_LEN] = "teensy41-vcu";

// All fields little-endian
struct ImageHeader {
  char magic[4];
  uint16_t header_version;
  uint16_t header_size;
  uint32_t image_size;   // firmware bytes, from IMAGE_FIRMWARE_OFFSET
  uint32_t image_crc32;  // over the firmware
  uint32_t fw_version;   // AUTO_VERSION (git hash) of the image
  char target[IMAGE_TARGET_LEN];
  uint32_t flags;        // bit 0: built from a dirty tree
  uint32_t load_address; // where the firmware is linked to run
  uint8_t reserved[16];
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
  WRONG_LOAD_ADDRESS, // linked for somewhere else (the other A/B slot)
  BAD_SIGNATURE,      // not signed with our key, or modified since
};

const char *image_error_name(ImageError e);

// Checks everything that can be checked from the header alone. `total_size`
// is how many bytes were received (the whole image), `max_image_size` is how
// much firmware the board can hold.
ImageError parse_header(const uint8_t *data, size_t total_size,
                        size_t max_image_size, ImageHeader *out);

// Wherever the image is (flash, a buffer), read `len` bytes at `offset`
// from its start
class ImageReader {
public:
  virtual void read(uint32_t offset, uint8_t *out, size_t len) const = 0;

protected:
  ~ImageReader() = default;
};

// The full check, on what's actually stored: header, that the firmware was
// linked for `expected_load_address`, its CRC, and the Ed25519 signature
// against `public_key`.
ImageError verify_image(const ImageReader &image, size_t total_size,
                        size_t max_image_size, uint32_t expected_load_address,
                        const uint8_t public_key[32], ImageHeader *out);

// The key images must be signed with, built in from VCU_OTA_PUBLIC_KEY (see
// scripts/ota_key.py)
extern const uint8_t IMAGE_PUBLIC_KEY[32];

} // namespace ota
