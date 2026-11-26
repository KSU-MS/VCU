#include "ota_image.hpp"
#include <string.h>

namespace ota {

uint32_t crc32(const uint8_t *data, size_t len, uint32_t crc) {
  // Bitwise rather than a 1 KB table, it only runs once per update
  crc = ~crc;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; bit++)
      crc = (crc >> 1) ^ (0xEDB88320u & -(crc & 1));
  }
  return ~crc;
}

const char *image_error_name(ImageError e) {
  switch (e) {
  case ImageError::OK:
    return "ok";
  case ImageError::TOO_SHORT:
    return "too short";
  case ImageError::BAD_MAGIC:
    return "bad magic";
  case ImageError::BAD_HEADER_VERSION:
    return "unsupported header version";
  case ImageError::BAD_HEADER_CRC:
    return "header crc mismatch";
  case ImageError::WRONG_TARGET:
    return "image is for a different board";
  case ImageError::SIZE_MISMATCH:
    return "size doesn't match header";
  case ImageError::TOO_BIG:
    return "image too big";
  case ImageError::BAD_IMAGE_CRC:
    return "image crc mismatch";
  }
  return "?";
}

ImageError parse_header(const uint8_t *data, size_t total_size,
                        size_t max_image_size, ImageHeader *out) {
  if (total_size < IMAGE_HEADER_SIZE)
    return ImageError::TOO_SHORT;

  // memcpy rather than casting, the buffer may not be aligned
  ImageHeader h;
  memcpy(&h, data, sizeof(h));

  if (memcmp(h.magic, IMAGE_MAGIC, sizeof(IMAGE_MAGIC)) != 0)
    return ImageError::BAD_MAGIC;
  if (h.header_version != IMAGE_HEADER_VERSION ||
      h.header_size != IMAGE_HEADER_SIZE)
    return ImageError::BAD_HEADER_VERSION;
  if (crc32(data, offsetof(ImageHeader, header_crc32)) != h.header_crc32)
    return ImageError::BAD_HEADER_CRC;
  if (strncmp(h.target, IMAGE_TARGET, IMAGE_TARGET_LEN) != 0)
    return ImageError::WRONG_TARGET;
  if (h.image_size > max_image_size)
    return ImageError::TOO_BIG;
  if (total_size != IMAGE_HEADER_SIZE + size_t(h.image_size))
    return ImageError::SIZE_MISMATCH;

  *out = h;
  return ImageError::OK;
}

} // namespace ota
