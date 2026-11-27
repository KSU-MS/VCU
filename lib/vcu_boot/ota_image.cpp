#include "ota_image.hpp"
#include <monocypher-ed25519.h>
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
  case ImageError::WRONG_LOAD_ADDRESS:
    return "image is linked for a different address";
  case ImageError::BAD_SIGNATURE:
    return "bad signature";
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
  if (total_size != IMAGE_FIRMWARE_OFFSET + size_t(h.image_size))
    return ImageError::SIZE_MISMATCH;

  *out = h;
  return ImageError::OK;
}

ImageError verify_image(const ImageReader &image, size_t total_size,
                        size_t max_image_size, uint32_t expected_load_address,
                        const uint8_t public_key[32], ImageHeader *out) {
  uint8_t head[IMAGE_HEADER_SIZE];
  if (total_size < IMAGE_FIRMWARE_OFFSET)
    return ImageError::TOO_SHORT;
  image.read(0, head, sizeof(head));

  ImageHeader header;
  ImageError err = parse_header(head, total_size, max_image_size, &header);
  if (err != ImageError::OK)
    return err;
  if (header.load_address != expected_load_address)
    return ImageError::WRONG_LOAD_ADDRESS;

  // One pass over the firmware for both the CRC and the signed digest
  crypto_sha512_ctx sha;
  crypto_sha512_init(&sha);
  crypto_sha512_update(&sha, head, sizeof(head));
  uint32_t crc = 0;
  uint8_t chunk[256];
  for (uint32_t done = 0; done < header.image_size; done += sizeof(chunk)) {
    size_t n = header.image_size - done < sizeof(chunk)
                   ? header.image_size - done
                   : sizeof(chunk);
    image.read(IMAGE_FIRMWARE_OFFSET + done, chunk, n);
    crc = crc32(chunk, n, crc);
    crypto_sha512_update(&sha, chunk, n);
  }
  uint8_t digest[64];
  crypto_sha512_final(&sha, digest);

  if (crc != header.image_crc32)
    return ImageError::BAD_IMAGE_CRC;

  uint8_t signature[IMAGE_SIGNATURE_SIZE];
  image.read(IMAGE_SIGNATURE_OFFSET, signature, sizeof(signature));
  if (crypto_ed25519_check(signature, public_key, digest,
                                       sizeof(digest)) != 0)
    return ImageError::BAD_SIGNATURE;

  *out = header;
  return ImageError::OK;
}

} // namespace ota
