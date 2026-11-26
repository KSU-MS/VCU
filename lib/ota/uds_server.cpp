#include "uds_server.hpp"
#include <string.h>

namespace ota {

namespace {

constexpr uint8_t SUPPRESS_POSITIVE_RESPONSE = 0x80;

size_t negative(uint8_t *resp, uint8_t service, uint8_t code) {
  resp[0] = sid::NEGATIVE_RESPONSE;
  resp[1] = service;
  resp[2] = code;
  return 3;
}

uint32_t read_be32(const uint8_t *p) {
  return (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) |
         (uint32_t(p[2]) << 8) | p[3];
}

void write_be32(uint8_t *p, uint32_t v) {
  p[0] = uint8_t(v >> 24);
  p[1] = uint8_t(v >> 16);
  p[2] = uint8_t(v >> 8);
  p[3] = uint8_t(v);
}

} // namespace

UdsServer::UdsServer(Platform &platform, StagingFlash &staging)
    : platform(platform), staging(staging) {}

void UdsServer::abandon_download() {
  downloading = false;
  download_complete = false;
  verified = false;
  expected = received = 0;
  blocks_accepted = 0;
  next_block = 1;
}

void UdsServer::end_session() {
  session = Session::DEFAULT;
  abandon_download();
}

void UdsServer::poll(uint32_t now) {
  if (session != Session::PROGRAMMING)
    return;

  // Tester went away, or the car stopped being safe to program under us
  if (now - last_request_ms > SESSION_TIMEOUT_MS || !platform.safe_to_program())
    end_session();
}

size_t UdsServer::handle(const uint8_t *req, size_t len, uint8_t *resp,
                         uint32_t now) {
  if (len == 0)
    return 0;
  last_request_ms = now;

  switch (req[0]) {
  case sid::SESSION_CONTROL:
    return session_control(req, len, resp);
  case sid::TESTER_PRESENT:
    return tester_present(req, len, resp);
  case sid::READ_DID:
    return read_did(req, len, resp);
  case sid::ECU_RESET:
    return ecu_reset(req, len, resp);
  }

  // Everything below here reprograms the board
  switch (req[0]) {
  case sid::REQUEST_DOWNLOAD:
  case sid::TRANSFER_DATA:
  case sid::TRANSFER_EXIT:
  case sid::ROUTINE_CONTROL:
    if (session != Session::PROGRAMMING)
      return negative(resp, req[0], nrc::NOT_SUPPORTED_IN_SESSION);
    break;
  default:
    return negative(resp, req[0], nrc::SERVICE_NOT_SUPPORTED);
  }

  switch (req[0]) {
  case sid::REQUEST_DOWNLOAD:
    return request_download(req, len, resp);
  case sid::TRANSFER_DATA:
    return transfer_data(req, len, resp);
  case sid::TRANSFER_EXIT:
    return transfer_exit(req, len, resp);
  default:
    return routine_control(req, len, resp);
  }
}

size_t UdsServer::session_control(const uint8_t *req, size_t len,
                                  uint8_t *resp) {
  if (len != 2)
    return negative(resp, req[0], nrc::INCORRECT_LENGTH);

  const uint8_t sub = req[1] & ~SUPPRESS_POSITIVE_RESPONSE;
  switch (sub) {
  case uint8_t(Session::DEFAULT):
    end_session();
    break;
  case uint8_t(Session::PROGRAMMING):
    if (!platform.safe_to_program())
      return negative(resp, req[0], nrc::CONDITIONS_NOT_CORRECT);
    if (session != Session::PROGRAMMING)
      abandon_download();
    session = Session::PROGRAMMING;
    break;
  default:
    return negative(resp, req[0], nrc::SUBFUNCTION_NOT_SUPPORTED);
  }

  if (req[1] & SUPPRESS_POSITIVE_RESPONSE)
    return 0;

  // P2 = 50 ms, P2* = 5000 ms (in 10 ms units)
  resp[0] = req[0] + 0x40;
  resp[1] = sub;
  resp[2] = 0x00;
  resp[3] = 50;
  resp[4] = 0x01;
  resp[5] = 0xF4;
  return 6;
}

size_t UdsServer::tester_present(const uint8_t *req, size_t len,
                                 uint8_t *resp) {
  if (len != 2)
    return negative(resp, req[0], nrc::INCORRECT_LENGTH);
  if ((req[1] & ~SUPPRESS_POSITIVE_RESPONSE) != 0)
    return negative(resp, req[0], nrc::SUBFUNCTION_NOT_SUPPORTED);
  if (req[1] & SUPPRESS_POSITIVE_RESPONSE)
    return 0;
  resp[0] = req[0] + 0x40;
  resp[1] = 0x00;
  return 2;
}

size_t UdsServer::read_did(const uint8_t *req, size_t len, uint8_t *resp) {
  if (len != 3)
    return negative(resp, req[0], nrc::INCORRECT_LENGTH);

  const uint16_t did = uint16_t((req[1] << 8) | req[2]);
  resp[0] = req[0] + 0x40;
  resp[1] = req[1];
  resp[2] = req[2];

  switch (did) {
  case DID_SOFTWARE_VERSION:
    write_be32(resp + 3, platform.running_version());
    return 7;
  case DID_OTA_STATUS:
    resp[3] = uint8_t(session);
    resp[4] = uint8_t((downloading ? 1 : 0) | (download_complete ? 2 : 0) |
                      (verified ? 4 : 0));
    resp[5] = uint8_t(image_error);
    resp[6] = 0;
    return 7;
  default:
    return negative(resp, req[0], nrc::REQUEST_OUT_OF_RANGE);
  }
}

size_t UdsServer::request_download(const uint8_t *req, size_t len,
                                   uint8_t *resp) {
  // 34 <dataFormat> <addressAndLengthFormat=0x44> <addr:4> <size:4>
  if (len != 11)
    return negative(resp, req[0], nrc::INCORRECT_LENGTH);
  if (req[1] != 0x00 || req[2] != 0x44)
    return negative(resp, req[0], nrc::REQUEST_OUT_OF_RANGE);

  const uint32_t size = read_be32(req + 7);
  if (size < IMAGE_HEADER_SIZE || size > staging.size())
    return negative(resp, req[0], nrc::UPLOAD_DOWNLOAD_NOT_ACCEPTED);

  // Starting again is always allowed, it throws away whatever was staged
  abandon_download();
  downloading = true;
  expected = size;
  erased_to = 0;

  // lengthFormatIdentifier 0x20: 2 byte maxNumberOfBlockLength, which
  // counts the SID and block counter too
  const uint16_t max_block = BLOCK_DATA_SIZE + 2;
  resp[0] = req[0] + 0x40;
  resp[1] = 0x20;
  resp[2] = uint8_t(max_block >> 8);
  resp[3] = uint8_t(max_block);
  return 4;
}

bool UdsServer::stage(const uint8_t *data, size_t len) {
  // Erase sectors just ahead of where we're about to write. Erasing as we go
  // rather than all up front keeps each request short.
  const uint32_t end = received + len;
  while (erased_to < end) {
    if (!staging.erase_sector(erased_to))
      return false;
    erased_to += staging.sector_size();
  }
  return staging.program(received, data, len);
}

size_t UdsServer::transfer_data(const uint8_t *req, size_t len, uint8_t *resp) {
  if (!downloading || download_complete)
    return negative(resp, req[0], nrc::REQUEST_SEQUENCE_ERROR);
  if (len < 3 || len > BLOCK_DATA_SIZE + 2)
    return negative(resp, req[0], nrc::INCORRECT_LENGTH);

  const uint8_t block = req[1];
  resp[0] = req[0] + 0x40;
  resp[1] = block;

  // The tester resends a block if our response got lost. Ack it again
  // without writing it twice.
  if (blocks_accepted > 0 && block == uint8_t(next_block - 1))
    return 2;

  if (block != next_block)
    return negative(resp, req[0], nrc::WRONG_BLOCK_SEQUENCE_COUNTER);

  const size_t data_len = len - 2;
  if (received + data_len > expected)
    return negative(resp, req[0], nrc::TRANSFER_DATA_SUSPENDED);

  if (!stage(req + 2, data_len)) {
    abandon_download();
    return negative(resp, req[0], nrc::GENERAL_PROGRAMMING_FAILURE);
  }

  received += data_len;
  blocks_accepted++;
  next_block++; // wraps 0xFF -> 0x00 as the spec says
  return 2;
}

size_t UdsServer::transfer_exit(const uint8_t *req, size_t len, uint8_t *resp) {
  if (len != 1)
    return negative(resp, req[0], nrc::INCORRECT_LENGTH);
  if (!downloading || received != expected)
    return negative(resp, req[0], nrc::REQUEST_SEQUENCE_ERROR);

  download_complete = true;
  resp[0] = req[0] + 0x40;
  return 1;
}

ImageError UdsServer::verify_staged() {
  uint8_t head[IMAGE_HEADER_SIZE];
  staging.read(0, head, sizeof(head));

  ImageError err = parse_header(head, received, staging.size(), &header);
  if (err != ImageError::OK)
    return err;

  // CRC what's actually in flash, not what we meant to write
  uint32_t crc = 0;
  uint8_t chunk[256];
  for (uint32_t off = IMAGE_HEADER_SIZE; off < received; off += sizeof(chunk)) {
    size_t n = received - off < sizeof(chunk) ? received - off : sizeof(chunk);
    staging.read(off, chunk, n);
    crc = crc32(chunk, n, crc);
  }
  return crc == header.image_crc32 ? ImageError::OK : ImageError::BAD_IMAGE_CRC;
}

size_t UdsServer::routine_control(const uint8_t *req, size_t len,
                                  uint8_t *resp) {
  // 31 01 <routine:2>, only "start" is supported
  if (len != 4)
    return negative(resp, req[0], nrc::INCORRECT_LENGTH);
  if (req[1] != 0x01)
    return negative(resp, req[0], nrc::SUBFUNCTION_NOT_SUPPORTED);
  if (uint16_t((req[2] << 8) | req[3]) != ROUTINE_CHECK_IMAGE)
    return negative(resp, req[0], nrc::REQUEST_OUT_OF_RANGE);
  if (!download_complete)
    return negative(resp, req[0], nrc::REQUEST_SEQUENCE_ERROR);

  image_error = verify_staged();
  verified = image_error == ImageError::OK;

  // Positive response either way, with the result as routine status
  resp[0] = req[0] + 0x40;
  resp[1] = req[1];
  resp[2] = req[2];
  resp[3] = req[3];
  resp[4] = uint8_t(image_error);
  return 5;
}

size_t UdsServer::ecu_reset(const uint8_t *req, size_t len, uint8_t *resp) {
  if (len != 2)
    return negative(resp, req[0], nrc::INCORRECT_LENGTH);
  const uint8_t sub = req[1] & ~SUPPRESS_POSITIVE_RESPONSE;
  if (sub != 0x01)
    return negative(resp, req[0], nrc::SUBFUNCTION_NOT_SUPPORTED);

  if (verified) {
    // Last chance to back out: never start overwriting the running firmware
    // unless the car is parked
    if (!platform.safe_to_program())
      return negative(resp, req[0], nrc::CONDITIONS_NOT_CORRECT);
    reset = PendingReset::APPLY;
  } else {
    reset = PendingReset::REBOOT;
  }

  if (req[1] & SUPPRESS_POSITIVE_RESPONSE)
    return 0;
  resp[0] = req[0] + 0x40;
  resp[1] = sub;
  return 2;
}

void UdsServer::do_pending_reset() {
  PendingReset r = reset;
  reset = PendingReset::NONE;

  if (r == PendingReset::APPLY && verified && platform.safe_to_program()) {
    verified = false;
    session = Session::DEFAULT;
    platform.apply_and_reboot(IMAGE_HEADER_SIZE, header.image_size);
  } else if (r != PendingReset::NONE) {
    end_session();
    platform.reboot();
  }
}

} // namespace ota
