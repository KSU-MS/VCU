#pragma once
// The subset of UDS (ISO 14229-1) needed to download and apply an image:
//
//   10 02            DiagnosticSessionControl -> programming (only when safe)
//   3E 00            TesterPresent, keeps the session alive
//   22 F189 / F101   ReadDataByIdentifier: running version / OTA status
//   34 00 44 a4 s4   RequestDownload of s bytes (address ignored)
//   36 nn data       TransferData block nn (1, 2, ... 0xFF, 0, 1, ...)
//   37               RequestTransferExit
//   31 01 FF01       RoutineControl: check the staged image (crc etc)
//   11 01            ECUReset: apply the staged image if verified, reboot
//
// Anything else gets the matching negative response.
#include <stddef.h>
#include <stdint.h>

#include "ota_image.hpp"
#include "ota_platform.hpp"

namespace ota {

namespace sid {
constexpr uint8_t SESSION_CONTROL = 0x10;
constexpr uint8_t ECU_RESET = 0x11;
constexpr uint8_t READ_DID = 0x22;
constexpr uint8_t ROUTINE_CONTROL = 0x31;
constexpr uint8_t REQUEST_DOWNLOAD = 0x34;
constexpr uint8_t TRANSFER_DATA = 0x36;
constexpr uint8_t TRANSFER_EXIT = 0x37;
constexpr uint8_t TESTER_PRESENT = 0x3E;
constexpr uint8_t NEGATIVE_RESPONSE = 0x7F;
} // namespace sid

namespace nrc {
constexpr uint8_t SERVICE_NOT_SUPPORTED = 0x11;
constexpr uint8_t SUBFUNCTION_NOT_SUPPORTED = 0x12;
constexpr uint8_t INCORRECT_LENGTH = 0x13;
constexpr uint8_t CONDITIONS_NOT_CORRECT = 0x22;
constexpr uint8_t REQUEST_SEQUENCE_ERROR = 0x24;
constexpr uint8_t REQUEST_OUT_OF_RANGE = 0x31;
constexpr uint8_t UPLOAD_DOWNLOAD_NOT_ACCEPTED = 0x70;
constexpr uint8_t TRANSFER_DATA_SUSPENDED = 0x71;
constexpr uint8_t GENERAL_PROGRAMMING_FAILURE = 0x72;
constexpr uint8_t WRONG_BLOCK_SEQUENCE_COUNTER = 0x73;
constexpr uint8_t NOT_SUPPORTED_IN_SESSION = 0x7F;
} // namespace nrc

constexpr uint16_t DID_SOFTWARE_VERSION = 0xF189;
constexpr uint16_t DID_OTA_STATUS = 0xF101;
constexpr uint16_t ROUTINE_CHECK_IMAGE = 0xFF01;

// TransferData payload per block (the block length we advertise is this
// plus the SID and counter bytes). One flash sector = 4 blocks on Teensy.
constexpr size_t BLOCK_DATA_SIZE = 1024;

// Drop back to the default session after this long without a request
constexpr uint32_t SESSION_TIMEOUT_MS = 5000;

class UdsServer {
public:
  enum class Session : uint8_t { DEFAULT = 1, PROGRAMMING = 2 };

  UdsServer(Platform &platform, StagingFlash &staging);

  // Handle one request, writing the response into `resp` (at least 16 bytes).
  // Returns the response length, which is 0 when the request asked for no
  // positive response.
  size_t handle(const uint8_t *req, size_t len, uint8_t *resp, uint32_t now);

  // Session timeout, and checking the vehicle is still safe to program
  void poll(uint32_t now);

  // In a programming session the VCU must stay in TS_DISABLED
  bool programming() const { return session == Session::PROGRAMMING; }

  // Set by ECUReset; carried out once the response has gone out
  enum class PendingReset : uint8_t { NONE, REBOOT, APPLY };
  PendingReset pending_reset() const { return reset; }
  void do_pending_reset();

  // For the status DID and tests
  bool image_verified() const { return verified; }
  ImageError last_image_error() const { return image_error; }
  uint32_t bytes_received() const { return received; }

private:
  Platform &platform;
  StagingFlash &staging;

  Session session = Session::DEFAULT;
  uint32_t last_request_ms = 0;
  PendingReset reset = PendingReset::NONE;

  // Download in progress
  bool downloading = false;
  bool download_complete = false;
  uint32_t expected = 0;
  uint32_t received = 0;
  uint8_t next_block = 1;
  uint32_t blocks_accepted = 0;
  uint32_t erased_to = 0; // everything below this offset has been erased

  bool verified = false;
  ImageError image_error = ImageError::OK;
  ImageHeader header{};

  void end_session();
  void abandon_download();
  bool stage(const uint8_t *data, size_t len);
  ImageError verify_staged();

  size_t session_control(const uint8_t *req, size_t len, uint8_t *resp);
  size_t tester_present(const uint8_t *req, size_t len, uint8_t *resp);
  size_t read_did(const uint8_t *req, size_t len, uint8_t *resp);
  size_t request_download(const uint8_t *req, size_t len, uint8_t *resp);
  size_t transfer_data(const uint8_t *req, size_t len, uint8_t *resp);
  size_t transfer_exit(const uint8_t *req, size_t len, uint8_t *resp);
  size_t routine_control(const uint8_t *req, size_t len, uint8_t *resp);
  size_t ecu_reset(const uint8_t *req, size_t len, uint8_t *resp);
};

} // namespace ota
