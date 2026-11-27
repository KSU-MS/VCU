// Firmware update over CAN, end to end on the simulated car: a tester on the
// DAQ bus talks UDS over ISO-TP to the VCU, which stages the image in
// (simulated NOR) flash and applies it.
//
// The property that matters most: the running firmware is only ever touched
// by a verified image, on a parked car. Everything else may fail, but must
// fail back to the old firmware still being there.
#include <sim_test_main.hpp>

#include <algorithm>
#include <monocypher-ed25519.h>
#include <random>
#include <sim_car.hpp>
#include <string.h>

using sim::Car;
using Bytes = std::vector<uint8_t>;

namespace {

Bytes random_firmware(size_t size, uint32_t seed = 1) {
  std::mt19937 rng(seed);
  Bytes out(size);
  for (auto &b : out)
    b = uint8_t(rng());
  return out;
}

struct BufferImage : ota::ImageReader {
  const Bytes &bytes;
  explicit BufferImage(const Bytes &bytes) : bytes(bytes) {}
  void read(uint32_t offset, uint8_t *out, size_t len) const override {
    memcpy(out, bytes.data() + offset, len);
  }
};

// The full check the VCU does on a staged image, against the built-in key
ota::ImageError verify(const Bytes &img, uint32_t load_address,
                       ota::ImageHeader *h) {
  return ota::verify_image(BufferImage(img), img.size(), 1 << 20, load_address,
                           ota::IMAGE_PUBLIC_KEY, h);
}

// The car as it'll usually be on the bench: GLV on, TS off
void park(Car &car) {
  car.run_for(10);
  REQUIRE(car.vcu.get_current_state() == TRACTIVE_SYSTEM_DISABLED);
  car.board.firmware = random_firmware(1000, 99); // "old" firmware
}

bool positive(const Bytes &r, uint8_t sid) { return !r.empty() && r[0] == sid + 0x40; }

uint8_t nrc_of(const Bytes &r) {
  return r.size() == 3 && r[0] == 0x7F ? r[2] : 0;
}

Bytes request_download(uint32_t size) {
  return {0x34, 0x00, 0x44, 0, 0, 0, 0, uint8_t(size >> 24),
          uint8_t(size >> 16), uint8_t(size >> 8), uint8_t(size)};
}

Bytes transfer_data(uint8_t counter, const Bytes &image, size_t off, size_t n) {
  Bytes td = {0x36, counter};
  td.insert(td.end(), image.begin() + off, image.begin() + off + n);
  return td;
}

// Programming session + RequestDownload for `image`
void start_download(Car &car, const Bytes &image) {
  REQUIRE(positive(car.uds({0x10, 0x02}), 0x10));
  REQUIRE(positive(car.uds(request_download(image.size())), 0x34));
}

} // namespace

//
//// Image format

TEST_CASE("crc32 matches the standard check value") {
  const char *s = "123456789";
  CHECK(ota::crc32(reinterpret_cast<const uint8_t *>(s), 9) == 0xCBF43926);
}

TEST_CASE("image matches what tools/ota/image.py builds") {
  // Same golden bytes as tools/ota/test_image.py. Ed25519 is deterministic,
  // so the signature (dev key) is fixed too.
  Bytes fw;
  for (int i = 0; i < 4; i++)
    for (int b = 0; b < 256; b++)
      fw.push_back(uint8_t(b));
  const Bytes golden_header = {
      0x4b, 0x56, 0x43, 0x55, 0x02, 0x00, 0x40, 0x00, 0x00, 0x04, 0x00,
      0x00, 0x26, 0x4c, 0x0b, 0xb7, 0xef, 0xbe, 0xad, 0xde, 0x74, 0x65,
      0x65, 0x6e, 0x73, 0x79, 0x34, 0x31, 0x2d, 0x76, 0x63, 0x75, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x02, 0x60,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0xee, 0xdf, 0x7f, 0xf8};
  const Bytes golden_signature = {
      0xea, 0x17, 0x12, 0x65, 0xb8, 0xc2, 0x8a, 0xc9, 0xc1, 0xb7, 0x22,
      0x6d, 0x97, 0x9f, 0x0c, 0x00, 0x53, 0xed, 0xaa, 0x12, 0xaa, 0x68,
      0xed, 0x58, 0xb8, 0xdb, 0x1d, 0x4b, 0x52, 0x1f, 0x6c, 0xd1, 0xba,
      0x2b, 0x3e, 0xf9, 0x9e, 0x28, 0xc4, 0x7c, 0x97, 0xe4, 0x59, 0x8a,
      0x29, 0x4d, 0xf8, 0x77, 0x76, 0xf7, 0xd3, 0x69, 0x2a, 0x51, 0xe4,
      0x6f, 0xd6, 0xf4, 0x1c, 0xbf, 0xfd, 0xa5, 0xe7, 0x08};

  Bytes img = sim::make_image(fw, 0xDEADBEEF, ota::IMAGE_TARGET, 0x60021000);
  CHECK(Bytes(img.begin(), img.begin() + 64) == golden_header);
  CHECK(Bytes(img.begin() + 64, img.begin() + 128) == golden_signature);

  ota::ImageHeader h;
  CHECK(verify(img, 0x60021000, &h) == ota::ImageError::OK);
  CHECK(h.load_address == 0x60021000);
}

TEST_CASE("the dev signing key is the one native builds trust") {
  uint8_t seed[32], secret[64], pub[32];
  memcpy(seed, sim::DEV_SIGNING_SEED, sizeof(seed));
  crypto_ed25519_key_pair(secret, pub, seed);
  CHECK(memcmp(pub, ota::IMAGE_PUBLIC_KEY, 32) == 0);
}

TEST_CASE("signature check") {
  Bytes fw = random_firmware(5000);
  ota::ImageHeader h;

  SUBCASE("signed with our key") {
    CHECK(verify(sim::make_image(fw, 1), 0x60000000, &h) == ota::ImageError::OK);
  }
  SUBCASE("signed with someone else's key") {
    uint8_t stranger[32];
    for (int i = 0; i < 32; i++)
      stranger[i] = uint8_t(i * 7 + 3);
    Bytes img = sim::make_image(fw, 1, ota::IMAGE_TARGET, 0x60000000, stranger);
    CHECK(verify(img, 0x60000000, &h) == ota::ImageError::BAD_SIGNATURE);
  }
  SUBCASE("firmware changed after signing, CRC fixed up") {
    Bytes img = sim::make_image(fw, 1);
    img[ota::IMAGE_FIRMWARE_OFFSET + 100] ^= 0x01;
    ota::ImageHeader fixed;
    memcpy(&fixed, img.data(), sizeof(fixed));
    fixed.image_crc32 = ota::crc32(img.data() + ota::IMAGE_FIRMWARE_OFFSET,
                                   fw.size());
    fixed.header_crc32 = ota::crc32(reinterpret_cast<uint8_t *>(&fixed),
                                    offsetof(ota::ImageHeader, header_crc32));
    memcpy(img.data(), &fixed, sizeof(fixed));
    CHECK(verify(img, 0x60000000, &h) == ota::ImageError::BAD_SIGNATURE);
  }
  SUBCASE("signature bytes damaged") {
    Bytes img = sim::make_image(fw, 1);
    img[ota::IMAGE_SIGNATURE_OFFSET + 5] ^= 0x80;
    CHECK(verify(img, 0x60000000, &h) == ota::ImageError::BAD_SIGNATURE);
  }
  SUBCASE("linked for somewhere else") {
    Bytes img = sim::make_image(fw, 1, ota::IMAGE_TARGET, 0x60201000);
    CHECK(verify(img, 0x60021000, &h) == ota::ImageError::WRONG_LOAD_ADDRESS);
  }
  SUBCASE("load address rewritten in the header, CRC fixed up") {
    // The header is signed too, so a valid image for one slot can't be
    // relabelled for the other
    Bytes img = sim::make_image(fw, 1, ota::IMAGE_TARGET, 0x60201000);
    ota::ImageHeader fixed;
    memcpy(&fixed, img.data(), sizeof(fixed));
    fixed.load_address = 0x60021000;
    fixed.header_crc32 = ota::crc32(reinterpret_cast<uint8_t *>(&fixed),
                                    offsetof(ota::ImageHeader, header_crc32));
    memcpy(img.data(), &fixed, sizeof(fixed));
    CHECK(verify(img, 0x60021000, &h) == ota::ImageError::BAD_SIGNATURE);
  }
}

TEST_CASE("image header validation") {
  Bytes fw = random_firmware(5000);
  Bytes img = sim::make_image(fw, 0xABCD1234);
  ota::ImageHeader h;

  SUBCASE("good") {
    CHECK(ota::parse_header(img.data(), img.size(), 1 << 20, &h) ==
          ota::ImageError::OK);
    CHECK(h.fw_version == 0xABCD1234);
    CHECK(h.image_size == fw.size());
  }
  SUBCASE("truncated") {
    CHECK(ota::parse_header(img.data(), 40, 1 << 20, &h) ==
          ota::ImageError::TOO_SHORT);
    CHECK(verify(Bytes(img.begin(), img.begin() + 2000), 0x60000000, &h) ==
          ota::ImageError::TOO_SHORT);
  }
  SUBCASE("not an image") {
    img[0] = 'X';
    CHECK(ota::parse_header(img.data(), img.size(), 1 << 20, &h) ==
          ota::ImageError::BAD_MAGIC);
  }
  SUBCASE("header corrupted") {
    img[20] ^= 0x01; // inside fw_version
    CHECK(ota::parse_header(img.data(), img.size(), 1 << 20, &h) ==
          ota::ImageError::BAD_HEADER_CRC);
  }
  SUBCASE("for another board") {
    img = sim::make_image(fw, 1, "teensy41-dash");
    CHECK(ota::parse_header(img.data(), img.size(), 1 << 20, &h) ==
          ota::ImageError::WRONG_TARGET);
  }
  SUBCASE("missing bytes") {
    CHECK(ota::parse_header(img.data(), img.size() - 1, 1 << 20, &h) ==
          ota::ImageError::SIZE_MISMATCH);
  }
  SUBCASE("too big for the board") {
    CHECK(ota::parse_header(img.data(), img.size(), 4096, &h) ==
          ota::ImageError::TOO_BIG);
  }
}

//
//// Happy path

TEST_CASE("flashes a new image and reboots into it") {
  Car car;
  park(car);

  Bytes fw = random_firmware(120 * 1024); // about the size of the VCU today
  CHECK(car.flash(sim::make_image(fw, 0xC0FFEE01)) == nullptr);

  CHECK(car.board.applies == 1);
  CHECK(car.board.firmware == fw);
  CHECK(car.board.version == 0xC0FFEE01);
  CHECK(car.staging.unerased_writes == 0);
}

TEST_CASE("reports the running version") {
  Car car;
  park(car);
  car.board.version = 0x12345678;
  CHECK(car.uds({0x22, 0xF1, 0x89}) == Bytes{0x62, 0xF1, 0x89, 0x12, 0x34, 0x56, 0x78});
}

TEST_CASE("block counter wraps from 0xFF to 0x00") {
  // > 256 blocks of 1 KB
  Car car;
  park(car);
  Bytes fw = random_firmware(300 * 1024, 7);
  CHECK(car.flash(sim::make_image(fw, 2)) == nullptr);
  CHECK(car.board.firmware == fw);
}

TEST_CASE("a resent block is acked again but not written twice") {
  // What happens when our response is lost and the tester retries
  Car car;
  park(car);
  Bytes img = sim::make_image(random_firmware(3000), 3);
  start_download(car, img);

  CHECK(positive(car.uds(transfer_data(1, img, 0, 1024)), 0x36));
  CHECK(positive(car.uds(transfer_data(1, img, 0, 1024)), 0x36));
  CHECK(car.updater.server().bytes_received() == 1024);

  // and the rest as normal
  uint8_t counter = 2;
  for (size_t off = 1024; off < img.size(); off += 1024, counter++) {
    size_t n = std::min<size_t>(1024, img.size() - off);
    CHECK(positive(car.uds(transfer_data(counter, img, off, n)), 0x36));
  }
  CHECK(positive(car.uds({0x37}), 0x37));
  Bytes r = car.uds({0x31, 0x01, 0xFF, 0x01});
  REQUIRE(r.size() == 5);
  CHECK(r[4] == 0);
  CHECK(car.staging.unerased_writes == 0);
}

TEST_CASE("flow control keeps bursts within the receive queue") {
  // Renode running the real firmware found that a 1 KB TransferData sent
  // as one ISO-TP burst overran FlexCAN_T4's 4 standard-ID mailboxes. Frames
  // now go through ksu-fw-common's receive queue, and the VCU's flow control
  // has to make the tester pause well before that fills.
  Car car;
  park(car);
  start_download(car, sim::make_image(random_firmware(2000), 4));

  size_t cursor = car.daq().tx.size();
  car.uds(transfer_data(1, sim::make_image(random_firmware(2000), 4), 0, 1024));

  const can_message *fc = nullptr;
  for (size_t i = cursor; i < car.daq().tx.size(); i++)
    if (car.daq().tx[i].id == ota::RESPONSE_ID &&
        (car.daq().tx[i].buf.byte[0] >> 4) == 0x3) {
      fc = &car.daq().tx[i];
      break;
    }
  REQUIRE(fc != nullptr);
  const uint8_t block_size = fc->buf.byte[1];
  CHECK(block_size > 0);
  CHECK(block_size <= CAN_RX_QUEUE_SIZE / 2);
  CHECK(car.daq().rx_dropped() == 0);
}

TEST_CASE("the car keeps running normally during a download") {
  Car car;
  park(car);
  start_download(car, sim::make_image(random_firmware(50 * 1024), 4));

  size_t before = car.inv().sent(CAN_ID_M192_COMMAND_MESSAGE).size();
  car.run_for(1000);
  CHECK(car.inv().sent(CAN_ID_M192_COMMAND_MESSAGE).size() - before >= 9);
  CHECK(car.last_reported_state() == TRACTIVE_SYSTEM_DISABLED);
}

//
//// Safety interlock

TEST_CASE("won't enter programming unless the car is parked") {
  Car car;

  SUBCASE("energized") {
    car.run_for(10);
    car.make_ts_healthy();
    car.run_for(10);
    REQUIRE(car.vcu.get_current_state() == TRACTIVE_SYSTEM_ENERGIZED);
  }
  SUBCASE("ready to drive") {
    car.drive_to_rtd();
    REQUIRE(car.vcu.get_current_state() == READY_TO_DRIVE);
  }

  CHECK(nrc_of(car.uds({0x10, 0x02})) == ota::nrc::CONDITIONS_NOT_CORRECT);
  CHECK_FALSE(car.updater.programming());
}

TEST_CASE("holds the car in TS_DISABLED while programming") {
  Car car;
  park(car);
  REQUIRE(positive(car.uds({0x10, 0x02}), 0x10));

  car.make_ts_healthy();
  car.run_for(1000);
  CHECK(car.vcu.get_current_state() == TRACTIVE_SYSTEM_DISABLED);
  CHECK_FALSE(car.ever_reached(TRACTIVE_SYSTEM_ENERGIZED));

  SUBCASE("until the tester leaves the session") {
    REQUIRE(positive(car.uds({0x10, 0x01}), 0x10));
  }
  SUBCASE("or the session times out") {
    car.run_for(ota::SESSION_TIMEOUT_MS + 100);
  }

  car.make_ts_healthy();
  car.run_for(20);
  CHECK(car.vcu.get_current_state() == TRACTIVE_SYSTEM_ENERGIZED);
}

TEST_CASE("TesterPresent keeps the session open") {
  Car car;
  park(car);
  REQUIRE(positive(car.uds({0x10, 0x02}), 0x10));
  for (int i = 0; i < 5; i++) {
    car.run_for(2000);
    CHECK(positive(car.uds({0x3E, 0x00}), 0x3E));
  }
  CHECK(car.updater.programming());
}

TEST_CASE("won't apply if the car stops being safe before the reset") {
  Car car;
  park(car);
  Bytes old = car.board.firmware;
  Bytes img = sim::make_image(random_firmware(4000), 5);
  start_download(car, img);
  for (size_t off = 0, n = 1; off < img.size(); off += 1024, n++)
    REQUIRE(positive(car.uds(transfer_data(n, img, off, std::min<size_t>(1024, img.size() - off))), 0x36));
  REQUIRE(positive(car.uds({0x37}), 0x37));
  REQUIRE(positive(car.uds({0x31, 0x01, 0xFF, 0x01}), 0x31));

  car.board.safe = [] { return false; };
  CHECK(nrc_of(car.uds({0x11, 0x01})) == ota::nrc::CONDITIONS_NOT_CORRECT);
  car.run_for(100);
  CHECK(car.board.applies == 0);
  CHECK(car.board.firmware == old);
}

//
//// Protocol errors

TEST_CASE("download services need a programming session") {
  Car car;
  park(car);
  CHECK(nrc_of(car.uds(request_download(1000))) == ota::nrc::NOT_SUPPORTED_IN_SESSION);
  CHECK(nrc_of(car.uds({0x36, 0x01, 0x00})) == ota::nrc::NOT_SUPPORTED_IN_SESSION);
  CHECK(nrc_of(car.uds({0x37})) == ota::nrc::NOT_SUPPORTED_IN_SESSION);
  CHECK(nrc_of(car.uds({0x31, 0x01, 0xFF, 0x01})) == ota::nrc::NOT_SUPPORTED_IN_SESSION);
}

TEST_CASE("rejects unsupported services and DIDs") {
  Car car;
  park(car);
  CHECK(nrc_of(car.uds({0x27, 0x01})) == ota::nrc::SERVICE_NOT_SUPPORTED);
  CHECK(nrc_of(car.uds({0x22, 0x12, 0x34})) == ota::nrc::REQUEST_OUT_OF_RANGE);
  CHECK(nrc_of(car.uds({0x10, 0x03})) == ota::nrc::SUBFUNCTION_NOT_SUPPORTED);
}

TEST_CASE("rejects a download bigger than the staging area") {
  Car car;
  park(car);
  REQUIRE(positive(car.uds({0x10, 0x02}), 0x10));
  CHECK(nrc_of(car.uds(request_download(2 * 1024 * 1024))) ==
        ota::nrc::UPLOAD_DOWNLOAD_NOT_ACCEPTED);
}

TEST_CASE("rejects out of order blocks") {
  Car car;
  park(car);
  Bytes img = sim::make_image(random_firmware(5000), 6);
  start_download(car, img);
  CHECK(nrc_of(car.uds(transfer_data(2, img, 0, 1024))) ==
        ota::nrc::WRONG_BLOCK_SEQUENCE_COUNTER);
}

TEST_CASE("rejects more data than was requested") {
  Car car;
  park(car);
  Bytes img = sim::make_image(random_firmware(500), 6); // 4096 + 500 bytes
  start_download(car, img);
  for (uint8_t block = 1; block <= 4; block++)
    REQUIRE(positive(car.uds(transfer_data(block, img, (block - 1) * 1024, 1024)), 0x36));
  // 500 bytes left, the tester sends a full block
  Bytes big = img;
  big.resize(5 * 1024, 0);
  CHECK(nrc_of(car.uds(transfer_data(5, big, 4096, 1024))) ==
        ota::nrc::TRANSFER_DATA_SUSPENDED);
}

TEST_CASE("won't finish a download that's missing data") {
  Car car;
  park(car);
  Bytes img = sim::make_image(random_firmware(5000), 6);
  start_download(car, img);
  REQUIRE(positive(car.uds(transfer_data(1, img, 0, 1024)), 0x36));
  CHECK(nrc_of(car.uds({0x37})) == ota::nrc::REQUEST_SEQUENCE_ERROR);
}

//
//// Bad images never get applied

TEST_CASE("a bad image is caught by the check and never applied") {
  Car car;
  park(car);
  Bytes old = car.board.firmware;
  Bytes fw = random_firmware(20000, 8);
  Bytes img = sim::make_image(fw, 9);
  ota::ImageError expected;

  SUBCASE("corrupted in transit") {
    img[10000] ^= 0x40;
    expected = ota::ImageError::BAD_IMAGE_CRC;
  }
  SUBCASE("built for another board") {
    img = sim::make_image(fw, 9, "teensy41-dash");
    expected = ota::ImageError::WRONG_TARGET;
  }
  SUBCASE("not an image at all") {
    img = random_firmware(20000, 10);
    expected = ota::ImageError::BAD_MAGIC;
  }

  CHECK(car.flash(img) != nullptr);
  CHECK(car.updater.server().last_image_error() == expected);

  // Even if the tester ignores the failed check and resets anyway
  CHECK(positive(car.uds({0x11, 0x01}), 0x11));
  car.run_for(10);
  CHECK(car.board.applies == 0);
  CHECK(car.board.reboots >= 1);
  CHECK(car.board.firmware == old);
}

TEST_CASE("a flash write failure aborts the download") {
  Car car;
  park(car);
  Bytes old = car.board.firmware;
  Bytes img = sim::make_image(random_firmware(10000), 11);
  car.staging.fail_after_bytes = 5000;

  start_download(car, img);
  Bytes r;
  for (size_t off = 0, n = 1; off < img.size(); off += 1024, n++) {
    r = car.uds(transfer_data(n, img, off, std::min<size_t>(1024, img.size() - off)));
    if (!positive(r, 0x36))
      break;
  }
  CHECK(nrc_of(r) == ota::nrc::GENERAL_PROGRAMMING_FAILURE);
  CHECK(car.board.firmware == old);

  // And it recovers once the flash behaves again
  car.staging.fail_after_bytes = SIZE_MAX;
  CHECK(car.flash(img) == nullptr);
  CHECK(car.board.applies == 1);
}

TEST_CASE("giving up at any point leaves the old firmware in place") {
  // Stop after every step of the sequence in turn and let the session time
  // out, then check a full update still works from scratch afterwards
  Bytes fw = random_firmware(6000, 12);
  Bytes img = sim::make_image(fw, 13);
  const size_t blocks = (img.size() + 1023) / 1024;

  for (size_t stop_after = 0; stop_after <= blocks + 2; stop_after++) {
    CAPTURE(stop_after);
    Car car;
    park(car);
    Bytes old = car.board.firmware;

    size_t steps = 0;
    auto step = [&](const Bytes &req, uint8_t sid) {
      if (steps++ >= stop_after)
        return false;
      REQUIRE(positive(car.uds(req), sid));
      return true;
    };

    start_download(car, img);
    bool done = true;
    for (size_t off = 0, n = 1; off < img.size() && done; off += 1024, n++)
      done = step(transfer_data(n, img, off, std::min<size_t>(1024, img.size() - off)), 0x36);
    done = done && step({0x37}, 0x37);
    done = done && step({0x31, 0x01, 0xFF, 0x01}, 0x31);

    car.run_for(ota::SESSION_TIMEOUT_MS + 100);
    CHECK_FALSE(car.updater.programming());
    CHECK(car.board.applies == 0);
    CHECK(car.board.firmware == old);

    CHECK(car.flash(img) == nullptr);
    CHECK(car.board.firmware == fw);
  }
}
