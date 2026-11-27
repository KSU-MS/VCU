// A/B boot logic on simulated NOR flash: the boot-state journal, the
// bootloader's decision, and whole update / rollback sequences, including
// losing power at every byte of every flash write along the way.
//
// The property that matters: after anything, there's a valid image to boot,
// and it's the one the rules say.
#include <sim_test_main.hpp>

#include <boot.hpp>
#include <random>
#include <sim_car.hpp>
#include <sim_ota.hpp>
#include <string.h>

using namespace boot;
using Bytes = std::vector<uint8_t>;

namespace {

constexpr size_t FLASH_SIZE = 8 * 1024 * 1024;

Bytes random_firmware(size_t size, uint32_t seed) {
  std::mt19937 rng(seed);
  Bytes out(size);
  for (auto &b : out)
    b = uint8_t(rng());
  return out;
}

// A signed image built for `slot`, with version `version`
Bytes image_for(uint8_t slot, uint32_t version, size_t size = 20000) {
  return sim::make_image(random_firmware(size, version), version,
                         ota::IMAGE_TARGET, slot_firmware_address(slot));
}

void write_slot(sim::NorFlash &f, uint8_t slot, const Bytes &img) {
  for (uint32_t off = 0; off < img.size(); off += SECTOR_SIZE)
    REQUIRE(f.erase_sector(SLOT_OFFSET[slot] + off));
  REQUIRE(f.program(SLOT_OFFSET[slot], img.data(), img.size()));
}

// What the bootloader does on every reset. Returns the slot it jumps to.
struct Boot {
  uint8_t slot;
  bool trial;
};
Boot boot_once(sim::NorFlash &f) {
  BootState state(f);
  state.load();
  bool valid[2] = {
      verify_slot(f, SLOT_A, ota::IMAGE_PUBLIC_KEY) == ota::ImageError::OK,
      verify_slot(f, SLOT_B, ota::IMAGE_PUBLIC_KEY) == ota::ImageError::OK};
  BootDecision d = decide(state.current(), state.has_record(), valid);
  if (d.write)
    state.write(d.active, d.trial_slot, d.attempts);
  return {d.slot, d.trial};
}

// A board fresh from a USB flash: bootloader + slot A, no boot state
struct Board {
  sim::NorFlash flash{FLASH_SIZE};
  Board() { write_slot(flash, SLOT_A, image_for(SLOT_A, 0xA1)); }

  uint32_t version_in(uint8_t slot) {
    ota::ImageHeader h;
    REQUIRE(verify_slot(flash, slot, ota::IMAGE_PUBLIC_KEY, &h) == ota::ImageError::OK);
    return h.fw_version;
  }
};

} // namespace

//
//// Journal

TEST_CASE("blank flash has no state, and means slot A") {
  sim::NorFlash f(FLASH_SIZE);
  BootState s(f);
  CHECK_FALSE(s.load());
  CHECK(s.current().active == SLOT_A);
  CHECK(s.current().trial == NO_SLOT);
}

TEST_CASE("a written state reads back after a reset") {
  sim::NorFlash f(FLASH_SIZE);
  {
    BootState s(f);
    s.load();
    REQUIRE(s.write(SLOT_A, SLOT_B, 0));
    REQUIRE(s.write(SLOT_A, SLOT_B, 1));
  }
  BootState s(f);
  REQUIRE(s.load());
  CHECK(s.current().active == SLOT_A);
  CHECK(s.current().trial == SLOT_B);
  CHECK(s.current().attempts == 1);
  CHECK(s.current().sequence == 2);
}

TEST_CASE("the journal moves between its two sectors as they fill") {
  sim::NorFlash f(FLASH_SIZE);
  BootState s(f);
  s.load();
  // 2.5 sectors' worth
  for (uint32_t i = 0; i < RECORDS_PER_SECTOR * 5 / 2; i++) {
    REQUIRE(s.write(i % 2, NO_SLOT, 0));
    BootState fresh(f);
    REQUIRE(fresh.load());
    REQUIRE(fresh.current().sequence == i + 1);
    REQUIRE(fresh.current().active == i % 2);
  }
  CHECK(f.erases == 2); // switched sector twice
}

TEST_CASE("losing power at any byte of a record keeps the old state") {
  for (size_t k = 0; k <= sizeof(BootRecord); k++) {
    CAPTURE(k);
    sim::NorFlash f(FLASH_SIZE);
    BootState s(f);
    s.load();
    REQUIRE(s.write(SLOT_A, NO_SLOT, 0));

    f.fail_after_bytes = f.bytes_programmed + k;
    bool landed = s.write(SLOT_A, SLOT_B, 0);
    f.fail_after_bytes = SIZE_MAX;

    BootState after(f);
    REQUIRE(after.load());
    if (k == sizeof(BootRecord)) {
      CHECK(landed);
      CHECK(after.current().trial == SLOT_B);
    } else {
      CHECK_FALSE(landed);
      CHECK(after.current().trial == NO_SLOT);
      CHECK(after.current().sequence == 1);
    }

    // and the journal still takes writes after the torn one
    REQUIRE(after.write(SLOT_B, NO_SLOT, 0));
    BootState again(f);
    REQUIRE(again.load());
    CHECK(again.current().active == SLOT_B);
  }
}

TEST_CASE("losing power while switching sectors keeps the old state") {
  sim::NorFlash f(FLASH_SIZE);
  BootState s(f);
  s.load();
  for (uint32_t i = 0; i < RECORDS_PER_SECTOR; i++)
    REQUIRE(s.write(SLOT_B, NO_SLOT, 0)); // sector 0 now full

  SUBCASE("erase of the other sector interrupted") {
    f.interrupt_next_erase = true;
    CHECK_FALSE(s.write(SLOT_A, NO_SLOT, 0));
  }
  SUBCASE("power lost halfway through the first record there") {
    f.fail_after_bytes = f.bytes_programmed + 10;
    CHECK_FALSE(s.write(SLOT_A, NO_SLOT, 0));
    f.fail_after_bytes = SIZE_MAX;
  }

  BootState after(f);
  REQUIRE(after.load());
  CHECK(after.current().active == SLOT_B);
  CHECK(after.current().sequence == RECORDS_PER_SECTOR);
  REQUIRE(after.write(SLOT_A, NO_SLOT, 0));
  BootState again(f);
  REQUIRE(again.load());
  CHECK(again.current().active == SLOT_A);
}

TEST_CASE("garbage in the journal is ignored") {
  sim::NorFlash f(FLASH_SIZE);
  BootState s(f);
  s.load();
  REQUIRE(s.write(SLOT_B, NO_SLOT, 0));
  // Something that isn't a record, after it
  uint8_t junk[32];
  memset(junk, 0x5A, sizeof(junk));
  REQUIRE(f.program(STATE_OFFSET + 32, junk, sizeof(junk)));

  BootState after(f);
  REQUIRE(after.load());
  CHECK(after.current().active == SLOT_B);
  REQUIRE(after.write(SLOT_A, NO_SLOT, 0)); // goes after the junk
  CHECK(f.unerased_writes == 0);
  BootState again(f);
  REQUIRE(again.load());
  CHECK(again.current().active == SLOT_A);
}

//
//// Policy

namespace {
BootRecord record(uint8_t active, uint8_t trial = NO_SLOT, uint8_t attempts = 0) {
  BootRecord r{};
  r.active = active;
  r.trial = trial;
  r.attempts = attempts;
  return r;
}
} // namespace

TEST_CASE("decide: what boots") {
  const bool both[2] = {true, true};
  const bool only_a[2] = {true, false};
  const bool only_b[2] = {false, true};
  const bool neither[2] = {false, false};

  SUBCASE("fresh from USB: A, nothing written") {
    BootDecision d = decide(record(SLOT_A), false, both);
    CHECK(d.slot == SLOT_A);
    CHECK_FALSE(d.write);
  }
  SUBCASE("confirmed B") {
    BootDecision d = decide(record(SLOT_B), true, both);
    CHECK(d.slot == SLOT_B);
    CHECK_FALSE(d.trial);
    CHECK_FALSE(d.write);
  }
  SUBCASE("first boot of a trial counts the attempt") {
    BootDecision d = decide(record(SLOT_A, SLOT_B, 0), true, both);
    CHECK(d.slot == SLOT_B);
    CHECK(d.trial);
    CHECK(d.write);
    CHECK(d.attempts == 1);
    CHECK(d.trial_slot == SLOT_B);
    CHECK(d.active == SLOT_A);
  }
  SUBCASE("trial out of attempts rolls back") {
    BootDecision d = decide(record(SLOT_A, SLOT_B, TRIAL_BOOTS), true, both);
    CHECK(d.slot == SLOT_A);
    CHECK_FALSE(d.trial);
    CHECK(d.write);
    CHECK(d.trial_slot == NO_SLOT);
  }
  SUBCASE("trial slot that doesn't verify rolls back") {
    BootDecision d = decide(record(SLOT_A, SLOT_B, 0), true, only_a);
    CHECK(d.slot == SLOT_A);
    CHECK(d.trial_slot == NO_SLOT);
  }
  SUBCASE("active slot gone bad: the other takes over") {
    BootDecision d = decide(record(SLOT_A), true, only_b);
    CHECK(d.slot == SLOT_B);
    CHECK(d.write);
    CHECK(d.active == SLOT_B);
  }
  SUBCASE("fresh board whose slot A is bad, B good") {
    BootDecision d = decide(record(SLOT_A), false, only_b);
    CHECK(d.slot == SLOT_B);
  }
  SUBCASE("nothing valid: nothing boots") {
    CHECK(decide(record(SLOT_A), true, neither).slot == NO_SLOT);
    CHECK(decide(record(SLOT_A, SLOT_B, 0), true, neither).slot == NO_SLOT);
  }
}

//
//// Slots

TEST_CASE("slot verification") {
  sim::NorFlash f(FLASH_SIZE);
  SUBCASE("an image built for the slot") {
    write_slot(f, SLOT_B, image_for(SLOT_B, 7));
    ota::ImageHeader h;
    CHECK(verify_slot(f, SLOT_B, ota::IMAGE_PUBLIC_KEY, &h) == ota::ImageError::OK);
    CHECK(h.fw_version == 7);
  }
  SUBCASE("an image built for the other slot") {
    write_slot(f, SLOT_B, image_for(SLOT_A, 7));
    CHECK(verify_slot(f, SLOT_B, ota::IMAGE_PUBLIC_KEY) ==
          ota::ImageError::WRONG_LOAD_ADDRESS);
  }
  SUBCASE("blank") {
    CHECK(verify_slot(f, SLOT_A, ota::IMAGE_PUBLIC_KEY) == ota::ImageError::BAD_MAGIC);
  }
  SUBCASE("a header claiming more than the slot holds") {
    Bytes img = image_for(SLOT_A, 7);
    ota::ImageHeader h;
    memcpy(&h, img.data(), sizeof(h));
    h.image_size = 0xFFFFFF00;
    h.header_crc32 = ota::crc32(reinterpret_cast<uint8_t *>(&h),
                                offsetof(ota::ImageHeader, header_crc32));
    memcpy(img.data(), &h, sizeof(h));
    write_slot(f, SLOT_A, img);
    CHECK(verify_slot(f, SLOT_A, ota::IMAGE_PUBLIC_KEY) != ota::ImageError::OK);
  }
  SUBCASE("one flipped bit in the firmware") {
    Bytes img = image_for(SLOT_A, 7);
    img[ota::IMAGE_FIRMWARE_OFFSET + 1234] ^= 0x04;
    write_slot(f, SLOT_A, img);
    CHECK(verify_slot(f, SLOT_A, ota::IMAGE_PUBLIC_KEY) != ota::ImageError::OK);
  }
}

//
//// Whole sequences: bootloader + firmware

TEST_CASE("a board fresh from USB boots slot A") {
  Board b;
  Boot r = boot_once(b.flash);
  CHECK(r.slot == SLOT_A);
  CHECK_FALSE(r.trial);
}

TEST_CASE("update, trial boot, confirm: the new slot sticks") {
  Board b;
  REQUIRE(boot_once(b.flash).slot == SLOT_A);

  // Firmware in A downloads B and asks for a trial
  write_slot(b.flash, SLOT_B, image_for(SLOT_B, 0xB2));
  {
    BootState s(b.flash);
    s.load();
    REQUIRE(start_trial(s, SLOT_A, SLOT_B));
  }

  Boot r = boot_once(b.flash);
  CHECK(r.slot == SLOT_B);
  CHECK(r.trial);

  // B comes up fine, the tester confirms it
  {
    BootState s(b.flash);
    s.load();
    CHECK(running_trial(s, SLOT_B));
    REQUIRE(confirm(s, SLOT_B));
    CHECK_FALSE(running_trial(s, SLOT_B));
  }

  for (int i = 0; i < 3; i++) {
    r = boot_once(b.flash);
    CHECK(r.slot == SLOT_B);
    CHECK_FALSE(r.trial);
  }
  CHECK(b.version_in(SLOT_B) == 0xB2);
}

TEST_CASE("an update that never confirms is rolled back on the next reset") {
  Board b;
  write_slot(b.flash, SLOT_B, image_for(SLOT_B, 0xB2));
  {
    BootState s(b.flash);
    s.load();
    REQUIRE(start_trial(s, SLOT_A, SLOT_B));
  }
  CHECK(boot_once(b.flash).slot == SLOT_B); // the trial
  // ...hangs, crashes, or nobody confirms it before the power cycles
  Boot r = boot_once(b.flash);
  CHECK(r.slot == SLOT_A);
  CHECK_FALSE(r.trial);
  CHECK(boot_once(b.flash).slot == SLOT_A); // and stays there
}

TEST_CASE("an update that doesn't verify is never booted") {
  Board b;
  Bytes img = image_for(SLOT_B, 0xB2);
  img[ota::IMAGE_FIRMWARE_OFFSET + 10] ^= 1;
  write_slot(b.flash, SLOT_B, img);
  {
    BootState s(b.flash);
    s.load();
    REQUIRE(start_trial(s, SLOT_A, SLOT_B));
  }
  CHECK(boot_once(b.flash).slot == SLOT_A);
}

TEST_CASE("a confirmed slot that goes bad falls back to the other") {
  Board b;
  write_slot(b.flash, SLOT_B, image_for(SLOT_B, 0xB2));
  {
    BootState s(b.flash);
    s.load();
    REQUIRE(confirm(s, SLOT_B));
  }
  // Bit rot (or a half-finished USB flash) in B
  uint8_t zero = 0;
  REQUIRE(b.flash.program(SLOT_OFFSET[SLOT_B] + ota::IMAGE_FIRMWARE_OFFSET + 99, &zero, 1));
  CHECK(boot_once(b.flash).slot == SLOT_A);
  BootState s(b.flash);
  s.load();
  CHECK(s.current().active == SLOT_A);
}

TEST_CASE("losing power anywhere while the update is written keeps slot A") {
  // Every 997th byte of the image, through erase and program
  Board b;
  Bytes img = image_for(SLOT_B, 0xB2, 12000);
  for (size_t cut = 0; cut < img.size(); cut += 997) {
    CAPTURE(cut);
    b.flash.fail_after_bytes = b.flash.bytes_programmed + cut;
    for (uint32_t off = 0; off < img.size(); off += SECTOR_SIZE)
      b.flash.erase_sector(SLOT_OFFSET[SLOT_B] + off);
    b.flash.program(SLOT_OFFSET[SLOT_B], img.data(), img.size());
    b.flash.fail_after_bytes = SIZE_MAX;
    // (the firmware never gets to start_trial: its write failed)
    Boot r = boot_once(b.flash);
    CHECK(r.slot == SLOT_A);
    CHECK_FALSE(r.trial);
  }
}

TEST_CASE("losing power while the trial or confirm is recorded is safe") {
  for (size_t k = 0; k < sizeof(BootRecord); k++) {
    CAPTURE(k);
    Board b;
    write_slot(b.flash, SLOT_B, image_for(SLOT_B, 0xB2));

    SUBCASE("starting the trial") {
      BootState s(b.flash);
      s.load();
      b.flash.fail_after_bytes = b.flash.bytes_programmed + k;
      CHECK_FALSE(start_trial(s, SLOT_A, SLOT_B));
      b.flash.fail_after_bytes = SIZE_MAX;
      // Never recorded, so A as before
      CHECK(boot_once(b.flash).slot == SLOT_A);
    }
    SUBCASE("confirming") {
      {
        BootState s(b.flash);
        s.load();
        REQUIRE(start_trial(s, SLOT_A, SLOT_B));
      }
      REQUIRE(boot_once(b.flash).slot == SLOT_B);
      BootState s(b.flash);
      s.load();
      b.flash.fail_after_bytes = b.flash.bytes_programmed + k;
      CHECK_FALSE(confirm(s, SLOT_B));
      b.flash.fail_after_bytes = SIZE_MAX;
      // Not confirmed, so it counts as a failed trial: back to A, which
      // still works. The tester sees the old version and can try again.
      CHECK(boot_once(b.flash).slot == SLOT_A);
    }
    SUBCASE("the bootloader counting the trial boot") {
      {
        BootState s(b.flash);
        s.load();
        REQUIRE(start_trial(s, SLOT_A, SLOT_B));
      }
      b.flash.fail_after_bytes = b.flash.bytes_programmed + k;
      boot_once(b.flash); // the count didn't land; the bootloader still
                          // jumps to B on real hardware
      b.flash.fail_after_bytes = SIZE_MAX;
      // Next reset: the attempt wasn't counted, so B gets its one try again
      // rather than nothing working
      Boot r = boot_once(b.flash);
      CHECK(r.slot == SLOT_B);
      CHECK(r.trial);
      CHECK(boot_once(b.flash).slot == SLOT_A);
    }
  }
}

TEST_CASE("a trial can't start from the slot it would replace") {
  Board b;
  BootState s(b.flash);
  s.load();
  CHECK_FALSE(start_trial(s, SLOT_A, SLOT_A));
}

TEST_CASE("a hundred updates back and forth") {
  Board b;
  uint8_t running = boot_once(b.flash).slot;
  for (uint32_t v = 1; v <= 100; v++) {
    uint8_t target = other_slot(running);
    write_slot(b.flash, target, image_for(target, v, 5000));
    {
      BootState s(b.flash);
      s.load();
      REQUIRE(start_trial(s, running, target));
    }
    Boot r = boot_once(b.flash);
    REQUIRE(r.slot == target);
    BootState s(b.flash);
    s.load();
    REQUIRE(confirm(s, target));
    running = target;
    REQUIRE(boot_once(b.flash).slot == running);
    REQUIRE(b.version_in(running) == v);
  }
  CHECK(b.flash.unerased_writes == 0);
}
