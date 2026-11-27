// ksu-fw-common's CAN receive queue on its own: the FlexCAN interrupt pushes,
// the main loop pops.
#include <sim_test_main.hpp>

#include <can_tools.hpp>

namespace {

can_message frame(uint32_t id) {
  can_message m;
  m.id = id;
  m.length = 8;
  m.buf.val = 0x0102030405060708ull + id;
  return m;
}

} // namespace

TEST_CASE("frames come out in the order they arrived") {
  CanRxQueue<8> q;
  for (uint32_t id = 1; id <= 5; id++)
    REQUIRE(q.push(frame(id)));
  CHECK(q.size() == 5);

  can_message out;
  for (uint32_t id = 1; id <= 5; id++) {
    REQUIRE(q.pop(&out));
    CHECK(out.id == id);
    CHECK(out.buf.val == frame(id).buf.val);
  }
  CHECK_FALSE(q.pop(&out));
  CHECK(q.size() == 0);
}

TEST_CASE("an empty queue leaves the output alone") {
  CanRxQueue<4> q;
  can_message out = frame(42);
  CHECK_FALSE(q.pop(&out));
  CHECK(out.id == 42);
}

TEST_CASE("a full queue drops the newest frame and counts it") {
  CanRxQueue<4> q;
  for (uint32_t id = 1; id <= 4; id++)
    REQUIRE(q.push(frame(id)));
  CHECK_FALSE(q.push(frame(5)));
  CHECK_FALSE(q.push(frame(6)));
  CHECK(q.dropped() == 2);
  CHECK(q.size() == 4);

  // What's already queued survives intact
  can_message out;
  REQUIRE(q.pop(&out));
  CHECK(out.id == 1);
  // ...and there's room again
  CHECK(q.push(frame(7)));
}

TEST_CASE("slots are reused as the queue goes round") {
  // The indices are free-running uint32_t and only reduced mod N on use.
  // Because N is a power of two (asserted) it divides 2^32, so head - tail
  // and h % N stay right when they wrap too (~6 days of a busy 1 Mbit/s bus).
  CanRxQueue<4> q;
  can_message out;
  uint32_t next_in = 0, next_out = 0;
  for (int round = 0; round < 1000; round++) {
    for (int i = 0; i < 3; i++)
      REQUIRE(q.push(frame(next_in++)));
    for (int i = 0; i < 3; i++) {
      REQUIRE(q.pop(&out));
      CHECK(out.id == next_out++);
    }
  }
  CHECK(q.dropped() == 0);
}
