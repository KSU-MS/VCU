#pragma once
#include <atomic>
#include <stddef.h>
#include <stdint.h>

// TODO: Add in the MCP2515 CAN fella
enum can_controller { TEENSY_CAN1 = 0, TEENSY_CAN2 = 1, TEENSY_CAN3 = 2 };

// TODO: Maybe add more of the CAN message settings to this fella
struct can_message {
  uint32_t id = 0;
  uint8_t length = 8;
  union {
    uint8_t byte[8];
    uint64_t val = 0;
  } buf;
  bool is_extended = false;
};

// Frames received in the FlexCAN interrupt, waiting for the main loop.
//
// FlexCAN_T4 in polling mode only has 4 receive mailboxes for standard IDs,
// emptied by read() once per loop, so a 5th frame arriving within one loop
// was lost. The interrupt now moves each frame here as it arrives; the loop
// reads them at its own pace.
//
// One producer (the ISR) and one consumer (the loop) on a single core, so
// no locks: each side only writes its own index, and a fence keeps the
// frame copy ordered before the index update.
template <size_t N> class CanRxQueue {
  static_assert(N && (N & (N - 1)) == 0, "N must be a power of two");

public:
  // ISR side. A full queue drops the new frame and counts it.
  bool push(const can_message &msg) {
    uint32_t h = head;
    if (h - tail == N) {
      dropped_count = dropped_count + 1;
      return false;
    }
    frames[h % N] = msg;
    std::atomic_signal_fence(std::memory_order_release);
    head = h + 1;
    return true;
  }

  // Loop side
  bool pop(can_message *out) {
    uint32_t t = tail;
    if (t == head)
      return false;
    std::atomic_signal_fence(std::memory_order_acquire);
    *out = frames[t % N];
    std::atomic_signal_fence(std::memory_order_release);
    tail = t + 1;
    return true;
  }

  size_t size() const { return head - tail; }
  uint32_t dropped() const { return dropped_count; }

private:
  can_message frames[N];
  volatile uint32_t head = 0; // written by push only
  volatile uint32_t tail = 0; // written by pop only
  volatile uint32_t dropped_count = 0;
};

// Per bus. A full 8-byte frame takes ~130 us at 1 Mbit/s, so this is ~33 ms
// of back-to-back traffic the loop can fall behind by without losing any.
constexpr size_t CAN_RX_QUEUE_SIZE = 256;

// TODO: Make this a template function type beat
class canMan {
private:
  bool (*controller_has_new_msg)(can_message *);
  void (*send_controller_new_msg)(can_message *);

  can_message controller_message;

public:
  canMan(can_controller target_controller, int target_baud);

  inline bool check_controller_message() {
    return this->controller_has_new_msg(&controller_message);
  }

  inline can_message get_controller_message() {
    return this->controller_message;
  }

  inline void send_controller_message(can_message out_msg) {
    this->send_controller_new_msg(&out_msg);
  }
};

// TODO: Sadge
#if defined(__IMXRT1062__)
#include <FlexCAN_T4.h>
#endif

void init_flexcan_1(int target_baud);
void init_flexcan_2(int target_baud);
void init_flexcan_3(int target_baud);

bool flexcan_1_has_new_msg(can_message *message_out);
bool flexcan_2_has_new_msg(can_message *message_out);
bool flexcan_3_has_new_msg(can_message *message_out);

void flexcan_1_send_msg(can_message *message_out);
void flexcan_2_send_msg(can_message *message_out);
void flexcan_3_send_msg(can_message *message_out);
