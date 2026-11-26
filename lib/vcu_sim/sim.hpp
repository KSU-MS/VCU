#pragma once
// Simulated hardware for native builds. Provides the hal:: functions and the
// flexcan_N_* hooks that ksu-fw-common's canMan calls, so the firmware's own
// canMan/VCU/Inverter code runs unmodified on top of it.
#include <can_tools.hpp>
#include <cstdint>
#include <deque>
#include <map>
#include <string>
#include <vector>

namespace sim {

struct Bus {
  int baud = 0;
  bool initialized = false;
  std::deque<can_message> rx; // frames waiting for the VCU to read
  std::vector<can_message> tx; // everything the VCU has sent

  // ksu-fw-common runs FlexCAN_T4 in polling mode with its default layout:
  // mailboxes 0-3 take standard IDs, 4-7 extended, 8-15 transmit. So only
  // 4 standard frames can wait between reads; more are lost on the real
  // board, and here too.
  size_t rx_mailboxes = 4;
  size_t rx_dropped = 0;

  void inject(const can_message &msg) {
    if (rx.size() >= rx_mailboxes) {
      rx_dropped++;
      return;
    }
    rx.push_back(msg);
  }

  // Frames sent with a given ID, oldest first
  std::vector<can_message> sent(uint32_t id) const;
  // Most recent frame sent with a given ID, nullptr if none
  const can_message *last_sent(uint32_t id) const;
};

// TEENSY_CAN1/2/3 -> ACC / INV / DAQ in this car
Bus &bus(can_controller controller);

uint32_t now();
void set_now(uint32_t ms);
void advance(uint32_t ms);

bool pin(uint8_t pin);
bool pin_is_output(uint8_t pin);

// Everything passed to hal::log since the last reset
const std::string &log_output();
// Mirror hal::log to stdout (off by default to keep test output readable)
void set_echo_log(bool echo);

// Wipe clock, pins, buses and log back to power-on state
void reset();

} // namespace sim
