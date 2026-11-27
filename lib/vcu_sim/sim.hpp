#pragma once
// Simulated hardware for native builds. Provides the hal:: functions and the
// flexcan_N_* hooks that ksu-fw-common's canMan calls, so the firmware's own
// canMan/VCU/Inverter code runs unmodified on top of it.
#include <can_tools.hpp>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace sim {

struct Bus {
  int baud = 0;
  bool initialized = false;
  // Frames waiting for the VCU to read: the same queue the FlexCAN receive
  // interrupt fills on the real board, so it overflows the same way
  std::unique_ptr<CanRxQueue<CAN_RX_QUEUE_SIZE>> rx =
      std::make_unique<CanRxQueue<CAN_RX_QUEUE_SIZE>>();
  std::vector<can_message> tx; // everything the VCU has sent

  // A frame arriving on the bus (what the interrupt does on target)
  void inject(const can_message &msg) { rx->push(msg); }
  size_t rx_waiting() const { return rx->size(); }
  uint32_t rx_dropped() const { return rx->dropped(); }

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
