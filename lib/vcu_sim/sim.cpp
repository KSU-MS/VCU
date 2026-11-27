#include "sim.hpp"
#include <cstdio>
#include <hal.hpp>

namespace {
uint32_t clock_ms = 0;
std::map<uint8_t, bool> pin_levels;
std::map<uint8_t, bool> pin_outputs;
std::string log_buffer;
bool echo_log = false;
sim::Bus buses[3];

void init_bus(sim::Bus &b, int baud) {
  b.baud = baud;
  b.initialized = true;
}

bool read_bus(sim::Bus &b, can_message *out) { return b.rx->pop(out); }
} // namespace

namespace sim {

std::vector<can_message> Bus::sent(uint32_t id) const {
  std::vector<can_message> out;
  for (const auto &m : tx)
    if (m.id == id)
      out.push_back(m);
  return out;
}

const can_message *Bus::last_sent(uint32_t id) const {
  for (auto it = tx.rbegin(); it != tx.rend(); ++it)
    if (it->id == id)
      return &*it;
  return nullptr;
}

Bus &bus(can_controller controller) { return buses[controller]; }

uint32_t now() { return clock_ms; }
void set_now(uint32_t ms) { clock_ms = ms; }
void advance(uint32_t ms) { clock_ms += ms; }

bool pin(uint8_t p) { return pin_levels[p]; }
bool pin_is_output(uint8_t p) { return pin_outputs[p]; }

const std::string &log_output() { return log_buffer; }
void set_echo_log(bool echo) { echo_log = echo; }

void reset() {
  clock_ms = 0;
  pin_levels.clear();
  pin_outputs.clear();
  log_buffer.clear();
  for (auto &b : buses)
    b = Bus{};
}

} // namespace sim

//
//// hal:: backed by the simulator
namespace hal {

uint32_t millis() { return clock_ms; }

void pin_mode(uint8_t pin, pin_dir dir) { pin_outputs[pin] = dir == PIN_OUTPUT; }
void digital_write(uint8_t pin, bool level) { pin_levels[pin] = level; }

void log(const char *msg) {
  log_buffer += msg;
  if (echo_log)
    fputs(msg, stdout);
}
void log(long value) { log(std::to_string(value).c_str()); }

} // namespace hal

//
//// The hooks can_tools.cpp expects; on target these wrap FlexCAN_T4
void init_flexcan_1(int baud) { init_bus(buses[0], baud); }
void init_flexcan_2(int baud) { init_bus(buses[1], baud); }
void init_flexcan_3(int baud) { init_bus(buses[2], baud); }

bool flexcan_1_has_new_msg(can_message *out) { return read_bus(buses[0], out); }
bool flexcan_2_has_new_msg(can_message *out) { return read_bus(buses[1], out); }
bool flexcan_3_has_new_msg(can_message *out) { return read_bus(buses[2], out); }

void flexcan_1_send_msg(can_message *m) { buses[0].tx.push_back(*m); }
void flexcan_2_send_msg(can_message *m) { buses[1].tx.push_back(*m); }
void flexcan_3_send_msg(can_message *m) { buses[2].tx.push_back(*m); }
