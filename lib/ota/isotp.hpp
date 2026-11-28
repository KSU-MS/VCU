#pragma once
// Minimal ISO-TP (ISO 15765-2) for classic CAN with normal addressing: enough
// to carry UDS requests up to 4095 bytes in, and short responses out. Talks
// to the Linux kernel's isotp sockets and python-can-isotp.
#include <can_tools.hpp>
#include <stddef.h>
#include <stdint.h>

namespace ota {

class IsoTp {
public:
  static constexpr size_t MAX_RX = 4095; // largest classic ISO-TP message
  static constexpr size_t MAX_TX = 128;  // our responses are tiny

  struct Config {
    uint32_t tx_id;
    uint32_t rx_id;
    uint8_t block_size = 0;      // CFs between our flow controls, 0 = all
    uint8_t st_min_ms = 0;       // gap we ask the sender to leave between CFs
    uint32_t timeout_ms = 1000;  // N_Bs / N_Cr
    uint8_t padding = 0xCC;      // frames are always padded to 8 bytes
  };

  using SendFn = void (*)(const can_message &msg, void *ctx);

  IsoTp(const Config &config, SendFn send, void *ctx);

  // Feed every frame from the bus, ones for other IDs are ignored
  void on_frame(const can_message &msg, uint32_t now);
  // Timeouts and pacing of outgoing consecutive frames
  void poll(uint32_t now);

  // Queue a message. False if one is still going out or it's too long.
  bool send(const uint8_t *data, size_t len, uint32_t now);
  bool tx_busy() const { return tx_state != TxState::IDLE; }

  // A complete incoming message, held until release()
  bool available() const { return rx_state == RxState::DONE; }
  const uint8_t *data() const { return rx_buf; }
  size_t length() const { return rx_len; }
  void release() { rx_state = RxState::IDLE; }

private:
  enum class RxState : uint8_t { IDLE, RECEIVING, DONE };
  enum class TxState : uint8_t { IDLE, WAIT_FC, SENDING };

  Config cfg;
  SendFn send_fn;
  void *send_ctx;

  RxState rx_state = RxState::IDLE;
  uint8_t rx_buf[MAX_RX];
  size_t rx_len = 0;
  size_t rx_expected = 0;
  uint8_t rx_next_sn = 0;
  uint8_t rx_block_count = 0;
  uint32_t rx_deadline = 0;

  TxState tx_state = TxState::IDLE;
  uint8_t tx_buf[MAX_TX];
  size_t tx_len = 0;
  size_t tx_pos = 0;
  uint8_t tx_next_sn = 0;
  uint8_t tx_block_size = 0;
  uint8_t tx_block_count = 0;
  uint32_t tx_st_min_ms = 0;
  uint32_t tx_next_ms = 0;
  uint32_t tx_deadline = 0;

  void send_frame(const uint8_t *bytes, size_t len);
  void send_flow_control(uint8_t status);
  void send_consecutive_frame();
  void on_flow_control(const can_message &msg, uint32_t now);
};

} // namespace ota
