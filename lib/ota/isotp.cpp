#include "isotp.hpp"
#include <string.h>

namespace ota {

namespace {

// Protocol control information, top nibble of byte 0
constexpr uint8_t PCI_SINGLE = 0x0;
constexpr uint8_t PCI_FIRST = 0x1;
constexpr uint8_t PCI_CONSECUTIVE = 0x2;
constexpr uint8_t PCI_FLOW_CONTROL = 0x3;

constexpr uint8_t FC_CONTINUE = 0x0;
constexpr uint8_t FC_WAIT = 0x1;
constexpr uint8_t FC_OVERFLOW = 0x2;

// Wrap-safe "a is later than b" for millisecond timestamps
bool after(uint32_t a, uint32_t b) { return int32_t(a - b) > 0; }

// STmin encoding: 0x00-0x7F is ms, 0xF1-0xF9 is 100-900 us. We can't time
// sub-ms gaps, so round those up to 1 ms; anything reserved means the max.
uint32_t decode_st_min(uint8_t raw) {
  if (raw <= 0x7F)
    return raw;
  if (raw >= 0xF1 && raw <= 0xF9)
    return 1;
  return 0x7F;
}

} // namespace

IsoTp::IsoTp(const Config &config, SendFn send, void *ctx)
    : cfg(config), send_fn(send), send_ctx(ctx) {}

void IsoTp::send_frame(const uint8_t *bytes, size_t len) {
  can_message msg;
  msg.id = cfg.tx_id;
  msg.length = 8;
  memset(msg.buf.byte, cfg.padding, 8);
  memcpy(msg.buf.byte, bytes, len);
  send_fn(msg, send_ctx);
}

void IsoTp::send_flow_control(uint8_t status) {
  uint8_t fc[3] = {uint8_t((PCI_FLOW_CONTROL << 4) | status), cfg.block_size,
                   cfg.st_min_ms};
  send_frame(fc, sizeof(fc));
}

void IsoTp::on_frame(const can_message &msg, uint32_t now) {
  if (msg.id != cfg.rx_id || msg.is_extended || msg.length < 1)
    return;

  const uint8_t *b = msg.buf.byte;
  const uint8_t pci = b[0] >> 4;

  switch (pci) {
  case PCI_SINGLE: {
    size_t len = b[0] & 0x0F;
    if (rx_state == RxState::DONE || len == 0 || len > 7 ||
        len > size_t(msg.length - 1))
      return;
    // A new single frame also abandons any half-received message
    memcpy(rx_buf, b + 1, len);
    rx_len = len;
    rx_state = RxState::DONE;
    break;
  }

  case PCI_FIRST: {
    if (rx_state == RxState::DONE || msg.length < 8)
      return;
    size_t len = (size_t(b[0] & 0x0F) << 8) | b[1];
    if (len < 8)
      return; // would have fit in a single frame, so it's malformed
    if (len > MAX_RX) {
      send_flow_control(FC_OVERFLOW);
      return;
    }
    memcpy(rx_buf, b + 2, 6);
    rx_len = 6;
    rx_expected = len;
    rx_next_sn = 1;
    rx_block_count = 0;
    rx_deadline = now + cfg.timeout_ms;
    rx_state = RxState::RECEIVING;
    send_flow_control(FC_CONTINUE);
    break;
  }

  case PCI_CONSECUTIVE: {
    if (rx_state != RxState::RECEIVING)
      return;
    if ((b[0] & 0x0F) != rx_next_sn) {
      rx_state = RxState::IDLE; // lost a frame, the sender has to start over
      return;
    }
    size_t chunk = rx_expected - rx_len;
    if (chunk > 7)
      chunk = 7;
    if (msg.length < chunk + 1)
      return;
    memcpy(rx_buf + rx_len, b + 1, chunk);
    rx_len += chunk;
    rx_next_sn = (rx_next_sn + 1) & 0x0F;
    rx_deadline = now + cfg.timeout_ms;

    if (rx_len == rx_expected) {
      rx_state = RxState::DONE;
    } else if (cfg.block_size && ++rx_block_count == cfg.block_size) {
      rx_block_count = 0;
      send_flow_control(FC_CONTINUE);
    }
    break;
  }

  case PCI_FLOW_CONTROL:
    on_flow_control(msg, now);
    break;

  default:
    break;
  }
}

void IsoTp::on_flow_control(const can_message &msg, uint32_t now) {
  if (tx_state != TxState::WAIT_FC || msg.length < 3)
    return;

  const uint8_t *b = msg.buf.byte;
  switch (b[0] & 0x0F) {
  case FC_CONTINUE:
    tx_block_size = b[1];
    tx_block_count = 0;
    tx_st_min_ms = decode_st_min(b[2]);
    tx_next_ms = now;
    tx_state = TxState::SENDING;
    break;
  case FC_WAIT:
    tx_deadline = now + cfg.timeout_ms;
    break;
  default: // overflow or garbage, give up on this message
    tx_state = TxState::IDLE;
    break;
  }
}

bool IsoTp::send(const uint8_t *data, size_t len, uint32_t now) {
  if (tx_busy() || len == 0 || len > MAX_TX)
    return false;

  if (len <= 7) {
    uint8_t sf[8];
    sf[0] = uint8_t((PCI_SINGLE << 4) | len);
    memcpy(sf + 1, data, len);
    send_frame(sf, len + 1);
    return true;
  }

  memcpy(tx_buf, data, len);
  tx_len = len;

  uint8_t ff[8];
  ff[0] = uint8_t((PCI_FIRST << 4) | ((len >> 8) & 0x0F));
  ff[1] = uint8_t(len & 0xFF);
  memcpy(ff + 2, tx_buf, 6);
  send_frame(ff, 8);

  tx_pos = 6;
  tx_next_sn = 1;
  tx_deadline = now + cfg.timeout_ms;
  tx_state = TxState::WAIT_FC;
  return true;
}

void IsoTp::send_consecutive_frame() {
  size_t chunk = tx_len - tx_pos;
  if (chunk > 7)
    chunk = 7;

  uint8_t cf[8];
  cf[0] = uint8_t((PCI_CONSECUTIVE << 4) | tx_next_sn);
  memcpy(cf + 1, tx_buf + tx_pos, chunk);
  send_frame(cf, chunk + 1);

  tx_pos += chunk;
  tx_next_sn = (tx_next_sn + 1) & 0x0F;
}

void IsoTp::poll(uint32_t now) {
  if (rx_state == RxState::RECEIVING && after(now, rx_deadline))
    rx_state = RxState::IDLE;

  if (tx_state == TxState::WAIT_FC && after(now, tx_deadline))
    tx_state = TxState::IDLE;

  while (tx_state == TxState::SENDING && !after(tx_next_ms, now)) {
    send_consecutive_frame();

    if (tx_pos >= tx_len) {
      tx_state = TxState::IDLE;
    } else if (tx_block_size && ++tx_block_count == tx_block_size) {
      tx_deadline = now + cfg.timeout_ms;
      tx_state = TxState::WAIT_FC;
    } else if (tx_st_min_ms) {
      tx_next_ms = now + tx_st_min_ms;
      break;
    }
  }
}

} // namespace ota
