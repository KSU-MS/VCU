#include "ota.hpp"

namespace ota {

namespace {
IsoTp::Config transport_config() {
  IsoTp::Config c;
  c.tx_id = RESPONSE_ID;
  c.rx_id = REQUEST_ID;
  // FlexCAN_T4's default layout (polling mode, as ksu-fw-common uses it)
  // only has 4 receive mailboxes for standard IDs, emptied once per loop.
  // Ask for 3 frames per flow-control block so a TransferData never overruns
  // them (one is left for other DAQ traffic); the VCU reads all of them in
  // one loop and sends the next flow control. Without this the tester bursts
  // ~150 frames and most are lost.
  c.block_size = 3;
  return c;
}
} // namespace

Service::Service(Platform &platform, StagingFlash &staging, canMan *bus)
    : platform(platform), bus(bus), transport(transport_config(), send_frame, this),
      uds(platform, staging) {}

void Service::send_frame(const can_message &msg, void *ctx) {
  static_cast<Service *>(ctx)->bus->send_controller_message(msg);
}

void Service::on_frame(const can_message &msg, uint32_t now) {
  transport.on_frame(msg, now);
}

void Service::poll(uint32_t now) {
  transport.poll(now);

  if (transport.available()) {
    uint8_t resp[16];
    size_t n = uds.handle(transport.data(), transport.length(), resp, now);
    transport.release();
    if (n)
      transport.send(resp, n, now);
  }

  uds.poll(now);

  // Only reboot once the ECUReset response is actually on the wire
  if (uds.pending_reset() != UdsServer::PendingReset::NONE &&
      !transport.tx_busy())
    uds.do_pending_reset();
}

} // namespace ota
