#pragma once
// Firmware update over CAN. Feed it the frames from the bus the tester is on
// (DAQ on this car) and call poll() every loop.
//
// The two IDs aren't in ksu-ms-dbc yet; they should be reserved there before
// this goes on the car.
#include <can_tools.hpp>

#include "isotp.hpp"
#include "uds_server.hpp"

namespace ota {

constexpr uint32_t REQUEST_ID = 0x7E0;  // tester -> VCU
constexpr uint32_t RESPONSE_ID = 0x7E8; // VCU -> tester

class Service {
public:
  Service(Platform &platform, StagingFlash &staging, canMan *bus);

  static bool wants(const can_message &msg) { return msg.id == REQUEST_ID; }

  void on_frame(const can_message &msg, uint32_t now);
  void poll(uint32_t now);

  // True while a tester holds a programming session. The VCU must not
  // leave TS_DISABLED while this is set.
  bool programming() const { return uds.programming(); }

  const UdsServer &server() const { return uds; }

private:
  Platform &platform;
  canMan *bus;
  IsoTp transport;
  UdsServer uds;

  static void send_frame(const can_message &msg, void *ctx);
};

} // namespace ota
