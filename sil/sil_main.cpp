// The VCU firmware as a Linux program, on SocketCAN.
//
// Runs the real control logic (the same sim::Car the tests use) in real time
// at 1 kHz, bridging its three buses to (virtual) CAN interfaces. Watch it
// with candump, flash it with tools/ota/flash.py.
//
//   tools/sil/setup_vcan.sh            # once, needs root
//   pio run -e sil
//   .pio/build/sil/program --healthy-ts
//   candump vcan_daq
#include <sim_car.hpp>

#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <deque>
#include <memory>
#include <string>

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace {

volatile sig_atomic_t stop = 0;

class SocketCan {
public:
  explicit SocketCan(const std::string &ifname) : name(ifname) {
    fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd < 0)
      die("socket");

    ifreq ifr{};
    strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);
    if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0)
      die("no such interface (run tools/sil/setup_vcan.sh?)");

    sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0)
      die("bind");

    fcntl(fd, F_SETFL, O_NONBLOCK);
  }
  ~SocketCan() { close(fd); }

  // Everything waiting on the interface, into the VCU's RX queue
  void receive_into(sim::Bus &bus) {
    can_frame f;
    while (read(fd, &f, sizeof(f)) == sizeof(f)) {
      can_message m;
      m.is_extended = f.can_id & CAN_EFF_FLAG;
      m.id = f.can_id & (m.is_extended ? CAN_EFF_MASK : CAN_SFF_MASK);
      m.length = f.can_dlc;
      memcpy(m.buf.byte, f.data, 8);
      bus.inject(m);
    }
  }

  // Everything the VCU sent this loop, out on the interface
  void send_from(sim::Bus &bus) {
    for (const auto &m : bus.tx)
      pending.push_back(m);
    bus.tx.clear();

    while (!pending.empty()) {
      const can_message &m = pending.front();
      can_frame f{};
      f.can_id = m.id | (m.is_extended ? CAN_EFF_FLAG : 0);
      f.can_dlc = m.length;
      memcpy(f.data, m.buf.byte, 8);
      if (write(fd, &f, sizeof(f)) != sizeof(f)) {
        if (errno == EAGAIN || errno == ENOBUFS)
          break; // queue full, try again next loop
        die("write");
      }
      pending.pop_front();
    }
    // Like a real controller, drop the oldest if we're hopelessly behind
    while (pending.size() > 4096)
      pending.pop_front();
  }

private:
  std::string name;
  int fd;
  std::deque<can_message> pending;

  [[noreturn]] void die(const char *what) {
    fprintf(stderr, "%s: %s: %s\n", name.c_str(), what, strerror(errno));
    exit(1);
  }
};

uint32_t monotonic_ms() {
  timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return uint32_t(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

const char *state_name(state s) {
  switch (s) {
  case STARTUP: return "STARTUP";
  case TRACTIVE_SYSTEM_DISABLED: return "TRACTIVE_SYSTEM_DISABLED";
  case TRACTIVE_SYSTEM_ENERGIZED: return "TRACTIVE_SYSTEM_ENERGIZED";
  case TRACTIVE_SYSTEM_ENABLED: return "TRACTIVE_SYSTEM_ENABLED";
  case READY_TO_DRIVE: return "READY_TO_DRIVE";
  case LAUNCH_WAIT: return "LAUNCH_WAIT";
  case LAUNCH: return "LAUNCH";
  }
  return "?";
}

void usage(const char *argv0) {
  fprintf(stderr,
          "usage: %s [--acc IF] [--inv IF] [--daq IF] [--healthy-ts]\n"
          "          [--version HEX] [--verbose]\n"
          "\n"
          "  --acc/--inv/--daq  interfaces for the three buses\n"
          "                     (default vcan_acc, vcan_inv, vcan_daq)\n"
          "  --healthy-ts       pretend to be a happy ACU and inverter, so\n"
          "                     the VCU energizes (and refuses OTA)\n"
          "  --version HEX      firmware version to report before any update\n"
          "  --verbose          echo the firmware's own log output\n",
          argv0);
}

} // namespace

int main(int argc, char **argv) {
  std::string acc_if = "vcan_acc", inv_if = "vcan_inv", daq_if = "vcan_daq";
  bool healthy_ts = false, verbose = false;
  uint32_t version = 0x00000001;

  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    auto next = [&]() -> const char * {
      if (i + 1 >= argc) {
        usage(argv[0]);
        exit(2);
      }
      return argv[++i];
    };
    if (a == "--acc") acc_if = next();
    else if (a == "--inv") inv_if = next();
    else if (a == "--daq") daq_if = next();
    else if (a == "--healthy-ts") healthy_ts = true;
    else if (a == "--version") version = strtoul(next(), nullptr, 16);
    else if (a == "--verbose") verbose = true;
    else {
      usage(argv[0]);
      return 2;
    }
  }

  signal(SIGINT, [](int) { stop = 1; });
  signal(SIGTERM, [](int) { stop = 1; });
  setvbuf(stdout, nullptr, _IOLBF, 0);

  SocketCan acc(acc_if), inv(inv_if), daq(daq_if);

  const uint32_t start = monotonic_ms();
  auto car = std::make_unique<sim::Car>();
  car->board.version = version;
  sim::set_echo_log(verbose);
  printf("VCU SIL up: acc=%s inv=%s daq=%s, firmware %08x\n", acc_if.c_str(),
         inv_if.c_str(), daq_if.c_str(), version);

  size_t transitions_seen = 0;
  uint32_t last_fake = 0;

  while (!stop) {
    const uint32_t now = monotonic_ms() - start;
    sim::set_now(now);

    if (healthy_ts && now - last_fake >= 100) {
      car->make_ts_healthy();
      last_fake = now;
    }

    acc.receive_into(car->acc());
    inv.receive_into(car->inv());
    daq.receive_into(car->daq());

    const size_t reboots = car->board.reboots;
    car->tick();

    acc.send_from(car->acc());
    inv.send_from(car->inv());
    daq.send_from(car->daq());

    for (; transitions_seen < car->transitions.size(); transitions_seen++) {
      const auto &t = car->transitions[transitions_seen];
      printf("[%7.3f] state -> %s\n", t.at_ms / 1000.0, state_name(t.to));
    }

    // The updater asked for a reboot: start over with whatever firmware
    // the board now has, like the real thing coming back up
    if (car->board.reboots != reboots) {
      const bool applied = car->board.applies != 0;
      const uint32_t new_version = car->board.version;
      std::vector<uint8_t> firmware = std::move(car->board.firmware);
      printf("[%7.3f] %s, rebooting into %08x\n", now / 1000.0,
             applied ? "applied update" : "reset requested", new_version);

      car = std::make_unique<sim::Car>();
      sim::set_now(now);
      car->board.version = new_version;
      car->board.firmware = std::move(firmware);
      sim::set_echo_log(verbose);
      transitions_seen = 0;
    }

    // Sleep to the next millisecond
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    ts.tv_nsec = (ts.tv_nsec / 1000000 + 1) * 1000000;
    if (ts.tv_nsec >= 1000000000) {
      ts.tv_sec++;
      ts.tv_nsec -= 1000000000;
    }
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr);
  }

  printf("bye\n");
  return 0;
}
