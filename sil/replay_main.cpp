// A drive log played into the VCU, as fast as it'll go: the same sim::Car
// the tests use, on simulated time, fed what the log says the rest of the
// car was doing, writing down what the VCU told each motor.
//
// The log comes in as a CSV, one row every few milliseconds (tools/replay
// makes it from a kmslap mcap). Columns, by name, any order:
//   t_s                    time, s (rows in order)
//   throttle, brake        pedal travel, 0..1
//   pack_v                 the pack's voltage (the ACU's 0x6B1)
//   bus_v_<w>              each inverter's DC voltage
//   rpm_<w>, dc_a_<w>      each motor's speed and DC current, as its
//                          inverter reports them
//   vn_n, vn_e, vn_ax      VectorNav: north and east speed, m/s, and
//                          forward acceleration, m/s^2
//   wheel_rpm_<w>          the wheel speed sensors
// with <w> fl, fr, rl, rr. Each row holds until the next.
//
// Out, a CSV with a row per input row: what the VCU was sending by the end
// of that row's time, having seen it.
//   t_s, state, fault, torque_<w> (N m at the motor), enable_<w>
//
// Each inverter's DC current is the one thing that has to answer the VCU:
// it's what the VCU's 80 kW loop closes on. A log's currents answer the
// torques in the log, not the VCU's, so by default the replay makes its
// own: the VCU's torque at the logged speed, through an assumed efficiency
// (--efficiency, 0.90 like the power tests), over the logged voltage.
// --logged-dc uses the log's currents instead.
//
// The BMS's charge limit, which regen keeps to, isn't in kmslap's logs: the
// replay's BMS says --charge-limit (A, default 234: the pack's peak charge
// current in kmslap) and a 600 V maximum pack voltage throughout.
//
// The car is walked to ready to drive first, with the first row's values,
// the way a driver would (sim::Car::drive_to_rtd), then the log runs.
//
//   pio run -e replay_4wd_dti
//   .pio/build/replay_4wd_dti/program [--logged-dc] [--efficiency E] \
//       [--charge-limit A] \
//       in.csv out.csv
#include <sim_car.hpp>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#if !VEHICLE_HAS_DTI
#error "the replay is for the hub motor cars so far (env:replay_4wd_dti)"
#endif

namespace {

const char *const WHEELS[] = {"fl", "fr", "rl", "rr"};

struct Csv {
  std::vector<std::string> names;
  std::vector<std::vector<double>> rows;

  size_t column(const std::string &name) const {
    for (size_t i = 0; i < names.size(); i++)
      if (names[i] == name)
        return i;
    fprintf(stderr, "input has no column %s\n", name.c_str());
    exit(2);
  }
};

std::vector<std::string> split(const std::string &line) {
  std::vector<std::string> out;
  std::stringstream ss(line);
  std::string cell;
  while (std::getline(ss, cell, ','))
    out.push_back(cell);
  return out;
}

Csv read_csv(const char *path) {
  std::ifstream in(path);
  if (!in) {
    fprintf(stderr, "can't read %s\n", path);
    exit(2);
  }
  Csv csv;
  std::string line;
  std::getline(in, line);
  csv.names = split(line);
  while (std::getline(in, line)) {
    if (line.empty())
      continue;
    std::vector<double> row;
    for (const auto &cell : split(line))
      row.push_back(strtod(cell.c_str(), nullptr));
    if (row.size() != csv.names.size()) {
      fprintf(stderr, "row %zu has %zu cells, header %zu\n",
              csv.rows.size() + 2, row.size(), csv.names.size());
      exit(2);
    }
    csv.rows.push_back(std::move(row));
  }
  return csv;
}

// The log's columns, looked up once
struct Columns {
  size_t t, throttle, brake, pack_v, vn_n, vn_e, vn_ax;
  size_t bus_v[4], rpm[4], dc_a[4], wheel_rpm[4];

  explicit Columns(const Csv &csv) {
    t = csv.column("t_s");
    throttle = csv.column("throttle");
    brake = csv.column("brake");
    pack_v = csv.column("pack_v");
    vn_n = csv.column("vn_n");
    vn_e = csv.column("vn_e");
    vn_ax = csv.column("vn_ax");
    for (size_t i = 0; i < 4; i++) {
      const std::string w = WHEELS[i];
      bus_v[i] = csv.column("bus_v_" + w);
      rpm[i] = csv.column("rpm_" + w);
      dc_a[i] = csv.column("dc_a_" + w);
      wheel_rpm[i] = csv.column("wheel_rpm_" + w);
    }
  }
};

uint16_t wheel_rpm(double rpm) {
  return uint16_t(std::lround(std::fmin(std::fmax(rpm, 0), 65535)));
}

// The rest of the car as the row has it
void apply(sim::Car &car, const Columns &c, const std::vector<double> &row,
           bool logged_dc) {
  car.set_throttle(row[c.throttle]);
  car.set_brake(row[c.brake]);
  double dc_total = 0;
  for (size_t i = 0; i < sim::Car::HUBS; i++) {
    car.hub_bus_voltage(i, row[c.bus_v[i]]);
    car.hub_rpm(i, row[c.rpm[i]]);
    if (logged_dc)
      car.hub_dc_current(i, row[c.dc_a[i]]);
    dc_total += row[c.dc_a[i]];
  }
  car.pack_power(row[c.pack_v], dc_total);
  car.vectornav(row[c.vn_n], row[c.vn_e], row[c.vn_ax]);
  car.wheel_rpm(wheel_rpm(row[c.wheel_rpm[0]]), wheel_rpm(row[c.wheel_rpm[1]]),
                wheel_rpm(row[c.wheel_rpm[2]]), wheel_rpm(row[c.wheel_rpm[3]]));
}

// Each inverter's DC current from what the VCU is asking of it now
void model_dc(sim::Car &car, const Columns &c, const std::vector<double> &row,
              double efficiency) {
  for (size_t i = 0; i < sim::Car::HUBS; i++) {
    const double torque = car.last_hub_torque(i);
    const double omega = row[c.rpm[i]] * 2 * M_PI / 60;
    const double mech_w = torque * omega;
    const double elec_w = mech_w >= 0 ? mech_w / efficiency : mech_w * efficiency;
    const double volts = row[c.bus_v[i]];
    car.hub_dc_current(i, volts > 1 ? elec_w / volts : 0);
  }
}

void usage(const char *argv0) {
  fprintf(stderr,
          "usage: %s [--logged-dc] [--efficiency E] [--charge-limit A]\n"
          "          in.csv out.csv\n", argv0);
  exit(2);
}

} // namespace

int main(int argc, char **argv) {
  bool logged_dc = false;
  double charge_limit_a = 234;
  double efficiency = 0.90;
  const char *paths[2] = {};
  size_t n_paths = 0;
  for (int i = 1; i < argc; i++) {
    if (!strcmp(argv[i], "--logged-dc"))
      logged_dc = true;
    else if (!strcmp(argv[i], "--efficiency") && i + 1 < argc)
      efficiency = strtod(argv[++i], nullptr);
    else if (!strcmp(argv[i], "--charge-limit") && i + 1 < argc)
      charge_limit_a = strtod(argv[++i], nullptr);
    else if (argv[i][0] != '-' && n_paths < 2)
      paths[n_paths++] = argv[i];
    else
      usage(argv[0]);
  }
  if (n_paths != 2 || !(efficiency > 0 && efficiency <= 1))
    usage(argv[0]);

  const Csv log = read_csv(paths[0]);
  if (log.rows.empty()) {
    fprintf(stderr, "%s: no rows\n", paths[0]);
    return 2;
  }
  const Columns c(log);

  FILE *out = fopen(paths[1], "w");
  if (!out) {
    fprintf(stderr, "can't write %s\n", paths[1]);
    return 2;
  }
  fprintf(out, "t_s,state,fault");
  for (const char *w : WHEELS)
    fprintf(out, ",torque_%s", w);
  for (const char *w : WHEELS)
    fprintf(out, ",enable_%s", w);
  fprintf(out, "\n");

  auto car = std::make_unique<sim::Car>();
  // To ready to drive with the log's first voltages, then its pedals
  const std::vector<double> &first = log.rows.front();
  car->run_for(10);
  car->acu_status(true, true);
  car->precharge_state(PRECHARGE_OK_STATE);
  car->bms_charge_limit(charge_limit_a, 600);
  car->inverter_bus_voltage(first[c.pack_v]);
  apply(*car, c, first, true);
  car->set_throttle(0);
  car->set_brake(0);
  car->run_for(10);
  car->drive_to_rtd();
  if (car->vcu.get_current_state() != READY_TO_DRIVE) {
    fprintf(stderr, "the VCU didn't get to ready to drive (state %d, "
                    "fault %02x)\n",
            car->vcu.get_current_state(), car->vcu.fault_code());
    return 1;
  }

  // The log's time on the VCU's clock
  const double t0 = first[c.t];
  const uint32_t start_ms = sim::now();
  auto ms_of = [&](size_t k) {
    return start_ms + uint32_t(std::lround((log.rows[k][c.t] - t0) * 1000));
  };
  for (size_t k = 0; k < log.rows.size(); k++) {
    const auto &row = log.rows[k];
    apply(*car, c, row, logged_dc);
    // Through to the next row (the last for as long as the one before it)
    const uint32_t until =
        k + 1 < log.rows.size()
            ? ms_of(k + 1)
            : ms_of(k) + (k ? ms_of(k) - ms_of(k - 1) : 10);
    while (sim::now() < until) {
      if (!logged_dc)
        model_dc(*car, c, row, efficiency);
      car->tick();
      sim::advance(1);
    }

    fprintf(out, "%.3f,%d,%u", row[c.t], car->vcu.get_current_state(),
            car->vcu.fault_code());
    sim::Car::HubCommand cmd[sim::Car::HUBS];
    for (size_t i = 0; i < sim::Car::HUBS; i++) {
      if (!car->last_hub_command(i, &cmd[i]))
        cmd[i] = {0, false};
      fprintf(out, ",%.3f", car->last_hub_torque(i));
    }
    for (size_t i = 0; i < sim::Car::HUBS; i++)
      fprintf(out, ",%d", int(cmd[i].enable));
    fprintf(out, "\n");
  }
  fclose(out);
  return 0;
}
