#pragma once
// EV.3.3.1 limits the power the Energy Meter measures, V x I at the
// accumulator, not shaft power. The VCU caps torque at the power limit x
// 0.90 assumed drivetrain efficiency / motor speed (open loop); this is the
// trim on that cap, closed on measured DC power: each new sample over
// POWER_LIMIT_TARGET of the limit scales it down so the next one lands on
// it, and below the target it creeps back up (to 1/0.90, the full limit at
// the shaft, if the drivetrain turns out lossless). Without fresh samples it
// never allows more than the open-loop cap.
//
// One per inverter on rwd_emrax (Inverter::regulate_power); one on all the
// hubs' total on the hub motor cars (Drivetrain::regulate_power).
#include <cstdint>

class PowerLoop {
public:
  // Call every loop. `sample`: a new measurement of the power being
  // limited arrived since the last call, and it's `measured_w`
  void update(uint32_t now_ms, double limit_w, bool sample, double measured_w);

  // The share of the open-loop cap allowed now
  double trim() const { return trim_; }

  // 1/0.90 undoes the open-loop efficiency guess: the full limit at the
  // shaft, the most it could ever be with a lossless drivetrain
  static constexpr double MAX_TRIM = 1.0 / 0.90;
  static constexpr double MIN_TRIM = 0.1;
  // Back up from a cut to the full limit in about two seconds
  static constexpr double RECOVER_PER_MS = (MAX_TRIM - MIN_TRIM) / 2000.0;
  // Samples older than this don't count
  static constexpr uint32_t STALE_MS = 100;

private:
  double trim_ = 1.0;
  bool ever = false;
  uint32_t last_ms = 0;
};
