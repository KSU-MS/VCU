#include "power_loop.hpp"
#include "parameters.hpp"
#include <cmath>

void PowerLoop::update(uint32_t now_ms, double limit_w, bool sample,
                       double measured_w) {
  const double target_w = limit_w * POWER_LIMIT_TARGET;

  if (sample) {
    const uint32_t since = ever ? now_ms - last_ms : 0;
    ever = true;
    last_ms = now_ms;

    if (std::isfinite(measured_w) && measured_w > target_w) {
      // Torque cap ~ trim, so this puts the next sample on the target. Only
      // per new sample: the measurement lags, so cutting again every loop
      // until it catches up would overshoot
      trim_ *= target_w / measured_w;
    } else if (std::isfinite(measured_w)) {
      trim_ += RECOVER_PER_MS * since;
    }
  }

  if (!ever || now_ms - last_ms > STALE_MS) {
    // Flying blind: no more than the open-loop cap
    if (trim_ > 1.0)
      trim_ = 1.0;
  }

  if (trim_ > MAX_TRIM)
    trim_ = MAX_TRIM;
  if (trim_ < MIN_TRIM || !std::isfinite(trim_))
    trim_ = MIN_TRIM;
}
