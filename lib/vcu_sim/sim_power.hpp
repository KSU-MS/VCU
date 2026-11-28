#pragma once
// EV.3.4.1, how the power tests judge a run: using more than the limit is a
// violation if it lasts 100 ms continuously, or after a 500 ms moving
// average. `watts`: electrical power every ms.
#include <algorithm>
#include <vector>

#include <doctest.h>
#include <parameters.hpp>

namespace sim {

inline void no_violation(const std::vector<double> &watts) {
  const double limit = POWER_LIMIT_KW * 1000.0;
  uint32_t over_run = 0, longest = 0;
  for (double w : watts) {
    over_run = w > limit ? over_run + 1 : 0;
    longest = std::max(longest, over_run);
  }
  CHECK(longest < 100);

  double window = 0, worst_average = 0;
  for (size_t i = 0; i < watts.size(); i++) {
    window += watts[i];
    if (i >= 500)
      window -= watts[i - 500];
    if (i >= 499)
      worst_average = std::max(worst_average, window / 500);
  }
  CHECK(worst_average <= limit);
}

} // namespace sim
