#include "tuushin/net/clock_sync.hpp"

#include <algorithm>
#include <cstddef>
#include <limits>
#include <stdexcept>

namespace tuushin::net {

namespace {

struct WindowPoint {
  double t;      // client time relative to t_ref, ns
  double theta;  // ns
};

double median(std::vector<double>& v) {
  const std::size_t n = v.size();
  const auto mid = v.begin() + static_cast<std::ptrdiff_t>(n / 2);
  std::nth_element(v.begin(), mid, v.end());
  if (n % 2 == 1) return *mid;
  const double hi = *mid;
  const double lo = *std::max_element(v.begin(), mid);
  return 0.5 * (lo + hi);
}

}  // namespace

ClockModel fit_clock(const std::vector<TimestampSample>& samples, Nanos window) {
  if (window <= 0) throw std::invalid_argument("fit_clock: window must be positive");
  ClockModel m;
  if (samples.empty()) return m;
  m.t_ref = samples.front().t1;

  std::vector<WindowPoint> points;
  std::size_t i = 0;
  while (i < samples.size()) {
    const Nanos w = (samples[i].t1 - m.t_ref) / window;
    Nanos min_f = std::numeric_limits<Nanos>::max();
    Nanos min_b = std::numeric_limits<Nanos>::max();
    double t_sum = 0.0;
    std::size_t n = 0;
    for (; i < samples.size() && (samples[i].t1 - m.t_ref) / window == w; ++i) {
      min_f = std::min(min_f, samples[i].raw_forward());
      min_b = std::min(min_b, samples[i].raw_backward());
      t_sum += static_cast<double>(samples[i].t1 - m.t_ref);
      ++n;
    }
    points.push_back({t_sum / static_cast<double>(n),
                      0.5 * (static_cast<double>(min_f) - static_cast<double>(min_b))});
  }

  const std::size_t n = points.size();
  if (n >= 2) {
    std::vector<double> slopes;
    auto add = [&](std::size_t a, std::size_t b) {
      const double dt = points[b].t - points[a].t;
      if (dt > 0.0) slopes.push_back((points[b].theta - points[a].theta) / dt);
    };
    if (n <= 512) {
      slopes.reserve(n * (n - 1) / 2);
      for (std::size_t a = 0; a < n; ++a) {
        for (std::size_t b = a + 1; b < n; ++b) add(a, b);
      }
    } else {
      // All pairs would be O(n^2); pairing each window with the one half the
      // recording later keeps long baselines and the median's robustness.
      for (std::size_t a = 0; a + n / 2 < n; ++a) add(a, a + n / 2);
    }
    if (!slopes.empty()) m.skew = median(slopes);
  }
  std::vector<double> offsets;
  offsets.reserve(n);
  for (const auto& p : points) offsets.push_back(p.theta - m.skew * p.t);
  m.offset = median(offsets);
  return m;
}

}  // namespace tuushin::net
