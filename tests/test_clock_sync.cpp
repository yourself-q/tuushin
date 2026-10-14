#include <cmath>
#include <stdexcept>
#include <vector>

#include "test.hpp"
#include "tuushin/net/clock_sync.hpp"
#include "tuushin/rng.hpp"

using namespace tuushin;
using namespace tuushin::net;

namespace {

// Two hosts whose clocks differ by theta(t) = offset + skew * (t - t0),
// t in client time, joined by a path with a fixed minimum delay per
// direction plus exponential queueing.
struct SyntheticPath {
  Nanos t0 = 1'000'000'000'000;  // the client clock does not start at zero
  double offset = 3.7e6;         // ns
  double skew = 50e-6;           // 50 ppm
  double min_fwd = 10e6, min_bwd = 10e6;
  double queue_mean = 2e6;
  Nanos hold = 30'000;  // server turnaround

  double theta(double client_time) const {
    return offset + skew * (client_time - static_cast<double>(t0));
  }

  std::vector<TimestampSample> run(Rng& rng, Nanos interval, Nanos duration,
                                   double extra_queue_from = -1, double extra_queue_to = -1,
                                   double extra_queue = 0) const {
    std::vector<TimestampSample> out;
    for (Nanos k = 0; k * interval < duration; ++k) {
      const double c1 = static_cast<double>(t0 + k * interval);
      const double rel = c1 - static_cast<double>(t0);
      const double extra = rel >= extra_queue_from && rel < extra_queue_to ? extra_queue : 0.0;
      const double c2 = c1 + min_fwd + rng.exponential(queue_mean) + extra;
      const double s2 = c2 + theta(c2);
      const double s3 = s2 + static_cast<double>(hold);
      // Client time at which the server clock reads s3.
      const double c3 =
          (s3 - offset + skew * static_cast<double>(t0)) / (1.0 + skew);
      const double c4 = c3 + min_bwd + rng.exponential(queue_mean) + extra;
      TimestampSample s;
      s.t1 = static_cast<Nanos>(c1);
      s.t2 = static_cast<Nanos>(std::llround(s2));
      s.t3 = static_cast<Nanos>(std::llround(s3));
      s.t4 = static_cast<Nanos>(std::llround(c4));
      out.push_back(s);
    }
    return out;
  }
};

}  // namespace

TEST_CASE(clock_sync_sample_accessors) {
  TimestampSample s{100, 1150, 1170, 290};
  CHECK(s.rtt() == 190 - 20);
  CHECK(s.raw_forward() == 1050);
  CHECK(s.raw_backward() == -880);
}

TEST_CASE(clock_sync_recovers_offset_and_skew) {
  SyntheticPath path;
  Rng rng(5);
  const auto samples = path.run(rng, 10 * kNanosPerMilli, 60 * kNanosPerSecond);
  const ClockModel m = fit_clock(samples);
  CHECK(m.t_ref == path.t0);
  // Per-window minima of 100 exponential delays (mean 2 ms) sit about 20 us
  // above the floor in each direction; the median over 60 windows is much
  // tighter than that.
  CHECK_NEAR(m.skew, path.skew, 0.5e-6);
  for (double t : {0.0, 30e9, 59e9}) {
    const Nanos ct = path.t0 + static_cast<Nanos>(t);
    CHECK_NEAR(m.offset_at(ct), path.theta(static_cast<double>(ct)), 30e3);
  }
}

TEST_CASE(clock_sync_corrected_one_way_delays_are_flat) {
  // After removing theta(t), the floor of the one-way delays neither drifts
  // nor jumps: a residual skew would show up as a slope of 3 ms per minute.
  SyntheticPath path;
  Rng rng(9);
  const auto samples = path.run(rng, 10 * kNanosPerMilli, 60 * kNanosPerSecond);
  const ClockModel m = fit_clock(samples);
  double first_min = 1e18, last_min = 1e18;
  for (const auto& s : samples) {
    const double fwd = static_cast<double>(s.raw_forward()) - m.offset_at(s.t1);
    const double rel = static_cast<double>(s.t1 - path.t0);
    if (rel < 5e9) first_min = std::fmin(first_min, fwd);
    if (rel >= 55e9) last_min = std::fmin(last_min, fwd);
    CHECK(fwd > path.min_fwd - 100e3);
  }
  CHECK_NEAR(first_min, path.min_fwd, 50e3);
  CHECK_NEAR(last_min, path.min_fwd, 50e3);
}

TEST_CASE(clock_sync_ignores_windows_with_a_standing_queue) {
  // Ten seconds of a queue that never drains (bufferbloat behind a bulk
  // transfer) lift every sample by 30 ms. A least-squares line through the
  // window minima would be dragged; the median of slopes is not.
  SyntheticPath path;
  Rng rng(13);
  const auto samples =
      path.run(rng, 10 * kNanosPerMilli, 60 * kNanosPerSecond, 40e9, 50e9, 30e6);
  const ClockModel m = fit_clock(samples);
  CHECK_NEAR(m.skew, path.skew, 1e-6);
  CHECK_NEAR(m.offset_at(path.t0), path.theta(static_cast<double>(path.t0)), 50e3);
}

TEST_CASE(clock_sync_asymmetry_shifts_the_offset_by_half) {
  // Documented limitation: with unequal minimum delays the offset is off by
  // half the asymmetry, while the RTT and the skew are unaffected.
  SyntheticPath path;
  path.min_fwd = 6e6;
  path.min_bwd = 14e6;
  Rng rng(17);
  const auto samples = path.run(rng, 10 * kNanosPerMilli, 30 * kNanosPerSecond);
  const ClockModel m = fit_clock(samples);
  CHECK_NEAR(m.skew, path.skew, 1e-6);
  const double expected = path.theta(static_cast<double>(path.t0)) - 0.5 * (14e6 - 6e6);
  CHECK_NEAR(m.offset_at(path.t0), expected, 50e3);
}

TEST_CASE(clock_sync_many_windows_use_the_long_baseline_path) {
  // 3000 windows of 20 ms exceed the all-pairs limit.
  SyntheticPath path;
  Rng rng(21);
  const auto samples = path.run(rng, kNanosPerMilli, 60 * kNanosPerSecond);
  const ClockModel m = fit_clock(samples, 20 * kNanosPerMilli);
  CHECK_NEAR(m.skew, path.skew, 0.5e-6);
  CHECK_NEAR(m.offset_at(path.t0 + 30 * kNanosPerSecond),
             path.theta(static_cast<double>(path.t0 + 30 * kNanosPerSecond)), 50e3);
}

TEST_CASE(clock_sync_degenerate_inputs) {
  const ClockModel empty = fit_clock({});
  CHECK(empty.offset == 0.0 && empty.skew == 0.0);

  SyntheticPath path;
  Rng rng(3);
  const auto one_window = path.run(rng, 10 * kNanosPerMilli, 500 * kNanosPerMilli);
  const ClockModel m = fit_clock(one_window);
  CHECK(m.skew == 0.0);
  CHECK_NEAR(m.offset, path.offset, 100e3);

  bool threw = false;
  try {
    fit_clock(one_window, 0);
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  CHECK(threw);
}
