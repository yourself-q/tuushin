#pragma once

#include <vector>

#include "tuushin/time.hpp"

namespace tuushin::net {

// The four timestamps of one request/response exchange, NTP style:
// the client sends at t1 and receives the answer at t4 (client clock), the
// server receives at t2 and answers at t3 (server clock).
struct TimestampSample {
  Nanos t1 = 0, t2 = 0, t3 = 0, t4 = 0;

  Nanos rtt() const { return (t4 - t1) - (t3 - t2); }
  // One-way delays plus or minus the clock offset server - client.
  Nanos raw_forward() const { return t2 - t1; }
  Nanos raw_backward() const { return t4 - t3; }
};

// Server clock minus client clock as a linear function of client time:
//   theta(t) = offset + skew * (t - t_ref).
// Crystal oscillators of ordinary computers differ by tens of ppm, i.e. some
// milliseconds per minute, so a constant offset is not enough for one-way
// delays over a measurement of more than a few seconds.
struct ClockModel {
  Nanos t_ref = 0;
  double offset = 0.0;  // ns
  double skew = 0.0;    // ns per ns
  double offset_at(Nanos client_time) const {
    return offset + skew * static_cast<double>(client_time - t_ref);
  }
};

// Fits a ClockModel from exchanges sorted by t1.
//
// Method: queueing only ever adds delay, so the smallest raw delay in a
// window is the least-queued one. Per window of `window` ns, the minima of
// the raw forward and backward delays give
//   theta_w = (min(t2 - t1) - min(t4 - t3)) / 2,
// the offset if the two directions have equal minimum delay (the standard
// assumption, which cannot be avoided without an outside time reference: an
// asymmetry a shifts the offset by a / 2 and moves delay from one direction
// to the other, but leaves the RTT and every delay variation intact). The
// skew is the Theil-Sen slope of theta_w over time (the median of pairwise
// slopes), which ignores windows spoilt by route changes or queues that
// never drained. This follows the lower-envelope idea of V. Paxson, "On
// calibrating measurements of packet transit times", SIGMETRICS 1998, and
// S. B. Moon, P. Skelly and D. Towsley, "Estimation and removal of clock
// skew from network delay measurements", INFOCOM 1999, with a median
// instead of their linear program.
//
// Returns a zero model for no samples and a zero-skew model for one window.
ClockModel fit_clock(const std::vector<TimestampSample>& samples, Nanos window = kNanosPerSecond);

}  // namespace tuushin::net
