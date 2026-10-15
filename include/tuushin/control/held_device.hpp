#pragma once

#include <array>
#include <vector>

#include "tuushin/control/predictive_pc.hpp"

namespace tuushin::control {

// An extended Kalman filter of a force-controlled device together with what
// holds it: the operator's hand, or an environment it rests against.
//
// Why: to tell how long a device takes to apply a force (its actuation
// delay), compare how well each candidate delay predicts the readings
// (teleop::DeviceModel). DevicePredictor's model, a mass pushed by a force H
// that wanders, is not good enough for that. A hand is a damper and a
// spring as well (OperatorParams), and its damping answers a force step at
// once, through the velocity the step builds: at low frequencies the
// device's response leads the mass's by up to a few milliseconds of phase.
// A filter that sees only the mass takes the lead for an earlier actuation.
// With an exact encoder its innovations are made of the last few samples,
// where the mass dominates, and the delay comes out right. With a coarse one
// the filter averages the reading noise over tens of samples, and a 55 um
// encoder under a 5 N s/m, 300 N/m hand made 2 ms look like 1.4 ms and 1 ms
// like 0.6 ms. With the damping and stiffness known, the same filter put
// both delays on the 62.5 us grid point. The confound is the reason:
// shortening the delay mimics more damping, by about 0.25 ms per N s/m on a
// 0.2 kg device, so the two have to be estimated together. That is what
// this filter does, for one delay; each candidate's score is then that of
// the delay with the damping and stiffness that fit it best (a profile
// likelihood). Open loop under random force steps, the bank (taking the
// longest delay it cannot tell from the best) put 1 and 2 ms from a 55 um
// encoder 0.125 ms short to 0.375 ms long, over hands of 5-20 N s/m and
// 300-1000 N/m, and on them from exact readings. A free device, which
// nothing damps, tells the delays apart less well: up to the 4 ms maximum.
//
// Model: m x'' = H - phi - b x' - k x, with phi the port force as the device
// applied it (the candidate's schedule), H everything else (the hand's pull
// toward where it wants to be, gravity), a random walk of `force_rate`, and
// the damping b and stiffness k random walks slow enough to count as
// constants over the forced responses that show them. The state
// [x, v, H, b, k] is estimated from the readings, x being the position
// relative to the first reading. b and k are kept nonnegative: a hand or an
// environment does not push along with the motion, and the filter would
// otherwise trade a negative stiffness against H. They are kept below m/dt
// and m/dt^2 as well, what a sample resolves: beyond them the device's
// response decays or turns within a sample, and the RK4 steps that advance
// the mean stay well inside their region of stability.
//
// H's walk has to keep up with the hand: one that moves the device 4 cm at
// 0.5 Hz changes its force at up to 80 N/s, over 25 N/s a tenth of the
// time, and with a fixed walk of 20 N/s the filters put the rest into the
// damping, which ran to its bound. DeviceModel adapts the walk's variance
// by the innovations, as DevicePredictor adapts its own, through
// set_force_walk_scale.
class HeldDeviceFilter {
 public:
  // `mass` in kg, `dt` the nominal sample period in s, `resolution` the
  // encoder's in m (0 for exact readings), `force_rate` in N/s.
  HeldDeviceFilter(double mass, double dt, double resolution, double force_rate);

  // Advances over a sample of length h (s) in which the reading moved by dx
  // and the port force was `segments` in turn (their lengths are scaled to
  // h), and updates with the reading.
  void observe(double dx, const std::vector<ForceSegment>& segments, double h);

  // The reading minus its prediction at the last observe(), in m, and its
  // square over the variance the filter expected for it.
  double innovation() const { return innovation_; }
  double normalized_innovation_squared() const { return nis_; }
  // Scales the variance H's random walk gains per second (1 to start).
  void set_force_walk_scale(double scale) { walk_scale_ = scale; }
  double damping() const { return s_[3]; }    // N s/m
  double stiffness() const { return s_[4]; }  // N/m

 private:
  static constexpr int kN = 5;
  using Vec = std::array<double, kN>;
  using Mat = std::array<Vec, kN>;

  void advance(double tau, double phi);
  // Keeps estimate i within [0, most].
  void bound(int i, double most);

  double mass_;
  double dt_;
  double noise_;  // reading variance, m^2
  double force_walk_;  // H's variance growth per s, N^2/s
  double walk_scale_ = 1.0;
  double y_ = 0.0;     // the reading relative to the first, m
  Vec s_{};            // x, v, H, b, k
  Mat p_{};
  double innovation_ = 0.0;
  double nis_ = 0.0;
};

}  // namespace tuushin::control
