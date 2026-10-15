#include "tuushin/control/held_device.hpp"

#include <algorithm>
#include <cmath>

namespace tuushin::control {

namespace {

// Exact readings still need a reading variance for the gain: a nanometre.
constexpr double kExactNoise = 1e-18;
// A new filter's uncertainty: velocity (m/s), H (N), damping (N s/m) and
// stiffness (N/m) of a device that may be moving at a centimetre per
// second, pushed by a few newtons and held by anything from nothing to a
// firm hand.
constexpr double kInitialVelocityVar = 1e-4;
constexpr double kInitialForceVar = 25.0;
constexpr double kInitialDampingVar = 100.0;
constexpr double kInitialStiffnessVar = 1e6;
// How fast the damping and stiffness may drift: 0.3 N s/m and 30 N/m per
// square root of a second, a grip that changes over seconds. Faster walks
// make the filter chase the readings' noise with them; slower, a grip it
// never catches up with. The simulated hands do not change their grip, so
// only the first shows there: open loop, from a 55 um encoder, a hundred
// times these walks put a held device's delays of 1 and 2 ms up to 0.44
// and 0.31 ms long and a free device's at 2-4 ms. A tenth of them, in the
// teleoperator, kept a coarse master's estimates a little closer (0.875-
// 1.44 ms for 1 ms, against 0.75-1.44) but let an exact slave generate
// 30 uJ.
constexpr double kDampingWalk = 0.09;    // (N s/m)^2 per s
constexpr double kStiffnessWalk = 900.0;  // (N/m)^2 per s

}  // namespace

HeldDeviceFilter::HeldDeviceFilter(double mass, double dt, double resolution, double force_rate)
    : mass_(mass),
      dt_(dt),
      noise_(std::max(resolution * resolution / 12.0, kExactNoise)),
      force_walk_(force_rate * force_rate * dt) {
  p_[0][0] = noise_;
  p_[1][1] = kInitialVelocityVar;
  p_[2][2] = kInitialForceVar;
  p_[3][3] = kInitialDampingVar;
  p_[4][4] = kInitialStiffnessVar;
}

void HeldDeviceFilter::advance(double tau, double phi) {
  if (tau <= 0.0) return;
  // The mean through tau with phi, H, b and k held: RK4 in steps of at most
  // a sample, whose error over a millisecond is ten orders below the motion
  // for any grip a hand has, and which the bounds on b and k keep stable.
  const double h = s_[2], b = s_[3], k = s_[4];
  auto acc = [&](double x, double v) { return (h - phi - b * v - k * x) / mass_; };
  const int n = std::max(1, static_cast<int>(std::ceil(tau / dt_ - 1e-9)));
  const double t = tau / n;
  for (int i = 0; i < n; ++i) {
    const double x = s_[0], v = s_[1];
    const double x1 = v, v1 = acc(x, v);
    const double x2 = v + 0.5 * t * v1, v2 = acc(x + 0.5 * t * x1, v + 0.5 * t * v1);
    const double x3 = v + 0.5 * t * v2, v3 = acc(x + 0.5 * t * x2, v + 0.5 * t * v2);
    const double x4 = v + t * v3, v4 = acc(x + t * x3, v + t * v3);
    s_[0] = x + t / 6.0 * (x1 + 2.0 * x2 + 2.0 * x3 + x4);
    s_[1] = v + t / 6.0 * (v1 + 2.0 * v2 + 2.0 * v3 + v4);
  }
}

void HeldDeviceFilter::observe(double dx, const std::vector<ForceSegment>& segments, double h) {
  y_ += dx;
  if (h <= 0.0) return;
  // The transition's Jacobian at the start of the sample, to second order
  // in h; the force enters additively and does not change it.
  const double x = s_[0], v = s_[1], b = s_[3], k = s_[4];
  const double c = h * h / (2.0 * mass_), g = h / mass_;
  Mat f{};
  for (int i = 0; i < kN; ++i) f[i][i] = 1.0;
  f[0] = {1.0 - k * c, h - b * c, c, -v * c, -x * c};
  f[1] = {-k * g, 1.0 - b * g, g, -v * g, -x * g};
  double total = 0.0;
  for (const auto& seg : segments) total += seg.samples;
  const double scale = total > 0.0 ? h / total : 0.0;
  for (const auto& seg : segments) advance(seg.samples * scale, seg.phi);
  Mat fp{}, p{};
  for (int i = 0; i < kN; ++i) {
    for (int j = 0; j < kN; ++j) {
      for (int l = 0; l < kN; ++l) fp[i][j] += f[i][l] * p_[l][j];
    }
  }
  for (int i = 0; i < kN; ++i) {
    for (int j = 0; j < kN; ++j) {
      for (int l = 0; l < kN; ++l) p[i][j] += fp[i][l] * f[j][l];
    }
  }
  p[2][2] += force_walk_ * walk_scale_ * h;
  p[3][3] += kDampingWalk * h;
  p[4][4] += kStiffnessWalk * h;
  const double s = p[0][0] + noise_;
  innovation_ = y_ - s_[0];
  nis_ = innovation_ * innovation_ / s;
  Vec gain;
  for (int i = 0; i < kN; ++i) gain[i] = p[i][0] / s;
  for (int i = 0; i < kN; ++i) s_[i] += gain[i] * innovation_;
  for (int i = 0; i < kN; ++i) {
    for (int j = 0; j < kN; ++j) p_[i][j] = p[i][j] - gain[i] * p[0][j];
  }
  bound(3, mass_ / dt_);
  bound(4, mass_ / (dt_ * dt_));
}

void HeldDeviceFilter::bound(int i, double most) {
  const double target = std::clamp(s_[i], 0.0, most);
  if (target == s_[i]) return;
  // The estimate is projected onto the bound: the others move as they are
  // correlated with this one (Simon, "Kalman filtering with state
  // constraints: a survey of linear and nonlinear algorithms", 2010).
  // Clamping it alone left them inconsistent with it, and with an exact
  // encoder the innovations grew from there until the damping sat on its
  // bound. The covariance is left as it is: projecting it as well takes the
  // estimate's variance to zero, after which the filter no longer moves it
  // off the bound when the readings ask for it, and the bank leaned toward
  // short delays.
  const double var = p_[i][i];
  if (var > 0.0) {
    const double miss = s_[i] - target;
    const Vec col = p_[i];
    for (int j = 0; j < kN; ++j) s_[j] -= col[j] / var * miss;
  }
  s_[i] = target;
}

}  // namespace tuushin::control
