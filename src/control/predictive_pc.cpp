#include "tuushin/control/predictive_pc.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace tuushin::control {

namespace {

// A new filter's uncertainty about the velocity (m/s) and H (N): a device
// that may be moving at a centimetre per second and pushed by a few newtons.
constexpr double kInitialVelocityVar = 1e-4;
constexpr double kInitialForceVar = 25.0;
// After a sample the model does not describe: a hand that may have moved on
// at up to a decimetre per second and changed its push by ten newtons.
constexpr double kLostVelocityVar = 1e-2;
constexpr double kLostForceVar = 100.0;

}  // namespace

DevicePredictor::DevicePredictor(double mass, double dt, double resolution, double force_rate)
    : dt_(dt), mass_(mass), q_(resolution) {
  if (!(mass > 0.0) || !(dt > 0.0)) {
    throw std::invalid_argument("DevicePredictor: mass and dt must be positive");
  }
  if (!(resolution >= 0.0) || !(force_rate >= 0.0)) {
    throw std::invalid_argument("DevicePredictor: resolution and force_rate must not be negative");
  }
  c_ = dt * dt / (2.0 * mass);
  noise_ = q_ * q_ / 12.0;
  h_walk_ = (force_rate * dt) * (force_rate * dt);
  p_[0][0] = noise_;
  p_[1][1] = kInitialVelocityVar;
  p_[2][2] = kInitialForceVar;
}

void DevicePredictor::observe(double dx, double phi_held, double h) {
  const bool regular = std::fabs(h - dt_) <= 0.5 * dt_;
  push(dx, phi_held, regular ? h / dt_ : 1.0, regular);
  held_ = phi_held;
}

void DevicePredictor::observe_irregular(double dx, double phi_held) {
  push(dx, phi_held, 1.0, false);
  held_ = 0.0;
}

void DevicePredictor::push(double dx, double phi_held, double r, bool regular) {
  dx2_ = dx1_;
  phi2_ = phi1_;
  r2_ = r1_;
  dx1_ = dx;
  phi1_ = phi_held;
  r1_ = r;
  regular_ = regular ? std::min(regular_ + 1, 2) : 0;
  if (q_ > 0.0) filter(dx, phi_held, r, regular);
}

void DevicePredictor::filter(double dx, double phi_held, double r, bool regular) {
  if (!regular) {
    // The model did not hold over this sample, so its residual says nothing
    // about v or H: restart from the reading, with the estimates kept as
    // guesses and their uncertainty raised.
    e_ = 0.0;
    const double pv = p_[1][1] + kLostVelocityVar, ph = p_[2][2] + kLostForceVar;
    for (auto& row : p_) {
      for (double& x : row) x = 0.0;
    }
    p_[0][0] = noise_;
    p_[1][1] = pv;
    p_[2][2] = ph;
    return;
  }
  // Predict over the sample, of length h, with the held force: the exact
  // transition of a mass under a constant net force H - phi.
  const double h = r * dt_;
  const double a = h * h / (2.0 * mass_), b = h / mass_;
  const double f[3][3] = {{1.0, h, a}, {0.0, 1.0, b}, {0.0, 0.0, 1.0}};
  e_ += v_ * h + a * (h_ - phi_held) - dx;  // the reading moved by dx
  v_ += b * (h_ - phi_held);
  double fp[3][3], p[3][3];
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      fp[i][j] = 0.0;
      for (int k = 0; k < 3; ++k) fp[i][j] += f[i][k] * p_[k][j];
    }
  }
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      p[i][j] = 0.0;
      for (int k = 0; k < 3; ++k) p[i][j] += fp[i][k] * f[j][k];
    }
  }
  p[2][2] += h_walk_ * r;
  // Update with the reading; the innovation is reading - estimate = -e.
  const double s = p[0][0] + noise_;
  const double k[3] = {p[0][0] / s, p[1][0] / s, p[2][0] / s};
  const double nu = -e_;
  e_ += k[0] * nu;
  v_ += k[1] * nu;
  h_ += k[2] * nu;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) p_[i][j] = p[i][j] - k[i] * p[0][j];
  }
}

double DevicePredictor::base() const {
  // The history starts at zero, i.e. the device is taken to have been at rest
  // and unforced before the first sample.
  return hold_base(1.0);
}

double DevicePredictor::external_force() const {
  if (q_ > 0.0) return h_;
  // With r_j = h_j / dt and F_j = H - phi_j, dx_j / r_j is the displacement
  // at the mean velocity over sample j scaled to dt, and the mean velocities
  // of two adjacent samples differ by (F_1 h_1 + F_2 h_2) / (2 m). For equal
  // samples: dx1 - dx2 = c * (F1 + F2).
  return ((dx1_ / r1_ - dx2_ / r2_) / c_ + phi1_ * r1_ + phi2_ * r2_) / (r1_ + r2_);
}

double DevicePredictor::velocity_dt() const {
  if (q_ > 0.0) return v_ * dt_;
  // The velocity at the end of the last sample: its mean velocity plus half
  // the change over it, F_1 h_1 / (2 m), times dt.
  return dx1_ / r1_ + c_ * r1_ * (external_force() - phi1_);
}

double DevicePredictor::hold_base(double n) const {
  return n * velocity_dt() + n * n * c_ * external_force();
}

namespace {

// Largest lambda in [0, 1] with lambda * g - lambda^2 * h >= -b, for h > 0
// and b >= 0.
double affordable_scale(double g, double h, double b) {
  if (g - h >= -b) return 1.0;
  // The positive root of h lambda^2 - g lambda - b = 0. It lies in [0, 1)
  // because the left side is <= 0 at lambda = 0 and > 0 at lambda = 1.
  const double lambda = (g + std::sqrt(g * g + 4.0 * h * b)) / (2.0 * h);
  return std::clamp(lambda, 0.0, 1.0);
}

// The same with the tax q * max(0, lambda * F - a) added to the cost, for
// F = |phi| > 0 and a >= 0 the part of the held force with phi's sign.
double affordable_scale_taxed(double g, double h, double b, double F, double a, double q) {
  const double l = affordable_scale(g, h, b);
  if (q <= 0.0 || l * F <= a) return l;
  // Beyond lambda0 = a / F the cost is steeper by q * F. The constraint is
  // still concave and holds at lambda0 (untaxed there, and lambda0 < l), so
  // its largest root beyond lambda0 is the answer.
  return std::max(a / F, affordable_scale(g - q * F, h, b + q * a));
}

}  // namespace

double quantization_tax(double q, double prev, double phi) {
  const double kept = phi * prev > 0.0 ? std::fabs(prev) : 0.0;
  return q * std::max(0.0, std::fabs(phi) - kept);
}

PcResult limit_force_predictive(double budget, double phi, const DevicePredictor& device,
                                int hold_samples) {
  PcResult r;
  r.value = phi;
  if (phi == 0.0) return r;
  if (!device.ready()) {
    r.value = 0.0;
    r.gain = 1.0;
    r.active = true;
    return r;
  }
  const double b = std::max(0.0, budget);
  // W(lambda) = lambda * g - lambda^2 * h with h > 0, less the tax, which
  // is paid once whatever the horizon.
  const double q = device.resolution();
  const double F = std::fabs(phi);
  const double kept = phi * device.held_force() > 0.0 ? std::fabs(device.held_force()) : 0.0;
  double l = affordable_scale_taxed(phi * device.base(), phi * phi * device.compliance(), b, F,
                                    kept, q);
  if (hold_samples > 1) {
    l = std::min(l, affordable_scale_taxed(phi * device.hold_base(hold_samples),
                                           phi * phi * device.hold_compliance(hold_samples), b,
                                           F, kept, q));
  }
  if (l >= 1.0) return r;  // the full force is affordable
  r.value = l * phi;
  r.gain = 1.0 - l;
  r.active = true;
  return r;
}

}  // namespace tuushin::control
