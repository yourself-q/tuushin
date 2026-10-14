#include "tuushin/control/predictive_pc.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace tuushin::control {

DevicePredictor::DevicePredictor(double mass, double dt) {
  if (!(mass > 0.0) || !(dt > 0.0)) {
    throw std::invalid_argument("DevicePredictor: mass and dt must be positive");
  }
  c_ = dt * dt / (2.0 * mass);
}

void DevicePredictor::observe(double dx, double phi_held) {
  dx2_ = dx1_;
  phi2_ = phi1_;
  dx1_ = dx;
  phi1_ = phi_held;
}

double DevicePredictor::base() const {
  // The history starts at zero, i.e. the device is taken to have been at rest
  // and unforced before the first sample.
  return 2.0 * dx1_ - dx2_ + c_ * phi2_;
}

double DevicePredictor::external_force() const {
  // dx1 - dx2 = c * (F1 + F2) with F_j = H - phi_j.
  return 0.5 * ((dx1_ - dx2_) / c_ + phi1_ + phi2_);
}

double DevicePredictor::velocity_dt() const {
  // dx1 = v1 * dt + c * F1 and v * dt = v1 * dt + 2 c * F1.
  return dx1_ + c_ * (external_force() - phi1_);
}

double DevicePredictor::hold_base(int n) const {
  const double nn = static_cast<double>(n);
  return nn * velocity_dt() + nn * nn * c_ * external_force();
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

}  // namespace

PcResult limit_force_predictive(double budget, double phi, const DevicePredictor& device,
                                int hold_samples) {
  PcResult r;
  r.value = phi;
  if (phi == 0.0) return r;
  const double b = std::max(0.0, budget);
  // W(lambda) = lambda * g - lambda^2 * h with h > 0.
  double l = affordable_scale(phi * device.base(), phi * phi * device.compliance(), b);
  if (hold_samples > 1) {
    l = std::min(l, affordable_scale(phi * device.hold_base(hold_samples),
                                     phi * phi * device.hold_compliance(hold_samples), b));
  }
  if (l >= 1.0) return r;  // the full force is affordable
  r.value = l * phi;
  r.gain = 1.0 - l;
  r.active = true;
  return r;
}

}  // namespace tuushin::control
