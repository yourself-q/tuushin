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

PcResult limit_force_predictive(double budget, double phi, const DevicePredictor& device) {
  PcResult r;
  r.value = phi;
  if (phi == 0.0) return r;
  const double b = std::max(0.0, budget);
  // W(lambda) = lambda * g - lambda^2 * h with h > 0.
  const double g = phi * device.base();
  const double h = phi * phi * device.compliance();
  if (g - h >= -b) return r;  // the full force is affordable
  // Largest lambda with W(lambda) = -b: the positive root of
  // h lambda^2 - g lambda - b = 0. It lies in [0, 1) because W(0) = 0 >= -b
  // and W(1) < -b.
  const double lambda = (g + std::sqrt(g * g + 4.0 * h * b)) / (2.0 * h);
  const double l = std::clamp(lambda, 0.0, 1.0);
  r.value = l * phi;
  r.gain = 1.0 - l;
  r.active = true;
  return r;
}

}  // namespace tuushin::control
