#include <algorithm>
#include <cmath>

#include "test.hpp"
#include "tuushin/control/wall_model.hpp"
#include "tuushin/rng.hpp"

using tuushin::Rng;
using tuushin::control::WallEstimator;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kQ = 55e-6;  // m, an encoder count
constexpr double kWall = 0.02, kStiffness = 5000.0;

struct Fit {
  double lo = 1e9, hi = 0.0, mean = 0.0;  // the model's stiffness after the first second
};

// Two seconds of a slave pressing up to 1.6 mm into the wall and back at
// 0.5 Hz, read through `noise` counts of dither and the rounding, with the
// estimator told of a position error of variance `told` (m^2).
Fit press(double noise, double told) {
  WallEstimator est;
  est.set_position_noise(told);
  Rng rng(3);
  Fit fit;
  int n = 0;
  for (int i = 0; i < 4000; ++i) {
    const double t = i * 1e-3;
    const double depth = 0.8e-3 * (1.0 - std::cos(2.0 * kPi * 0.5 * t));
    const double read = std::round((kWall + depth + rng.normal(0.0, noise * kQ)) / kQ) * kQ;
    est.observe(read, 0.0, kStiffness * depth);
    if (t >= 1.0 && est.model().known()) {
      const double k = est.model().stiffness;
      fit.lo = std::min(fit.lo, k);
      fit.hi = std::max(fit.hi, k);
      fit.mean += k;
      ++n;
    }
  }
  if (n > 0) fit.mean /= n;
  return fit;
}

}  // namespace

TEST_CASE(wall_estimator_corrects_its_stiffness_for_the_positions_noise) {
  // Half a count of dither and the rounding: 31.8 um, which over a window
  // of 100 samples spreads the positions without spreading the forces. The
  // plain fit averages 3500 N/m (2300 to 4400), the corrected 4800 (4000
  // to 5900).
  const double variance = (0.25 + 1.0 / 12.0) * kQ * kQ;
  const Fit naive = press(0.5, 0.0);
  const Fit corrected = press(0.5, variance);
  CHECK(naive.mean < 0.9 * kStiffness);
  CHECK(std::fabs(corrected.mean - kStiffness) < 0.1 * kStiffness);
}
