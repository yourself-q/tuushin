#include "tuushin/control/reading_noise.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace tuushin::control {

namespace {

// u is clipped at this many times the current variance: three standard
// deviations beyond the threshold.
constexpr double kClip = 9.0;

// E[(|z| - t)_+^2] for a standard normal z.
double truncated_moment(double t) {
  const double tail = 0.5 * std::erfc(t / std::sqrt(2.0));
  const double density = std::exp(-0.5 * t * t) / std::sqrt(2.0 * 3.14159265358979323846);
  return 2.0 * ((1.0 + t * t) * tail - t * density);
}

// The variance sigma^2 >= floor at which E[(|z| - c)_+^2] = m for z of
// variance sigma^2. The moment is sigma^2 truncated_moment(c / sigma),
// which grows with sigma, so bisect in log sigma.
double solve_variance(double m, double c, double floor) {
  const double lo_sigma = std::sqrt(floor);
  auto moment = [c](double sigma) { return sigma * sigma * truncated_moment(c / sigma); };
  if (m <= moment(lo_sigma)) return floor;
  double lo = std::log(lo_sigma), hi = lo + 1.0;
  while (moment(std::exp(hi)) < m) hi += 1.0;
  for (int i = 0; i < 40; ++i) {
    const double mid = 0.5 * (lo + hi);
    (moment(std::exp(mid)) < m ? lo : hi) = mid;
  }
  return std::exp(lo + hi);
}

}  // namespace

ReadingNoise::ReadingNoise(double dt, double resolution, double time_constant)
    : dt_(dt),
      floor_(resolution * resolution / 12.0),
      threshold_(resolution / std::sqrt(6.0)),
      rate_(1.0 / time_constant),
      var_(floor_) {
  if (!(dt > 0.0) || !(resolution > 0.0) || !(time_constant > 0.0)) {
    throw std::invalid_argument("ReadingNoise: dt, resolution and time_constant must be positive");
  }
}

void ReadingNoise::observe(double dx, double h, double expected_change) {
  if (!(std::fabs(h - dt_) <= 0.5 * dt_)) {
    have_prev_ = false;
    return;
  }
  if (have_prev_) {
    const double a = 1.0 / h, b = 1.0 / prev_h_;
    // The change of mean velocity, scaled to the error of one reading.
    const double z =
        (dx * a - prev_dx_ * b - expected_change) / std::sqrt(a * a + (a + b) * (a + b) + b * b);
    const double beyond = std::max(0.0, std::fabs(z) - threshold_);
    const double u = std::min(beyond * beyond, kClip * var_);
    mean_u_ += std::min(1.0, h * rate_) * (u - mean_u_);
    var_ = solve_variance(mean_u_, threshold_, floor_);
    ++samples_;
  }
  have_prev_ = true;
  prev_dx_ = dx;
  prev_h_ = h;
}

}  // namespace tuushin::control
