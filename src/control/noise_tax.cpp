#include "tuushin/control/noise_tax.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace tuushin::control {

namespace {

constexpr double kPi = 3.14159265358979323846;

// The suprema over x of |E[e | x]| and of the sub-Gaussian proxy of
// e - E[e | x], in counts, for e = round(x + n) - x and n Gaussian of
// standard deviation 0.05 i counts, i = 0..20: computed from the exact
// distribution of round(x + n) on 5001 places x in half a count and the
// moment generating function at 2 x 1601 points t from 1e-3 to 200 /
// sigma per count, rounded up in the seventh decimal. At zero noise the
// bias is the rounding's q / 2, and the proxy the limit of a reading that
// falls on either side of a count's edge with even odds. A finer grid
// of noises showed both convex: the chords between grid points lie above.
constexpr double kStep = 0.05;
constexpr double kBias[] = {0.5,       0.3773251, 0.2855439, 0.2092437, 0.1450683, 0.0927227,
                            0.0538666, 0.0283599, 0.0135280, 0.0058466, 0.0022893, 0.0008122,
                            0.0002611, 0.0000761, 0.0000202, 0.0000049, 0.0000011, 0.0000003,
                            0.0000001, 0.0000001, 0.0000001};
constexpr double kProxy[] = {0.5,       0.5000001, 0.5000001, 0.5000001, 0.5000012, 0.5001268,
                             0.5017134, 0.5084777, 0.5242529, 0.5500654, 0.5841695, 0.6238423,
                             0.6667814, 0.7115204, 0.7572736, 0.8036597, 0.8504953, 0.8976834,
                             0.9451634, 0.9928915, 1.0408331};
constexpr int kLast = 20;
// Beyond the table the proxy came out at sqrt(sigma^2 + 1 / 12) to six
// digits (at 1.25, 1.5, 2 and 3 counts).
constexpr double kProxyMargin = 1e-4;

double interpolate(const double* table, double sigma) {
  const double at = sigma / kStep;
  const int i = std::min(kLast - 1, static_cast<int>(at));
  const double f = at - i;
  return table[i] + f * (table[i + 1] - table[i]);
}

// sum_k exp(-2 pi^2 k^2 sigma^2) / (pi k): the bias's Fourier series term
// by term, an upper bound for every sigma > 0.
double fourier_bias(double sigma) {
  const double a = 2.0 * kPi * kPi * sigma * sigma;
  double sum = 0.0;
  for (int k = 1; k <= 64; ++k) {
    const double term = std::exp(-a * k * k) / (kPi * k);
    sum += term;
    if (term < 1e-18) break;
  }
  return sum;
}

}  // namespace

DitherBounds dither_bounds(double lo, double hi) {
  if (!(lo >= 0.0) || !(hi >= lo)) {
    throw std::invalid_argument("dither_bounds: need 0 <= lo <= hi");
  }
  DitherBounds d;
  d.bias = lo < kLast * kStep ? interpolate(kBias, lo) : kBias[kLast];
  if (lo > 0.0) d.bias = std::min(d.bias, fourier_bias(lo));
  d.proxy = hi < kLast * kStep ? interpolate(kProxy, hi)
                               : std::sqrt(hi * hi + 1.0 / 12.0) * (1.0 + kProxyMargin);
  return d;
}

NoiseTax::NoiseTax(double bias, double proxy, double delta, double force_scale)
    : bias_(bias),
      proxy_(proxy),
      rho_(proxy * force_scale * proxy * force_scale),
      log_delta2_(2.0 * std::log(1.0 / delta)) {
  if (!(bias >= 0.0) || !(proxy > 0.0) || !(delta > 0.0 && delta < 1.0) ||
      !(force_scale > 0.0)) {
    throw std::invalid_argument("NoiseTax: need bias >= 0, proxy > 0, 0 < delta < 1, force_scale > 0");
  }
}

double NoiseTax::bound(double v) const {
  const double w = v + rho_;
  return std::sqrt(w * (std::log(w / rho_) + log_delta2_));
}

double NoiseTax::open() {
  if (opened_) return 0.0;
  opened_ = true;
  peak_ = v_;
  paid_ = bound(peak_);
  return paid_;
}

double NoiseTax::slope(double v) const {
  // U'(V) = (log((V + rho) / rho) + 2 log(1 / delta) + 1) / (2 U(V)).
  const double w = v + rho_;
  return (std::log(w / rho_) + log_delta2_ + 1.0) / (2.0 * bound(v));
}

double NoiseTax::charge(double prev, double phi) {
  double out = open();
  if (phi == prev) return out;
  // The bias part: 2 bias per newton by which |phi| grows.
  const double kept = phi * prev > 0.0 ? std::fabs(prev) : 0.0;
  out += 2.0 * bias_ * std::max(0.0, std::fabs(phi) - kept);
  const double s2 = proxy_ * proxy_;
  v_ += s2 * (phi - prev) * (phi - prev);
  const double reserved = v_ + s2 * phi * phi;
  if (reserved > peak_) {
    peak_ = reserved;
    const double u = bound(peak_);
    out += u - paid_;
    paid_ = u;
  }
  return out;
}

double NoiseTax::cost(double prev, double phi) const {
  if (phi == prev) return 0.0;
  const double kept = phi * prev > 0.0 ? std::fabs(prev) : 0.0;
  double out = 2.0 * bias_ * std::max(0.0, std::fabs(phi) - kept);
  const double s2 = proxy_ * proxy_;
  const double reserved = v_ + s2 * (phi - prev) * (phi - prev) + s2 * phi * phi;
  if (reserved > peak_) out += bound(reserved) - bound(peak_);
  return out;
}

ChangeTax NoiseTax::plan() const { return {2.0 * bias_, proxy_ * proxy_ * slope(peak_), 0.0}; }

ChangeTax NoiseTax::plan(double prev, double target) const {
  // A change to phi moves V* from v_ + s^2 prev^2 by d = 2 s^2 phi (phi -
  // prev), and U(V*) - U(peak) <= U(x) + U'(x) (V* - x) - U(peak) for every
  // x: with x the V* of the target, at most fixed + U'(x) d, fixed =
  // U(x) - U(peak) - U'(x) (x - v_ - s^2 prev^2) where that is positive.
  const double s2 = proxy_ * proxy_;
  const double from = v_ + s2 * prev * prev;
  const double d = 2.0 * s2 * target * (target - prev);
  const double x = from + d;
  if (!(x > peak_)) return plan();
  const double du = slope(x);
  const double fixed = std::max(0.0, bound(x) - bound(peak_) - du * d);
  return {2.0 * bias_, s2 * du, fixed};
}

}  // namespace tuushin::control
