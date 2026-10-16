#include <algorithm>
#include <cmath>
#include <vector>

#include "test.hpp"
#include "tuushin/control/noise_tax.hpp"
#include "tuushin/rng.hpp"

using tuushin::Rng;
using tuushin::control::ChangeTax;
using tuushin::control::dither_bounds;
using tuushin::control::NoiseTax;

namespace {

// The distribution of e = round(x + n) - x for n Gaussian of standard
// deviation sigma, in counts: the values e and their probabilities.
struct Pmf {
  std::vector<double> e, p;
};

Pmf reading_error(double x, double sigma) {
  Pmf d;
  const int k = static_cast<int>(std::ceil(12.0 * sigma)) + 2;
  for (int j = -k; j <= k; ++j) {
    const double a = (j - 0.5 - x) / (sigma * std::sqrt(2.0));
    const double b = (j + 0.5 - x) / (sigma * std::sqrt(2.0));
    const double p = 0.5 * (std::erfc(a) - std::erfc(b));
    if (p > 0.0) {
      d.e.push_back(j - x);
      d.p.push_back(p);
    }
  }
  return d;
}

double mean(const Pmf& d) {
  double m = 0.0;
  for (std::size_t i = 0; i < d.e.size(); ++i) m += d.e[i] * d.p[i];
  return m;
}

}  // namespace

TEST_CASE(dither_bounds_follow_the_table) {
  const auto zero = dither_bounds(0.0, 0.0);
  CHECK_NEAR(zero.bias, 0.5, 1e-12);
  CHECK_NEAR(zero.proxy, 0.5, 1e-12);
  const auto half = dither_bounds(0.5, 0.5);
  CHECK_NEAR(half.bias, 0.0022892, 2e-7);
  CHECK_NEAR(half.proxy, 0.5841694, 2e-7);
  // The bias falls with the noise and the proxy grows with it; each is
  // taken at its own end of the range.
  const auto range = dither_bounds(0.45, 0.55);
  CHECK_NEAR(range.bias, dither_bounds(0.45, 0.45).bias, 1e-15);
  CHECK_NEAR(range.proxy, dither_bounds(0.55, 0.55).proxy, 1e-15);
  double last_bias = 1.0, last_proxy = 0.0;
  for (double s = 0.0; s <= 3.0; s += 0.01) {
    const auto d = dither_bounds(s, s);
    CHECK(d.bias <= last_bias);
    CHECK(d.proxy >= last_proxy);
    // The proxy is never below the error's standard deviation.
    CHECK(d.proxy >= std::sqrt(s * s + 1.0 / 12.0) - 1e-9);
    last_bias = d.bias;
    last_proxy = d.proxy;
  }
  // Beyond the table: the standard deviation, with its margin.
  CHECK_NEAR(dither_bounds(2.0, 2.0).proxy, std::sqrt(4.0 + 1.0 / 12.0), 1e-3);
  CHECK(dither_bounds(2.0, 2.0).bias < 1e-6);
}

// The bounds hold for the exact distribution of the rounded reading at
// noises between the grid points too: the conditional mean within the
// bias, and the moment generating function of the rest within the
// sub-Gaussian bound, wherever the device lies in its count.
TEST_CASE(dither_bounds_hold_for_rounded_gaussian_readings) {
  for (double sigma : {0.02, 0.12, 0.27, 0.33, 0.48, 0.52, 0.71, 0.93, 1.2}) {
    const auto d = dither_bounds(sigma, sigma);
    double worst_bias = 0.0, worst_ratio = 0.0;
    for (int i = 0; i <= 200; ++i) {
      const double x = 0.5 * i / 200.0;
      const Pmf pmf = reading_error(x, sigma);
      const double b = mean(pmf);
      worst_bias = std::max(worst_bias, std::fabs(b));
      for (int j = -30; j <= 30; ++j) {
        const double t = (j < 0 ? -1.0 : 1.0) * std::pow(10.0, std::abs(j) / 10.0 - 1.5);
        if (j == 0) continue;
        double m = 0.0;
        for (std::size_t k = 0; k < pmf.e.size(); ++k) m += pmf.p[k] * std::exp(t * (pmf.e[k] - b));
        // log M(t) <= t^2 s^2 / 2
        worst_ratio = std::max(worst_ratio, 2.0 * std::log(m) / (t * t * d.proxy * d.proxy));
      }
    }
    // Beyond half a count the bound is the Fourier series' first term,
    // which is the bias itself: allow for the rounding of the sum above.
    CHECK(worst_bias <= d.bias + 1e-14);
    CHECK(worst_ratio <= 1.0 + 1e-9);
  }
}

TEST_CASE(noise_tax_charges_rises_and_lets_falls_go) {
  NoiseTax tax(0.01, 0.5, 1e-3, 0.25);
  const double opening = tax.open();
  CHECK_NEAR(opening, tax.bound(0.0), 1e-15);
  CHECK_NEAR(opening, std::sqrt(0.5 * 0.25 * 0.5 * 0.25 * 2.0 * std::log(1e3)), 1e-12);
  CHECK(tax.open() == 0.0);
  // A rise from rest to 2 N: the bias part and U(V*) - U(0), with
  // V* = s^2 (2^2 + 2^2).
  const double rise = tax.charge(0.0, 2.0);
  CHECK_NEAR(rise, 0.02 * 2.0 + tax.bound(0.25 * 8.0) - opening, 1e-12);
  // Falls towards zero are free.
  CHECK(tax.charge(2.0, 1.0) == 0.0);
  CHECK(tax.charge(1.0, 0.0) == 0.0);
  CHECK_NEAR(tax.variance(), 0.25 * (4.0 + 1.0 + 1.0), 1e-12);
  // Rising again pays only where V* passes its peak.
  const double again = tax.charge(0.0, 2.0);
  CHECK_NEAR(again, 0.04 + tax.bound(0.25 * 14.0) - tax.bound(0.25 * 8.0), 1e-12);
  // A change of sign pays its whole new force's bias part.
  const double flip = tax.charge(2.0, -1.0);
  CHECK(flip >= 0.02 * 1.0);
  CHECK_NEAR(tax.charged(), tax.bound(tax.variance() + 0.25 * 1.0), 1e-12);
}

TEST_CASE(noise_tax_plan_bounds_every_charge) {
  Rng rng(7);
  NoiseTax tax(0.003, 0.58, 1e-3, 0.25);
  tax.open();
  double prev = 0.0;
  int changes = 0;
  for (int k = 0; k < 20000; ++k) {
    const double phi = rng.uniform() < 0.3 ? 0.0 : 6.0 * (rng.uniform() - 0.3);
    const ChangeTax plan = tax.plan();
    const double kept = phi * prev > 0.0 ? std::fabs(prev) : 0.0;
    const double most = plan.rise * std::max(0.0, std::fabs(phi) - kept) +
                        2.0 * plan.slope * std::max(0.0, phi * (phi - prev));
    CHECK_NEAR(plan.most(prev, phi), most, 1e-15);
    const double c = tax.charge(prev, phi);
    // The charge is a difference of two values of U: rounding.
    const double slack = most * 1e-12 + tax.charged() * 1e-14;
    CHECK(c >= 0.0);
    CHECK(c <= most + slack);
    if (c > most + slack) break;
    changes += phi != prev;
    prev = phi;
  }
  CHECK(changes > 10000);
}

// The plan on the tangent at a target bounds every change too, the more
// tightly the nearer the change is to its target, and is exact there; cost()
// is what charge() returns.
TEST_CASE(noise_tax_plan_at_a_target_bounds_every_charge_and_meets_it_there) {
  Rng rng(9);
  NoiseTax tax(0.003, 0.58, 1e-3, 0.25);
  tax.open();
  // From rest to 2 N: the peak's tangent takes several times the charge.
  CHECK(tax.plan().most(0.0, 2.0) > 4.0 * tax.cost(0.0, 2.0));
  CHECK_NEAR(tax.plan(0.0, 2.0).most(0.0, 2.0), tax.cost(0.0, 2.0), 1e-12);
  double prev = 0.0;
  for (int k = 0; k < 5000; ++k) {
    const double target = 6.0 * (rng.uniform() - 0.3);
    const ChangeTax plan = tax.plan(prev, target);
    CHECK(plan.fixed >= 0.0);
    for (int j = 0; j < 8; ++j) {
      const double phi = j == 0 ? target : 6.0 * (rng.uniform() - 0.3);
      const double c = tax.cost(prev, phi);
      const double most = plan.most(prev, phi);
      CHECK(c <= most + most * 1e-12 + tax.charged() * 1e-14);
      if (j == 0 && plan.fixed > 0.0) CHECK_NEAR(most, c, 1e-12 * (1.0 + c));
    }
    const double phi = rng.uniform() < 0.3 ? 0.0 : target;
    const double expected = tax.cost(prev, phi);
    CHECK_NEAR(tax.charge(prev, phi), expected, 1e-15);
    prev = phi;
  }
}

// Monte Carlo: readings of a device that dwells near the edge of a count,
// rounded after Gaussian noise, and a force that switches on what the
// readings before each change showed. The books' error less the newest
// reading's noise times the force stays within what the tax charged, at
// every reading, in all but a fraction delta of the runs; and over a long
// chatter the tax is far below the one that assumes the error in a count.
TEST_CASE(noise_tax_covers_the_books_with_its_probability) {
  constexpr double kDelta = 0.1;
  constexpr int kRuns = 300, kSteps = 3000;
  for (double sigma : {0.3, 0.5}) {
    const auto d = dither_bounds(sigma, sigma);
    Rng rng(11);
    int failed = 0;
    double ratio = 0.0;  // charged / the interval tax, at the end, summed
    for (int run = 0; run < kRuns; ++run) {
      NoiseTax tax(d.bias, d.proxy, kDelta, 0.25);
      double books = 0.0;  // -sum dphi_j e_j over the changes so far
      double charged = tax.open(), interval = 0.0;
      double phi = 0.0, y_prev = 0.0;
      bool bad = false;
      const double centre = 0.5 + 0.2 * (rng.uniform() - 0.5);
      for (int k = 0; k < kSteps; ++k) {
        const double x = centre + 0.3 * std::sin(0.002 * k + run) + 0.05 * rng.normal();
        const double b = mean(reading_error(x, sigma));
        const double e = std::round(x + sigma * rng.normal()) - x;
        // At reading k, before the change on it: B_k - W_k less
        // phi_{k-1} xi_k is -sum_{j<k} dphi_j e_j + phi_{k-1} b(x_k).
        if (books + phi * b > charged + 1e-12) bad = true;
        // The change on reading k, decided from the readings before it.
        const double next = y_prev >= 1.0 ? 2.0 : (y_prev >= 0.0 ? -1.0 : 0.5);
        books -= (next - phi) * e;
        charged += tax.charge(phi, next);
        interval += std::max(0.0, std::fabs(next) - (next * phi > 0.0 ? std::fabs(phi) : 0.0));
        phi = next;
        y_prev = x + e;
      }
      failed += bad;
      ratio += charged / interval;
    }
    CHECK(failed <= kDelta * kRuns);
    // Charged per newton of rise, in counts: well below the interval
    // tax's one count.
    CHECK(ratio / kRuns < 0.5);
  }
}
