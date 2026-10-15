#include <cmath>
#include <stdexcept>

#include "test.hpp"
#include "tuushin/control/reading_noise.hpp"
#include "tuushin/rng.hpp"

using tuushin::Rng;
using tuushin::control::ReadingNoise;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDt = 1e-3;

// A hand's stroke: 30 mm out and back once a second.
double stroke(double t) { return 0.015 * (1.0 - std::cos(2.0 * kPi * t)); }

struct Run {
  double last_sigma = 0.0;
  double mean_sigma = 0.0;  // over the last three seconds
  double min_sigma = 1e9, max_sigma = 0.0;
};

// Feeds `seconds` of readings of `x(t)`, quantized to q after Gaussian noise
// of `noise` counts, to the estimator; `expected` gives the change of mean
// velocity the estimator is told of.
template <class X, class E>
Run feed(ReadingNoise& est, double q, double noise, double seconds, X x, E expected,
         std::uint64_t seed = 1) {
  Rng rng(seed);
  auto read = [&](double t) { return q * std::round((x(t) + noise * q * rng.normal()) / q); };
  Run r;
  double prev = read(0.0);
  const int n = static_cast<int>(seconds / kDt);
  int counted = 0;
  for (int k = 1; k <= n; ++k) {
    const double t = k * kDt;
    const double now = read(t);
    est.observe(now - prev, kDt, expected(t));
    prev = now;
    const double sigma = std::sqrt(est.variance());
    if (t > seconds - 3.0) {
      r.mean_sigma += sigma;
      ++counted;
      r.min_sigma = std::min(r.min_sigma, sigma);
      r.max_sigma = std::max(r.max_sigma, sigma);
    }
    r.last_sigma = sigma;
  }
  r.mean_sigma /= counted;
  return r;
}

double none(double) { return 0.0; }

// The change of mean velocity the stroke makes from one sample to the
// next, as a device model that knows the forces would expect it.
double stroke_change(double t) { return (stroke(t) - 2.0 * stroke(t - kDt) + stroke(t - 2.0 * kDt)) / kDt; }

}  // namespace

TEST_CASE(reading_noise_of_a_quantized_encoder_is_the_quantization) {
  // A device moving by less than a count per sample reads a sawtooth, and
  // one moving faster a staircase: neither shows a second difference beyond
  // a count, so the estimate stays at q^2 / 12. (On the micrometre encoder
  // the stroke's acceleration alone makes two counts, which the device
  // model explains.)
  for (const double q : {55e-6, 7.7e-6, 1e-6}) {
    ReadingNoise est(kDt, q);
    const Run r = feed(est, q, 0.0, 5.0, stroke, stroke_change);
    CHECK(est.samples() == 4999);
    CHECK_NEAR(r.max_sigma, q / std::sqrt(12.0), 1e-12 * q);
    CHECK_NEAR(r.min_sigma, q / std::sqrt(12.0), 1e-12 * q);
  }
}

TEST_CASE(reading_noise_finds_dither_beyond_a_count) {
  // Gaussian noise before the rounding: the readings' error has the
  // variance noise^2 + q^2 / 12 (for noise of half a count or more, where
  // the rounding error is nearly independent of the reading).
  for (const double q : {55e-6, 7.7e-6}) {
    for (const double noise : {0.5, 1.0, 2.0}) {
      ReadingNoise est(kDt, q);
      const Run r = feed(est, q, noise, 6.0, stroke, none, 7);
      const double truth = q * std::sqrt(noise * noise + 1.0 / 12.0);
      CHECK_NEAR(r.mean_sigma / truth, 1.0, 0.06);
      CHECK(r.max_sigma < 1.25 * truth);
      CHECK(r.min_sigma > 0.75 * truth);
    }
  }
}

TEST_CASE(reading_noise_subtracts_the_expected_acceleration) {
  // On an encoder of a micrometre an acceleration of 50 m/s^2 is fifty
  // counts of second difference a sample: told of it the estimator sees
  // only the quantization, not told it takes the acceleration for noise.
  const double q = 1e-6, a = 50.0;
  auto x = [a](double t) { return 0.5 * a * t * t; };
  ReadingNoise told(kDt, q), blind(kDt, q);
  const Run r = feed(told, q, 0.0, 4.0, x, [a](double) { return a * kDt; });
  const Run b = feed(blind, q, 0.0, 4.0, x, none);
  CHECK_NEAR(r.max_sigma, q / std::sqrt(12.0), 1e-9 * q);
  CHECK(b.mean_sigma > 10.0 * q);
}

TEST_CASE(reading_noise_breaks_the_chain_at_irregular_samples) {
  const double q = 10e-6;
  ReadingNoise est(kDt, q);
  est.observe(0.0, kDt);
  CHECK(est.samples() == 0);  // a second difference needs two samples
  est.observe(q, kDt);
  CHECK(est.samples() == 1);
  // A sample twice as long: no difference across it, and none with the
  // next either.
  est.observe(100 * q, 2.0 * kDt);
  CHECK(est.samples() == 1);
  est.observe(q, kDt);
  CHECK(est.samples() == 1);
  est.observe(q, kDt);
  CHECK(est.samples() == 2);
  // An interruption, such as a cut force, likewise.
  est.interrupt();
  est.observe(-200 * q, kDt);
  CHECK(est.samples() == 2);
  CHECK_NEAR(est.variance(), q * q / 12.0, 1e-24);
}

TEST_CASE(reading_noise_clips_an_impact) {
  // A jump of velocity the model did not expect, as at an impact: a
  // hundred counts in one sample on a quantized encoder moves the estimate
  // by little, and it decays back to the quantization.
  const double q = 10e-6;
  ReadingNoise est(kDt, q);
  for (int k = 0; k < 1000; ++k) est.observe(0.0, kDt);
  est.observe(100 * q, kDt);
  est.observe(100 * q, kDt);
  CHECK(est.variance() < 4.0 * q * q / 12.0);
  for (int k = 0; k < 3000; ++k) est.observe(100 * q, kDt);
  CHECK_NEAR(est.variance(), q * q / 12.0, 1e-3 * q * q);
}

TEST_CASE(reading_noise_rejects_bad_parameters) {
  bool threw = false;
  try {
    ReadingNoise bad(kDt, 0.0);
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  CHECK(threw);
}
