#pragma once

#include <cmath>
#include <cstdint>
#include <random>

namespace tuushin {

// Deterministic random source for simulations.
//
// std::mt19937_64 produces the same sequence on every standard library, but
// std::normal_distribution and friends do not (libc++ and libstdc++ use
// different algorithms). The distributions below are written out explicitly
// so a seed reproduces the same run on macOS, Linux and Windows.
class Rng {
 public:
  explicit Rng(std::uint64_t seed) : gen_(seed) {}

  // Uniform in [0, 1).
  double uniform() {
    // 53 random bits mapped onto the double mantissa.
    return static_cast<double>(gen_() >> 11) * 0x1.0p-53;
  }

  bool bernoulli(double p) { return uniform() < p; }

  // Exponential with the given mean.
  double exponential(double mean) {
    // 1 - u is in (0, 1], so the log is finite.
    return -mean * std::log(1.0 - uniform());
  }

  // Standard normal via Box-Muller; the second value is cached.
  double normal() {
    if (has_spare_) {
      has_spare_ = false;
      return spare_;
    }
    const double u1 = 1.0 - uniform();
    const double u2 = uniform();
    const double r = std::sqrt(-2.0 * std::log(u1));
    const double theta = 2.0 * 3.14159265358979323846 * u2;
    spare_ = r * std::sin(theta);
    has_spare_ = true;
    return r * std::cos(theta);
  }

  double normal(double mean, double stddev) { return mean + stddev * normal(); }

 private:
  std::mt19937_64 gen_;
  bool has_spare_ = false;
  double spare_ = 0.0;
};

}  // namespace tuushin
