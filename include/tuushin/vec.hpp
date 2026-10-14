#pragma once

#include <array>
#include <cmath>
#include <cstddef>

namespace tuushin {

// Small fixed-size vector for per-axis haptic quantities (1-DoF and 3-DoF).
template <std::size_t N>
struct Vec {
  std::array<double, N> v{};

  double& operator[](std::size_t i) { return v[i]; }
  double operator[](std::size_t i) const { return v[i]; }

  Vec& operator+=(const Vec& o) {
    for (std::size_t i = 0; i < N; ++i) v[i] += o.v[i];
    return *this;
  }
  Vec& operator-=(const Vec& o) {
    for (std::size_t i = 0; i < N; ++i) v[i] -= o.v[i];
    return *this;
  }
  Vec& operator*=(double s) {
    for (std::size_t i = 0; i < N; ++i) v[i] *= s;
    return *this;
  }

  friend Vec operator+(Vec a, const Vec& b) { return a += b; }
  friend Vec operator-(Vec a, const Vec& b) { return a -= b; }
  friend Vec operator*(Vec a, double s) { return a *= s; }
  friend Vec operator*(double s, Vec a) { return a *= s; }
};

template <std::size_t N>
double dot(const Vec<N>& a, const Vec<N>& b) {
  double s = 0.0;
  for (std::size_t i = 0; i < N; ++i) s += a.v[i] * b.v[i];
  return s;
}

template <std::size_t N>
double norm(const Vec<N>& a) {
  return std::sqrt(dot(a, a));
}

using Vec1 = Vec<1>;
using Vec3 = Vec<3>;

}  // namespace tuushin
