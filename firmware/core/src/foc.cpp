#include "tuushin/fw/foc.hpp"

#include <cmath>

namespace tuushin::fw {

float wrap_two_pi(float a) {
  a = std::fmod(a, kTwoPi);
  return a < 0.0f ? a + kTwoPi : a;
}

float wrap_pi(float a) {
  return wrap_two_pi(a + kPi) - kPi;
}

BridgeVoltages space_vector(float ud, float uq, float theta, float vbus) {
  const float c = std::cos(theta), s = std::sin(theta);
  const float ua = ud * c - uq * s;
  const float ub = ud * s + uq * c;
  // Inverse Clarke.
  float a = ua;
  float b = -0.5f * ua + 0.5f * kSqrt3 * ub;
  float cc = -0.5f * ua - 0.5f * kSqrt3 * ub;
  // Min-max zero sequence: centre the three between their extremes, which
  // is what space-vector modulation does with the null vectors.
  const float hi = std::fmax(a, std::fmax(b, cc));
  const float lo = std::fmin(a, std::fmin(b, cc));
  const float shift = 0.5f * vbus - 0.5f * (hi + lo);
  auto clamp = [vbus](float v) { return v < 0.0f ? 0.0f : (v > vbus ? vbus : v); };
  return {clamp(a + shift), clamp(b + shift), clamp(cc + shift)};
}

float MotorParams::kt() const {
  if (torque_constant > 0.0f) return torque_constant;
  if (kv <= 0.0f) return 0.0f;
  // Line-to-line peak back-EMF sqrt(3) lambda p omega = omega / (kv 2 pi / 60).
  return 1.5f / (kSqrt3 * kv * kTwoPi / 60.0f);
}

float MotorParams::back_emf_constant() const {
  return kt() / 1.5f;
}

float q_voltage(const MotorParams& m, float torque, float omega, float voltage_limit,
                float* torque_out) {
  const float kt = m.kt();
  if (kt <= 0.0f || m.phase_resistance <= 0.0f) {
    if (torque_out) *torque_out = 0.0f;
    return 0.0f;
  }
  const float emf = m.back_emf ? m.back_emf_constant() * omega : 0.0f;
  float uq = m.phase_resistance * torque / kt + emf;
  if (uq > voltage_limit) uq = voltage_limit;
  if (uq < -voltage_limit) uq = -voltage_limit;
  if (torque_out) *torque_out = kt * (uq - emf) / m.phase_resistance;
  return uq;
}

float VelocityFilter::update(float dtheta, float dt) {
  if (dt <= 0.0f) return v_;
  const float raw = dtheta / dt;
  // Exact discretization of a first-order lag over dt.
  const float a = 1.0f - std::exp(-kTwoPi * cutoff_ * dt);
  v_ += a * (raw - v_);
  return v_;
}

float VelocityFilter::spread(float reading_sd, float dt) const {
  if (dt <= 0.0f) return 0.0f;
  const float a = 1.0f - std::exp(-kTwoPi * cutoff_ * dt);
  return reading_sd / dt * a * std::sqrt(2.0f / (2.0f - a));
}

namespace {

// The gains that put the observer's three poles at z = p, for the state
// (theta, omega dt, acceleration dt^2) and the correction x += k (reading
// - prediction): the error's transition (I - k C) A has the
// characteristic polynomial z^3 - (3 - k1 - k2 - k3 / 2) z^2 +
// (3 - 2 k1 - k2 + k3 / 2) z - (1 - k1), which (z - p)^3 matches with:
void observer_gains(float p, float k[3]) {
  const float m = 1.0f - p;
  k[0] = 1.0f - p * p * p;
  k[1] = 1.5f * m * m * (1.0f + p);
  k[2] = m * m * m;
}

}  // namespace

VelocityObserver::VelocityObserver(float bandwidth_hz, float inertia, float nominal_dt)
    : hz_(bandwidth_hz), inertia_(inertia > 0.0f ? inertia : 0.0f) {
  // The estimate's response to one reading's error, summed in squares:
  // the error goes in through k and dies out through (I - k C) A.
  float k[3];
  observer_gains(std::exp(-kTwoPi * hz_ * nominal_dt), k);
  const float a[3][3] = {{1.0f, 1.0f, 0.5f}, {0.0f, 1.0f, 1.0f}, {0.0f, 0.0f, 1.0f}};
  float v[3] = {k[0], k[1], k[2]};
  float sum = 0.0f;
  for (int n = 0; n < 400; ++n) {
    sum += v[1] * v[1];
    float av[3];
    for (int i = 0; i < 3; ++i) av[i] = a[i][0] * v[0] + a[i][1] * v[1] + a[i][2] * v[2];
    v[0] = av[0] - k[0] * av[0];
    v[1] = av[1] - k[1] * av[0];
    v[2] = av[2] - k[2] * av[0];
  }
  noise_gain_ = std::sqrt(sum);
}

void VelocityObserver::update(float dtheta, float dt, float torque) {
  if (dt <= 0.0f) return;
  float k[3];
  observer_gains(std::exp(-kTwoPi * hz_ * dt), k);
  auto step = [&](float& r, float& w, float& acc, float push, float moved) {
    r += w * dt + 0.5f * (acc + push) * dt * dt - moved;
    w += (acc + push) * dt;
    const float e = -r;
    r += k[0] * e;
    w += k[1] * e / dt;
    acc += k[2] * e / (dt * dt);
  };
  step(r_, w_, acc_, 0.0f, dtheta);
  step(rt_, wt_, acct_, inertia_ > 0.0f ? torque / inertia_ : 0.0f, 0.0f);
}

std::int32_t TurnCounter::update(std::uint32_t raw) {
  if (!started_) {
    reset(raw);
    return 0;
  }
  std::int32_t d = static_cast<std::int32_t>(raw) - static_cast<std::int32_t>(last_);
  if (d > cpr_ / 2) d -= cpr_;
  if (d < -cpr_ / 2) d += cpr_;
  last_ = raw;
  count_ += d;
  return d;
}

}  // namespace tuushin::fw
