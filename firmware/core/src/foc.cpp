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
