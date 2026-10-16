#include "tuushin/fw/foc.hpp"

#include <cmath>
#include <utility>

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

namespace {

// y going from y0 towards v over s (s) with time constant tau (0: at
// once), adding its integral over s to *area.
float toward(float y0, float v, float s, float tau, float* area) {
  if (s <= 0.0f) return y0;
  if (tau <= 0.0f) {
    *area += v * s;
    return v;
  }
  const float gone = -std::expm1(-s / tau);
  *area += v * s + (y0 - v) * tau * gone;
  return y0 + (v - y0) * gone;
}

}  // namespace

void CurrentLag::set(float torque, bool driving) {
  if (!on()) return;
  u_old_ = u_;
  u_ = driving ? torque : 0.0f;
  driving_ = driving;
  fresh_ = u_ != u_old_ || fresh_;
}

CurrentLag::Span CurrentLag::advance(float dt) {
  if (!on() || dt <= 0.0f) return {ylo_, yhi_};
  // The fastest towards the torque with the bridge open is at once.
  const float fast = driving_ ? lo_ : 0.0f;
  auto end = [&](float y, float sign) {
    // sign +1 for the upper end, which goes up as fast as it can and down
    // as slowly; -1 for the lower.
    float area = 0.0f, s = dt;
    auto tau = [&](float from, float v) { return sign * (v - from) >= 0.0f ? fast : hi_; };
    if (fresh_ && driving_ && sign * (u_old_ - u_) > 0.0f) {
      // A tick shorter than the delay has the rest of it go uncounted: the
      // ticks are some ten times longer.
      const float d = std::fmin(delay_, dt);
      y = toward(y, u_old_, d, tau(y, u_old_), &area);
      s -= d;
    }
    y = toward(y, u_, s, tau(y, u_), &area);
    return std::make_pair(y, area / dt);
  };
  const auto hi = end(yhi_, 1.0f), lo = end(ylo_, -1.0f);
  yhi_ = hi.first;
  ylo_ = lo.first;
  fresh_ = false;
  // Settled to the float's resolution: the books then take the torque set.
  const float tiny = 1e-6f * std::fmax(std::fabs(u_), 1e-3f);
  if (std::fabs(yhi_ - u_) <= tiny && std::fabs(ylo_ - u_) <= tiny) ylo_ = yhi_ = u_;
  return {lo.second, hi.second};
}

CurrentLag::Span CurrentLag::over(float torque, bool driving, float s) const {
  if (s <= 0.0f) return now();
  const float to = driving ? torque : 0.0f;
  // An end's average with the torque set taking effect at the start, or
  // not within s. A switch between does no more: taking it later changes
  // the average by the rate under the old torque less that under the new,
  // where the end then is, which keeps its sign along the way or turns
  // from the new's favour to the old's (an open bridge's current gone at
  // once towards 0, until the end passes 0), and never back from where
  // advance() leaves the ends; so the most is at a switch at either end.
  auto mean = [&](float y, float sign, float v, bool drv) {
    const float fast = drv ? lo_ : 0.0f;
    float area = 0.0f;
    toward(y, v, s, sign * (v - y) >= 0.0f ? fast : hi_, &area);
    return area / s;
  };
  return {std::fmin(mean(ylo_, -1.0f, u_, driving_), mean(ylo_, -1.0f, to, driving)),
          std::fmax(mean(yhi_, 1.0f, u_, driving_), mean(yhi_, 1.0f, to, driving))};
}

float CurrentLag::ahead(float sign, float torque, float hold) const {
  if (!on()) return torque * hold;
  float y = sign > 0.0f ? yhi_ : ylo_, area = 0.0f, s = hold;
  auto tau = [&](float from, float v) { return sign * (v - from) >= 0.0f ? lo_ : hi_; };
  if (sign * (u_ - torque) > 0.0f) {
    const float d = std::fmin(delay_, hold);
    y = toward(y, u_, d, tau(y, u_), &area);
    s -= d;
  }
  y = toward(y, torque, s, tau(y, torque), &area);
  if (sign * torque > 0.0f) y = toward(y, torque, delay_, tau(y, torque), &area);
  return area + y * tau(y, 0.0f);
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
