#include "tuushin/fw/plant.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace tuushin::fw {

namespace {

constexpr double kPiD = 3.14159265358979323846;

}  // namespace

Plant::Plant(const PlantParams& p, std::uint64_t seed)
    : p_(p), theta_(p.start_angle), rng_(seed ? seed : 1) {}

void Plant::set_duties(double a, double b, double c) {
  auto clamp = [](double d) { return d < 0.0 ? 0.0 : (d > 1.0 ? 1.0 : d); };
  pa_ = clamp(a) * p_.supply;
  pb_ = clamp(b) * p_.supply;
  pc_ = clamp(c) * p_.supply;
  if (p_.pwm_period > 0.0) {
    // A write a hair before a period's start, by the clock's rounding,
    // waits for the next one, as a write at it does.
    pending_at_ = (std::floor(t_ / p_.pwm_period + 1e-6) + 1.0) * p_.pwm_period;
    pending_ = true;
  } else {
    va_ = pa_;
    vb_ = pb_;
    vc_ = pc_;
  }
}

void Plant::enable(bool on) {
  if (p_.phase_inductance > 0.0) {
    if (enabled_ && !on) {
      off_t_ = t_;
      off_s_ = std::hypot(ia_, ib_);
      off_a_ = off_s_ > 0.0 ? ia_ / off_s_ : 0.0;
      off_b_ = off_s_ > 0.0 ? ib_ / off_s_ : 0.0;
    } else if (!enabled_ && on) {
      off_currents(t_, &ia_, &ib_);
    }
  }
  enabled_ = on;
}

void Plant::off_currents(double t, double* ia, double* ib) const {
  // L ds/dt = -(V / sqrt 3 + R s) for the magnitude s, until it is zero.
  const double r = p_.phase_resistance, tau = p_.phase_inductance / r;
  const double c = std::max(p_.supply / std::sqrt(3.0) / r, 1e-9);
  const double s = std::max(0.0, (off_s_ + c) * std::exp(-(t - off_t_) / tau) - c);
  *ia = s * off_a_;
  *ib = s * off_b_;
}

double Plant::off_end() const {
  const double r = p_.phase_resistance, tau = p_.phase_inductance / r;
  const double c = std::max(p_.supply / std::sqrt(3.0) / r, 1e-9);
  return off_t_ + tau * std::log((off_s_ + c) / c);
}

void Plant::set_hand(const HandParams& h, double t0) {
  hand_ = h;
  hand_t0_ = t0;
}

template <class F>
void Plant::voltage_ab(F* alpha, F* beta) const {
  const F va = F(va_), vb = F(vb_), vc = F(vc_);
  const F mean = (va + vb + vc) / F(3);
  const F a = va - mean, b = vb - mean, c = vc - mean;
  *alpha = a;
  *beta = (b - c) / std::sqrt(F(3));
}

template <class F>
F Plant::q_current_at(double theta, double omega, double ia, double ib, double* dia,
                      double* dib) const {
  const bool rl = p_.phase_inductance > 0.0;
  if (!rl && !enabled_) return F(0);
  const F dir = p_.direction >= 0 ? F(1) : F(-1);
  const F te = dir * F(p_.pole_pairs) * F(theta) - F(p_.electrical_offset);
  const F sin_te = std::sin(te), cos_te = std::cos(te);
  const F iq = -F(ia) * sin_te + F(ib) * cos_te;
  if (rl && !dia) return iq;
  F alpha, beta;
  voltage_ab(&alpha, &beta);
  // The back-EMF lies along q: (-sin, cos) of the electrical angle.
  const F emf = F(p_.kt) / F(1.5) * dir * F(omega);
  const F r = F(p_.phase_resistance);
  if (!rl) return (-alpha * sin_te + beta * cos_te - emf) / r;
  const F l = F(p_.phase_inductance);
  *dia = static_cast<double>((alpha - r * F(ia) + emf * sin_te) / l);
  *dib = static_cast<double>((beta - r * F(ib) - emf * cos_te) / l);
  return iq;
}

template <class F>
F Plant::hand_at(double t, double theta, double omega) const {
  if (!hand_.present) return F(0);
  const F w = F(2.0 * kPiD * hand_.ref_freq);
  const F s = F(t - hand_t0_);
  const F amp = F(hand_.ref_amplitude);
  const F ref = F(hand_.ref_offset) + amp * F(0.5) * (F(1) - std::cos(w * s));
  const F vref = amp * F(0.5) * w * std::sin(w * s);
  return F(hand_.k) * (ref - F(theta)) + F(hand_.b) * (vref - F(omega));
}

double Plant::q_current() const {
  return q_current_at<double>(theta_, omega_, ia_, ib_);
}

double Plant::motor_torque() const {
  return (p_.direction >= 0 ? 1.0 : -1.0) * p_.kt * q_current();
}

DeviceConfig device_config_for(const PlantParams& p, const char* name) {
  DeviceConfig c;
  c.motor.pole_pairs = p.pole_pairs;
  c.motor.phase_resistance = static_cast<float>(p.phase_resistance);
  c.motor.torque_constant = static_cast<float>(p.kt);
  c.counts_per_rev = p.counts_per_rev;
  c.max_torque = 0.1f;
  c.inertia = static_cast<float>(p.inertia);
  c.supply_min = static_cast<float>(0.8 * p.supply);
  c.supply_max = static_cast<float>(1.2 * p.supply);
  if (p.encoder_noise > 0.0) {
    c.reading_noise_lo = static_cast<float>(p.encoder_noise / 1.1);
    c.reading_noise_hi = static_cast<float>(p.encoder_noise * 1.1);
  }
  if (p.phase_inductance > 0.0) {
    const double lag = p.phase_inductance / p.phase_resistance;
    c.current_lag_lo = static_cast<float>(lag / 1.2);
    c.current_lag_hi = static_cast<float>(lag * 1.2);
  }
  c.duty_delay = static_cast<float>(p.pwm_period);
  c.kind = device::DeviceKind::kSimulated;
  std::snprintf(c.name, sizeof c.name, "%s", name);
  return c;
}

template <class F>
F Plant::stop_at(double theta, double omega) const {
  if (!stop_.present || theta <= stop_.angle) return F(0);
  // The penetration in double: the stop's angle and the knob's are close.
  return std::min(F(0), -F(stop_.k) * F(theta - stop_.angle) - F(stop_.b) * F(omega));
}

double Plant::hand_torque() const {
  return hand_at<double>(t_, theta_, omega_);
}

double Plant::stop_torque() const {
  return stop_at<double>(theta_, omega_);
}

void Plant::advance(double t) {
  for (;;) {
    if (pending_ && pending_at_ <= t_) {
      va_ = pa_;
      vb_ = pb_;
      vc_ = pc_;
      pending_ = false;
    }
    if (t_ >= t) return;
    const double until = pending_ ? std::min(t, pending_at_) : t;
    if (p_.float_forces) integrate<float>(until);
    else integrate<double>(until);
  }
}

template <class F>
void Plant::integrate(double t) {
  const F inertia = F(p_.inertia + (hand_.present ? hand_.inertia : 0.0));
  const F dir = p_.direction >= 0 ? F(1) : F(-1);
  const bool rl = p_.phase_inductance > 0.0;
  const bool decaying = rl && !enabled_;
  // When the currents are gone, with the driver off (s).
  const double end = decaying ? off_end() : 0.0;
  struct Rate {
    double w, a, ia, ib, motor;
  };
  auto rate = [&](double ti, double th, double om, double ia, double ib) {
    Rate r{om, 0.0, 0.0, 0.0, 0.0};
    F m = F(0);
    if (!decaying) {
      m = dir * F(p_.kt) * q_current_at<F>(th, om, ia, ib, rl ? &r.ia : nullptr, &r.ib);
    } else if (ti < end) {
      off_currents(ti, &ia, &ib);
      m = dir * F(p_.kt) * q_current_at<F>(th, om, ia, ib);
    }
    r.motor = static_cast<double>(m);
    const F cog = -F(p_.cogging) * std::sin(F(p_.cogging_per_rev) * F(th));
    const F fric = -F(p_.viscous) * F(om) - F(p_.coulomb) * std::tanh(F(om) / F(0.02));
    const F sum = m + cog + fric + hand_at<F>(ti, th, om) + stop_at<F>(th, om) + F(load_);
    r.a = static_cast<double>(sum / inertia);
    return r;
  };
  // The currents settle over L / R: half of it keeps RK4 close. Once they
  // are gone, with the driver off, the step is the mechanics' again.
  const double settle = rl ? 0.5 * p_.phase_inductance / p_.phase_resistance : p_.step;
  while (t_ < t) {
    const bool live = rl && (!decaying || t_ < end);
    double h = std::min(live ? std::min(p_.step, settle) : p_.step, t - t_);
    if (decaying && end > t_ && end < t_ + h) h = end - t_;
    const Rate k1 = rate(t_, theta_, omega_, ia_, ib_);
    const Rate k2 = rate(t_ + h / 2, theta_ + h / 2 * k1.w, omega_ + h / 2 * k1.a,
                         ia_ + h / 2 * k1.ia, ib_ + h / 2 * k1.ib);
    const Rate k3 = rate(t_ + h / 2, theta_ + h / 2 * k2.w, omega_ + h / 2 * k2.a,
                         ia_ + h / 2 * k2.ia, ib_ + h / 2 * k2.ib);
    const Rate k4 =
        rate(t_ + h, theta_ + h * k3.w, omega_ + h * k3.a, ia_ + h * k3.ia, ib_ + h * k3.ib);
    work_ += h / 6.0 *
             (k1.motor * k1.w + 2 * k2.motor * k2.w + 2 * k3.motor * k3.w + k4.motor * k4.w);
    theta_ += h / 6.0 * (k1.w + 2 * k2.w + 2 * k3.w + k4.w);
    omega_ += h / 6.0 * (k1.a + 2 * k2.a + 2 * k3.a + k4.a);
    ia_ += h / 6.0 * (k1.ia + 2 * k2.ia + 2 * k3.ia + k4.ia);
    ib_ += h / 6.0 * (k1.ib + 2 * k2.ib + 2 * k3.ib + k4.ib);
    t_ = h == t - t_ ? t : t_ + h;
    if (decaying) off_currents(t_, &ia_, &ib_);
  }
}

std::uint32_t Plant::encoder_raw() {
  // A Gaussian from xorshift64* by Box-Muller.
  auto uniform = [this] {
    rng_ ^= rng_ >> 12;
    rng_ ^= rng_ << 25;
    rng_ ^= rng_ >> 27;
    return static_cast<double>((rng_ * 2685821657736338717ull) >> 11) * 0x1.0p-53;
  };
  double g;
  if (have_spare_) {
    g = spare_;
    have_spare_ = false;
  } else {
    const double u = std::max(uniform(), 1e-300), v = uniform();
    const double r = std::sqrt(-2.0 * std::log(u));
    g = r * std::cos(2 * kPiD * v);
    spare_ = r * std::sin(2 * kPiD * v);
    have_spare_ = true;
  }
  const double cpr = static_cast<double>(p_.counts_per_rev);
  const double counts = std::floor(theta_ / (2 * kPiD) * cpr + p_.encoder_noise * g);
  const double wrapped = counts - cpr * std::floor(counts / cpr);
  return static_cast<std::uint32_t>(wrapped) % p_.counts_per_rev;
}

std::uint32_t SimHal::micros() {
  return static_cast<std::uint32_t>(static_cast<std::uint64_t>(std::llround(now_ * rate_ * 1e6)));
}

bool SimHal::read_encoder(std::uint32_t* raw) {
  plant_.advance(now_);
  if (encoder_broken_) return false;
  *raw = last_raw_ = plant_.encoder_raw();
  return true;
}

bool SimHal::reread_encoder(std::uint32_t* raw) {
  if (encoder_broken_ || reread_ == Reread::kNone) return false;
  *raw = reread_ == Reread::kSame ? last_raw_ : plant_.encoder_raw();
  return true;
}

void SimHal::set_duties(float a, float b, float c) {
  plant_.advance(now_);
  plant_.set_duties(a, b, c);
}

void SimHal::enable_driver(bool on) {
  plant_.advance(now_);
  plant_.enable(on);
  driver_ = on;
}

std::size_t SimHal::read_bytes(std::uint8_t* buf, std::size_t cap) {
  std::size_t n = 0;
  while (n < cap && !to_device_.empty() && to_device_.front().first <= now_) {
    buf[n++] = to_device_.front().second;
    to_device_.pop_front();
  }
  return n;
}

std::size_t SimHal::write_bytes(const std::uint8_t* buf, std::size_t n) {
  for (std::size_t i = 0; i < n; ++i) to_host_.emplace_back(now_ + latency_to_host_, buf[i]);
  return n;
}

bool SimHal::load_calibration(Calibration* c) {
  *c = stored;
  return stored.valid;
}

void SimHal::store_calibration(const Calibration& c) {
  stored = c;
}

void SimHal::host_write(const std::uint8_t* buf, std::size_t n) {
  for (std::size_t i = 0; i < n; ++i) to_device_.emplace_back(now_ + latency_, buf[i]);
}

std::size_t SimHal::host_read(std::uint8_t* buf, std::size_t cap) {
  std::size_t n = 0;
  while (n < cap && !to_host_.empty() && to_host_.front().first <= now_) {
    buf[n++] = to_host_.front().second;
    to_host_.pop_front();
  }
  return n;
}

}  // namespace tuushin::fw
