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
  va_ = clamp(a) * p_.supply;
  vb_ = clamp(b) * p_.supply;
  vc_ = clamp(c) * p_.supply;
}

void Plant::set_hand(const HandParams& h, double t0) {
  hand_ = h;
  hand_t0_ = t0;
}

template <class F>
F Plant::torque_at(double theta, double omega) const {
  if (!enabled_) return F(0);
  const F va = F(va_), vb = F(vb_), vc = F(vc_);
  const F mean = (va + vb + vc) / F(3);
  const F a = va - mean, b = vb - mean, c = vc - mean;
  const F alpha = a;
  const F beta = (b - c) / std::sqrt(F(3));
  const F dir = p_.direction >= 0 ? F(1) : F(-1);
  const F te = dir * F(p_.pole_pairs) * F(theta) - F(p_.electrical_offset);
  const F vq = -alpha * std::sin(te) + beta * std::cos(te);
  const F emf = F(p_.kt) / F(1.5) * dir * F(omega);
  const F iq = (vq - emf) / F(p_.phase_resistance);
  return dir * F(p_.kt) * iq;
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

double Plant::motor_torque() const {
  return torque_at<double>(theta_, omega_);
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
  if (p_.float_forces) integrate<float>(t);
  else integrate<double>(t);
}

template <class F>
void Plant::integrate(double t) {
  const F inertia = F(p_.inertia + (hand_.present ? hand_.inertia : 0.0));
  auto accel = [&](double ti, double th, double om, double* motor) {
    const F m = torque_at<F>(th, om);
    *motor = static_cast<double>(m);
    const F cog = -F(p_.cogging) * std::sin(F(p_.cogging_per_rev) * F(th));
    const F fric = -F(p_.viscous) * F(om) - F(p_.coulomb) * std::tanh(F(om) / F(0.02));
    const F sum = m + cog + fric + hand_at<F>(ti, th, om) + stop_at<F>(th, om) + F(load_);
    return static_cast<double>(sum / inertia);
  };
  while (t_ < t) {
    const double h = std::min(p_.step, t - t_);
    double m1, m2, m3, m4;
    const double a1 = accel(t_, theta_, omega_, &m1);
    const double w1 = omega_;
    const double a2 = accel(t_ + h / 2, theta_ + h / 2 * w1, omega_ + h / 2 * a1, &m2);
    const double w2 = omega_ + h / 2 * a1;
    const double a3 = accel(t_ + h / 2, theta_ + h / 2 * w2, omega_ + h / 2 * a2, &m3);
    const double w3 = omega_ + h / 2 * a2;
    const double a4 = accel(t_ + h, theta_ + h * w3, omega_ + h * a3, &m4);
    const double w4 = omega_ + h * a3;
    work_ += h / 6.0 * (m1 * w1 + 2 * m2 * w2 + 2 * m3 * w3 + m4 * w4);
    theta_ += h / 6.0 * (w1 + 2 * w2 + 2 * w3 + w4);
    omega_ += h / 6.0 * (a1 + 2 * a2 + 2 * a3 + a4);
    t_ += h;
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
  *raw = plant_.encoder_raw();
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
