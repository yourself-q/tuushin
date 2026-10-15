#include "tuushin/fw/plant.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace tuushin::fw {

namespace {

constexpr double kPiD = 3.14159265358979323846;
constexpr double kStep = 20e-6;  // s, the longest integration step

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

double Plant::torque_at(double theta, double omega) const {
  if (!enabled_) return 0.0;
  const double mean = (va_ + vb_ + vc_) / 3.0;
  const double a = va_ - mean, b = vb_ - mean, c = vc_ - mean;
  const double alpha = a;
  const double beta = (b - c) / std::sqrt(3.0);
  const double dir = p_.direction >= 0 ? 1.0 : -1.0;
  const double te = dir * p_.pole_pairs * theta - p_.electrical_offset;
  const double vq = -alpha * std::sin(te) + beta * std::cos(te);
  const double emf = p_.kt / 1.5 * dir * omega;
  const double iq = (vq - emf) / p_.phase_resistance;
  return dir * p_.kt * iq;
}

double Plant::hand_at(double t, double theta, double omega) const {
  if (!hand_.present) return 0.0;
  const double w = 2.0 * kPiD * hand_.ref_freq;
  const double s = t - hand_t0_;
  const double ref = hand_.ref_offset + hand_.ref_amplitude * 0.5 * (1.0 - std::cos(w * s));
  const double vref = hand_.ref_amplitude * 0.5 * w * std::sin(w * s);
  return hand_.k * (ref - theta) + hand_.b * (vref - omega);
}

double Plant::motor_torque() const {
  return torque_at(theta_, omega_);
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

double Plant::stop_at(double theta, double omega) const {
  if (!stop_.present || theta <= stop_.angle) return 0.0;
  return std::min(0.0, -stop_.k * (theta - stop_.angle) - stop_.b * omega);
}

double Plant::hand_torque() const {
  return hand_at(t_, theta_, omega_);
}

void Plant::advance(double t) {
  const double inertia = p_.inertia + (hand_.present ? hand_.inertia : 0.0);
  auto accel = [&](double ti, double th, double om, double* motor) {
    const double m = torque_at(th, om);
    *motor = m;
    const double cog = -p_.cogging * std::sin(p_.cogging_per_rev * th);
    const double fric = -p_.viscous * om - p_.coulomb * std::tanh(om / 0.02);
    return (m + cog + fric + hand_at(ti, th, om) + stop_at(th, om) + load_) / inertia;
  };
  while (t_ < t) {
    const double h = std::min(kStep, t - t_);
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
  return static_cast<std::uint32_t>(static_cast<std::uint64_t>(std::llround(now_ * 1e6)));
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
  for (std::size_t i = 0; i < n; ++i) to_host_.emplace_back(now_ + latency_, buf[i]);
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
