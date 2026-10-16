#include "tuushin/teleop/device.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace tuushin::teleop {

namespace {

constexpr double kPi = 3.14159265358979323846;

// Motor side (x, v) and distal side (x2, v2); the distal side is unused on
// a rigid device.
struct State {
  double x, v, x2, v2;
  State plus(const State& d, double s) const {
    return {x + s * d.x, v + s * d.v, x2 + s * d.x2, v2 + s * d.v2};
  }
};

}  // namespace

double FrictionParams::force(double v) const {
  if (coulomb <= 0.0 && stiction <= 0.0) return 0.0;
  double level = coulomb;
  if (stiction > coulomb && stribeck_velocity > 0.0) {
    const double s = v / stribeck_velocity;
    level += (stiction - coulomb) * std::exp(-s * s);
  }
  return level * std::tanh(v / std::max(smoothing_velocity, 1e-9));
}

SimulatedDevice::SimulatedDevice(double mass, double damping, Nanos nominal_dt, int substeps,
                                 Nanos start, double position)
    : mass_(mass),
      damping_(damping),
      nominal_dt_(nominal_dt > 0 ? nominal_dt : kNanosPerMilli),
      substeps_(substeps > 0 ? substeps : 1),
      start_(start),
      t_(start),
      x_(position),
      command_time_(start) {}

double SimulatedDevice::read_position(Nanos now) {
  if (now > t_) {
    if (watchdog_ > 0 && commanded_ != 0.0 && cut_after_ == 0 &&
        now > command_time_ + watchdog_) {
      advance(std::max(t_, command_time_ + watchdog_));
      f_ = 0.0;
      pending_.clear();
      cut_after_ = watchdog_;
    }
    advance(now);
  }
  if (q_ <= 0.0) return x_;
  if (have_read_ && t_ <= y_time_) return y_;
  const double dither = noise_counts_ > 0.0 ? noise_counts_ * noise_.normal() : 0.0;
  const double y = std::round(x_ / q_ + dither) * q_;
  if (have_read_) v_read_ = (y - y_) / to_seconds(t_ - y_time_);
  have_read_ = true;
  y_ = y;
  y_time_ = t_;
  return y_;
}

void SimulatedDevice::command_force(double f) {
  if (delay_ > 0) {
    pending_.emplace_back(t_ + delay_, f);
  } else {
    f_ = f;
  }
  command_time_ = t_;
  commanded_ = f;
  cut_after_ = 0;
}

void SimulatedDevice::set_compliance(const ComplianceParams& compliance) {
  compliance_ = compliance;
  compliant_ = compliance.distal_mass > 0.0 && compliance.distal_mass < mass_;
  m2_ = compliant_ ? compliance.distal_mass : 0.0;
  m1_ = mass_ - m2_;
  x2_ = x_;
  v2_ = v_;
}

void SimulatedDevice::set_encoder_resolution(double q) {
  q_ = q > 0.0 ? q : 0.0;
  have_read_ = false;
  v_read_ = 0.0;
}

void SimulatedDevice::advance(Nanos now) {
  while (!pending_.empty() && pending_.front().first <= now) {
    integrate(pending_.front().first);
    f_ = pending_.front().second;
    pending_.pop_front();
  }
  integrate(now);
}

void SimulatedDevice::integrate(Nanos now) {
  if (now <= t_) return;
  const Nanos elapsed = now - t_;
  const std::int64_t n = std::max<std::int64_t>(
      1, (elapsed * substeps_ + nominal_dt_ - 1) / nominal_dt_);
  const double h = to_seconds(elapsed) / static_cast<double>(n);
  const double t0 = to_seconds(t_ - start_);
  auto deriv = [&](double ti, const State& p) {
    const double own = f_ - damping_ * p.v - friction_.force(p.v);
    if (!compliant_) {
      return State{p.v, (external_force(ti, p.x, p.v) + own) / mass_, 0.0, 0.0};
    }
    const double link = compliance_.stiffness * (p.x2 - p.x) + compliance_.damping * (p.v2 - p.v);
    return State{p.v, (own + link) / m1_, p.v2, (external_force(ti, p.x2, p.v2) - link) / m2_};
  };
  State p{x_, v_, x2_, v2_};
  for (std::int64_t i = 0; i < n; ++i) {
    const double ti = t0 + static_cast<double>(i) * h;
    const State k1 = deriv(ti, p);
    const State k2 = deriv(ti + 0.5 * h, p.plus(k1, 0.5 * h));
    const State k3 = deriv(ti + 0.5 * h, p.plus(k2, 0.5 * h));
    const State k4 = deriv(ti + h, p.plus(k3, h));
    p = p.plus(k1, h / 6.0).plus(k2, h / 3.0).plus(k3, h / 3.0).plus(k4, h / 6.0);
  }
  work_ += f_ * (p.x - x_);
  x_ = p.x;
  v_ = p.v;
  x2_ = p.x2;
  v2_ = p.v2;
  t_ = now;
}

bool SimulatedDevice::plausible() const {
  auto ok = [](double x, double v) {
    return std::isfinite(x) && std::isfinite(v) && std::fabs(x) < 1.0 && std::fabs(v) < 10.0;
  };
  return ok(x_, v_) && ok(distal_position(), distal_velocity());
}

SimulatedMaster::SimulatedMaster(double mass, double damping, const OperatorParams& op,
                                 Nanos nominal_dt, int substeps, Nanos start)
    : SimulatedDevice(mass, damping, nominal_dt, substeps, start, op.ref_offset), op_(op) {}

double SimulatedMaster::external_force(double t, double x, double v) const {
  const double w = 2.0 * kPi * op_.ref_freq;
  const double xr = op_.ref_offset + op_.ref_amplitude * 0.5 * (1.0 - std::cos(w * t));
  const double vr = op_.ref_amplitude * 0.5 * w * std::sin(w * t);
  return op_.hand_k * (xr - x) + op_.hand_b * (vr - v);
}

double SimulatedMaster::hand_force() const {
  return external_force(to_seconds(time() - start()), distal_position(), distal_velocity());
}

SimulatedSlave::SimulatedSlave(double mass, double damping, const WallParams& wall,
                               Nanos nominal_dt, int substeps, Nanos start, double position)
    : SimulatedDevice(mass, damping, nominal_dt, substeps, start, position), wall_(wall) {}

double WallParams::position_at(double t) const {
  return position + motion_amplitude * std::sin(2.0 * kPi * motion_freq * t);
}

double WallParams::velocity_at(double t) const {
  return motion_amplitude * 2.0 * kPi * motion_freq * std::cos(2.0 * kPi * motion_freq * t);
}

double WallParams::force(double t, double x, double v) const {
  const double pen = x - position_at(t);
  if (pen <= 0.0) return 0.0;
  const double spring = exponent == 1.0 ? pen : std::pow(pen, exponent);
  return std::max(0.0, k * spring + (b + hc_damping * spring) * (v - velocity_at(t)));
}

double SimulatedSlave::external_force(double t, double x, double v) const {
  return -wall_.force(t, x, v);
}

double SimulatedSlave::contact_force() const {
  return wall_.force(to_seconds(time() - start()), distal_position(), distal_velocity());
}

double SimulatedSlave::wall_position() const {
  return wall_.position_at(to_seconds(time() - start()));
}

}  // namespace tuushin::teleop
