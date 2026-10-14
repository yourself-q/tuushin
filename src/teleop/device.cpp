#include "tuushin/teleop/device.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace tuushin::teleop {

namespace {

constexpr double kPi = 3.14159265358979323846;

struct State {
  double x, v;
  State plus(const State& d, double s) const { return {x + s * d.x, v + s * d.v}; }
};

}  // namespace

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
  if (now <= t_) return x_;
  if (watchdog_ > 0 && f_ != 0.0 && now > command_time_ + watchdog_) {
    advance(std::max(t_, command_time_ + watchdog_));
    f_ = 0.0;
  }
  advance(now);
  return x_;
}

void SimulatedDevice::advance(Nanos now) {
  if (now <= t_) return;
  const Nanos elapsed = now - t_;
  const std::int64_t n = std::max<std::int64_t>(
      1, (elapsed * substeps_ + nominal_dt_ - 1) / nominal_dt_);
  const double h = to_seconds(elapsed) / static_cast<double>(n);
  const double t0 = to_seconds(t_ - start_);
  auto deriv = [&](double ti, const State& p) {
    return State{p.v, (external_force(ti, p.x, p.v) + f_ - damping_ * p.v) / mass_};
  };
  State p{x_, v_};
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
  t_ = now;
}

bool SimulatedDevice::plausible() const {
  return std::isfinite(x_) && std::isfinite(v_) && std::fabs(x_) < 1.0 && std::fabs(v_) < 10.0;
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
  return external_force(to_seconds(time() - start()), position(), velocity());
}

SimulatedSlave::SimulatedSlave(double mass, double damping, const WallParams& wall,
                               Nanos nominal_dt, int substeps, Nanos start, double position)
    : SimulatedDevice(mass, damping, nominal_dt, substeps, start, position), wall_(wall) {}

namespace {

double wall_force(const WallParams& w, double x, double v) {
  const double pen = x - w.position;
  return pen > 0.0 ? std::max(0.0, w.k * pen + w.b * v) : 0.0;
}

}  // namespace

double SimulatedSlave::external_force(double, double x, double v) const {
  return -wall_force(wall_, x, v);
}

double SimulatedSlave::contact_force() const { return wall_force(wall_, position(), velocity()); }

}  // namespace tuushin::teleop
