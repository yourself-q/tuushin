#include "tuushin/control/tdpa.hpp"

namespace tuushin::control {

namespace {

// Shared PO/PC logic. `power` is the would-be power into the channel. Returns
// the power actually allowed: unchanged when the budget covers the sample,
// otherwise exactly the amount that drains the remaining budget.
double limit_power(double budget, double power, double dt) {
  if (power >= 0.0) return power;
  if (budget < 0.0) budget = 0.0;
  if (-power * dt <= budget) return power;
  return -budget / dt;
}

}  // namespace

PcResult limit_force(const EnergyPort& port, double f, double v_est, double dt) {
  return limit_force(port.budget(), f, v_est, dt);
}

PcResult limit_force(double budget, double f, double v_est, double dt) {
  const double power = f * v_est;
  const double allowed = limit_power(budget, power, dt);
  PcResult r;
  if (allowed == power) {
    r.value = f;
  } else {
    // power < 0 here, so v_est != 0. f' = allowed / v is the same as
    // f + alpha * v with alpha = (allowed - power) / v^2 > 0.
    r.value = allowed / v_est;
    r.gain = (allowed - power) / (v_est * v_est);
    r.active = true;
  }
  return r;
}

PcResult passivate_force(EnergyPort& port, double f, double v, double dt) {
  const PcResult r = limit_force(port, f, v, dt);
  port.account(r.value * v, dt);
  return r;
}

PcResult passivate_velocity(EnergyPort& port, double v, double f, double dt) {
  const double power = -f * v;
  const double allowed = limit_power(port.budget(), power, dt);
  PcResult r;
  if (allowed == power) {
    r.value = v;
  } else {
    // power < 0 here, so f != 0. v' = -allowed / f is the same as
    // v - beta * f with beta = (allowed - power) / f^2 > 0.
    r.value = -allowed / f;
    r.gain = (allowed - power) / (f * f);
    r.active = true;
  }
  port.account(allowed, dt);
  return r;
}

}  // namespace tuushin::control
