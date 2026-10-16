#include "tuushin/control/wall_model.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace tuushin::control {

double WallModel::force(double x, double v) const {
  if (!known() || x <= surface) return 0.0;
  return std::max(0.0, stiffness * (x - surface) + damping * v);
}

double WallEstimator::force_noise() const {
  if (p_.force_noise > 0.0 || diffs_.size() < p_.noise_window / 2) return p_.force_noise;
  std::vector<double> d(diffs_.begin(), diffs_.end());
  auto mid = d.begin() + static_cast<std::ptrdiff_t>(d.size() / 2);
  std::nth_element(d.begin(), mid, d.end());
  return 1.4826 * *mid / std::sqrt(2.0);
}

double WallEstimator::contact_threshold() const {
  return std::max(p_.contact_force, p_.contact_sigmas * force_noise());
}

void WallEstimator::observe(double x, double v, double f) {
  if (p_.force_noise <= 0.0) {
    if (has_last_) {
      diffs_.push_back(std::fabs(f - f_last_));
      while (diffs_.size() > p_.noise_window) diffs_.pop_front();
    }
    f_last_ = f;
    has_last_ = true;
  }

  const double enter = contact_threshold();
  const double leave = 0.5 * enter;
  const bool past = contact_ ? f < leave : f > enter;
  run_ = past ? run_ + 1 : 0;
  if (run_ >= std::max<std::size_t>(1, p_.debounce)) {
    contact_ = !contact_;
    run_ = 0;
    window_.clear();
  }

  if (!contact_) {
    // Free space where the model has the wall: the wall is not there.
    if (model_.known()) {
      const double s = x - leave / model_.stiffness;
      if (s > model_.surface) model_.surface = s;
    }
    return;
  }
  window_.push_back({x, v, f});
  while (window_.size() > p_.window) window_.pop_front();
  fit();
  track_surface();
}

void WallEstimator::fit() {
  // Centred sums, so that a few micrometres of motion 2 cm from the origin
  // do not drown in rounding.
  const double n = static_cast<double>(window_.size());
  if (n < 3.0) return;
  double mx = 0.0, mv = 0.0, mf = 0.0;
  double lo = window_.front().x, hi = lo;
  for (const auto& p : window_) {
    mx += p.x;
    mv += p.v;
    mf += p.f;
    lo = std::min(lo, p.x);
    hi = std::max(hi, p.x);
  }
  mx /= n;
  mv /= n;
  mf /= n;
  if (hi - lo < p_.min_span) return;
  // A stiffness known from earlier samples is replaced only by a full
  // window's: the first samples of a contact are mostly its impact, whose
  // damping a spring takes for a softer or stiffer wall.
  if (model_.known() && window_.size() < p_.window) return;

  double sxx = 0.0, sxv = 0.0, svv = 0.0, sxf = 0.0, svf = 0.0, sff = 0.0;
  for (const auto& p : window_) {
    const double dx = p.x - mx, dv = p.v - mv, df = p.f - mf;
    sxx += dx * dx;
    sxv += dx * dv;
    svv += dv * dv;
    sxf += dx * df;
    svf += dv * df;
    sff += df * df;
  }
  if (!(sxx > 0.0)) return;
  double k = sxf / sxx, b = 0.0, params = 2.0;
  const double det = sxx * svv - sxv * sxv;
  if (p_.estimate_damping && det > 1e-9 * sxx * svv) {
    const double k2 = (sxf * svv - svf * sxv) / det;
    const double b2 = (svf * sxx - sxf * sxv) / det;
    if (b2 > 0.0) {
      k = k2;
      b = b2;
      params = 3.0;
    }
  }
  if (!(k > 0.0) || k > p_.max_stiffness) return;
  // The residuals' variance, and from it the slope's standard error; with
  // damping, that of the stiffness given the velocity's share.
  const double sse = std::max(0.0, sff - k * sxf - b * svf);
  const double s2 = n > params ? sse / (n - params) : 0.0;
  const double var_k = params > 2.0 ? s2 * svv / det : s2 / sxx;
  const double se = std::sqrt(var_k);
  if (se > p_.max_relative_error * k) return;
  if (model_.known() && std::fabs(k - model_.stiffness) <=
                            std::max(p_.stiffness_deadband * model_.stiffness, 2.0 * se)) {
    return;
  }
  model_.stiffness = k;
  model_.damping = b;
}

void WallEstimator::track_surface() {
  if (!model_.known()) return;
  const std::size_t m = std::min(window_.size(), std::max<std::size_t>(1, p_.surface_window));
  double s = 0.0;
  for (auto it = window_.end() - static_cast<std::ptrdiff_t>(m); it != window_.end(); ++it) {
    s += it->x - (it->f - model_.damping * it->v) / model_.stiffness;
  }
  model_.surface = s / static_cast<double>(m);
}

void WallRenderer::update(const WallModel& m, double x) {
  if (!m.known()) return;
  const double spring =
      model_.known() && x > surface_ ? model_.stiffness * (x - surface_) : 0.0;
  if (spring > 0.0) {
    // Inside: the same spring force from the new stiffness. A spring stores
    // f^2 / 2k: a stiffer one releases energy, a softer one would store more
    // than the operator put in, which only the reserve may pay for; beyond
    // it the force drops to what the stored energy gives.
    const double stored = 0.5 * spring * spring / model_.stiffness;
    const double extra = 0.5 * spring * spring / m.stiffness - stored;
    model_ = m;
    if (extra <= reserve_) {
      reserve_ -= extra;
      surface_ = x - spring / m.stiffness;
    } else {
      const double energy = stored + std::max(0.0, reserve_);
      reserve_ = 0.0;
      surface_ = x - std::sqrt(2.0 * energy / m.stiffness);
    }
  } else {
    // Outside, or the first model: the model's surface, but not past the
    // master.
    model_ = m;
    surface_ = std::max(m.surface, x);
  }
}

double WallRenderer::force(double x, double v, double dt) {
  if (!model_.known()) return 0.0;
  const double k = model_.stiffness;
  const double target = model_.surface;
  if (x <= surface_) {
    // Outside: the surface goes back to the model's, following the master
    // where it would have to pass it.
    surface_ = surface_ > target ? std::max(target, x) : target;
  } else if (surface_ < target) {
    // Towards the master: the wall releases what it stored.
    const double p = x - surface_;
    surface_ = std::min(target, surface_ + p_.relax_rate * dt);
    const double q = std::max(0.0, x - surface_);
    reserve_ += 0.5 * k * (p * p - q * q);
  } else if (surface_ > target && reserve_ > 0.0) {
    // Back: as far as the reserve pays for what the wall then stores.
    const double p = x - surface_;
    const double affordable = std::sqrt(p * p + 2.0 * reserve_ / k) - p;
    const double step = std::min({p_.active_relax_rate * dt, surface_ - target, affordable});
    surface_ -= step;
    reserve_ -= 0.5 * k * ((p + step) * (p + step) - p * p);
  }
  if (x <= surface_) return 0.0;
  const double f = model_.stiffness * (x - surface_) + model_.damping * v;
  if (f <= 0.0) return 0.0;
  reserve_ += model_.damping * v * v * dt;
  return f;
}

void WallRenderer::yield(double x, double v, double displayed) {
  if (!model_.known() || x <= surface_) return;
  const double s = x - (std::max(0.0, displayed) - model_.damping * v) / model_.stiffness;
  surface_ = std::min(x, std::max(surface_, s));
}

}  // namespace tuushin::control
