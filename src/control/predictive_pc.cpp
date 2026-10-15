#include "tuushin/control/predictive_pc.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace tuushin::control {

namespace {

// A new filter's uncertainty about the velocity (m/s) and H (N): a device
// that may be moving at a centimetre per second and pushed by a few newtons.
constexpr double kInitialVelocityVar = 1e-4;
constexpr double kInitialForceVar = 25.0;
// After a sample the model does not describe: a hand that may have moved on
// at up to a decimetre per second and changed its push by ten newtons.
constexpr double kLostVelocityVar = 1e-2;
constexpr double kLostForceVar = 100.0;
// Adapting the random walk: how fast its log-variance follows the
// normalized innovations, per second; the most a single innovation counts
// for (three standard deviations); and the largest factor on its variance,
// 16^2 (16 times the rate).
constexpr double kAdaptPerSecond = 3.0;
constexpr double kAdaptNisClip = 9.0;
constexpr double kAdaptLogMax = 5.545177444479562;  // 2 ln 16
// How long the model error margin remembers a miss: the time constant of
// its decay, in s.
constexpr double kMissSeconds = 0.05;

}  // namespace

DevicePredictor::DevicePredictor(double mass, double dt, double resolution, double force_rate,
                                 bool adapt_force_rate)
    : dt_(dt), mass_(mass), q_(resolution), force_rate_(force_rate), adapt_(adapt_force_rate) {
  if (!(mass > 0.0) || !(dt > 0.0)) {
    throw std::invalid_argument("DevicePredictor: mass and dt must be positive");
  }
  if (!(resolution >= 0.0) || !(force_rate >= 0.0)) {
    throw std::invalid_argument("DevicePredictor: resolution and force_rate must not be negative");
  }
  c_ = dt * dt / (2.0 * mass);
  noise_ = q_ * q_ / 12.0;
  h_walk_ = (force_rate * dt) * (force_rate * dt);
  p_[0][0] = noise_;
  p_[1][1] = kInitialVelocityVar;
  p_[2][2] = kInitialForceVar;
}

void DevicePredictor::observe(double dx, double phi_held, double h) {
  const bool regular = std::fabs(h - dt_) <= 0.5 * dt_;
  push(dx, phi_held, phi_held, regular ? h / dt_ : 1.0, regular);
  held_ = phi_held;
}

void DevicePredictor::observe(double dx, const std::vector<ForceSegment>& segments, double h) {
  if (segments.size() <= 1) {
    observe(dx, segments.empty() ? 0.0 : segments.front().phi, h);
    return;
  }
  const bool regular = std::fabs(h - dt_) <= 0.5 * dt_;
  const double r = regular ? h / dt_ : 1.0;
  double phi_d, phi_v;
  held_forces(segments, r, phi_d, phi_v);
  push(dx, phi_d, phi_v, r, regular);
  held_ = segments.back().phi;
}

void DevicePredictor::held_forces(const std::vector<ForceSegment>& segments, double r,
                                  double& phi_d, double& phi_v) {
  double total = 0.0;
  for (const auto& seg : segments) total += seg.samples;
  const double scale = total > 0.0 ? r / total : 0.0;
  double imp = 0.0, mom = 0.0, start = 0.0;
  for (const auto& seg : segments) {
    const double ri = seg.samples * scale;
    imp += seg.phi * ri;
    mom += seg.phi * ri * (2.0 * (r - start) - ri);
    start += ri;
  }
  phi_d = mom / (r * r);
  phi_v = imp / r;
}

double DevicePredictor::expected_velocity_change(const std::vector<ForceSegment>& segments,
                                                 double h) const {
  // From the mean velocity over the last sample (length h1) to the end of
  // it, the net force acts as H - phi_v1 over h1 less H - phi_d1 over h1 /
  // 2; from there to the mean over the coming one, as H - phi_d over h / 2.
  double phi_d = 0.0, phi_v = 0.0;
  if (!segments.empty()) held_forces(segments, h / dt_, phi_d, phi_v);
  const double ext = external_force(), h1 = r1_ * dt_;
  return ((ext - 2.0 * phiv1_ + phid1_) * h1 + (ext - phi_d) * h) / (2.0 * mass_);
}

void DevicePredictor::observe_irregular(double dx, double phi_held) {
  push(dx, phi_held, phi_held, 1.0, false);
  held_ = 0.0;
}

double DevicePredictor::realized_work(double dx, const std::vector<ForceSegment>& segments) const {
  if (segments.empty()) return 0.0;
  double total = 0.0;
  for (const auto& seg : segments) total += seg.samples;
  // From the state at the end of the last sample, which the lead does not
  // change.
  double v = velocity_dt();
  const double ext = external_force();
  double work = 0.0, moved = 0.0;
  for (std::size_t i = 0; i + 1 < segments.size(); ++i) {
    const double r = segments[i].samples, phi = segments[i].phi;
    const double d = ready() ? r * v + c_ * r * r * (ext - phi)
                             : (total > 0.0 ? dx * r / total : 0.0);
    work += phi * d;
    moved += d;
    v += 2.0 * c_ * r * (ext - phi);
  }
  return work + segments.back().phi * (dx - moved);
}

void DevicePredictor::set_lead(const std::vector<ForceSegment>& lead) {
  lead_ = lead;
  lead_samples_ = 0.0;
  lead_imp_ = 0.0;
  for (const auto& seg : lead_) {
    lead_samples_ += seg.samples;
    lead_imp_ += seg.phi * seg.samples;
  }
}

double DevicePredictor::lead_work() const {
  double v = velocity_dt(), work = 0.0;
  const double ext = external_force();
  for (const auto& seg : lead_) {
    const double r = seg.samples;
    work += seg.phi * (r * v + c_ * r * r * (ext - seg.phi));
    v += 2.0 * c_ * r * (ext - seg.phi);
  }
  return work;
}

double DevicePredictor::lead_sd() const {
  // A segment s samples into the lead and r long moves r more per unit of
  // v dt, and r (2 s + r) c more per newton of H.
  double gv = 0.0, k = 0.0, start = 0.0;
  for (const auto& seg : lead_) {
    gv += seg.phi * seg.samples;
    k += seg.phi * seg.samples * (2.0 * start + seg.samples);
    start += seg.samples;
  }
  return spread(gv * dt_, k);
}

void DevicePredictor::push(double dx, double phi_d, double phi_v, double r, bool regular) {
  lead_.clear();
  lead_samples_ = 0.0;
  lead_imp_ = 0.0;
  if (q_ <= 0.0 && regular && ready()) {
    // What the model missed over this sample, held as a decaying envelope:
    // exact readings leave no noise to average, only the model's error.
    const double e = dx - (hold_base(r) - hold_compliance(r) * phi_d);
    miss_ = std::max(e * e, (1.0 - std::min(1.0, r * dt_ / kMissSeconds)) * miss_);
  }
  dx2_ = dx1_;
  phid2_ = phid1_;
  phiv2_ = phiv1_;
  r2_ = r1_;
  dx1_ = dx;
  phid1_ = phi_d;
  phiv1_ = phi_v;
  r1_ = r;
  regular_ = regular ? std::min(regular_ + 1, 2) : 0;
  if (q_ > 0.0) filter(dx, phi_d, phi_v, r, regular);
}

void DevicePredictor::filter(double dx, double phi_d, double phi_v, double r, bool regular) {
  if (!regular) {
    // The model did not hold over this sample, so its residual says nothing
    // about v or H: restart from the reading, with the estimates kept as
    // guesses and their uncertainty raised.
    e_ = 0.0;
    const double pv = p_[1][1] + kLostVelocityVar, ph = p_[2][2] + kLostForceVar;
    for (auto& row : p_) {
      for (double& x : row) x = 0.0;
    }
    p_[0][0] = noise_;
    p_[1][1] = pv;
    p_[2][2] = ph;
    return;
  }
  // Predict over the sample, of length h, with the port force: the exact
  // transition of a mass under a net force H - phi.
  const double h = r * dt_;
  const double a = h * h / (2.0 * mass_), b = h / mass_;
  const double f[3][3] = {{1.0, h, a}, {0.0, 1.0, b}, {0.0, 0.0, 1.0}};
  e_ += v_ * h + a * (h_ - phi_d) - dx;  // the reading moved by dx
  v_ += b * (h_ - phi_v);
  double fp[3][3], p[3][3];
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      fp[i][j] = 0.0;
      for (int k = 0; k < 3; ++k) fp[i][j] += f[i][k] * p_[k][j];
    }
  }
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      p[i][j] = 0.0;
      for (int k = 0; k < 3; ++k) p[i][j] += fp[i][k] * f[j][k];
    }
  }
  p[2][2] += h_walk_ * (adapt_ ? std::exp(log_walk_) : 1.0) * r;
  // Update with the reading; the innovation is reading - estimate = -e.
  const double s = p[0][0] + noise_;
  const double k[3] = {p[0][0] / s, p[1][0] / s, p[2][0] / s};
  const double nu = -e_;
  // What the innovations show beyond the filter's own spread S, the model
  // error with noise averaged out; a consistent filter keeps it near zero.
  excess_ += std::min(1.0, h / kMissSeconds) * (nu * nu - s - excess_);
  miss_ = std::max(0.0, excess_);
  if (adapt_) {
    // Covariance matching: a consistent filter's normalized innovation
    // squared averages 1; more says it trusts its estimates of v and H too
    // much, and the walk's variance grows until it does not.
    const double nis = std::min(nu * nu / s, kAdaptNisClip);
    log_walk_ = std::clamp(log_walk_ + kAdaptPerSecond * h * (nis - 1.0), 0.0, kAdaptLogMax);
  }
  e_ += k[0] * nu;
  v_ += k[1] * nu;
  h_ += k[2] * nu;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) p_[i][j] = p[i][j] - k[i] * p[0][j];
  }
}

double DevicePredictor::force_rate() const {
  return adapt_ ? force_rate_ * std::exp(0.5 * log_walk_) : force_rate_;
}

double DevicePredictor::base() const {
  // The history starts at zero, i.e. the device is taken to have been at rest
  // and unforced before the first sample.
  return hold_base(1.0);
}

double DevicePredictor::external_force() const {
  if (q_ > 0.0) return h_;
  // With r_j = h_j / dt and F_j = H - phi_j, dx_j / r_j is the displacement
  // at the mean velocity over sample j scaled to dt, and the mean velocities
  // of two adjacent samples differ by (F_1 h_1 + F_2 h_2) / (2 m). For equal
  // samples: dx1 - dx2 = c * (F1 + F2). A force that changed within the
  // older sample enters by how it moved the device (phi_d) and how it left
  // its velocity (phi_v), as 2 phi_v - phi_d.
  return ((dx1_ / r1_ - dx2_ / r2_) / c_ + phid1_ * r1_ + (2.0 * phiv2_ - phid2_) * r2_) /
         (r1_ + r2_);
}

double DevicePredictor::velocity_dt() const {
  if (q_ > 0.0) return v_ * dt_;
  // The velocity at the end of the last sample: its mean velocity plus half
  // the change over it, F_1 h_1 / (2 m), times dt; for a force that changed
  // within the sample, with 2 phi_v - phi_d for phi_1.
  return dx1_ / r1_ + c_ * r1_ * (external_force() - (2.0 * phiv1_ - phid1_));
}

double DevicePredictor::hold_base(double n) const {
  if (lead_.empty()) return n * velocity_dt() + n * n * c_ * external_force();
  // From where the lead ends: the velocity it leaves, and H acting on.
  const double ext = external_force();
  const double v = velocity_dt() + 2.0 * c_ * (ext * lead_samples_ - lead_imp_);
  return n * v + n * n * c_ * ext;
}

double DevicePredictor::hold_sd(double n) const {
  // Over samples R to R + n after the last one, with R the lead's length,
  // the displacement grows by n per unit of v dt and by
  // (R + n)^2 - R^2 = n (2 R + n) times c per newton of H.
  return spread(n * dt_, n * (2.0 * lead_samples_ + n));
}

double DevicePredictor::spread(double gv, double k) const {
  // A miss over one sample taken as an unmodeled force grows as k: n^2
  // over n samples.
  const double miss = k * k * miss_;
  if (q_ <= 0.0) return std::sqrt(miss);
  const double gh = k * c_;
  const double var = gv * gv * p_[1][1] + 2.0 * gv * gh * p_[1][2] + gh * gh * p_[2][2] + miss;
  return std::sqrt(std::max(0.0, var));
}

namespace {

// Largest lambda in [0, 1] with lambda * g - lambda^2 * h >= -b, for h > 0
// and b >= 0.
double affordable_scale(double g, double h, double b) {
  if (g - h >= -b) return 1.0;
  // The positive root of h lambda^2 - g lambda - b = 0. It lies in [0, 1)
  // because the left side is <= 0 at lambda = 0 and > 0 at lambda = 1.
  const double lambda = (g + std::sqrt(g * g + 4.0 * h * b)) / (2.0 * h);
  return std::clamp(lambda, 0.0, 1.0);
}

// The same with the tax q * max(0, lambda * F - a) added to the cost, for
// F = |phi| > 0 and a >= 0 the part of the held force with phi's sign.
double affordable_scale_taxed(double g, double h, double b, double F, double a, double q) {
  const double l = affordable_scale(g, h, b);
  if (q <= 0.0 || l * F <= a) return l;
  // Beyond lambda0 = a / F the cost is steeper by q * F. The constraint is
  // still concave and holds at lambda0 (untaxed there, and lambda0 < l), so
  // its largest root beyond lambda0 is the answer.
  return std::max(a / F, affordable_scale(g - q * F, h, b + q * a));
}

}  // namespace

double quantization_tax(double q, double prev, double phi) {
  const double kept = phi * prev > 0.0 ? std::fabs(prev) : 0.0;
  return q * std::max(0.0, std::fabs(phi) - kept);
}

PcResult limit_force_predictive(double budget, double phi, const DevicePredictor& device,
                                int hold_samples, double confidence, int tax_samples,
                                int rise_samples) {
  PcResult r;
  r.value = phi;
  if (phi == 0.0) return r;
  if (!device.ready()) {
    r.value = 0.0;
    r.gain = 1.0;
    r.active = true;
    return r;
  }
  // Forces already in flight do their work first: the budget is what is
  // left after the lower bound of it.
  if (device.lead_samples() > 0.0) {
    budget += device.lead_work() - confidence * device.lead_sd();
  }
  const double b = std::max(0.0, budget);
  const double q = device.resolution();
  const double F = std::fabs(phi);
  const double kept = phi * device.held_force() > 0.0 ? std::fabs(device.held_force()) : 0.0;
  // W(lambda) = lambda * g - lambda^2 * h with h > 0 over n samples, its
  // lower confidence bound with `confidence`.
  auto g = [&](double n) { return phi * device.hold_base(n) - confidence * F * device.hold_sd(n); };
  auto h = [&](double n) { return phi * phi * device.hold_compliance(n); };
  // The tax is paid once whatever the horizon: within every horizon, or
  // with tax_samples over that horizon alone.
  const double tq = tax_samples > 0 ? 0.0 : q;
  double l = affordable_scale_taxed(g(1.0), h(1.0), b, F, kept, tq);
  if (hold_samples > 1) {
    l = std::min(l, affordable_scale_taxed(g(hold_samples), h(hold_samples), b, F, kept, tq));
  }
  if (rise_samples > 0 && q > 0.0 && l * F > kept) {
    // A rise, which pays a tax, goes no further than the force the expected
    // motion could pay for over rise_samples samples; the held force may
    // stay. This guards the tax, not passivity, so it takes the mean.
    const double gm = phi * device.hold_base(rise_samples);
    l = std::min(l, std::max(kept / F, affordable_scale(gm, h(rise_samples), b)));
  }
  if (tax_samples > 0 && l * F > kept) {
    // A rise must repay the books' deficit and its own tax within
    // tax_samples = N:
    // lambda * g(N) - lambda^2 * h(N) - q * (lambda * F - kept) >= -budget.
    // The left side is concave, so the rises that do lie in one interval
    // [lo, hi]. Books in deficit can exclude the small rises; if the other
    // constraints allow only those, the force stays at the kept one.
    const double l0 = kept / F, gn = g(tax_samples) - q * F, hn = h(tax_samples);
    const double c0 = q * kept + budget;
    const double disc = gn * gn + 4.0 * hn * c0;
    if (disc < 0.0) {
      l = l0;
    } else {
      const double root = std::sqrt(disc);
      // lo * hi = -c0 / hn; this form of lo does not cancel for small c0.
      const double lo = gn > 0.0 ? -2.0 * c0 / (gn + root) : (gn - root) / (2.0 * hn);
      const double hi = gn - hn + c0 >= 0.0 ? 1.0 : (gn + root) / (2.0 * hn);
      l = l >= lo ? std::max(l0, std::min(l, hi)) : l0;
    }
  }
  if (l >= 1.0) return r;  // the full force is affordable
  r.value = l * phi;
  r.gain = 1.0 - l;
  r.active = true;
  return r;
}

}  // namespace tuushin::control
