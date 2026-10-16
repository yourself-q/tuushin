#include "tuushin/control/predictive_pc.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
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
// The readings the bounds on where the device lies in its count rest on,
// and how fast they take the forces other than the port force to change,
// N/s ("Crossings").
constexpr std::size_t kCountWindow = 24;
constexpr double kForceDrift = 50.0;
// How far from a whole number of counts a reading displacement may lie and
// still count as one: rounding, not a resolution other than the filter's.
constexpr double kCountTolerance = 1e-6;
// How far outside its count, in counts, the linear program lets the device
// lie beyond what the force's drift within a sample allows: rounding.
constexpr double kCountSlack = 1e-9;

}  // namespace

DevicePredictor::DevicePredictor(double mass, double dt, double resolution, double force_rate,
                                 bool adapt_force_rate)
    : dt_(dt),
      mass_(mass),
      q_(resolution),
      force_rate_(force_rate),
      adapt_(adapt_force_rate),
      window_size_(kCountWindow),
      force_drift_(kForceDrift) {
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
  if (q_ > 0.0) {
    filter(dx, phi_d, phi_v, r, regular);
    remember(dx, phi_d, phi_v, r, regular);
  }
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

void DevicePredictor::remember(double dx, double phi_d, double phi_v, double r, bool regular) {
  lp_.reset();
  solved_ = false;
  found_.clear();
  const double counts = dx / q_;
  if (!regular || std::fabs(counts - std::round(counts)) > kCountTolerance) {
    // Not a sample the model describes, or not whole counts: the encoder's
    // resolution is not the one assumed.
    window_.clear();
    return;
  }
  window_.push_back({std::round(counts), phi_d, phi_v, r});
  while (window_.size() > window_size_) window_.pop_front();
}

void DevicePredictor::set_count_window(std::size_t readings, double force_drift) {
  window_size_ = readings;
  force_drift_ = force_drift;
  while (window_.size() > window_size_) window_.pop_front();
  lp_.reset();
  solved_ = false;
  found_.clear();
}

bool DevicePredictor::solve_window() const {
  // In counts, the state at the window's start: s0, where the device lies
  // from its count's lower edge; w0, its velocity times dt; g0, H times
  // c / q. H changes from one sample to the next by xi_j times the drift
  // over the time between their middles, xi_j in [-1, 1], and sigma_j is
  // where the device lies after sample j, an affine function of
  // z = (s0, w0, g0, xi): S_j z + K_j - sigma_j = 0, sigma_j in [0, 1]. Over
  // a sample of r, the model moves s by r w + r^2 (g - c phi_d / q) less the
  // counts the reading moved by, and w by 2 r (g - c phi_v / q).
  if (solved_) return lp_.has_value();
  solved_ = true;
  const std::size_t n = window_.size();
  if (n == 0) return false;
  const std::size_t nz = 3 + (n - 1), cols = nz + n;
  lp_.emplace(n, cols);
  BoundedLp& lp = *lp_;
  const double cq = c_ / q_;
  const double kappa = cq * force_drift_ * dt_;
  std::vector<double> sv(nz, 0.0), wv(nz, 0.0), gv(nz, 0.0);
  sv[0] = wv[1] = gv[2] = 1.0;
  double ks = 0.0, kw = 0.0, span = 0.0, moved = 0.0;
  for (const auto& rd : window_) {
    span += rd.r;
    moved += rd.counts;
  }
  // Phase 1 starts in the middle of the count, at the mean velocity, with
  // the filter's H.
  std::vector<double> z(nz, 0.0);
  z[0] = 0.5;
  z[1] = moved / span;
  z[2] = cq * h_;
  lp.set_bounds(0, 0.0, 1.0);
  lp.set_bounds(1, -std::numeric_limits<double>::infinity(),
                std::numeric_limits<double>::infinity());
  lp.set_bounds(2, -std::numeric_limits<double>::infinity(),
                std::numeric_limits<double>::infinity());
  for (std::size_t j = 0; j + 1 < n; ++j) lp.set_bounds(3 + j, -1.0, 1.0);
  for (std::size_t k = 0; k < nz; ++k) lp.set_start(k, z[k]);
  double slack = kCountSlack;
  for (std::size_t j = 0; j < n; ++j) {
    const Reading& rd = window_[j];
    const double r = rd.r;
    if (j > 0) gv[3 + j - 1] = kappa * 0.5 * (window_[j - 1].r + r);
    for (std::size_t k = 0; k < nz; ++k) sv[k] += r * wv[k] + r * r * gv[k];
    ks += r * kw - r * r * cq * rd.phi_d - rd.counts;
    for (std::size_t k = 0; k < nz; ++k) wv[k] += 2.0 * r * gv[k];
    kw -= 2.0 * r * cq * rd.phi_v;
    // Within a sample H may stray from its mean by the drift; that moves the
    // device by up to drift h^3 / (12 m), which the model leaves out and
    // which adds up from sample to sample.
    slack += kappa * r * r * r / 6.0;
    double at = ks;
    for (std::size_t k = 0; k < nz; ++k) {
      if (sv[k] != 0.0) lp.set(j, k, sv[k]);
      at += sv[k] * z[k];
    }
    lp.set(j, nz + j, -1.0);
    lp.set_rhs(j, -ks);
    lp.set_bounds(nz + j, -slack, 1.0 + slack);
    lp.set_start(nz + j, at);
  }
  if (!lp.solve_feasible()) {
    // The readings leave no motion the model allows: H changed faster than
    // assumed, or the model does not hold. Start again from the next.
    window_.clear();
    lp_.reset();
    return false;
  }
  return true;
}

double DevicePredictor::model_slack(std::size_t lag) const {
  double slack = kCountSlack;
  const double kappa = c_ / q_ * force_drift_ * dt_;
  for (std::size_t j = 0; j + lag < window_.size(); ++j) {
    const double r = window_[j].r;
    slack += kappa * r * r * r / 6.0;
  }
  return slack;
}

double DevicePredictor::worst_tax(const std::vector<LaggedTax>& taxes) const {
  constexpr double kUnknown = std::numeric_limits<double>::infinity();
  if (taxes.empty()) return 0.0;
  if (q_ <= 0.0 || noise_ > q_ * q_ / 12.0) return kUnknown;
  for (const auto& t : taxes) {
    if (t.lag >= window_.size()) return kUnknown;
  }
  if (!solve_window()) return kUnknown;
  // With u = q (1 - s) the sum is sum (alpha + q beta) - q sum beta s,
  // largest where sum beta s is least; each s may lie off the model's by
  // its slack.
  const std::size_t n = window_.size(), nz = 2 + n;
  std::vector<double> c(lp_->cols(), 0.0);
  double fixed = 0.0;
  for (const auto& t : taxes) {
    c[nz + (n - 1 - t.lag)] += t.line.beta;
    fixed += t.line.alpha + q_ * t.line.beta + q_ * std::fabs(t.line.beta) * model_slack(t.lag);
  }
  double least = 0.0;
  if (!lp_->minimize(c, least)) return kUnknown;
  return fixed - q_ * least;
}

DevicePredictor::CountBounds DevicePredictor::count_bounds() const {
  if (lead_samples_ >= 1.0) return {0.0, q_};
  return count_bounds_at(0);
}

DevicePredictor::CountBounds DevicePredictor::count_bounds_at(std::size_t lag) const {
  const CountBounds unknown{0.0, q_};
  // Nothing is known on readings noisier than their quantization.
  if (q_ <= 0.0 || noise_ > q_ * q_ / 12.0 || lag >= window_.size()) return unknown;
  if (!solve_window() || lag >= window_.size()) return unknown;
  if (found_.size() <= lag) found_.resize(lag + 1, CountBounds{1.0, 0.0});
  CountBounds& f = found_[lag];
  if (f.lo > f.hi) {
    const std::size_t n = window_.size(), nz = 2 + n, col = nz + (n - 1 - lag);
    std::vector<double> c(lp_->cols(), 0.0);
    double least = 0.0, most = 0.0;
    c[col] = 1.0;
    const bool ok = lp_->minimize(c, least);
    c[col] = -1.0;
    if (!ok || !lp_->minimize(c, most)) return unknown;
    most = -most;
    // The device's true place differs from the model's by up to the slack
    // within samples up to that reading.
    const double slack = model_slack(lag);
    least = std::clamp(least - slack, 0.0, 1.0);
    most = std::clamp(most + slack, 0.0, 1.0);
    // u = q (1 - s).
    f = {q_ * (1.0 - most), q_ * (1.0 - least)};
  }
  return f;
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

// The same with a tax added to the cost, for F = |phi| > 0 and a >= 0 the
// part of the held force with phi's sign: `rise` per newton by which
// lambda * F exceeds a, and `fall` per newton by which it falls short of
// it, a credit.
double affordable_scale_taxed(double g, double h, double b, double F, double a, double rise,
                              double fall) {
  // Up to lambda0 = a / F the cost is -lambda * (g - fall * F) +
  // lambda^2 * h - fall * a, which holds at lambda = 0.
  const double l = affordable_scale(g - fall * F, h, b + fall * a);
  if (rise == fall || l * F < a) return l;
  // Beyond lambda0 the cost is the other quadratic, which agrees with the
  // first at lambda0 and holds there (lambda0 <= l; with a = 0 there is no
  // first piece, and the cost holds at zero). Each holds on an
  // interval that contains zero, so the largest root of the second beyond
  // lambda0 is the answer.
  return std::max(a / F, affordable_scale(g - rise * F, h, b + rise * a));
}

}  // namespace

double quantization_tax(double q, double prev, double phi) {
  const double kept = phi * prev > 0.0 ? std::fabs(prev) : 0.0;
  return q * std::max(0.0, std::fabs(phi) - kept);
}

namespace {

// The tax of a change from `from` to `to` on one side of zero: per newton
// by which |phi| grows, q - u for a positive force and u for a negative one.
TaxLine side_tax(double q, double from, double to) {
  const double side = to != 0.0 ? to : from;
  const double d = std::fabs(to) - std::fabs(from);
  return side > 0.0 ? TaxLine{q * d, -d} : TaxLine{0.0, d};
}

}  // namespace

TaxLine quantization_tax_line(double q, double prev, double phi) {
  if (prev * phi >= 0.0) return side_tax(q, prev, phi);
  const TaxLine a = side_tax(q, prev, 0.0), b = side_tax(q, 0.0, phi);
  return {a.alpha + b.alpha, a.beta + b.beta};
}

double quantization_tax(double q, double prev, double phi, double lo, double hi) {
  return quantization_tax_line(q, prev, phi).worst(lo, hi);
}

double quantization_tax(const DevicePredictor& device, double phi) {
  const auto u = device.count_bounds();
  return quantization_tax(device.resolution(), device.held_force(), phi, u.lo, u.hi);
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
  // What a newton of rise beyond the held force is taxed, and what a newton
  // of fall towards zero is credited, as the books charge it
  // (quantization_tax_line at the device's count_bounds()); a held force of
  // the other sign is credited its fall to zero whatever phi's size.
  const auto u = device.count_bounds();
  const double rate = phi > 0.0 ? q - u.lo : u.hi;
  const double credit = phi > 0.0 ? q - u.hi : u.lo;
  const double F = std::fabs(phi);
  const double held = device.held_force();
  const double kept = phi * held > 0.0 ? std::fabs(held) : 0.0;
  const double flip = phi * held < 0.0 ? -quantization_tax(q, held, 0.0, u.lo, u.hi) : 0.0;
  // W(lambda) = lambda * g - lambda^2 * h with h > 0 over n samples, its
  // lower confidence bound with `confidence`.
  auto g = [&](double n) { return phi * device.hold_base(n) - confidence * F * device.hold_sd(n); };
  auto h = [&](double n) { return phi * phi * device.hold_compliance(n); };
  // The tax is paid once whatever the horizon: within every horizon, or
  // with tax_samples over that horizon alone.
  const double tq = tax_samples > 0 ? 0.0 : rate;
  double l = affordable_scale_taxed(g(1.0), h(1.0), b + flip, F, kept, tq, credit);
  if (hold_samples > 1) {
    l = std::min(l, affordable_scale_taxed(g(hold_samples), h(hold_samples), b + flip, F, kept,
                                           tq, credit));
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
    // tax_samples = N, less a held force of the other sign's credit:
    // lambda * g(N) - lambda^2 * h(N) - rate * (lambda * F - kept) >= -budget - flip.
    // The left side is concave, so the rises that do lie in one interval
    // [lo, hi]. Books in deficit can exclude the small rises; if the other
    // constraints allow only those, the force stays at the kept one.
    const double l0 = kept / F, gn = g(tax_samples) - rate * F, hn = h(tax_samples);
    const double c0 = rate * kept + budget + flip;
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
