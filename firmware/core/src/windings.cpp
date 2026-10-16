#include "tuushin/fw/windings.hpp"

#include <algorithm>
#include <cmath>

// The fit, in the frequencies' geometric mean w0 and a gain K that make
// every coefficient of order 1: with x = w / w0 and a = A0 / A, A0 the
// amplitudes' geometric mean,
//
//   1 / G = K e^{jxd} [c0 + c1 (jx) + c2 (jx)^2 + c3 (jx)^3 + c4 a (j - x c3 / c2)],
//
// d = w0 D, so that L / R = c3 / (c2 w0), the inertia's p2 = K c2 / w0^2
// and Coulomb's q = K c4 A0. For a d, the imaginary parts give c1, c3, c4
// by linear least squares, and the real parts c0, c2 for a c3 / c2, which
// a few rounds find. d is searched for: on a grid over the delays a tick
// allows, then by golden section. The voltage's images about the sampling
// rate are then taken out with the fit, and the fit made again, three
// times. Last, Gauss-Newton on all six finds the least squares the rounds
// came near, and its normal matrix their covariance.
//
// The squares are each point's, relative to its noise, which the lock-in
// measured from what it left (WindingIdent): the points at the lowest
// frequency, where friction and cogging are most of the torque, and at
// the highest, where the angle is least, count for less. Unweighted, the
// spread of L / R in simulation was 1.6 %, weighted 0.7 %, and the
// spread the fit itself gives (tau_sd) matched it only weighted.
//
// In simulation (the motor of tests/test_fw_windings.cpp: a 200 us tick,
// L 1.5 mH, R 5 ohm, J 7e-6 kg m^2, a 50 us PWM period, 0.5 s at each of 6
// frequencies from 125 to 625 Hz, an 8 mrad amplitude, a 5 V limit, the
// encoder's noise half a count; 8 seeds each, 4.1 s a run): L / R off by
// -0.4 % on average, spread 1.1 %, tau_sd 1.3 %; with Coulomb friction of
// 4e-3 N m, +0.9 %; with cogging of 1.2e-2 N m, eight times a gimbal
// motor's, +0.4 %; with both at 4e-3, +0.7 %; with the encoder's noise at
// 2 counts, -1.3 %, spread 3.6 %, tau_sd 4.4 %; at L 0.5 mH, where the
// inductance's lag is a smaller part of the phase, -0.5 %, spread 5 %,
// tau_sd 5.7 % (32 seeds); at 3 mH, +0.3 %, spread 0.7 %; with the PWM's
// period 0 or 100 us, -0.3 and 0.0 %. A hand on the knob (an inertia of
// 1e-5 kg m^2 and a damping of 2e-3 N m s/rad) was rejected 8 times of 8.
// The bias grows with friction, +2.7 % at a friction ratio of 0.22, and
// with the cogging's stiffness, +3.4 % at a ratio of 0.33, past the
// checks' limits. At a 1 ms tick (QEMU's) the frequencies come down to
// 50-250 Hz, where friction is a larger part of the torque: a gimbal
// motor's friction read as a ratio of about 0.23, and L / R was rejected.

namespace tuushin::fw {

namespace {

using cd = std::complex<double>;
constexpr double kPiD = 3.14159265358979323846;
constexpr int kMax = WindingIdent::kMaxPoints;

// Solves a x = b for a square a of order n (row-major, overwritten) by
// Gaussian elimination with partial pivoting; false if it is singular.
bool solve(double* a, double* b, int n) {
  for (int k = 0; k < n; ++k) {
    int piv = k;
    for (int i = k + 1; i < n; ++i)
      if (std::fabs(a[i * n + k]) > std::fabs(a[piv * n + k])) piv = i;
    if (!(std::fabs(a[piv * n + k]) > 0.0)) return false;
    if (piv != k) {
      for (int j = 0; j < n; ++j) std::swap(a[k * n + j], a[piv * n + j]);
      std::swap(b[k], b[piv]);
    }
    for (int i = k + 1; i < n; ++i) {
      const double r = a[i * n + k] / a[k * n + k];
      for (int j = k; j < n; ++j) a[i * n + j] -= r * a[k * n + j];
      b[i] -= r * b[k];
    }
  }
  for (int k = n - 1; k >= 0; --k) {
    double s = b[k];
    for (int j = k + 1; j < n; ++j) s -= a[k * n + j] * b[j];
    b[k] = s / a[k * n + k];
  }
  return true;
}

struct Data {
  int n = 0;
  double w0 = 0.0, k = 0.0, ts = 0.0;
  double x[kMax] = {}, a[kMax] = {}, wt[kMax] = {};
  cd z[kMax];      // 1 / (K g): the normalized 1 / G, the delay in it
  cd raw[kMax];    // z before the images are taken out
};

struct Coef {
  double c[5] = {};
  double d = 0.0;
  double res = 0.0;  // the weighted squares
};

// The model's 1 / (K G) at x, without the delay.
cd model(const Coef& f, const Data& m, int i) {
  const double x = m.x[i], a = m.a[i];
  const double r = f.c[2] != 0.0 ? f.c[3] / f.c[2] : 0.0;
  return {f.c[0] - f.c[2] * x * x - f.c[4] * a * x * r, f.c[1] * x - f.c[3] * x * x * x + f.c[4] * a};
}

cd measured(const Data& m, int i, double d) {
  return m.z[i] * std::exp(cd(0.0, -m.x[i] * d));
}

double squares(const Coef& f, const Data& m) {
  double s = 0.0;
  for (int i = 0; i < m.n; ++i) s += m.wt[i] * std::norm(measured(m, i, f.d) - model(f, m, i));
  return s;
}

// The least squares for the delay d.
Coef fit_at(const Data& m, double d) {
  Coef f;
  f.d = d;
  cd y[kMax];
  for (int i = 0; i < m.n; ++i) y[i] = measured(m, i, d);
  // The imaginary parts: c1 x - c3 x^3 + c4 a.
  double a3[9] = {}, b3[3] = {};
  for (int i = 0; i < m.n; ++i) {
    const double x = m.x[i];
    const double r[3] = {x, -x * x * x, m.a[i]};
    for (int u = 0; u < 3; ++u) {
      b3[u] += m.wt[i] * r[u] * y[i].imag();
      for (int v = 0; v < 3; ++v) a3[u * 3 + v] += m.wt[i] * r[u] * r[v];
    }
  }
  if (!solve(a3, b3, 3)) {
    f.res = HUGE_VAL;
    return f;
  }
  f.c[1] = b3[0];
  f.c[3] = b3[1];
  f.c[4] = b3[2];
  // The real parts: c0 - c2 x^2 - c4 a x c3 / c2, for c3 / c2 as it was.
  double r = 0.0;
  for (int round = 0; round < 4; ++round) {
    double a2[4] = {}, b2[2] = {};
    for (int i = 0; i < m.n; ++i) {
      const double x = m.x[i];
      const double q[2] = {1.0, -x * x};
      const double rhs = y[i].real() + f.c[4] * m.a[i] * x * r;
      for (int u = 0; u < 2; ++u) {
        b2[u] += m.wt[i] * q[u] * rhs;
        for (int v = 0; v < 2; ++v) a2[u * 2 + v] += m.wt[i] * q[u] * q[v];
      }
    }
    if (!solve(a2, b2, 2)) {
      f.res = HUGE_VAL;
      return f;
    }
    f.c[0] = b2[0];
    f.c[2] = b2[1];
    if (f.c[2] == 0.0) break;
    r = f.c[3] / f.c[2];
  }
  f.res = squares(f, m);
  return f;
}

// The delay that fits best within [lo, hi], by golden section.
Coef golden(const Data& m, double lo, double hi, int iterations) {
  const double g = 0.5 * (std::sqrt(5.0) - 1.0);
  double a = hi - g * (hi - lo), b = lo + g * (hi - lo);
  double fa = fit_at(m, a).res, fb = fit_at(m, b).res;
  for (int i = 0; i < iterations; ++i) {
    if (fa < fb) {
      hi = b;
      b = a;
      fb = fa;
      a = hi - g * (hi - lo);
      fa = fit_at(m, a).res;
    } else {
      lo = a;
      a = b;
      fa = fb;
      b = lo + g * (hi - lo);
      fb = fit_at(m, b).res;
    }
  }
  return fit_at(m, 0.5 * (lo + hi));
}

// What the held voltage's images about the sampling rate add, by the fit,
// to what the readings fold onto each point: the sum over them of the
// response, held, over the fundamental's.
void take_out_images(Data& m, const Coef& f) {
  const double ws = 2.0 * kPiD / m.ts;
  auto held = [&](double w) {
    const double x = w / m.w0, h = 0.5 * w * m.ts;
    const cd s(0.0, x);
    const cd p = f.c[0] + s * (f.c[1] + s * (f.c[2] + s * f.c[3]));
    return std::exp(cd(0.0, -x * f.d - h)) * (std::sin(h) / h) / p;
  };
  for (int i = 0; i < m.n; ++i) {
    const double w = m.x[i] * m.w0;
    cd sum = 0.0;
    for (int k = -6; k <= 6; ++k) sum += held(w + k * ws);
    m.z[i] = m.raw[i] * (sum / held(w));
  }
}

// The residuals' derivatives in c0..c4 and d at point i, each over the
// point's noise: the rows of the Jacobian, as complex numbers whose parts
// are its two real rows.
void jacobian(const Coef& f, const Data& m, int i, cd* row, cd* e) {
  const double x = m.x[i], a = m.a[i], c2 = f.c[2], c3 = f.c[3], c4 = f.c[4];
  const double s = std::sqrt(m.wt[i]);
  const cd y = measured(m, i, f.d);
  *e = s * (y - model(f, m, i));
  row[0] = -s;
  row[1] = -s * cd(0.0, x);
  row[2] = -s * cd(-x * x + c4 * a * x * c3 / (c2 * c2), 0.0);
  row[3] = -s * cd(-c4 * a * x / c2, -x * x * x);
  row[4] = -s * a * cd(-x * c3 / c2, 1.0);
  row[5] = s * cd(0.0, -x) * y;
}

// The normal matrix and gradient of the squares in the six.
void normal(const Coef& f, const Data& m, double* nm, double* gr) {
  for (int u = 0; u < 36; ++u) nm[u] = 0.0;
  for (int u = 0; u < 6; ++u) gr[u] = 0.0;
  for (int i = 0; i < m.n; ++i) {
    cd row[6], e;
    jacobian(f, m, i, row, &e);
    for (int u = 0; u < 6; ++u) {
      gr[u] += (std::conj(row[u]) * e).real();
      for (int v = 0; v < 6; ++v) nm[u * 6 + v] += (std::conj(row[u]) * row[v]).real();
    }
  }
}

void gauss_newton(Coef& f, const Data& m) {
  for (int it = 0; it < 4; ++it) {
    double nm[36], gr[6];
    normal(f, m, nm, gr);
    for (double& g : gr) g = -g;
    if (!solve(nm, gr, 6)) return;
    for (double step = 1.0; step > 1e-3; step *= 0.5) {
      Coef t = f;
      for (int u = 0; u < 5; ++u) t.c[u] += step * gr[u];
      t.d += step * gr[5];
      if (t.c[2] == 0.0) continue;
      t.res = squares(t, m);
      if (t.res <= f.res) {
        f = t;
        break;
      }
    }
  }
}

}  // namespace

WindingFit fit_windings(const WindingPoint* pts, int n, double ts, double delay_max) {
  WindingFit out;
  if (n < 4 || n > kMax || !(ts > 0.0) || !(delay_max > -0.1 * ts)) {
    out.why = "too few points";
    return out;
  }
  Data m;
  m.n = n;
  m.ts = ts;
  double lw = 0.0, la = 0.0;
  for (int i = 0; i < n; ++i) {
    if (!(pts[i].w > 0.0 && pts[i].amp > 0.0 && std::abs(pts[i].g) > 0.0)) {
      out.why = "a point without a response";
      return out;
    }
    lw += std::log(pts[i].w);
    la += std::log(pts[i].amp);
  }
  m.w0 = std::exp(lw / n);
  const double a0 = std::exp(la / n);
  double lk = 0.0;
  for (int i = 0; i < n; ++i) {
    m.x[i] = pts[i].w / m.w0;
    lk += std::log(1.0 / (std::abs(pts[i].g) * m.x[i] * m.x[i]));
  }
  m.k = std::exp(lk / n);
  for (int i = 0; i < n; ++i) {
    m.a[i] = a0 / pts[i].amp;
    m.raw[i] = m.z[i] = 1.0 / (m.k * pts[i].g);
    // Each part of z is off by about rel |z|; no point counts for more
    // than a part in 1e6 of its value would.
    const double rel = std::max(pts[i].rel, 1e-6);
    m.wt[i] = 1.0 / (std::norm(m.z[i]) * rel * rel);
  }

  // The delay, from none (less a margin for the noise) to the longest
  // given, where L / R comes out positive. The data can hardly tell D
  // from L / R: both lag the angle by about w (D + L / R), and only at
  // the highest frequencies, where w L / R nears 1, do they differ. At a
  // 1 ms tick, which brings the frequencies down to 50-250 Hz, the
  // squares had a second minimum near 650 us, with L / R negative, below
  // the one near the true 50 us.
  const double lo = -0.1 * ts * m.w0, hi = delay_max * m.w0;
  constexpr int kGrid = 25;
  const double step = (hi - lo) / (kGrid - 1);
  Coef f;
  f.res = HUGE_VAL;
  for (int i = 0; i < kGrid; ++i) {
    const Coef g = fit_at(m, lo + i * step);
    if (g.c[2] > 0.0 && g.c[3] > 0.0 && g.res < f.res) f = g;
  }
  if (f.res == HUGE_VAL) {
    out.why = "no inductance in the response";
    return out;
  }
  f = golden(m, std::max(lo, f.d - step), std::min(hi, f.d + step), 30);
  for (int round = 0; round < 3; ++round) {
    take_out_images(m, f);
    f = golden(m, std::max(lo, f.d - step), std::min(hi, f.d + step), 20);
  }
  take_out_images(m, f);
  f.res = squares(f, m);
  gauss_newton(f, m);

  const double c2 = f.c[2], c3 = f.c[3];
  out.tau = c2 != 0.0 ? c3 / (c2 * m.w0) : 0.0;
  out.delay = f.d / m.w0;
  out.p1 = m.k * f.c[1] / m.w0;
  out.p2 = m.k * c2 / (m.w0 * m.w0);
  const int dof = 2 * n - 6;
  out.chi2 = dof > 0 ? f.res / dof : f.res;
  for (int i = 0; i < n; ++i) {
    const double x2 = m.x[i] * m.x[i];
    if (c2 > 0.0) {
      out.friction = std::max(out.friction, std::fabs(f.c[4]) * m.a[i] / (c2 * x2));
      out.stiffness = std::max(out.stiffness, std::fabs(f.c[0]) / (c2 * x2));
    }
  }
  // The covariance: the normal matrix's inverse, scaled up by the misfit
  // where it is more than the noise.
  bool cov = false;
  if (c2 > 0.0 && c3 > 0.0) {
    double nm[36], gr[6];
    normal(f, m, nm, gr);
    double nm2[36];
    std::copy(nm, nm + 36, nm2);
    double gt[6] = {0.0, 0.0, -out.tau / c2, 1.0 / (c2 * m.w0), 0.0, 0.0};
    const double grad[6] = {gt[0], gt[1], gt[2], gt[3], gt[4], gt[5]};
    double gd[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    if (solve(nm, gt, 6) && solve(nm2, gd, 6)) {
      const double scale = std::max(1.0, out.chi2);
      double vt = 0.0;
      for (int u = 0; u < 6; ++u) vt += grad[u] * gt[u];
      if (vt >= 0.0 && gd[5] >= 0.0) {
        out.tau_sd = std::sqrt(vt * scale);
        out.delay_sd = std::sqrt(gd[5] * scale) / m.w0;
        cov = true;
      }
    }
  }

  if (!(c2 > 0.0)) {
    out.why = "no inertia in the response";
  } else if (!(c3 > 0.0)) {
    out.why = "no inductance in the response";
  } else if (!cov) {
    out.why = "singular fit";
  } else if (f.d < lo + 0.02 * (hi - lo) || f.d > hi - 0.02 * (hi - lo)) {
    out.why = "delay at the edge of its range";
  } else if (out.friction > 0.15) {
    out.why = "friction too large";
  } else if (out.stiffness > 0.25) {
    out.why = "shaft held or sprung";
  } else if (out.chi2 > 25.0) {
    out.why = "response not the model's";
  } else if (out.tau_sd > 0.1 * out.tau) {
    out.why = "too imprecise";
  } else {
    out.ok = true;
  }
  return out;
}

void WindingIdent::start(const Setup& s, std::uint32_t now) {
  s_ = s;
  count_ = point_ = 0;
  last_ = now;
  steps_ = late_ = 0;
  wandered_ = false;
  const double ts = s.tick_us * 1e-6;
  if (!(ts > 0.0)) return;
  // Six frequencies, evenly in log from a fifth of the highest to the
  // highest: 625 Hz, or a quarter of the tick's rate, whichever is lower,
  // each period a whole number of ticks, 4 at the least.
  const double fhi = std::min(625.0, 1.0 / (4.0 * ts)), flo = fhi / 5.0;
  for (int i = 0; i < kMaxPoints; ++i) {
    const double f = flo * std::pow(fhi / flo, i / (kMaxPoints - 1.0));
    const long n = std::max(4L, std::lround(1.0 / (ts * f)));
    const std::uint32_t period = static_cast<std::uint32_t>(n) * s.tick_us;
    if (count_ > 0 && period_us_[count_ - 1] == period) continue;
    period_us_[count_] = period;
    const double cycles = std::ceil(s.seconds / (period * 1e-6));
    measure_us_[count_] = static_cast<std::uint32_t>(cycles) * period;
    // The voltage that turns the shaft by the amplitude, by the
    // configuration: theta = kt u / (R sqrt(1 + (w tau)^2) J w^2).
    const double w = 2.0 * kPiD / (period * 1e-6);
    const double v = static_cast<double>(s.amplitude) * s.resistance *
                     std::sqrt(1.0 + w * w * s.tau * s.tau) * s.inertia * w * w / s.kt;
    volts_[count_] = static_cast<float>(std::min(v, static_cast<double>(s.voltage_limit)));
    ++count_;
  }
  t0_ = now;
  std::fill(&gram_[0][0], &gram_[0][0] + 16, 0.0);
  std::fill(rhs_, rhs_ + 4, 0.0);
  sq_ = 0.0;
  samples_ = 0;
}

namespace {
// Of each point: the ramps' periods, up and down, and the periods before
// the measurement. Without the ramp down, the shaft kept the velocity it
// had when the voltage stopped, and wandered 0.15 rad from where it began.
constexpr std::uint32_t kRampPeriods = 5;
constexpr std::uint32_t kSettlePeriods = 20;
}  // namespace

std::uint32_t WindingIdent::duration_us() const {
  std::uint32_t t = 0;
  for (int i = 0; i < count_; ++i) t += (kSettlePeriods + kRampPeriods) * period_us_[i] + measure_us_[i];
  return t;
}

float WindingIdent::step(std::uint32_t now, double theta) {
  if (!running()) return 0.0f;
  if (steps_ == 0) home_ = theta;
  if (now - last_ > s_.tick_us + s_.tick_us / 2) ++late_;
  last_ = now;
  ++steps_;
  if (std::fabs(theta - home_) > kWander) {
    wandered_ = true;
    return 0.0f;
  }
  std::uint32_t t = now - t0_;
  const std::uint32_t period = period_us_[point_];
  const std::uint32_t settle = kSettlePeriods * period;
  const std::uint32_t ramp = kRampPeriods * period;
  const std::uint32_t end = settle + measure_us_[point_];
  if (t >= end + ramp) {
    close_point();
    ++point_;
    t0_ = now;
    std::fill(&gram_[0][0], &gram_[0][0] + 16, 0.0);
    std::fill(rhs_, rhs_ + 4, 0.0);
    sq_ = 0.0;
    samples_ = 0;
    return 0.0f;  // the next point's ramp starts from 0
  }
  const double phase = 2.0 * kPiD * static_cast<double>(t % period) / static_cast<double>(period);
  const double c = std::cos(phase), s = std::sin(phase);
  double env = 1.0;
  if (t < ramp) env = 0.5 * (1.0 - std::cos(kPiD * static_cast<double>(t) / static_cast<double>(ramp)));
  if (t >= end) env = 0.5 * (1.0 + std::cos(kPiD * static_cast<double>(t - end) / static_cast<double>(ramp)));
  if (t >= settle && t < end) {
    if (samples_ == 0) theta0_ = theta;
    const double y = theta - theta0_;
    const double basis[4] = {1.0, static_cast<double>(t - settle) / static_cast<double>(measure_us_[point_]), c, s};
    for (int u = 0; u < 4; ++u) {
      rhs_[u] += basis[u] * y;
      for (int v = u; v < 4; ++v) gram_[u][v] += basis[u] * basis[v];
    }
    sq_ += y * y;
    ++samples_;
  }
  return static_cast<float>(volts_[point_] * env * s);
}

void WindingIdent::close_point() {
  WindingPoint& p = pts_[point_];
  const double period = period_us_[point_] * 1e-6;
  p.w = 2.0 * kPiD / period;
  p.g = 0.0;
  p.amp = 0.0;
  p.rel = 0.0;
  if (samples_ <= 8) return;
  double a[16], b[4];
  for (int u = 0; u < 4; ++u) {
    b[u] = rhs_[u];
    for (int v = 0; v < 4; ++v) a[u * 4 + v] = u <= v ? gram_[u][v] : gram_[v][u];
  }
  if (!solve(a, b, 4)) return;
  // What the line and the fundamental leave: the readings' noise, and
  // whatever else moved the shaft.
  double left = sq_;
  for (int u = 0; u < 4; ++u) left -= b[u] * rhs_[u];
  const double sd = std::sqrt(std::max(left, 0.0) / static_cast<double>(samples_ - 4));
  // theta = C cos + S sin = Re((C - jS) e^{j phase}); u = V sin = Re(-jV e^{j phase}).
  const cd x(b[2], -b[3]);
  const cd u(0.0, -static_cast<double>(volts_[point_]));
  // The voltage held over a tick: its fundamental is sinc(w ts / 2) e^{-j w ts / 2} of it.
  const double h = 0.5 * p.w * s_.tick_us * 1e-6;
  const cd held = std::sin(h) / h * std::exp(cd(0.0, -h));
  if (std::abs(x) > 0.0) {
    p.g = x / (u * held);
    p.amp = std::abs(x);
    p.rel = sd * std::sqrt(2.0 / static_cast<double>(samples_)) / std::abs(x);
  }
}

WindingFit WindingIdent::fit() const {
  if (wandered_) {
    WindingFit f;
    f.why = "shaft wandered off";
    return f;
  }
  const double ts = s_.tick_us * 1e-6;
  WindingFit f = fit_windings(pts_, point_, ts, std::max(0.0, static_cast<double>(s_.duty_delay)) + ts);
  // Late ticks first: what else failed may have been their doing.
  if (late_ > kLateShare * steps_) {
    f.ok = false;
    f.why = "ticks missed";
  }
  if (!(s_.resistance > 0.0f && s_.kt > 0.0f && s_.inertia > 0.0f)) return f;
  f.inertia = f.p2 * s_.kt / s_.resistance;
  f.damping = (f.p1 - s_.back_emf) * s_.kt / s_.resistance;
  if (!f.ok) return f;
  if (f.inertia < 0.5 * s_.inertia || f.inertia > 2.0 * s_.inertia) {
    f.ok = false;
    f.why = "inertia not the shaft's alone";
  } else if (f.tau * f.damping / f.inertia > 0.02) {
    f.ok = false;
    f.why = "shaft damped: held?";
  }
  return f;
}

}  // namespace tuushin::fw
