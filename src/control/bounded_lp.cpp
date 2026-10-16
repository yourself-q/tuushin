#include "tuushin/control/bounded_lp.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace tuushin::control {

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();
// Tableau entries smaller than this count as zero in the ratio test.
constexpr double kPivotTol = 1e-9;
// How far a nonbasic variable must be from a bound to move towards it, and
// reduced costs smaller than this count as zero.
constexpr double kBoundTol = 1e-12;
constexpr double kCostTol = 1e-9;
// Phase 1 is feasible when the artificials sum to no more than this times
// the size of the rows' terms.
constexpr double kFeasibleTol = 1e-9;
// Steps shorter than this are degenerate; after this many of them in a row
// the entering variable is chosen by Bland's rule.
constexpr double kDegenerateStep = 1e-12;
constexpr int kDegenerateRun = 16;
// Basic values are recomputed from the original data this often.
constexpr int kRefreshPivots = 64;

}  // namespace

BoundedLp::BoundedLp(std::size_t rows, std::size_t cols)
    : m_(rows),
      n_(cols),
      w_(rows + cols),
      a_(rows * cols, 0.0),
      b_(rows, 0.0),
      start_(cols, 0.0),
      lower_(w_, 0.0),
      upper_(w_, kInf),
      x_(w_, 0.0),
      cost_(w_, 0.0),
      t_(rows * w_, 0.0),
      sign_(rows, 1.0),
      basis_(rows, 0),
      row_of_(w_, -1) {}

void BoundedLp::set_bounds(std::size_t col, double lower, double upper) {
  if (!(lower <= upper)) throw std::invalid_argument("BoundedLp: lower bound above upper bound");
  lower_[col] = lower;
  upper_[col] = upper;
}

bool BoundedLp::solve_feasible() {
  feasible_ = false;
  for (std::size_t j = 0; j < n_; ++j) {
    x_[j] = std::clamp(start_[j], lower_[j], upper_[j]);
    row_of_[j] = -1;
  }
  std::vector<double> r(b_);
  double scale = 0.0;
  for (std::size_t i = 0; i < m_; ++i) {
    double size = std::fabs(b_[i]);
    for (std::size_t j = 0; j < n_; ++j) {
      r[i] -= a_[i * n_ + j] * x_[j];
      size += std::fabs(a_[i * n_ + j] * x_[j]);
    }
    scale = std::max(scale, size);
  }

  // The starting basis: per row a singleton column that can make it hold,
  // or an artificial.
  std::vector<double> beta(m_, 0.0);
  for (std::size_t i = 0; i < m_; ++i) basis_[i] = n_ + i;
  for (std::size_t j = 0; j < n_; ++j) {
    if (!(lower_[j] < upper_[j])) continue;
    std::size_t row = m_, count = 0;
    for (std::size_t i = 0; i < m_ && count < 2; ++i) {
      if (a_[i * n_ + j] != 0.0) {
        row = i;
        ++count;
      }
    }
    if (count != 1 || basis_[row] != n_ + row) continue;
    const double aij = a_[row * n_ + j];
    const double xj = x_[j] + r[row] / aij;
    if (xj < lower_[j] - kFeasibleTol * (1.0 + std::fabs(xj)) ||
        xj > upper_[j] + kFeasibleTol * (1.0 + std::fabs(xj))) {
      continue;
    }
    x_[j] = std::clamp(xj, lower_[j], upper_[j]);
    r[row] = 0.0;
    basis_[row] = j;
    beta[row] = aij;
  }
  for (std::size_t i = 0; i < m_; ++i) {
    const std::size_t art = n_ + i;
    row_of_[art] = -1;
    cost_[art] = 1.0;
    if (basis_[i] == art) {
      sign_[i] = r[i] >= 0.0 ? 1.0 : -1.0;
      beta[i] = sign_[i];
      lower_[art] = 0.0;
      upper_[art] = kInf;
      x_[art] = std::fabs(r[i]);
    } else {
      sign_[i] = 1.0;
      lower_[art] = upper_[art] = x_[art] = 0.0;
    }
    row_of_[basis_[i]] = static_cast<int>(i);
  }
  for (std::size_t j = 0; j < n_; ++j) cost_[j] = 0.0;
  // The basis is diagonal: its inverse scales each row.
  std::fill(t_.begin(), t_.end(), 0.0);
  for (std::size_t i = 0; i < m_; ++i) {
    double* row = &t_[i * w_];
    for (std::size_t j = 0; j < n_; ++j) row[j] = a_[i * n_ + j] / beta[i];
    row[n_ + i] = sign_[i] / beta[i];
  }

  if (!iterate()) return false;
  refresh_basics();
  double infeasibility = 0.0;
  for (std::size_t i = 0; i < m_; ++i) infeasibility += std::fabs(x_[n_ + i]);
  if (infeasibility > kFeasibleTol * (1.0 + scale)) return false;

  // Fix the artificials at zero and replace those still basic where a
  // structural column can take their row; a row none can take is
  // redundant, and its artificial stays basic at zero.
  for (std::size_t i = 0; i < m_; ++i) {
    const std::size_t art = n_ + i;
    lower_[art] = upper_[art] = 0.0;
    cost_[art] = 0.0;
    if (row_of_[art] < 0) x_[art] = 0.0;
  }
  for (std::size_t i = 0; i < m_; ++i) {
    if (basis_[i] < n_) continue;
    std::size_t best = n_;
    double most = kPivotTol;
    for (std::size_t j = 0; j < n_; ++j) {
      if (row_of_[j] >= 0) continue;
      const double tij = std::fabs(t_[i * w_ + j]);
      if (tij > most) {
        most = tij;
        best = j;
      }
    }
    if (best < n_) {
      const std::size_t art = basis_[i];
      pivot(i, best);
      x_[art] = 0.0;
    }
  }
  refresh_basics();
  feasible_ = true;
  return true;
}

bool BoundedLp::minimize(const std::vector<double>& c, double& value) {
  if (!feasible_ || c.size() != n_) return false;
  for (std::size_t j = 0; j < n_; ++j) cost_[j] = c[j];
  for (std::size_t j = n_; j < w_; ++j) cost_[j] = 0.0;
  const bool ok = iterate();
  refresh_basics();
  value = 0.0;
  for (std::size_t j = 0; j < n_; ++j) value += c[j] * x_[j];
  return ok;
}

bool BoundedLp::iterate() {
  Rule rule = Rule::kDantzig;
  int degenerate = 0, since_refresh = 0;
  const std::size_t limit = 50 * (m_ + w_) + 100;
  std::vector<double> d(w_);
  for (std::size_t iter = 0; iter < limit; ++iter) {
    // Reduced costs d = c - c_B T.
    d = cost_;
    for (std::size_t i = 0; i < m_; ++i) {
      const double cb = cost_[basis_[i]];
      if (cb == 0.0) continue;
      const double* row = &t_[i * w_];
      for (std::size_t j = 0; j < w_; ++j) d[j] -= cb * row[j];
    }
    // Entering: a nonbasic variable whose move lowers the cost.
    std::size_t enter = w_;
    double dir = 0.0, steepest = 0.0;
    for (std::size_t j = 0; j < w_; ++j) {
      if (row_of_[j] >= 0) continue;
      double s = 0.0;
      if (d[j] < -kCostTol && x_[j] < upper_[j] - kBoundTol) s = 1.0;
      else if (d[j] > kCostTol && x_[j] > lower_[j] + kBoundTol) s = -1.0;
      if (s == 0.0) continue;
      if (rule == Rule::kBland) {
        enter = j;
        dir = s;
        break;
      }
      if (std::fabs(d[j]) > steepest) {
        steepest = std::fabs(d[j]);
        enter = j;
        dir = s;
      }
    }
    if (enter == w_) return true;

    // Ratio test: how far it can move before it or a basic variable meets a
    // bound.
    double step = dir > 0.0 ? upper_[enter] - x_[enter] : x_[enter] - lower_[enter];
    std::size_t leave = m_;
    double leave_piv = 0.0;
    for (std::size_t i = 0; i < m_; ++i) {
      const double tij = t_[i * w_ + enter];
      if (std::fabs(tij) <= kPivotTol) continue;
      const double rate = -dir * tij;  // of the basic variable per unit step
      const std::size_t k = basis_[i];
      double lim;
      if (rate < 0.0) {
        if (lower_[k] == -kInf) continue;
        lim = std::max(0.0, x_[k] - lower_[k]) / -rate;
      } else {
        if (upper_[k] == kInf) continue;
        lim = std::max(0.0, upper_[k] - x_[k]) / rate;
      }
      bool take = lim < step;
      if (!take && leave < m_ && lim <= step) {
        // A tie: the larger pivot is the stabler one; under Bland's rule
        // the lower index.
        take = rule == Rule::kBland ? k < basis_[leave] : std::fabs(tij) > leave_piv;
      }
      if (take) {
        step = lim;
        leave = i;
        leave_piv = std::fabs(tij);
      }
    }
    if (step == kInf) return false;  // unbounded

    x_[enter] += dir * step;
    for (std::size_t i = 0; i < m_; ++i) x_[basis_[i]] -= dir * t_[i * w_ + enter] * step;
    if (leave < m_) {
      // The leaving variable sits exactly at the bound it met.
      const std::size_t k = basis_[leave];
      x_[k] = -dir * t_[leave * w_ + enter] < 0.0 ? lower_[k] : upper_[k];
      pivot(leave, enter);
      if (++since_refresh >= kRefreshPivots) {
        refresh_basics();
        since_refresh = 0;
      }
    }
    degenerate = step <= kDegenerateStep ? degenerate + 1 : 0;
    if (degenerate >= kDegenerateRun) rule = Rule::kBland;
  }
  return false;
}

void BoundedLp::pivot(std::size_t r, std::size_t j) {
  double* pr = &t_[r * w_];
  const double p = pr[j];
  for (std::size_t k = 0; k < w_; ++k) pr[k] /= p;
  pr[j] = 1.0;
  for (std::size_t i = 0; i < m_; ++i) {
    if (i == r) continue;
    double* row = &t_[i * w_];
    const double f = row[j];
    if (f == 0.0) continue;
    for (std::size_t k = 0; k < w_; ++k) row[k] -= f * pr[k];
    row[j] = 0.0;
  }
  row_of_[basis_[r]] = -1;
  basis_[r] = j;
  row_of_[j] = static_cast<int>(r);
}

void BoundedLp::refresh_basics() {
  // x_B = B^-1 (b - N x_N); the artificial columns hold B^-1 D, and D is
  // its own inverse.
  std::vector<double> rhs(b_);
  for (std::size_t j = 0; j < n_; ++j) {
    if (row_of_[j] >= 0 || x_[j] == 0.0) continue;
    for (std::size_t i = 0; i < m_; ++i) rhs[i] -= a_[i * n_ + j] * x_[j];
  }
  for (std::size_t i = 0; i < m_; ++i) {
    if (row_of_[n_ + i] < 0) rhs[i] -= sign_[i] * x_[n_ + i];
    rhs[i] *= sign_[i];
  }
  for (std::size_t r = 0; r < m_; ++r) {
    const double* row = &t_[r * w_ + n_];
    double v = 0.0;
    for (std::size_t i = 0; i < m_; ++i) v += row[i] * rhs[i];
    x_[basis_[r]] = v;
  }
}

}  // namespace tuushin::control
