#pragma once

#include <cstddef>
#include <vector>

namespace tuushin::control {

// A small dense linear program with bounded variables,
//
//   minimize c x   subject to   A x = b,   lower <= x <= upper,
//
// solved by the primal simplex method for bounded variables (G. B. Dantzig,
// "Upper bounds, secondary constraints, and block triangularity in linear
// programming", Econometrica 23(2), 1955; the textbook form is in
// V. Chvatal, "Linear Programming", 1983, ch. 8). Bounds may be infinite, so
// a variable may be free; a nonbasic variable may sit anywhere within its
// bounds, not only at one of them, which lets a caller start it at a guess.
//
// Why a solver of its own: the problems are tiny (tens of rows) and solved
// many times a second, and the libraries that would do (GLPK, lp_solve) are
// GPL or LGPL, which this project does not take. HiGHS is MIT but large.
//
// Method. Phase 1 starts from a diagonal basis: a row whose column is a
// singleton (nonzero in that row alone, as a slack is) and whose value the
// row then implies lies within its bounds takes that column; every other
// row an artificial variable signed so that it starts nonnegative. Phase 1
// minimizes the artificials' sum; at zero they are fixed at zero and pivoted
// out where a structural column can replace them. Each minimize() is then a
// phase 2 from wherever the last one ended, so a sequence of objectives
// over the same constraints shares phase 1. The entering variable is the
// one of steepest reduced cost (Dantzig's rule) until a run of degenerate
// pivots, then the lowest index (R. G. Bland, "New finite pivoting rules for
// the simplex method", Math. Oper. Res. 2(2), 1977), which cannot cycle.
// The ratio test includes the entering variable's own bound, a bound flip
// that needs no pivot. The tableau carries the inverse of the basis in its
// artificial columns, from which the basic values are recomputed from the
// original data after each phase, so that rounding does not accumulate in
// them.
class BoundedLp {
 public:
  BoundedLp(std::size_t rows, std::size_t cols);

  std::size_t rows() const { return m_; }
  std::size_t cols() const { return n_; }

  // The problem. Every coefficient and right-hand side starts at zero, every
  // variable at [0, +inf) with a start of 0.
  void set(std::size_t row, std::size_t col, double a) { a_[row * n_ + col] = a; }
  void set_rhs(std::size_t row, double b) { b_[row] = b; }
  void set_bounds(std::size_t col, double lower, double upper);
  // Where phase 1 starts the variable (clamped to its bounds).
  void set_start(std::size_t col, double x) { start_[col] = x; }

  // Phase 1: finds a feasible point. False if there is none (within
  // tolerance) or the iteration limit was reached; minimize() then fails
  // too. Changing the problem afterwards needs another call.
  bool solve_feasible();
  // Phase 2 for the objective c (one entry per column), from the point the
  // last phase ended at. Writes the optimum to `value` and returns true;
  // false if the objective is unbounded below, the iteration limit was
  // reached, or there is no feasible point.
  bool minimize(const std::vector<double>& c, double& value);

  // The current point: after solve_feasible() a feasible one, after
  // minimize() an optimal one.
  double value(std::size_t col) const { return x_[col]; }

 private:
  enum class Rule { kDantzig, kBland };
  // Runs the simplex on cost_ until optimal; false if unbounded or out of
  // iterations.
  bool iterate();
  void pivot(std::size_t row, std::size_t col);
  // Basic values from the nonbasic ones and the original data.
  void refresh_basics();

  std::size_t m_, n_;  // rows, structural columns
  std::size_t w_;      // tableau width: n_ structural + m_ artificial
  std::vector<double> a_, b_;
  std::vector<double> start_;
  // Over all w_ columns.
  std::vector<double> lower_, upper_, x_, cost_;
  std::vector<double> t_;               // m_ x w_: the basis' inverse times [A | D]
  std::vector<double> sign_;            // D: the artificials' signs
  std::vector<std::size_t> basis_;      // the basic column of each row
  std::vector<int> row_of_;             // the row a column is basic in, or -1
  bool feasible_ = false;
};

}  // namespace tuushin::control
