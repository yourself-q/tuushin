#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "test.hpp"
#include "tuushin/control/bounded_lp.hpp"
#include "tuushin/rng.hpp"

using tuushin::Rng;
using tuushin::control::BoundedLp;

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

double uniform(Rng& rng, double lo, double hi) { return lo + (hi - lo) * rng.uniform(); }

}  // namespace

TEST_CASE(bounded_lp_solves_a_textbook_problem) {
  // Maximize x + y with x + 2y <= 4, 3x + y <= 6, x, y >= 0, slacks s1, s2.
  BoundedLp lp(2, 4);
  lp.set(0, 0, 1.0);
  lp.set(0, 1, 2.0);
  lp.set(0, 2, 1.0);
  lp.set(1, 0, 3.0);
  lp.set(1, 1, 1.0);
  lp.set(1, 3, 1.0);
  lp.set_rhs(0, 4.0);
  lp.set_rhs(1, 6.0);
  REQUIRE(lp.solve_feasible());
  double v = 0.0;
  REQUIRE(lp.minimize({-1.0, -1.0, 0.0, 0.0}, v));
  CHECK_NEAR(v, -2.8, 1e-12);
  CHECK_NEAR(lp.value(0), 1.6, 1e-12);
  CHECK_NEAR(lp.value(1), 1.2, 1e-12);
  // Another objective from the same phase 1.
  REQUIRE(lp.minimize({-1.0, 0.0, 0.0, 0.0}, v));
  CHECK_NEAR(v, -2.0, 1e-12);
  REQUIRE(lp.minimize({0.0, 1.0, 0.0, 0.0}, v));
  CHECK_NEAR(v, 0.0, 1e-12);
}

TEST_CASE(bounded_lp_handles_free_variables_and_equalities) {
  // x + y = 1 and x - y = 0 with x, y free: one point.
  BoundedLp lp(2, 2);
  lp.set(0, 0, 1.0);
  lp.set(0, 1, 1.0);
  lp.set(1, 0, 1.0);
  lp.set(1, 1, -1.0);
  lp.set_rhs(0, 1.0);
  lp.set_bounds(0, -kInf, kInf);
  lp.set_bounds(1, -kInf, kInf);
  lp.set_start(0, 7.0);
  REQUIRE(lp.solve_feasible());
  double lo = 0.0, hi = 0.0;
  REQUIRE(lp.minimize({1.0, 0.0}, lo));
  REQUIRE(lp.minimize({-1.0, 0.0}, hi));
  CHECK_NEAR(lo, 0.5, 1e-12);
  CHECK_NEAR(-hi, 0.5, 1e-12);
}

TEST_CASE(bounded_lp_tolerates_a_redundant_row) {
  BoundedLp lp(2, 2);
  for (std::size_t i = 0; i < 2; ++i) {
    lp.set(i, 0, 1.0);
    lp.set(i, 1, 1.0);
    lp.set_rhs(i, 1.0);
  }
  lp.set_bounds(0, 0.0, 0.75);
  REQUIRE(lp.solve_feasible());
  double v = 0.0;
  REQUIRE(lp.minimize({0.0, 1.0}, v));
  CHECK_NEAR(v, 0.25, 1e-12);
  CHECK_NEAR(lp.value(0) + lp.value(1), 1.0, 1e-12);
}

TEST_CASE(bounded_lp_reports_infeasible_and_unbounded_problems) {
  {
    BoundedLp lp(1, 2);
    lp.set(0, 0, 1.0);
    lp.set(0, 1, 1.0);
    lp.set_rhs(0, 5.0);
    lp.set_bounds(0, 0.0, 2.0);
    lp.set_bounds(1, 0.0, 2.0);
    CHECK(!lp.solve_feasible());
    double v = 0.0;
    CHECK(!lp.minimize({1.0, 0.0}, v));
  }
  {
    // x - y = 0, y >= 0, x free: x is unbounded above.
    BoundedLp lp(1, 2);
    lp.set(0, 0, 1.0);
    lp.set(0, 1, -1.0);
    lp.set_bounds(0, -kInf, kInf);
    REQUIRE(lp.solve_feasible());
    double v = 0.0;
    CHECK(!lp.minimize({-1.0, 0.0}, v));
    CHECK(lp.minimize({1.0, 0.0}, v));
    CHECK_NEAR(v, 0.0, 1e-12);
  }
}

TEST_CASE(bounded_lp_does_not_cycle_on_beales_example) {
  // E. M. L. Beale, "Cycling in the dual simplex algorithm", Naval Res.
  // Logist. Q. 2(4), 1955: the textbook rule cycles from the slack basis.
  // Columns x1..x3 are the slacks, x4..x7 the variables; optimum -5/4.
  BoundedLp lp(3, 7);
  const double rows[3][7] = {{1, 0, 0, 0.25, -8, -1, 9},
                             {0, 1, 0, 0.5, -12, -0.5, 3},
                             {0, 0, 1, 0, 0, 1, 0}};
  for (std::size_t i = 0; i < 3; ++i) {
    for (std::size_t j = 0; j < 7; ++j) lp.set(i, j, rows[i][j]);
  }
  lp.set_rhs(2, 1.0);
  REQUIRE(lp.solve_feasible());
  double v = 0.0;
  REQUIRE(lp.minimize({0, 0, 0, -0.75, 20, -0.5, 6}, v));
  CHECK_NEAR(v, -1.25, 1e-12);
}

TEST_CASE(bounded_lp_matches_vertex_enumeration_on_random_problems) {
  // Two free variables z in a box, and ranged rows lo <= g z + k <= hi as
  // a slack each: g z - s = -k, s in [lo, hi]. The optimum of a linear
  // objective lies on a vertex: where two of the lines meet.
  Rng rng(7);
  int feasible = 0;
  for (int trial = 0; trial < 300; ++trial) {
    const std::size_t m = 2 + trial % 6;
    struct Line {
      double g0, g1, lo, hi;
    };
    std::vector<Line> lines;
    // The box, as rows too.
    lines.push_back({1.0, 0.0, -10.0, 10.0});
    lines.push_back({0.0, 1.0, -10.0, 10.0});
    BoundedLp lp(m, 2 + m);
    lp.set_bounds(0, -10.0, 10.0);
    lp.set_bounds(1, -10.0, 10.0);
    lp.set_start(0, uniform(rng, -10.0, 10.0));
    lp.set_start(1, uniform(rng, -10.0, 10.0));
    for (std::size_t i = 0; i < m; ++i) {
      const double g0 = uniform(rng, -1.0, 1.0), g1 = uniform(rng, -1.0, 1.0);
      const double k = uniform(rng, -3.0, 3.0), width = uniform(rng, 0.1, 4.0);
      lp.set(i, 0, g0);
      lp.set(i, 1, g1);
      lp.set(i, 2 + i, -1.0);
      lp.set_rhs(i, -k);
      lp.set_bounds(2 + i, 0.0, width);
      lines.push_back({g0, g1, -k, width - k});
    }
    const double c0 = uniform(rng, -1.0, 1.0), c1 = uniform(rng, -1.0, 1.0);

    double best = kInf;
    auto inside = [&](double z0, double z1) {
      for (const auto& l : lines) {
        const double y = l.g0 * z0 + l.g1 * z1;
        if (y < l.lo - 1e-9 || y > l.hi + 1e-9) return false;
      }
      return true;
    };
    for (std::size_t a = 0; a < lines.size(); ++a) {
      for (std::size_t b = a + 1; b < lines.size(); ++b) {
        const double det = lines[a].g0 * lines[b].g1 - lines[a].g1 * lines[b].g0;
        if (std::fabs(det) < 1e-12) continue;
        for (double ya : {lines[a].lo, lines[a].hi}) {
          for (double yb : {lines[b].lo, lines[b].hi}) {
            const double z0 = (ya * lines[b].g1 - yb * lines[a].g1) / det;
            const double z1 = (lines[a].g0 * yb - lines[b].g0 * ya) / det;
            if (inside(z0, z1)) best = std::min(best, c0 * z0 + c1 * z1);
          }
        }
      }
    }

    std::vector<double> c(2 + m, 0.0);
    c[0] = c0;
    c[1] = c1;
    double v = 0.0;
    if (best == kInf) {
      CHECK(!lp.solve_feasible());
      continue;
    }
    ++feasible;
    REQUIRE(lp.solve_feasible());
    REQUIRE(lp.minimize(c, v));
    CHECK_NEAR(v, best, 1e-9);
    CHECK(inside(lp.value(0), lp.value(1)));
  }
  CHECK(feasible > 50);
}
