#include <algorithm>
#include <cmath>

#include "test.hpp"
#include "tuushin/control/tdpa.hpp"
#include "tuushin/rng.hpp"

using tuushin::Rng;
using tuushin::control::EnergyPort;
using tuushin::control::passivate_force;
using tuushin::control::passivate_velocity;

TEST_CASE(tdpa_energy_books_split_by_sign) {
  EnergyPort p;
  p.account(2.0, 0.5);   // +1 J in
  p.account(-4.0, 0.5);  // +2 J out
  p.account(0.0, 0.5);
  CHECK_NEAR(p.input_energy(), 1.0, 1e-15);
  CHECK_NEAR(p.output_energy(), 2.0, 1e-15);
}

TEST_CASE(tdpa_remote_energy_is_monotonic) {
  EnergyPort p;
  p.on_remote_input_energy(3.0);
  p.on_remote_input_energy(1.0);  // stale / reordered report
  CHECK_NEAR(p.remote_input_energy(), 3.0, 0.0);
}

TEST_CASE(tdpa_input_power_passes_unchanged) {
  EnergyPort p;
  const auto r = passivate_force(p, 2.0, 1.5, 0.001);  // f*v > 0
  CHECK(!r.active);
  CHECK_NEAR(r.value, 2.0, 0.0);
  CHECK_NEAR(p.input_energy(), 3.0e-3, 1e-15);
}

TEST_CASE(tdpa_force_pc_blocks_output_without_budget) {
  EnergyPort p;  // the peer has not reported any input energy yet
  const auto r = passivate_force(p, 5.0, -0.2, 0.001);  // would output energy
  CHECK(r.active);
  CHECK_NEAR(r.value, 0.0, 1e-15);
  CHECK(r.gain > 0.0);
  CHECK_NEAR(p.output_energy(), 0.0, 1e-15);
}

TEST_CASE(tdpa_force_pc_spends_exactly_the_budget) {
  EnergyPort p;
  p.on_remote_input_energy(1e-4);
  // Wants 5 * 0.2 * 1e-3 = 1e-3 J out; only 1e-4 J available.
  const auto r = passivate_force(p, 5.0, -0.2, 0.001);
  CHECK(r.active);
  CHECK_NEAR(p.output_energy(), 1e-4, 1e-18);
  CHECK_NEAR(p.budget(), 0.0, 1e-18);
  // f' = f + alpha * v must hold for the reported alpha.
  CHECK_NEAR(r.value, 5.0 + r.gain * -0.2, 1e-12);
  CHECK_NEAR(r.value, 0.5, 1e-12);
}

TEST_CASE(tdpa_velocity_pc_spends_exactly_the_budget) {
  EnergyPort p;
  p.on_remote_input_energy(2e-4);
  // Power into channel = -f*v = -(4)(0.25) = -1 W -> 1e-3 J out wanted.
  const auto r = passivate_velocity(p, 0.25, 4.0, 0.001);
  CHECK(r.active);
  CHECK_NEAR(p.output_energy(), 2e-4, 1e-18);
  CHECK_NEAR(r.value, 0.25 - r.gain * 4.0, 1e-12);
  CHECK_NEAR(r.value, 0.05, 1e-12);
}

// Property: whatever the signals and however late the peer's reports, the
// output energy never exceeds the largest input energy reported so far.
TEST_CASE(tdpa_output_never_exceeds_reported_input) {
  Rng rng(42);
  EnergyPort p;
  double peer_in = 0.0;
  for (int k = 0; k < 200000; ++k) {
    if (rng.bernoulli(0.3)) {
      peer_in += rng.exponential(1e-4);
      // Deliver a possibly stale value: sometimes the current, sometimes older.
      p.on_remote_input_energy(rng.bernoulli(0.5) ? peer_in : peer_in * rng.uniform());
    }
    const double f = rng.normal(0.0, 5.0);
    const double v = rng.normal(0.0, 0.3);
    if (k % 2 == 0) {
      passivate_force(p, f, v, 0.001);
    } else {
      passivate_velocity(p, v, f, 0.001);
    }
    if (p.output_energy() > p.remote_input_energy() + 1e-12) {
      CHECK(p.output_energy() <= p.remote_input_energy() + 1e-12);
      return;
    }
  }
  CHECK(p.output_energy() <= peer_in + 1e-12);
}

TEST_CASE(tdpa_port_pays_its_own_excess_before_reporting) {
  EnergyPort p;
  p.on_remote_input_energy(1e-3);
  p.account_energy(-1.5e-3);  // realized output beyond the peer's input
  CHECK_NEAR(p.budget(), 0.0, 0.0);
  p.account_energy(2e-4);  // pays part of the 5e-4 owed
  CHECK_NEAR(p.covered_energy(), 2e-4, 1e-18);
  CHECK_NEAR(p.reported_input_energy(), 0.0, 1e-18);
  p.account_energy(5e-4);  // pays the rest; 2e-4 is left to report
  CHECK_NEAR(p.covered_energy(), 5e-4, 1e-18);
  CHECK_NEAR(p.reported_input_energy(), 2e-4, 1e-18);
  CHECK_NEAR(p.input_energy(), 7e-4, 1e-18);
  CHECK_NEAR(p.budget(), 0.0, 1e-18);
  // Later input from the peer is this port's to spend again, and what was
  // reported stays reported.
  p.on_remote_input_energy(2e-3);
  CHECK_NEAR(p.budget(), 1e-3, 1e-18);
  p.account_energy(3e-4);
  CHECK_NEAR(p.reported_input_energy(), 5e-4, 1e-18);
}

// Property: two ports that book realized work, each output limited to the
// port's budget plus a random overshoot (a prediction error), exchanging
// their reports over a lossy, late channel. What each reports only grows;
// an input pays the port's own debt before anything is reported; and the
// two ends' outputs never exceed their inputs by more than the debts not
// yet paid. The last holds only because covered input is both added to the
// budget and withheld from the report.
TEST_CASE(tdpa_own_cover_keeps_the_channel_books_whole) {
  Rng rng(7);
  EnergyPort a, b;
  const auto debt = [](const EnergyPort& p) {
    return std::max(0.0, p.output_energy() - p.remote_input_energy() - p.covered_energy());
  };
  for (int k = 0; k < 200000; ++k) {
    for (EnergyPort* p : {&a, &b}) {
      const double reported = p->reported_input_energy();
      if (rng.bernoulli(0.5)) {
        const double owed = debt(*p);
        const double e = rng.exponential(1e-4);
        p->account_energy(e);
        const double paid = std::min(e, owed);
        if (std::fabs(p->reported_input_energy() - reported - (e - paid)) > 1e-15 ||
            debt(*p) > owed - paid + 1e-15) {
          CHECK_NEAR(p->reported_input_energy() - reported, e - paid, 1e-15);
          CHECK(debt(*p) <= owed - paid + 1e-15);
          return;
        }
      } else {
        p->account_energy(-(rng.uniform() * p->budget() + rng.exponential(2e-5)));
        if (p->reported_input_energy() != reported) {
          CHECK(p->reported_input_energy() == reported);
          return;
        }
      }
    }
    if (rng.bernoulli(0.2)) a.on_remote_input_energy(b.reported_input_energy() * rng.uniform());
    if (rng.bernoulli(0.2)) b.on_remote_input_energy(a.reported_input_energy() * rng.uniform());
    const double net = a.input_energy() - a.output_energy() + b.input_energy() - b.output_energy();
    if (net < -debt(a) - debt(b) - 1e-12) {
      CHECK(net >= -debt(a) - debt(b) - 1e-12);
      return;
    }
  }
}
