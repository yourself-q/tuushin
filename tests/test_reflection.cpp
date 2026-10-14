#include <algorithm>
#include <cmath>
#include <vector>

#include "test.hpp"
#include "tuushin/control/predictive_pc.hpp"
#include "tuushin/control/reflection.hpp"
#include "tuushin/rng.hpp"

using tuushin::Rng;
using tuushin::control::DevicePredictor;
using tuushin::control::limit_force_predictive;
using tuushin::control::quantization_tax;
using tuushin::control::ReflectiveStore;

namespace {

// A point mass under a constant external force H and a zero-order-held port
// force phi, integrated in closed form (constant acceleration per sample).
struct HeldMass {
  double m, dt, H;
  double x = 0.0, v = 0.0;
  // Applies phi over one sample and returns the displacement.
  double step(double phi) { return step_for(phi, dt); }
  // The same over a sample of length h.
  double step_for(double phi, double h) {
    const double a = (H - phi) / m;
    const double dx = v * h + 0.5 * a * h * h;
    x += dx;
    v += a * h;
    return dx;
  }
};

}  // namespace

TEST_CASE(predictor_is_exact_for_a_mass_under_held_force) {
  const double m = 0.2, dt = 1e-3;
  HeldMass dev{m, dt, 3.0};
  dev.v = 0.05;
  DevicePredictor pred(m, dt);
  const double phis[] = {0.0, 0.0, 7.0, -2.0, 4.5, 4.5, 0.0, 10.0, -6.0, 1.0};
  // Two samples of history are needed before the prediction is exact.
  pred.observe(dev.step(0.0), 0.0);
  pred.observe(dev.step(0.0), 0.0);
  for (double phi : phis) {
    const double predicted = pred.displacement(phi);
    const double realized = dev.step(phi);
    CHECK_NEAR(predicted, realized, 1e-15);
    CHECK_NEAR(pred.work(phi), phi * realized, 1e-15);
    pred.observe(realized, phi);
  }
}

TEST_CASE(predictor_is_exact_after_samples_of_unequal_length) {
  // A loop that is 0.4 ms late makes one sample 1.4 ms and the next 0.6 ms.
  // Read as two 1 ms samples, the history would put the velocity 40 % low.
  const double m = 0.2, dt = 1e-3;
  HeldMass dev{m, dt, -1.5};
  dev.v = -0.15;
  DevicePredictor pred(m, dt);
  pred.observe(dev.step(0.0), 0.0);
  pred.observe(dev.step(2.0), 2.0);
  const double lengths[] = {1.4e-3, 0.6e-3, 1.0e-3, 0.55e-3, 1.45e-3, 1.2e-3};
  const double phis[] = {-3.0, -15.0, 4.0, 0.0, -8.0, 6.0};
  for (int i = 0; i < 6; ++i) {
    pred.observe(dev.step_for(phis[i], lengths[i]), phis[i], lengths[i]);
    CHECK(pred.ready());
    CHECK_NEAR(pred.external_force(), dev.H, 1e-7);
    CHECK_NEAR(pred.velocity_dt(), dev.v * dt, 1e-15);
    // The coming sample is predicted at the nominal length, held or not.
    HeldMass copy = dev;
    const double phi = -12.0;
    const double one = copy.step(phi);
    CHECK_NEAR(pred.displacement(phi), one, 1e-15);
    CHECK_NEAR(pred.hold_displacement(phi, 2), one + copy.step(phi), 1e-15);
  }
  // Equal lengths reproduce the recursion of the class comment,
  // dx_k = 2 dx_{k-1} - dx_{k-2} + c phi_{k-2} - c phi_k.
  DevicePredictor a(m, dt), b(m, dt);
  const double c = dt * dt / (2.0 * m);
  const double dx2 = dev.step(-2.0), dx1 = dev.step(3.0);
  a.observe(dx2, -2.0);
  a.observe(dx1, 3.0);
  b.observe(dx2, -2.0, dt);
  b.observe(dx1, 3.0, dt);
  CHECK(a.base() == b.base());
  CHECK_NEAR(a.base(), 2.0 * dx1 - dx2 + c * -2.0, 1e-18);
}

TEST_CASE(predictor_waits_two_samples_after_an_irregular_one) {
  const double m = 0.2, dt = 1e-3;
  HeldMass dev{m, dt, 3.0};
  dev.v = 0.05;
  for (int kind = 0; kind < 3; ++kind) {
    DevicePredictor pred(m, dt);
    CHECK(pred.ready());  // a new predictor: a device at rest
    pred.observe(dev.step(0.0), 0.0);
    pred.observe(dev.step(0.0), 0.0);
    if (kind == 0) pred.observe(dev.step_for(0.0, 50e-3), 0.0, 50e-3);  // a stall
    if (kind == 1) pred.observe(dev.step_for(0.0, 0.3e-3), 0.0, 0.3e-3);  // too short
    if (kind == 2) pred.observe_irregular(dev.step(0.0), 0.0);  // the force was cut
    CHECK(!pred.ready());
    // Not ready: no force at all, whatever the budget.
    const auto r = limit_force_predictive(1.0, 5.0, pred);
    CHECK(r.active);
    CHECK(r.value == 0.0);
    CHECK(r.gain == 1.0);
    pred.observe(dev.step(0.0), 0.0);
    CHECK(!pred.ready());
    pred.observe(dev.step(0.0), 0.0);
    REQUIRE(pred.ready());
    CHECK_NEAR(pred.displacement(2.0), HeldMass(dev).step(2.0), 1e-15);
  }
}

TEST_CASE(predictive_pc_passes_input_force_unchanged) {
  const double m = 0.2, dt = 1e-3;
  HeldMass dev{m, dt, 5.0};
  DevicePredictor pred(m, dt);
  pred.observe(dev.step(0.0), 0.0);
  pred.observe(dev.step(0.0), 0.0);
  // The hand accelerates the device into a force smaller than its own push:
  // the force only absorbs energy.
  const auto r = limit_force_predictive(0.0, 2.0, pred);
  CHECK(!r.active);
  CHECK_NEAR(r.value, 2.0, 0.0);
  CHECK(pred.work(r.value) > 0.0);
}

TEST_CASE(predictive_pc_holds_a_static_force_without_budget) {
  // At rest under a held force that balances the hand: holding it does no
  // work, raising it would push the hand back.
  const double m = 0.2, dt = 1e-3, held = 4.0;
  HeldMass dev{m, dt, held};
  DevicePredictor pred(m, dt);
  pred.observe(dev.step(held), held);
  pred.observe(dev.step(held), held);
  const auto up = limit_force_predictive(0.0, 9.0, pred);
  CHECK(up.active);
  CHECK_NEAR(up.value, held, 1e-12);
  CHECK_NEAR(pred.work(up.value), 0.0, 1e-18);
  const auto same = limit_force_predictive(0.0, held, pred);
  CHECK(!same.active);
}

TEST_CASE(predictive_pc_spends_exactly_the_budget) {
  const double m = 0.2, dt = 1e-3;
  HeldMass dev{m, dt, 0.0};
  dev.v = -0.02;  // moving out, against the force
  DevicePredictor pred(m, dt);
  pred.observe(dev.step(0.0), 0.0);
  pred.observe(dev.step(0.0), 0.0);
  const double budget = 2e-5;
  const auto r = limit_force_predictive(budget, 6.0, pred, 1);
  CHECK(r.active);
  CHECK(r.value > 0.0 && r.value < 6.0);
  CHECK_NEAR(pred.work(r.value), -budget, 1e-15);
  CHECK_NEAR(r.gain, 1.0 - r.value / 6.0, 1e-15);
  const double realized = dev.step(r.value);
  CHECK_NEAR(r.value * realized, -budget, 1e-15);
}

TEST_CASE(predictor_recovers_external_force_and_velocity) {
  const double m = 0.2, dt = 1e-3;
  HeldMass dev{m, dt, 2.5};
  dev.v = 0.03;
  DevicePredictor pred(m, dt);
  pred.observe(dev.step(1.0), 1.0);
  pred.observe(dev.step(-4.0), -4.0);
  CHECK_NEAR(pred.external_force(), 2.5, 1e-9);
  CHECK_NEAR(pred.velocity_dt(), dev.v * dt, 1e-15);
  CHECK_NEAR(pred.hold_base(1), pred.base(), 1e-15);
  // Three samples with the force held.
  const double phi = 3.0;
  const double predicted = pred.hold_displacement(phi, 3);
  const double realized = dev.step(phi) + dev.step(phi) + dev.step(phi);
  CHECK_NEAR(predicted, realized, 1e-15);
}

TEST_CASE(predictive_pc_stops_the_device_instead_of_bouncing_it) {
  // Without budget, a device running into a large force. Over one sample the
  // affordable force would reverse it; the two-sample hold stops it.
  const double m = 0.2, dt = 1e-3, hand = 2.0;
  HeldMass dev{m, dt, hand};
  dev.v = 0.01;
  DevicePredictor pred(m, dt);
  pred.observe(dev.step(0.0), 0.0);
  pred.observe(dev.step(0.0), 0.0);
  const double v0 = dev.v;
  const auto greedy = limit_force_predictive(0.0, 20.0, pred, 1);
  const auto held = limit_force_predictive(0.0, 20.0, pred, 2);
  CHECK(greedy.active && held.active);
  CHECK_NEAR(greedy.value, hand + 2.0 * m * v0 / dt, 1e-9);
  CHECK_NEAR(held.value, hand + m * v0 / dt, 1e-9);
  HeldMass copy = dev;
  copy.step(greedy.value);
  CHECK_NEAR(copy.v, -v0, 1e-12);  // bounced
  const double dx = dev.step(held.value);
  CHECK_NEAR(dev.v, 0.0, 1e-12);   // stopped
  CHECK(held.value * dx > 0.0);    // and the force absorbed energy
}

TEST_CASE(predictive_pc_never_flips_the_force) {
  const double m = 0.2, dt = 1e-3;
  HeldMass dev{m, dt, 0.0};
  dev.v = -0.5;  // fast, so even a tiny force would cost more than the budget
  DevicePredictor pred(m, dt);
  pred.observe(dev.step(0.0), 0.0);
  pred.observe(dev.step(0.0), 0.0);
  const auto r = limit_force_predictive(0.0, 6.0, pred);
  CHECK(r.active);
  CHECK_NEAR(r.value, 0.0, 0.0);
  CHECK_NEAR(r.gain, 1.0, 0.0);
  const auto neg = limit_force_predictive(-1.0, -3.0, pred);  // negative budget = 0
  CHECK(neg.value <= 0.0 && neg.value >= -3.0);
  CHECK(pred.work(neg.value) >= -1e-18);
}

TEST_CASE(quantization_tax_charges_each_rise_of_the_force_once) {
  const double q = 55e-6;
  CHECK_NEAR(quantization_tax(q, 0.0, 5.0), 5.0 * q, 1e-18);   // from zero
  CHECK_NEAR(quantization_tax(q, 5.0, 5.0), 0.0, 0.0);         // held
  CHECK_NEAR(quantization_tax(q, 5.0, 3.0), 0.0, 0.0);         // shrinking
  CHECK_NEAR(quantization_tax(q, -3.0, -5.0), 2.0 * q, 1e-18); // growing
  CHECK_NEAR(quantization_tax(q, 5.0, -2.0), 2.0 * q, 1e-18);  // sign flip
  CHECK_NEAR(quantization_tax(0.0, 0.0, 5.0), 0.0, 0.0);       // exact encoder
}

// The tax summed over a force history is (q/2) (sum |phi_k - phi_{k-1}| +
// |phi_last|), and that is exactly the most the readings can misattribute:
// the work booked from readings y = x + e exceeds the work done by
// sum_k phi_k (e_{k+1} - e_k), which, for errors in an interval of width q,
// an adversary can push to the tax and no further.
TEST_CASE(quantization_tax_is_the_worst_misattributed_work) {
  Rng rng(3);
  const double q = 55e-6;
  for (int trial = 0; trial < 200; ++trial) {
    // Forces that ramp, hold, chatter and change sign.
    std::vector<double> phi(400);
    double f = 0.0;
    for (auto& p : phi) {
      const double u = rng.uniform();
      if (u < 0.3) f += rng.normal() * 2.0;
      else if (u < 0.5) f = -f;
      else if (u < 0.6) f = 0.0;
      p = f;
    }
    double tax = 0.0, variation = 0.0, prev = 0.0;
    for (double p : phi) {
      tax += quantization_tax(q, prev, p);
      variation += std::fabs(p - prev);
      prev = p;
    }
    CHECK_NEAR(tax, 0.5 * q * (variation + std::fabs(phi.back())), 1e-12 * (1.0 + tax));
    // Reading errors e_0..e_N in [c, c + q]: random, and the adversary's,
    // which sits at the top of the interval wherever the force drops.
    const double c = (rng.uniform() - 1.0) * q;
    const auto misattributed = [&](bool adversarial) {
      double m = 0.0, e = 0.0;
      for (std::size_t k = 0; k <= phi.size(); ++k) {
        const double before = k == 0 ? 0.0 : phi[k - 1];
        const double after = k == phi.size() ? 0.0 : phi[k];
        const double ek = c + (adversarial ? (before > after ? q : 0.0) : q * rng.uniform());
        if (k > 0) m += before * (ek - e);
        e = ek;
      }
      return m;
    };
    CHECK(misattributed(false) <= tax + 1e-12);
    CHECK_NEAR(misattributed(true), tax, 1e-12 * (1.0 + tax));
  }
}

TEST_CASE(predictive_pc_pays_the_quantization_tax_from_the_budget) {
  // A device moving into the force at 2 cm/s: 20 um per sample, less than a
  // 55 um count. The 4 N it is asked for slows it without stopping it, so
  // the force absorbs 0.04 mJ and an exact encoder's PC passes it in full.
  // With the coarse encoder the rise costs q * 4 N = 0.22 mJ, which no
  // displacement under a count can pay.
  const double m = 0.2, dt = 1e-3, q = 55e-6;
  HeldMass dev{m, dt, 0.0};
  dev.v = 0.02;
  DevicePredictor exact(m, dt), coarse(m, dt, q);
  for (int k = 0; k < 3; ++k) {
    const double dx = dev.step(0.0);
    exact.observe(dx, 0.0);
    coarse.observe(dx, 0.0);
  }
  CHECK(exact.work(4.0) > 0.0);
  CHECK(!limit_force_predictive(0.0, 4.0, exact, 1).active);
  const auto blocked = limit_force_predictive(0.0, 4.0, coarse, 1);
  CHECK(blocked.active);
  CHECK(quantization_tax(q, 0.0, blocked.value) - coarse.work(blocked.value) <= 1e-15);
  // With a budget the PC spends exactly it, tax included.
  const double budget = 1e-4;
  const auto r = limit_force_predictive(budget, 4.0, coarse, 1);
  CHECK(r.active && r.value > 0.0 && r.value < 4.0);
  CHECK_NEAR(quantization_tax(q, 0.0, r.value) - coarse.work(r.value), budget, 1e-15);
}

// Property: whatever the device state, the held force, the request and the
// budget, the PC's force costs at most the budget over every horizon it
// checks, tax included, and any larger force of the same sign would not.
TEST_CASE(predictive_pc_force_is_the_largest_affordable_with_the_tax) {
  Rng rng(11);
  const double dt = 1e-3;
  for (int trial = 0; trial < 20000; ++trial) {
    const double m = 0.05 + rng.uniform();
    const double q = rng.bernoulli(0.2) ? 0.0 : rng.uniform() * 1e-4;
    HeldMass dev{m, dt, rng.normal() * 3.0};
    dev.v = rng.normal() * 0.05;
    DevicePredictor pred(m, dt, q);
    double held = 0.0;
    for (int k = 0; k < 4; ++k) {
      held = rng.bernoulli(0.3) ? 0.0 : rng.normal() * 5.0;
      const double dx = dev.step(held);
      pred.observe(q > 0.0 ? std::round(dev.x / q) * q - std::round((dev.x - dx) / q) * q : dx,
                   held);
    }
    const double phi = rng.normal() * 8.0;
    const double budget = rng.bernoulli(0.3) ? 0.0 : rng.exponential(5e-4);
    const int hold = rng.bernoulli(0.5) ? 1 : 2;
    const auto r = limit_force_predictive(budget, phi, pred, hold);
    const auto cost = [&](double f) {
      double worst = quantization_tax(q, held, f) - pred.work(f);
      if (hold > 1) worst = std::max(worst, quantization_tax(q, held, f) - pred.hold_work(f, hold));
      return worst;
    };
    const bool sign_ok = r.value * phi >= 0.0 && std::fabs(r.value) <= std::fabs(phi);
    const double scale = 1e-12 + 1e-9 * std::fabs(phi) * (q + 1e-4);
    const bool affordable = r.value == phi ? true : cost(r.value) <= budget + scale;
    const double bigger = r.value + 1e-6 * phi;
    const bool largest = !r.active || r.value == phi || cost(bigger) > budget - scale;
    if (!sign_ok || !affordable || !largest) {
      CHECK(sign_ok);
      CHECK(affordable);
      CHECK(largest);
      std::printf("trial %d m %g q %g held %g phi %g budget %g hold %d -> %g\n", trial, m, q,
                  held, phi, budget, hold, r.value);
      return;
    }
  }
}

TEST_CASE(reflective_store_grants_in_full_when_covered) {
  ReflectiveStore s;
  s.on_master_input_energy(1.0);
  const auto g = s.allocate(0.3, 0.2);
  CHECK(!g.limited);
  CHECK_NEAR(g.slave, 0.3, 0.0);
  CHECK_NEAR(g.master, 0.2, 0.0);
  // Only the master's share is booked; the slave's realized output is booked
  // when it happens.
  CHECK_NEAR(s.granted(), 0.2, 0.0);
  CHECK_NEAR(s.level(), 0.8, 1e-15);
  s.account_slave_energy(-0.3);
  CHECK_NEAR(s.level(), 0.5, 1e-15);
}

TEST_CASE(reflective_store_scales_both_requests_by_the_same_factor) {
  ReflectiveStore s;
  s.on_master_input_energy(0.5);
  const auto g = s.allocate(0.6, 0.4);  // wants 1.0, has 0.5
  CHECK(g.limited);
  CHECK_NEAR(g.slave, 0.3, 1e-15);
  CHECK_NEAR(g.master, 0.2, 1e-15);
  CHECK_NEAR(g.slave / g.master, 0.6 / 0.4, 1e-12);
  CHECK_NEAR(g.slave + g.master, 0.5, 1e-15);
}

TEST_CASE(reflective_store_takes_slave_input_and_ignores_stale_reports) {
  ReflectiveStore s;
  s.on_master_input_energy(0.2);
  s.on_master_input_energy(0.1);  // reordered packet
  CHECK_NEAR(s.master_input_energy(), 0.2, 0.0);
  s.account_slave_energy(0.05);   // the slave pushed into the coupling
  CHECK_NEAR(s.slave_input_energy(), 0.05, 0.0);
  CHECK_NEAR(s.level(), 0.25, 1e-15);
  // Requests that are negative (inputs) are not grants.
  const auto g = s.allocate(-1.0, -1.0);
  CHECK(!g.limited);
  CHECK_NEAR(g.slave, 0.0, 0.0);
  CHECK_NEAR(g.master, 0.0, 0.0);
}

TEST_CASE(reflective_store_grants_nothing_after_a_deficit) {
  ReflectiveStore s;
  s.on_master_input_energy(0.1);
  s.account_slave_energy(-0.15);  // the slave overshot its allowance
  CHECK(s.level() < 0.0);
  const auto g = s.allocate(0.1, 0.1);
  CHECK(g.limited);
  CHECK_NEAR(g.slave, 0.0, 0.0);
  CHECK_NEAR(g.master, 0.0, 0.0);
}

TEST_CASE(reflective_store_initial_energy_bounds_net_output) {
  ReflectiveStore s(0.05);
  CHECK_NEAR(s.level(), 0.05, 0.0);
  const auto g = s.allocate(0.04, 0.03);  // more than the initial storage
  CHECK(g.limited);
  CHECK_NEAR(g.slave + g.master, 0.05, 1e-15);
  s.account_slave_energy(-g.slave);
  CHECK_NEAR(s.level(), 0.0, 1e-15);
  // Net output never exceeds inputs plus the initial storage.
  CHECK(s.slave_output_energy() + s.granted() <=
        s.master_input_energy() + s.slave_input_energy() + s.initial_energy() + 1e-15);
  CHECK_NEAR(ReflectiveStore(-1.0).level(), 0.0, 0.0);
}
