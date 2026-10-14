#include "test.hpp"
#include "tuushin/control/predictive_pc.hpp"
#include "tuushin/control/reflection.hpp"

using tuushin::control::DevicePredictor;
using tuushin::control::limit_force_predictive;
using tuushin::control::ReflectiveStore;

namespace {

// A point mass under a constant external force H and a zero-order-held port
// force phi, integrated in closed form (constant acceleration per sample).
struct HeldMass {
  double m, dt, H;
  double x = 0.0, v = 0.0;
  // Applies phi over one sample and returns the displacement.
  double step(double phi) {
    const double a = (H - phi) / m;
    const double dx = v * dt + 0.5 * a * dt * dt;
    x += dx;
    v += a * dt;
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
