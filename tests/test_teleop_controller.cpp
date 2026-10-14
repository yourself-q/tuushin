#include <cmath>

#include "test.hpp"
#include "tuushin/teleop/controller.hpp"
#include "tuushin/teleop/device.hpp"

using namespace tuushin;
using namespace tuushin::teleop;

TEST_CASE(device_integrates_irregular_reads_like_regular_ones) {
  // A mass with viscous friction under a constant force, away from the wall:
  // v(t) = (F / b) (1 - exp(-b t / m)). Reads at irregular times, as a late
  // real-time loop makes them, must land on the same trajectory.
  WallParams far;
  far.position = 10.0;
  const double m = 1.0, b = 2.0, f = 1.5;
  SimulatedSlave regular(m, b, far, kNanosPerMilli, 10);
  SimulatedSlave irregular(m, b, far, kNanosPerMilli, 10);
  regular.command_force(f);
  irregular.command_force(f);
  for (Nanos t = 0; t <= 500 * kNanosPerMilli; t += kNanosPerMilli) regular.read_position(t);
  const Nanos gaps[] = {300'000, 1'700'000, 1'000'000, 4'200'000, 50'000, 2'750'000};
  Nanos t = 0;
  for (int i = 0; t < 500 * kNanosPerMilli; ++i) {
    t = std::min<Nanos>(t + gaps[i % 6], 500 * kNanosPerMilli);
    irregular.read_position(t);
  }
  const double ts = 0.5;
  const double v = f / b * (1.0 - std::exp(-b * ts / m));
  const double x = f / b * (ts - m / b * (1.0 - std::exp(-b * ts / m)));
  CHECK(std::fabs(regular.velocity() - v) < 1e-12);
  CHECK(std::fabs(regular.position() - x) < 1e-12);
  CHECK(std::fabs(irregular.velocity() - v) < 1e-10);
  CHECK(std::fabs(irregular.position() - x) < 1e-10);
  // Reading twice at the same time changes nothing.
  const double x0 = irregular.position();
  CHECK(irregular.read_position(t) == x0);
}

TEST_CASE(device_watchdog_cuts_a_stale_force_and_reports_it) {
  // A force commanded at 0 with a 2 ms watchdog, then no command until
  // 10 ms: the device behaves as if the host had commanded zero at 2 ms.
  WallParams far;
  far.position = 10.0;
  SimulatedSlave dog(1.0, 2.0, far, kNanosPerMilli, 10);
  SimulatedSlave host(1.0, 2.0, far, kNanosPerMilli, 10);
  dog.set_watchdog(2 * kNanosPerMilli);
  dog.read_position(0);
  dog.command_force(1.5);
  dog.read_position(kNanosPerMilli);
  CHECK(dog.force() == 1.5);
  CHECK(dog.force_cut_after() == 0);
  dog.read_position(10 * kNanosPerMilli);
  CHECK(dog.force() == 0.0);
  CHECK(dog.force_cut_after() == 2 * kNanosPerMilli);
  host.read_position(0);
  host.command_force(1.5);
  host.read_position(2 * kNanosPerMilli);
  host.command_force(0.0);
  host.read_position(10 * kNanosPerMilli);
  CHECK_NEAR(dog.position(), host.position(), 1e-15);
  CHECK_NEAR(dog.velocity(), host.velocity(), 1e-12);
  CHECK_NEAR(dog.actuator_work(), host.actuator_work(), 1e-18);
  CHECK(dog.actuator_work() > 0.0);
  // The next command clears the report.
  dog.command_force(1.0);
  CHECK(dog.force_cut_after() == 0);
}

TEST_CASE(master_books_a_cut_force_as_predicted_output_only) {
  // The master displays 5 N, then its loop stalls for 10 ms and the device's
  // watchdog cuts the force after 2 ms. The encoder shows the hand pushed
  // 1 mm into the force over the whole stall, but the force did not act for
  // most of it: crediting 5 N * 1 mm = 5 mJ of input would let the PC pay
  // out energy that never entered.
  for (const bool cut : {false, true}) {
    MasterController m(MasterConfig{});
    m.receive(1, Sample{5.0, 0.0, 1.0});  // plenty of granted energy
    CHECK(m.step(0, 0.0).display_force == 5.0);
    const Nanos now = cut ? 10 * kNanosPerMilli : kNanosPerMilli;
    const double x = cut ? 1e-3 : 1e-4;
    const auto s = m.step(now, x, cut ? 2 * kNanosPerMilli : 0);
    CHECK_NEAR(s.sample.value, 0.1, 1e-12);  // dx over the time that passed
    if (!cut) {
      CHECK_NEAR(m.port().input_energy(), 5.0 * 1e-4, 1e-15);
      CHECK(m.port().output_energy() == 0.0);
      continue;
    }
    // From rest, 5 N held for 2 ms moves the 0.2 kg model by
    // -5 * (2 ms)^2 / (2 * 0.2 kg) = -50 um: an output of 0.25 mJ.
    CHECK(m.port().input_energy() == 0.0);
    CHECK_NEAR(m.port().output_energy(), 5.0 * 50e-6, 1e-12);
    // The model has to see two regular samples before it trusts itself again.
    CHECK(s.display_force == 0.0);
    CHECK(m.step(11 * kNanosPerMilli, x).display_force == 0.0);
    CHECK(m.step(12 * kNanosPerMilli, x).display_force > 0.0);
  }
}

TEST_CASE(wall_only_pushes) {
  WallParams w;
  SimulatedSlave s(1.0, 2.0, w, kNanosPerMilli, 10, 0, w.position + 0.001);
  CHECK(std::fabs(s.contact_force() - w.k * 0.001) < 1e-9);
  // Leaving the wall fast enough would make the damper pull; it does not.
  s.command_force(-200.0);
  s.read_position(5 * kNanosPerMilli);
  CHECK(s.velocity() < 0.0);
  CHECK(s.contact_force() >= 0.0);
}

TEST_CASE(master_displays_newest_sample_and_counts_every_energy) {
  MasterConfig cfg;
  cfg.passivity = Passivity::kNone;
  MasterController m(cfg);
  m.receive(5, Sample{2.0, 0.0, 0.010});
  m.receive(3, Sample{9.0, 0.0, 0.020});  // late: its force is stale, its energy is not
  const auto s = m.step(0, 0.0);
  CHECK(s.display_force == 2.0);
  CHECK(m.port().remote_input_energy() == 0.020);
  CHECK(s.send);
  CHECK(s.sample.value == 0.0);  // no velocity on the first sample
}

TEST_CASE(master_pc_blocks_output_without_energy) {
  // A force that would push the operator back while the device moves into
  // it is free; one that does work on the operator needs granted energy.
  MasterConfig cfg;
  MasterController m(cfg);
  m.receive(1, Sample{5.0, 0.0, 0.0});
  double x = 0.0;
  m.step(0, x);
  // The device moves away from the force direction (towards -x): the force
  // would deliver energy, and nothing was granted.
  for (int k = 1; k < 20; ++k) {
    x -= 1e-5;
    const auto s = m.step(k * kNanosPerMilli, x);
    CHECK(m.port().output_energy() - m.port().input_energy() < 1e-6);
    (void)s;
  }
}

TEST_CASE(slave_holds_after_the_link_goes_quiet) {
  // One sample says the master moves at 0.1 m/s, then nothing arrives. With
  // hold_after the estimate stops after that long; without it the slave
  // would be dragged along indefinitely.
  for (const Nanos hold : {Nanos{0}, from_millis(50)}) {
    SlaveConfig cfg;
    cfg.passivity = Passivity::kNone;
    cfg.hold_after = hold;
    SlaveController s(cfg, 0.0);
    s.receive(1, Sample{0.1, 0.0, 0.0});
    for (int k = 0; k < 300; ++k) s.step(k * kNanosPerMilli, 0.0, 0.0, 0.0);
    if (hold == 0) {
      CHECK(std::fabs(s.master_estimate() - 0.1 * 0.300) < 1e-9);
    } else {
      // Extrapolated over the fresh step and the 50 that followed it.
      CHECK(std::fabs(s.master_estimate() - 0.1 * 0.051) < 1e-9);
      CHECK(s.requested_velocity() == 0.0);
    }
  }
}

TEST_CASE(slave_resets_estimate_to_each_new_position) {
  SlaveConfig cfg;
  cfg.passivity = Passivity::kReflective;
  SlaveController s(cfg, 0.0);
  s.receive(1, Sample{0.0, 0.005, 0.0});
  s.step(0, 0.0, 0.0, 0.0);
  CHECK(s.command() == 0.005);
  // A reordered older sample does not move it back.
  s.receive(0, Sample{0.0, 0.001, 0.0});
  s.step(kNanosPerMilli, 0.0, 0.0, 0.0);
  CHECK(s.command() == 0.005);
  // Without energy in the store the coupling force is reported but not
  // applied: the report is how the slave asks for energy.
  const auto st = s.step(2 * kNanosPerMilli, 0.0, 0.0, 0.0);
  CHECK(std::fabs(st.sample.value - cfg.kp * 0.005) < 1e-9);
  CHECK(st.force == 0.0 || std::fabs(st.force) < 1e-12);
}
