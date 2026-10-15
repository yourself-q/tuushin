#include <cmath>
#include <vector>

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

TEST_CASE(device_friction_follows_the_stribeck_curve) {
  FrictionParams fr;
  fr.coulomb = 0.5;
  fr.stiction = 0.8;
  CHECK(fr.force(0.0) == 0.0);
  CHECK_NEAR(fr.force(1.0), 0.5, 1e-12);    // far beyond the Stribeck velocity
  CHECK_NEAR(fr.force(-1.0), -0.5, 1e-12);  // against the motion
  // Below the Stribeck velocity and above the smoothing one: toward the
  // stiction force, 0.5 + 0.3 exp(-0.25) = 0.73 N.
  CHECK(fr.force(5e-3) > 0.73 && fr.force(5e-3) < 0.74);
  // A mass with viscous and Coulomb friction under 2.5 N settles where
  // 2.5 = b v + 0.5: at 1 m/s.
  WallParams far;
  far.position = 10.0;
  SimulatedSlave s(0.1, 2.0, far, kNanosPerMilli, 10);
  s.set_friction(fr);
  s.command_force(2.5);
  s.read_position(500 * kNanosPerMilli);  // ten time constants m / b
  CHECK_NEAR(s.velocity(), 1.0, 1e-3);
}

TEST_CASE(device_compliance_resonates_and_keeps_momentum) {
  // 0.2 kg split evenly, the halves joined at 50 Hz without damping: an
  // impulse on the motor side sets the two halves swinging against each
  // other at 50 Hz while their common momentum stays what the impulse gave.
  WallParams far;
  far.position = 10.0;
  const double m = 0.2, m2 = 0.1, mu = (m - m2) * m2 / m, w = 2.0 * 3.14159265358979323846 * 50.0;
  SimulatedSlave s(m, 0.0, far, kNanosPerMilli, 10);
  s.set_compliance(ComplianceParams{m2, mu * w * w, 0.0});
  s.command_force(1.0);
  s.read_position(kNanosPerMilli);
  s.command_force(0.0);
  std::vector<double> up;  // times the distal side passes the motor side upward
  double prev = s.distal_position() - s.position();
  for (Nanos t = kNanosPerMilli + 10'000; t <= 101 * kNanosPerMilli; t += 10'000) {
    s.read_position(t);
    const double d = s.distal_position() - s.position();
    if (prev < 0.0 && d >= 0.0) up.push_back(to_seconds(t) - d / (d - prev) * 1e-5);
    prev = d;
  }
  CHECK(up.size() >= 4);
  CHECK_NEAR((up.back() - up.front()) / static_cast<double>(up.size() - 1), 0.02, 2e-5);
  const double p = (m - m2) * s.true_velocity() + m2 * s.distal_velocity();
  CHECK_NEAR(p, 1.0 * 1e-3, 1e-9);
  CHECK_NEAR(s.actuator_work(), 0.5 * (m - m2) * s.true_velocity() * s.true_velocity() +
                                    0.5 * m2 * s.distal_velocity() * s.distal_velocity() +
                                    0.5 * mu * w * w * prev * prev,
             1e-9);
  // A distal mass outside (0, mass) leaves the device rigid.
  SimulatedSlave rigid(m, 0.0, far, kNanosPerMilli, 10);
  rigid.set_compliance(ComplianceParams{m, 1.0, 0.0});
  rigid.command_force(1.0);
  rigid.read_position(10 * kNanosPerMilli);
  CHECK(rigid.distal_position() == rigid.position());
  CHECK_NEAR(rigid.position(), 1.0 * 0.01 * 0.01 / (2.0 * m), 1e-12);
}

TEST_CASE(device_actuation_delay_shifts_the_commands) {
  // Commands that take effect 3 ms late move the device as the same
  // commands issued 3 ms later do; the watchdog counts from the command.
  WallParams far;
  far.position = 10.0;
  const Nanos d = 3 * kNanosPerMilli;
  SimulatedSlave late(1.0, 2.0, far, kNanosPerMilli, 10);
  SimulatedSlave shifted(1.0, 2.0, far, kNanosPerMilli, 10);
  late.set_actuation_delay(d);
  const double forces[] = {1.5, -0.5, 2.0, 0.0, 0.7};
  for (int k = 0; k < 5; ++k) {
    const Nanos t = k * 2 * kNanosPerMilli;
    late.read_position(t);
    late.command_force(forces[k]);
  }
  for (int k = 0; k < 5; ++k) {
    const Nanos t = k * 2 * kNanosPerMilli + d;
    shifted.read_position(t);
    shifted.command_force(forces[k]);
  }
  late.read_position(30 * kNanosPerMilli);
  shifted.read_position(30 * kNanosPerMilli);
  CHECK(late.force() == 0.7);
  CHECK_NEAR(late.position(), shifted.position(), 1e-15);
  CHECK_NEAR(late.actuator_work(), shifted.actuator_work(), 1e-18);
  // A command still in flight when the watchdog cuts never takes effect.
  SimulatedSlave dog(1.0, 2.0, far, kNanosPerMilli, 10);
  dog.set_actuation_delay(d);
  dog.set_watchdog(2 * kNanosPerMilli);
  dog.read_position(0);
  dog.command_force(1.5);
  dog.read_position(kNanosPerMilli);
  CHECK(dog.force() == 0.0);  // not yet in effect
  dog.read_position(10 * kNanosPerMilli);
  CHECK(dog.force() == 0.0);
  CHECK(dog.force_cut_after() == 2 * kNanosPerMilli);
  CHECK(dog.actuator_work() == 0.0);
}

TEST_CASE(actuator_schedule_splits_samples_where_commands_take_effect) {
  const Nanos ms = kNanosPerMilli;
  auto check = [](const std::vector<control::ForceSegment>& segs,
                  std::vector<control::ForceSegment> want) {
    REQUIRE(segs.size() == want.size());
    for (std::size_t i = 0; i < want.size(); ++i) {
      CHECK(segs[i].phi == want[i].phi);
      CHECK_NEAR(segs[i].samples, want[i].samples, 1e-12);
    }
  };
  // Without a delay each sample holds the force commanded at its start.
  ActuatorSchedule now(0, ms);
  now.command(0, 3.0);
  check(now.advance(0, ms, 0), {{3.0, 1.0}});
  check(now.lead(ms), {});
  // 0.4 ms: the force before acts for 0.4 ms of the sample, and the force
  // just commanded is in flight until then.
  ActuatorSchedule soon(400'000, ms);
  soon.command(0, 2.0);
  check(soon.advance(0, ms, 0), {{0.0, 0.4}, {2.0, 0.6}});
  check(soon.lead(ms), {{2.0, 0.4}});
  // A force equal to the one before continues it.
  soon.command(ms, 2.0);
  check(soon.advance(ms, 2 * ms, 0), {{2.0, 1.0}});
  // 2.5 ms: forces commanded over the last two samples are in flight.
  ActuatorSchedule late(2'500'000, ms);
  late.command(0, 1.0);
  check(late.advance(0, ms, 0), {{0.0, 1.0}});
  late.command(ms, 2.0);
  check(late.advance(ms, 2 * ms, 0), {{0.0, 1.0}});
  check(late.lead(2 * ms), {{0.0, 0.5}, {1.0, 1.0}, {2.0, 1.0}});
  check(late.advance(2 * ms, 3 * ms, 0), {{0.0, 0.5}, {1.0, 0.5}});
  // The watchdog cuts the force 0.2 ms after the last command, before the
  // command takes effect: zero from then on, and nothing in flight.
  soon.command(2 * ms, 5.0);
  check(soon.advance(2 * ms, 3 * ms, 200'000), {{2.0, 0.2}, {0.0, 0.8}});
  check(soon.lead(3 * ms), {{0.0, 0.4}});
}

TEST_CASE(master_books_forces_in_flight_by_when_they_act) {
  // A pure mass whose actuator applies each force 0.4 ms after it is
  // commanded, under forces that change every sample. Told the delay, the
  // master books what the actuator really did; taking each force to act at
  // once, it credits each with the displacement the one before it caused.
  WallParams far;
  far.position = 10.0;
  const Nanos d = 400'000;
  double error[2];
  for (const bool known : {true, false}) {
    SimulatedSlave dev(0.2, 0.0, far, kNanosPerMilli, 10);
    dev.set_actuation_delay(d);
    MasterConfig cfg;
    cfg.passivity = Passivity::kNone;
    cfg.actuation_delay = known ? d : 0;
    MasterController m(cfg);
    for (int k = 0; k < 40; ++k) {
      const Nanos now = k * kNanosPerMilli;
      m.receive(k + 1, Sample{3.0 * std::sin(0.7 * k), 0.0, 0.0});
      const auto s = m.step(now, dev.read_position(now));
      dev.command_force(-s.display_force);
    }
    // The books' net input against the work the actuator did on the device.
    error[known ? 0 : 1] =
        m.port().input_energy() - m.port().output_energy() + dev.actuator_work();
  }
  CHECK(std::fabs(error[0]) < 1e-15);
  CHECK(std::fabs(error[1]) > 1e-6);
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
