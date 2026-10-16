#include <cmath>
#include <cstdio>
#include <iterator>
#include <memory>
#include <stdexcept>

#include "test.hpp"
#include "tuushin/net/channel.hpp"
#include "tuushin/sim/teleop1d.hpp"

using namespace tuushin;

namespace {

using Passivity = sim::Teleop1DConfig::Passivity;

// Energy a sampled-data passivity controller may overshoot by. The device
// PCs predict the displacement over the coming sample from a mass model; the
// operator's damping and stiffness act within the sample and are not in the
// model, so a prediction can be off by a few microjoules per sample while the
// controller sits at its limit. The books then block output until inputs
// cover the error, so the deficit stays bounded instead of growing with time.
// 1 mJ is about the energy of a 3 N force over 0.3 mm, well below what an
// operator can feel, and five orders of magnitude below the deficit of the
// uncontrolled teleoperator at 50 ms.
constexpr double kPredictionTolerance = 1e-3;  // J

sim::Teleop1DResult run(const sim::Teleop1DConfig& cfg, Nanos one_way) {
  net::FixedDelay fwd(one_way), bwd(one_way);
  return sim::run_teleop1d(cfg, fwd, bwd);
}

}  // namespace

TEST_CASE(teleop_without_delay_is_stable_and_transparent) {
  sim::Teleop1DConfig cfg;
  cfg.passivity = Passivity::kNone;
  const auto r = run(cfg, 0);
  CHECK(!r.diverged);
  // The operator pushes 2 cm past the wall with a 300 N/m hand: about 6 N.
  CHECK(r.max_display_force > 4.0 && r.max_display_force < 9.0);
  CHECK(r.pos_rmse < 0.002);
  CHECK(r.min_passivity_margin > -1e-9);
}

TEST_CASE(teleop_delay_without_tdpa_loses_passivity) {
  sim::Teleop1DConfig cfg;
  cfg.passivity = Passivity::kNone;
  const auto r = run(cfg, from_millis(50));
  // With 100 ms round trip and a stiff wall the channel generates energy.
  CHECK(r.min_passivity_margin < -1e-3);
}

TEST_CASE(teleop_channel_tdpa_keeps_the_channel_passive) {
  sim::Teleop1DConfig cfg;
  cfg.passivity = Passivity::kChannel;
  const auto r = run(cfg, from_millis(50));
  CHECK(!r.diverged);
  CHECK(r.min_channel_margin > -kPredictionTolerance);
  CHECK(r.max_master_pos < 0.1);
  CHECK(r.max_display_force < 20.0);
  // The conventional scheme changes the velocity command, so the slave
  // drifts away from the master by several millimetres. Where it ends up
  // depends on every sample the PC cut and is chaotic: at 50 ms a change of
  // one part in 10^12 in the hand's motion moves it between -4 and 14 mm,
  // so a single run can land near zero; the mean over several delays does
  // not.
  double drift = 0.0;
  const int delays[] = {20, 50, 80, 100, 150};
  for (int ms : delays) drift += std::fabs(run(cfg, from_millis(ms)).final_drift);
  CHECK(drift / std::size(delays) > 0.003);
}

TEST_CASE(teleop_reflective_tdpa_is_passive_at_the_device_ports) {
  sim::Teleop1DConfig cfg;
  cfg.passivity = Passivity::kReflective;
  for (int ms : {0, 10, 50, 100}) {
    const auto r = run(cfg, from_millis(ms));
    CHECK(!r.diverged);
    // The device-port margin covers the slave coupling controller too, which
    // the conventional scheme leaves outside its books.
    CHECK(r.min_passivity_margin > -kPredictionTolerance);
    CHECK(r.max_display_force < 20.0);
    // The coupling follows the delayed master position itself.
    CHECK(std::fabs(r.final_drift) < 0.003);
  }
}

TEST_CASE(teleop_reflective_tdpa_renders_a_stiffer_wall_than_channel_tdpa) {
  // At 5 ms the conventional scheme's stiffness is chaotic: over runs whose
  // hand motion differs by one part in 10^12 it spans 252 to 656 N/m,
  // against 1262 N/m with energy reflection. From 10 ms the gap is tenfold.
  sim::Teleop1DConfig channel, reflective;
  channel.passivity = Passivity::kChannel;
  reflective.passivity = Passivity::kReflective;
  for (int ms : {5, 10, 25}) {
    const auto c = run(channel, from_millis(ms));
    const auto r = run(reflective, from_millis(ms));
    CHECK(r.displayed_stiffness > 1.5 * c.displayed_stiffness);
    CHECK(r.pos_rmse < c.pos_rmse);
  }
}

TEST_CASE(teleop_contact_force_reports_need_a_scheme_that_can_use_them) {
  // The coupling force is the conjugate of the commanded velocity and the
  // contact force is not, so with the contact force the conventional books
  // miss what the coupling does: even without delay the teleoperator then
  // generates energy. Energy reflection cannot run with it at all.
  sim::Teleop1DConfig cfg;
  cfg.passivity = Passivity::kChannel;
  CHECK(run(cfg, 0).min_true_margin > -1e-9);
  cfg.slave.report_contact_force = true;
  const auto r = run(cfg, 0);
  CHECK(!r.diverged);
  CHECK(r.min_true_margin < -1.0);
  cfg.passivity = Passivity::kReflective;
  bool threw = false;
  try {
    run(cfg, 0);
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  CHECK(threw);
}

TEST_CASE(teleop_tdpa_with_jitter_and_loss_stays_passive) {
  for (Passivity p : {Passivity::kChannel, Passivity::kReflective}) {
    sim::Teleop1DConfig cfg;
    cfg.passivity = p;
    net::GilbertElliottLoss::Params ge;
    ge.p_good_to_bad = 0.005;
    ge.p_bad_to_good = 0.1;
    ge.loss_good = 0.001;
    ge.loss_bad = 0.6;
    net::GilbertElliottLoss fwd(std::make_unique<net::JitterDelay>(from_millis(15), from_millis(5), false, 11), ge, 12);
    net::GilbertElliottLoss bwd(std::make_unique<net::JitterDelay>(from_millis(15), from_millis(5), false, 13), ge, 14);
    const auto r = sim::run_teleop1d(cfg, fwd, bwd);
    CHECK(!r.diverged);
    CHECK(r.forward_loss > 0.01);
    if (p == Passivity::kChannel) {
      CHECK(r.min_channel_margin > -kPredictionTolerance);
    } else {
      CHECK(r.min_passivity_margin > -kPredictionTolerance);
    }
  }
}

TEST_CASE(teleop_sampling_jitter_stays_passive) {
  // Each end samples up to 0.45 ms off its 1 ms grid, as a control loop on
  // a general-purpose OS does. A sample that strays more than half a period
  // from the last restarts the predictor's model, so the PCs block more,
  // but the books stay exact and nothing is generated.
  for (int ms : {10, 50}) {
    sim::Teleop1DConfig cfg;
    cfg.passivity = Passivity::kReflective;
    cfg.master_encoder_resolution = 55e-6;
    cfg.slave_encoder_resolution = 1e-6;
    cfg.sample_jitter = 450'000;
    const auto r = run(cfg, from_millis(ms));
    CHECK(!r.diverged);
    CHECK(r.max_book_excess < 1e-12);
    CHECK(r.min_true_margin > -1e-9);
  }
  sim::Teleop1DConfig bad;
  bad.sample_jitter = bad.dt / 2;
  bool threw = false;
  try {
    run(bad, 0);
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  CHECK(threw);
}

TEST_CASE(teleop_deadband_cuts_packet_rate) {
  for (Passivity p : {Passivity::kChannel, Passivity::kReflective}) {
    sim::Teleop1DConfig cfg;
    cfg.passivity = p;
    cfg.forward_deadband = true;
    cfg.backward_deadband = true;
    const auto r = run(cfg, 0);
    CHECK(!r.diverged);
    CHECK(r.forward_packet_rate < 300.0);
    CHECK(r.backward_packet_rate < 300.0);
  }
  // Known limitation: once the passivity controllers engage they chop the
  // signals at the sample rate, the deadband sees every chop as a change,
  // and the packet rate climbs back towards the sample rate (about 500 /s at
  // 10 ms one-way for kChannel). X. Xu, C. Schuwerk, B. Cizmeci and
  // E. Steinbach, "Energy prediction for teleoperation systems that combine
  // the time domain passivity approach with perceptual deadband-based haptic
  // data reduction", IEEE Trans. Haptics 9(4), 2016, address this.
}

TEST_CASE(teleop_reflective_tdpa_stays_passive_on_coarse_encoders) {
  // A 55 um master encoder (3D Systems Touch) and a 1 um slave encoder. With
  // the resolutions known the books never credit more than the actuators
  // took in, whatever the motion, so the true margin is that of the books'
  // guarantee: nothing is generated. The price is transparency: over nine
  // hand motions the master conveys as much of the contact force as with
  // exact encoders at 10 ms, but only 65 % and 80 % of it at 50 and 100 ms.
  for (int ms : {10, 50, 100}) {
    for (double freq : {0.4, 0.6}) {
      for (double amplitude : {0.03, 0.05}) {
        sim::Teleop1DConfig cfg;
        cfg.passivity = Passivity::kReflective;
        cfg.master_encoder_resolution = 55e-6;
        cfg.slave_encoder_resolution = 1e-6;
        cfg.op.ref_freq = freq;
        cfg.op.ref_amplitude = amplitude;
        const auto r = run(cfg, from_millis(ms));
        CHECK(!r.diverged);
        CHECK(r.max_book_excess < 1e-12);
        CHECK(r.min_true_margin > -1e-9);
      }
    }
  }
  // Told nothing, the controllers treat the readings as exact, and the
  // prediction differentiates quantization noise twice. Before the model
  // error margin the teleoperator generated a third of a joule at 50 ms.
  // The margin takes the noise for model error and outgrows any motion:
  // apart from the first force, displayed before the margin has seen a
  // count (1.9 N, 7.5 uJ), nothing is generated and nothing of the wall is
  // displayed.
  sim::Teleop1DConfig cfg;
  cfg.passivity = Passivity::kReflective;
  cfg.master_encoder_resolution = 55e-6;
  cfg.slave_encoder_resolution = 1e-6;
  cfg.encoder_resolution_known = false;
  for (int ms : {10, 50, 100}) {
    const auto r = run(cfg, from_millis(ms));
    CHECK(r.min_true_margin > -1e-5);
    CHECK(r.max_display_force < 2.5);
    CHECK(r.displayed_stiffness < 1.0);
  }
}

TEST_CASE(teleop_reflective_tdpa_estimates_dithering_encoders) {
  // Magnetic encoders dither by about half a count. Taking their readings
  // to err by the quantization alone, as before the controllers estimated
  // the noise, the Kalman filters trust each reading too much, the force
  // rate's adaptation takes the surprise for a hand pushing faster and the
  // predictions grow too wide for the PCs to let any force through: no
  // wall is displayed at all. Estimated, the wall comes out at 50 to 100
  // N/m at 10 to 100 ms (1040, 173 and 72 N/m without dither), and nothing
  // is generated.
  for (int ms : {10, 50, 100}) {
    for (bool estimate : {true, false}) {
      sim::Teleop1DConfig cfg;
      cfg.passivity = Passivity::kReflective;
      cfg.master_encoder_resolution = 55e-6;
      cfg.slave_encoder_resolution = 1e-6;
      cfg.master_encoder_noise = 0.5;
      cfg.slave_encoder_noise = 0.5;
      cfg.estimate_reading_noise = estimate;
      const auto r = run(cfg, from_millis(ms));
      CHECK(!r.diverged);
      CHECK(r.min_true_margin > -1e-7);
      const double master_truth = 55e-6 * std::sqrt(0.25 + 1.0 / 12.0);
      const double slave_truth = 1e-6 * std::sqrt(0.25 + 1.0 / 12.0);
      if (estimate) {
        CHECK(r.max_display_force > 2.0);
        CHECK(r.displayed_stiffness > 30.0);
        CHECK_NEAR(r.master_reading_noise / master_truth, 1.0, 0.15);
        CHECK_NEAR(r.slave_reading_noise / slave_truth, 1.0, 0.15);
      } else {
        CHECK(r.max_display_force < 0.1);
        CHECK_NEAR(r.master_reading_noise, 55e-6 / std::sqrt(12.0), 1e-12);
      }
    }
  }
}

namespace {

sim::Teleop1DConfig coarse(double freq, double amplitude) {
  sim::Teleop1DConfig cfg;
  cfg.passivity = Passivity::kReflective;
  cfg.master_encoder_resolution = 55e-6;
  cfg.slave_encoder_resolution = 1e-6;
  cfg.op.ref_freq = freq;
  cfg.op.ref_amplitude = amplitude;
  return cfg;
}

// The PCs as they were before the confidence bound, the tax horizon, the
// adaptation and the rise hold: the tax prepaid within the sample.
sim::Teleop1DConfig prepaid(sim::Teleop1DConfig cfg) {
  cfg.adapt_force_rate = false;
  cfg.pc_confidence = 0.0;
  cfg.pc_tax_samples = 0;
  cfg.master_pc_rise_samples = 0;
  return cfg;
}

}  // namespace

TEST_CASE(teleop_coarse_encoder_rises_that_last_chatter_less) {
  // Every count the master's reading moves into the force raises the
  // affordable force, and every count it does not lowers it again: each
  // rise pays the tax at once, and the force chatters at the count rate.
  // Holding a rise to the force the expected motion could keep paying for
  // over 64 samples calms it on every hand motion, and costs no energy.
  // With the bounds the readings put on where the master lies in its count
  // the tax is lighter, and how much chatter is left to remove varies
  // more: the hold removes 12 % of it (100 ms, 0.6 Hz, 5 cm, where the
  // bounds alone calm the force nearly as far as the hold does under the
  // tax that assumes nothing) to 70 %, and 58 % over all twelve motions.
  // (Under that tax it removed a quarter to two thirds on every motion;
  // before the model error margin, which calms the force too, one half to
  // three quarters.)
  double held_sum = 0.0, unheld_sum = 0.0;
  for (int ms : {10, 50, 100}) {
    for (double freq : {0.4, 0.6}) {
      for (double amplitude : {0.03, 0.05}) {
        auto cfg = coarse(freq, amplitude);
        const auto held = run(cfg, from_millis(ms));
        cfg.master_pc_rise_samples = 0;
        const auto unheld = run(cfg, from_millis(ms));
        CHECK(!held.diverged);
        CHECK(held.max_book_excess < 1e-12);
        CHECK(held.min_true_margin > -1e-9);
        CHECK(held.display_force_variation < 0.95 * unheld.display_force_variation);
        held_sum += held.display_force_variation;
        unheld_sum += unheld.display_force_variation;
      }
    }
  }
  CHECK(held_sum < 0.5 * unheld_sum);
}

TEST_CASE(teleop_coarse_encoder_forces_start_on_slow_motions) {
  // A rise of the master's force from zero costs the tax q |phi| at once.
  // Prepaid from the input of the coming sample, it needs the master to
  // move more than a count per sample into the force, q / dt = 5.5 cm/s
  // on a Touch at 1 kHz; a hand that presses slower never feels the wall,
  // for the master displays nothing, takes in nothing and so never earns
  // a budget. Repaid over four samples of predicted input, the force starts
  // above about q / (4 dt), 1.4 cm/s. This hand peaks at 1.9 cm/s. Holding
  // each rise to what the expected motion could keep paying for lowers the
  // peak (1.4 to 2.6 N against 2.9 to 3.3 N without), but the force starts.
  // At 100 ms the peak turns on rounding. The hand pauses near a count
  // boundary, and differences of the last bit, grown to 1e-9 m by 1.96 s,
  // move the next count transition by a sample; the force the PC then lets
  // through differs (0.42 N against 0.25 N) and the runs part. The peak is
  // 1.96 N with clang on arm64 macOS, 1.94 N there without fused
  // multiply-adds and 1.37 N with GCC on arm64 Linux.
  for (int ms : {10, 50, 100}) {
    const auto cfg = coarse(0.2, 0.03);
    const auto r = run(cfg, from_millis(ms));
    CHECK(!r.diverged);
    CHECK(r.max_book_excess < 1e-12);
    CHECK(r.min_true_margin > -1e-9);
    CHECK(r.max_display_force > 1.0);
    CHECK(run(prepaid(cfg), from_millis(ms)).max_display_force == 0.0);
  }
}

TEST_CASE(teleop_coarse_encoder_pc_survives_a_dishonest_filter) {
  // Repaying the tax over several samples trusts the filter's prediction
  // over them, and a filter told that the hand's push changes at 5 N/s
  // when it changes several times faster trusts it too much: its
  // predictions miss by many of its own standard deviations. Before the
  // model error margin the lower confidence bound was then too high, and
  // without adaptation the actuators generated up to 7 mJ. The margin, what
  // the innovations show beyond the filter's spread, widens the bound by
  // the misses, and adapting the rate makes the spread honest itself;
  // either restores the guarantee.
  for (int ms : {50, 100}) {
    auto cfg = coarse(0.6, 0.05);
    cfg.force_rate = 5.0;
    const auto r = run(cfg, from_millis(ms));
    CHECK(!r.diverged);
    CHECK(r.min_true_margin > -1e-9);
    cfg.adapt_force_rate = false;
    CHECK(run(cfg, from_millis(ms)).min_true_margin > -1e-9);
    cfg.master_pc_rise_samples = 0;
    CHECK(run(cfg, from_millis(ms)).min_true_margin > -1e-9);
  }
}

namespace {

// A fraction of a device's mass beyond a spring resonating at hz.
teleop::ComplianceParams resonance(double mass, double fraction, double hz, double zeta = 0.05) {
  const double m2 = fraction * mass, mu = (mass - m2) * m2 / mass;
  const double w = 2.0 * 3.141592653589793 * hz;
  return teleop::ComplianceParams{m2, mu * w * w, 2.0 * zeta * mu * w};
}

// Devices that are not what the controllers' model says: a rigid mass of
// the model's size under the held force and a slowly changing push.
struct Mismatch {
  const char* name;
  void (*apply)(sim::Teleop1DConfig&);
};

const Mismatch kMismatches[] = {
    {"master half the model's mass", [](sim::Teleop1DConfig& c) {
       c.master_model_mass = c.master.mass;
       c.master.mass *= 0.5;
     }},
    {"master half beyond 100 Hz",
     [](sim::Teleop1DConfig& c) { c.master.compliance = resonance(c.master.mass, 0.5, 100.0); }},
    {"master 80 % beyond 50 Hz",
     [](sim::Teleop1DConfig& c) { c.master.compliance = resonance(c.master.mass, 0.8, 50.0); }},
    {"master 1 N Coulomb", [](sim::Teleop1DConfig& c) { c.master.friction.coulomb = 1.0; }},
    {"slave half the model's mass", [](sim::Teleop1DConfig& c) {
       c.slave_model_mass = c.slave.mass;
       c.slave.mass *= 0.5;
     }},
    {"slave half beyond 50 Hz",
     [](sim::Teleop1DConfig& c) { c.slave.compliance = resonance(c.slave.mass, 0.5, 50.0); }},
    {"slave half beyond 20 Hz",
     [](sim::Teleop1DConfig& c) { c.slave.compliance = resonance(c.slave.mass, 0.5, 20.0); }},
    {"slave 10 N Coulomb", [](sim::Teleop1DConfig& c) { c.slave.friction.coulomb = 10.0; }},
};

}  // namespace

TEST_CASE(teleop_reflective_tdpa_stays_passive_on_devices_the_model_misdescribes) {
  // The PCs trust a prediction of the coming samples, and a device that
  // moves otherwise than predicted makes them spend energy that does not
  // come. Before the model error margin, on coarse encoders, a master with
  // 80 % of its mass beyond a 50 Hz resonance generated 31 J here, and half
  // the mass beyond 100 Hz 12 J; with exact encoders the first 1.3 J. The
  // margin widens each prediction by what the model has lately missed. On
  // coarse encoders, where the misses show as innovations beyond the
  // filter's spread, nothing is generated whatever the device.
  for (const auto& m : kMismatches) {
    for (int ms : {10, 50, 100}) {
      auto cfg = coarse(0.5, 0.04);
      m.apply(cfg);
      const auto r = run(cfg, from_millis(ms));
      if (r.diverged || r.min_true_margin < -1e-9) {
        std::printf("%s, %d ms: %g J\n", m.name, ms, r.min_true_margin);
      }
      CHECK(!r.diverged);
      CHECK(r.min_true_margin > -1e-9);
    }
  }
}

TEST_CASE(teleop_reflective_tdpa_nearly_passive_on_exact_readings_of_misdescribed_devices) {
  // With exact readings the miss of each sample is known exactly but only
  // after the fact, and the margin is the decaying envelope of the misses
  // seen: a miss larger than any before it is not covered. Before the
  // margin these devices generated up to 1.3 J here and 3.9 J over nine
  // hand motions; the worst over the nine now leaves a fifth of a
  // millijoule.
  for (const auto& m : kMismatches) {
    for (int ms : {10, 50, 100}) {
      sim::Teleop1DConfig cfg;
      cfg.passivity = Passivity::kReflective;
      m.apply(cfg);
      const auto r = run(cfg, from_millis(ms));
      if (r.diverged || r.min_true_margin < -5e-4) {
        std::printf("%s, %d ms: %g J\n", m.name, ms, r.min_true_margin);
      }
      CHECK(!r.diverged);
      CHECK(r.min_true_margin > -5e-4);
    }
  }
}

TEST_CASE(teleop_reflective_tdpa_pays_for_the_forces_in_flight) {
  // Devices that apply each force some time after it is commanded: 1 ms at
  // both ends with the loops up to 0.2 ms off their grid, so that the
  // forces change within samples, and 2 ms on the grid. Unaware of it, the
  // controllers credit each force with the displacement the one before it
  // caused and predict as if the new force acted at once: the books run
  // ahead of the actuators, which generate 34 to 58 mJ with 1 ms and 0.4
  // to 0.5 J with 2 ms at 50 and 100 ms, and the master's force varies 7
  // to 25 times as much. Told the delay, they split each sample's
  // displacement where the force changed and pay for the forces in flight
  // before they choose the next. What remains is model error: the split
  // misplaces up to 6 uJ against the hand's impedance within a sample, and
  // with 2 ms a prediction four samples ahead overshoots by up to 16 uJ.
  struct Lag {
    Nanos delay, jitter;
  };
  for (const Lag lag : {Lag{from_millis(1.0), from_millis(0.2)}, Lag{from_millis(2.0), 0}}) {
    for (int ms : {10, 50, 100}) {
      sim::Teleop1DResult r[2];
      for (const bool known : {true, false}) {
        sim::Teleop1DConfig cfg;
        cfg.passivity = Passivity::kReflective;
        cfg.master.actuation_delay = lag.delay;
        cfg.slave.actuation_delay = lag.delay;
        cfg.sample_jitter = lag.jitter;
        if (!known) cfg.master_model_actuation_delay = cfg.slave_model_actuation_delay = 0;
        r[known ? 0 : 1] = run(cfg, from_millis(ms));
      }
      CHECK(!r[0].diverged);
      CHECK(r[0].min_true_margin > -5e-5);
      CHECK(r[0].max_book_excess < 1e-5);
      if (ms > 10) {
        CHECK(r[1].min_true_margin < -0.03);
        CHECK(r[0].display_force_variation < 0.2 * r[1].display_force_variation);
      }
    }
  }
  // With coarse encoders the books already allow for misses, but unaware of
  // a delay of 2 ms the teleoperator still generates 45 and 72 mJ at 50 and
  // 100 ms (0.10 and 0.09 J with the reading noise taken to be the
  // quantization's: the estimate of it takes in some of the misses, and
  // the filters trust the readings less). Told it, it generates nothing.
  for (int ms : {10, 50, 100}) {
    for (const bool known : {true, false}) {
      auto cfg = coarse(0.5, 0.04);
      cfg.master.actuation_delay = from_millis(2.0);
      cfg.slave.actuation_delay = from_millis(2.0);
      if (!known) cfg.master_model_actuation_delay = cfg.slave_model_actuation_delay = 0;
      const auto r = run(cfg, from_millis(ms));
      CHECK(!r.diverged);
      if (known) {
        CHECK(r.max_book_excess < 1e-12);
        CHECK(r.min_true_margin > -1e-9);
      } else if (ms > 10) {
        CHECK(r.min_true_margin < -0.03);
      }
    }
  }
}

TEST_CASE(teleop_reflective_tdpa_estimates_the_forces_in_flight) {
  // The devices of the test above, with the controllers not told their
  // delays but estimating them from 0-4 ms (teleop::DeviceModel). From
  // exact readings both find them, and the teleoperator stays as passive
  // as with them told. With coarse encoders the master's estimate ends up
  // to 0.375 ms long, and at 10 ms the slave's stays at 4 ms, its readings
  // never telling it from 2 ms: mostly long, the side that holds the
  // forces back, and the teleoperator generates nothing. Where it lands
  // depends on the last bits of the libm's sines: with glibc the slave's
  // ends up a step of the grid short at 100 ms (1.9375 ms), and the
  // teleoperator still generates nothing.
  struct Lag {
    Nanos delay, jitter;
  };
  for (const Lag lag : {Lag{from_millis(1.0), from_millis(0.2)}, Lag{from_millis(2.0), 0}}) {
    for (int ms : {10, 50, 100}) {
      sim::Teleop1DConfig cfg;
      cfg.passivity = Passivity::kReflective;
      cfg.master.actuation_delay = lag.delay;
      cfg.slave.actuation_delay = lag.delay;
      cfg.sample_jitter = lag.jitter;
      cfg.master_actuation_delay_max = cfg.slave_actuation_delay_max = from_millis(4.0);
      const auto r = run(cfg, from_millis(ms));
      CHECK(!r.diverged);
      CHECK(r.master_actuation_delay == lag.delay);
      CHECK(r.slave_actuation_delay == lag.delay);
      CHECK(r.min_true_margin > -5e-5);
      CHECK(r.max_book_excess < 1e-5);
    }
  }
  for (int ms : {10, 50, 100}) {
    auto cfg = coarse(0.5, 0.04);
    cfg.master.actuation_delay = from_millis(2.0);
    cfg.slave.actuation_delay = from_millis(2.0);
    cfg.master_actuation_delay_max = cfg.slave_actuation_delay_max = from_millis(4.0);
    const auto r = run(cfg, from_millis(ms));
    CHECK(!r.diverged);
    CHECK(r.master_actuation_delay >= from_millis(2.0) - from_millis(0.25));
    CHECK(r.slave_actuation_delay >= from_millis(2.0) - from_millis(0.25));
    CHECK(r.max_book_excess < 1e-12);
    CHECK(r.min_true_margin > -1e-9);
  }
}

TEST_CASE(teleop_model_mediated_books_dithering_encoders_by_a_second_reading) {
  // Half a count of dither, 100 ms each way, a force that acts at once. The
  // interval tax bounds nothing for noisy readings; the noise tax needs
  // each change booked by a reading its choice did not see, which without
  // an actuation lead only a second reading after the command provides.
  // With it the master's PC error falls from 15 N to 2.5 N and the master
  // port stays passive.
  sim::Teleop1DConfig cfg;
  cfg.passivity = Passivity::kModelMediated;
  cfg.duration = from_seconds(20.0);
  cfg.op.ref_freq = 0.45;
  cfg.op.ref_amplitude = 0.04;
  cfg.master_encoder_resolution = cfg.slave_encoder_resolution = 55e-6;
  cfg.master_encoder_noise = cfg.slave_encoder_noise = 0.5;
  const auto interval = run(cfg, from_millis(100));
  cfg.encoder_noise_known = true;
  const auto unread = run(cfg, from_millis(100));
  cfg.reread = true;
  const auto reread = run(cfg, from_millis(100));
  CHECK(!reread.diverged);
  CHECK(reread.min_master_true_margin >= 0.0);
  CHECK(interval.pc_force_rmse > 10.0);
  // Without the second reading the noise tax does not apply.
  CHECK(unread.pc_force_rmse > 10.0);
  CHECK(reread.pc_force_rmse < 0.25 * interval.pc_force_rmse);
  // The reflective books, with the same tax: what remains is the end term,
  // the newest reading's noise times the force now, which is not charged.
  cfg.passivity = Passivity::kReflective;
  const auto reflective = run(cfg, from_millis(100));
  CHECK(!reflective.diverged);
  CHECK(reflective.min_true_margin > -1e-7);
  CHECK(reflective.max_display_force > 1.0);
}
