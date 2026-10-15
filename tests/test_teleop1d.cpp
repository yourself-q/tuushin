#include <cmath>
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
  sim::Teleop1DConfig channel, reflective;
  channel.passivity = Passivity::kChannel;
  reflective.passivity = Passivity::kReflective;
  for (int ms : {5, 10, 25}) {
    const auto c = run(channel, from_millis(ms));
    const auto r = run(reflective, from_millis(ms));
    CHECK(r.displayed_stiffness > 2.0 * c.displayed_stiffness);
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
  // hand motions at 10-100 ms the master conveys a little over half of the
  // contact force it conveys with exact encoders.
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
  // Told nothing, the controllers treat the readings as exact: the
  // prediction differentiates quantization noise twice, and at 50 ms the
  // teleoperator generates a third of a joule.
  sim::Teleop1DConfig cfg;
  cfg.passivity = Passivity::kReflective;
  cfg.master_encoder_resolution = 55e-6;
  cfg.slave_encoder_resolution = 1e-6;
  cfg.encoder_resolution_known = false;
  CHECK(run(cfg, from_millis(50)).min_true_margin < -0.1);
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

// The PCs as they were before the confidence bound, the tax horizon and the
// adaptation: the tax prepaid within the sample.
sim::Teleop1DConfig prepaid(sim::Teleop1DConfig cfg) {
  cfg.adapt_force_rate = false;
  cfg.pc_confidence = 0.0;
  cfg.pc_tax_samples = 0;
  return cfg;
}

}  // namespace

TEST_CASE(teleop_coarse_encoder_forces_start_on_slow_motions) {
  // A rise of the master's force from zero costs the tax q |phi| at once.
  // Prepaid from the input of the coming sample, it needs the master to
  // move more than a count per sample into the force, q / dt = 5.5 cm/s
  // on a Touch at 1 kHz; a hand that presses slower never feels the wall,
  // for the master displays nothing, takes in nothing and so never earns
  // a budget. Repaid over four samples of predicted input, the force starts
  // above about q / (4 dt), 1.4 cm/s. This hand peaks at 1.9 cm/s.
  for (int ms : {10, 50, 100}) {
    const auto cfg = coarse(0.2, 0.03);
    const auto r = run(cfg, from_millis(ms));
    CHECK(!r.diverged);
    CHECK(r.max_book_excess < 1e-12);
    CHECK(r.min_true_margin > -1e-9);
    CHECK(r.max_display_force > 2.0);
    CHECK(run(prepaid(cfg), from_millis(ms)).max_display_force == 0.0);
  }
}

TEST_CASE(teleop_coarse_encoder_pc_needs_an_honest_filter) {
  // Repaying the tax over several samples trusts the filter's prediction
  // over them, and a filter told that the hand's push changes at 5 N/s
  // when it changes several times faster trusts it too much: its
  // predictions miss by many of its own standard deviations, the lower
  // confidence bound is too high, and the actuators generate energy.
  // Adapting the rate to the innovations restores the guarantee.
  for (int ms : {50, 100}) {
    auto cfg = coarse(0.6, 0.05);
    cfg.force_rate = 5.0;
    const auto r = run(cfg, from_millis(ms));
    CHECK(!r.diverged);
    CHECK(r.min_true_margin > -1e-9);
    cfg.adapt_force_rate = false;
    CHECK(run(cfg, from_millis(ms)).min_true_margin < -1e-3);
  }
}
