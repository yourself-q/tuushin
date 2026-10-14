#include <cmath>
#include <memory>

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
  // drifts away from the master by several millimetres.
  CHECK(std::fabs(r.final_drift) > 0.002);
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
