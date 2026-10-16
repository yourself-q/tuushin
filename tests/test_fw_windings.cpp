#include <cmath>
#include <complex>
#include <cstdio>
#include <cstring>
#include <random>

#include "test.hpp"
#include "tuushin/fw/foc.hpp"
#include "tuushin/fw/plant.hpp"
#include "tuushin/fw/windings.hpp"

using namespace tuushin::fw;

namespace {

constexpr double kPiD = 3.14159265358979323846;

// The gimbal motor of the simulation with its windings' inductance and a
// PWM that holds a new duty until its next period: at a 200 us tick every
// write lands on a period's edge, so the delay is the period itself.
PlantParams motor(double inductance = 1.5e-3) {
  PlantParams p;  // with cogging, friction and the encoder's noise
  p.phase_inductance = inductance;
  p.pwm_period = 50e-6;
  p.step = 5e-6;
  return p;
}

WindingIdent::Setup setup_for(const PlantParams& p, double tau_configured, std::uint32_t tick_us = 200) {
  WindingIdent::Setup s;
  s.tick_us = tick_us;
  s.tau = static_cast<float>(tau_configured);
  s.resistance = static_cast<float>(p.phase_resistance);
  s.kt = static_cast<float>(p.kt);
  s.inertia = static_cast<float>(p.inertia);
  s.back_emf = static_cast<float>(p.kt / 1.5);
  s.duty_delay = static_cast<float>(p.pwm_period);
  s.voltage_limit = 5.0f;
  return s;
}

// Ticks that do not come: each at random with a probability, and one
// stall from a time (us) for a while, the voltage held all along.
struct Missed {
  double share = 0.0;
  std::uint32_t stall_at = 0, stall_us = 0;
};

// The excitation and the lock-in on the simulated motor, a tick at a
// time, the q voltage put on at the electrical angle read, as the device
// does; the fit.
WindingFit identify(const PlantParams& p, const WindingIdent::Setup& s, std::uint64_t seed,
                    const HandParams* hand = nullptr, const Missed& missed = {}, double* most = nullptr) {
  Plant plant(p, seed);
  plant.enable(true);
  if (hand) plant.set_hand(*hand, 0.0);
  TurnCounter turns(p.counts_per_rev);
  turns.reset(plant.encoder_raw());
  WindingIdent id;
  std::uint32_t now = 0;
  double t = 0.0;
  id.start(s, now);
  const double q = 2.0 * kPiD / p.counts_per_rev, start = plant.angle();
  std::mt19937_64 rng(seed);
  std::bernoulli_distribution skip(missed.share);
  while (id.running()) {
    t += s.tick_us * 1e-6;
    now += s.tick_us;
    plant.advance(t);
    if (most) *most = std::max(*most, std::fabs(plant.angle() - start));
    const bool stalled = missed.stall_us > 0 && now > missed.stall_at && now <= missed.stall_at + missed.stall_us;
    if (stalled || (missed.share > 0.0 && skip(rng))) continue;
    const std::uint32_t raw = plant.encoder_raw();
    turns.update(raw);
    const float u = id.step(now, turns.count() * q);
    const double te = p.direction * p.pole_pairs * (raw * q) - p.electrical_offset;
    const BridgeVoltages v = space_vector(0.0f, u, static_cast<float>(te), static_cast<float>(p.supply));
    plant.set_duties(v.a / p.supply, v.b / p.supply, v.c / p.supply);
  }
  return id.fit();
}

// How far L / R may be off: three of the fit's own standard deviations,
// and 1 % for what its model leaves out, as the device bounds it.
double allowed(const WindingFit& f) {
  const double rel = f.tau_sd / f.tau;
  return 3.0 * std::sqrt(rel * rel + 1e-4);
}

}  // namespace

TEST_CASE(fw_windings_fit_an_exact_response) {
  // Points of G itself, at the frequencies a 200 us tick measures: the
  // fit's model, without the readings' noise, gives back L / R and D.
  const double fs[6] = {125.0, 172.4, 238.1, 333.3, 454.5, 625.0};
  const double l = 1.5e-3, r = 5.0, j = 7e-6, b = 2e-6, kt = 0.08, d = 50e-6;
  WindingPoint pts[6];
  for (int i = 0; i < 6; ++i) {
    const double w = 2.0 * kPiD * fs[i];
    const std::complex<double> s(0.0, w);
    const std::complex<double> den = (l * s + r) * (j * s * s + b * s) + kt * kt / 1.5 * s;
    pts[i].w = w;
    pts[i].g = kt * std::exp(-s * d) / den;
    pts[i].amp = 8e-3 / (1.0 + i);
    pts[i].rel = 1e-3 * (1.0 + i);
  }
  const WindingFit f = fit_windings(pts, 6, 200e-6, 250e-6);
  REQUIRE(f.ok);
  CHECK_NEAR(f.tau, l / r, 1e-3 * l / r);
  CHECK_NEAR(f.delay, d, 0.5e-6);
  CHECK_NEAR(f.p2, (r * j + l * b) / kt, 1e-3 * r * j / kt);
  CHECK(f.chi2 < 1e-3);
  // Four points are enough; three are not.
  CHECK(fit_windings(pts, 4, 200e-6, 250e-6).ok);
  CHECK(!fit_windings(pts, 3, 200e-6, 250e-6).ok);
}

TEST_CASE(fw_windings_find_l_over_r_from_the_encoder) {
  // A gimbal motor's 300 us, at the board's 200 us tick: within the
  // bounds the fit gives, and the delay, the inertia and the damping
  // besides the back-EMF's as the motor has them.
  const PlantParams p = motor();
  const double tau = p.phase_inductance / p.phase_resistance;
  double sum = 0.0;
  for (std::uint64_t seed = 1; seed <= 4; ++seed) {
    const WindingFit f = identify(p, setup_for(p, tau), seed);
    std::printf("  seed %d: L/R %.1f us +- %.1f %%, D %.1f +- %.1f us, J %.3e, b %.2e, chi2 %.2f %s\n",
                static_cast<int>(seed), f.tau * 1e6, 100.0 * f.tau_sd / f.tau, f.delay * 1e6,
                f.delay_sd * 1e6, f.inertia, f.damping, f.chi2, f.why);
    REQUIRE(f.ok);
    CHECK(std::fabs(f.tau / tau - 1.0) < allowed(f));
    CHECK(f.tau_sd < 0.03 * tau);
    CHECK(std::fabs(f.delay - p.pwm_period) < 4.0 * f.delay_sd + 3e-6);
    CHECK_NEAR(f.inertia, p.inertia, 0.05 * p.inertia);
    CHECK(f.chi2 < 4.0);
    sum += f.tau / tau - 1.0;
  }
  CHECK(std::fabs(sum / 4.0) < 0.02);
}

TEST_CASE(fw_windings_find_it_whatever_was_configured) {
  // The configured L / R only designs the voltages: half or twice it, the
  // fit still finds the motor's.
  const PlantParams p = motor();
  const double tau = p.phase_inductance / p.phase_resistance;
  for (double configured : {0.5 * tau, 2.0 * tau}) {
    const WindingFit f = identify(p, setup_for(p, configured), 3);
    std::printf("  configured %.0f us: L/R %.1f us +- %.1f %% %s\n", configured * 1e6, f.tau * 1e6,
                100.0 * f.tau_sd / f.tau, f.why);
    REQUIRE(f.ok);
    CHECK(std::fabs(f.tau / tau - 1.0) < allowed(f));
  }
  // And another motor's: 3 mH configured as 1.5.
  const PlantParams q = motor(3e-3);
  const WindingFit f = identify(q, setup_for(q, tau), 5);
  std::printf("  3 mH: L/R %.1f us +- %.1f %% %s\n", f.tau * 1e6, 100.0 * f.tau_sd / f.tau, f.why);
  REQUIRE(f.ok);
  CHECK(std::fabs(f.tau / (q.phase_inductance / q.phase_resistance) - 1.0) < allowed(f));
}

TEST_CASE(fw_windings_reject_what_is_not_a_free_shaft) {
  const PlantParams p = motor();
  const double tau = p.phase_inductance / p.phase_resistance;
  // A hand on the knob where it is: its inertia and damping.
  HandParams h;
  h.present = true;
  h.ref_offset = p.start_angle;
  h.ref_amplitude = 0.0;
  h.inertia = 1e-5;
  h.b = 2e-3;
  for (std::uint64_t seed = 1; seed <= 2; ++seed) {
    const WindingFit f = identify(p, setup_for(p, tau), seed, &h);
    std::printf("  held: ok %d (%s), J %.2e\n", f.ok, f.why, f.inertia);
    CHECK(!f.ok);
  }
  // Friction ten times a gimbal motor's, which took L / R 8 % off.
  PlantParams sticky = p;
  sticky.coulomb = 1e-2;
  const WindingFit f = identify(sticky, setup_for(sticky, tau), 1);
  std::printf("  sticky: ok %d (%s), L/R %.1f us\n", f.ok, f.why, f.tau * 1e6);
  CHECK(!f.ok);
  CHECK(std::strcmp(f.why, "friction too large") == 0);
  // No inductance: nothing to measure.
  PlantParams bare = p;
  bare.phase_inductance = 0.0;
  const WindingFit g = identify(bare, setup_for(bare, tau), 1);
  std::printf("  no inductance: ok %d (%s)\n", g.ok, g.why);
  CHECK(!g.ok);
}

TEST_CASE(fw_windings_keep_the_shaft_in_place) {
  // The hum ramps down at each point's end: the shaft ends where it began
  // instead of coasting off at the speed it had when the voltage stopped.
  const PlantParams p = motor();
  Plant plant(p, 1);
  plant.enable(true);
  TurnCounter turns(p.counts_per_rev);
  turns.reset(plant.encoder_raw());
  WindingIdent id;
  const WindingIdent::Setup s = setup_for(p, p.phase_inductance / p.phase_resistance);
  std::uint32_t now = 0;
  double t = 0.0, most = 0.0;
  const double start = plant.angle(), q = 2.0 * kPiD / p.counts_per_rev;
  id.start(s, now);
  CHECK(id.duration_us() > 3000000 && id.duration_us() < 5000000);
  while (id.running()) {
    t += s.tick_us * 1e-6;
    now += s.tick_us;
    plant.advance(t);
    const std::uint32_t raw = plant.encoder_raw();
    turns.update(raw);
    const float u = id.step(now, turns.count() * q);
    const double te = p.direction * p.pole_pairs * (raw * q) - p.electrical_offset;
    const BridgeVoltages v = space_vector(0.0f, u, static_cast<float>(te), static_cast<float>(p.supply));
    plant.set_duties(v.a / p.supply, v.b / p.supply, v.c / p.supply);
    most = std::max(most, std::fabs(plant.angle() - start));
  }
  CHECK_NEAR(now, id.duration_us(), 4.0 * s.tick_us);
  plant.enable(false);
  const double end = plant.angle();
  plant.advance(t + 0.5);
  std::printf("  the shaft: %.1f mrad at the most, %.1f at the end, coasted %.1f\n", 1e3 * most,
              1e3 * (end - start), 1e3 * (plant.angle() - end));
  CHECK(most < 0.015);
  CHECK(std::fabs(end - start) < 0.005);
  CHECK(std::fabs(plant.angle() - end) < 0.002);
}

TEST_CASE(fw_windings_give_up_on_missed_ticks) {
  // A tick missed holds the voltage a tick longer than the fit takes it,
  // and the shaft drifts off the line each point fits: the noise the
  // points measure takes that in, and the bounds widen. Half a percent
  // missed took L / R's standard deviation from 1.2 % to some 7.5 %, its
  // error within it, 4 seeds; more than the percent allowed and the
  // measurement is given up, whatever it found.
  const PlantParams p = motor();
  const double tau = p.phase_inductance / p.phase_resistance;
  Missed few;
  few.share = 0.005;
  const WindingFit f = identify(p, setup_for(p, tau), 2, nullptr, few);
  std::printf("  0.5 %% missed: ok %d (%s), L/R %.1f us +- %.1f %%\n", f.ok, f.why, f.tau * 1e6,
              100.0 * f.tau_sd / f.tau);
  REQUIRE(f.ok);
  CHECK(std::fabs(f.tau / tau - 1.0) < allowed(f));
  CHECK(f.tau_sd > 0.03 * f.tau);
  Missed many;
  many.share = 0.05;
  const WindingFit g = identify(p, setup_for(p, tau), 2, nullptr, many);
  std::printf("  5 %% missed: ok %d (%s), L/R %.1f us\n", g.ok, g.why, g.tau * 1e6);
  CHECK(!g.ok);
  CHECK(std::strcmp(g.why, "ticks missed") == 0);
}

TEST_CASE(fw_windings_stop_when_the_shaft_wanders_off) {
  // A stall of 30 ms in the second point, its voltage held: the shaft
  // turns off, and the hum stops at the first tick after.
  const PlantParams p = motor();
  const double tau = p.phase_inductance / p.phase_resistance;
  Missed stall;
  stall.stall_at = 900000;
  stall.stall_us = 30000;
  double most = 0.0;
  const WindingFit f = identify(p, setup_for(p, tau), 1, nullptr, stall, &most);
  std::printf("  stalled 30 ms: ok %d (%s), the shaft %.2f rad at the most\n", f.ok, f.why, most);
  CHECK(!f.ok);
  CHECK(std::strcmp(f.why, "shaft wandered off") == 0);
  CHECK(most > WindingIdent::kWander);
}
