#pragma once

#include <complex>
#include <cstdint>

// The windings' L / R, and the delay from a duty written to the voltage it
// puts on them, measured from the encoder alone, with no current sensor,
// during calibration.
//
// A sinusoidal q voltage at a few frequencies of a few hundred hertz moves
// the shaft by a few milliradians, and a lock-in on the readings gives the
// angle per volt at each. With the motor's equations (foc.hpp) and a shaft
// of inertia J, viscous damping b and a spring k (the cogging about its
// detent), that is
//
//   G(s) = kt e^{-sD} / ((L s + R)(J s^2 + b s + k) + kt ke s)
//        = e^{-sD} / (p3 s^3 + p2 s^2 + p1 s + p0),
//
// with p3 = L J / kt and p2 = (R J + L b) / kt, whose ratio is L / R less
// its part b L / (R J): 1e-4 for a gimbal motor's rotor and knob, some 4 %
// with a hand holding the knob, which is one reason the shaft must be free.
// Coulomb friction F enters by its describing function, a damping of
// 4 F / (pi A w) at an angle amplitude A, which is q / A (j - w L / R) in
// 1 / G; that holds while the friction is small against the inertia's
// torque, J w^2 A, and the fit says how small it was. D takes in all that
// lags between the reading and the voltage: the computation, the PWM's
// wait for its next period, the encoder's own lag. The voltage is held
// over a tick: its fundamental is divided out of each point, and its
// images about the sampling rate, which the readings fold back onto the
// frequency, are taken out with the fit, and the fit made again.
//
// The established ways to measure L and R all measure the current (ODrive's
// and SimpleFOC's calibrations, TI's InstaSPIN motor identification); this
// one does without. In simulation (firmware/core/src/windings.cpp) it finds L / R
// within about 1 % at a 200 us tick, with friction and cogging at up to
// four times a gimbal motor's, in some 4 s.

namespace tuushin::fw {

// One frequency's measurement.
struct WindingPoint {
  double w = 0.0;            // rad/s
  std::complex<double> g;    // the angle per volt of u_q (rad/V), the hold's fundamental divided out
  double amp = 0.0;          // the angle's amplitude (rad)
  double rel = 0.0;          // the standard deviation of each part of g, relative to |g|
};

struct WindingFit {
  bool ok = false;
  const char* why = "";  // what failed the checks, when not ok
  double tau = 0.0;      // L / R (s)
  double tau_sd = 0.0;   // its standard deviation (s)
  double delay = 0.0;    // D (s)
  double delay_sd = 0.0;
  double p1 = 0.0;       // (R b + L k) / kt + ke (V s/rad)
  double p2 = 0.0;       // (R J + L b) / kt (V s^2/rad)
  // By WindingIdent::fit, from the motor as configured: the inertia,
  // p2 kt / R (kg m^2), and the damping besides the back-EMF's,
  // (p1 - ke) kt / R (N m s/rad).
  double inertia = 0.0;
  double damping = 0.0;
  // The misfit's chi-square per degree of freedom: about 1 when it is the
  // readings' noise. The standard deviations are scaled up by its root
  // when it is more.
  double chi2 = 0.0;
  // The largest ratio, over the points, of the friction's term in 1 / G
  // and of the spring's (the cogging's) to the inertia's.
  double friction = 0.0;
  double stiffness = 0.0;
};

// Fits G to n points (4 to WindingIdent::kMaxPoints) measured at a tick of
// ts (s), D within [-ts / 10, delay_max] (s), and checks the fit.
WindingFit fit_windings(const WindingPoint* pts, int n, double ts, double delay_max);

// The excitation and the lock-in, a tick at a time, and the fit.
class WindingIdent {
 public:
  static constexpr int kMaxPoints = 6;

  struct Setup {
    std::uint32_t tick_us = 200;
    // What the amplitudes are designed with: L / R as configured (s), the
    // phase resistance (ohm), kt (N m/A), the inertia (kg m^2).
    float tau = 300e-6f;
    float resistance = 5.0f;
    float kt = 0.08f;
    float inertia = 7e-6f;
    // u_q per rad/s (V s/rad), and the longest the PWM makes a new duty
    // wait (s): the fit's D is sought up to a tick longer than that.
    float back_emf = 0.0533f;
    float duty_delay = 50e-6f;
    float voltage_limit = 5.0f;  // V
    // The shaft's amplitude aimed at (rad), as far as voltage_limit
    // allows: well beyond what friction holds at the lowest frequency, and
    // little enough against a cogging period that the cogging stays a
    // spring. Its stiffness falls with the amplitude A, by 2 J1(N A) /
    // (N A) for N periods a turn: at 84 (a 12N14P motor) by 6 % at 8 mrad,
    // by 13 % at 12, where, the points' amplitudes differing (the voltage
    // limit takes the higher frequencies' down), cogging 8 times a gimbal
    // motor's took L / R 6 % off, and 0.0 % at 8.
    float amplitude = 8e-3f;
    float seconds = 0.5f;  // each frequency's measurement
  };

  // How far the shaft may turn from where it was first read (rad) before
  // the hum stops: ten times the most it turned in simulation (9 mrad),
  // and less than a stall turns it, the hum's voltage held (0.3 rad in one
  // of 30 ms, in simulation).
  static constexpr double kWander = 0.1;
  // The share of ticks that may come late, a tick or more missed, before
  // the measurement is given up: the voltage put on is then not the one
  // the fit takes it to be. Fewer only widen the bounds (L / R's standard
  // deviation 7.5 % at 0.5 %, in simulation); at 1 % it neared the 10 %
  // that fails the fit as too imprecise.
  static constexpr double kLateShare = 0.01;

  // Starts at now (us).
  void start(const Setup& s, std::uint32_t now);
  // A tick: the shaft's angle (rad, over turns) as read at now; returns the
  // q voltage to hold until the next.
  float step(std::uint32_t now, double theta);
  bool running() const { return !wandered_ && point_ < count_; }
  // How long it takes at a regular tick (us).
  std::uint32_t duration_us() const;
  int points() const { return point_; }
  const WindingPoint& point(int i) const { return pts_[i]; }
  // Of the ticks so far: how many, and how many came late.
  std::uint32_t ticks() const { return steps_; }
  std::uint32_t late_ticks() const { return late_; }
  // The fit of the points measured, and besides its own checks, the
  // shaft's: its inertia within a factor 2 of the one set up, and no
  // damping besides the back-EMF's that would take more than 2 % off
  // L / R (b L / (R J): a hand on the knob); and the ticks': no more than
  // kLateShare of them late. None when the shaft wandered off.
  WindingFit fit() const;

 private:
  void close_point();

  Setup s_;
  int count_ = 0, point_ = 0;
  std::uint32_t period_us_[kMaxPoints] = {};
  std::uint32_t measure_us_[kMaxPoints] = {};
  float volts_[kMaxPoints] = {};
  WindingPoint pts_[kMaxPoints];
  std::uint32_t t0_ = 0;  // when the point began
  // The least squares of the angle on 1, t, cos, sin over the point's
  // measurement: the sums of the basis' products, and of its products with
  // the angle (from where it was first read), and of the angle squared.
  double gram_[4][4] = {};
  double rhs_[4] = {};
  double sq_ = 0.0;
  double theta0_ = 0.0;
  std::uint32_t samples_ = 0;
  // The ticks: when the last came, how many, how many late; where the
  // shaft was first read, and whether it turned more than kWander from it.
  std::uint32_t last_ = 0, steps_ = 0, late_ = 0;
  double home_ = 0.0;
  bool wandered_ = false;
};

}  // namespace tuushin::fw
