#pragma once

namespace tuushin::control {

// The variance of an encoder's reading error, estimated online from the
// readings themselves.
//
// Why: DevicePredictor and HeldDeviceFilter take a reading's error to be
// the quantization's, uniform over a count: q^2 / 12. A magnetic encoder
// (MT6701, AS5047P) also dithers, by about half a count, and then reads
// twice as noisily as that. A Kalman filter told less trusts every reading
// too much: its innovations come out larger than it expects, covariance
// matching takes that for a hand that pushes faster and raises H's random
// walk to its bound, and the predictions spread several times wider than
// they need to, which the predictive PC pays for in force.
//
// How: the second difference of three readings is the change of velocity
// over the two samples plus e0 - 2 e1 + e2, whose variance is 6 sigma^2 for
// independent errors of variance sigma^2 (for samples of lengths h1 and
// h2, the difference of the two mean velocities has sigma^2 (1/h1^2 +
// (1/h1 + 1/h2)^2 + 1/h2^2)): the difference-based variance estimators of
// nonparametric regression (J. Rice, "Bandwidth choice for nonparametric
// regression", Ann. Statist. 12(4), 1984; T. Gasser, L. Sroka and
// C. Jennen-Steinmetz, "Residual variance and residual pattern in nonlinear
// regression", Biometrika 73(3), 1986). Two things keep them from applying
// as they are:
//
//  * Quantization alone does not make independent errors. A device moving
//    by less than a count per sample reads a sawtooth, whose second
//    differences are 0 or one count: 1.4 times q^2 / 12 by their mean, 4.4
//    times by their median, on a 55 um encoder under the simulated hand.
//    So only what lies beyond one count is counted: u = (|z| - q / sqrt 6)^2
//    for the second difference z scaled to a reading's error, whose mean
//    for a Gaussian z of variance sigma^2 is 2 sigma^2 ((1 + t^2) Q(t) -
//    t phi(t)) with t = q / (sqrt 6 sigma), the truncated second moment;
//    the estimate is the sigma that explains the mean of u. A sawtooth
//    shows no u at all, and dither of half a count makes the errors
//    independent and the sum of three nearly Gaussian.
//  * Where the device accelerates the second difference shows that too: on
//    an encoder of a micrometre, a robot's acceleration of 1 m/s^2 is a
//    count. The change of velocity the forces the model knows of explain
//    (the port force, and its estimate of everything else) is subtracted
//    first; what it misses, as at an impact, is an outlier, and u is
//    clipped at nine times the current variance.
//
// The mean of u is a running mean of time constant `time_constant`. On
// readings quantized to 7.7 and 55 um under 0.1 to 2 counts of Gaussian
// noise, moved as by a hand, the estimate of sigma came out within 6 % of
// sqrt(noise^2 + q^2 / 12), within 9 % 95 % of the time, and on quantized
// readings without noise at q^2 / 12 exactly, which it never goes below.
class ReadingNoise {
 public:
  // `dt` the nominal sample period (s), `resolution` the encoder's (m,
  // positive), `time_constant` how fast the estimate follows (s).
  ReadingNoise(double dt, double resolution, double time_constant = 0.25);

  // The displacement the readings showed over a sample of length h (s), and
  // how much the device's mean velocity is expected to have changed from
  // the sample before (m/s; DevicePredictor::expected_velocity_change). A
  // sample further than dt / 2 from dt breaks the chain of readings.
  void observe(double dx, double h, double expected_change = 0.0);
  // A sample the readings cannot be compared across, such as one in which
  // the force was cut: breaks the chain.
  void interrupt() { have_prev_ = false; }

  // The estimated variance of one reading's error, m^2, at least q^2 / 12.
  double variance() const { return var_; }
  // Second differences seen so far.
  long samples() const { return samples_; }

 private:
  double dt_;
  double floor_;      // q^2 / 12
  double threshold_;  // q / sqrt 6
  double rate_;       // 1 / time_constant
  double mean_u_ = 0.0;
  double var_;
  bool have_prev_ = false;
  double prev_dx_ = 0.0, prev_h_ = 0.0;
  long samples_ = 0;
};

}  // namespace tuushin::control
