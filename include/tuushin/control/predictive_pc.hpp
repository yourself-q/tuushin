#pragma once

#include "tuushin/control/tdpa.hpp"

namespace tuushin::control {

// Impedance-type passivity control for a force-controlled device, with the
// device's response to the force itself included in the energy prediction.
//
// Why: a force held over a sample (zero-order hold) does the work
// phi * (x(k+1) - x(k)) on the device, and a passivity controller has to
// decide phi before that displacement is known. The classical controllers
// (tdpa.hpp, and J.-H. Ryu, D.-S. Kwon and B. Hannaford, IEEE T-RA 2002)
// assume the velocity stays constant over the sample. On a light device that
// fails exactly when it matters: a 10 N force switched onto a 0.2 kg master
// that creeps at 1 mm/s reverses it within the 1 ms sample, so a sample the
// controller expected to be an input turns out to be an output. Booked from
// the realized displacement (EnergyPort::account_energy) the deficit is
// visible, but a controller that lets predicted inputs through keeps adding
// to it, and the force chatters between zero and full. J.-H. Ryu, Y. S. Kim
// and B. Hannaford, "Sampled- and continuous-time passivity and stability of
// virtual environments", IEEE T-RA 20(4), 2004, likewise drop the
// constant-velocity assumption.
//
// Model: the device is a mass m driven by the port force and by everything
// else (the operator's hand, gravity, friction), which changes little over
// two samples. With F_j = H - phi_j the net force during sample j, a mass
// under zero-order hold moves
//
//   dx_j - dx_{j-1} = (F_j + F_{j-1}) * dt^2 / (2 m),
//
// and subtracting the same relation one sample earlier eliminates both the
// unknown H and the unknown velocity:
//
//   dx_k = 2 dx_{k-1} - dx_{k-2} - (phi_k - phi_{k-2}) * dt^2 / (2 m)
//        = P - c * phi_k,          c = dt^2 / (2 m).
//
// The work done on the port over the coming sample is then the concave
// quadratic W(phi) = phi * (P - c * phi). Scaling the requested force by
// lambda in [0, 1] keeps W(lambda phi) >= -budget for every lambda up to the
// positive root, which is computed in closed form.
//
// Consequences worth knowing:
//  * lambda = 0 is always feasible and does exactly zero work, so a deficit
//    can only grow through model error, not through the controller's logic.
//  * With no budget the force may still be held while the device is at rest
//    (W = 0) and may rise by about m * v / dt per sample while the device
//    moves into it, so the force ramps up instead of jumping. This is the
//    behaviour gradient-limited passivity controllers are designed to have;
//    here it falls out of the device dynamics.
//  * The mass is a model parameter. Underestimating it overestimates the
//    device's response and makes the controller more conservative; in the
//    1-DoF simulator (sim/teleop1d.hpp, reflective scheme, 10-100 ms) half
//    the true mass and +-20 % keep the deficit below 0.1 mJ, twice the true
//    mass lets it reach tens of mJ. When in doubt, err light.
//
// Holding horizon. The largest force with W >= -budget over one sample is,
// without budget, the force that brings the device's mean displacement over
// the sample to zero: the device enters the sample moving into the force and
// leaves it moving out at the same speed. The next sample then has to drop
// the force to zero, the hand pushes the device back in, and the force
// chatters between zero and full at half the sample rate while the books
// stay balanced. The classical TDPA has the same on/off behaviour; H. Singh,
// A. Jafari and J.-H. Ryu, "Enhancing the force transparency of time domain
// passivity approach: observer-based gradient controller", ICRA 2019, attack
// it with an observer as well (their method is not reproduced here).
// Requiring in addition that the force stays affordable when held for n
// samples removes the reversal. With the same model the displacement over n
// held samples is
//
//   D_n = n * v * dt + n^2 * c * (H - phi),
//
// where v and H are recovered from the history like P above, and the work is
// again a concave quadratic in phi. For n = 2 and no budget the bound is
// H + m * v / dt, the force that stops the device within one sample: contact
// without stored energy becomes perfectly inelastic instead of bouncing, and
// at rest the force follows the operator's push. Larger n only soften it.
//
// Samples of unequal length. A real-time loop that is a fraction of a period
// late shortens the next sample by as much, and the recursion above, which
// assumes every sample lasts dt, then misreads the velocity in proportion:
// a 0.68 ms sample read as 1 ms underestimates it by a third, and so the
// output a large force will draw from a device moving into it. With the
// actual lengths h_1 (last sample) and h_2 (the one before) the same model,
// solved for the two unknowns, gives
//
//   H = (2 m (dx_1 / h_1 - dx_2 / h_2) + phi_1 h_1 + phi_2 h_2) / (h_1 + h_2),
//   v = dx_1 / h_1 + (H - phi_1) h_1 / (2 m),
//
// which reduce to the expressions above when h_1 = h_2 = dt. The coming
// sample is predicted at the nominal length; if it turns out longer, the
// device's watchdog bounds how long the force acts (teleop::Device1D).
//
// Irregular samples. The model takes everything except the port force to be
// constant over the history; that fails for a sample much longer than dt
// (the loop was descheduled: after a 50 ms stall dx_1 is fifty samples'
// worth of motion under a hand that has moved on), and the recursion fails
// for a sample during which the device's watchdog cut the force. Samples
// further than half a period from dt, and samples observed as irregular, put
// the predictor out of ready() until two regular samples have refilled the
// history; until then limit_force_predictive applies no force, which does
// exactly zero work whatever the device does. At 1 kHz that is 2 ms without
// force after a stall of tens of milliseconds. Samples much shorter than dt
// count as irregular too: on a real encoder dx_1 / h_1 amplifies the
// quantization by dt / h_1.
//
// Encoder quantization. Everything above takes the encoder to be exact. A
// real one reads the position to a resolution q (about 55 um on a
// 3D Systems Touch, 10 um on a Force Dimension omega), and that breaks two
// things.
//
//  * The books. Work booked from readings y = x + e, phi_k (y_{k+1} - y_k),
//    differs from the work done, phi_k (x_{k+1} - x_k), by phi_k times the
//    change of the reading error e. Summed by parts the difference is
//    sum_k e_k (phi_{k-1} - phi_k) plus the end terms: every change of the
//    force is booked against a position error of up to q/2, and nothing
//    makes those errors cancel. A force that chatters between 0 and 10 N
//    while the reading dithers between two counts draws energy the books
//    show as entering. quantization_tax() bounds the term sample by sample,
//    and the controllers book it as an output, which makes their books a
//    lower bound on the true work for any reading error confined to an
//    interval of width q (rounding, truncation, a constant offset). The
//    tax charges only increases of |phi| (and the whole force when its sign
//    flips), so a force that rises once and is held pays q |phi| once.
//    J. J. Abbott and A. M. Okamura, "Effects of position quantization and
//    sampling rate on virtual-wall passivity", IEEE T-RO 21(5), 2005, and
//    N. Diolaiti et al., "Stability of haptic rendering: discretization,
//    quantization, time delay, and Coulomb effects", IEEE T-RO 22(2), 2006,
//    show the same mechanism for virtual walls: quantization alone can make
//    a sampled wall non-passive.
//  * The prediction. The recursion above differentiates the readings twice:
//    one count in dx_1 moves the estimate of H by up to 2 m q / dt^2, which
//    is 22 N for a 0.2 kg device with a 55 um encoder at 1 kHz. With a
//    resolution given, the predictor estimates v and H with a Kalman filter
//    on the same model instead, as P. R. Belanger et al., "Estimation of
//    angular velocity and acceleration from shaft-encoder measurements",
//    IJRR 17(11), 1998, do for encoders: the state is the position, the
//    velocity and H; the reading error is taken as uniform over a count
//    (variance q^2 / 12), and H as a random walk whose change over a sample
//    has standard deviation force_rate * dt. The predicted displacement is
//    still that of the device, not of its readings; the difference between
//    the two is what the tax pays for. Smoother estimates also mean fewer
//    and smaller force changes, and so a smaller tax.
//
// Repaying the tax. Without a budget the tax of a rise has to come from the
// input the new force draws. Paid within the coming sample, it needs the
// device to move more than q per sample into the force, 5.5 cm/s for a
// Touch at 1 kHz: on a device pressed more slowly a force cannot start from
// zero, so the master displays nothing, takes in nothing and never earns
// the budget that would let it. With tax_samples = N the one-sample and
// hold constraints leave the tax out, and a rise must instead repay it,
// with any deficit of the books, from the work predicted over N samples
// with the force held:
//
//   lambda * G_N - lambda^2 * C_N - q * (lambda |phi| - kept) >= -budget,
//
// with G_N and C_N the coefficients of the work over N samples above and
// kept the part of the held force with phi's sign. That lowers the
// threshold to about q / (N dt), 1.4 cm/s for N = 4. The books still take
// the tax at once; if the device does not come, they go into deficit by at
// most about one tax and block further rises until inputs repay it.
//
// Trusting the prediction. A horizon of N samples trusts the prediction
// over N samples, so its error matters. limit_force_predictive takes
// `confidence` standard deviations (hold_sd) off the predicted work: a
// lower confidence bound. That bound is only as good as the filter's idea
// of its own uncertainty, which rests on force_rate. Told 5 N/s while the
// hand's push changes faster, the filter misses the device's displacement
// by about fifteen of its standard deviations, told 20 N/s by about four,
// and a horizon of four samples on such predictions let the actuators
// generate up to 7 mJ at 50 ms. With adapt_force_rate the filter tunes its
// random walk to its innovations, the covariance matching of R. K. Mehra,
// "On the identification of variances and adaptive Kalman filtering",
// IEEE T-AC 15(2), 1970: a consistent filter's normalized innovation
// squared nu^2 / S averages 1, so the log of the walk's variance moves by
// 3/s times (min(nu^2 / S, 9) - 1), with the rate kept between force_rate
// and 16 force_rate. Its misses are then about one standard deviation.
//
// Measured in the 1-DoF simulator (reflective scheme, 55 um master and 1 um
// slave encoders, 20 s over nine hand motions): with the filter, the tax and
// the ports paying their own excess first (control/tdpa.hpp), the actuators
// never generate energy at 10, 50 or 100 ms each way. Controllers that take
// the readings as exact let the teleoperator generate up to 0.5 mJ, 0.8 J
// and 1.1 J. The price is transparency: with adaptation, confidence 1 and
// the tax repaid over four samples the master conveys 40, 17 and 8 % of the
// contact force at 10, 50 and 100 ms, against 77, 29 and 13 % with exact
// encoders. With the tax prepaid it conveyed 32, 12 and 4 %, and nothing
// at all on the three motions too slow to pay it.
//
// Sign convention: power into the controlled two-port is phi * dx, and the
// device accelerates by -phi / m. At a haptic master phi is the displayed
// force when it pushes against the positive direction; at a slave robot phi
// is minus the force the controller applies.
class DevicePredictor {
 public:
  // `resolution` is the encoder's resolution in m, 0 for an exact encoder
  // (the three-point recursion). `force_rate` is how fast the forces other
  // than the port force are expected to change, in N/s; it only matters
  // with a resolution, as the random walk of the Kalman filter's H. With
  // `adapt_force_rate` it is the least rate, and the filter raises it up to
  // 16 times as far as its innovations show it to be overconfident.
  DevicePredictor(double mass, double dt, double resolution = 0.0, double force_rate = 20.0,
                  bool adapt_force_rate = false);

  // Call once per sample with the displacement realized over the last sample,
  // the port force that was held throughout it, and its length h in seconds
  // (dt if omitted). A sample further than dt / 2 from dt is irregular.
  void observe(double dx, double phi_held) { observe(dx, phi_held, dt_); }
  void observe(double dx, double phi_held, double h);
  // A sample during which the port force was not held throughout, e.g. the
  // device's watchdog cut it.
  void observe_irregular(double dx, double phi_held);
  // True when the last two samples were regular. A new predictor is ready:
  // its history is that of a device at rest.
  bool ready() const { return regular_ >= 2; }

  double resolution() const { return q_; }
  // The rate the filter's random walk currently assumes: force_rate, or
  // with adapt_force_rate what the innovations have raised it to.
  double force_rate() const;
  // The port force held at the end of the last sample: the force observed
  // with it, or 0 after observe_irregular(), whose force was cut. The
  // quantization tax of the next force is measured from it.
  double held_force() const { return held_; }

  // Coefficients of the predicted displacement dx = base() - compliance() * phi.
  double base() const;
  double compliance() const { return c_; }
  double displacement(double phi) const { return base() - c_ * phi; }
  double work(double phi) const { return phi * displacement(phi); }

  // The same over n samples with phi held: D_n = hold_base(n) - n^2 c phi.
  // hold_base(1) equals base(). n need not be an integer: the model is a
  // mass under a constant net force, so it predicts any horizon.
  double hold_base(double n) const;
  double hold_compliance(double n) const { return n * n * c_; }
  double hold_displacement(double phi, double n) const {
    return hold_base(n) - hold_compliance(n) * phi;
  }
  double hold_work(double phi, double n) const { return phi * hold_displacement(phi, n); }
  // Standard deviation of hold_displacement() from the Kalman filter's
  // uncertainty about v and H; 0 without a resolution.
  double hold_sd(double n) const;

  // Estimates recovered from the history (or the Kalman filter, with a
  // resolution): everything acting on the device except the port force (the
  // operator's hand, gravity, friction) in N, and the velocity at the start
  // of the coming sample times dt.
  double external_force() const;
  double velocity_dt() const;

 private:
  void push(double dx, double phi_held, double r, bool regular);
  void filter(double dx, double phi_held, double r, bool regular);

  double dt_;
  double mass_;
  double c_;
  double dx1_ = 0.0, dx2_ = 0.0;      // last two realized displacements
  double phi1_ = 0.0, phi2_ = 0.0;    // forces held during them
  double r1_ = 1.0, r2_ = 1.0;        // their lengths in units of dt
  int regular_ = 2;                   // regular samples since an irregular one, up to 2
  double held_ = 0.0;
  // Kalman filter, used when q_ > 0. The position is kept relative to the
  // last reading (e_ = estimate - reading), so that it never grows large.
  double q_;
  double noise_;     // variance of a reading, q^2 / 12
  double h_walk_;    // variance of the change of H over a nominal sample
  double force_rate_;
  bool adapt_;
  double log_walk_ = 0.0;  // log of the factor adaptation applies to h_walk_
  double e_ = 0.0, v_ = 0.0, h_ = 0.0;
  double p_[3][3] = {};
};

// Energy (J) to book as an output when the port force goes from `prev`, the
// force held until now, to `phi`, on an encoder of resolution `q` (m):
// q * (|phi| - |prev|) when the force grows in the same direction, q * |phi|
// when it changes sign or starts from zero, and 0 when it shrinks. Booked
// for every new force, it bounds the work the readings misattribute (see
// DevicePredictor): with phi_{-1} = 0 the sum over samples 0..N-1 is
// (q / 2) (sum_k |phi_k - phi_{k-1}| + |phi_{N-1}|), the largest
// |sum_k phi_k (e_{k+1} - e_k)| can be when every reading error lies in one
// interval of width q.
double quantization_tax(double q, double prev, double phi);

// Scales `phi` towards zero so that the predicted work over the coming
// sample, and over `hold_samples` samples if that is larger than one, less
// `confidence` standard deviations of it (DevicePredictor::hold_sd), stays
// above -budget (budget in J; negative values count as zero). With an
// encoder resolution the quantization tax of moving from the held force to
// the result counts as well: against each of those predictions when
// `tax_samples` is 0, and otherwise only for a rise beyond the held force,
// which must repay it and any deficit (a negative budget) from the work
// predicted over `tax_samples` samples. The result's gain is the fraction
// of the requested force that was removed, 1 - lambda. While the device is
// not ready() the result is zero force.
PcResult limit_force_predictive(double budget, double phi, const DevicePredictor& device,
                                int hold_samples = 2, double confidence = 0.0,
                                int tax_samples = 0);

}  // namespace tuushin::control
