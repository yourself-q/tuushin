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
// Sign convention: power into the controlled two-port is phi * dx, and the
// device accelerates by -phi / m. At a haptic master phi is the displayed
// force when it pushes against the positive direction; at a slave robot phi
// is minus the force the controller applies.
class DevicePredictor {
 public:
  DevicePredictor(double mass, double dt);

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

  // Estimates recovered from the history: everything acting on the device
  // except the port force (the operator's hand, gravity, friction) in N, and
  // the velocity at the start of the coming sample times dt.
  double external_force() const;
  double velocity_dt() const;

 private:
  void push(double dx, double phi_held, double r, bool regular);

  double dt_;
  double c_;
  double dx1_ = 0.0, dx2_ = 0.0;      // last two realized displacements
  double phi1_ = 0.0, phi2_ = 0.0;    // forces held during them
  double r1_ = 1.0, r2_ = 1.0;        // their lengths in units of dt
  int regular_ = 2;                   // regular samples since an irregular one, up to 2
};

// Scales `phi` towards zero so that the predicted work over the coming
// sample, and over `hold_samples` samples if that is larger than one, stays
// above -budget (budget in J; negative values count as zero). The result's
// gain is the fraction of the requested force that was removed, 1 - lambda.
// While the device is not ready() the result is zero force.
PcResult limit_force_predictive(double budget, double phi, const DevicePredictor& device,
                                int hold_samples = 2);

}  // namespace tuushin::control
