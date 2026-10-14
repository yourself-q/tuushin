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
//    device's response and makes the controller more conservative.
// Sign convention: power into the controlled two-port is phi * dx, and the
// device accelerates by -phi / m. At a haptic master phi is the displayed
// force when it pushes against the positive direction; at a slave robot phi
// is minus the force the controller applies.
class DevicePredictor {
 public:
  DevicePredictor(double mass, double dt);

  // Call once per sample with the displacement realized over the last sample
  // and the port force that was held during it.
  void observe(double dx, double phi_held);

  // Coefficients of the predicted displacement dx = base() - compliance() * phi.
  double base() const;
  double compliance() const { return c_; }
  double displacement(double phi) const { return base() - c_ * phi; }
  double work(double phi) const { return phi * displacement(phi); }

 private:
  double c_;
  double dx1_ = 0.0, dx2_ = 0.0;      // last two realized displacements
  double phi1_ = 0.0, phi2_ = 0.0;    // forces held during them
};

// Scales `phi` towards zero so that the predicted work stays above -budget
// (budget in J; negative values count as zero). The result's gain is the
// fraction of the requested force that was removed, 1 - lambda.
PcResult limit_force_predictive(double budget, double phi, const DevicePredictor& device);

}  // namespace tuushin::control
