#pragma once

namespace tuushin::control {

// Time Domain Passivity Approach (TDPA) for a delayed communication channel.
//
// Formulation: J.-H. Ryu and C. Preusche, "Stable bilateral control of
// teleoperators under time-varying communication delay: time domain
// passivity approach", ICRA 2007, reviewed in J.-H. Ryu, "Stable
// teleoperation with time domain passivity approach", IFAC 2008, eqs. (8)-(16).
//
// The channel between master and slave is a two-port. At each port the
// power flowing *into* the channel is split by sign into input energy and
// output energy. Passivity of the channel is guaranteed when, at every
// sample,
//
//   E_out(slave)  <= E_in(master) as last received at the slave,  and
//   E_out(master) <= E_in(slave)  as last received at the master.
//
// Input energy only grows, so a delayed, lost or reordered report can only
// understate the peer's input energy. The check therefore stays sufficient
// under time-varying delay and packet loss; the price is conservatism.
//
// Outputs beyond the peer's input. A port that books the work its device
// actually did (account_energy) learns of an output one sample after the
// controller allowed it, and it can exceed what the controller predicted:
// a model error, a coarse encoder's noise, a force that was cut. The excess
// is energy this end owes. If the port kept reporting all its input to the
// peer, the peer would spend that input a second time, and the excess would
// stay owed until the peer's own input covered it. In the reflective scheme
// the peer's "input" is a grant the slave's store makes only for the work
// it expects the reflected force to do at the master, so the debt can last
// for seconds and the peer keeps spending: with a 55 um encoder at the
// master a 50 ms stall left the master 8.7 mJ beyond its grants and the
// devices 1.9 mJ in deficit a second later.
//
// So this port pays its own debt first. Input booked while the outputs
// exceed the peer's input is kept back (covered_energy) instead of being
// reported, and it counts towards this port's own budget. Then
//
//   E_out(this) <= E_in(peer) + covered        (this port's PC),
//   E_out(peer) <= E_in(this) - covered        (the peer's PC),
//
// and the two sum to the condition for the whole channel. An output beyond
// both is still possible for a sample (that is what a prediction error is);
// it is paid back from the next input of either end, and never reported.
class EnergyPort {
 public:
  // Energy that has entered the channel through this port.
  double input_energy() const { return in_; }
  // Energy that has left the channel through this port.
  double output_energy() const { return out_; }
  // Input energy that paid for this port's own outputs beyond the peer's
  // input, and was therefore not reported.
  double covered_energy() const { return covered_; }
  // Input energy the peer may spend, input_energy() - covered_energy().
  // Send it to the peer; it only grows.
  double reported_input_energy() const { return reported_; }

  // Records the peer's input energy. Values that are not larger than the
  // current one (stale or reordered packets) are ignored.
  void on_remote_input_energy(double e) {
    if (e > remote_in_) remote_in_ = e;
  }
  double remote_input_energy() const { return remote_in_; }

  // Energy that may still leave the channel here without violating passivity.
  double budget() const {
    const double b = remote_in_ + covered_ - out_;
    return b > 0.0 ? b : 0.0;
  }

  // Books one sample of power flowing into the channel (W, positive = input).
  void account(double power, double dt) { account_energy(power * dt); }

  // Books energy measured directly (J, positive = into the channel). Use it
  // where a physical device meets the sampled controller: the work a
  // zero-order-held force f(k-1) did over the last sample is exactly
  // f(k-1) * (x(k) - x(k-1)), whereas f(k) * v(k) * dt assumes the velocity
  // stayed constant over the sample. See S. Stramigioli, C. Secchi,
  // A. J. van der Schaft and C. Fantuzzi, "Sampled data systems passivity and
  // discrete port-Hamiltonian systems", IEEE Trans. Robotics 21(4), 2005.
  void account_energy(double e) {
    if (e > 0.0) {
      in_ += e;
      const double debt = out_ - remote_in_ - covered_;
      const double paid = debt <= 0.0 ? 0.0 : e < debt ? e : debt;
      covered_ += paid;
      reported_ += e - paid;
    } else {
      out_ -= e;
    }
  }

 private:
  double in_ = 0.0;
  double out_ = 0.0;
  double covered_ = 0.0;
  double reported_ = 0.0;
  double remote_in_ = 0.0;
};

// Result of one passivity-controller step.
struct PcResult {
  double value = 0.0;   // corrected controller output
  double gain = 0.0;    // damping inserted this step (alpha or beta), 0 if idle
  bool active = false;  // true when the controller had to modify the output
};

// Passivity controller for an impedance-type port: velocity `v` is measured
// and force `f` is the output (the master side when the slave reflects force,
// or both sides in a position-position architecture). Power into the channel
// is f * v. When this sample would let more energy out than the budget allows,
// a parallel damper f' = f + alpha * v (alpha > 0) dissipates the excess.
// The sample is booked on `port` in either case.
PcResult passivate_force(EnergyPort& port, double f, double v, double dt);

// The impedance-type controller of passivate_force without the booking, for
// ports whose energy is booked afterwards from the realized displacement with
// EnergyPort::account_energy. `v_est` (for example the backward difference of
// the encoder position) predicts the displacement over the coming sample.
// Booking the realized work closes the gap between prediction and reality
// one sample later: if the prediction was too optimistic the deficit shows up
// in the books and the controller blocks output until input energy covers it.
PcResult limit_force(const EnergyPort& port, double f, double v_est, double dt);

// The same controller against an explicit budget in joules, for schemes that
// keep their energy books outside an EnergyPort (see reflection.hpp).
PcResult limit_force(double budget, double f, double v_est, double dt);

// Passivity controller for an admittance-type port: force `f` is measured and
// velocity `v` is the output (the slave side when the master sends velocity).
// Power into the channel is -f * v. When the budget would be exceeded, a
// series damper v' = v - beta * f (beta > 0) dissipates the excess.
// The sample is booked on `port` in either case.
PcResult passivate_velocity(EnergyPort& port, double v, double f, double dt);

}  // namespace tuushin::control
