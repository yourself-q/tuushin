#pragma once

namespace tuushin::control {

// Slave-side energy monitor of the TDPA that considers energy reflection:
// M. Panzirsch, J.-H. Ryu and M. Ferre, "Reducing the conservatism of the time
// domain passivity approach through consideration of energy reflection in
// delayed coupled network systems", Mechatronics 58 (2019) 58-69, "Approach 3".
//
// The conventional TDPA (tdpa.hpp) keeps the delayed channel passive in each
// direction separately. A position coupling on the slave side stores energy:
// the master compresses the coupling spring against a wall (input in the
// master-to-slave direction) and the spring later pushes the master back out
// (output in the slave-to-master direction). Booked per direction, that
// returning energy looks like energy the channel generated, so the passivity
// controllers dissipate it and the master feels the wall vanish while it
// pulls out (Panzirsch et al., Exp2), or both sides block each other.
//
// Approach 3 moves the coupling controller inside the passivity-controlled
// two-port, between the master device port and the slave device port, and
// keeps a single direction-independent budget next to the controller:
//
//   E_St(k) = E_in^M(k - T1) + E_in^S(k) - E_out^S(k) - G(k)
//
// E_in^M is the master's input energy at its device port as last reported,
// E_in^S and E_out^S are the energies that entered from and left towards the
// slave device, and G is the energy granted to the master so far. Each sample
// the predicted outputs towards the slave and towards the master pass in full
// when E_St covers them; otherwise both are scaled by the same factor, i.e.
// the excess is split in proportion to each output (eqs. 26-28). The master's
// passivity controller limits its device output to the grants it received:
//
//   E_out^M(k) + E_out^S(k) <= G(k - T2) + E_out^S(k)
//                           <= E_in^M(k - T1) + E_in^S(k)
//                           <= E_in^M(k) + E_in^S(k),
//
// so the two-port between the two devices is passive for any delay, delay
// variation, reordering or loss: all three quantities are cumulative and only
// grow, so a late or missing report can only understate them.
//
// The paper books the master side at the channel port (its port 2); this
// class takes whatever cumulative input energy the master reports, and the
// simulator reports the master device port (port 1), which includes the
// master PC in the two-port and is equally valid.
class ReflectiveStore {
 public:
  // `initial_energy` (J, >= 0) is stored before anything entered, like a
  // spring that starts compressed: the two-port is then passive with respect
  // to that initial storage, E_out(k) <= E_in(k) + initial_energy for all k,
  // which still bounds the energy it can ever generate. Energy tanks in
  // teleoperation are usually started this way (M. Franken, S. Stramigioli,
  // S. Misra, C. Secchi and A. Macchelli, "Bilateral telemanipulation with
  // time delays: a two-layer approach combining passivity and transparency",
  // IEEE T-RO 27(4), 2011). It lets the slave start following before the
  // master's first energy report has crossed the channel.
  explicit ReflectiveStore(double initial_energy = 0.0)
      : initial_(initial_energy > 0.0 ? initial_energy : 0.0) {}

  struct Grant {
    double slave = 0.0;    // J that may leave towards the slave this sample
    double master = 0.0;   // J granted to the master this sample (booked)
    bool limited = false;  // the requests exceeded the store and were scaled
  };

  // Records the master's cumulative input energy. Smaller values (stale or
  // reordered reports) are ignored.
  void on_master_input_energy(double e) {
    if (e > master_in_) master_in_ = e;
  }

  // Books energy realized at the slave device port, positive when it entered
  // the two-port (the slave pushed against the coupling). As with
  // EnergyPort::account_energy, book the work the held force actually did,
  // f(k-1) * (x(k) - x(k-1)), not a prediction.
  void account_slave_energy(double e) {
    if (e > 0.0) {
      slave_in_ += e;
    } else {
      slave_out_ -= e;
    }
  }

  // E_St. It can be slightly negative after the slave device moved further
  // than predicted during a sample; nothing is granted until inputs cover it.
  double level() const { return initial_ + master_in_ + slave_in_ - slave_out_ - granted_; }

  // Splits the store between this sample's predicted outputs (J, negative
  // values count as zero). The master's share is booked immediately because
  // it leaves for the master and cannot be taken back; the slave's share is
  // only an allowance for the slave PC, and the realized output is booked
  // through account_slave_energy on the next sample.
  //
  // `reserved` (J) is what the slave device may still take out under the
  // work caps it was sent (teleop::Device1D::work_reserved) beyond what is
  // booked. The store will pay it whatever the PC grants now, so the master
  // gets none of it: its share is at most level() - reserved, and the
  // slave's grows into what that leaves.
  Grant allocate(double want_slave, double want_master, double reserved = 0.0);

  // Cumulative energy granted to the master. Send it with every backward
  // packet; the master uses it as the remote input energy of its EnergyPort.
  double granted() const { return granted_; }

  double initial_energy() const { return initial_; }
  double master_input_energy() const { return master_in_; }
  double slave_input_energy() const { return slave_in_; }
  double slave_output_energy() const { return slave_out_; }

 private:
  double initial_;
  double master_in_ = 0.0;
  double slave_in_ = 0.0;
  double slave_out_ = 0.0;
  double granted_ = 0.0;
};

}  // namespace tuushin::control
