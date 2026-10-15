#pragma once

#include <cstdint>

#include "tuushin/codec/deadband.hpp"
#include "tuushin/control/predictive_pc.hpp"
#include "tuushin/control/reflection.hpp"
#include "tuushin/control/tdpa.hpp"
#include "tuushin/time.hpp"

namespace tuushin::teleop {

// The two ends of a one-degree-of-freedom bilateral teleoperator in the
// velocity-force architecture of the TDPA literature, without any I/O:
//
//   master device --velocity, position, energy--> slave controller -> robot
//   master device <------force, energy----------- slave controller
//
// Each controller is stepped once per sample with the device's encoder
// position and fed whatever samples arrived from the other end. The same
// classes run inside the simulator (sim/teleop1d.hpp), where the network is a
// model, and inside the networked nodes (teleop/node.hpp), where it is UDP;
// nothing in them depends on which.

// How passivity is enforced; see sim::Teleop1DConfig for the comparison.
//  kNone       no passivity controllers.
//  kChannel    conventional TDPA on the delayed channel in each direction
//              (Ryu & Preusche, ICRA 2007): impedance-type PC at the master,
//              admittance-type PC on the velocity command at the slave, which
//              integrates the corrected velocity and therefore drifts.
//  kReflective one budget at the slave covering the channel and the coupling
//              controller (control/reflection.hpp; Panzirsch et al.'s
//              "Approach 3"); impedance-type PCs at both device ports.
enum class Passivity { kNone, kChannel, kReflective };

// One haptic sample as the two ends exchange it.
//  master -> slave: value = velocity (m/s), aux = position (m),
//                   energy = the master's cumulative input energy (J).
//  slave -> master: value = force (N), aux = free for monitoring,
//                   energy = cumulative energy the slave grants the master
//                   (kReflective) or its channel input energy (kChannel).
struct Sample {
  double value = 0.0;
  double aux = 0.0;
  double energy = 0.0;
};

struct MasterConfig {
  Nanos dt = kNanosPerMilli;  // sample period
  Passivity passivity = Passivity::kReflective;
  // Impedance-type PC with a device model (control/predictive_pc.hpp)
  // instead of the constant-velocity assumption.
  bool predictive_pc = true;
  double model_mass = 0.2;  // kg, the mass the PC believes the device has
  int pc_hold_samples = 2;
  // Actuator limit in N, 0 for none. The controller clamps itself so that it
  // books the force the device really applies.
  double max_force = 20.0;
  // The encoder's resolution in m, 0 for an exact encoder. With one, every
  // new force is booked with its quantization tax, the device model becomes
  // a Kalman filter, and the velocity sent to the slave is the filter's
  // instead of the backward difference (control/predictive_pc.hpp).
  // `force_rate` (N/s) is how fast the filter expects the operator's push
  // to change; with `adapt_force_rate` it is the least, and the filter
  // raises it as far as its innovations show it to be overconfident. The
  // PC then predicts with the lower bound of `pc_confidence` standard
  // deviations, and a rise of the force repays its tax over
  // `pc_tax_samples` samples instead of within the coming one, which a
  // master pressed slower than a count per sample cannot pay.
  double encoder_resolution = 0.0;
  double force_rate = 20.0;
  bool adapt_force_rate = true;
  double pc_confidence = 1.0;
  int pc_tax_samples = 4;
  bool deadband = false;
  codec::DeadbandParams deadband_params{0.1, 0.002, from_millis(100)};
};

class MasterController {
 public:
  explicit MasterController(const MasterConfig& cfg);

  // A sample from the slave with its sequence number. Only the newest sample
  // (by sequence number) is displayed; every sample's energy counts, since
  // energies are cumulative and the largest one is the newest information.
  void receive(std::uint32_t seq, const Sample& s);

  struct Step {
    // Force the operator feels, positive when it pushes the hand towards
    // negative x. The device applies -display_force along +x.
    double display_force = 0.0;
    bool pc_active = false;
    bool send = false;  // false when the deadband suppressed the sample
    Sample sample;      // for the slave, valid when send is set
  };

  // One sample at time `now` with the encoder at `position`. Books the work
  // the previously applied force did since the last call; `cut_after` is the
  // device's Device1D::force_cut_after() (0 when the force stayed on).
  //
  // Samples need not come at exactly dt: the velocity and the device model
  // use the time that really passed, and a sample far from dt (the loop was
  // descheduled) or one during which the watchdog cut the force makes the
  // predictive PC apply no force for two samples (control::DevicePredictor).
  // A force that was cut did work only until the cut, over a displacement
  // no encoder read; the device model that chose the force predicts it, and
  // only an output is booked, since entering energy is credited only when
  // measured.
  Step step(Nanos now, double position, Nanos cut_after = 0);

  // The velocity sent with the last step: the backward difference of the
  // encoder positions, or the device model's estimate with an encoder
  // resolution set and a regular history.
  double velocity() const { return v_sent_; }
  // The force the newest slave sample asks the master to display (0 before
  // one arrived); display_force is what the passivity controller and the
  // actuator limit leave of it.
  double requested_force() const { return has_value_ ? f_reflected_ : 0.0; }
  bool has_slave_sample() const { return has_value_; }
  // Energy books at the master device port (power into the teleoperator).
  const control::EnergyPort& port() const { return port_; }
  std::uint64_t samples_sent() const { return sent_; }

 private:
  MasterConfig cfg_;
  double dt_;
  control::EnergyPort port_;
  control::DevicePredictor dev_;
  codec::DeadbandEncoder<1> enc_;
  bool first_ = true;
  Nanos last_now_ = 0;
  double x_sampled_ = 0.0;
  double v_meas_ = 0.0;
  double v_sent_ = 0.0;
  double f_display_ = 0.0;
  bool has_value_ = false;
  std::uint32_t last_seq_ = 0;
  double f_reflected_ = 0.0;
  std::uint64_t sent_ = 0;
};

struct SlaveConfig {
  Nanos dt = kNanosPerMilli;
  Passivity passivity = Passivity::kReflective;
  double kp = 2000.0;  // N/m, position coupling to the command
  double kd = 60.0;    // N s/m, velocity coupling to the command
  // Report the measured contact force instead of the coupling force. See
  // sim::SlaveParams for why the coupling force is the default.
  bool report_contact_force = false;
  // kChannel only: position-drift compensation gain in 1/s, 0 to disable,
  // and what the drift is measured against (sim::SlaveParams).
  double drift_gain = 0.0;
  enum class DriftReference { kSlavePosition, kCommand };
  DriftReference drift_reference = DriftReference::kCommand;
  // kReflective: energy in the store before anything entered (J).
  double initial_energy = 0.0;
  bool predictive_pc = true;
  double model_mass = 1.0;  // kg
  int pc_hold_samples = 2;
  // As in MasterConfig. The velocity the coupling damps is still the one
  // passed to step().
  double encoder_resolution = 0.0;  // m
  double force_rate = 20.0;         // N/s
  bool adapt_force_rate = true;
  double pc_confidence = 1.0;
  int pc_tax_samples = 4;
  bool deadband = false;
  codec::DeadbandParams deadband_params{0.1, 0.05, from_millis(100)};
  // Between samples the slave extrapolates the master position with the last
  // velocity, which bridges deadband gaps and single losses. When no sample
  // has arrived for longer than this, it stops extrapolating and holds the
  // last estimate instead, so that a dead link cannot drag the robot along
  // the last velocity. 0 extrapolates indefinitely; it must exceed the
  // deadband's max_interval to leave the deadband alone.
  Nanos hold_after = 0;
};

class SlaveController {
 public:
  // `position` is the slave's encoder reading at start; the command starts
  // there and moves once the master's samples arrive.
  SlaveController(const SlaveConfig& cfg, double position);

  void receive(std::uint32_t seq, const Sample& s);

  struct Step {
    double force = 0.0;  // applied by the slave actuator along +x
    bool pc_active = false;
    bool send = false;
    Sample sample;  // for the master, valid when send is set
  };

  // One sample at time `now` with the encoder at `position`, the device's
  // velocity estimate `velocity` (the damping term of the coupling acts on
  // it) and the force sensor (if any) at `contact_force`, which is only used
  // when the slave reports it. Irregular samples and `cut_after` are handled
  // as in MasterController::step.
  Step step(Nanos now, double position, double velocity, double contact_force,
            Nanos cut_after = 0);

  // Diagnostics of the last step.
  double command() const { return x_cmd_; }            // coupling target (m)
  double master_estimate() const { return x_master_est_; }
  double requested_velocity() const { return v_requested_; }
  double command_velocity() const { return v_c_; }
  double reported_force() const { return f_slave_; }
  bool has_master_sample() const { return has_value_; }
  // Net energy that entered through the slave device port, booked from the
  // realized work of the applied force (predicted, for a sample in which
  // the watchdog cut it), less the quantization tax.
  double terminal_net() const { return terminal_net_; }
  // kReflective: the store. kNone and kChannel: the channel end at the slave.
  const control::ReflectiveStore& store() const { return store_; }
  const control::EnergyPort& channel_port() const { return port_; }
  std::uint64_t samples_sent() const { return sent_; }

 private:
  SlaveConfig cfg_;
  double dt_;
  control::EnergyPort port_;
  control::ReflectiveStore store_;
  control::DevicePredictor dev_;
  codec::DeadbandEncoder<1> enc_;
  double x_sampled_;
  double x_cmd_;
  double x_master_est_;
  double v_requested_ = 0.0;
  double v_c_ = 0.0;
  double f_ctrl_ = 0.0;
  double f_slave_ = 0.0;
  double terminal_net_ = 0.0;
  bool has_value_ = false;
  bool fresh_ = false;
  std::uint32_t last_seq_ = 0;
  Sample last_;
  Nanos fresh_at_ = 0;  // when the newest sample was first used
  bool stepped_ = false;
  Nanos last_now_ = 0;
  std::uint64_t sent_ = 0;
};

}  // namespace tuushin::teleop
