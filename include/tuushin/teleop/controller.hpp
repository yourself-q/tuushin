#pragma once

#include <algorithm>
#include <cstdint>
#include <deque>
#include <optional>
#include <utility>
#include <vector>

#include "tuushin/codec/deadband.hpp"
#include "tuushin/control/held_device.hpp"
#include "tuushin/control/noise_tax.hpp"
#include "tuushin/control/predictive_pc.hpp"
#include "tuushin/control/reading_noise.hpp"
#include "tuushin/control/reflection.hpp"
#include "tuushin/control/tdpa.hpp"
#include "tuushin/control/wall_model.hpp"
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
//  kModelMediated
//              no force crosses the channel: the slave estimates the wall it
//              touches and sends the model, the master renders it
//              (control/wall_model.hpp) through an impedance-type PC on its
//              own books, and the slave follows the master's position with
//              the force the master displays as a feedforward. The master's
//              device port is passive whatever the models are, to the PC's
//              prediction: a lossless rendered spring returns all the
//              operator put in, so the books stand at zero as a contact
//              ends, and the one-sigma prediction of the sample in which the
//              master turns overran them by 0.14 uJ against a 20 kN/m wall
//              in simulation, repaid by the next rise (pc_tax_samples);
//              pc_confidence 1.5 left no overrun. The slave's actuator is a
//              position servo with its own supply, and nothing books its
//              energy.
enum class Passivity { kNone, kChannel, kReflective, kModelMediated };

// One haptic sample as the two ends exchange it.
//  master -> slave: value = velocity (m/s), aux = position (m),
//                   energy = the master's cumulative input energy (J).
//  slave -> master: value = force (N), aux = free for monitoring,
//                   energy = cumulative energy the slave grants the master
//                   (kReflective) or its channel input energy (kChannel).
// kModelMediated:
//  master -> slave: value = velocity (m/s), aux = the position consistent
//                   with the model (MasterController::command_position),
//                   energy = the force the master displays (N).
//  slave -> master: value = the wall's stiffness (N/m, 0 before a contact),
//                   aux = its surface (m), energy = its damping (N s/m).
struct Sample {
  double value = 0.0;
  double aux = 0.0;
  double energy = 0.0;
};

// The port forces a device applies when its commands take effect `delay`
// after they are issued, as with a servo loop that picks them up on its own
// ticks or a bus that carries them. Both controllers book and predict with
// it; without a delay every sample holds the force commanded at its start.
class ActuatorSchedule {
 public:
  ActuatorSchedule(Nanos delay, Nanos dt);

  // The port forces over the sample from `from` to `to`, in order, with
  // their lengths in units of dt. If the watchdog cut the force `cut_after`
  // after `from`, when the last command was issued, the force is zero from
  // then on and nothing is in flight any more.
  //
  // advance() and lead() return buffers the schedule reuses, valid until
  // the next call of the same function: a control loop that estimates the
  // delay runs a schedule per candidate, and returning new vectors cost it
  // two heap allocations per candidate and sample, 137 per millisecond with
  // 65 candidates.
  const std::vector<control::ForceSegment>& advance(Nanos from, Nanos to, Nanos cut_after) {
    return advance(from, to, cut_after, from);
  }
  // The same with the last command issued at `issued`, which need not be
  // when the sample began: a device whose positions come over a link was
  // sampled some time before the host's tick (Device1D::sampled_at).
  const std::vector<control::ForceSegment>& advance(Nanos from, Nanos to, Nanos cut_after,
                                                    Nanos issued);
  // The forces in flight at `now`: what acts until a command issued now
  // takes effect. Empty without a delay.
  const std::vector<control::ForceSegment>& lead(Nanos now) { return lead(now, now); }
  // What acts from `from`, the end of the last sample, until a command
  // issued at `issue` takes effect.
  const std::vector<control::ForceSegment>& lead(Nanos from, Nanos issue);
  void command(Nanos now, double phi);
  // The delay of the commands issued from now on, as a device that
  // measures it reports it (Device1D::actuation_delay). A device applies
  // its commands in the order they came, so none takes effect before the
  // one issued before it, whatever the delays.
  void set_delay(Nanos delay) { delay_ = delay > 0 ? delay : 0; }
  Nanos delay() const { return delay_; }

 private:
  // When a command issued at `issue` takes effect.
  Nanos effective(Nanos issue) const {
    return pending_.empty() ? issue + delay_ : std::max(issue + delay_, pending_.back().first);
  }

  Nanos delay_;
  double dt_;  // ns
  double active_ = 0.0;
  std::deque<std::pair<Nanos, double>> pending_;  // effective time, force
  std::vector<control::ForceSegment> segs_;  // advance()'s result
  std::vector<control::ForceSegment> lead_;  // lead()'s result
};

// The device model a controller books and predicts with, and the schedule
// of the commands it issued. With the actuation delay known, one of each.
// With it unknown, a bank of them, one per candidate delay on a grid, all
// fed the same samples and commands: each candidate's model, wrong by the
// forces it times wrongly, predicts the displacements the worse the further
// its delay is from the device's (the prediction error method, run as a
// bank: multiple-model adaptive estimation). The candidates score with a
// HeldDeviceFilter each, which estimates the hand's damping and stiffness
// along with the delay, as a hand's damping otherwise passes for an earlier
// actuation; their force walks are adapted together.
//
// The controller uses the longest delay whose score the readings cannot
// tell from the best one's (a likelihood-ratio confidence set), not the
// best one itself: taken too short, the delay lets the PC spend energy the
// device has yet to receive, and on a simulated 0.2 kg device under a hand
// a 2 ms delay taken as 0-1 ms made it active, while 1.25-4 ms did not;
// the best score alone wandered to 0.3 ms within the first half second.
// In the simulated teleoperator, from a 55 um encoder the estimates ended
// up to 0.25 ms short and 0.5 ms long, a slave's at the 4 ms maximum in
// some motions, whose readings never told the candidates apart; from an
// exact encoder, on the delay.
class DeviceModel {
 public:
  // `delay` is the actuation delay, or with `max_delay` > 0 the guess the
  // estimate starts from; the candidates then lie `step` apart from 0 to
  // max_delay.
  // With `estimate_noise` and a resolution the filters' reading noise is
  // estimated from the readings (control::ReadingNoise).
  DeviceModel(double mass, Nanos dt, double resolution, double force_rate, bool adapt_force_rate,
              Nanos delay, Nanos max_delay = 0, Nanos step = 0, bool estimate_noise = false);

  // Books and observes the sample from `from` to `now` in which the encoder
  // saw the displacement dx (h = its length in s), and sets the forces in
  // flight as the predictor's lead. Returns the work the port forces did
  // on the two-port over it (positive = in), as the predictor in use
  // splits it. `cut_after` as in MasterController::step.
  double book_and_observe(Nanos from, Nanos now, double dx, double h, Nanos cut_after) {
    return book_and_observe(from, now, dx, h, cut_after, from, now);
  }
  // The same for a sample that ended at `to`, before the host's tick: the
  // last command was issued at `issued` (cut_after counts from it) and the
  // next one will be at `issue`, so the lead runs from `to` until that one
  // takes effect.
  double book_and_observe(Nanos from, Nanos to, double dx, double h, Nanos cut_after,
                          Nanos issued, Nanos issue);
  // No new reading since the sample that ended at `to`: the lead runs from
  // it until a command issued at `issue` takes effect, and includes the
  // commands issued since.
  void refresh_lead(Nanos to, Nanos issue);
  void command(Nanos now, double phi);
  // The delay of the commands from now on, measured by the device
  // (Device1D::actuation_delay). With the delay estimated it stays the
  // estimate's: a bank of candidates has a delay each.
  void set_actuation_delay(Nanos delay);

  const control::DevicePredictor& predictor() const { return bank_[best_].dev; }
  // The delay the predictor in use assumes: the estimate, if it is estimated.
  Nanos actuation_delay() const { return bank_[best_].delay; }
  // The variance of a reading's error the filters assume, m^2.
  double reading_noise() const { return bank_[best_].dev.reading_noise(); }

  // The quantization tax of commanding phi next (control::quantization_tax,
  // at the worst place in its count the device may lie in), to book as an
  // output, and the refunds of taxes booked `refund_lag` readings ago,
  // whose worst place the readings since have narrowed (see "Crossings" in
  // control/predictive_pc.hpp), to book as an input. Call once per command
  // with a resolution.
  struct Tax {
    double charged = 0.0;
    double refund = 0.0;
  };
  Tax book_tax(double phi);
  // How many readings the bounds on where the device lies in its count rest
  // on (0: none, the tax assumes nothing), how fast they take the forces
  // other than the port force to change (N/s), and how many readings after
  // a tax it is refunded.
  void set_count_window(std::size_t readings, double force_drift, std::size_t refund_lag);
  // For an encoder whose noise before its rounding has a standard deviation
  // within [lo, hi] counts: a command that takes effect a whole number of
  // samples after the last reading, and so on a reading it did not see, is
  // taxed as control::NoiseTax charges it, with `delta` and `force_scale`,
  // and other commands as above. With a resolution only.
  void set_noise_tax(double lo, double hi, double delta, double force_scale);
  // Whether the device reads its encoder again once a command has taken
  // effect, and reread() books by that reading: a command that takes effect
  // at once, the first on its reading, is then also taxed as NoiseTax
  // charges it, as the reading it is booked by is one it did not see.
  void set_reread(bool on) { reread_ = on; }
  // Whether the device books the tax of every change of its force itself,
  // in the work it reports (Device1D::taxes_changes): book_tax() then
  // books none, and keeps the noise tax's books only as the plan of what
  // the device will charge (change_tax()).
  void set_device_taxes(bool on) { device_taxes_ = on; }
  // The correction to the books when the command just booked took effect
  // with the encoder `dy` from the reading it was chosen on: the boundary
  // between the force before and the force after moves to the new reading,
  // -(phi - prev) dy (J). Zero unless set_reread() and the noise tax took
  // the command.
  double reread(double dy);
  // The noise tax when it applies to the next command, as
  // limit_force_predictive takes it (with set_device_taxes(), whenever it
  // is set); null otherwise.
  const control::NoiseTax* change_tax() const;
  // The noise tax's books, if it was set.
  const control::NoiseTax* noise_tax() const { return noise_tax_ ? &*noise_tax_ : nullptr; }

 private:
  struct Candidate {
    Nanos delay;
    ActuatorSchedule actuator;
    control::DevicePredictor dev;
    control::HeldDeviceFilter id;
    double score = 0.0;  // running mean of the squared innovation, m^2
    double miss = 0.0;   // the last sample's squared innovation, m^2
  };
  std::vector<Candidate> bank_;
  std::size_t best_ = 0;
  long n_scored_ = 0;  // samples scored so far
  double shorter_for_ = 0.0;  // s the readings have kept to a shorter delay
  double dt_;  // s
  bool adapt_;
  double log_walk_ = 0.0;  // log of the factor on the filters' force walk
  std::optional<control::ReadingNoise> noise_;
  // Taxes booked within the last refund_lag_ readings, with the line of
  // each in u, the bounds on u when it was booked and what it charged.
  struct TaxRecord {
    long reading;
    control::TaxLine line;
    double lo, hi, charged;
  };
  std::deque<TaxRecord> taxes_;
  // Taxes refunded within the window, with what each was finally charged:
  // their sum bounds the true taxes since the first of them, which a joint
  // bound over the window may undercut (see book_tax).
  struct Settled {
    long reading;
    control::TaxLine line;
    double net;
  };
  std::deque<Settled> settled_;
  long readings_ = 0;  // readings observed so far
  std::size_t refund_lag_;
  std::optional<control::NoiseTax> noise_tax_;
  bool reread_ = false;
  bool device_taxes_ = false;
  double pending_change_ = 0.0;  // phi - prev of the command to reread after
  long reread_reading_ = -1;     // the reading the last such command was chosen on
  // Whether the next command takes effect on a reading it did not see.
  bool on_unseen_reading() const;
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
  // raises it as far as its innovations show it to be overconfident.
  // With or without a resolution the PC predicts with the lower bound of
  // `pc_confidence` standard deviations, which include what the model has
  // lately missed, and a rise of the force repays any deficit of the books
  // over `pc_tax_samples` samples. So does the tax, instead of within the
  // coming sample, which a master pressed slower than a count per sample
  // cannot pay. With a resolution a rise also goes no further than the
  // force the expected motion could pay for over `pc_rise_samples` samples,
  // so that the force does not rise on every count transition and decay
  // before the next, paying the tax each time (0 lets it). Unset, it is 64
  // in the reflective architecture and 0 in the model-mediated one: there
  // the rendered wall holds the master where the mass the PC predicts with
  // would yield over 64 samples, and the hold kept the force far below the
  // wall's (an RMS shortfall of 7.5 N against 0.58 N without at 100 ms,
  // over tools/tuushin-sim's hand sweep; 32 samples left 1.5 N).
  // With `estimate_reading_noise` the filters take a reading's error to be
  // as large as the readings' second differences show it to be, and at
  // least the quantization's (control/reading_noise.hpp).
  double encoder_resolution = 0.0;
  double force_rate = 20.0;
  bool adapt_force_rate = true;
  bool estimate_reading_noise = true;
  double pc_confidence = 1.0;
  int pc_tax_samples = 4;
  std::optional<int> pc_rise_samples;
  // With a resolution: how many readings the bounds on where the device
  // lies in its count rest on (0: the tax assumes nothing about it), how
  // fast they take the operator's push to change (N/s), and how many
  // readings after a tax it is refunded as far as the readings since
  // narrow those bounds (0: never; control/predictive_pc.hpp, "Crossings").
  int count_window = 24;
  double force_drift = 50.0;
  int refund_lag = 12;
  // With a resolution: the standard deviation of the encoder's noise before
  // its rounding, in counts, as a range [lo, hi] the true one lies in. With
  // hi > 0 a force that takes effect a whole number of samples after it is
  // commanded is taxed as control::NoiseTax charges it, which bounds the
  // books with probability 1 - noise_tax_delta (control/noise_tax.hpp);
  // other forces, and every force with hi = 0, as above.
  double encoder_noise_lo = 0.0;
  double encoder_noise_hi = 0.0;
  double noise_tax_delta = 1e-3;
  double noise_tax_force_scale = 0.25;  // N
  // Whether the device reads its encoder again after each command takes
  // effect and the caller passes that reading to reread() before the next
  // step: a force that takes effect at once is then taxed by the noise
  // bound too (DeviceModel::set_reread).
  bool reread = false;
  // Whether the device books the tax of every change of its force in the
  // port_work() it reports (Device1D::taxes_changes), and the books take
  // none of their own (DeviceModel::set_device_taxes).
  bool device_taxes_changes = false;
  bool deadband = false;
  codec::DeadbandParams deadband_params{0.1, 0.002, from_millis(100)};
  // How long after a command the device applies it (ActuatorSchedule). The
  // books then credit each force with the displacement while it acted, and
  // the predictive PC pays for the forces in flight before the new one.
  Nanos actuation_delay = 0;
  // With a positive maximum the delay is estimated online instead, between
  // 0 and the maximum on a grid of the given step (dt / 16 for 0), with
  // actuation_delay as the first guess (DeviceModel).
  Nanos actuation_delay_max = 0;
  Nanos actuation_delay_step = 0;
  // kModelMediated: how the master moves to a new model.
  control::WallRendererParams wall_renderer;
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
    // The cap on the device's port_work() (same count) to command the
    // force with (Device1D::command_force_capped): what the books allow
    // it from the reading on, the budget. Set with a passivity controller
    // and the device's measured work.
    std::optional<double> work_cap;
  };

  // One sample at time `now` with the encoder at `position`. Books the work
  // the previously applied forces did since the last call; `cut_after` is
  // the device's Device1D::force_cut_after() (0 when the force stayed on).
  //
  // Samples need not come at exactly dt: the velocity and the device model
  // use the time that really passed, and a sample far from dt (the loop was
  // descheduled) or one during which the watchdog cut the force makes the
  // predictive PC apply no force for two samples (control::DevicePredictor).
  // A force that was cut did work only until the cut, over a displacement
  // no encoder read; the device model that chose the force predicts it, and
  // only an output is booked, since entering energy is credited only when
  // measured.
  Step step(Nanos now, double position, Nanos cut_after = 0) {
    return step(now, now, position, cut_after);
  }
  // The same for a position sampled at `sampled`, at most `now`, as a
  // device on a link reports it (Device1D::sampled_at). The books and the
  // device model run on the samples' times and the commands on `now`, so
  // that the sample's age, which varies with where the host's tick falls
  // between the device's, does not pass for motion: at 5 cm/s each 0.2 ms
  // of it is 10 um. A reading no newer than the last one is not observed
  // again; the force is chosen anew with the commands issued since in
  // flight. `port_work` is the device's Device1D::port_work(): when given,
  // the books take the work the device measured instead of the device
  // model's split of the displacement, which the model still observes.
  // `work_reserved` is the device's Device1D::work_reserved(): of the
  // input energy it reports, the controller holds back what the device
  // may still spend beyond the budget under the caps it was sent, which
  // the slave would otherwise spend as well (reported_energy()).
  Step step(Nanos now, Nanos sampled, double position, Nanos cut_after,
            std::optional<double> port_work = std::nullopt,
            std::optional<double> work_reserved = std::nullopt);
  // With MasterConfig::reread: the encoder read again once the last step's
  // command took effect, at which the books then take the force to change
  // (DeviceModel::reread). Ignored when the device measures its work.
  void reread(double position);

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
  // The input energy reported to the slave with the last step: the port's
  // reported_input_energy(), less what the device's caps may still spend
  // of it. It only grows.
  double reported_energy() const { return reported_; }
  // The actuation delay the controller books and predicts with: the
  // configured one, the device's measurement, or the estimate.
  Nanos actuation_delay() const { return model_.actuation_delay(); }
  // The device's measurement, for the commands from the next step on
  // (DeviceModel::set_actuation_delay).
  void set_actuation_delay(Nanos delay) { model_.set_actuation_delay(delay); }
  double reading_noise() const { return model_.reading_noise(); }  // m^2
  // The device model the controller predicts and books with.
  const control::DevicePredictor& predictor() const { return model_.predictor(); }
  std::uint64_t samples_sent() const { return sent_; }
  // kModelMediated: the model the master renders, and the position sent
  // with the last step, the master's less the rendering's offset.
  const control::WallRenderer& renderer() const { return renderer_; }
  double command_position() const { return x_command_; }

 private:
  // What the PC may still let out: the slave's grants or channel input,
  // or with kModelMediated the device port's own input.
  double balance() const;

  MasterConfig cfg_;
  double dt_;
  control::EnergyPort port_;
  DeviceModel model_;
  control::WallRenderer renderer_;
  double render_at_ = 0.0;  // the position the renderer took last
  std::optional<control::WallModel> received_model_;  // not yet rendered
  double x_command_ = 0.0;
  codec::DeadbandEncoder<1> enc_;
  bool first_ = true;
  Nanos last_now_ = 0;      // when the last sample ended
  Nanos last_issued_ = 0;   // when the last command was issued
  double x_sampled_ = 0.0;
  std::optional<double> port_work_;  // the device's measured work at the last sample
  double v_meas_ = 0.0;
  double v_sent_ = 0.0;
  double f_display_ = 0.0;
  bool has_value_ = false;
  std::uint32_t last_seq_ = 0;
  double f_reflected_ = 0.0;
  double reported_ = 0.0;
  std::uint64_t sent_ = 0;
};

struct SlaveConfig {
  Nanos dt = kNanosPerMilli;
  Passivity passivity = Passivity::kReflective;
  double kp = 2000.0;  // N/m, position coupling to the command
  double kd = 60.0;    // N s/m, velocity coupling to the command
  // Report the measured contact force instead of the coupling force. See
  // sim::SlaveParams for why the coupling force is the default; kReflective
  // cannot work with it, and the constructor rejects the combination.
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
  // passed to step(). The rise horizon is off by default: on the slave it
  // holds back the coupling force against the environment, and in the
  // simulator the robot then pressed into the wall about a third less hard.
  double encoder_resolution = 0.0;  // m
  double force_rate = 20.0;         // N/s
  bool adapt_force_rate = true;
  bool estimate_reading_noise = true;
  double pc_confidence = 1.0;
  int pc_tax_samples = 4;
  int pc_rise_samples = 0;
  // As at the master, with the environment's push.
  int count_window = 24;
  double force_drift = 50.0;
  int refund_lag = 12;
  double encoder_noise_lo = 0.0;
  double encoder_noise_hi = 0.0;
  double noise_tax_delta = 1e-3;
  double noise_tax_force_scale = 0.25;  // N
  bool reread = false;
  bool device_taxes_changes = false;
  bool deadband = false;
  codec::DeadbandParams deadband_params{0.1, 0.05, from_millis(100)};
  // Between samples the slave extrapolates the master position with the last
  // velocity, which bridges deadband gaps and single losses. When no sample
  // has arrived for longer than this, it stops extrapolating and holds the
  // last estimate instead, so that a dead link cannot drag the robot along
  // the last velocity. 0 extrapolates indefinitely; it must exceed the
  // deadband's max_interval to leave the deadband alone.
  Nanos hold_after = 0;
  // As in MasterConfig.
  Nanos actuation_delay = 0;
  Nanos actuation_delay_max = 0;
  Nanos actuation_delay_step = 0;
  // kModelMediated: the estimate of the wall from the contact force, and
  // whether the slave goes into the wall no further than the force the
  // master displays explains (surface + force / stiffness, by its own
  // model). Without that, a master that has not learnt of the wall yet,
  // as at a first contact for a round trip, drives the slave into it on
  // the coupling alone.
  control::WallEstimatorParams wall_estimator;
  bool consistent_commands = true;
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
    // As in MasterController::Step, kReflective only: the slave's grant,
    // and a share of the store beyond it (kCapSpareShare in the source).
    std::optional<double> work_cap;
  };

  // One sample at time `now` with the encoder at `position`, the device's
  // velocity estimate `velocity` (the damping term of the coupling acts on
  // it) and the force sensor (if any) at `contact_force`, which is only used
  // when the slave reports it. Irregular samples and `cut_after` are handled
  // as in MasterController::step.
  Step step(Nanos now, double position, double velocity, double contact_force,
            Nanos cut_after = 0) {
    return step(now, now, position, velocity, contact_force, cut_after);
  }
  // The same for a position sampled at `sampled`, and with the work the
  // device measured, as in MasterController. kReflective grants the master
  // none of what the device may still take out under its caps
  // (`work_reserved`, control::ReflectiveStore::allocate).
  Step step(Nanos now, Nanos sampled, double position, double velocity, double contact_force,
            Nanos cut_after, std::optional<double> port_work = std::nullopt,
            std::optional<double> work_reserved = std::nullopt);
  // As MasterController::reread, with SlaveConfig::reread.
  void reread(double position);

  // Diagnostics of the last step.
  double command() const { return x_cmd_; }            // coupling target (m)
  double master_estimate() const { return x_master_est_; }
  double requested_velocity() const { return v_requested_; }
  double command_velocity() const { return v_c_; }
  double reported_force() const { return f_slave_; }
  bool has_master_sample() const { return has_value_; }
  // Net energy that entered through the slave device port, booked from the
  // work of the applied force as the device measured it, or else as the
  // device model splits it (predicted, for a sample in which the watchdog
  // cut it), less the quantization tax.
  double terminal_net() const { return terminal_net_; }
  // kReflective: the store. kNone and kChannel: the channel end at the slave.
  const control::ReflectiveStore& store() const { return store_; }
  const control::EnergyPort& channel_port() const { return port_; }
  Nanos actuation_delay() const { return model_.actuation_delay(); }
  void set_actuation_delay(Nanos delay) { model_.set_actuation_delay(delay); }
  double reading_noise() const { return model_.reading_noise(); }  // m^2
  // The device model the controller predicts and books with.
  const control::DevicePredictor& predictor() const { return model_.predictor(); }
  std::uint64_t samples_sent() const { return sent_; }
  // kModelMediated: the slave's estimate of the wall.
  const control::WallModel& wall() const { return estimator_.model(); }

 private:
  SlaveConfig cfg_;
  double dt_;
  control::EnergyPort port_;
  control::ReflectiveStore store_;
  DeviceModel model_;
  control::WallEstimator estimator_;
  codec::DeadbandEncoder<1> enc_;
  double x_sampled_;
  std::optional<double> port_work_;  // the device's measured work at the last sample
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
  Nanos last_now_ = 0;     // when the last sample ended
  Nanos last_issued_ = 0;  // when the last command was issued
  std::uint64_t sent_ = 0;
};

}  // namespace tuushin::teleop
