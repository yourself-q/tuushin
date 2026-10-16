#pragma once

#include <cstdint>
#include <deque>
#include <optional>
#include <utility>

#include "tuushin/rng.hpp"
#include "tuushin/time.hpp"

namespace tuushin::teleop {

// A force-controlled one-axis device as a controller sees it: an encoder, a
// velocity estimate, an actuator and possibly a force sensor. A haptic device
// driver or a robot interface implements it; the simulated devices below
// stand in for both until hardware is attached.
class Device1D {
 public:
  virtual ~Device1D() = default;
  // Samples the encoder at `now` (m). A simulated device advances its
  // physics to `now` with the force commanded last.
  virtual double read_position(Nanos now) = 0;
  // When the position the last read_position(now) returned was sampled,
  // on the same clock: `now` for a device sampled when it is read; earlier
  // for one whose samples come over a link (device::RemoteDevice), whose
  // age varies with the link and with where the host's tick falls between
  // the device's. The controllers book and predict with the sample's own
  // time, so that its age does not pass for motion.
  virtual Nanos sampled_at(Nanos now) const { return now; }
  // How long after command_force a force takes effect, on the clock
  // sampled_at() places the positions on, if the device measures it: a
  // servo loop that says when it applied each command, as the firmware
  // behind device::RemoteDevice does. The controllers then book and
  // predict with it (ActuatorSchedule) rather than estimate it from the
  // readings, which tell delays apart only as far as the motion lets
  // them. It may change from command to command, as the host's ticks
  // drift against the device's.
  virtual std::optional<Nanos> actuation_delay() const { return std::nullopt; }
  // Velocity estimate at the last read (m/s).
  virtual double velocity() const = 0;
  // Force along +x (N), held until the next command.
  virtual void command_force(double f) = 0;
  // Interaction force with the environment at the last read (N), 0 without a
  // force sensor.
  virtual double contact_force() const { return 0.0; }
  // If the device's watchdog cut the force commanded last because no new
  // command came in time, how long after that command it did (ns); 0 while
  // the force is applied. The servo loops of haptic devices do this when the
  // host stops sending (Force Dimension's dhdSetWatchdog, for one), and
  // report it in their status. The controllers need to know: a force that
  // was cut did no work for the rest of the sample, and booking it as if it
  // had credits energy that never entered.
  virtual Nanos force_cut_after() const { return 0; }
  // The work the commanded force has done on the device (J: the force
  // along +x times the displacement while it acted), summed up to the
  // reading sampled_at() refers to, if the device measures it. A servo loop
  // that sums it over its own ticks knows it to its encoder's resolution.
  // The host only knows a sample's displacement, and when the force changed
  // within the sample (a command takes effect between two readings) it can
  // only split the displacement among the forces by a model, which a device
  // against a stiff object, or chattering forces, put hundreds of
  // micrometres off: work booked that no force did. The controllers book
  // this instead when it is given.
  virtual std::optional<double> port_work() const { return std::nullopt; }
  // Whether the device keeps the commanded force's port_work() within a
  // cap the host sends with it (command_force_capped), on its own ticks.
  // The books learn of the work a sample and a link's latency after it
  // was done, and a device against a stiff object can do a contact's
  // worth of it in that time; the device can stop it within a tick.
  virtual bool limits_work() const { return false; }
  // Whether port_work() carries the tax of every change of the force, the
  // bound on what the encoder's resolution and noise hide in the work,
  // booked by the device as it applies the changes (device::kInfoTaxesChanges).
  // The controllers then book none of their own for the changes they
  // command, which would count it twice.
  virtual bool taxes_changes() const { return false; }
  // command_force, with port_work() kept at most `work_cap` (J, on
  // port_work()'s count) while this force is in effect, if the device
  // limits_work(); a force that takes energy out is never held back.
  virtual void command_force_capped(double f, double /*work_cap*/) { command_force(f); }
  // How much further than the port_work() the last read returned the
  // forces commanded so far may still take it (J, at least 0): the
  // largest cap among the commands that may be in effect now or later.
  // A controller keeps that much of its energy for the device instead of
  // passing it on, which the device would otherwise spend a second time.
  // Infinite while a force without a cap may still be in effect; empty
  // for a device that does not limit its work.
  virtual std::optional<double> work_reserved() const { return std::nullopt; }
  // Why the device cannot go on (a latched fault, a link that went
  // quiet), or nullptr while it can. A node ends its session on it.
  virtual const char* fault() const { return nullptr; }
};

struct OperatorParams {
  // The hand pulls the master toward a reference position through its own
  // stiffness and damping, which is how human arm impedance is usually
  // modelled in teleoperation studies.
  double hand_k = 300.0;   // N/m
  double hand_b = 5.0;     // N s/m
  // Reference: offset + amplitude * (1 - cos(2 pi f t)) / 2, i.e. a smooth
  // push from `offset` to `offset + amplitude` and back, repeated.
  double ref_offset = 0.0;
  double ref_amplitude = 0.04;  // m
  double ref_freq = 0.5;        // Hz
};

struct WallParams {
  double position = 0.02;  // m
  double k = 5000.0;       // N/m, or N/m^exponent
  double b = 10.0;         // N s/m, only while penetrating
  // A Hunt-Crossley contact (K. H. Hunt and F. R. E. Crossley, J. Appl.
  // Mech. 42(2), 1975), as tissue, rubber or a Hertzian contact pushes:
  // k d^exponent + (b + hc_damping d^exponent) d' at a penetration d, which
  // is no line through any samples. 1 and 0 make the spring-damper wall.
  double exponent = 1.0;
  double hc_damping = 0.0;  // N s/m^(exponent + 1)
  // The wall moves as position + motion_amplitude sin(2 pi motion_freq t),
  // as a beating organ or a part on a conveyor does.
  double motion_amplitude = 0.0;  // m
  double motion_freq = 0.0;       // Hz
  double position_at(double t) const;
  double velocity_at(double t) const;
  // The force with which the wall pushes a point at x moving at v back at
  // time t (N, at least 0).
  double force(double t, double x, double v) const;
};

// Friction in a device's transmission besides the viscous damping, as in
// the model of Armstrong-Helouvry et al. (Automatica 30(7), 1994): Coulomb
// friction that drops from the stiction force to the Coulomb force over
// the Stribeck velocity, smoothed through zero velocity with tanh(v / v0) so
// that the integrator need not find the stops. A tanh with a small v0 lets
// the device creep under a force F below the stiction force Fs instead of
// sticking, at v0 atanh(F / Fs).
struct FrictionParams {
  double coulomb = 0.0;              // N
  double stiction = 0.0;             // N, ignored below `coulomb`
  double stribeck_velocity = 0.01;   // m/s
  double smoothing_velocity = 1e-3;  // m/s, v0
  // The friction force on a device moving at v, against the motion.
  double force(double v) const;
};

// A device whose structure gives: the encoder and the actuator sit on the
// motor side, and the hand or the wall pushes on a distal mass joined to it
// by a spring and a damper (a cable, a belt, a harmonic drive or a link).
// Below the resonance the two move as one mass; above it the controller
// drives the motor side alone, which is lighter than any model of the
// whole device says.
struct ComplianceParams {
  double distal_mass = 0.0;  // kg, part of the device's mass; 0 for a rigid device
  double stiffness = 0.0;    // N/m
  double damping = 0.0;      // N s/m
};

// Physics shared by the simulated devices: a mass with viscous friction,
// integrated with classical Runge-Kutta under a zero-order-held actuator
// force. RK4 is exact for a constant force on a mass, as a real device under
// zero-order hold behaves, whereas semi-implicit Euler with n substeps
// overstates the displacement a held force causes within the sample by a
// factor (n + 1) / n, an artefact the device-model PC would otherwise be
// tuned against.
//
// Reads may come at any time: the device integrates the elapsed time in
// substeps of at most nominal_dt / substeps, so a late control step sees the
// device where it really is by then, as with hardware.
class SimulatedDevice : public Device1D {
 public:
  double read_position(Nanos now) override;
  // The velocity at the last read, or with an encoder resolution the
  // backward difference of the readings, as device drivers report it.
  double velocity() const override { return q_ > 0.0 ? v_read_ : v_; }
  void command_force(double f) override;
  Nanos force_cut_after() const override { return cut_after_; }

  // Rounds the readings to multiples of `q` (m), as an encoder with that
  // resolution does; 0 (the default) reads the exact position.
  void set_encoder_resolution(double q);
  double encoder_resolution() const { return q_; }
  // Adds Gaussian noise of `counts` standard deviations of a count to the
  // position before it is rounded, drawn anew for every reading, as a
  // magnetic encoder dithers (about half a count on an MT6701 or AS5047P).
  // Only with a resolution; 0 (the default) adds none.
  void set_encoder_noise(double counts, std::uint64_t seed = 1) {
    noise_counts_ = counts > 0.0 ? counts : 0.0;
    noise_ = Rng(seed);
  }
  // Reads the encoder again where the device is now, with the noise drawn
  // anew, as a second conversion after a command took effect does
  // (MasterController::reread); what read_position() returns is unchanged.
  double reread_position();

  // Friction besides the viscous damping, acting on the motor side.
  void set_friction(const FrictionParams& friction) { friction_ = friction; }
  // Splits the device's mass into a motor side, which the encoder reads and
  // the actuator drives, and a distal side, on which the external force
  // acts; the two start together at rest. A distal mass outside (0, mass)
  // makes the device rigid again.
  void set_compliance(const ComplianceParams& compliance);
  // A commanded force takes effect `delay` later, as with a servo loop that
  // picks up commands on its own ticks or a bus that carries them. The
  // watchdog still counts from the command.
  void set_actuation_delay(Nanos delay) { delay_ = delay > 0 ? delay : 0; }

  // The physical state, whatever the encoder reads: of the motor side.
  double position() const { return x_; }
  double true_velocity() const { return v_; }
  // The distal side, where the hand or the wall acts; the motor side for a
  // rigid device.
  double distal_position() const { return compliant_ ? x2_ : x_; }
  double distal_velocity() const { return compliant_ ? v2_ : v_; }
  // The force the actuator applies now: the last command that has taken
  // effect, or 0 once the watchdog has cut it.
  double force() const { return f_; }
  Nanos time() const { return t_; }
  // Work done on the device by its actuator so far (J): the integral of the
  // applied force over the displacement, exact under zero-order hold. The
  // physical counterpart of a controller's energy books.
  double actuator_work() const { return work_; }
  // Cuts the actuator force to 0 once no command has come for `timeout`
  // after the read it followed, as the watchdog of a haptic device's servo
  // loop does when the host stops sending. 0 disables it (the default).
  void set_watchdog(Nanos timeout) { watchdog_ = timeout; }
  Nanos watchdog() const { return watchdog_; }
  // False once the state has left the plausible range (|x| < 1 m,
  // |v| < 10 m/s, finite), i.e. the loop went unstable.
  bool plausible() const;

 protected:
  SimulatedDevice(double mass, double damping, Nanos nominal_dt, int substeps, Nanos start,
                  double position);
  // Forces other than the actuator and friction at time `t` (s since start)
  // for the state (x, v) of the distal side.
  virtual double external_force(double t, double x, double v) const = 0;
  Nanos start() const { return start_; }

 private:
  void advance(Nanos to);
  void integrate(Nanos to);

  double mass_, damping_;
  FrictionParams friction_;
  bool compliant_ = false;
  double m1_ = 0.0, m2_ = 0.0;  // motor and distal masses when compliant
  ComplianceParams compliance_;
  double x2_ = 0.0, v2_ = 0.0;
  Nanos delay_ = 0;
  std::deque<std::pair<Nanos, double>> pending_;  // commands not yet in effect
  Nanos nominal_dt_;
  int substeps_;
  Nanos start_;
  Nanos t_;
  double x_, v_ = 0.0;
  double f_ = 0.0;
  double commanded_ = 0.0;
  Nanos command_time_;
  Nanos watchdog_ = 0;
  Nanos cut_after_ = 0;
  double work_ = 0.0;
  double q_ = 0.0;
  double noise_counts_ = 0.0;
  Rng noise_{1};
  bool have_read_ = false;
  double y_ = 0.0;       // last reading
  Nanos y_time_ = 0;     // and when it was taken
  double v_read_ = 0.0;  // backward difference of the readings
};

// A desktop haptic device held by a simulated operator.
class SimulatedMaster final : public SimulatedDevice {
 public:
  // The reference trajectory starts at `start`; the device starts at rest at
  // the reference's offset.
  SimulatedMaster(double mass, double damping, const OperatorParams& op, Nanos nominal_dt,
                  int substeps, Nanos start = 0);
  // The operator's hand force at the last read (N).
  double hand_force() const;
  const OperatorParams& op() const { return op_; }

 private:
  double external_force(double t, double x, double v) const override;
  OperatorParams op_;
};

// A robot that touches a spring-damper wall; the wall only pushes.
class SimulatedSlave final : public SimulatedDevice {
 public:
  SimulatedSlave(double mass, double damping, const WallParams& wall, Nanos nominal_dt,
                 int substeps, Nanos start = 0, double position = 0.0);
  double contact_force() const override;
  // Where the wall's surface is now.
  double wall_position() const;

 private:
  double external_force(double t, double x, double v) const override;
  WallParams wall_;
};

}  // namespace tuushin::teleop
