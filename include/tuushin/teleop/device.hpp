#pragma once

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
  double k = 5000.0;       // N/m
  double b = 10.0;         // N s/m, only while penetrating
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
  double velocity() const override { return v_; }
  void command_force(double f) override {
    f_ = f;
    command_time_ = t_;
    cut_after_ = 0;
  }
  Nanos force_cut_after() const override { return cut_after_; }

  double position() const { return x_; }
  // The force the actuator applies now: the last command, or 0 once the
  // watchdog has cut it.
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
  // for the state (x, v).
  virtual double external_force(double t, double x, double v) const = 0;
  Nanos start() const { return start_; }

 private:
  void advance(Nanos to);

  double mass_, damping_;
  Nanos nominal_dt_;
  int substeps_;
  Nanos start_;
  Nanos t_;
  double x_, v_ = 0.0;
  double f_ = 0.0;
  Nanos command_time_;
  Nanos watchdog_ = 0;
  Nanos cut_after_ = 0;
  double work_ = 0.0;
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

 private:
  double external_force(double t, double x, double v) const override;
  WallParams wall_;
};

}  // namespace tuushin::teleop
