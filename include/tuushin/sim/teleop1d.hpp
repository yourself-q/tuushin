#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include "tuushin/codec/deadband.hpp"
#include "tuushin/net/channel.hpp"
#include "tuushin/teleop/controller.hpp"
#include "tuushin/teleop/device.hpp"
#include "tuushin/time.hpp"

namespace tuushin::sim {

// One-degree-of-freedom bilateral teleoperation in the velocity-force
// ("force reflecting") architecture used by the TDPA+PD literature:
//
//   operator hand -> master device --velocity--> network --> slave robot -> wall
//                    master device <---force---- network <-- slave robot
//
// The slave tracks the integrated velocity command with a PD coupling and
// reports the coupling force. The master displays that force. Each direction
// can carry a perceptual deadband, and TDPA passivity controllers can sit at
// both ends of the channel. The two ends are teleop::MasterController and
// teleop::SlaveController, the same code the networked nodes run.
//
// Units are SI: metres, seconds, newtons, joules.

// The operator model and the wall live with the simulated devices.
using OperatorParams = teleop::OperatorParams;
using WallParams = teleop::WallParams;

struct MasterParams {
  double mass = 0.2;       // kg, apparent mass of a desktop haptic device
  double damping = 0.5;    // N s/m
  // Actuator limit in N, 0 for none. Desktop devices peak between about
  // 3 N (3D Systems Touch) and 20 N (Force Dimension sigma.7).
  double max_force = 20.0;
  // What the controller's model leaves out (teleop/device.hpp): friction
  // besides the viscous damping and a compliant structure; and a delay
  // before a commanded force takes effect, which the controllers are told
  // (Teleop1DConfig::master_model_actuation_delay).
  teleop::FrictionParams friction;
  teleop::ComplianceParams compliance;
  Nanos actuation_delay = 0;
};

struct SlaveParams {
  double mass = 1.0;       // kg
  double damping = 2.0;    // N s/m
  double kp = 2000.0;      // N/m, position coupling to the command
  double kd = 60.0;        // N s/m, velocity coupling to the command
  // Report the measured contact force instead of the coupling force. The
  // coupling force is what a robot without a force sensor has (motor current),
  // and it is the conjugate of the commanded velocity, so TDPA's energy books
  // describe the real slave port. With the contact force they describe a port
  // the slave does not have: under kChannel without delay, where the coupling
  // force keeps min_true_margin at 0, the default scenario then generates
  // 4.7 J and runs into the force limit. kReflective cannot run with it at
  // all: in free space the contact force is zero, so nothing the operator
  // does puts energy in, and the slave, which needs energy to follow, never
  // moves. run_teleop1d rejects that combination.
  bool report_contact_force = false;
  // Position-drift compensation gain in 1/s, 0 to disable. The slave adds
  // v_ad = drift_gain * (x_master_delayed - x_slave) to the received velocity
  // *before* its passivity controller, so the PC still enforces passivity and
  // the correction only happens in passivity gaps. This is the compensator of
  // A. Coelho et al., "Smoother position-drift compensation for time domain
  // passivity approach based teleoperation", IROS 2018 (arXiv 2002.02296),
  // eq. (16), with K = drift_gain * dt. Their experiments use K = 0.025 at
  // 1 kHz, i.e. 25 1/s, and recommend 0 < K < 1.
  double drift_gain = 0.0;
  // What the drift is measured against. kSlavePosition is Coelho et al.'s
  // choice (x_master_delayed - x_slave); it also cancels the steady coupling
  // error, which acts like integral action and pushes the slave further into
  // a wall than the plain coupling would. kCommand (x_master_delayed - x_cmd)
  // only returns what the passivity controller took away, as in the drift
  // signal of Artigas et al. (IROS 2010) and Chawda et al. (Haptics Symp.
  // 2014), but released gradually through the gain instead of at once.
  using DriftReference = teleop::SlaveConfig::DriftReference;
  DriftReference drift_reference = DriftReference::kCommand;
  // As for the master.
  teleop::FrictionParams friction;
  teleop::ComplianceParams compliance;
  Nanos actuation_delay = 0;
};

struct Teleop1DConfig {
  Nanos dt = kNanosPerMilli;   // control and packet period (1 kHz)
  int substeps = 10;           // physics steps per control step
  Nanos duration = from_seconds(10.0);
  // Each end samples its device at k * dt plus a time drawn uniformly from
  // [-sample_jitter, sample_jitter], independently, as control loops on a
  // general-purpose OS do; less than dt / 2.
  Nanos sample_jitter = 0;
  std::uint64_t jitter_seed = 1;

  OperatorParams op;
  MasterParams master;
  SlaveParams slave;
  WallParams wall;

  // How passivity is enforced.
  //  kNone       no passivity controllers (the slave integrates the received
  //              velocity, as in kChannel).
  //  kChannel    conventional TDPA on the delayed channel in each direction:
  //              impedance-type PC at the master, admittance-type PC on the
  //              velocity command at the slave, which integrates the corrected
  //              velocity (Ryu & Preusche, ICRA 2007; Panzirsch et al.'s
  //              "Approach 1"). The PC changes the command, so the slave
  //              drifts away from the master.
  //  kReflective one budget at the slave covering the channel and the coupling
  //              controller, which lets energy stored in the coupling flow
  //              back to the master (control/reflection.hpp; Panzirsch et
  //              al.'s "Approach 3"). The slave couples to the delayed master
  //              position itself, and impedance-type PCs at both device ports
  //              only scale forces, so there is no position drift and
  //              drift_gain is not used.
  //  kModelMediated
  //              the slave sends a model of the wall, which the master
  //              renders through a PC on its own books (control/wall_model.hpp,
  //              teleop::Passivity). Only the master's device port is
  //              passive: min_master_true_margin, not min_true_margin, is the
  //              margin to compare.
  using Passivity = teleop::Passivity;
  Passivity passivity = Passivity::kChannel;
  // kModelMediated: the estimate at the slave, the rendering at the master,
  // and whether the slave's commands are kept consistent with the force the
  // master displays (teleop::SlaveConfig).
  control::WallEstimatorParams wall_estimator;
  control::WallRendererParams wall_renderer;
  bool consistent_commands = true;
  // kReflective: energy in the slave-side store before anything entered (J).
  // The teleoperator is then passive with respect to this initial storage;
  // the device-port margin can go down to -initial_energy.
  double initial_energy = 0.0;
  // Impedance-type PCs at the device ports predict the device's response to
  // their own force (control/predictive_pc.hpp) instead of assuming the
  // velocity stays constant over the sample. The model masses are what the
  // controllers believe; 0 means the true mass.
  bool predictive_pc = true;
  double master_model_mass = 0.0;  // kg
  double slave_model_mass = 0.0;   // kg
  // Samples over which a predictive PC's force must stay affordable if held.
  // 1 is the greedy per-sample limit, which chatters between zero and full
  // force once the budget is exhausted; 2 stops the device instead of
  // bouncing it (see limit_force_predictive). Measured on the default
  // scenario, 2 at the master cuts the residual energy deficit by 5 to 10
  // times and removes the chatter, at the price of a softer-looking wall in
  // the least-squares stiffness metric, which the chatter had inflated.
  int pc_hold_samples = 2;
  // Encoder resolutions in m, 0 for exact encoders. The simulated devices
  // round their readings, and their velocity becomes the backward
  // difference of the readings. If the controllers know the resolutions
  // their predictors filter the readings and they book the quantization tax
  // (control/predictive_pc.hpp); if not, they treat the readings as exact.
  double master_encoder_resolution = 0.0;
  double slave_encoder_resolution = 0.0;
  // Gaussian noise added to the positions before they are rounded, in
  // counts (teleop::SimulatedDevice::set_encoder_noise).
  double master_encoder_noise = 0.0;
  double slave_encoder_noise = 0.0;
  // Gaussian noise on the slave's force sensor, N (standard deviation).
  // The metrics use the true contact force.
  double force_sensor_noise = 0.0;
  bool encoder_resolution_known = true;
  // The predictors and their use by the PCs (teleop::MasterConfig): with
  // known resolutions, the Kalman filters' least force rate in N/s,
  // whether they may raise it and whether they estimate the reading noise;
  // with any encoders, the confidence factor
  // and the tax horizon of limit_force_predictive; with known resolutions,
  // its rise horizons.
  double force_rate = 20.0;
  bool adapt_force_rate = true;
  bool estimate_reading_noise = true;
  double pc_confidence = 1.0;
  int pc_tax_samples = 4;
  int master_pc_rise_samples = 64;
  int slave_pc_rise_samples = 0;
  // The actuation delays the controllers book and predict with
  // (teleop::ActuatorSchedule); negative means the devices' true ones
  // (MasterParams::actuation_delay), 0 takes every force to act at once.
  Nanos master_model_actuation_delay = -1;
  Nanos slave_model_actuation_delay = -1;
  // With a positive maximum the controller estimates its device's delay
  // online instead (teleop::DeviceModel), up to the maximum, starting from
  // the model delay above as its guess, or if that is negative from the
  // maximum: the safe side, as a delay taken too long only holds the force
  // back while one taken too short lets the PC spend early.
  Nanos master_actuation_delay_max = 0;
  Nanos slave_actuation_delay_max = 0;
  bool forward_deadband = false;
  codec::DeadbandParams forward_deadband_params{0.1, 0.002, from_millis(100)};
  bool backward_deadband = false;
  codec::DeadbandParams backward_deadband_params{0.1, 0.05, from_millis(100)};

  // Size reported to the channel model for each haptic packet.
  std::size_t packet_bytes = 64;

  // If non-empty, a per-sample CSV log is written here.
  std::string log_path;
};

struct Teleop1DResult {
  bool diverged = false;          // state left the plausible range
  double diverged_at = 0.0;       // s

  double max_master_pos = 0.0;    // m, max |x_m|
  double max_slave_pos = 0.0;     // m
  double max_display_force = 0.0; // N, max |f_display|
  double pos_rmse = 0.0;          // m, x_m - x_s
  double final_drift = 0.0;       // m, mean (x_m - x_s) over the last second
  double force_rmse = 0.0;        // N, displayed force vs. contact force
  // Displayed stiffness during contact: least-squares slope of displayed force
  // over master penetration. Compare with the wall stiffness to see how much
  // softer the remote wall feels.
  double displayed_stiffness = 0.0;  // N/m
  // What the master's passivity controller (and actuator limit) took away:
  // RMS of the displayed force less the force the newest slave sample asked
  // for. Unlike force_rmse it leaves out the delay and the coupling, which
  // no passivity controller can fix.
  double pc_force_rmse = 0.0;  // N
  // Total variation of the displayed force per second, sum |f_k - f_{k-1}|
  // over the run's length: how much it chatters.
  double display_force_variation = 0.0;  // N/s
  // The displayed force over the run as a share of the contact force over
  // it (sums of their positive parts): how much of the wall the operator
  // felt, whichever end leads.
  double conveyed_force = 0.0;
  double max_contact_force = 0.0;     // N
  double max_slave_penetration = 0.0; // m, into the wall

  double forward_packet_rate = 0.0;   // packets/s actually sent
  double backward_packet_rate = 0.0;
  double forward_loss = 0.0;          // fraction lost by the channel
  double backward_loss = 0.0;

  double master_pc_active = 0.0;  // fraction of samples with the PC engaged
  double slave_pc_active = 0.0;
  // Minimum over time of the net energy that entered the teleoperator through
  // the master and slave device ports, booked from the realized work of the
  // held forces. Everything between the operator and the environment
  // (channel, controllers, PCs) is inside, so this is the passivity margin
  // that matters physically and it is comparable across schemes. Negative
  // means the teleoperator handed out more energy than it received.
  // A sampled-data PC that predicts the device motion over the coming sample
  // can overshoot by that prediction error once before it blocks output, so
  // expect small negative values of the order of f * a * dt^2.
  double min_passivity_margin = 0.0;  // J
  // The same from the work the actuators really did
  // (teleop::SimulatedDevice::actuator_work), which the books only see
  // through the encoders. With exact encoders the two agree to rounding.
  double min_true_margin = 0.0;  // J
  // The same at the master's device port alone: what the operator's hand
  // took out of the teleoperator beyond what it put in, at worst.
  double min_master_true_margin = 0.0;  // J
  // The largest amount by which the booked margin exceeded the true one at
  // any sample: positive means the books credited energy that never
  // entered, which the quantization tax is there to prevent.
  double max_book_excess = 0.0;  // J
  // Minimum over time, in each direction, of the input one end reported to
  // the other less the other's output beyond what it paid from its own
  // input (control::EnergyPort::covered_energy), from true (undelayed)
  // books: the per-direction condition kChannel enforces. The two sum to
  // the channel's net energy. Not computed for kReflective, whose coupling
  // sits inside the two-port.
  double min_channel_margin = 0.0;  // J
  // The actuation delays the controllers ended with: the model delays, or
  // their estimates.
  Nanos master_actuation_delay = 0;
  Nanos slave_actuation_delay = 0;
  // The standard deviation of a reading's error the controllers ended up
  // assuming (control::ReadingNoise), m; 0 with an exact encoder.
  double master_reading_noise = 0.0;
  double slave_reading_noise = 0.0;
};

Teleop1DResult run_teleop1d(const Teleop1DConfig& cfg, net::ChannelModel& forward,
                            net::ChannelModel& backward);

}  // namespace tuushin::sim
