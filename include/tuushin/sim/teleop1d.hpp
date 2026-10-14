#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include "tuushin/codec/deadband.hpp"
#include "tuushin/net/channel.hpp"
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
// both ends of the channel.
//
// Units are SI: metres, seconds, newtons, joules.

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

struct MasterParams {
  double mass = 0.2;       // kg, apparent mass of a desktop haptic device
  double damping = 0.5;    // N s/m
  // Actuator limit in N, 0 for none. Desktop devices peak between about
  // 3 N (3D Systems Touch) and 20 N (Force Dimension sigma.7).
  double max_force = 20.0;
};

struct SlaveParams {
  double mass = 1.0;       // kg
  double damping = 2.0;    // N s/m
  double kp = 2000.0;      // N/m, position coupling to the command
  double kd = 60.0;        // N s/m, velocity coupling to the command
  // Report the measured contact force instead of the coupling force. The
  // coupling force is what a robot without a force sensor has (motor current),
  // and it is the conjugate of the commanded velocity, so TDPA's energy books
  // describe the real slave port.
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
  enum class DriftReference { kSlavePosition, kCommand };
  DriftReference drift_reference = DriftReference::kCommand;
};

struct WallParams {
  double position = 0.02;  // m
  double k = 5000.0;       // N/m
  double b = 10.0;         // N s/m, only while penetrating
};

struct Teleop1DConfig {
  Nanos dt = kNanosPerMilli;   // control and packet period (1 kHz)
  int substeps = 10;           // physics steps per control step
  Nanos duration = from_seconds(10.0);

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
  enum class Passivity { kNone, kChannel, kReflective };
  Passivity passivity = Passivity::kChannel;
  // Impedance-type PCs at the device ports predict the device's response to
  // their own force (control/predictive_pc.hpp) instead of assuming the
  // velocity stays constant over the sample. The model masses are what the
  // controllers believe; 0 means the true mass.
  bool predictive_pc = true;
  double master_model_mass = 0.0;  // kg
  double slave_model_mass = 0.0;   // kg
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
  // Minimum over time of E_in(master) - E_out(slave) and of
  // E_in(slave) - E_out(master) at the two ends of the channel, from true
  // (undelayed) books: the per-direction condition kChannel enforces.
  // Not computed for kReflective, whose coupling sits inside the two-port.
  double min_channel_margin = 0.0;  // J
};

Teleop1DResult run_teleop1d(const Teleop1DConfig& cfg, net::ChannelModel& forward,
                            net::ChannelModel& backward);

}  // namespace tuushin::sim
