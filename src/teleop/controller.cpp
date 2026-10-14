#include "tuushin/teleop/controller.hpp"

#include <algorithm>

namespace tuushin::teleop {

namespace {

// Work the port force phi, held since the last sample, did on the two-port
// over that sample (positive = in), given the displacement dx the encoder
// saw. If the watchdog cut the force `cut_after` after the command, the
// displacement while it acted was not seen: the model the PC chose the force
// with predicts it, and only an output is booked. Without a usable model the
// whole sample's work is booked if it was an output.
double held_work(const control::DevicePredictor& dev, double phi, double dx, Nanos cut_after,
                 double dt) {
  if (cut_after <= 0) return phi * dx;
  const double predicted = dev.ready() ? dev.hold_work(phi, to_seconds(cut_after) / dt) : phi * dx;
  return std::min(0.0, predicted);
}

// Feeds the last sample, of length h (s), to the device model. A sample far
// from the nominal length is the model's to reject; one during which the
// force was cut is not described by it at all.
void observe(control::DevicePredictor& dev, double dx, double phi, double h, Nanos cut_after) {
  if (cut_after > 0) {
    dev.observe_irregular(dx, phi);
  } else {
    dev.observe(dx, phi, h);
  }
}

}  // namespace

MasterController::MasterController(const MasterConfig& cfg)
    : cfg_(cfg),
      dt_(to_seconds(cfg.dt)),
      dev_(cfg.model_mass, dt_),
      enc_(cfg.deadband_params) {}

void MasterController::receive(std::uint32_t seq, const Sample& s) {
  port_.on_remote_input_energy(s.energy);
  if (!has_value_ || seq > last_seq_) {
    has_value_ = true;
    last_seq_ = seq;
    f_reflected_ = s.value;
  }
}

MasterController::Step MasterController::step(Nanos now, double position, Nanos cut_after) {
  // Book the work the force held over the last sample actually did, then
  // display the reflected force through the PC. The velocity is the backward
  // difference of the encoder position, as on a real device.
  if (first_) x_sampled_ = position;
  const double h = first_ || now <= last_now_ ? dt_ : to_seconds(now - last_now_);
  const double dx = position - x_sampled_;
  v_meas_ = first_ ? 0.0 : dx / h;
  first_ = false;
  last_now_ = now;
  port_.account_energy(held_work(dev_, f_display_, dx, cut_after, dt_));
  observe(dev_, dx, f_display_, h, cut_after);
  x_sampled_ = position;

  Step out;
  const double f_reflected = has_value_ ? f_reflected_ : 0.0;
  if (cfg_.passivity != Passivity::kNone) {
    const auto pc = cfg_.predictive_pc
                        ? control::limit_force_predictive(port_.budget(), f_reflected, dev_,
                                                          cfg_.pc_hold_samples)
                        : control::limit_force(port_, f_reflected, v_meas_, dt_);
    out.display_force = pc.value;
    out.pc_active = pc.active;
  } else {
    out.display_force = f_reflected;
  }
  // Saturation keeps the sign, so it can only reduce the energy leaving.
  if (cfg_.max_force > 0.0) {
    out.display_force = std::clamp(out.display_force, -cfg_.max_force, cfg_.max_force);
  }
  f_display_ = out.display_force;

  out.send = cfg_.deadband ? enc_.update(Vec1{{v_meas_}}, now) : true;
  if (out.send) {
    out.sample = Sample{v_meas_, position, port_.input_energy()};
    ++sent_;
  }
  return out;
}

SlaveController::SlaveController(const SlaveConfig& cfg, double position)
    : cfg_(cfg),
      dt_(to_seconds(cfg.dt)),
      store_(cfg.initial_energy),
      dev_(cfg.model_mass, dt_),
      enc_(cfg.deadband_params),
      x_sampled_(position),
      x_cmd_(position),
      x_master_est_(position) {}

void SlaveController::receive(std::uint32_t seq, const Sample& s) {
  if (cfg_.passivity == Passivity::kReflective) {
    store_.on_master_input_energy(s.energy);
  } else {
    port_.on_remote_input_energy(s.energy);
  }
  if (!has_value_ || seq > last_seq_) {
    has_value_ = true;
    fresh_ = true;
    last_seq_ = seq;
    last_ = s;
  }
}

SlaveController::Step SlaveController::step(Nanos now, double position, double velocity,
                                            double contact_force, Nanos cut_after) {
  const bool fresh = fresh_;
  fresh_ = false;
  if (fresh) fresh_at_ = now;
  const double h = stepped_ && now > last_now_ ? to_seconds(now - last_now_) : dt_;
  stepped_ = true;
  last_now_ = now;

  // Book the work the applied force did on the slave over the last sample;
  // this is the true slave terminal for every scheme, and the energy source
  // or sink of the reflective store.
  const double dx = position - x_sampled_;
  const double work_in = held_work(dev_, -f_ctrl_, dx, cut_after, dt_);
  terminal_net_ += work_in;
  observe(dev_, dx, -f_ctrl_, h, cut_after);
  x_sampled_ = position;

  const bool live = has_value_ && (cfg_.hold_after <= 0 || now - fresh_at_ <= cfg_.hold_after);
  const double v_received = live ? last_.value : 0.0;
  if (fresh) x_master_est_ = last_.aux;
  v_requested_ = v_received;

  Step out;
  if (cfg_.passivity == Passivity::kReflective) {
    // The coupling follows the delayed master position itself. The PC scales
    // the coupling force on the slave device only, so the command is never
    // altered and cannot drift.
    store_.account_slave_energy(work_in);
    x_cmd_ = x_master_est_;
    v_c_ = v_received;
    const double f_coupling = cfg_.kp * (x_cmd_ - position) + cfg_.kd * (v_c_ - velocity);
    // The coupling force is reported even when the PC applies less of it:
    // it is how the slave asks the operator for the energy it lacks. If the
    // applied force were reported instead, a slave without energy would
    // report nothing, the master would never push into anything, and no
    // energy would ever enter (the slave then never moves, even without
    // delay).
    f_slave_ = cfg_.report_contact_force ? contact_force : f_coupling;
    // Predicted outputs of the two-port over the coming sample: the work the
    // coupling would do on the slave, and the energy the reflected force
    // would return to the master, evaluated with the delayed master velocity
    // at the channel end (Panzirsch et al.'s P_3^{R2L}). The PCs' sign
    // convention is power *into* the two-port, which at the slave is the
    // slave pushing against the coupling: phi = -f.
    const double phi = -f_coupling;
    // A predictor that is not ready lets no force through (see step()), so
    // nothing will leave towards the slave.
    const double want_slave = !cfg_.predictive_pc ? f_coupling * velocity * dt_
                              : dev_.ready()      ? -dev_.work(phi)
                                                  : 0.0;
    const double want_master = -f_slave_ * v_c_ * dt_;
    const auto grant = store_.allocate(want_slave, want_master);
    const auto pc = cfg_.predictive_pc
                        ? control::limit_force_predictive(grant.slave, phi, dev_,
                                                          cfg_.pc_hold_samples)
                        : control::limit_force(grant.slave, phi, velocity, dt_);
    f_ctrl_ = -pc.value;
    out.pc_active = pc.active || grant.limited;
  } else {
    // The coupling force for this step follows from the command up to the
    // previous step, so the force the PC books is exactly the force applied
    // during the step (Ryu & Preusche, ICRA 2007, take f_s(n) as a function
    // of v_sd(n-1) for the same reason). Recomputing the force from the new
    // command would add kd * (v_c(n) - v_c(n-1)) to the applied force behind
    // the PC's back, and large command jumps (drift compensation) then make
    // the loop active although the books balance.
    const double f_coupling = cfg_.kp * (x_cmd_ - position) + cfg_.kd * (v_c_ - velocity);
    f_slave_ = cfg_.report_contact_force ? contact_force : f_coupling;
    if (cfg_.drift_gain > 0.0 && has_value_) {
      const double ref =
          cfg_.drift_reference == SlaveConfig::DriftReference::kSlavePosition ? position : x_cmd_;
      v_requested_ += cfg_.drift_gain * (x_master_est_ - ref);
    }
    if (cfg_.passivity == Passivity::kChannel) {
      const auto pc = control::passivate_velocity(port_, v_requested_, f_slave_, dt_);
      v_c_ = pc.value;
      out.pc_active = pc.active;
    } else {
      v_c_ = v_requested_;
      port_.account(-f_slave_ * v_c_, dt_);
    }
    f_ctrl_ = f_coupling;
    x_cmd_ += v_c_ * dt_;
  }
  x_master_est_ += v_received * dt_;
  out.force = f_ctrl_;

  out.send = cfg_.deadband ? enc_.update(Vec1{{f_slave_}}, now) : true;
  if (out.send) {
    const double energy = cfg_.passivity == Passivity::kReflective ? store_.granted()
                                                                   : port_.input_energy();
    out.sample = Sample{f_slave_, 0.0, energy};
    ++sent_;
  }
  return out;
}

}  // namespace tuushin::teleop
