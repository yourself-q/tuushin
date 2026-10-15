#include "tuushin/teleop/controller.hpp"

#include <algorithm>
#include <stdexcept>

namespace tuushin::teleop {

ActuatorSchedule::ActuatorSchedule(Nanos delay, Nanos dt)
    : delay_(delay > 0 ? delay : 0), dt_(static_cast<double>(dt > 0 ? dt : kNanosPerMilli)) {}

namespace {

void append(std::vector<control::ForceSegment>& segs, double phi, Nanos length, double dt) {
  if (length <= 0) return;
  if (!segs.empty() && segs.back().phi == phi) {
    segs.back().samples += static_cast<double>(length) / dt;
  } else {
    segs.push_back({phi, static_cast<double>(length) / dt});
  }
}

}  // namespace

std::vector<control::ForceSegment> ActuatorSchedule::advance(Nanos from, Nanos to,
                                                             Nanos cut_after) {
  std::vector<control::ForceSegment> segs;
  const Nanos cut = cut_after > 0 ? from + cut_after : to + 1;
  Nanos t = from;
  while (!pending_.empty() && pending_.front().first <= to && pending_.front().first <= cut) {
    const Nanos at = std::max(t, pending_.front().first);
    append(segs, active_, at - t, dt_);
    t = at;
    active_ = pending_.front().second;
    pending_.pop_front();
  }
  if (cut <= to) {
    const Nanos at = std::max(t, cut);
    append(segs, active_, at - t, dt_);
    t = at;
    active_ = 0.0;
    pending_.clear();
  }
  append(segs, active_, to - t, dt_);
  if (segs.empty()) segs.push_back({active_, 1.0});
  return segs;
}

std::vector<control::ForceSegment> ActuatorSchedule::lead(Nanos now) const {
  std::vector<control::ForceSegment> segs;
  if (delay_ <= 0) return segs;
  const Nanos end = now + delay_;
  Nanos t = now;
  double phi = active_;
  for (const auto& [at, f] : pending_) {
    const Nanos from = std::clamp(at, t, end);
    append(segs, phi, from - t, dt_);
    t = from;
    phi = f;
  }
  append(segs, phi, end - t, dt_);
  return segs;
}

void ActuatorSchedule::command(Nanos now, double phi) {
  if (delay_ <= 0) {
    active_ = phi;
  } else {
    pending_.emplace_back(now + delay_, phi);
  }
}

namespace {

// Books and observes the sample from `from` to `now` in which the encoder
// saw the displacement dx, and sets the forces in flight as the predictor's
// lead. Returns the work the port forces did on the two-port over it
// (positive = in): each force over the displacement while it acted, which
// the model splits where the force changed within the sample. If the
// watchdog cut the force `cut_after` after the command, the displacement
// while it acted was not seen at all: the model the PC chose the force with
// predicts it, and only an output is booked. Without a usable model a
// sample's displacement is split in proportion to time. A sample far from
// the nominal length is the model's to reject; one during which the force
// was cut is not described by it at all.
double book_and_observe(control::DevicePredictor& dev, ActuatorSchedule& actuator, Nanos from,
                        Nanos now, double dx, double h, Nanos cut_after) {
  const auto segs = actuator.advance(from, now, cut_after);
  double work = dev.realized_work(dx, segs);
  if (cut_after > 0) {
    work = std::min(0.0, work);
    dev.observe_irregular(dx, segs.front().phi);
  } else {
    dev.observe(dx, segs, h);
  }
  dev.set_lead(actuator.lead(now));
  return work;
}

}  // namespace

MasterController::MasterController(const MasterConfig& cfg)
    : cfg_(cfg),
      dt_(to_seconds(cfg.dt)),
      dev_(cfg.model_mass, dt_, cfg.encoder_resolution, cfg.force_rate, cfg.adapt_force_rate),
      actuator_(cfg.actuation_delay, cfg.dt),
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
  // Book the work the forces over the last sample actually did, then
  // display the reflected force through the PC. The velocity is the backward
  // difference of the encoder position, as on a real device.
  if (first_) x_sampled_ = position;
  const bool fresh = first_ || now <= last_now_;
  const Nanos from = fresh ? now - cfg_.dt : last_now_;
  const double h = fresh ? dt_ : to_seconds(now - last_now_);
  const double dx = position - x_sampled_;
  v_meas_ = first_ ? 0.0 : dx / h;
  first_ = false;
  last_now_ = now;
  port_.account_energy(book_and_observe(dev_, actuator_, from, now, dx, h, cut_after));
  x_sampled_ = position;

  Step out;
  const double f_reflected = has_value_ ? f_reflected_ : 0.0;
  if (cfg_.passivity != Passivity::kNone) {
    const auto pc = cfg_.predictive_pc
                        ? control::limit_force_predictive(port_.budget(), f_reflected, dev_,
                                                          cfg_.pc_hold_samples, cfg_.pc_confidence,
                                                          cfg_.pc_tax_samples, cfg_.pc_rise_samples)
                        : control::limit_force(port_, f_reflected, v_meas_, dt_);
    out.display_force = pc.value;
    out.pc_active = pc.active;
  } else {
    out.display_force = f_reflected;
  }
  // Saturation keeps the sign, so it can only reduce the energy leaving
  // (and the tax).
  if (cfg_.max_force > 0.0) {
    out.display_force = std::clamp(out.display_force, -cfg_.max_force, cfg_.max_force);
  }
  if (cfg_.encoder_resolution > 0.0) {
    port_.account_energy(-control::quantization_tax(cfg_.encoder_resolution, dev_.held_force(),
                                                    out.display_force));
  }
  f_display_ = out.display_force;
  actuator_.command(now, f_display_);

  // A coarse encoder's backward difference is mostly quantization noise
  // (q / dt: 5.5 cm/s for 55 um at 1 kHz), which the slave's damping would
  // turn into force noise; the filter's estimate is sent instead once it
  // has a regular history.
  v_sent_ = cfg_.encoder_resolution > 0.0 && dev_.ready() ? dev_.velocity_dt() / dt_ : v_meas_;
  out.send = cfg_.deadband ? enc_.update(Vec1{{v_sent_}}, now) : true;
  if (out.send) {
    out.sample = Sample{v_sent_, position, port_.reported_input_energy()};
    ++sent_;
  }
  return out;
}

SlaveController::SlaveController(const SlaveConfig& cfg, double position)
    : cfg_(cfg),
      dt_(to_seconds(cfg.dt)),
      store_(cfg.initial_energy),
      dev_(cfg.model_mass, dt_, cfg.encoder_resolution, cfg.force_rate, cfg.adapt_force_rate),
      actuator_(cfg.actuation_delay, cfg.dt),
      enc_(cfg.deadband_params),
      x_sampled_(position),
      x_cmd_(position),
      x_master_est_(position) {
  if (cfg.passivity == Passivity::kReflective && cfg.report_contact_force) {
    throw std::invalid_argument(
        "SlaveController: energy reflection needs the coupling force reported, not the contact "
        "force");
  }
}

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
  const bool regular = stepped_ && now > last_now_;
  const Nanos from = regular ? last_now_ : now - cfg_.dt;
  const double h = regular ? to_seconds(now - last_now_) : dt_;
  stepped_ = true;
  last_now_ = now;

  // Book the work the applied forces did on the slave over the last sample;
  // this is the true slave terminal for every scheme, and the energy source
  // or sink of the reflective store.
  const double dx = position - x_sampled_;
  const double work_in = book_and_observe(dev_, actuator_, from, now, dx, h, cut_after);
  terminal_net_ += work_in;
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
    // nothing will leave towards the slave. The predictive PC also pays the
    // force's quantization tax from its grant, and what the forces in flight
    // are expected to take out at worst.
    const double tax =
        control::quantization_tax(cfg_.encoder_resolution, dev_.held_force(), phi);
    const double lead = dev_.lead_samples() > 0.0
                            ? dev_.lead_work() - cfg_.pc_confidence * dev_.lead_sd()
                            : 0.0;
    const double want_slave = !cfg_.predictive_pc ? f_coupling * velocity * dt_
                              : dev_.ready()      ? tax - dev_.work(phi) - lead
                                                  : 0.0;
    const double want_master = -f_slave_ * v_c_ * dt_;
    const auto grant = store_.allocate(want_slave, want_master);
    const auto pc = cfg_.predictive_pc
                        ? control::limit_force_predictive(grant.slave, phi, dev_,
                                                          cfg_.pc_hold_samples, cfg_.pc_confidence,
                                                          cfg_.pc_tax_samples, cfg_.pc_rise_samples)
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
  if (cfg_.encoder_resolution > 0.0) {
    const double tax =
        control::quantization_tax(cfg_.encoder_resolution, dev_.held_force(), -f_ctrl_);
    terminal_net_ -= tax;
    if (cfg_.passivity == Passivity::kReflective) store_.account_slave_energy(-tax);
  }
  actuator_.command(now, -f_ctrl_);
  x_master_est_ += v_received * dt_;
  out.force = f_ctrl_;

  out.send = cfg_.deadband ? enc_.update(Vec1{{f_slave_}}, now) : true;
  if (out.send) {
    const double energy = cfg_.passivity == Passivity::kReflective ? store_.granted()
                                                                   : port_.reported_input_energy();
    out.sample = Sample{f_slave_, 0.0, energy};
    ++sent_;
  }
  return out;
}

}  // namespace tuushin::teleop
