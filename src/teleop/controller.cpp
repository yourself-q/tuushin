#include "tuushin/teleop/controller.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
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

const std::vector<control::ForceSegment>& ActuatorSchedule::advance(Nanos from, Nanos to,
                                                                    Nanos cut_after,
                                                                    Nanos issued) {
  auto& segs = segs_;
  segs.clear();
  const Nanos cut = cut_after > 0 ? issued + cut_after : to + 1;
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

const std::vector<control::ForceSegment>& ActuatorSchedule::lead(Nanos from, Nanos issue) {
  auto& segs = lead_;
  segs.clear();
  const Nanos end = effective(issue);
  if (end <= from) return segs;
  Nanos t = from;
  double phi = active_;
  for (const auto& [at, f] : pending_) {
    const Nanos start = std::clamp(at, t, end);
    append(segs, phi, start - t, dt_);
    t = start;
    phi = f;
  }
  append(segs, phi, end - t, dt_);
  return segs;
}

void ActuatorSchedule::command(Nanos now, double phi) {
  // Even without a delay the command waits for advance(): the sample it
  // ends may have been read before it was issued.
  pending_.emplace_back(effective(now), phi);
}

namespace {

// How long the candidates' scores remember a miss: the time constant of
// their running mean, in s.
constexpr double kScoreSeconds = 1.0;
// A sample that even the candidate that predicted it best missed by more
// than this, in units of the least score (three standard deviations), is
// one no candidate's model describes, such as an impact, and none scores.
constexpr double kScoreGate = 9.0;
// The 95 % point of chi-square with one degree of freedom: the likelihood
// ratio a candidate may trail the best by and still be taken (above).
constexpr double kScoreChiSquare = 3.84;
// How long the readings must keep to a shorter delay before it is taken.
// A contact the gate lets through favours short delays for some 20 ms; on
// a simulated slave reaching a wall that dropped a 2 ms delay to 0, and the
// teleoperator generated 2 mJ. A hold much longer keeps a delay that
// started too long for longer than its booking can afford: from 4 ms down
// to 0-2 ms, 0.3 s let the books run up to 0.5 mJ ahead of the devices.
constexpr double kShorterSeconds = 0.1;
// Adapting the candidates' force walk, as DevicePredictor adapts its own:
// per second, the most one innovation counts for, the largest factor.
constexpr double kAdaptPerSecond = 3.0;
constexpr double kAdaptNisClip = 9.0;
constexpr double kAdaptLogMax = 5.545177444479562;  // 2 ln 16
// The share of the reflective store, beyond its grant, that the slave
// device's work cap lets it take out (SlaveController::step).
constexpr double kCapSpareShare = 0.5;
// How many readings after a tax its bound on where the device lay in its
// count is narrowed by the readings since (DeviceModel::book_tax).
constexpr std::size_t kRefundLag = 12;
// The master's rise horizon in the reflective architecture when its
// configuration leaves it unset (MasterConfig::pc_rise_samples).
constexpr int kReflectiveRiseSamples = 64;

// The work into the two-port over a sample: the device's measured work
// when it reports it, else the device model's. The port force phi opposes
// the device's force, so the work in is minus the increase of the work the
// device's force did. The first measurement only sets the origin.
double booked_work(std::optional<double>& last, const std::optional<double>& measured,
                   double modeled) {
  if (!measured) return modeled;
  const double w = last ? *last - *measured : 0.0;
  last = measured;
  return w;
}

}  // namespace

DeviceModel::DeviceModel(double mass, Nanos dt, double resolution, double force_rate,
                         bool adapt_force_rate, Nanos delay, Nanos max_delay, Nanos step,
                         bool estimate_noise)
    : dt_(to_seconds(dt)), adapt_(adapt_force_rate), refund_lag_(kRefundLag) {
  if (estimate_noise && resolution > 0.0) {
    noise_.emplace(to_seconds(dt), resolution);
  }
  auto add = [&](Nanos d) {
    bank_.push_back({d, ActuatorSchedule(d, dt),
                     control::DevicePredictor(mass, to_seconds(dt), resolution, force_rate,
                                              adapt_force_rate),
                     control::HeldDeviceFilter(mass, to_seconds(dt), resolution, force_rate)});
  };
  if (max_delay <= 0) {
    add(delay);
    return;
  }
  if (step <= 0) step = std::max<Nanos>(1, dt / 16);
  for (Nanos d = 0; d <= max_delay; d += step) {
    add(d);
    // Start from the candidate nearest the guess.
    if (std::llabs(d - delay) < std::llabs(bank_[best_].delay - delay)) best_ = bank_.size() - 1;
  }
}

bool DeviceModel::on_unseen_reading() const {
  // A lead of whole samples: the command takes effect on a later reading,
  // not within a sample, which the model would split by readings it saw.
  const double lead = predictor().lead_samples();
  return lead >= 1.0 - 1e-9 && std::fabs(lead - std::round(lead)) <= 1e-6;
}

void DeviceModel::set_noise_tax(double lo, double hi, double delta, double force_scale) {
  const double q = predictor().resolution();
  if (!(q > 0.0) || !(hi > 0.0)) {
    noise_tax_.reset();
    return;
  }
  const auto d = control::dither_bounds(lo, hi);
  noise_tax_.emplace(q * d.bias, q * d.proxy, delta, force_scale);
}

const control::NoiseTax* DeviceModel::change_tax() const {
  return noise_tax_ && on_unseen_reading() ? &*noise_tax_ : nullptr;
}

DeviceModel::Tax DeviceModel::book_tax(double phi) {
  Tax t;
  const control::DevicePredictor& dev = predictor();
  const double q = dev.resolution();
  if (!(q > 0.0)) return t;
  if (noise_tax_) {
    // Opened with the first command, before any force: U(0) is a debt the
    // first rise must repay, as with a deficit.
    t.charged += noise_tax_->open();
    if (on_unseen_reading()) {
      t.charged += noise_tax_->charge(dev.held_force(), phi);
      return t;
    }
  }
  const auto line = control::quantization_tax_line(q, dev.held_force(), phi);
  if (line.beta == 0.0) {
    t.charged += line.alpha;
  } else {
    const auto u = dev.count_bounds();
    t.charged += line.worst(u.lo, u.hi);
    if (!taxes_.empty() && taxes_.back().reading == readings_) {
      // A second command on the same reading acts from later than it, where
      // its bounds no longer hold: neither tax is refunded.
      taxes_.pop_back();
    } else if (refund_lag_ > 0 && dev.lead_samples() < 1.0) {
      taxes_.push_back({readings_, line, u.lo, u.hi, t.charged});
    }
  }
  // The readings since a tax tell where the device lay when it was booked
  // better than those before it; the true u lies within both bounds. The
  // taxes settled so far are charged at most an upper bound D on the sum of
  // their true values; each new one raises it by the least of its own bound
  // (termwise) and what bounding it jointly with those settled within the
  // window allows: D before the first of them plus the joint worst. Over a
  // chatter of the port force the joint bound is far below the termwise
  // sum, since the device cannot lie at the worst place for every tax at
  // once.
  const std::size_t window = dev.count_window_readings();
  while (!settled_.empty() &&
         readings_ - settled_.front().reading >= static_cast<long>(window)) {
    settled_.pop_front();
  }
  while (!taxes_.empty() &&
         readings_ - taxes_.front().reading >= static_cast<long>(refund_lag_)) {
    const TaxRecord rec = taxes_.front();
    taxes_.pop_front();
    const auto lag = static_cast<std::size_t>(readings_ - rec.reading);
    const auto later = dev.count_bounds_at(lag);
    const double lo = std::max(rec.lo, later.lo), hi = std::min(rec.hi, later.hi);
    if (lo > hi) {
      // The readings contradict the bounds the tax was charged by: refund
      // nothing, and start the sum afresh.
      settled_.clear();
      continue;
    }
    double net = std::min(rec.charged, rec.line.worst(lo, hi));
    if (!settled_.empty()) {
      std::vector<control::DevicePredictor::LaggedTax> joint;
      double before = 0.0;
      for (const auto& st : settled_) {
        joint.push_back({static_cast<std::size_t>(readings_ - st.reading), st.line});
        before += st.net;
      }
      joint.push_back({lag, rec.line});
      net = std::min(net, dev.worst_tax(joint) - before);
    }
    settled_.push_back({rec.reading, rec.line, net});
    t.refund += rec.charged - net;
  }
  return t;
}

void DeviceModel::set_count_window(std::size_t readings, double force_drift,
                                   std::size_t refund_lag) {
  for (auto& c : bank_) c.dev.set_count_window(readings, force_drift);
  refund_lag_ = refund_lag;
  if (refund_lag_ == 0) {
    taxes_.clear();
    settled_.clear();
  }
}

void DeviceModel::set_actuation_delay(Nanos delay) {
  if (bank_.size() != 1) return;
  bank_[0].actuator.set_delay(delay);
  bank_[0].delay = bank_[0].actuator.delay();
}

// Each candidate books the sample by its own schedule and model: each force
// over the displacement while it acted, which the model splits where the
// force changed within the sample. If the watchdog cut the force
// `cut_after` after the command, the displacement while it acted was not
// seen at all: the model the PC chose the force with predicts it, and only
// an output is booked. Without a usable model a sample's displacement is
// split in proportion to time. A sample far from the nominal length is the
// model's to reject; one during which the force was cut is not described by
// it at all, and neither scores.
double DeviceModel::book_and_observe(Nanos from, Nanos to, double dx, double h,
                                     Nanos cut_after, Nanos issued, Nanos issue) {
  ++readings_;
  double work = 0.0;
  double least_miss = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < bank_.size(); ++i) {
    auto& c = bank_[i];
    const auto& segs = c.actuator.advance(from, to, cut_after, issued);
    if (noise_ && i == best_) {
      if (cut_after > 0) {
        noise_->interrupt();
      } else {
        noise_->observe(dx, h, c.dev.expected_velocity_change(segs, h));
      }
    }
    double w = c.dev.realized_work(dx, segs);
    c.miss = 0.0;
    if (cut_after > 0) {
      w = std::min(0.0, w);
      c.dev.observe_irregular(dx, segs.front().phi);
    } else {
      c.dev.observe(dx, segs, h);
    }
    if (bank_.size() > 1) {
      c.id.observe(dx, segs, h);
      c.miss = c.id.innovation() * c.id.innovation();
      least_miss = std::min(least_miss, c.miss);
    }
    c.dev.set_lead(c.actuator.lead(to, issue));
    if (i == best_) work = w;
  }
  if (noise_) {
    // For the next sample: a quarter of a second's estimate does not miss
    // one sample's lag.
    for (auto& c : bank_) {
      c.dev.set_reading_noise(noise_->variance());
      c.id.set_reading_noise(noise_->variance());
    }
  }
  const bool regular = std::fabs(h - dt_) <= 0.5 * dt_;
  if (bank_.size() > 1 && adapt_ && cut_after <= 0 && regular) {
    // A hand moves the device by a force that changes faster than any one
    // walk suits: H's walk is adapted as DevicePredictor adapts its own, by
    // the chosen candidate's innovations, and shared by all candidates, so
    // that their scores differ by their delays alone. With a walk too slow
    // for the hand the filters put its force into the damping instead.
    const double nis = std::min(bank_[best_].id.normalized_innovation_squared(), kAdaptNisClip);
    log_walk_ = std::clamp(log_walk_ + kAdaptPerSecond * h * (nis - 1.0), 0.0, kAdaptLogMax);
    for (auto& c : bank_) c.id.set_force_walk_scale(std::exp(log_walk_));
  }
  if (bank_.size() <= 1 || cut_after > 0 || !regular || least_miss == 0.0) return work;
  double least = std::numeric_limits<double>::infinity();
  for (const auto& c : bank_) least = std::min(least, c.score);
  if (n_scored_ > 0 && least_miss > kScoreGate * least) return work;
  ++n_scored_;
  const double a = std::max(std::min(1.0, h / kScoreSeconds), 1.0 / static_cast<double>(n_scored_));
  least = std::numeric_limits<double>::infinity();
  for (auto& c : bank_) {
    c.score += a * (c.miss - c.score);
    least = std::min(least, c.score);
  }
  // The scores are running means over about n samples; with Gaussian
  // innovations n ln(score / least) is the likelihood ratio of a candidate
  // against the best, and the candidates under the chi-square bound are the
  // delays the readings cannot tell from it (Wilks). Of those, the longest
  // is used: a delay taken too short books a force as acting before it
  // does and lets the PC spend energy the device has not yet received,
  // while one taken too long only holds the force back a little. For the
  // same reason a longer delay is taken at once, a shorter one only once
  // the readings have kept to it for a while (kShorterSeconds).
  const double n = (2.0 - a) / a;
  std::size_t top = 0;
  for (std::size_t i = bank_.size(); i-- > 0;) {
    if (n * std::log(bank_[i].score / least) < kScoreChiSquare) {
      top = i;
      break;
    }
  }
  if (top >= best_) {
    best_ = top;
    shorter_for_ = 0.0;
  } else if ((shorter_for_ += h) >= kShorterSeconds) {
    best_ = top;
    shorter_for_ = 0.0;
  }
  return work;
}

void DeviceModel::refresh_lead(Nanos to, Nanos issue) {
  for (auto& c : bank_) c.dev.set_lead(c.actuator.lead(to, issue));
}

void DeviceModel::command(Nanos now, double phi) {
  for (auto& c : bank_) c.actuator.command(now, phi);
}

MasterController::MasterController(const MasterConfig& cfg)
    : cfg_(cfg),
      dt_(to_seconds(cfg.dt)),
      model_(cfg.model_mass, cfg.dt, cfg.encoder_resolution, cfg.force_rate, cfg.adapt_force_rate,
             cfg.actuation_delay, cfg.actuation_delay_max, cfg.actuation_delay_step,
             cfg.estimate_reading_noise),
      renderer_(cfg.wall_renderer),
      enc_(cfg.deadband_params) {
  model_.set_count_window(static_cast<std::size_t>(std::max(0, cfg.count_window)),
                          cfg.force_drift, static_cast<std::size_t>(std::max(0, cfg.refund_lag)));
  model_.set_noise_tax(cfg.encoder_noise_lo, cfg.encoder_noise_hi, cfg.noise_tax_delta,
                       cfg.noise_tax_force_scale);
}

void MasterController::receive(std::uint32_t seq, const Sample& s) {
  const bool newest = !has_value_ || seq > last_seq_;
  if (cfg_.passivity == Passivity::kModelMediated) {
    if (newest) {
      has_value_ = true;
      last_seq_ = seq;
      received_model_ = control::WallModel{s.value, s.aux, s.energy};
    }
    return;
  }
  port_.on_remote_input_energy(s.energy);
  if (newest) {
    has_value_ = true;
    last_seq_ = seq;
    f_reflected_ = s.value;
  }
}

double MasterController::balance() const {
  // The model is the master's own: what the operator put into the port is
  // all it may give back.
  return cfg_.passivity == Passivity::kModelMediated ? port_.input_energy() - port_.output_energy()
                                                     : port_.balance();
}

MasterController::Step MasterController::step(Nanos now, Nanos sampled, double position,
                                              Nanos cut_after, std::optional<double> port_work,
                                              std::optional<double> work_reserved) {
  // Book the work the forces over the last sample actually did, then
  // display the reflected force through the PC. The velocity is the backward
  // difference of the encoder position, as on a real device.
  const Nanos at = std::min(sampled, now);
  if (!first_ && at < now && at <= last_now_) {
    // The device has not been read anew since the last step.
    model_.refresh_lead(last_now_, now);
  } else {
    if (first_) x_sampled_ = position;
    const bool fresh = first_ || at <= last_now_;
    const Nanos from = fresh ? at - cfg_.dt : last_now_;
    const double h = fresh ? dt_ : to_seconds(at - last_now_);
    const Nanos issued = first_ ? from : last_issued_;
    const double dx = position - x_sampled_;
    v_meas_ = first_ ? 0.0 : dx / h;
    first_ = false;
    last_now_ = at;
    port_.account_energy(booked_work(
        port_work_, port_work, model_.book_and_observe(from, at, dx, h, cut_after, issued, now)));
    x_sampled_ = position;
  }
  last_issued_ = now;
  const control::DevicePredictor& dev = model_.predictor();

  // A coarse encoder's backward difference is mostly quantization noise
  // (q / dt: 5.5 cm/s for 55 um at 1 kHz), which the slave's damping would
  // turn into force noise; the filter's estimate is sent instead once it
  // has a regular history.
  v_sent_ = cfg_.encoder_resolution > 0.0 && dev.ready() ? dev.velocity_dt() / dt_ : v_meas_;

  Step out;
  if (cfg_.passivity == Passivity::kModelMediated) {
    if (received_model_) {
      renderer_.update(*received_model_, position);
      received_model_.reset();
    }
    f_reflected_ = renderer_.force(position, v_sent_, dt_);
  }
  const double f_reflected = has_value_ ? f_reflected_ : 0.0;
  if (cfg_.passivity != Passivity::kNone) {
    const auto pc = cfg_.predictive_pc
                        ? control::limit_force_predictive(
                              std::max(0.0, balance()), f_reflected, dev, cfg_.pc_hold_samples,
                              cfg_.pc_confidence, cfg_.pc_tax_samples,
                              cfg_.pc_rise_samples.value_or(
                                  cfg_.passivity == Passivity::kModelMediated
                                      ? 0
                                      : kReflectiveRiseSamples),
                              model_.change_tax())
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
    const auto tax = model_.book_tax(out.display_force);
    port_.account_energy(tax.refund - tax.charged);
  }
  f_display_ = out.display_force;
  model_.command(now, f_display_);
  if (cfg_.passivity != Passivity::kNone && port_work_) {
    // With the port in debt the cap is below the work: the device takes
    // the debt back out before its force does work again. A cap at the
    // work would let it spend again what paid the debt, and each overrun
    // the device's own misprediction makes would stay owed.
    out.work_cap = *port_work_ + balance();
  }
  // The device may take the port's work up to the largest cap it still
  // holds, and an input booked since that cap was sent lowered the work
  // without raising the budget: it was reported, and the device can
  // spend it again. So what the caps reach beyond the budget is held back
  // from the report until the device has commands with smaller ones.
  const double hold = work_reserved ? std::max(0.0, *work_reserved - port_.budget()) : 0.0;
  reported_ = std::max(reported_, port_.reported_input_energy() - hold);

  out.send = cfg_.deadband ? enc_.update(Vec1{{v_sent_}}, now) : true;
  if (cfg_.passivity == Passivity::kModelMediated) {
    x_command_ = position - renderer_.offset();
    out.send = true;
    out.sample = Sample{v_sent_, x_command_, f_display_};
    ++sent_;
  } else if (out.send) {
    out.sample = Sample{v_sent_, position, reported_};
    ++sent_;
  }
  return out;
}

SlaveController::SlaveController(const SlaveConfig& cfg, double position)
    : cfg_(cfg),
      dt_(to_seconds(cfg.dt)),
      store_(cfg.initial_energy),
      model_(cfg.model_mass, cfg.dt, cfg.encoder_resolution, cfg.force_rate, cfg.adapt_force_rate,
             cfg.actuation_delay, cfg.actuation_delay_max, cfg.actuation_delay_step,
             cfg.estimate_reading_noise),
      estimator_(cfg.wall_estimator),
      enc_(cfg.deadband_params),
      x_sampled_(position),
      x_cmd_(position),
      x_master_est_(position) {
  model_.set_count_window(static_cast<std::size_t>(std::max(0, cfg.count_window)),
                          cfg.force_drift, static_cast<std::size_t>(std::max(0, cfg.refund_lag)));
  model_.set_noise_tax(cfg.encoder_noise_lo, cfg.encoder_noise_hi, cfg.noise_tax_delta,
                       cfg.noise_tax_force_scale);
  if (cfg.passivity == Passivity::kReflective && cfg.report_contact_force) {
    throw std::invalid_argument(
        "SlaveController: energy reflection needs the coupling force reported, not the contact "
        "force");
  }
}

void SlaveController::receive(std::uint32_t seq, const Sample& s) {
  if (cfg_.passivity == Passivity::kReflective) {
    store_.on_master_input_energy(s.energy);
  } else if (cfg_.passivity != Passivity::kModelMediated) {
    port_.on_remote_input_energy(s.energy);
  }
  if (!has_value_ || seq > last_seq_) {
    has_value_ = true;
    fresh_ = true;
    last_seq_ = seq;
    last_ = s;
  }
}

SlaveController::Step SlaveController::step(Nanos now, Nanos sampled, double position,
                                            double velocity, double contact_force,
                                            Nanos cut_after, std::optional<double> port_work,
                                            std::optional<double> work_reserved) {
  const bool fresh = fresh_;
  fresh_ = false;
  if (fresh) fresh_at_ = now;

  // Book the work the applied forces did on the slave over the last sample;
  // this is the true slave terminal for every scheme, and the energy source
  // or sink of the reflective store. A reading no newer than the last one
  // books nothing (MasterController::step).
  const Nanos at = std::min(sampled, now);
  double work_in = 0.0;
  if (stepped_ && at < now && at <= last_now_) {
    model_.refresh_lead(last_now_, now);
  } else {
    const bool regular = stepped_ && at > last_now_;
    const Nanos from = regular ? last_now_ : at - cfg_.dt;
    const double h = regular ? to_seconds(at - last_now_) : dt_;
    const Nanos issued = stepped_ ? last_issued_ : from;
    stepped_ = true;
    last_now_ = at;
    const double dx = position - x_sampled_;
    work_in = booked_work(port_work_, port_work,
                          model_.book_and_observe(from, at, dx, h, cut_after, issued, now));
    x_sampled_ = position;
  }
  last_issued_ = now;
  terminal_net_ += work_in;
  const control::DevicePredictor& dev = model_.predictor();

  const bool live = has_value_ && (cfg_.hold_after <= 0 || now - fresh_at_ <= cfg_.hold_after);
  const double v_received = live ? last_.value : 0.0;
  if (fresh) x_master_est_ = last_.aux;
  v_requested_ = v_received;

  Step out;
  control::ReflectiveStore::Grant grant;
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
    const control::NoiseTax* noise = model_.change_tax();
    const double tax =
        noise ? noise->cost(dev.held_force(), phi) : control::quantization_tax(dev, phi);
    const double lead = dev.lead_samples() > 0.0
                            ? dev.lead_work() - cfg_.pc_confidence * dev.lead_sd()
                            : 0.0;
    const double want_slave = !cfg_.predictive_pc ? f_coupling * velocity * dt_
                              : dev.ready()      ? tax - dev.work(phi) - lead
                                                  : 0.0;
    const double want_master = -f_slave_ * v_c_ * dt_;
    grant = store_.allocate(want_slave, want_master, work_reserved.value_or(0.0));
    const auto pc = cfg_.predictive_pc
                        ? control::limit_force_predictive(grant.slave, phi, dev,
                                                          cfg_.pc_hold_samples, cfg_.pc_confidence,
                                                          cfg_.pc_tax_samples, cfg_.pc_rise_samples,
                                                          noise)
                        : control::limit_force(grant.slave, phi, velocity, dt_);
    f_ctrl_ = -pc.value;
    out.pc_active = pc.active || grant.limited;
  } else if (cfg_.passivity == Passivity::kModelMediated) {
    // A position servo on the master's model-consistent position, with the
    // force the operator feels as a feedforward: in contact with a wall the
    // model describes, the slave then rests where the master is and pushes
    // as hard as the operator does, whatever the coupling's stiffness.
    estimator_.observe(position, velocity, contact_force);
    const double f_ff = live ? std::max(0.0, last_.energy) : 0.0;
    x_cmd_ = x_master_est_;
    v_c_ = v_received;
    const control::WallModel& wall = estimator_.model();
    if (cfg_.consistent_commands && wall.known()) {
      const double limit = wall.surface + f_ff / wall.stiffness;
      if (x_cmd_ > limit) {
        x_cmd_ = limit;
        v_c_ = std::min(v_c_, 0.0);
      }
    }
    f_ctrl_ = cfg_.kp * (x_cmd_ - position) + cfg_.kd * (v_c_ - velocity) + f_ff;
    f_slave_ = contact_force;
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
  const auto booked = cfg_.encoder_resolution > 0.0 ? model_.book_tax(-f_ctrl_) : DeviceModel::Tax{};
  const double tax = booked.charged;
  terminal_net_ += booked.refund - tax;
  if (cfg_.passivity == Passivity::kReflective) {
    store_.account_slave_energy(booked.refund - tax);
    if (port_work_) {
      // The device's cap: its grant, less the tax, which is booked and
      // is no work of the device's, and a share of what the store holds
      // beyond, which the master then cannot be granted while the cap is
      // in effect. Without the share a misprediction would scale the
      // force down at once; with all of it the master would get nothing.
      // A store in deficit puts the cap below the work, as at the master.
      const double level = store_.level();
      const double spare = std::max(0.0, level);
      const double own = std::clamp(grant.slave - tax, 0.0, spare);
      out.work_cap = *port_work_ + std::min(0.0, level) + own + kCapSpareShare * (spare - own);
    }
  }
  model_.command(now, -f_ctrl_);
  x_master_est_ += v_received * dt_;
  out.force = f_ctrl_;

  if (cfg_.passivity == Passivity::kModelMediated) {
    // The model, every sample: the master renders whichever came last.
    const control::WallModel& wall = estimator_.model();
    out.send = true;
    out.sample = Sample{wall.stiffness, wall.surface, wall.damping};
    ++sent_;
    return out;
  }
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
