#include "tuushin/sim/teleop1d.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <queue>
#include <stdexcept>
#include <vector>

#include "tuushin/rng.hpp"

namespace tuushin::sim {

namespace {

constexpr double kPi = 3.14159265358979323846;

// A haptic sample in flight.
struct InFlight {
  Nanos arrival;
  std::uint32_t seq;
  teleop::Sample sample;
};

struct LaterArrival {
  bool operator()(const InFlight& a, const InFlight& b) const {
    if (a.arrival != b.arrival) return a.arrival > b.arrival;
    return a.seq > b.seq;
  }
};

using InFlightQueue = std::priority_queue<InFlight, std::vector<InFlight>, LaterArrival>;

struct Sender {
  std::uint32_t next_seq = 1;
  std::uint64_t sent = 0;
  std::uint64_t lost = 0;

  void send(net::ChannelModel& ch, InFlightQueue& q, Nanos now, std::size_t bytes,
            const teleop::Sample& s) {
    const auto transit = ch.transit(now, bytes);
    ++sent;
    if (transit) {
      q.push(InFlight{now + *transit, next_seq, s});
    } else {
      ++lost;
    }
    ++next_seq;
  }
};

}  // namespace

Teleop1DResult run_teleop1d(const Teleop1DConfig& cfg, net::ChannelModel& forward,
                            net::ChannelModel& backward) {
  if (cfg.dt <= 0 || cfg.substeps <= 0 || cfg.duration <= 0) {
    throw std::invalid_argument("run_teleop1d: dt, substeps and duration must be positive");
  }
  if (cfg.sample_jitter < 0 || 2 * cfg.sample_jitter >= cfg.dt) {
    throw std::invalid_argument("run_teleop1d: sample_jitter must be in [0, dt / 2)");
  }
  using Passivity = Teleop1DConfig::Passivity;
  const bool reflective = cfg.passivity == Passivity::kReflective;
  const double dt = to_seconds(cfg.dt);
  const std::int64_t steps = cfg.duration / cfg.dt;

  std::unique_ptr<std::FILE, int (*)(std::FILE*)> log(nullptr, &std::fclose);
  if (!cfg.log_path.empty()) {
    log.reset(std::fopen(cfg.log_path.c_str(), "w"));
    if (!log) throw std::runtime_error("cannot open log: " + cfg.log_path);
    std::fprintf(log.get(),
                 "t,x_ref,x_m,v_m,f_hand,f_display,x_s,v_s,x_cmd,x_master_est,v_req,v_c,f_slave,"
                 "f_contact,ein_m,eout_m,ein_s,eout_s,store,grant_m,pc_m,pc_s,sent_fwd,sent_bwd\n");
  }

  // Physics: the devices integrate between control steps with the forces
  // the controllers held (teleop/device.hpp).
  teleop::SimulatedMaster master_dev(cfg.master.mass, cfg.master.damping, cfg.op, cfg.dt,
                                     cfg.substeps);
  teleop::SimulatedSlave slave_dev(cfg.slave.mass, cfg.slave.damping, cfg.wall, cfg.dt,
                                   cfg.substeps, 0, cfg.op.ref_offset);
  master_dev.set_encoder_resolution(cfg.master_encoder_resolution);
  slave_dev.set_encoder_resolution(cfg.slave_encoder_resolution);
  master_dev.set_friction(cfg.master.friction);
  slave_dev.set_friction(cfg.slave.friction);
  master_dev.set_compliance(cfg.master.compliance);
  slave_dev.set_compliance(cfg.slave.compliance);
  master_dev.set_actuation_delay(cfg.master.actuation_delay);
  slave_dev.set_actuation_delay(cfg.slave.actuation_delay);
  Rng jitter(cfg.jitter_seed);
  auto sample_time = [&](Nanos nominal) {
    if (cfg.sample_jitter <= 0) return nominal;
    const double u = 2.0 * jitter.uniform() - 1.0;
    return nominal + static_cast<Nanos>(std::llround(u * static_cast<double>(cfg.sample_jitter)));
  };

  teleop::MasterConfig mc;
  mc.dt = cfg.dt;
  mc.passivity = cfg.passivity;
  mc.predictive_pc = cfg.predictive_pc;
  mc.model_mass = cfg.master_model_mass > 0.0 ? cfg.master_model_mass : cfg.master.mass;
  mc.pc_hold_samples = cfg.pc_hold_samples;
  mc.max_force = cfg.master.max_force;
  mc.encoder_resolution = cfg.encoder_resolution_known ? cfg.master_encoder_resolution : 0.0;
  mc.force_rate = cfg.force_rate;
  mc.adapt_force_rate = cfg.adapt_force_rate;
  mc.pc_confidence = cfg.pc_confidence;
  mc.pc_tax_samples = cfg.pc_tax_samples;
  mc.pc_rise_samples = cfg.master_pc_rise_samples;
  mc.deadband = cfg.forward_deadband;
  mc.deadband_params = cfg.forward_deadband_params;
  mc.actuation_delay = cfg.master_model_actuation_delay >= 0 ? cfg.master_model_actuation_delay
                                                              : cfg.master.actuation_delay;
  teleop::MasterController master(mc);

  teleop::SlaveConfig sc;
  sc.dt = cfg.dt;
  sc.passivity = cfg.passivity;
  sc.kp = cfg.slave.kp;
  sc.kd = cfg.slave.kd;
  sc.report_contact_force = cfg.slave.report_contact_force;
  sc.drift_gain = cfg.slave.drift_gain;
  sc.drift_reference = cfg.slave.drift_reference;
  sc.initial_energy = cfg.initial_energy;
  sc.predictive_pc = cfg.predictive_pc;
  sc.model_mass = cfg.slave_model_mass > 0.0 ? cfg.slave_model_mass : cfg.slave.mass;
  sc.pc_hold_samples = cfg.pc_hold_samples;
  sc.encoder_resolution = cfg.encoder_resolution_known ? cfg.slave_encoder_resolution : 0.0;
  sc.force_rate = cfg.force_rate;
  sc.adapt_force_rate = cfg.adapt_force_rate;
  sc.pc_confidence = cfg.pc_confidence;
  sc.pc_tax_samples = cfg.pc_tax_samples;
  sc.pc_rise_samples = cfg.slave_pc_rise_samples;
  sc.deadband = cfg.backward_deadband;
  sc.deadband_params = cfg.backward_deadband_params;
  sc.actuation_delay = cfg.slave_model_actuation_delay >= 0 ? cfg.slave_model_actuation_delay
                                                            : cfg.slave.actuation_delay;
  teleop::SlaveController slave(sc, slave_dev.read_position(0));

  Sender fwd_sender, bwd_sender;
  InFlightQueue fwd_queue, bwd_queue;

  Teleop1DResult r;
  double sum_pos_err2 = 0.0, sum_force_err2 = 0.0;
  double sum_kp_num = 0.0, sum_kp_den = 0.0;
  double sum_pc_err2 = 0.0, variation = 0.0, f_prev = 0.0;
  double drift_sum = 0.0;
  std::int64_t drift_n = 0;
  std::int64_t pc_m_count = 0, pc_s_count = 0;
  double min_margin = 0.0, min_channel_margin = 0.0, min_true = 0.0, max_excess = 0.0;
  std::int64_t done = 0;

  auto check_plausible = [&](Nanos now) {
    if (master_dev.plausible() && slave_dev.plausible()) return true;
    r.diverged = true;
    r.diverged_at = to_seconds(now);
    return false;
  };

  for (std::int64_t k = 0; k < steps; ++k) {
    const Nanos now = k * cfg.dt;
    const double t = to_seconds(now);
    // When each end samples, k = 0 on time.
    const Nanos now_m = k > 0 ? sample_time(now) : now;
    const Nanos now_s = k > 0 ? sample_time(now) : now;

    // 1. Deliver everything that has arrived by then.
    while (!fwd_queue.empty() && fwd_queue.top().arrival <= now_s) {
      slave.receive(fwd_queue.top().seq, fwd_queue.top().sample);
      fwd_queue.pop();
    }
    while (!bwd_queue.empty() && bwd_queue.top().arrival <= now_m) {
      master.receive(bwd_queue.top().seq, bwd_queue.top().sample);
      bwd_queue.pop();
    }

    // 2. Sample both devices; this integrates the physics over the last
    //    sample with the forces that were held during it.
    const double x_m = master_dev.read_position(now_m);
    const double x_s = slave_dev.read_position(now_s);
    if (!check_plausible(now)) break;
    const double v_m = master_dev.true_velocity();
    const double v_s = slave_dev.velocity();

    // 3. Master step.
    const auto ms = master.step(now_m, x_m, master_dev.force_cut_after());
    master_dev.command_force(-ms.display_force);
    const double f_display = ms.display_force;
    if (ms.send) fwd_sender.send(forward, fwd_queue, now_m, cfg.packet_bytes, ms.sample);

    // 4. Slave step.
    const double f_contact = slave_dev.contact_force();
    const auto ss = slave.step(now_s, x_s, v_s, f_contact, slave_dev.force_cut_after());
    slave_dev.command_force(ss.force);
    if (ss.send) bwd_sender.send(backward, bwd_queue, now_s, cfg.packet_bytes, ss.sample);

    // Metrics on the samples of this step.
    const auto& mp = master.port();
    const double terminal_net = mp.input_energy() - mp.output_energy() + slave.terminal_net();
    min_margin = std::min(min_margin, terminal_net);
    // The actuators do work on the devices; what enters the teleoperator is
    // its negative.
    const double true_net = -master_dev.actuator_work() - slave_dev.actuator_work();
    min_true = std::min(min_true, true_net);
    max_excess = std::max(max_excess, terminal_net - true_net);
    if (!reflective) {
      const auto& sp = slave.channel_port();
      // Each end's outputs, less what it paid from its own input, against
      // what the other end reported; the two margins sum to the net.
      const double margin_fwd =
          mp.reported_input_energy() - (sp.output_energy() - sp.covered_energy());
      const double margin_bwd =
          sp.reported_input_energy() - (mp.output_energy() - mp.covered_energy());
      min_channel_margin = std::min({min_channel_margin, margin_fwd, margin_bwd});
    }
    if (ms.pc_active) ++pc_m_count;
    if (ss.pc_active) ++pc_s_count;

    const double x_ref = cfg.op.ref_offset +
                         cfg.op.ref_amplitude * 0.5 * (1.0 - std::cos(2.0 * kPi * cfg.op.ref_freq * t));
    const double f_hand = master_dev.hand_force();

    if (log) {
      const auto& st = slave.store();
      const auto& sp = slave.channel_port();
      std::fprintf(log.get(),
                   "%.4f,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,"
                   "%.6g,%.6g,%.6g,%.6g,%.6g,%d,%d,%d,%d\n",
                   t, x_ref, x_m, v_m, f_hand, f_display, x_s, v_s, slave.command(),
                   slave.master_estimate(), slave.requested_velocity(), slave.command_velocity(),
                   slave.reported_force(), f_contact, mp.input_energy(), mp.output_energy(),
                   reflective ? st.slave_input_energy() : sp.input_energy(),
                   reflective ? st.slave_output_energy() : sp.output_energy(),
                   reflective ? st.level() : terminal_net, mp.remote_input_energy(),
                   ms.pc_active ? 1 : 0, ss.pc_active ? 1 : 0, ms.send ? 1 : 0, ss.send ? 1 : 0);
    }

    sum_pos_err2 += (x_m - x_s) * (x_m - x_s);
    sum_force_err2 += (f_display - f_contact) * (f_display - f_contact);
    const double pc_err = f_display - master.requested_force();
    sum_pc_err2 += pc_err * pc_err;
    variation += std::fabs(f_display - f_prev);
    f_prev = f_display;
    const double penetration_m = x_m - cfg.wall.position;
    if (penetration_m > 0.0 && f_display > 0.0) {
      sum_kp_num += f_display * penetration_m;
      sum_kp_den += penetration_m * penetration_m;
    }
    if (now >= cfg.duration - kNanosPerSecond) {
      drift_sum += x_m - x_s;
      ++drift_n;
    }
    r.max_master_pos = std::max(r.max_master_pos, std::fabs(x_m));
    r.max_slave_pos = std::max(r.max_slave_pos, std::fabs(x_s));
    r.max_display_force = std::max(r.max_display_force, std::fabs(f_display));
    ++done;
  }
  // The physics of the last sample.
  if (!r.diverged) {
    master_dev.read_position(steps * cfg.dt);
    slave_dev.read_position(steps * cfg.dt);
    check_plausible(steps * cfg.dt);
  }

  const double n = static_cast<double>(std::max<std::int64_t>(done, 1));
  const double sim_s = static_cast<double>(done) * dt;
  r.pos_rmse = std::sqrt(sum_pos_err2 / n);
  r.force_rmse = std::sqrt(sum_force_err2 / n);
  r.pc_force_rmse = std::sqrt(sum_pc_err2 / n);
  r.display_force_variation = sim_s > 0 ? variation / sim_s : 0.0;
  r.displayed_stiffness = sum_kp_den > 0.0 ? sum_kp_num / sum_kp_den : 0.0;
  r.final_drift = drift_n > 0 ? drift_sum / static_cast<double>(drift_n) : 0.0;
  r.forward_packet_rate = sim_s > 0 ? static_cast<double>(fwd_sender.sent) / sim_s : 0.0;
  r.backward_packet_rate = sim_s > 0 ? static_cast<double>(bwd_sender.sent) / sim_s : 0.0;
  r.forward_loss = fwd_sender.sent ? static_cast<double>(fwd_sender.lost) / fwd_sender.sent : 0.0;
  r.backward_loss = bwd_sender.sent ? static_cast<double>(bwd_sender.lost) / bwd_sender.sent : 0.0;
  r.master_pc_active = static_cast<double>(pc_m_count) / n;
  r.slave_pc_active = static_cast<double>(pc_s_count) / n;
  r.min_passivity_margin = min_margin;
  r.min_true_margin = min_true;
  r.max_book_excess = max_excess;
  r.min_channel_margin = min_channel_margin;
  return r;
}

}  // namespace tuushin::sim
