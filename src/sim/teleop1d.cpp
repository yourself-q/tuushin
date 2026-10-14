#include "tuushin/sim/teleop1d.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <queue>
#include <stdexcept>
#include <vector>

#include "tuushin/control/predictive_pc.hpp"
#include "tuushin/control/reflection.hpp"
#include "tuushin/control/tdpa.hpp"

namespace tuushin::sim {

namespace {

constexpr double kPi = 3.14159265358979323846;

// A haptic sample in flight: the power variable, an auxiliary value (the
// master position on the forward path, unused on the backward path) and the
// sender's cumulative energy (its input energy, or for kReflective on the
// backward path the energy granted to the master).
struct InFlight {
  Nanos arrival;
  std::uint32_t seq;
  double value;
  double aux;
  double energy;
};

struct LaterArrival {
  bool operator()(const InFlight& a, const InFlight& b) const {
    if (a.arrival != b.arrival) return a.arrival > b.arrival;
    return a.seq > b.seq;
  }
};

using InFlightQueue = std::priority_queue<InFlight, std::vector<InFlight>, LaterArrival>;

// Receiver side of one direction: holds the newest sample (by sequence
// number) and the largest reported energy. Energies are cumulative, so the
// largest one is the newest information even when packets are reordered.
struct Receiver {
  bool has_value = false;
  bool fresh = false;  // a newer sample arrived this step
  std::uint32_t last_seq = 0;
  double value = 0.0;
  double aux = 0.0;
  double energy = 0.0;

  void deliver(const InFlight& p) {
    energy = std::max(energy, p.energy);
    if (!has_value || p.seq > last_seq) {
      has_value = true;
      fresh = true;
      last_seq = p.seq;
      value = p.value;
      aux = p.aux;
    }
  }
};

struct Sender {
  std::uint32_t next_seq = 1;
  std::uint64_t sent = 0;
  std::uint64_t lost = 0;

  void send(net::ChannelModel& ch, InFlightQueue& q, Nanos now, std::size_t bytes, double value,
            double aux, double energy) {
    const auto transit = ch.transit(now, bytes);
    ++sent;
    if (transit) {
      q.push(InFlight{now + *transit, next_seq, value, aux, energy});
    } else {
      ++lost;
    }
    ++next_seq;
  }
};

// Positions and velocities of both devices, or their time derivatives.
struct PlantState {
  double x_m, v_m, x_s, v_s;
  PlantState plus(const PlantState& d, double s) const {
    return {x_m + s * d.x_m, v_m + s * d.v_m, x_s + s * d.x_s, v_s + s * d.v_s};
  }
};

bool plausible(double x, double v) {
  return std::isfinite(x) && std::isfinite(v) && std::fabs(x) < 1.0 && std::fabs(v) < 10.0;
}

}  // namespace

Teleop1DResult run_teleop1d(const Teleop1DConfig& cfg, net::ChannelModel& forward,
                            net::ChannelModel& backward) {
  if (cfg.dt <= 0 || cfg.substeps <= 0 || cfg.duration <= 0) {
    throw std::invalid_argument("run_teleop1d: dt, substeps and duration must be positive");
  }
  using Passivity = Teleop1DConfig::Passivity;
  const bool pcs = cfg.passivity != Passivity::kNone;
  const bool reflective = cfg.passivity == Passivity::kReflective;
  const double dt = to_seconds(cfg.dt);
  const double h = dt / cfg.substeps;
  const std::int64_t steps = cfg.duration / cfg.dt;

  std::unique_ptr<std::FILE, int (*)(std::FILE*)> log(nullptr, &std::fclose);
  if (!cfg.log_path.empty()) {
    log.reset(std::fopen(cfg.log_path.c_str(), "w"));
    if (!log) throw std::runtime_error("cannot open log: " + cfg.log_path);
    std::fprintf(log.get(),
                 "t,x_ref,x_m,v_m,f_hand,f_display,x_s,v_s,x_cmd,x_master_est,v_req,v_c,f_slave,"
                 "f_contact,ein_m,eout_m,ein_s,eout_s,store,grant_m,pc_m,pc_s,sent_fwd,sent_bwd\n");
  }

  // Plant state.
  double x_m = cfg.op.ref_offset, v_m = 0.0;
  double x_m_sampled = x_m;  // encoder reading at the previous control step
  double x_s = cfg.op.ref_offset, v_s = 0.0;
  double x_s_sampled = x_s;
  double x_cmd = x_s;
  // The slave's estimate of the delayed master position: reset to the
  // position carried by each new packet, extrapolated with the held velocity
  // in between (deadband gaps, losses).
  double x_master_est = x_m;
  double v_c = 0.0;         // velocity command applied at the slave
  double f_display = 0.0;   // force applied by the master device
  double f_ctrl = 0.0;      // slave coupling force applied this step

  control::EnergyPort master_port;
  control::EnergyPort slave_port;       // kNone, kChannel: channel end at the slave
  control::ReflectiveStore store;       // kReflective
  control::DevicePredictor master_dev(
      cfg.master_model_mass > 0.0 ? cfg.master_model_mass : cfg.master.mass, dt);
  control::DevicePredictor slave_dev(
      cfg.slave_model_mass > 0.0 ? cfg.slave_model_mass : cfg.slave.mass, dt);
  double slave_terminal_net = 0.0;      // true books at the slave device port
  Receiver at_slave;   // receives master velocity
  Receiver at_master;  // receives slave force
  Sender fwd_sender, bwd_sender;
  InFlightQueue fwd_queue, bwd_queue;
  codec::DeadbandEncoder<1> fwd_enc(cfg.forward_deadband_params);
  codec::DeadbandEncoder<1> bwd_enc(cfg.backward_deadband_params);

  Teleop1DResult r;
  double sum_pos_err2 = 0.0, sum_force_err2 = 0.0;
  double sum_kp_num = 0.0, sum_kp_den = 0.0;
  double drift_sum = 0.0;
  std::int64_t drift_n = 0;
  std::int64_t pc_m_count = 0, pc_s_count = 0;
  double min_margin = 0.0, min_channel_margin = 0.0;
  std::int64_t done = 0;

  for (std::int64_t k = 0; k < steps; ++k) {
    const Nanos now = k * cfg.dt;
    const double t = to_seconds(now);

    // 1. Deliver everything that has arrived by now.
    at_slave.fresh = at_master.fresh = false;
    while (!fwd_queue.empty() && fwd_queue.top().arrival <= now) {
      at_slave.deliver(fwd_queue.top());
      fwd_queue.pop();
    }
    while (!bwd_queue.empty() && bwd_queue.top().arrival <= now) {
      at_master.deliver(bwd_queue.top());
      bwd_queue.pop();
    }
    master_port.on_remote_input_energy(at_master.energy);
    if (reflective) {
      store.on_master_input_energy(at_slave.energy);
    } else {
      slave_port.on_remote_input_energy(at_slave.energy);
    }

    // 2. Master: sample the encoder, book the work the force held over the
    //    last sample actually did, then display the reflected force through
    //    the master PC. The velocity is the backward difference of the
    //    encoder position, as on a real device.
    const double dx_m = x_m - x_m_sampled;
    const double v_meas = k == 0 ? 0.0 : dx_m / dt;
    master_port.account_energy(f_display * dx_m);
    master_dev.observe(dx_m, f_display);
    x_m_sampled = x_m;
    const double f_reflected = at_master.has_value ? at_master.value : 0.0;
    bool pc_m = false;
    if (pcs) {
      const auto pc =
          cfg.predictive_pc
              ? control::limit_force_predictive(master_port.budget(), f_reflected, master_dev,
                                                cfg.pc_hold_samples)
              : control::limit_force(master_port, f_reflected, v_meas, dt);
      f_display = pc.value;
      pc_m = pc.active;
    } else {
      f_display = f_reflected;
    }
    // Actuator saturation keeps the sign, so it can only reduce the energy
    // leaving the channel.
    if (cfg.master.max_force > 0.0) {
      f_display = std::clamp(f_display, -cfg.master.max_force, cfg.master.max_force);
    }
    bool sent_fwd = true;
    if (cfg.forward_deadband) sent_fwd = fwd_enc.update(Vec1{{v_meas}}, now);
    if (sent_fwd) {
      fwd_sender.send(forward, fwd_queue, now, cfg.packet_bytes, v_meas, x_m,
                      master_port.input_energy());
    }

    // 3. Slave. Book the work the coupling force held over the last sample
    //    did on the slave device; this is the true slave terminal for every
    //    scheme, and the energy source/sink of the reflective store.
    const double dx_s = x_s - x_s_sampled;
    const double slave_work_in = -f_ctrl * dx_s;
    slave_terminal_net += slave_work_in;
    slave_dev.observe(dx_s, -f_ctrl);
    x_s_sampled = x_s;
    const double penetration_s = x_s - cfg.wall.position;
    double f_contact = 0.0;
    if (penetration_s > 0.0) {
      f_contact = std::max(0.0, cfg.wall.k * penetration_s + cfg.wall.b * v_s);
    }
    const double v_received = at_slave.has_value ? at_slave.value : 0.0;
    if (at_slave.fresh) x_master_est = at_slave.aux;
    double v_requested = v_received;
    double f_slave = 0.0;  // force reported to the master
    bool pc_s = false;
    if (reflective) {
      // The coupling follows the delayed master position itself. The PC
      // scales the coupling force on the slave device only, so the command is
      // never altered and cannot drift.
      store.account_slave_energy(slave_work_in);
      x_cmd = x_master_est;
      v_c = v_received;
      const double f_coupling = cfg.slave.kp * (x_cmd - x_s) + cfg.slave.kd * (v_c - v_s);
      f_slave = cfg.slave.report_contact_force ? f_contact : f_coupling;
      // Predicted outputs of the two-port over the coming sample: the work
      // the coupling would do on the slave, and the energy the reflected
      // force would return to the master, evaluated with the delayed master
      // velocity at the channel end (the paper's P_3^{R2L}).
      // The PCs' sign convention is power *into* the two-port, which at the
      // slave is the slave pushing against the coupling: phi = -f.
      const double phi = -f_coupling;
      const double want_slave =
          cfg.predictive_pc ? -slave_dev.work(phi) : f_coupling * v_s * dt;
      const double want_master = -f_slave * v_c * dt;
      const auto grant = store.allocate(want_slave, want_master);
      const auto pc = cfg.predictive_pc
                          ? control::limit_force_predictive(grant.slave, phi, slave_dev,
                                                            cfg.pc_hold_samples)
                          : control::limit_force(grant.slave, phi, v_s, dt);
      f_ctrl = -pc.value;
      pc_s = pc.active || grant.limited;
    } else {
      // The coupling force for this step follows from the command up to the
      // previous step, so the force the PC books is exactly the force applied
      // during the step (Ryu & Preusche, ICRA 2007, take f_s(n) as a function
      // of v_sd(n-1) for the same reason). Recomputing the force from the new
      // command would add kd * (v_c(n) - v_c(n-1)) to the applied force behind
      // the PC's back, and large command jumps (drift compensation) then make
      // the loop active although the books balance.
      const double f_coupling = cfg.slave.kp * (x_cmd - x_s) + cfg.slave.kd * (v_c - v_s);
      f_slave = cfg.slave.report_contact_force ? f_contact : f_coupling;
      if (cfg.slave.drift_gain > 0.0 && at_slave.has_value) {
        const double ref =
            cfg.slave.drift_reference == SlaveParams::DriftReference::kSlavePosition ? x_s : x_cmd;
        v_requested += cfg.slave.drift_gain * (x_master_est - ref);
      }
      if (pcs) {
        const auto pc = control::passivate_velocity(slave_port, v_requested, f_slave, dt);
        v_c = pc.value;
        pc_s = pc.active;
      } else {
        v_c = v_requested;
        slave_port.account(-f_slave * v_c, dt);
      }
      f_ctrl = f_coupling;
      x_cmd += v_c * dt;
    }
    x_master_est += v_received * dt;

    bool sent_bwd = true;
    if (cfg.backward_deadband) sent_bwd = bwd_enc.update(Vec1{{f_slave}}, now);
    if (sent_bwd) {
      bwd_sender.send(backward, bwd_queue, now, cfg.packet_bytes, f_slave, 0.0,
                      reflective ? store.granted() : slave_port.input_energy());
    }

    // Metrics on the samples of this step.
    const double terminal_net =
        master_port.input_energy() - master_port.output_energy() + slave_terminal_net;
    min_margin = std::min(min_margin, terminal_net);
    if (!reflective) {
      const double margin_fwd = master_port.input_energy() - slave_port.output_energy();
      const double margin_bwd = slave_port.input_energy() - master_port.output_energy();
      min_channel_margin = std::min({min_channel_margin, margin_fwd, margin_bwd});
    }
    if (pc_m) ++pc_m_count;
    if (pc_s) ++pc_s_count;

    const double x_ref = cfg.op.ref_offset +
                         cfg.op.ref_amplitude * 0.5 * (1.0 - std::cos(2.0 * kPi * cfg.op.ref_freq * t));
    const double v_ref = cfg.op.ref_amplitude * 0.5 * 2.0 * kPi * cfg.op.ref_freq *
                         std::sin(2.0 * kPi * cfg.op.ref_freq * t);
    const double f_hand = cfg.op.hand_k * (x_ref - x_m) + cfg.op.hand_b * (v_ref - v_m);

    if (log) {
      std::fprintf(log.get(),
                   "%.4f,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,"
                   "%.6g,%.6g,%.6g,%.6g,%.6g,%d,%d,%d,%d\n",
                   t, x_ref, x_m, v_m, f_hand, f_display, x_s, v_s, x_cmd, x_master_est,
                   v_requested, v_c, f_slave, f_contact,
                   master_port.input_energy(), master_port.output_energy(),
                   reflective ? store.slave_input_energy() : slave_port.input_energy(),
                   reflective ? store.slave_output_energy() : slave_port.output_energy(),
                   reflective ? store.level() : terminal_net, master_port.remote_input_energy(),
                   pc_m ? 1 : 0,
                   pc_s ? 1 : 0, sent_fwd ? 1 : 0, sent_bwd ? 1 : 0);
    }

    sum_pos_err2 += (x_m - x_s) * (x_m - x_s);
    sum_force_err2 += (f_display - f_contact) * (f_display - f_contact);
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

    // 4. Physics over [now, now + dt). Controller outputs are held; the hand
    //    and the wall are physical and evaluated inside the integrator.
    //    Classical Runge-Kutta per substep: it is exact for a constant force
    //    on a mass, as a real device under zero-order hold behaves, whereas
    //    semi-implicit Euler with n substeps overstates the displacement a
    //    held force causes within the sample by a factor (n + 1) / n, an
    //    artefact the device-model PC would otherwise be tuned against.
    //    Its truncation error at 10 kHz is far below the energies measured.
    auto deriv = [&](double ti, const PlantState& p) {
      const double w = 2.0 * kPi * cfg.op.ref_freq;
      const double xr = cfg.op.ref_offset + cfg.op.ref_amplitude * 0.5 * (1.0 - std::cos(w * ti));
      const double vr = cfg.op.ref_amplitude * 0.5 * w * std::sin(w * ti);
      const double fh = cfg.op.hand_k * (xr - p.x_m) + cfg.op.hand_b * (vr - p.v_m);
      const double pen = p.x_s - cfg.wall.position;
      const double fe = pen > 0.0 ? std::max(0.0, cfg.wall.k * pen + cfg.wall.b * p.v_s) : 0.0;
      return PlantState{p.v_m, (fh - f_display - cfg.master.damping * p.v_m) / cfg.master.mass,
                        p.v_s, (f_ctrl - fe - cfg.slave.damping * p.v_s) / cfg.slave.mass};
    };
    PlantState p{x_m, v_m, x_s, v_s};
    for (int i = 0; i < cfg.substeps; ++i) {
      const double ti = t + i * h;
      const PlantState k1 = deriv(ti, p);
      const PlantState k2 = deriv(ti + 0.5 * h, p.plus(k1, 0.5 * h));
      const PlantState k3 = deriv(ti + 0.5 * h, p.plus(k2, 0.5 * h));
      const PlantState k4 = deriv(ti + h, p.plus(k3, h));
      p = p.plus(k1, h / 6.0).plus(k2, h / 3.0).plus(k3, h / 3.0).plus(k4, h / 6.0);
    }
    x_m = p.x_m;
    v_m = p.v_m;
    x_s = p.x_s;
    v_s = p.v_s;

    if (!plausible(x_m, v_m) || !plausible(x_s, v_s)) {
      r.diverged = true;
      r.diverged_at = to_seconds(now + cfg.dt);
      break;
    }
  }

  const double n = static_cast<double>(std::max<std::int64_t>(done, 1));
  const double sim_s = static_cast<double>(done) * dt;
  r.pos_rmse = std::sqrt(sum_pos_err2 / n);
  r.force_rmse = std::sqrt(sum_force_err2 / n);
  r.displayed_stiffness = sum_kp_den > 0.0 ? sum_kp_num / sum_kp_den : 0.0;
  r.final_drift = drift_n > 0 ? drift_sum / static_cast<double>(drift_n) : 0.0;
  r.forward_packet_rate = sim_s > 0 ? static_cast<double>(fwd_sender.sent) / sim_s : 0.0;
  r.backward_packet_rate = sim_s > 0 ? static_cast<double>(bwd_sender.sent) / sim_s : 0.0;
  r.forward_loss = fwd_sender.sent ? static_cast<double>(fwd_sender.lost) / fwd_sender.sent : 0.0;
  r.backward_loss = bwd_sender.sent ? static_cast<double>(bwd_sender.lost) / bwd_sender.sent : 0.0;
  r.master_pc_active = static_cast<double>(pc_m_count) / n;
  r.slave_pc_active = static_cast<double>(pc_s_count) / n;
  r.min_passivity_margin = min_margin;
  r.min_channel_margin = min_channel_margin;
  return r;
}

}  // namespace tuushin::sim
