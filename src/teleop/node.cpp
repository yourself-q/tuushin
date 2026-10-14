#include "tuushin/teleop/node.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <optional>
#include <random>
#include <stdexcept>
#include <utility>

#include "tuushin/wire/packet.hpp"

namespace tuushin::teleop {

// ---------------------------------------------------------------- Ticker

void Ticker::taken(Nanos now) {
  ++ticks_;
  if (lateness_.size() < (std::size_t{1} << 22)) lateness_.push_back(static_cast<double>(now - next_));
  next_ += period_;
  if (now - next_ > kMaxBehind * period_) {
    const Nanos missed = (now - next_) / period_ + 1;
    skipped_ += static_cast<std::uint64_t>(missed);
    next_ += missed * period_;
  }
}

namespace {

constexpr std::size_t kSeriesLimit = std::size_t{1} << 22;

// Waits until `deadline`, handing every datagram that arrives to `on_rx` and
// releasing the link's emulated datagrams on time. Returns the time it
// returned at, which is the sample time of the tick that follows.
template <class OnRx>
Nanos serve_until(net::UdpSocket& sock, HapticLink* link, Nanos deadline,
                  std::vector<std::uint8_t>& buf, OnRx&& on_rx) {
  for (;;) {
    for (;;) {
      net::Endpoint from;
      Nanos rx = 0;
      const auto n = sock.recv_from(buf.data(), buf.size(), &from, &rx);
      if (!n) break;
      if (*n > wire::kMaxDatagram) continue;  // truncated: not ours
      on_rx(buf.data(), *n, from, rx);
    }
    const Nanos now = monotonic_now();
    if (link) link->pump(now);
    if (now >= deadline) return now;
    Nanos wake = deadline;
    if (link) {
      if (const auto r = link->next_release()) wake = std::min(wake, *r);
    }
    if (wake > now) sock.wait_readable(wake - now);
  }
}

// A per-tick CSV file with a large buffer, so that a 1 kHz loop does not
// wait for the disk.
class CsvLog {
 public:
  CsvLog(const std::string& path, const char* header) {
    if (path.empty()) return;
    f_ = std::fopen(path.c_str(), "w");
    if (!f_) throw std::runtime_error("cannot open log: " + path);
    std::setvbuf(f_, nullptr, _IOFBF, 1 << 20);
    std::fputs(header, f_);
  }
  ~CsvLog() {
    if (f_) std::fclose(f_);
  }
  CsvLog(const CsvLog&) = delete;
  CsvLog& operator=(const CsvLog&) = delete;
  std::FILE* get() const { return f_; }

 private:
  std::FILE* f_ = nullptr;
};

std::uint32_t random_session() {
  std::random_device rd;
  std::uint32_t s = 0;
  while (s == 0) s = rd();
  return s;
}

double transmitted_energy(const SlaveController& ctl, Passivity scheme) {
  return scheme == Passivity::kReflective ? ctl.store().granted() : ctl.channel_port().input_energy();
}

}  // namespace

// ---------------------------------------------------------------- master

MasterNodeReport run_master_node(net::UdpSocket& socket, const net::Endpoint& slave,
                                 Device1D& device, const MasterNodeConfig& cfg,
                                 std::unique_ptr<net::ChannelModel> emulation,
                                 const std::atomic<bool>* stop) {
  MasterNodeReport rep;
  rep.session = cfg.session != 0 ? cfg.session : random_session();
  socket.enable_rx_timestamps();
  HapticLink link(socket, rep.session, slave, cfg.control.passivity);
  if (emulation) link.set_emulation(std::move(emulation));
  MasterController ctl(cfg.control);
  CsvLog log(cfg.log_path, "t,late_us,x_m,v_m,f_display,ein,eout,granted,pc,sent,rx,rtt_us\n");

  std::vector<std::pair<Nanos, double>> own;     // (master time, master net) per tick
  std::vector<std::pair<Nanos, double>> remote;  // (slave time, slave net) per sample
  Nanos newest_remote = INT64_MIN;
  Nanos slave_final_at = -1;
  std::vector<std::uint8_t> buf(wire::kMaxDatagram + 1);
  auto on_rx = [&](const std::uint8_t* data, std::size_t n, const net::Endpoint& from, Nanos rx) {
    const auto r = link.on_datagram(data, n, from, rx);
    if (!r) return;
    ctl.receive(r->seq, r->sample);
    rep.slave_answered = true;
    if (remote.size() < kSeriesLimit) remote.emplace_back(r->send_time, r->monitor);
    if (r->send_time > newest_remote) {
      newest_remote = r->send_time;
      rep.slave_net = r->monitor;
    }
    if (r->final) {
      rep.slave_final = true;
      if (slave_final_at < 0) slave_final_at = rx;
    }
  };

  const Nanos start = monotonic_now();
  Ticker ticker(cfg.control.dt, start);
  const Nanos end = start + cfg.duration;
  Nanos ending_since = -1;
  std::uint64_t pc_count = 0;
  for (;;) {
    const Nanos now = serve_until(socket, &link, ticker.due(), buf, on_rx);
    if (ending_since < 0 && (now >= end || (stop && stop->load()))) ending_since = now;
    if (ending_since >= 0) {
      if (slave_final_at >= 0 && now >= slave_final_at + cfg.final_linger) break;
      if (now >= ending_since + cfg.final_timeout) break;
    }
    const Nanos late = now - ticker.due();
    ticker.taken(now);

    const double x = device.read_position(now);
    const auto st = ctl.step(now, x);
    device.command_force(-st.display_force);
    const auto& port = ctl.port();
    const double net = port.input_energy() - port.output_energy();
    const bool final = ending_since >= 0;
    if (st.send || final) {
      const Sample s = st.send ? st.sample : Sample{ctl.velocity(), x, port.input_energy()};
      link.send(now, s, net, final);
    }

    if (own.size() < kSeriesLimit) own.emplace_back(now, net);
    if (st.pc_active) ++pc_count;
    rep.max_position = std::max(rep.max_position, std::fabs(x));
    rep.max_force = std::max(rep.max_force, std::fabs(st.display_force));
    if (log.get()) {
      const auto& rtts = link.rtts();
      std::fprintf(log.get(), "%.6f,%.1f,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%d,%d,%llu,%.1f\n",
                   to_seconds(now - start), static_cast<double>(late) / 1e3, x, device.velocity(),
                   st.display_force, port.input_energy(), port.output_energy(),
                   port.remote_input_energy(), st.pc_active ? 1 : 0, st.send || final ? 1 : 0,
                   static_cast<unsigned long long>(link.counters().received),
                   rtts.empty() ? -1.0 : rtts.back() / 1e3);
    }
  }
  device.command_force(0.0);

  rep.ticks = ticker.ticks();
  rep.skipped = ticker.skipped();
  rep.lateness = net::delay_stats(ticker.lateness());
  rep.link = link.counters();
  rep.master_pc_active =
      rep.ticks ? static_cast<double>(pc_count) / static_cast<double>(rep.ticks) : 0.0;
  rep.master_net = own.empty() ? 0.0 : own.back().second;
  rep.rtt = net::delay_stats(link.rtts());

  // One-way delays from the exchanges and the clock fit.
  auto ex = link.exchanges();
  std::sort(ex.begin(), ex.end(), [](const auto& a, const auto& b) { return a.t1 < b.t1; });
  rep.clock = net::fit_clock(ex);
  std::vector<double> fwd, bwd;
  std::vector<std::pair<Nanos, double>> fwd_arr, bwd_arr;
  for (const auto& e : ex) {
    const double off = rep.clock.offset_at(e.t1);
    fwd.push_back(static_cast<double>(e.t2 - e.t1) - off);
    bwd.push_back(static_cast<double>(e.t4 - e.t3) + off);
    fwd_arr.emplace_back(e.t2, fwd.back());
    bwd_arr.emplace_back(e.t4, bwd.back());
  }
  rep.forward = net::delay_stats(std::move(fwd), std::move(fwd_arr));
  rep.backward = net::delay_stats(std::move(bwd), std::move(bwd_arr));

  // Passivity margin over time: the slave's books, placed on the master's
  // clock, added to the master's at each tick.
  for (auto& r : remote) {
    const Nanos approx = r.first - static_cast<Nanos>(rep.clock.offset);
    r.first -= static_cast<Nanos>(rep.clock.offset_at(approx));
  }
  std::sort(remote.begin(), remote.end());
  double slave_net = 0.0;
  std::size_t j = 0;
  for (const auto& o : own) {
    while (j < remote.size() && remote[j].first <= o.first) slave_net = remote[j++].second;
    rep.min_margin = std::min(rep.min_margin, o.second + slave_net);
  }
  return rep;
}

// ---------------------------------------------------------------- slave

std::vector<SlaveSessionReport> run_slave_node(net::UdpSocket& socket, Device1D& device,
                                               const SlaveNodeConfig& cfg,
                                               const SlaveNodeHooks& hooks,
                                               const std::atomic<bool>* stop) {
  socket.enable_rx_timestamps();
  std::vector<SlaveSessionReport> reports;
  std::vector<std::uint8_t> buf(wire::kMaxDatagram + 1);
  auto stopped = [&] { return stop && stop->load(); };
  std::uint32_t last_refused = 0;

  while (!stopped()) {
    // Idle: wait for the first datagram of a session, keeping a simulated
    // device's physics current with the actuator off.
    device.command_force(0.0);
    struct First {
      wire::Header h;
      net::Endpoint from;
      std::vector<std::uint8_t> bytes;
      Nanos rx;
      Passivity scheme;
    };
    std::optional<First> first;
    while (!stopped() && !first) {
      socket.wait_readable(100 * kNanosPerMilli);
      device.read_position(monotonic_now());
      for (;;) {
        net::Endpoint from;
        Nanos rx = 0;
        const auto n = socket.recv_from(buf.data(), buf.size(), &from, &rx);
        if (!n) break;
        if (*n > wire::kMaxDatagram) continue;
        const auto h = wire::decode_header(buf.data(), *n);
        if (!h || h->type != wire::PacketType::kHaptic || (h->flags & wire::kFlagFinal) ||
            !wire::decode_haptic(buf.data(), *n)) {
          continue;
        }
        const unsigned code = (h->flags & wire::kSchemeMask) >> wire::kSchemeShift;
        if (code > static_cast<unsigned>(Passivity::kReflective)) continue;
        const auto scheme = static_cast<Passivity>(code);
        if (scheme != cfg.control.passivity && !cfg.adopt_scheme) {
          if (hooks.on_refused && h->session != last_refused) {
            hooks.on_refused("session from " + from.to_string() +
                             " runs another passivity scheme; refused");
          }
          last_refused = h->session;
          continue;
        }
        first = First{*h, from, std::vector<std::uint8_t>(buf.begin(), buf.begin() + *n), rx, scheme};
        break;  // the rest stays queued for the session
      }
    }
    if (!first) break;

    SlaveSessionReport rep;
    rep.session = first->h.session;
    rep.master = first->from;
    rep.scheme = first->scheme;
    SlaveConfig sc = cfg.control;
    sc.passivity = first->scheme;
    HapticLink link(socket, rep.session, first->from, sc.passivity);
    const Nanos start = monotonic_now();
    if (hooks.emulation) link.set_emulation(hooks.emulation(start));
    SlaveController ctl(sc, device.read_position(start));
    std::string log_path;
    if (!cfg.log_prefix.empty()) {
      char id[16];
      std::snprintf(id, sizeof id, "%08x", rep.session);
      log_path = cfg.log_prefix + id + ".csv";
    }
    CsvLog log(log_path,
               "t,late_us,x_s,v_s,f_contact,f_cmd,x_cmd,x_master_est,f_slave,store,ein_s,eout_s,"
               "net_s,pc,sent,rx\n");

    Nanos last_rx = first->rx;
    Nanos master_final_at = -1;
    auto on_rx = [&](const std::uint8_t* data, std::size_t n, const net::Endpoint& from, Nanos rx) {
      const auto r = link.on_datagram(data, n, from, rx);
      if (!r) return;
      ctl.receive(r->seq, r->sample);
      last_rx = std::max(last_rx, rx);
      if (r->final && master_final_at < 0) master_final_at = rx;
    };
    on_rx(first->bytes.data(), first->bytes.size(), first->from, first->rx);

    Ticker ticker(sc.dt, start);
    Nanos stop_since = -1;
    std::uint64_t pc_count = 0;
    for (;;) {
      const Nanos now = serve_until(socket, &link, ticker.due(), buf, on_rx);
      if (now - last_rx > cfg.idle_timeout) break;
      if (master_final_at >= 0 && now >= master_final_at + cfg.final_linger) {
        rep.ended_by_final = true;
        break;
      }
      if (stop_since < 0 && stopped()) stop_since = now;
      if (stop_since >= 0 && now >= stop_since + cfg.final_linger) break;
      const Nanos late = now - ticker.due();
      ticker.taken(now);

      const double x = device.read_position(now);
      const double fc = device.contact_force();
      const auto st = ctl.step(now, x, device.velocity(), fc);
      device.command_force(st.force);
      const bool final = master_final_at >= 0 || stop_since >= 0;
      if (st.send || final) {
        Sample s = st.send ? st.sample
                           : Sample{ctl.reported_force(), 0.0, transmitted_energy(ctl, sc.passivity)};
        s.aux = x;  // for monitoring at the master
        link.send(now, s, ctl.terminal_net(), final);
      }

      if (st.pc_active) ++pc_count;
      rep.max_position = std::max(rep.max_position, std::fabs(x));
      rep.max_force = std::max(rep.max_force, std::fabs(st.force));
      if (log.get()) {
        const bool refl = sc.passivity == Passivity::kReflective;
        const auto& stv = ctl.store();
        const auto& sp = ctl.channel_port();
        std::fprintf(log.get(),
                     "%.6f,%.1f,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%d,%d,%llu\n",
                     to_seconds(now - start), static_cast<double>(late) / 1e3, x, device.velocity(),
                     fc, st.force, ctl.command(), ctl.master_estimate(), ctl.reported_force(),
                     refl ? stv.level() : 0.0, refl ? stv.slave_input_energy() : sp.input_energy(),
                     refl ? stv.slave_output_energy() : sp.output_energy(), ctl.terminal_net(),
                     st.pc_active ? 1 : 0, st.send || final ? 1 : 0,
                     static_cast<unsigned long long>(link.counters().received));
      }
    }
    device.command_force(0.0);

    rep.ticks = ticker.ticks();
    rep.skipped = ticker.skipped();
    rep.lateness = net::delay_stats(ticker.lateness());
    rep.link = link.counters();
    rep.slave_net = ctl.terminal_net();
    rep.slave_pc_active =
        rep.ticks ? static_cast<double>(pc_count) / static_cast<double>(rep.ticks) : 0.0;
    if (hooks.on_session_end) hooks.on_session_end(rep);
    reports.push_back(rep);
    if (cfg.once) break;
  }
  device.command_force(0.0);
  return reports;
}

}  // namespace tuushin::teleop
