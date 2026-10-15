#include "tuushin/teleop/node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
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
  if (next_ <= now) {
    const Nanos missed = (now - next_) / period_ + 1;
    skipped_ += static_cast<std::uint64_t>(missed);
    next_ += missed * period_;
  }
}

namespace {

constexpr std::size_t kSeriesLimit = std::size_t{1} << 22;

// Waits until `deadline`, handing every datagram that arrives either way to
// `on_rx`, releasing the link's emulated datagrams on time and keeping the
// relay, if any, served. Returns the time it returned at, which is the
// sample time of the tick that follows.
template <class OnRx>
Nanos serve_until(net::Paths& paths, HapticLink* link, Nanos deadline,
                  std::vector<std::uint8_t>& buf, OnRx&& on_rx) {
  for (;;) {
    for (;;) {
      net::Route from;
      Nanos rx = 0;
      const auto n = paths.recv(buf.data(), buf.size(), &from, &rx);
      if (!n) break;
      if (*n > wire::kMaxDatagram) continue;  // truncated: not ours
      on_rx(buf.data(), *n, from, rx);
    }
    const Nanos now = monotonic_now();
    if (link) link->pump(now);
    paths.service(now);
    if (now >= deadline) return now;
    Nanos wake = std::min(deadline, paths.next_due());
    if (link) {
      if (const auto r = link->next_release()) wake = std::min(wake, *r);
    }
    if (wake > now) paths.wait_readable(wake - now);
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
  return scheme == Passivity::kReflective ? ctl.store().granted()
                                          : ctl.channel_port().reported_input_energy();
}

std::uint16_t scheme_flags(Passivity scheme) {
  return static_cast<std::uint16_t>((static_cast<unsigned>(scheme) << wire::kSchemeShift) &
                                    wire::kSchemeMask);
}

std::optional<Passivity> scheme_of(const wire::Header& h) {
  const unsigned code = (h.flags & wire::kSchemeMask) >> wire::kSchemeShift;
  if (code > static_cast<unsigned>(Passivity::kReflective)) return std::nullopt;
  return static_cast<Passivity>(code);
}

// The empty payload of the hello reply.
const std::uint8_t kNoPayload = 0;

std::uint64_t wall_clock_ns() {
  return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                        std::chrono::system_clock::now().time_since_epoch())
                                        .count());
}

// A datagram that only opens the sender's NAT.
std::vector<std::uint8_t> punch_datagram() {
  wire::Header h;
  h.type = wire::PacketType::kPunch;
  std::vector<std::uint8_t> out;
  wire::encode_raw(h, nullptr, 0, out);
  return out;
}

// Punches carry nothing; during a session they are expected (the other
// end's last ones before its session started, or a rejoin's), so they are
// not counted as the link's foreign datagrams.
bool is_punch(const std::uint8_t* data, std::size_t n) {
  const auto h = wire::decode_header(data, n);
  return h && h->type == wire::PacketType::kPunch;
}

// Opens this end's NAT towards `to` with punches that expire on the way
// (RendezvousConfig::prime_hops). False if none could be sent so, which is
// what a system that cannot limit the hop count gives.
bool prime(net::UdpSocket& socket, const std::vector<net::Endpoint>& to, int hops,
           const std::vector<std::uint8_t>& punch) {
  bool any = false;
  for (const auto& t : to) any = socket.send_limited(t, punch.data(), punch.size(), hops) || any;
  return any;
}

// Keeps this end's relayed address announced through the rendezvous: the
// new one after every allocation, none while there is none. When the
// rendezvous server sees this end at a new address, its NAT has likely
// moved the relay's socket too, which the TURN server then no longer
// knows; the TURN client is told to find out.
class RelayAnnouncer {
 public:
  // Returns a line for the log if anything changed.
  std::optional<std::string> update(net::Paths& paths, net::RendezvousClient& rv, Nanos now) {
    net::TurnClient* turn = paths.turn();
    if (!turn) return std::nullopt;
    if (rv.observed() != observed_) {
      if (observed_.valid()) turn->check(now);
      observed_ = rv.observed();
    }
    if (turn->allocated() && turn->allocations() != allocations_) {
      // A loss and the new allocation can come between two calls.
      const bool untold = allocations_ != 0 && !lost_told_;
      allocations_ = turn->allocations();
      lost_told_ = false;
      rv.set_relays({turn->relayed()});
      return std::string(untold ? "relay lost (" + turn->last_error() + "), and " : "") +
             "relay allocated at " + turn->relayed().to_string() + " on " +
             turn->server().to_string() + ", which sees this end at " +
             turn->mapped().to_string();
    }
    if (!turn->allocated() && !rv.relays().empty()) {
      rv.set_relays({});
      lost_told_ = true;
      return "relay lost (" + turn->last_error() + "); allocating again";
    }
    if (turn->failed() && !failed_) {
      failed_ = true;
      return "relay refused: " + turn->last_error();
    }
    return std::nullopt;
  }

 private:
  std::uint64_t allocations_ = 0;
  bool lost_told_ = false;
  bool failed_ = false;
  net::Endpoint observed_;
};

// The rendezvous during a session (RendezvousConfig::rejoin_after): looks
// for an end that has moved when the other end falls silent, and punches
// towards the other end's new address when the server announces one. The
// punches are copies of the link's last datagram (HapticLink::send_copy):
// they open this end's NAT as a punch does, and they also authenticate,
// so the other end's link follows this end to the port its NAT sent them
// from. A NAT that gives every destination its own port (or the other
// end's relay, which sees this end at such a port) has the session found
// again only so: the other end would otherwise keep sending to the port
// it knew, which now lets nothing in.
class SessionRejoin {
 public:
  SessionRejoin(net::RendezvousClient& rv, const net::Endpoint& server,
                const RendezvousConfig& cfg, net::Paths& paths)
      : rv_(rv), server_(server), cfg_(cfg), paths_(paths), socket_(paths.direct()),
        changes_(rv.peer_changes()), punch_(punch_datagram()) {}

  // True for the server's datagrams, which are taken here.
  bool on_datagram(const std::uint8_t* data, std::size_t n, const net::Route& from,
                   Nanos now) {
    if (from.relayed || !rv_.on_datagram(data, n, from.peer, now)) return false;
    if (rv_.peer_changes() != changes_) {
      changes_ = rv_.peer_changes();
      ++rejoins_;
      // The relayed addresses too: a relay that allocated anew sends from
      // a new address, which this end's NAT lets in only after a punch.
      to_ = net::sendable(rv_.peer(), socket_);
      for (const auto& r : net::sendable(rv_.peer_relays(), socket_)) {
        if (std::find(to_.begin(), to_.end(), r) == to_.end()) to_.push_back(r);
      }
      paths_.permit(rv_.peer());
      punch_until_ = now + cfg_.rejoin_punch;
      next_punch_ = now;
      // The moved end is sending already, so there is nothing to order:
      // this end is ready as soon as it punches.
      rv_.set_ready(true);
    }
    return true;
  }

  // Every tick, before the link sends; `last_rx` is when the last authentic
  // sample arrived.
  void tick(Nanos now, Nanos last_rx, HapticLink& link) {
    const bool silent = now - last_rx > cfg_.rejoin_after;
    if (silent) {
      if (now >= next_rejoin_) {
        rv_.refresh_now();
        // A NAT that moved this end's socket moved the relay's too.
        if (net::TurnClient* turn = paths_.turn()) turn->check(now);
        next_rejoin_ = now + net::RendezvousClient::kRetryInterval;
      }
    } else {
      next_rejoin_ = 0;
    }
    if (const auto reg = rv_.poll(now)) socket_.send_to(server_, reg->data(), reg->size());
    // Also while the silence lasts: the server announces a move only once,
    // and a punch towards it that was lost would otherwise not be repeated.
    if ((now < punch_until_ || silent) && now >= next_punch_) {
      for (const auto& t : to_) {
        if (!link.send_copy(net::Route{t, false})) socket_.send_to(t, punch_.data(), punch_.size());
      }
      next_punch_ = now + cfg_.punch_interval;
    }
  }

  std::uint64_t rejoins() const { return rejoins_; }

 private:
  net::RendezvousClient& rv_;
  net::Endpoint server_;
  const RendezvousConfig& cfg_;
  net::Paths& paths_;
  net::UdpSocket& socket_;
  std::uint64_t changes_;
  std::uint64_t rejoins_ = 0;
  std::vector<net::Endpoint> to_;
  Nanos punch_until_ = -1, next_punch_ = 0, next_rejoin_ = 0;
  std::vector<std::uint8_t> punch_;
};

// The master's side of the handshake: resends its hello every
// hello_interval to every candidate address of the slave until a reply
// authenticates, keeping a simulated device's physics current with the
// actuator off meanwhile. With a rendezvous the candidates come from the
// server, and the reply may come from anywhere: a slave behind a NAT that
// maps each destination to its own port answers from an address the server
// never saw, and only the slave can make a reply that authenticates. The
// candidates get hellos only once the slave is ready (or ready_wait after
// the announcement); the master primes its own NAT before it says it is
// ready, or, where it cannot, says so after its first hellos. The slave's
// relayed addresses get hellos relay_delay after its own candidates; they
// need no priming, since the relay sends to this end only once it has
// heard from it.
struct Handshake {
  crypto::SessionKeys keys;
  net::Endpoint slave;
};

std::optional<Handshake> master_handshake(net::UdpSocket& socket, const net::Endpoint& slave,
                                          std::uint32_t session, Device1D& device,
                                          const MasterNodeConfig& cfg,
                                          const std::atomic<bool>* stop,
                                          std::vector<std::uint8_t>& buf, MasterNodeReport& rep,
                                          std::optional<net::RendezvousClient>& rv) {
  const Passivity scheme = cfg.control.passivity;
  const auto prologue = handshake_prologue(session, scheme);
  crypto::NoiseNNpsk0 hs(crypto::NoiseNNpsk0::Role::kInitiator, *cfg.psk, prologue.data(),
                         prologue.size());
  const auto stamp = hello_payload(wall_clock_ns());
  const auto msg1 = hs.write_message(stamp.data(), stamp.size());
  wire::Header h;
  h.type = wire::PacketType::kHello;
  h.session = session;
  h.flags = scheme_flags(scheme);
  std::vector<std::uint8_t> out;
  device.command_force(0.0);

  std::optional<net::Endpoint> server;
  if (cfg.rendezvous) server = cfg.rendezvous->server.for_family(socket.is_ipv6());
  // A slave address given directly, in the form this socket sends to.
  std::optional<net::Endpoint> direct;
  if (slave.valid()) direct = slave.for_family(socket.is_ipv6());
  // The rendezvous candidates, apart from `direct`, and the relayed ones
  // apart from both.
  std::vector<net::Endpoint> targets, relays;
  std::uint64_t peer_changes = 0;
  Nanos announced = -1;  // when the server last announced the slave
  Nanos targets_since = -1;  // when the candidates got their first hellos
  bool primed = false, slave_ready = false;
  const auto punch = punch_datagram();

  const Nanos start = monotonic_now();
  Nanos known_since = direct ? start : -1;  // when the first candidate was known
  Nanos next_hello = start;
  for (;;) {
    const Nanos now = monotonic_now();
    device.read_position(now);
    if (stop && stop->load()) return std::nullopt;
    if (known_since < 0 && now - start >= cfg.rendezvous->wait) {
      rep.rendezvous_failed = true;
      return std::nullopt;
    }
    if (known_since >= 0 && now - known_since >= cfg.handshake_timeout) return std::nullopt;
    if (rv) {
      if (const auto reg = rv->poll(now); reg && server) {
        socket.send_to(*server, reg->data(), reg->size());
      }
    }
    // The candidates get hellos once the slave's NAT expects them.
    const bool to_targets =
        announced >= 0 && (slave_ready || now - announced >= cfg.rendezvous->ready_wait);
    if (known_since >= 0 && now >= next_hello) {
      auto send_hello = [&](const net::Endpoint& t) {
        ++h.seq;
        h.send_time = monotonic_now();
        wire::encode_raw(h, msg1.data(), msg1.size(), out);
        socket.send_to(t, out.data(), out.size());
      };
      if (direct) send_hello(*direct);
      if (to_targets) {
        for (const auto& t : targets) send_hello(t);
        // Unprimed, this end's NAT expects the slave from now on.
        if (!primed) rv->set_ready(true);
        if (targets_since < 0) targets_since = now;
        if (now - targets_since >= cfg.rendezvous->relay_delay) {
          for (const auto& r : relays) send_hello(r);
        }
      }
      next_hello = now + cfg.hello_interval;
    }
    Nanos wake = now + 10 * kNanosPerMilli;
    if (known_since >= 0) wake = std::min(wake, next_hello);
    if (rv) wake = std::min(wake, rv->next_due());
    if (wake > now) socket.wait_readable(wake - now);
    for (;;) {
      net::Endpoint from;
      const auto n = socket.recv_from(buf.data(), buf.size(), &from);
      if (!n) break;
      if (*n > wire::kMaxDatagram) continue;
      if (rv && rv->on_datagram(buf.data(), *n, from, monotonic_now())) {
        const Nanos t = monotonic_now();
        rep.observed = rv->observed();
        if (rv->peer_changes() != peer_changes) {
          peer_changes = rv->peer_changes();
          rep.candidates = rv->peer();
          rep.relays = rv->peer_relays();
          targets = net::sendable(rv->peer(), socket);
          if (direct) targets.erase(std::remove(targets.begin(), targets.end(), *direct),
                                    targets.end());
          relays.clear();
          for (const auto& r : net::sendable(rv->peer_relays(), socket)) {
            if (r != direct && std::find(targets.begin(), targets.end(), r) == targets.end()) {
              relays.push_back(r);
            }
          }
          announced = t;
          primed = prime(socket, targets, cfg.rendezvous->prime_hops, punch);
          if (primed) rv->set_ready(true);
          if (known_since < 0) {
            known_since = t;
            next_hello = t + cfg.rendezvous->ready_wait;
          }
        }
        // The first hellos to the candidates go as soon as the slave is ready.
        if (rv->peer_ready() && !slave_ready) next_hello = t;
        slave_ready = rv->peer_ready();
        continue;
      }
      const auto rh = wire::decode_header(buf.data(), *n);
      if (!rh || rh->type != wire::PacketType::kHelloReply || rh->session != session ||
          scheme_of(*rh) != scheme) {
        continue;
      }
      if (hs.read_message(buf.data() + wire::kHeaderSize, rh->payload_len)) {
        rep.relayed = std::find(relays.begin(), relays.end(), from) != relays.end();
        return Handshake{hs.split(), from};
      }
    }
  }
}

}  // namespace

std::vector<std::uint8_t> handshake_prologue(std::uint32_t session, Passivity scheme) {
  static const char kLabel[] = "tuushin haptic session v1";
  std::vector<std::uint8_t> p(kLabel, kLabel + sizeof kLabel - 1);
  for (int i = 0; i < 4; ++i) p.push_back(static_cast<std::uint8_t>(session >> (8 * i)));
  p.push_back(static_cast<std::uint8_t>(scheme));
  return p;
}

std::vector<std::uint8_t> hello_payload(std::uint64_t wall_ns) {
  std::vector<std::uint8_t> p(kHelloPayloadSize);
  for (std::size_t i = 0; i < kHelloPayloadSize; ++i) {
    p[i] = static_cast<std::uint8_t>(wall_ns >> (8 * i));
  }
  return p;
}

std::optional<std::uint64_t> hello_time(const std::vector<std::uint8_t>& payload) {
  if (payload.size() != kHelloPayloadSize) return std::nullopt;
  std::uint64_t t = 0;
  for (std::size_t i = 0; i < kHelloPayloadSize; ++i) {
    t |= static_cast<std::uint64_t>(payload[i]) << (8 * i);
  }
  return t;
}

// ---------------------------------------------------------------- master

MasterNodeReport run_master_node(net::UdpSocket& socket, const net::Endpoint& slave,
                                 Device1D& device, const MasterNodeConfig& cfg,
                                 std::unique_ptr<net::ChannelModel> emulation,
                                 const std::atomic<bool>* stop) {
  if (cfg.rendezvous && !cfg.psk) {
    throw std::invalid_argument("run_master_node: a rendezvous needs a pre-shared key");
  }
  if (!cfg.rendezvous && !slave.valid()) {
    throw std::invalid_argument("run_master_node: no slave address and no rendezvous");
  }
  MasterNodeReport rep;
  rep.session = cfg.session != 0 ? cfg.session : random_session();
  // The form this socket sends to and receives from: a dual-stack socket
  // sees an IPv4 slave as IPv4-mapped.
  if (slave.valid()) rep.slave = slave.for_family(socket.is_ipv6()).value_or(slave);
  socket.enable_rx_timestamps();
  std::vector<std::uint8_t> buf(wire::kMaxDatagram + 1);
  std::optional<crypto::SessionKeys> keys;
  std::optional<net::RendezvousClient> rv;
  if (cfg.rendezvous) {
    rv.emplace(cfg.rendezvous->server, cfg.rendezvous->id, net::RendezvousRole::kMaster,
               net::host_addresses(socket));
  }
  if (cfg.psk) {
    const auto hs = master_handshake(socket, slave, rep.session, device, cfg, stop, buf, rep, rv);
    if (!hs) {
      rep.handshake_failed = true;
      return rep;
    }
    keys = hs->keys;
    rep.slave = hs->slave;
  }
  // The master has no relay of its own: a relay at the slave suffices.
  net::Paths paths(socket);
  HapticLink link(paths, rep.session, net::Route{rep.slave, false}, cfg.control.passivity);
  if (keys) {
    link.set_keys(*keys);
    rep.authenticated = true;
  }
  if (emulation) link.set_emulation(std::move(emulation));
  MasterController ctl(cfg.control);
  CsvLog log(cfg.log_path, "t,late_us,x_m,v_m,f_display,ein,eout,granted,pc,sent,rx,rtt_us\n");
  std::optional<SessionRejoin> rejoin;
  std::optional<net::Endpoint> server;
  if (rv) server = cfg.rendezvous->server.for_family(socket.is_ipv6());
  if (rv && server) rejoin.emplace(*rv, *server, *cfg.rendezvous, paths);

  std::vector<std::pair<Nanos, double>> own;     // (master time, master net) per tick
  std::vector<std::pair<Nanos, double>> remote;  // (slave time, slave net) per sample
  Nanos newest_remote = INT64_MIN;
  Nanos slave_final_at = -1;
  Nanos last_rx = monotonic_now();
  auto on_rx = [&](const std::uint8_t* data, std::size_t n, const net::Route& from, Nanos rx) {
    if (rejoin && rejoin->on_datagram(data, n, from, monotonic_now())) return;
    if (is_punch(data, n)) return;
    const auto r = link.on_datagram(data, n, from, rx);
    if (!r) return;
    last_rx = std::max(last_rx, rx);
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
    const Nanos now = serve_until(paths, &link, ticker.due(), buf, on_rx);
    if (ending_since < 0 && (now >= end || (stop && stop->load()) || slave_final_at >= 0)) {
      ending_since = now;
      rep.ended_by_slave = slave_final_at >= 0;
    }
    if (ending_since >= 0) {
      if (slave_final_at >= 0 && now >= slave_final_at + cfg.final_linger) break;
      if (now >= ending_since + cfg.final_timeout) break;
    }
    const Nanos late = now - ticker.due();
    ticker.taken(now);
    if (rejoin) rejoin->tick(now, last_rx, link);

    const double x = device.read_position(now);
    const auto& port = ctl.port();
    const bool final = ending_since >= 0;
    // While the session ends the actuator is off and the controller no
    // longer steps, so the port's books stay as they are and every final
    // sample carries the same, final, figures.
    MasterController::Step st;
    if (!final) {
      st = ctl.step(now, x, device.force_cut_after());
      device.command_force(-st.display_force);
    } else {
      device.command_force(0.0);
    }
    const double net = port.input_energy() - port.output_energy();
    if (st.send || final) {
      const Sample s =
          st.send ? st.sample : Sample{ctl.velocity(), x, port.reported_input_energy()};
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
  if (rejoin) rep.rejoins = rejoin->rejoins();
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

  // Answered hellos waiting for the master's first sealed sample. A hello is
  // recognized by its bytes, not by where it came from: a copy that an
  // attacker forwards ahead of the genuine one gets the same reply, which
  // only the master can use, instead of a slot of its own.
  struct PendingHandshake {
    std::uint32_t session;
    std::vector<std::uint8_t> hello;  // handshake message 1 as received
    crypto::SessionKeys keys;
    std::vector<std::uint8_t> reply;  // sent again for every copy of the hello
    Nanos answered;
  };
  constexpr std::size_t kMaxPending = 16;
  std::vector<PendingHandshake> pending;
  std::vector<std::uint8_t> scratch;
  std::uint64_t newest_hello = cfg.newest_hello;
  auto refuse = [&](std::uint32_t session, const std::string& why) {
    if (hooks.on_refused && session != last_refused) hooks.on_refused(why);
    last_refused = session;
  };

  // Rendezvous: registered while idle, it announces masters. For each one
  // announced this end primes its NAT and says it is ready, then punches
  // towards the master for a while once that is ready too, so that its
  // hellos get through NATs further away than the priming reaches.
  std::optional<net::RendezvousClient> rv;
  std::optional<net::Endpoint> rv_server;
  if (cfg.rendezvous) {
    if (!cfg.psk) throw std::invalid_argument("run_slave_node: a rendezvous needs a pre-shared key");
    rv.emplace(cfg.rendezvous->server, cfg.rendezvous->id, net::RendezvousRole::kSlave,
               net::host_addresses(socket));
    rv_server = cfg.rendezvous->server.for_family(socket.is_ipv6());
  }
  // The relay: allocated at once and kept, so that its address is in every
  // registration; the masters the server announces get permissions on it.
  net::Paths paths(socket);
  RelayAnnouncer relay_announcer;
  if (cfg.relay) {
    if (!cfg.rendezvous) throw std::invalid_argument("run_slave_node: a relay needs a rendezvous");
    paths.enable_relay(cfg.relay->server, cfg.relay->username, cfg.relay->password);
  }
  bool rv_answered = false;
  std::uint64_t rv_changes = 0;
  std::vector<net::Endpoint> punch_to;
  Nanos announced = -1;  // when the master to punch towards was announced
  Nanos punch_until = -1, next_punch = 0;
  const auto punch = punch_datagram();
  auto tell = [&](const std::string& what) {
    if (hooks.on_rendezvous) hooks.on_rendezvous(what);
  };

  while (!stopped()) {
    // Idle: wait for the first datagram of a session, keeping a simulated
    // device's physics current with the actuator off.
    device.command_force(0.0);
    struct First {
      wire::Header h;
      net::Route from;
      std::vector<std::uint8_t> bytes;
      Nanos rx;
      Passivity scheme;
      std::optional<crypto::SessionKeys> keys;
    };
    std::optional<First> first;
    pending.clear();
    while (!stopped() && !first) {
      {
        const Nanos t = monotonic_now();
        Nanos wake = t + 100 * kNanosPerMilli;
        if (announced >= 0 && punch_until < 0) {
          // Punches go the whole way once the master's NAT expects them.
          if (rv->peer_ready() || t - announced >= cfg.rendezvous->ready_wait) {
            punch_until = t + cfg.rendezvous->punch_duration;
            next_punch = t;
          } else {
            wake = std::min(wake, announced + cfg.rendezvous->ready_wait);
          }
        }
        if (rv) wake = std::min(wake, rv->next_due());
        if (punch_until > t) wake = std::min(wake, next_punch);
        wake = std::min(wake, paths.next_due());
        if (wake > t) paths.wait_readable(wake - t);
      }
      const Nanos now = monotonic_now();
      device.read_position(now);
      paths.service(now);
      if (rv) {
        if (const auto line = relay_announcer.update(paths, *rv, now)) tell(*line);
        if (const auto reg = rv->poll(now); reg && rv_server) {
          socket.send_to(*rv_server, reg->data(), reg->size());
        }
      }
      if (now < punch_until && now >= next_punch) {
        for (const auto& t : punch_to) socket.send_to(t, punch.data(), punch.size());
        next_punch = now + cfg.rendezvous->punch_interval;
      }
      pending.erase(std::remove_if(pending.begin(), pending.end(),
                                   [&](const PendingHandshake& p) {
                                     return now - p.answered > cfg.handshake_timeout;
                                   }),
                    pending.end());
      for (;;) {
        net::Route from;
        Nanos rx = 0;
        const auto n = paths.recv(buf.data(), buf.size(), &from, &rx);
        if (!n) break;
        if (*n > wire::kMaxDatagram) continue;
        if (rv && !from.relayed && rv->on_datagram(buf.data(), *n, from.peer, now)) {
          if (!rv_answered) {
            rv_answered = true;
            tell("registered at " + rv->server().to_string() + ", which sees this end at " +
                 rv->observed().to_string());
          }
          if (rv->peer_changes() != rv_changes) {
            rv_changes = rv->peer_changes();
            punch_to = net::sendable(rv->peer(), socket);
            // The master's NAT gives it the same address towards the relay
            // as towards the server, whatever port (RFC 4787, REQ-2).
            paths.permit(rv->peer());
            announced = now;
            punch_until = -1;
            if (!prime(socket, punch_to, cfg.rendezvous->prime_hops, punch)) {
              // Unprimed: the punches open this end's NAT, at once.
              punch_until = now + cfg.rendezvous->punch_duration;
              next_punch = now;
            }
            rv->set_ready(true);
            std::string list;
            for (const auto& c : rv->peer()) list += (list.empty() ? "" : ", ") + c.to_string();
            tell("master announced at " + list +
                 (punch_until < 0 ? "" : " (cannot limit the hop count; punching at once)"));
          }
          continue;
        }
        const auto h = wire::decode_header(buf.data(), *n);
        if (!h || (h->type != wire::PacketType::kHaptic && h->type != wire::PacketType::kHello)) {
          continue;
        }
        const auto scheme = scheme_of(*h);
        if (!scheme) continue;
        if (*scheme != cfg.control.passivity && !cfg.adopt_scheme) {
          refuse(h->session, "session from " + from.to_string() +
                                 " runs another passivity scheme; refused");
          continue;
        }
        const std::uint8_t* const payload = buf.data() + wire::kHeaderSize;

        if (h->type == wire::PacketType::kHello) {
          if (!cfg.psk) {
            refuse(h->session, "session from " + from.to_string() +
                                   " wants authentication, but this slave has no key; refused");
            continue;
          }
          const auto same = std::find_if(pending.begin(), pending.end(),
                                         [&](const PendingHandshake& p) {
                                           return p.session == h->session &&
                                                  p.hello.size() == h->payload_len &&
                                                  std::memcmp(p.hello.data(), payload,
                                                              h->payload_len) == 0;
                                         });
          if (same != pending.end()) {  // resent because our reply was lost, or a copy
            paths.send(from, same->reply.data(), same->reply.size());
            continue;
          }
          const auto prologue = handshake_prologue(h->session, *scheme);
          crypto::NoiseNNpsk0 hs(crypto::NoiseNNpsk0::Role::kResponder, *cfg.psk,
                                 prologue.data(), prologue.size());
          const auto hello = hs.read_message(payload, h->payload_len);
          if (!hello) {
            refuse(h->session, "handshake from " + from.to_string() +
                                   " does not authenticate (another key?); refused");
            continue;
          }
          const auto stamp = hello_time(*hello);
          if (!stamp) {
            refuse(h->session, "handshake from " + from.to_string() +
                                   " carries no time (another version?); refused");
            continue;
          }
          if (*stamp <= newest_hello) {
            refuse(h->session, "handshake from " + from.to_string() +
                                   " is not newer than the last one answered (replayed?); refused");
            continue;
          }
          newest_hello = *stamp;
          const auto msg2 = hs.write_message(&kNoPayload, 0);
          wire::Header rh;
          rh.type = wire::PacketType::kHelloReply;
          rh.session = h->session;
          rh.seq = 1;
          rh.flags = scheme_flags(*scheme);
          rh.send_time = monotonic_now();
          rh.echo_seq = h->seq;
          rh.echo_hold = wire::hold_field(rh.send_time - rx);
          PendingHandshake p{h->session, std::vector<std::uint8_t>(payload, payload + h->payload_len),
                             hs.split(), {}, rh.send_time};
          wire::encode_raw(rh, msg2.data(), msg2.size(), p.reply);
          paths.send(from, p.reply.data(), p.reply.size());
          if (pending.size() == kMaxPending) pending.erase(pending.begin());
          pending.push_back(std::move(p));
          continue;
        }

        // kHaptic. With a key, only a datagram sealed with a pending
        // handshake's keys starts a session: it proves the master knows the
        // psk and holds the ephemeral key of that handshake, wherever it
        // comes from (a NAT may have moved the master since its hello).
        if ((h->flags & wire::kFlagFinal)) continue;
        std::optional<crypto::SessionKeys> keys;
        if (cfg.psk) {
          if (!(h->flags & wire::kFlagSealed)) {
            refuse(h->session,
                   "unauthenticated session from " + from.to_string() + " refused");
            continue;
          }
          for (const auto& p : pending) {
            if (p.session != h->session) continue;
            scratch.clear();
            if (crypto::open(p.keys.recv, h->seq, buf.data(), wire::kHeaderSize, payload,
                             h->payload_len, scratch) &&
                wire::decode_haptic_payload(scratch.data(), scratch.size())) {
              keys = p.keys;
              break;
            }
          }
          if (!keys) continue;
        } else if (!wire::decode_haptic(buf.data(), *n)) {
          continue;
        }
        first = First{*h, from, std::vector<std::uint8_t>(buf.begin(), buf.begin() + *n), rx,
                      *scheme, keys};
        break;  // the rest stays queued for the session
      }
    }
    if (!first) break;
    announced = -1;
    punch_until = -1;

    SlaveSessionReport rep;
    rep.session = first->h.session;
    rep.master = first->from.peer;
    rep.relayed = first->from.relayed;
    rep.scheme = first->scheme;
    rep.authenticated = first->keys.has_value();
    SlaveConfig sc = cfg.control;
    sc.passivity = first->scheme;
    HapticLink link(paths, rep.session, first->from, sc.passivity);
    if (first->keys) link.set_keys(*first->keys);
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
    std::optional<SessionRejoin> rejoin;
    if (rv && rv_server) rejoin.emplace(*rv, *rv_server, *cfg.rendezvous, paths);
    auto on_rx = [&](const std::uint8_t* data, std::size_t n, const net::Route& from, Nanos rx) {
      if (rejoin && rejoin->on_datagram(data, n, from, monotonic_now())) return;
      if (is_punch(data, n)) return;
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
      const Nanos now = serve_until(paths, &link, ticker.due(), buf, on_rx);
      if (now - last_rx > cfg.idle_timeout) break;
      if (master_final_at >= 0 && now >= master_final_at + cfg.final_linger) {
        rep.ended_by_final = true;
        break;
      }
      if (stop_since < 0 && stopped()) stop_since = now;
      if (stop_since >= 0 && now >= stop_since + cfg.final_linger) break;
      const Nanos late = now - ticker.due();
      ticker.taken(now);
      if (rejoin) rejoin->tick(now, last_rx, link);
      if (rv) {
        if (const auto line = relay_announcer.update(paths, *rv, now)) tell(*line);
      }

      const double x = device.read_position(now);
      const double fc = device.contact_force();
      const bool final = master_final_at >= 0 || stop_since >= 0;
      // As at the master: once the session ends the actuator is off and the
      // books are frozen, so the master gets the same final figures from
      // whichever final sample reaches it.
      SlaveController::Step st;
      if (!final) {
        st = ctl.step(now, x, device.velocity(), fc, device.force_cut_after());
        device.command_force(st.force);
      } else {
        device.command_force(0.0);
      }
      if (st.send || final) {
        Sample s = st.send ? st.sample
                           : Sample{final ? 0.0 : ctl.reported_force(), 0.0,
                                    transmitted_energy(ctl, sc.passivity)};
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
    if (rejoin) rep.rejoins = rejoin->rejoins();
    rep.slave_net = ctl.terminal_net();
    rep.slave_pc_active =
        rep.ticks ? static_cast<double>(pc_count) / static_cast<double>(rep.ticks) : 0.0;
    // Announcements during the session were handled there.
    if (rv) rv_changes = rv->peer_changes();
    if (hooks.on_session_end) hooks.on_session_end(rep);
    reports.push_back(rep);
    if (cfg.once) break;
  }
  device.command_force(0.0);
  // Frees the relay at the server at once instead of at the end of its
  // lifetime; the answer is not waited for.
  if (net::TurnClient* turn = paths.turn()) {
    turn->release();
    paths.service(monotonic_now());
  }
  return reports;
}

}  // namespace tuushin::teleop
