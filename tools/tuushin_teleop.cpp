// tuushin-teleop: runs the master and the slave of the 1-DoF teleoperator as
// two processes that talk over UDP in real time, with simulated devices (an
// operator pushing a desktop haptic device, a robot touching a wall) until
// real ones are attached through teleop::Device1D.
//
//   on the robot side:     tuushin-teleop slave --port 47100
//   on the operator side:  tuushin-teleop master HOST --port 47100 --duration 20 --log m.csv
//
// With a shared key (tuushin-teleop keygen FILE, copied to both machines,
// then --key-file FILE on both ends) the ends authenticate each other and
// seal every datagram; without one, anyone who can reach the slave's port
// can drive it.
//
// When the slave is behind a NAT (no port forwarded to it), both ends can
// meet through a rendezvous server that both can reach, which tells each the
// other's addresses and carries nothing else (net/rendezvous.hpp):
//
//   somewhere reachable:   tuushin-teleop rendezvous --port 47200
//   on the robot side:     tuushin-teleop slave --key-file K --rendezvous SERVER
//   on the operator side:  tuushin-teleop master --key-file K --rendezvous SERVER
//
// Both ends can impair their outgoing datagrams in-process to try a path on
// loopback: --emulate-delay 25 on each end gives a 50 ms round trip, and
// --emulate-trace replays a path recorded with tuushin-probe.

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include "tuushin/crypto/noise.hpp"
#include "tuushin/net/channel.hpp"
#include "tuushin/net/rendezvous.hpp"
#include "tuushin/net/trace.hpp"
#include "tuushin/net/udp.hpp"
#include "tuushin/rt/thread.hpp"
#include "tuushin/teleop/device.hpp"
#include "tuushin/teleop/node.hpp"
#include "tuushin/time.hpp"

using namespace tuushin;

namespace {

std::atomic<bool> g_stop{false};

void on_signal(int) { g_stop.store(true); }

void usage() {
  std::fprintf(stderr,
               "usage:\n"
               "  tuushin-teleop keygen FILE\n"
               "      write a new random key to FILE (mode 0600; never overwrites)\n"
               "  tuushin-teleop slave [options]\n"
               "      --bind HOST          local address (all, IPv6 and IPv4)\n"
               "      --once               serve one session, then exit\n"
               "      --adopt-scheme       run whatever scheme the master announces\n"
               "      --initial-energy E   reflect: energy in the store at start in J (0)\n"
               "      --hold-after MS      stop extrapolating the master after MS without\n"
               "                           a sample (150)\n"
               "      --contact-force      report the contact force, not the coupling force\n"
               "      --wall-k K           simulated wall stiffness in N/m (5000)\n"
               "      --log PREFIX         per-session CSV log PREFIX<session>.csv\n"
               "  tuushin-teleop master [HOST] [options]\n"
               "      HOST                 the slave; optional with --rendezvous\n"
               "      --duration S         seconds to run (10)\n"
               "      --max-force F        master actuator limit in N (20)\n"
               "      --ipv6               prefer IPv6 when HOST has both\n"
               "      --log FILE           per-tick CSV log\n"
               "  tuushin-teleop rendezvous [options]\n"
               "      introduce masters and slaves that share a key to each other, so\n"
               "      that they can reach each other through NATs; relays nothing\n"
               "      --bind HOST          local address (all, IPv6 and IPv4)\n"
               "      --port P             UDP port (47200)\n"
               "  common options\n"
               "      --key-file FILE      shared key (keygen): authenticate the other end\n"
               "                           and seal every datagram\n"
               "      --rendezvous HOST[:PORT]\n"
               "                           meet the other end through this rendezvous\n"
               "                           server (port 47200); needs --key-file\n"
               "      --prime-hops N       with --rendezvous: hop limit of the datagrams that\n"
               "                           open this end's NAT before the other end's\n"
               "                           arrive (3; 0 does not limit them)\n"
               "      --port P             slave's UDP port (47100)\n"
               "      --scheme S           none, channel or reflect (reflect)\n"
               "      --rate HZ            control rate (1000)\n"
               "      --deadband K         perceptual deadband with Weber fraction K\n"
               "      --dscp D             DSCP of outgoing datagrams (46)\n"
               "      --no-rt              do not ask for low-latency scheduling\n"
               "      --watchdog MS        the simulated device cuts its force after MS\n"
               "                           without a command (two periods; 0 disables)\n"
               "      --emulate-delay MS   add this one-way delay to outgoing datagrams\n"
               "      --emulate-jitter MS  mean exponential jitter on top (FIFO)\n"
               "      --emulate-reorder    let the jitter reorder datagrams\n"
               "      --emulate-loss P     independent loss probability\n"
               "      --emulate-trace FILE replay a recorded one-way path (tuushin-probe)\n");
}

double parse_double(const std::string& s, const char* what) {
  char* end = nullptr;
  const double v = std::strtod(s.c_str(), &end);
  if (end == s.c_str() || *end != '\0') throw std::runtime_error(std::string("bad ") + what + ": " + s);
  return v;
}

constexpr std::uint16_t kDefaultPort = 47100;
constexpr std::uint16_t kDefaultRendezvousPort = 47200;

struct Options {
  std::string mode;
  std::string host, bind;
  std::optional<std::uint16_t> port;
  std::string rendezvous;
  teleop::Passivity scheme = teleop::Passivity::kReflective;
  double rate = 1000.0;
  double deadband = 0.0;
  int dscp = 46;
  int prime_hops = 3;
  bool rt = true;
  double watchdog_ms = -1.0;  // negative: the PC's holding horizon
  bool ipv6 = false;
  std::string log;
  std::string key_file;
  // master
  double duration = 10.0;
  double max_force = 20.0;
  // slave
  bool once = false;
  bool adopt = false;
  double initial_energy = 0.0;
  double hold_after_ms = 150.0;
  bool contact_force = false;
  double wall_k = 5000.0;
  // emulation
  double delay_ms = 0.0, jitter_ms = 0.0, loss = 0.0;
  bool reorder = false;
  std::string trace;
};

std::uint16_t parse_port(const std::string& s) {
  const double p = parse_double(s, "port");
  if (p < 1 || p > 65535 || p != static_cast<double>(static_cast<int>(p))) {
    throw std::runtime_error("bad port: " + s);
  }
  return static_cast<std::uint16_t>(p);
}

// "HOST", "HOST:PORT", "[V6]" or "[V6]:PORT"; a bare IPv6 literal has no port.
std::pair<std::string, std::uint16_t> split_host_port(const std::string& s, std::uint16_t def) {
  if (!s.empty() && s[0] == '[') {
    const auto close = s.find(']');
    if (close == std::string::npos) throw std::runtime_error("bad address: " + s);
    const std::string host = s.substr(1, close - 1);
    if (close + 1 == s.size()) return {host, def};
    if (s[close + 1] != ':') throw std::runtime_error("bad address: " + s);
    return {host, parse_port(s.substr(close + 2))};
  }
  const auto colon = s.rfind(':');
  if (colon == std::string::npos || s.find(':') != colon) return {s, def};
  return {s.substr(0, colon), parse_port(s.substr(colon + 1))};
}

bool emulating(const Options& o) {
  return o.delay_ms > 0.0 || o.jitter_ms > 0.0 || o.loss > 0.0 || !o.trace.empty();
}

std::unique_ptr<net::ChannelModel> make_emulation(const Options& o, Nanos start,
                                                  std::uint64_t seed) {
  if (!emulating(o)) return nullptr;
  std::unique_ptr<net::ChannelModel> ch;
  if (!o.trace.empty()) {
    ch = std::make_unique<net::TraceChannel>(net::Trace::load_csv(o.trace), start);
  } else if (o.jitter_ms > 0.0) {
    ch = std::make_unique<net::JitterDelay>(from_millis(o.delay_ms), from_millis(o.jitter_ms),
                                            !o.reorder, seed);
  } else {
    ch = std::make_unique<net::FixedDelay>(from_millis(o.delay_ms));
  }
  if (o.loss > 0.0) {
    net::GilbertElliottLoss::Params iid{0.0, 1.0, o.loss, 1.0};
    ch = std::make_unique<net::GilbertElliottLoss>(std::move(ch), iid, seed + 100);
  }
  return ch;
}

// The predictive PCs choose a force they can afford to hold for
// pc_hold_samples periods. A device watchdog at that horizon keeps a stalled
// loop from holding it longer, which is what makes stalls safe: without one
// the last force acts for the whole stall and no controller can stop it.
Nanos watchdog_of(const Options& o, Nanos period, int pc_hold_samples) {
  if (o.watchdog_ms < 0.0) return std::max(1, pc_hold_samples) * period;
  return from_millis(o.watchdog_ms);
}

const char* scheme_name(teleop::Passivity p) {
  switch (p) {
    case teleop::Passivity::kNone: return "none";
    case teleop::Passivity::kChannel: return "channel";
    case teleop::Passivity::kReflective: return "reflect";
  }
  return "?";
}

Nanos period_of(const Options& o) {
  if (o.rate < 10.0 || o.rate > 10000.0) throw std::runtime_error("--rate must be 10..10000 Hz");
  return static_cast<Nanos>(1e9 / o.rate);
}

void print_stats(const char* name, const net::DelayStats& s, double unit, const char* u) {
  if (s.count == 0) {
    std::printf("%-14s none\n", name);
    return;
  }
  std::printf("%-14s min %.3f  p50 %.3f  p99 %.3f  p99.9 %.3f  max %.3f %s", name, s.min / unit,
              s.p50 / unit, s.p99 / unit, s.p999 / unit, s.max / unit, u);
  if (s.rfc3550_jitter > 0) std::printf("  jitter %.3f", s.rfc3550_jitter / unit);
  std::printf("\n");
}

void print_link(const teleop::HapticLink::Counters& c, const char* rx_dir) {
  const double lost = c.received + c.missing > 0
                          ? 100.0 * static_cast<double>(c.missing) /
                                static_cast<double>(c.received + c.missing)
                          : 0.0;
  std::printf("sent %llu (send failures %llu, emulated losses %llu)\n",
              static_cast<unsigned long long>(c.sent),
              static_cast<unsigned long long>(c.send_failed),
              static_cast<unsigned long long>(c.emulated_lost));
  std::printf("received %llu %s, missing %llu (%.2f %%), late %llu, duplicates %llu, foreign %llu",
              static_cast<unsigned long long>(c.received), rx_dir,
              static_cast<unsigned long long>(c.missing), lost,
              static_cast<unsigned long long>(c.late),
              static_cast<unsigned long long>(c.duplicates),
              static_cast<unsigned long long>(c.foreign));
  if (c.unauthentic) {
    std::printf(", not authentic %llu", static_cast<unsigned long long>(c.unauthentic));
  }
  std::printf("\n");
}

// keygen: a fresh key in hex, in a file only its owner can read. An existing
// file is left alone, since overwriting a key locks out the other end.
int run_keygen(const std::string& path) {
  const std::string text = crypto::to_hex(crypto::random_key()) + "\n";
  const int fd = ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL, 0600);
  if (fd < 0) throw std::runtime_error("cannot create " + path + ": " + std::strerror(errno));
  const bool ok = ::write(fd, text.data(), text.size()) == static_cast<ssize_t>(text.size());
  ::close(fd);
  if (!ok) throw std::runtime_error("cannot write " + path);
  std::fprintf(stderr, "wrote a new key to %s; copy it to the other end over a channel you trust\n",
               path.c_str());
  return 0;
}

std::optional<crypto::Key> load_key(const Options& o) {
  if (o.key_file.empty()) {
    std::fprintf(stderr,
                 "note: no --key-file; datagrams are not authenticated, so anyone who can "
                 "reach the slave can drive it\n");
    return std::nullopt;
  }
  std::FILE* f = std::fopen(o.key_file.c_str(), "r");
  if (!f) throw std::runtime_error("cannot open key file " + o.key_file);
  char text[256];
  const std::size_t n = std::fread(text, 1, sizeof text - 1, f);
  std::fclose(f);
  text[n] = '\0';
  const auto key = crypto::key_from_hex(text);
  if (!key) throw std::runtime_error(o.key_file + " does not hold a key (64 hex digits)");
  struct stat st;
  if (::stat(o.key_file.c_str(), &st) == 0 && (st.st_mode & 077) != 0) {
    std::fprintf(stderr, "note: %s is readable by other users (chmod 600 it)\n",
                 o.key_file.c_str());
  }
  return key;
}

net::UdpSocket open_socket(const Options& o, const net::Endpoint& local) {
  net::UdpSocket sock(local);
  if (!sock.set_dscp(o.dscp)) std::fprintf(stderr, "note: DSCP %d refused by the system\n", o.dscp);
  return sock;
}

// A socket on all addresses of both families where the host has IPv6, and
// on all IPv4 addresses where it does not.
net::UdpSocket open_dual_stack(const Options& o, std::uint16_t port, bool dscp = true) {
  auto open = [&](const char* any) {
    const auto local = net::Endpoint::resolve(any, port);
    if (!local) throw std::runtime_error(std::string("cannot make a local address ") + any);
    return dscp ? open_socket(o, *local) : net::UdpSocket(*local);
  };
  try {
    return open("::");
  } catch (const std::exception&) {
    return open("0.0.0.0");
  }
}

// --bind if given, both families otherwise.
net::UdpSocket open_listening(const Options& o, std::uint16_t port, bool dscp = true) {
  if (o.bind.empty()) return open_dual_stack(o, port, dscp);
  const auto local = net::Endpoint::resolve(o.bind, port);
  if (!local) throw std::runtime_error("cannot resolve bind address '" + o.bind + "'");
  return dscp ? open_socket(o, *local) : net::UdpSocket(*local);
}

// The meeting of --rendezvous, reached from `sock`.
std::optional<teleop::RendezvousConfig> rendezvous_of(const Options& o,
                                                      const std::optional<crypto::Key>& psk,
                                                      const net::UdpSocket& sock) {
  if (o.rendezvous.empty()) return std::nullopt;
  if (!psk) throw std::runtime_error("--rendezvous needs --key-file");
  const auto [host, port] = split_host_port(o.rendezvous, kDefaultRendezvousPort);
  const auto server = net::Endpoint::resolve(host, port, o.ipv6);
  if (!server) throw std::runtime_error("cannot resolve rendezvous server " + host);
  if (!server->for_family(sock.is_ipv6())) {
    throw std::runtime_error("the rendezvous server " + server->to_string() +
                             " is IPv6, and this host has no IPv6 socket");
  }
  if (o.prime_hops < 0 || o.prime_hops > 255) {
    throw std::runtime_error("--prime-hops must be 0..255");
  }
  teleop::RendezvousConfig rc;
  rc.server = *server;
  rc.id = crypto::rendezvous_id(*psk);
  rc.prime_hops = o.prime_hops;
  return rc;
}

std::string join(const std::vector<net::Endpoint>& v) {
  std::string s;
  for (const auto& e : v) s += (s.empty() ? "" : ", ") + e.to_string();
  return s;
}

std::string utc_now() {
  const std::time_t t = std::time(nullptr);
  std::tm tm{};
  ::gmtime_r(&t, &tm);
  char b[32];
  std::strftime(b, sizeof b, "%Y-%m-%dT%H:%M:%SZ", &tm);
  return b;
}

int run_master(const Options& o) {
  if (!o.rendezvous.empty() && o.key_file.empty()) {
    throw std::runtime_error("--rendezvous needs --key-file");
  }
  std::optional<net::Endpoint> peer;
  if (!o.host.empty()) {
    peer = net::Endpoint::resolve(o.host, o.port.value_or(kDefaultPort), o.ipv6);
    if (!peer) throw std::runtime_error("cannot resolve " + o.host);
  }
  // Through a rendezvous the slave's addresses may be of either family.
  std::optional<net::UdpSocket> opened;
  if (!o.rendezvous.empty()) {
    opened.emplace(open_dual_stack(o, 0));
  } else {
    const auto local = net::Endpoint::resolve(peer->is_ipv6() ? "::" : "0.0.0.0", 0);
    if (!local) throw std::runtime_error("cannot make a local address");
    opened.emplace(open_socket(o, *local));
  }
  net::UdpSocket& sock = *opened;
  const Nanos period = period_of(o);
  const std::string sched = o.rt ? rt::prefer_low_latency(period) : "default scheduling";

  teleop::MasterNodeConfig cfg;
  cfg.control.dt = period;
  cfg.control.passivity = o.scheme;
  cfg.control.max_force = o.max_force;
  cfg.control.deadband = o.deadband > 0.0;
  cfg.control.deadband_params.k = o.deadband;
  cfg.duration = from_seconds(o.duration);
  cfg.log_path = o.log;
  cfg.psk = load_key(o);
  cfg.rendezvous = rendezvous_of(o, cfg.psk, sock);

  const Nanos start = monotonic_now();
  teleop::OperatorParams op;
  teleop::SimulatedMaster device(0.2, 0.5, op, period, 10, start);
  device.set_watchdog(watchdog_of(o, period, cfg.control.pc_hold_samples));
  cfg.control.model_mass = 0.2;
  std::string target = peer ? peer->to_string() : std::string();
  if (cfg.rendezvous) {
    target += (target.empty() ? "" : " and ") +
              std::string("the slave met at ") + cfg.rendezvous->server.to_string();
  }
  std::fprintf(stderr, "master -> %s, scheme %s, %.0f Hz, %s%s\n", target.c_str(),
               scheme_name(o.scheme), o.rate, sched.c_str(),
               emulating(o) ? ", emulating impairments" : "");
  const auto rep = teleop::run_master_node(sock, peer ? *peer : net::Endpoint{}, device, cfg,
                                           make_emulation(o, start, 1), &g_stop);
  if (cfg.rendezvous) {
    if (rep.observed.valid()) {
      std::printf("rendezvous: %s sees this end at %s\n",
                  cfg.rendezvous->server.to_string().c_str(), rep.observed.to_string().c_str());
    } else {
      std::printf("rendezvous: no answer from %s\n", cfg.rendezvous->server.to_string().c_str());
    }
    if (!rep.candidates.empty()) {
      std::printf("rendezvous: the slave at %s\n", join(rep.candidates).c_str());
    }
  }
  if (rep.rendezvous_failed) {
    std::printf("the rendezvous server announced no slave within %.0f s (is the slave registered "
                "there, with the same key?)\n",
                to_seconds(cfg.rendezvous->wait));
    return 2;
  }
  if (rep.handshake_failed) {
    std::printf("handshake failed: no authenticated reply within %.0f s (is the slave running, "
                "with the same key?",
                to_seconds(cfg.handshake_timeout));
    if (!rep.candidates.empty()) {
      std::printf(" Or a NAT on the way gives every destination its own port, which hole "
                  "punching cannot cross; see the README");
    }
    std::printf(")\n");
    return 2;
  }
  if (cfg.rendezvous) std::printf("reached the slave at %s\n", rep.slave.canonical().to_string().c_str());
  if (rep.authenticated) std::printf("authenticated (%s)\n", crypto::NoiseNNpsk0::kProtocolName);

  std::printf("session %08x, %llu ticks (%llu skipped), master PC active %.1f %%\n", rep.session,
              static_cast<unsigned long long>(rep.ticks),
              static_cast<unsigned long long>(rep.skipped), 100.0 * rep.master_pc_active);
  if (rep.ended_by_slave) std::printf("the slave ended the session\n");
  print_stats("tick late", rep.lateness, 1e3, "us");
  print_link(rep.link, "from the slave");
  if (!rep.slave_answered) {
    std::printf("no answer from the slave at %s%s\n", rep.slave.canonical().to_string().c_str(),
                cfg.psk ? "" : " (a slave with --key-file refuses a master without one)");
    return 2;
  }
  print_stats("rtt", rep.rtt, 1e6, "ms");
  print_stats("one-way fwd", rep.forward, 1e6, "ms");
  print_stats("one-way bwd", rep.backward, 1e6, "ms");
  std::printf("clock skew %.2f ppm (one-way delays assume equal minimum delays)\n",
              rep.clock.skew * 1e6);
  std::printf("max position %.4f m, max displayed force %.3f N\n", rep.max_position, rep.max_force);
  std::printf("energy: master %.6g J, slave %.6g J%s, end margin %.6g J, min margin %.6g J\n",
              rep.master_net, rep.slave_net, rep.slave_final ? "" : " (no final books)",
              rep.master_net + rep.slave_net, rep.min_margin);
  return 0;
}

int run_slave(const Options& o) {
  if (!o.rendezvous.empty() && o.key_file.empty()) {
    throw std::runtime_error("--rendezvous needs --key-file");
  }
  net::UdpSocket sock = open_listening(o, o.port.value_or(kDefaultPort));
  const Nanos period = period_of(o);
  const std::string sched = o.rt ? rt::prefer_low_latency(period) : "default scheduling";

  teleop::SlaveNodeConfig cfg;
  cfg.control.dt = period;
  cfg.control.passivity = o.scheme;
  cfg.control.initial_energy = o.initial_energy;
  cfg.control.report_contact_force = o.contact_force;
  cfg.control.hold_after = from_millis(o.hold_after_ms);
  cfg.control.deadband = o.deadband > 0.0;
  cfg.control.deadband_params.k = o.deadband;
  cfg.control.model_mass = 1.0;
  cfg.adopt_scheme = o.adopt;
  cfg.once = o.once;
  cfg.log_prefix = o.log;
  cfg.psk = load_key(o);
  cfg.rendezvous = rendezvous_of(o, cfg.psk, sock);

  teleop::WallParams wall;
  wall.k = o.wall_k;
  teleop::SimulatedSlave device(1.0, 2.0, wall, period, 10, monotonic_now(), 0.0);
  device.set_watchdog(watchdog_of(o, period, cfg.control.pc_hold_samples));

  teleop::SlaveNodeHooks hooks;
  std::uint64_t seed = 2;
  if (emulating(o)) {
    hooks.emulation = [&](Nanos start) { return make_emulation(o, start, seed++); };
  }
  hooks.on_refused = [](const std::string& why) { std::fprintf(stderr, "%s\n", why.c_str()); };
  hooks.on_rendezvous = [](const std::string& what) {
    std::fprintf(stderr, "rendezvous: %s\n", what.c_str());
  };
  hooks.on_session_end = [](const teleop::SlaveSessionReport& r) {
    std::printf("session %08x from %s, scheme %s, %s, ended by %s\n", r.session,
                r.master.canonical().to_string().c_str(), scheme_name(r.scheme),
                r.authenticated ? "authenticated" : "not authenticated",
                r.ended_by_final ? "the master" : "timeout or stop");
    std::printf("%llu ticks (%llu skipped), slave PC active %.1f %%\n",
                static_cast<unsigned long long>(r.ticks), static_cast<unsigned long long>(r.skipped),
                100.0 * r.slave_pc_active);
    print_stats("tick late", r.lateness, 1e3, "us");
    print_link(r.link, "from the master");
    std::printf("max position %.4f m, max force %.3f N, slave energy %.6g J\n", r.max_position,
                r.max_force, r.slave_net);
    std::fflush(stdout);
  };
  std::fprintf(stderr, "slave on %s, scheme %s%s, %.0f Hz, %s%s%s\n",
               sock.local_endpoint().to_string().c_str(), scheme_name(o.scheme),
               o.adopt ? " (or the master's)" : "", o.rate, sched.c_str(),
               cfg.psk ? ", key required" : "", emulating(o) ? ", emulating impairments" : "");
  teleop::run_slave_node(sock, device, cfg, hooks, &g_stop);
  return 0;
}

// The rendezvous server: one line per registration that is new or moved.
int run_rendezvous(const Options& o) {
  net::UdpSocket sock = open_listening(o, o.port.value_or(kDefaultRendezvousPort), false);
  std::fprintf(stderr, "rendezvous server on %s%s\n", sock.local_endpoint().to_string().c_str(),
               sock.is_ipv6() ? " (IPv6 and IPv4)" : "");
  net::run_rendezvous_server(sock, &g_stop, [](const std::string& line) {
    std::printf("%s %s\n", utc_now().c_str(), line.c_str());
    std::fflush(stdout);
  });
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);
  if (argc < 2) {
    usage();
    return 1;
  }
  Options o;
  o.mode = argv[1];
  try {
    int i = 2;
    auto next = [&]() -> std::string {
      if (i + 1 >= argc) throw std::runtime_error(std::string("missing value for ") + argv[i]);
      return argv[++i];
    };
    for (; i < argc; ++i) {
      const std::string a = argv[i];
      if (a == "--port") {
        o.port = parse_port(next());
      } else if (a == "--scheme") {
        const std::string s = next();
        if (s == "none") o.scheme = teleop::Passivity::kNone;
        else if (s == "channel") o.scheme = teleop::Passivity::kChannel;
        else if (s == "reflect") o.scheme = teleop::Passivity::kReflective;
        else throw std::runtime_error("--scheme must be none, channel or reflect");
      }
      else if (a == "--rate") o.rate = parse_double(next(), "rate");
      else if (a == "--deadband") o.deadband = parse_double(next(), "deadband");
      else if (a == "--dscp") o.dscp = static_cast<int>(parse_double(next(), "dscp"));
      else if (a == "--no-rt") o.rt = false;
      else if (a == "--watchdog") o.watchdog_ms = parse_double(next(), "watchdog");
      else if (a == "--ipv6") o.ipv6 = true;
      else if (a == "--log") o.log = next();
      else if (a == "--key-file") o.key_file = next();
      else if (a == "--rendezvous") o.rendezvous = next();
      else if (a == "--prime-hops") o.prime_hops = static_cast<int>(parse_double(next(), "hops"));
      else if (a == "--duration") o.duration = parse_double(next(), "duration");
      else if (a == "--max-force") o.max_force = parse_double(next(), "max-force");
      else if (a == "--bind") o.bind = next();
      else if (a == "--once") o.once = true;
      else if (a == "--adopt-scheme") o.adopt = true;
      else if (a == "--initial-energy") o.initial_energy = parse_double(next(), "initial-energy");
      else if (a == "--hold-after") o.hold_after_ms = parse_double(next(), "hold-after");
      else if (a == "--contact-force") o.contact_force = true;
      else if (a == "--wall-k") o.wall_k = parse_double(next(), "wall-k");
      else if (a == "--emulate-delay") o.delay_ms = parse_double(next(), "emulate-delay");
      else if (a == "--emulate-jitter") o.jitter_ms = parse_double(next(), "emulate-jitter");
      else if (a == "--emulate-reorder") o.reorder = true;
      else if (a == "--emulate-loss") o.loss = parse_double(next(), "emulate-loss");
      else if (a == "--emulate-trace") o.trace = next();
      else if (a == "-h" || a == "--help") { usage(); return 0; }
      else if ((o.mode == "master" || o.mode == "keygen") && !a.empty() && a[0] != '-' &&
               o.host.empty()) o.host = a;
      else throw std::runtime_error("unknown option " + a);
    }
    if (o.mode == "master") {
      if (o.host.empty() && o.rendezvous.empty()) {
        throw std::runtime_error("master needs the slave's HOST or --rendezvous");
      }
      return run_master(o);
    }
    if (o.mode == "slave") return run_slave(o);
    if (o.mode == "rendezvous") return run_rendezvous(o);
    if (o.mode == "keygen") {
      if (o.host.empty()) throw std::runtime_error("keygen needs a FILE");
      return run_keygen(o.host);
    }
    usage();
    return 1;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "tuushin-teleop: %s\n", e.what());
    return 1;
  }
}
