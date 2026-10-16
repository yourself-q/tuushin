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
// Where both NATs give every destination its own port, no direct path opens;
// the slave can then take a relay on a TURN server (net/paths.hpp), which
// the master is told of and tries when its direct hellos go unanswered:
//
//   on the robot side:     tuushin-teleop slave --key-file K --rendezvous SERVER
//                              --turn TURNSERVER --turn-user U --turn-pass-file P
//
// Both ends can impair their outgoing datagrams in-process to try a path on
// loopback: --emulate-delay 25 on each end gives a 50 ms round trip, and
// --emulate-trace replays a path recorded with tuushin-probe.
//
// --device SPEC drives a knob running the firmware in firmware/ (or
// tuushin-devsim, its simulation) instead of the simulated device, over a
// serial port or TCP:
//
//   tuushin-teleop slave --device /dev/cu.usbmodem1101
//   tuushin-teleop master HOST --device serial:/dev/ttyACM0,radius=0.025

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
#include "tuushin/device/remote.hpp"
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
               "                           (not with reflect, which needs the coupling force)\n"
               "      --wall-k K           simulated wall stiffness in N/m (5000)\n"
               "      --kp K               coupling stiffness in N/m (2000; with --device,\n"
               "                           what the device's damping and delay allow)\n"
               "      --kd B               coupling damping in N s/m (60; 0 with --device)\n"
               "      --log PREFIX         per-session CSV log PREFIX<session>.csv\n"
               "      --hello-state FILE   keep the newest answered hello's time in FILE, so\n"
               "                           that hellos recorded before a restart stay refused\n"
               "      --turn HOST[:PORT]   with --rendezvous: take a relay on this TURN server\n"
               "                           (port 3478) and announce it, for masters that\n"
               "                           cannot reach this end directly\n"
               "      --turn-user NAME     the TURN server's long-term credentials: user name\n"
               "      --turn-pass-file FILE\n"
               "                           and a file holding the password (first line)\n"
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
               "      --prime-hops auto|N  with --rendezvous: hop limit of the datagrams that\n"
               "                           open this end's NAT before the other end's\n"
               "                           arrive (auto: measured on Linux, 3 where it\n"
               "                           cannot be; 0 primes nothing)\n"
               "      --port P             slave's UDP port (47100)\n"
               "      --scheme S           none, channel or reflect (reflect)\n"
               "      --rate HZ            control rate (1000)\n"
               "      --deadband K         perceptual deadband with Weber fraction K\n"
               "      --dscp D             DSCP of outgoing datagrams (46)\n"
               "      --no-rt              do not ask for low-latency scheduling\n"
               "      --device SPEC        drive a device running the firmware instead of the\n"
               "                           simulated one: serial:PATH (or PATH) or\n"
               "                           tcp:HOST:PORT, then ,option=value...: baud (921600),\n"
               "                           radius (0.02 m), reverse, torque-limit (N m),\n"
               "                           damping (0.001 N m s/rad), velocity-limit (rad/s),\n"
               "                           state-us (1000), calibrate=auto|always|never, nozero\n"
               "      --watchdog MS        the device cuts its force after MS without a\n"
               "                           command (two periods; 0 disables)\n"
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
constexpr std::uint16_t kDefaultTurnPort = 3478;  // RFC 8656, section 18

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
  bool prime_auto = true;
  bool rt = true;
  double watchdog_ms = -1.0;  // negative: the PC's holding horizon
  bool ipv6 = false;
  std::string log;
  std::string key_file;
  std::string device;
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
  double kp = -1.0, kd = -1.0;  // negative: the default
  std::string hello_state;
  std::string turn, turn_user, turn_pass_file;
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

// The knob's damping on its own servo ticks (N m s/rad) unless the spec
// sets one: 2.5 N s/m at a 20 mm radius, some 2 mN m of drag at 2 rad/s,
// about the cogging of a gimbal motor. It is what keeps the coupling below
// stable: damping on the device's ticks acts with no link in between.
constexpr double kDeviceDamping = 1e-3;

std::unique_ptr<device::RemoteDevice> open_device(const Options& o, Nanos period,
                                                  int pc_hold_samples) {
  device::DeviceSpec spec = device::parse_device_spec(o.device);
  device::RemoteConfig rc = spec.config;
  if (!spec.damping_given) rc.damping = kDeviceDamping;
  rc.watchdog = watchdog_of(o, period, pc_hold_samples);
  rc.reader_thread = true;
  rc.on_log = [](const std::string& line) { std::fprintf(stderr, "device: %s\n", line.c_str()); };
  auto dev = std::make_unique<device::RemoteDevice>(device::open_stream(spec), rc);
  dev->connect();
  const device::Info& in = dev->info();
  std::fprintf(stderr,
               "device: %s on %s, %.1f g at the rim, %.3g mm a count, %.2f N at most, "
               "round trip %.2f ms\n",
               in.name, dev->name().c_str(), 1e3 * dev->model_mass(),
               1e3 * dev->encoder_resolution(), dev->max_force(),
               to_millis(dev->connect_round_trip()));
  return dev;
}

// How long after the device samples a position the force computed from it
// acts: the position's way in and, on average, half the State period it
// waited for the host's tick, then the command's way out and the wait for
// the device's tick. The slave's coupling has to be stable over all of it.
Nanos loop_delay_of(const device::RemoteDevice& dev) {
  return dev.connect_round_trip() + dev.state_period() / 2;
}

// How long after it is issued a command takes effect, on the host's clock,
// for a device that could not measure it: the link one way, taken as half
// the least round trip as the clocks' mapping takes it, and the wait for
// the device's next tick, counted whole since a delay taken too short is
// the unsafe side. The positions' age is no part of it: the controllers
// book each at its own time (Device1D::sampled_at). Counting the age in
// as well put the estimate at the top of its range, 4.75 ms, where the
// forces in flight over six samples made the PC's bounds too wide to let
// any force through.
Nanos actuation_delay_of(const device::RemoteDevice& dev) {
  return dev.connect_round_trip() / 2 + Nanos{dev.info().tick_us} * 1000;
}

// The controllers' device model for a device on a link: its mass and
// encoder, and the delay of its actuation as the device measures it
// (Device1D::actuation_delay). Estimated from the readings instead, with
// the above as the first guess, the delay ended 0.29 ms short on the
// slave of device_lab.sh at a 200 us tick and 0.48 ms short at 1 ms (0.67
// and 1.60 ms measured); the master's ended 0.48 and 1.03 ms long.
template <class Config>
void model_device(Config& c, const device::RemoteDevice& dev) {
  if (dev.model_mass() > 0.0) {
    c.model_mass = dev.model_mass();
  } else {
    std::fprintf(stderr, "device: no inertia reported; the controllers assume %.3g kg\n",
                 c.model_mass);
  }
  c.encoder_resolution = dev.encoder_resolution();
  if (const auto measured = dev.actuation_delay()) {
    c.actuation_delay = *measured;
    c.actuation_delay_max = 0;
    return;
  }
  const Nanos delay = actuation_delay_of(dev);
  c.actuation_delay = delay;
  c.actuation_delay_max = std::max(2 * delay, delay + kNanosPerMilli);
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

// Where a session that ran through the slave's relay at some point went
// (RendezvousConfig::direct_check_interval).
void print_route_change(bool started_relayed, bool relayed_at_end, Nanos direct_after) {
  if (direct_after >= 0) {
    std::printf("left the relay for a direct path %.3f s into the session\n",
                static_cast<double>(direct_after) / 1e9);
  } else if (relayed_at_end && !started_relayed) {
    std::printf("ended on the relay\n");
  }
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
  if (c.moved) {
    std::printf(", the other end moved %llu times", static_cast<unsigned long long>(c.moved));
  }
  std::printf("\n");
}

void print_device(const device::RemoteDevice& dev) {
  const device::RemoteStats s = dev.stats();
  std::printf("device %s: %llu States (%llu lost, %llu CRC errors, %llu bad frames), "
              "%llu commands (%llu written short)\n",
              dev.name().c_str(), static_cast<unsigned long long>(s.states),
              static_cast<unsigned long long>(s.states_lost),
              static_cast<unsigned long long>(s.crc_errors),
              static_cast<unsigned long long>(s.bad_frames),
              static_cast<unsigned long long>(s.commands),
              static_cast<unsigned long long>(s.commands_short));
  print_stats("device rtt", net::delay_stats(s.round_trip), 1e6, "ms");
  print_stats("device act", net::delay_stats(s.actuation), 1e6, "ms");
  std::printf("device: watchdog cuts %llu (and %llu between commands), %llu States saturated, "
              "stalest State %.2f ms, actuator work %.6g J\n",
              static_cast<unsigned long long>(s.cuts),
              static_cast<unsigned long long>(s.cuts_between),
              static_cast<unsigned long long>(s.saturated), to_millis(s.worst_state_age),
              dev.actuator_work());
  if (const char* f = dev.fault()) std::printf("device: %s\n", f);
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

// The newest answered hello's time, kept across runs as one decimal number;
// a missing file is a first run.
std::uint64_t load_hello_state(const std::string& path) {
  std::FILE* f = std::fopen(path.c_str(), "r");
  if (!f) {
    if (errno == ENOENT) return 0;
    throw std::runtime_error("cannot open " + path + ": " + std::strerror(errno));
  }
  char text[64];
  const std::size_t n = std::fread(text, 1, sizeof text - 1, f);
  std::fclose(f);
  text[n] = '\0';
  char* end = nullptr;
  errno = 0;
  const unsigned long long v = std::strtoull(text, &end, 10);
  if (text[0] < '0' || text[0] > '9' || errno != 0 || (*end != '\0' && std::strcmp(end, "\n") != 0)) {
    throw std::runtime_error(path + " does not hold a hello time (one decimal number)");
  }
  return v;
}

// Replaces the file so that a crash leaves either the old time or the new
// one, and returns only once the new one is on disk.
void save_hello_state(const std::string& path, std::uint64_t stamp) {
  const std::string tmp = path + ".tmp";
  const std::string text = std::to_string(stamp) + "\n";
  const int fd = ::open(tmp.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0600);
  bool ok = fd >= 0 && ::write(fd, text.data(), text.size()) == static_cast<ssize_t>(text.size()) &&
            ::fsync(fd) == 0;
  const int err = errno;
  if (fd >= 0) ::close(fd);
  ok = ok && std::rename(tmp.c_str(), path.c_str()) == 0;
  if (!ok) throw std::runtime_error("cannot write " + path + ": " + std::strerror(ok ? errno : err));
  // The rename is durable once its directory is.
  const auto slash = path.find_last_of('/');
  const std::string dir = slash == std::string::npos ? "." : path.substr(0, slash ? slash : 1);
  const int dfd = ::open(dir.c_str(), O_RDONLY);
  if (dfd >= 0) {
    ::fsync(dfd);
    ::close(dfd);
  }
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
    throw std::runtime_error("--prime-hops must be auto or 0..255");
  }
  teleop::RendezvousConfig rc;
  rc.server = *server;
  rc.id = crypto::rendezvous_id(*psk);
  rc.prime_hops = o.prime_hops;
  rc.measure_hops = o.prime_auto;
  return rc;
}

// The relay of --turn, reached from `sock`. The password comes from a file
// rather than the command line, which other users can read in the process
// list.
std::optional<teleop::RelayConfig> relay_of(const Options& o, const net::UdpSocket& sock) {
  if (o.turn.empty()) {
    if (!o.turn_user.empty() || !o.turn_pass_file.empty()) {
      throw std::runtime_error("--turn-user and --turn-pass-file need --turn");
    }
    return std::nullopt;
  }
  if (o.rendezvous.empty()) {
    throw std::runtime_error("--turn needs --rendezvous, through which the relay is announced");
  }
  if (o.turn_user.empty() || o.turn_pass_file.empty()) {
    throw std::runtime_error("--turn needs --turn-user and --turn-pass-file");
  }
  const auto [host, port] = split_host_port(o.turn, kDefaultTurnPort);
  const auto server = net::Endpoint::resolve(host, port, o.ipv6);
  if (!server) throw std::runtime_error("cannot resolve TURN server " + host);
  if (!server->for_family(sock.is_ipv6())) {
    throw std::runtime_error("the TURN server " + server->to_string() +
                             " is IPv6, and this host has no IPv6 socket");
  }
  std::FILE* f = std::fopen(o.turn_pass_file.c_str(), "r");
  if (!f) throw std::runtime_error("cannot open " + o.turn_pass_file);
  char text[512];
  const std::size_t n = std::fread(text, 1, sizeof text - 1, f);
  std::fclose(f);
  std::string password(text, n);
  password = password.substr(0, password.find_first_of("\r\n"));
  if (password.empty()) throw std::runtime_error(o.turn_pass_file + " holds no password");
  struct stat st;
  if (::stat(o.turn_pass_file.c_str(), &st) == 0 && (st.st_mode & 077) != 0) {
    std::fprintf(stderr, "note: %s is readable by other users (chmod 600 it)\n",
                 o.turn_pass_file.c_str());
  }
  return teleop::RelayConfig{*server, o.turn_user, password};
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

  std::unique_ptr<device::RemoteDevice> remote;
  std::unique_ptr<teleop::SimulatedMaster> simulated;
  cfg.control.model_mass = 0.2;
  if (!o.device.empty()) {
    remote = open_device(o, period, cfg.control.pc_hold_samples);
    model_device(cfg.control, *remote);
    cfg.control.max_force = std::min(o.max_force, remote->max_force());
  }
  const Nanos start = monotonic_now();
  if (!remote) {
    teleop::OperatorParams op;
    simulated = std::make_unique<teleop::SimulatedMaster>(0.2, 0.5, op, period, 10, start);
    simulated->set_watchdog(watchdog_of(o, period, cfg.control.pc_hold_samples));
  }
  teleop::Device1D& device = remote ? static_cast<teleop::Device1D&>(*remote) : *simulated;
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
    if (rep.prime_hops > 0) {
      std::printf("rendezvous: priming with hop limit %d (%s)\n", rep.prime_hops,
                  rep.prime_how.c_str());
    } else {
      std::printf("rendezvous: not priming (%s)\n", rep.prime_how.c_str());
    }
    if (rep.observed.valid()) {
      std::printf("rendezvous: %s sees this end at %s\n",
                  cfg.rendezvous->server.to_string().c_str(), rep.observed.to_string().c_str());
    } else {
      std::printf("rendezvous: no answer from %s\n", cfg.rendezvous->server.to_string().c_str());
    }
    if (!rep.candidates.empty()) {
      std::printf("rendezvous: the slave at %s\n", join(rep.candidates).c_str());
    }
    if (!rep.relays.empty()) {
      std::printf("rendezvous: the slave's relay at %s\n", join(rep.relays).c_str());
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
    if (!rep.candidates.empty() && rep.relays.empty()) {
      std::printf(" Or a NAT on the way gives every destination its own port, which hole "
                  "punching cannot cross: give the slave a relay with --turn; see the README");
    }
    std::printf(")\n");
    return 2;
  }
  if (cfg.rendezvous) {
    std::printf("reached the slave at %s%s\n", rep.slave.canonical().to_string().c_str(),
                rep.relayed ? ", its relay" : "");
  }
  if (rep.authenticated) std::printf("authenticated (%s)\n", crypto::NoiseNNpsk0::kProtocolName);

  std::printf("session %08x, %llu ticks (%llu skipped), master PC active %.1f %%, "
              "actuation delay taken %.3f ms\n",
              rep.session, static_cast<unsigned long long>(rep.ticks),
              static_cast<unsigned long long>(rep.skipped), 100.0 * rep.master_pc_active,
              to_millis(rep.actuation_delay));
  if (rep.ended_by_slave) std::printf("the slave ended the session\n");
  if (!rep.device_fault.empty()) std::printf("ended by the device: %s\n", rep.device_fault.c_str());
  if (rep.rejoins) {
    std::printf("rendezvous: the slave announced anew %llu times during the session\n",
                static_cast<unsigned long long>(rep.rejoins));
  }
  print_route_change(rep.relayed, rep.relayed_at_end, rep.direct_after);
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
  if (remote) print_device(*remote);
  return rep.device_fault.empty() ? 0 : 3;
}

int run_slave(const Options& o) {
  if (!o.rendezvous.empty() && o.key_file.empty()) {
    throw std::runtime_error("--rendezvous needs --key-file");
  }
  if (o.contact_force && o.scheme == teleop::Passivity::kReflective && !o.adopt) {
    throw std::runtime_error(
        "--contact-force needs --scheme channel or none: reflect needs the coupling force");
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
  if (!o.hello_state.empty()) {
    if (!cfg.psk) throw std::runtime_error("--hello-state needs --key-file");
    cfg.newest_hello = load_hello_state(o.hello_state);
  }
  cfg.rendezvous = rendezvous_of(o, cfg.psk, sock);
  cfg.relay = relay_of(o, sock);

  std::unique_ptr<device::RemoteDevice> remote;
  std::unique_ptr<teleop::SimulatedSlave> simulated;
  if (!o.device.empty()) {
    remote = open_device(o, period, cfg.control.pc_hold_samples);
    model_device(cfg.control, *remote);
    // A sampled spring K is passive on a device with damping b when
    // b > K T / 2 + |B| (Colgate and Schenkel), T the period and B the
    // spring's own damping. Here the force also waits for the link and
    // the State's age, counted with the half period as the hold's delay
    // is; half that stiffness leaves a margin. B costs as much of b as it
    // adds, so the coupling has none unless asked: the device damps on
    // its own ticks, where no link delays it.
    const double b = remote->damping() / (remote->radius() * remote->radius());
    const double horizon = to_seconds(period / 2 + loop_delay_of(*remote));
    if (o.kp >= 0.0) {
      cfg.control.kp = o.kp;
    } else if (b > 0.0) {
      cfg.control.kp = 0.5 * b / horizon;
    } else {
      throw std::runtime_error("a device without damping needs --kp");
    }
    cfg.control.kd = o.kd < 0.0 ? 0.0 : o.kd;
    std::fprintf(stderr,
                 "coupling %.0f N/m and %.3g N s/m (the device damps %.3g N s/m at the rim, the "
                 "loop's delay %.2f ms)\n",
                 cfg.control.kp, cfg.control.kd, b, 1e3 * horizon);
  } else {
    if (o.kp >= 0.0) cfg.control.kp = o.kp;
    if (o.kd >= 0.0) cfg.control.kd = o.kd;
    teleop::WallParams wall;
    wall.k = o.wall_k;
    simulated =
        std::make_unique<teleop::SimulatedSlave>(1.0, 2.0, wall, period, 10, monotonic_now(), 0.0);
    simulated->set_watchdog(watchdog_of(o, period, cfg.control.pc_hold_samples));
  }
  teleop::Device1D& device = remote ? static_cast<teleop::Device1D&>(*remote) : *simulated;

  teleop::SlaveNodeHooks hooks;
  std::uint64_t seed = 2;
  if (emulating(o)) {
    hooks.emulation = [&](Nanos start) { return make_emulation(o, start, seed++); };
  }
  hooks.on_refused = [](const std::string& why) { std::fprintf(stderr, "%s\n", why.c_str()); };
  if (!o.hello_state.empty()) {
    hooks.on_hello_answered = [&](std::uint64_t stamp) { save_hello_state(o.hello_state, stamp); };
  }
  hooks.on_rendezvous = [](const std::string& what) {
    std::fprintf(stderr, "rendezvous: %s\n", what.c_str());
  };
  hooks.on_session_end = [](const teleop::SlaveSessionReport& r) {
    std::printf("session %08x from %s%s, scheme %s, %s, ended by %s\n", r.session,
                r.master.canonical().to_string().c_str(),
                r.relayed ? " through the relay" : "", scheme_name(r.scheme),
                r.authenticated ? "authenticated" : "not authenticated",
                !r.device_fault.empty() ? r.device_fault.c_str()
                : r.ended_by_final      ? "the master"
                                        : "timeout or stop");
    std::printf("%llu ticks (%llu skipped), slave PC active %.1f %%, actuation delay taken %.3f ms\n",
                static_cast<unsigned long long>(r.ticks), static_cast<unsigned long long>(r.skipped),
                100.0 * r.slave_pc_active, to_millis(r.actuation_delay));
    if (r.rejoins) {
      std::printf("rendezvous: the master announced anew %llu times during the session\n",
                  static_cast<unsigned long long>(r.rejoins));
    }
    print_route_change(r.relayed, r.relayed_at_end, r.direct_after);
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
  if (remote) {
    print_device(*remote);
    if (remote->fault()) return 3;
  }
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
      else if (a == "--prime-hops") {
        const std::string v = next();
        o.prime_auto = v == "auto";
        if (!o.prime_auto) o.prime_hops = static_cast<int>(parse_double(v, "hops"));
      }
      else if (a == "--duration") o.duration = parse_double(next(), "duration");
      else if (a == "--max-force") o.max_force = parse_double(next(), "max-force");
      else if (a == "--bind") o.bind = next();
      else if (a == "--once") o.once = true;
      else if (a == "--adopt-scheme") o.adopt = true;
      else if (a == "--initial-energy") o.initial_energy = parse_double(next(), "initial-energy");
      else if (a == "--hold-after") o.hold_after_ms = parse_double(next(), "hold-after");
      else if (a == "--contact-force") o.contact_force = true;
      else if (a == "--wall-k") o.wall_k = parse_double(next(), "wall-k");
      else if (a == "--kp") o.kp = parse_double(next(), "kp");
      else if (a == "--kd") o.kd = parse_double(next(), "kd");
      else if (a == "--device") o.device = next();
      else if (a == "--hello-state") o.hello_state = next();
      else if (a == "--turn") o.turn = next();
      else if (a == "--turn-user") o.turn_user = next();
      else if (a == "--turn-pass-file") o.turn_pass_file = next();
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
    if (o.mode != "slave" && !o.turn.empty()) {
      throw std::runtime_error("--turn is for the slave; the master reaches its relay");
    }
    if (o.mode != "slave" && (o.kp >= 0.0 || o.kd >= 0.0)) {
      throw std::runtime_error("--kp and --kd are the slave's coupling");
    }
    if (o.mode != "slave" && !o.hello_state.empty()) {
      throw std::runtime_error("--hello-state is for the slave, which answers hellos");
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
