// tuushin-probe: measures a real network path the way haptic traffic would
// use it, and writes traces that tuushin-sim replays.
//
//   on a host the other side can reach (a VPS, a friend's machine):
//     tuushin-probe reflect --port 47000
//   then, from the operator side:
//     tuushin-probe measure HOST --port 47000 --rate 1000 --duration 60 --out home-vps
//     tuushin-sim --trace-fwd home-vps.fwd.csv --trace-bwd home-vps.bwd.csv --passivity reflect
//
// The reflector answers each probe with one echo of the same size, so it can
// never amplify traffic, and it limits its total echo rate (--max-pps).
//
// measure splits each round trip with a clock model that assumes equal
// minimum delays both ways. pair needs no such assumption: on one host with
// two networks (Wi-Fi and a phone's mobile data, say) it sends probes from
// one interface to the other across the internet and back, and both ends
// read the same clock:
//
//     tuushin-probe pair --a en0 --b en7 --stun stun.l.google.com:19302 --out wifi-lte

#include <algorithm>
#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fstream>
#include <optional>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include "tuushin/net/probe.hpp"
#include "tuushin/net/stun.hpp"
#include "tuushin/net/udp.hpp"
#include "tuushin/rt/thread.hpp"
#include "tuushin/time.hpp"
#include "tuushin/wire/packet.hpp"

using namespace tuushin;

namespace {

volatile std::sig_atomic_t g_stop = 0;

void on_signal(int) { g_stop = 1; }

void usage() {
  std::fprintf(stderr,
               "usage:\n"
               "  tuushin-probe reflect [options]\n"
               "      --port P        UDP port to listen on (47000)\n"
               "      --bind HOST     local address (all)\n"
               "      --max-pps N     total echo rate limit (4000)\n"
               "      --dscp D        DSCP of echoes (46, Expedited Forwarding)\n"
               "  tuushin-probe measure HOST [options]\n"
               "      --port P        reflector port (47000)\n"
               "      --rate HZ       probes per second (1000)\n"
               "      --duration S    seconds of probing (60)\n"
               "      --size B        UDP payload bytes per probe, %zu..%zu (100: a\n"
               "                      3-axis haptic sample with two energies)\n"
               "      --dscp D        DSCP of probes (46)\n"
               "      --ipv6          prefer IPv6 when HOST has both\n"
               "      --out PREFIX    write PREFIX.fwd.csv, PREFIX.bwd.csv, PREFIX.raw.csv\n"
               "  tuushin-probe pair --a IF --b IF --stun HOST[:PORT] [options]\n"
               "      probes between two interfaces of this host, on two networks, each\n"
               "      way across the internet, on one clock (IPv4)\n"
               "      --a IF, --b IF  the interfaces (en0, wlan0, ...)\n"
               "      --stun H[:P]    a STUN server, to find each end's public address\n"
               "                      (port 3478)\n"
               "      --rate, --duration, --size, --dscp  as for measure\n"
               "      --out PREFIX    write PREFIX.ab.csv (a to b) and PREFIX.ba.csv\n",
               wire::kHeaderSize + wire::kReceiptMapSize, wire::kMaxDatagram);
}

double parse_double(const std::string& s, const char* what) {
  char* end = nullptr;
  const double v = std::strtod(s.c_str(), &end);
  if (end == s.c_str() || *end != '\0') throw std::runtime_error(std::string("bad ") + what + ": " + s);
  return v;
}

std::string utc_now() {
  const std::time_t t = std::time(nullptr);
  std::tm tm{};
  gmtime_r(&t, &tm);
  char buf[32];
  std::strftime(buf, sizeof buf, "%Y-%m-%dT%H:%M:%SZ", &tm);
  return buf;
}

// ------------------------------------------------------------------ reflect

struct ReflectOptions {
  std::string bind;
  std::uint16_t port = 47000;
  double max_pps = 4000;
  int dscp = 46;
};

int run_reflect(const ReflectOptions& o) {
  const auto local = net::Endpoint::resolve(o.bind, o.port);
  if (!local) throw std::runtime_error("cannot resolve bind address '" + o.bind + "'");
  net::UdpSocket sock(*local);
  if (!sock.set_dscp(o.dscp)) std::fprintf(stderr, "note: DSCP %d refused by the system\n", o.dscp);
  const bool kernel_ts = sock.enable_rx_timestamps();
  const std::string sched = rt::prefer_low_latency();
  std::fprintf(stderr, "reflecting on %s (%s, %s receive stamps)\n",
               sock.local_endpoint().to_string().c_str(), sched.c_str(),
               kernel_ts ? "kernel" : "user-space");

  net::ProbeReflector reflector;
  std::vector<std::uint8_t> in(wire::kMaxDatagram + 1), out;
  // Token bucket over all sessions: a reflector reachable from the internet
  // must not become a free traffic source for spoofed requests.
  double tokens = o.max_pps;
  Nanos last_refill = monotonic_now();
  std::uint64_t echoed = 0, dropped_rate = 0, dropped_bad = 0, refused = 0;
  Nanos next_report = monotonic_now() + 10 * kNanosPerSecond;
  std::uint64_t echoed_at_report = 0;
  while (!g_stop) {
    sock.wait_readable(200 * kNanosPerMilli);
    for (;;) {
      net::Endpoint from;
      Nanos rx = 0;
      const auto n = sock.recv_from(in.data(), in.size(), &from, &rx);
      if (!n) break;
      const auto h = wire::decode_header(in.data(), *n);
      if (!h || h->type != wire::PacketType::kProbe) {
        ++dropped_bad;
        continue;
      }
      const Nanos now = monotonic_now();
      tokens = std::min(o.max_pps, tokens + o.max_pps * to_seconds(now - last_refill));
      last_refill = now;
      if (tokens < 1.0) {
        ++dropped_rate;
        continue;
      }
      tokens -= 1.0;
      const auto echo = reflector.on_probe(*h, rx, monotonic_now());
      if (!echo) {
        ++refused;
        continue;
      }
      wire::encode_probe_echo(echo->header, echo->receipts, echo->payload, out);
      if (sock.send_to(from, out.data(), out.size())) ++echoed;
    }
    const Nanos now = monotonic_now();
    if (now >= next_report) {
      std::fprintf(stderr, "%s  sessions %zu  echo rate %.0f/s  total %llu  rate-limited %llu  invalid %llu\n",
                   utc_now().c_str(), reflector.sessions(),
                   static_cast<double>(echoed - echoed_at_report) / 10.0,
                   static_cast<unsigned long long>(echoed), static_cast<unsigned long long>(dropped_rate),
                   static_cast<unsigned long long>(dropped_bad));
      echoed_at_report = echoed;
      next_report = now + 10 * kNanosPerSecond;
    }
  }
  std::fprintf(stderr,
               "stopped after %llu echoes; dropped %llu over the rate limit, %llu invalid, %llu with "
               "the session table full\n",
               static_cast<unsigned long long>(echoed), static_cast<unsigned long long>(dropped_rate),
               static_cast<unsigned long long>(dropped_bad), static_cast<unsigned long long>(refused));
  return 0;
}

// ------------------------------------------------------------------ measure

struct MeasureOptions {
  std::string host;
  std::uint16_t port = 47000;
  double rate = 1000;
  double duration = 60;
  std::size_t size = 100;
  int dscp = 46;
  bool ipv6 = false;
  std::string out;
};

void print_delay(const char* name, const net::DelayStats& d) {
  if (d.count == 0) {
    std::printf("%s  no samples\n", name);
    return;
  }
  std::printf("%s  min %8.3f  p50 %8.3f  p90 %8.3f  p99 %8.3f  p99.9 %8.3f  max %8.3f  jitter %.3f\n",
              name, d.min / 1e6, d.p50 / 1e6, d.p90 / 1e6, d.p99 / 1e6, d.p999 / 1e6, d.max / 1e6,
              d.rfc3550_jitter / 1e6);
}

void print_direction(const char* name, const net::DirectionReport& r) {
  std::printf("%s  sent %zu  lost %zu (%.3f%%)  longest burst %zu  reordered %zu\n", name, r.sent,
              r.lost, 100.0 * r.loss_rate(), r.longest_burst, r.reordered);
}

void write_raw(const std::string& path, const net::ProbeClient& c) {
  std::ofstream f(path);
  if (!f) throw std::runtime_error("cannot write " + path);
  f << "# t1, t4: client monotonic clock; t2, t3: reflector clock; ns\n";
  f << "seq,t1,t2,t3,t4,reflector_seq,echoed,reflector_received,duplicates\n";
  const auto& recs = c.records();
  for (std::size_t k = 0; k < recs.size(); ++k) {
    const auto& r = recs[k];
    f << (k + 1) << ',' << r.t1 << ',' << r.t2 << ',' << r.t3 << ',' << r.t4 << ','
      << r.reflector_seq << ',' << (r.echoed ? 1 : 0) << ',' << (r.reflector_received ? 1 : 0)
      << ',' << r.duplicates << '\n';
  }
}

int run_measure(const MeasureOptions& o) {
  if (o.size < wire::kHeaderSize + wire::kReceiptMapSize || o.size > wire::kMaxDatagram) {
    throw std::runtime_error("--size out of range");
  }
  if (o.rate <= 0 || o.rate > 20000) throw std::runtime_error("--rate must be in (0, 20000]");
  const auto peer = net::Endpoint::resolve(o.host, o.port, o.ipv6);
  if (!peer) throw std::runtime_error("cannot resolve " + o.host);
  const auto local = net::Endpoint::resolve(peer->is_ipv6() ? "::" : "0.0.0.0", 0);
  net::UdpSocket sock(*local);
  if (!sock.set_dscp(o.dscp)) std::fprintf(stderr, "note: DSCP %d refused by the system\n", o.dscp);
  const bool kernel_ts = sock.enable_rx_timestamps();
  const Nanos interval = static_cast<Nanos>(1e9 / o.rate);
  const std::string sched = rt::prefer_low_latency(interval);

  std::random_device rd;
  const std::uint32_t session = (static_cast<std::uint32_t>(rd()) << 1) | 1u;  // never 0
  net::ProbeClient client(session);
  const std::size_t padding = o.size - wire::kHeaderSize;
  std::fprintf(stderr, "probing %s at %.0f Hz for %.0f s, %zu bytes (%s, %s receive stamps)\n",
               peer->to_string().c_str(), o.rate, o.duration, o.size, sched.c_str(),
               kernel_ts ? "kernel" : "user-space");

  std::vector<std::uint8_t> in(wire::kMaxDatagram + 1), out;

  // Warm-up under a separate session: the first packets of a flow create
  // NAT and firewall state and may meet a reflector that is still starting,
  // so their fate says nothing about the path. Measure only once an echo
  // has come back.
  {
    net::ProbeClient warm(session ^ 0x80000000u);
    const Nanos give_up = monotonic_now() + 5 * kNanosPerSecond;
    Nanos next_warm = monotonic_now();
    bool answered = false;
    while (!answered && !g_stop) {
      Nanos now = monotonic_now();
      if (now >= give_up) break;
      if (now >= next_warm) {
        wire::encode_padded(warm.next_probe(now), padding, out);
        sock.send_to(*peer, out.data(), out.size());
        next_warm = now + 100 * kNanosPerMilli;
      }
      now = monotonic_now();
      sock.wait_readable(std::min(next_warm, give_up) - now);
      for (;;) {
        Nanos rx = 0;
        const auto n = sock.recv_from(in.data(), in.size(), nullptr, &rx);
        if (!n) break;
        if (warm.on_datagram(in.data(), *n, rx)) answered = true;
      }
    }
    if (!answered) {
      std::fprintf(stderr,
                   "no echo from %s within 5 s: is the reflector running, and is UDP port %u "
                   "open on its side?\n",
                   peer->to_string().c_str(), static_cast<unsigned>(o.port));
      return 2;
    }
    for (const auto& r : warm.records()) {
      if (r.echoed) {
        std::fprintf(stderr, "reflector answers, rtt %.3f ms; measuring\n",
                     to_millis((r.t4 - r.t1) - (r.t3 - r.t2)));
        break;
      }
    }
  }

  const Nanos start = monotonic_now();
  const std::string started = utc_now();
  const Nanos send_end = start + from_seconds(o.duration);
  // Echoes still in flight when sending stops: wait a generous RTT.
  const Nanos grace = 2 * kNanosPerSecond;
  Nanos next = start;
  Nanos stop_at = send_end;
  bool sending = true;
  std::vector<Nanos> lateness;  // pacing error of each send
  lateness.reserve(static_cast<std::size_t>(o.rate * o.duration) + 16);
  std::size_t send_failures = 0, skipped = 0;
  Nanos next_report = start + kNanosPerSecond;
  std::size_t echoes_at_report = 0, sent_at_report = 0;

  for (;;) {
    Nanos now = monotonic_now();
    if (sending && (g_stop || now >= send_end)) {
      sending = false;
      stop_at = now + grace;
      if (g_stop) std::fprintf(stderr, "interrupted; collecting late echoes\n");
    }
    if (!sending && now >= stop_at) break;
    if (sending && now >= next) {
      // More than ten intervals behind (the process was descheduled):
      // resume the schedule from now instead of sending a burst.
      if (now - next > 10 * interval) {
        const Nanos missed = (now - next) / interval;
        skipped += static_cast<std::size_t>(missed);
        next += missed * interval;
      }
      const Nanos t1 = monotonic_now();
      lateness.push_back(t1 - next);
      wire::encode_padded(client.next_probe(t1), padding, out);
      if (!sock.send_to(*peer, out.data(), out.size())) ++send_failures;
      next += interval;
      now = monotonic_now();
    }
    const Nanos deadline = sending ? std::min(next, send_end) : stop_at;
    if (deadline > now) sock.wait_readable(deadline - now);
    for (;;) {
      Nanos rx = 0;
      const auto n = sock.recv_from(in.data(), in.size(), nullptr, &rx);
      if (!n) break;
      client.on_datagram(in.data(), *n, rx);
    }
    now = monotonic_now();
    if (now >= next_report) {
      const std::size_t sent = client.sent(), echoes = client.echoes();
      std::fprintf(stderr, "  %5.1f s  sent %zu  echoes %zu  (last second: %zu/%zu)\n",
                   to_seconds(now - start), sent, echoes, echoes - echoes_at_report,
                   sent - sent_at_report);
      echoes_at_report = echoes;
      sent_at_report = sent;
      next_report += kNanosPerSecond;
    }
  }

  const net::ProbeReport rep = net::analyze_probes(client);
  std::printf("path %s, %zu-byte probes at %.0f Hz, started %s\n", peer->to_string().c_str(), o.size,
              o.rate, started.c_str());
  std::printf("probes %zu  echoes %zu  duplicates %zu  stray %zu  send failures %zu  skipped %zu\n",
              rep.probes, client.echoes(), rep.duplicates, client.stray(), send_failures, skipped);
  print_delay("rtt  ms", rep.rtt);
  print_delay("fwd  ms", rep.fwd.delay);
  print_delay("bwd  ms", rep.bwd.delay);
  print_direction("fwd ", rep.fwd);
  print_direction("bwd ", rep.bwd);
  std::printf("loss direction ambiguous %zu, forward delays interpolated %zu\n", rep.ambiguous,
              rep.interpolated);
  std::printf("clock skew %.3f ppm; the fwd/bwd split assumes equal minimum delays\n",
              rep.clock.skew * 1e6);
  if (!lateness.empty()) {
    std::sort(lateness.begin(), lateness.end());
    const auto q = [&](double p) {
      return to_millis(lateness[std::min(lateness.size() - 1,
                                         static_cast<std::size_t>(p * static_cast<double>(lateness.size())))]);
    };
    std::printf("pacing lateness ms: p50 %.3f  p99 %.3f  max %.3f\n", q(0.5), q(0.99),
                to_millis(lateness.back()));
  }

  if (!o.out.empty()) {
    char meta[256];
    std::snprintf(meta, sizeof meta, "tuushin-probe measure %s, %zu bytes at %.0f Hz, %.0f s, started %s",
                  o.host.c_str(), o.size, o.rate, o.duration, started.c_str());
    net::Trace fwd = rep.fwd_trace, bwd = rep.bwd_trace;
    fwd.comments.insert(fwd.comments.begin(), std::string(meta) + ", client -> reflector");
    bwd.comments.insert(bwd.comments.begin(), std::string(meta) + ", reflector -> client");
    fwd.save_csv(o.out + ".fwd.csv");
    bwd.save_csv(o.out + ".bwd.csv");
    write_raw(o.out + ".raw.csv", client);
    std::printf("wrote %s.fwd.csv, %s.bwd.csv, %s.raw.csv\n", o.out.c_str(), o.out.c_str(),
                o.out.c_str());
  }
  return client.echoes() > 0 ? 0 : 2;
}

// --------------------------------------------------------------------- pair

struct PairOptions {
  std::string a, b;  // interfaces
  std::string stun;  // HOST[:PORT]
  double rate = 1000;
  double duration = 60;
  std::size_t size = 100;
  int dscp = 46;
  std::string out;
};

// One end of a pair: a socket tied to one interface.
struct PairEnd {
  std::string dev;
  net::UdpSocket sock;
  net::Endpoint mapped;  // where the STUN server saw it
  net::Endpoint peer;    // where the other end's datagrams come from
};

net::UdpSocket open_on(const std::string& dev) {
  const auto addr = net::interface_address(dev);
  if (!addr) throw std::runtime_error("interface " + dev + " has no IPv4 address (is it up?)");
  net::UdpSocket s(*addr);
  if (!s.bind_to_interface(dev)) {
    std::string why = std::strerror(errno);
#if defined(__linux__)
    why += "; before Linux 5.7 this needs root or CAP_NET_RAW";
#endif
    throw std::runtime_error("cannot tie a socket to " + dev + ": " + why);
  }
  return s;
}

// HOST[:PORT], IPv4: pair's sockets are.
net::Endpoint resolve_ipv4(const std::string& s, std::uint16_t default_port) {
  std::string host = s;
  std::uint16_t port = default_port;
  const auto colon = s.rfind(':');
  if (colon != std::string::npos) {
    host = s.substr(0, colon);
    const double p = parse_double(s.substr(colon + 1), "port");
    if (p < 1 || p > 65535) throw std::runtime_error("bad port in " + s);
    port = static_cast<std::uint16_t>(p);
  }
  const auto e = net::Endpoint::resolve(host, port);
  if (!e || e->is_ipv6()) throw std::runtime_error("cannot resolve " + host + " to an IPv4 address");
  return *e;
}

// The address a STUN server (RFC 8489) sees `sock`'s datagrams come from,
// asking every 200 ms; nullopt after 3 s without an answer.
std::optional<net::Endpoint> stun_mapped(net::UdpSocket& sock, const net::Endpoint& server) {
  namespace stun = net::stun;
  const stun::TransactionId tid = stun::random_transaction_id();
  const std::vector<std::uint8_t> request =
      stun::Builder(stun::method::kBinding, stun::Class::kRequest, tid).take();
  std::vector<std::uint8_t> in(2048);
  const Nanos give_up = monotonic_now() + 3 * kNanosPerSecond;
  Nanos next = 0;
  while (!g_stop) {
    const Nanos now = monotonic_now();
    if (now >= give_up) break;
    if (now >= next) {
      sock.send_to(server, request.data(), request.size());
      next = now + 200 * kNanosPerMilli;
    }
    sock.wait_readable(std::min(next, give_up) - now);
    for (;;) {
      const auto n = sock.recv_from(in.data(), in.size(), nullptr);
      if (!n) break;
      const auto m = stun::Message::parse(in.data(), *n);
      if (!m || m->method() != stun::method::kBinding || m->cls() != stun::Class::kSuccess ||
          m->transaction() != tid) {
        continue;
      }
      if (auto a = m->address(stun::attr::kXorMappedAddress)) return a;
      if (auto a = m->address(stun::attr::kMappedAddress)) return a;  // RFC 3489 servers
    }
  }
  return std::nullopt;
}

// Opens the way between the ends: each sends to the other's public address
// every 20 ms until both have heard from the other, or 5 s pass. A NAT that
// maps by destination gives the other end another port than the STUN
// server saw; the datagrams that do arrive tell where an end really is (a
// peer-reflexive address, in ICE's terms), and the other answers there.
bool open_pair(PairEnd& a, PairEnd& b, net::OneClockDirection& hello_ab,
               net::OneClockDirection& hello_ba, std::size_t padding) {
  a.peer = b.mapped;
  b.peer = a.mapped;
  std::vector<std::uint8_t> in(wire::kMaxDatagram + 1), out;
  net::UdpSocket* const both[2] = {&a.sock, &b.sock};
  bool a_heard = false, b_heard = false;
  const Nanos give_up = monotonic_now() + 5 * kNanosPerSecond;
  Nanos next = 0;
  while (!(a_heard && b_heard) && !g_stop) {
    Nanos now = monotonic_now();
    if (now >= give_up) return false;
    if (now >= next) {
      wire::encode_padded(hello_ab.next(now), padding, out);
      a.sock.send_to(a.peer, out.data(), out.size());
      wire::encode_padded(hello_ba.next(monotonic_now()), padding, out);
      b.sock.send_to(b.peer, out.data(), out.size());
      next = now + 20 * kNanosPerMilli;
    }
    now = monotonic_now();
    net::wait_any_readable(both, 2, std::min(next, give_up) - now);
    for (;;) {
      net::Endpoint from;
      const auto n = a.sock.recv_from(in.data(), in.size(), &from);
      if (!n) break;
      if (hello_ba.on_datagram(in.data(), *n, monotonic_now())) {
        a.peer = from;
        a_heard = true;
      }
    }
    for (;;) {
      net::Endpoint from;
      const auto n = b.sock.recv_from(in.data(), in.size(), &from);
      if (!n) break;
      if (hello_ab.on_datagram(in.data(), *n, monotonic_now())) {
        b.peer = from;
        b_heard = true;
      }
    }
  }
  return a_heard && b_heard;
}

int run_pair(const PairOptions& o) {
  if (o.size < wire::kHeaderSize || o.size > wire::kMaxDatagram) {
    throw std::runtime_error("--size out of range");
  }
  if (o.rate <= 0 || o.rate > 20000) throw std::runtime_error("--rate must be in (0, 20000]");
  if (o.a == o.b) throw std::runtime_error("--a and --b name the same interface");
  const net::Endpoint server = resolve_ipv4(o.stun, 3478);
  PairEnd a{o.a, open_on(o.a), {}, {}};
  PairEnd b{o.b, open_on(o.b), {}, {}};
  for (PairEnd* e : {&a, &b}) {
    if (!e->sock.set_dscp(o.dscp)) std::fprintf(stderr, "note: DSCP %d refused on %s\n", o.dscp, e->dev.c_str());
    if (!e->sock.enable_rx_timestamps()) {
      std::fprintf(stderr, "note: no kernel receive stamps on %s; reading time counts as delay\n",
                   e->dev.c_str());
    }
    const auto m = stun_mapped(e->sock, server);
    if (!m) {
      std::fprintf(stderr,
                   "no answer from the STUN server %s through %s within 3 s: does %s reach the "
                   "internet, and does the server answer UDP?\n",
                   server.to_string().c_str(), e->dev.c_str(), e->dev.c_str());
      return 2;
    }
    e->mapped = *m;
    std::fprintf(stderr, "%s: %s, seen from outside as %s\n", e->dev.c_str(),
                 e->sock.local_endpoint().to_string().c_str(), m->to_string().c_str());
  }
  if (a.mapped.address() == b.mapped.address()) {
    std::fprintf(stderr,
                 "%s and %s reach the internet through the same public address: they are one "
                 "network, or share its last router, so a datagram between them may turn round "
                 "inside that router without crossing the internet. Use two networks, such as "
                 "Wi-Fi and a phone's mobile data.\n",
                 o.a.c_str(), o.b.c_str());
    return 2;
  }

  std::random_device rd;
  const std::uint32_t session = (static_cast<std::uint32_t>(rd()) << 2) | 1u;  // never 0
  net::OneClockDirection ab(session), ba(session ^ 2u);
  net::OneClockDirection hello_ab(session ^ 0x80000000u), hello_ba(session ^ 0x80000002u);
  const std::size_t padding = o.size - wire::kHeaderSize;
  if (!open_pair(a, b, hello_ab, hello_ba, padding)) {
    std::fprintf(stderr,
                 "the two ends did not reach each other within 5 s (%s heard %zu of %zu, %s "
                 "heard %zu of %zu). A NAT that gives each destination a new port "
                 "(symmetric), facing one that lets in only replies from the address and port "
                 "it sent to, leaves no way through; another pair of networks may do better.\n",
                 o.b.c_str(), hello_ab.received(), hello_ab.sent(), o.a.c_str(),
                 hello_ba.received(), hello_ba.sent());
    return 2;
  }
  std::fprintf(stderr, "the ends reach each other (%s heard %s after %zu hellos)\n", o.b.c_str(),
               o.a.c_str(), hello_ab.sent());

  const Nanos interval = static_cast<Nanos>(1e9 / o.rate);
  const std::string sched = rt::prefer_low_latency(interval);
  std::fprintf(stderr, "probing %s <-> %s at %.0f Hz for %.0f s, %zu bytes (%s)\n", o.a.c_str(),
               o.b.c_str(), o.rate, o.duration, o.size, sched.c_str());
  std::vector<std::uint8_t> in(wire::kMaxDatagram + 1), out;
  net::UdpSocket* const both[2] = {&a.sock, &b.sock};
  const Nanos start = monotonic_now();
  const std::string started = utc_now();
  const Nanos send_end = start + from_seconds(o.duration);
  const Nanos grace = 2 * kNanosPerSecond;
  Nanos next = start, stop_at = send_end;
  bool sending = true;
  std::size_t send_failures = 0, skipped = 0;
  Nanos next_report = start + kNanosPerSecond;
  // Datagrams of the other session on a socket: late hellos.
  const auto drain = [&](PairEnd& e, net::OneClockDirection& dir, std::uint32_t hello) {
    for (;;) {
      Nanos rx = 0;
      const auto n = e.sock.recv_from(in.data(), in.size(), nullptr, &rx);
      if (!n) break;
      const auto h = wire::decode_header(in.data(), *n);
      if (h && h->session == hello) continue;
      dir.on_datagram(in.data(), *n, rx);
    }
  };
  for (;;) {
    Nanos now = monotonic_now();
    if (sending && (g_stop || now >= send_end)) {
      sending = false;
      stop_at = now + grace;
      if (g_stop) std::fprintf(stderr, "interrupted; collecting late probes\n");
    }
    if (!sending && now >= stop_at) break;
    if (sending && now >= next) {
      if (now - next > 10 * interval) {
        const Nanos missed = (now - next) / interval;
        skipped += static_cast<std::size_t>(missed);
        next += missed * interval;
      }
      wire::encode_padded(ab.next(monotonic_now()), padding, out);
      if (!a.sock.send_to(a.peer, out.data(), out.size())) ++send_failures;
      wire::encode_padded(ba.next(monotonic_now()), padding, out);
      if (!b.sock.send_to(b.peer, out.data(), out.size())) ++send_failures;
      next += interval;
      now = monotonic_now();
    }
    const Nanos deadline = sending ? std::min(next, send_end) : stop_at;
    if (deadline > now) net::wait_any_readable(both, 2, deadline - now);
    drain(b, ab, hello_ab.session());
    drain(a, ba, hello_ba.session());
    now = monotonic_now();
    if (now >= next_report) {
      std::fprintf(stderr, "  %5.1f s  sent %zu each way  %s->%s %zu  %s->%s %zu\n",
                   to_seconds(now - start), ab.sent(), o.a.c_str(), o.b.c_str(), ab.received(),
                   o.b.c_str(), o.a.c_str(), ba.received());
      next_report += kNanosPerSecond;
    }
  }

  const net::OneClockReport rab = net::analyze_one_clock(ab), rba = net::analyze_one_clock(ba);
  const std::string nab = o.a + "->" + o.b, nba = o.b + "->" + o.a;
  std::printf("pair %s and %s, %zu-byte probes at %.0f Hz, one clock, started %s\n", o.a.c_str(),
              o.b.c_str(), o.size, o.rate, started.c_str());
  std::printf("probes %zu each way  duplicates %zu/%zu  stray %zu/%zu  send failures %zu  skipped %zu\n",
              rab.dir.sent, rab.duplicates, rba.duplicates, ab.stray(), ba.stray(), send_failures,
              skipped);
  print_delay((nab + " ms").c_str(), rab.dir.delay);
  print_delay((nba + " ms").c_str(), rba.dir.delay);
  print_direction(nab.c_str(), rab.dir);
  print_direction(nba.c_str(), rba.dir);
  if (rab.dir.delay.count > 0 && rba.dir.delay.count > 0) {
    std::printf("%s minus %s: %.3f ms at the minimum, %.3f ms at the median; halving the round "
                "trip, as measure does, would put %.3f ms on each at the minimum\n",
                nab.c_str(), nba.c_str(), (rab.dir.delay.min - rba.dir.delay.min) / 1e6,
                (rab.dir.delay.p50 - rba.dir.delay.p50) / 1e6,
                (rab.dir.delay.min + rba.dir.delay.min) / 2e6);
  }

  if (!o.out.empty()) {
    char meta[256];
    std::snprintf(meta, sizeof meta, "tuushin-probe pair, %zu bytes at %.0f Hz, %.0f s, started %s",
                  o.size, o.rate, o.duration, started.c_str());
    net::Trace tab = rab.trace, tba = rba.trace;
    tab.comments.insert(tab.comments.begin(), std::string(meta) + ", " + nab);
    tba.comments.insert(tba.comments.begin(), std::string(meta) + ", " + nba);
    tab.save_csv(o.out + ".ab.csv");
    tba.save_csv(o.out + ".ba.csv");
    std::printf("wrote %s.ab.csv (%s), %s.ba.csv (%s)\n", o.out.c_str(), nab.c_str(),
                o.out.c_str(), nba.c_str());
  }
  return ab.received() > 0 && ba.received() > 0 ? 0 : 2;
}

}  // namespace

int main(int argc, char** argv) {
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);
  if (argc < 2) {
    usage();
    return 1;
  }
  const std::string mode = argv[1];
  try {
    int i = 2;
    auto next = [&]() -> std::string {
      if (i + 1 >= argc) throw std::runtime_error(std::string("missing value for ") + argv[i]);
      return argv[++i];
    };
    auto port = [&](const std::string& s) {
      const double p = parse_double(s, "port");
      if (p < 1 || p > 65535) throw std::runtime_error("bad port: " + s);
      return static_cast<std::uint16_t>(p);
    };
    if (mode == "reflect") {
      ReflectOptions o;
      for (; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--port") o.port = port(next());
        else if (a == "--bind") o.bind = next();
        else if (a == "--max-pps") o.max_pps = parse_double(next(), "max-pps");
        else if (a == "--dscp") o.dscp = static_cast<int>(parse_double(next(), "dscp"));
        else throw std::runtime_error("unknown option " + a);
      }
      return run_reflect(o);
    }
    if (mode == "measure") {
      MeasureOptions o;
      for (; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--port") o.port = port(next());
        else if (a == "--rate") o.rate = parse_double(next(), "rate");
        else if (a == "--duration") o.duration = parse_double(next(), "duration");
        else if (a == "--size") o.size = static_cast<std::size_t>(parse_double(next(), "size"));
        else if (a == "--dscp") o.dscp = static_cast<int>(parse_double(next(), "dscp"));
        else if (a == "--ipv6") o.ipv6 = true;
        else if (a == "--out") o.out = next();
        else if (!a.empty() && a[0] != '-' && o.host.empty()) o.host = a;
        else throw std::runtime_error("unknown option " + a);
      }
      if (o.host.empty()) throw std::runtime_error("measure needs a HOST");
      return run_measure(o);
    }
    if (mode == "pair") {
      PairOptions o;
      for (; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--a") o.a = next();
        else if (a == "--b") o.b = next();
        else if (a == "--stun") o.stun = next();
        else if (a == "--rate") o.rate = parse_double(next(), "rate");
        else if (a == "--duration") o.duration = parse_double(next(), "duration");
        else if (a == "--size") o.size = static_cast<std::size_t>(parse_double(next(), "size"));
        else if (a == "--dscp") o.dscp = static_cast<int>(parse_double(next(), "dscp"));
        else if (a == "--out") o.out = next();
        else throw std::runtime_error("unknown option " + a);
      }
      if (o.a.empty() || o.b.empty() || o.stun.empty()) {
        throw std::runtime_error("pair needs --a, --b and --stun");
      }
      return run_pair(o);
    }
    usage();
    return 1;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "tuushin-probe: %s\n", e.what());
    return 1;
  }
}
