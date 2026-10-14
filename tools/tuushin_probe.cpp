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

#include <algorithm>
#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include "tuushin/net/probe.hpp"
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
               "      --out PREFIX    write PREFIX.fwd.csv, PREFIX.bwd.csv, PREFIX.raw.csv\n",
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
  std::printf("path %s, %zu-byte probes at %.0f Hz, %s\n", peer->to_string().c_str(), o.size, o.rate,
              utc_now().c_str());
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
    std::snprintf(meta, sizeof meta, "tuushin-probe measure %s, %zu bytes at %.0f Hz, %.0f s, %s",
                  o.host.c_str(), o.size, o.rate, o.duration, utc_now().c_str());
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
    usage();
    return 1;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "tuushin-probe: %s\n", e.what());
    return 1;
  }
}
