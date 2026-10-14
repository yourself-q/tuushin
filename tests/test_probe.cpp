#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

#include "test.hpp"
#include "tuushin/net/channel.hpp"
#include "tuushin/net/probe.hpp"
#include "tuushin/wire/packet.hpp"

using namespace tuushin;
using namespace tuushin::net;

namespace {

wire::Header probe(std::uint32_t session, std::uint32_t seq, std::uint16_t payload = 68) {
  wire::Header h;
  h.type = wire::PacketType::kProbe;
  h.session = session;
  h.seq = seq;
  h.payload_len = payload;
  return h;
}

// Truth about one probe of the in-process run below.
struct Truth {
  bool fwd_lost = false;
  bool bwd_lost = false;  // meaningful if !fwd_lost
  double fwd = 0, bwd = 0;  // true one-way delays, ns
};

struct Run {
  ProbeClient client{0x5EED};
  std::vector<Truth> truth;
};

// Client -> forward path -> reflector -> backward path -> client, with the
// reflector clock running `skew` fast and `offset` ahead. Client time is the
// true time. Everything is encoded and decoded as on the wire.
Run simulate(ChannelModel& fwd_path, ChannelModel& bwd_path, int probes, Nanos interval,
             double offset, double skew, Nanos hold = 20'000) {
  Run run;
  run.truth.resize(static_cast<std::size_t>(probes));
  struct Arrival {
    Nanos time;
    std::vector<std::uint8_t> bytes;
    std::uint32_t seq;
  };
  std::vector<Arrival> at_reflector;
  std::vector<std::uint8_t> buf;
  const Nanos start = 5 * kNanosPerSecond;
  for (int i = 0; i < probes; ++i) {
    const Nanos t1 = start + i * interval;
    wire::Header h = run.client.next_probe(t1);
    wire::encode_padded(h, 68, buf);
    const auto d = fwd_path.transit(t1, buf.size());
    Truth& tr = run.truth[static_cast<std::size_t>(i)];
    if (!d) {
      tr.fwd_lost = true;
      continue;
    }
    tr.fwd = static_cast<double>(*d);
    at_reflector.push_back({t1 + *d, buf, h.seq});
  }
  std::stable_sort(at_reflector.begin(), at_reflector.end(),
                   [](const Arrival& a, const Arrival& b) { return a.time < b.time; });

  auto server_clock = [&](Nanos t) {
    return t + static_cast<Nanos>(std::llround(offset + skew * static_cast<double>(t)));
  };
  ProbeReflector reflector;
  std::vector<Arrival> at_client;
  for (const auto& a : at_reflector) {
    const auto h = wire::decode_header(a.bytes.data(), a.bytes.size());
    const Nanos rx = server_clock(a.time);
    const auto echo = reflector.on_probe(*h, rx, rx + hold);  // one session: never refused
    wire::encode_probe_echo(echo->header, echo->receipts, echo->payload, buf);
    const Nanos sent = a.time + hold;  // hold is short enough to ignore the skew
    const auto d = bwd_path.transit(sent, buf.size());
    Truth& tr = run.truth[a.seq - 1];
    if (!d) {
      tr.bwd_lost = true;
      continue;
    }
    tr.bwd = static_cast<double>(*d);
    at_client.push_back({sent + *d, buf, a.seq});
  }
  std::stable_sort(at_client.begin(), at_client.end(),
                   [](const Arrival& a, const Arrival& b) { return a.time < b.time; });
  for (const auto& a : at_client) run.client.on_datagram(a.bytes.data(), a.bytes.size(), a.time);
  return run;
}

std::unique_ptr<ChannelModel> lossy_path(Nanos base, Nanos jitter, bool fifo, double p_gb,
                                         std::uint64_t seed) {
  GilbertElliottLoss::Params p;
  p.p_good_to_bad = p_gb;
  p.p_bad_to_good = 0.3;
  p.loss_good = 0.0;
  p.loss_bad = 0.6;
  return std::make_unique<GilbertElliottLoss>(std::make_unique<JitterDelay>(base, jitter, fifo, seed),
                                              p, seed + 100);
}

}  // namespace

TEST_CASE(probe_reflector_echo_fields_and_receipts) {
  ProbeReflector r;
  for (std::uint32_t seq = 1; seq <= 9; ++seq) {
    if (seq == 5) continue;
    REQUIRE(r.on_probe(probe(7, seq), seq * 1000, seq * 1000 + 30).has_value());
  }
  const auto e = r.on_probe(probe(7, 10, 100), 50'000, 50'250);
  REQUIRE(e.has_value());
  CHECK(e->header.type == wire::PacketType::kProbeEcho);
  CHECK(e->header.session == 7);
  CHECK(e->header.seq == 9);  // the reflector's own count of answers
  CHECK(e->header.echo_seq == 10);
  CHECK(e->header.send_time == 50'250);
  CHECK(e->header.echo_hold == 250);
  CHECK(e->payload == 100);
  // Bit i is probe 10 - 1 - i: seqs 9..1 present except 5 (bit 4).
  CHECK(e->receipts == 0x1EFu);

  // Sessions count independently.
  const auto other = r.on_probe(probe(8, 1), 60'000, 60'000);
  CHECK(other->header.seq == 1);
  CHECK(other->receipts == 0);
  CHECK(r.sessions() == 2);
}

TEST_CASE(probe_reflector_handles_reordering_and_jumps) {
  ProbeReflector r;
  r.on_probe(probe(1, 1), 0, 0);
  r.on_probe(probe(1, 3), 0, 0);
  // Probe 2 arrives late: the echo of 4 sees all of 1..3.
  r.on_probe(probe(1, 2), 0, 0);
  CHECK(r.on_probe(probe(1, 4), 0, 0)->receipts == 0x7u);
  // A jump beyond the window forgets everything before it.
  CHECK(r.on_probe(probe(1, 5000), 0, 0)->receipts == 0);
  CHECK(r.on_probe(probe(1, 5001), 0, 0)->receipts == 0x1u);
  // A probe older than the window is still answered.
  CHECK(r.on_probe(probe(1, 10), 0, 0)->header.echo_seq == 10);
}

TEST_CASE(probe_reflector_limits_and_expires_sessions) {
  ProbeReflector r(kNanosPerSecond, 2);
  CHECK(r.on_probe(probe(1, 1), 0, 0).has_value());
  CHECK(r.on_probe(probe(2, 1), 0, 0).has_value());
  CHECK(!r.on_probe(probe(3, 1), 0, 0).has_value());
  // Session 2 stays alive, session 1 goes idle.
  CHECK(r.on_probe(probe(2, 2), 900'000'000, 0).has_value());
  CHECK(r.on_probe(probe(3, 1), 1'500'000'000, 0).has_value());
  CHECK(r.sessions() == 2);
}

TEST_CASE(probe_client_ignores_strays) {
  ProbeClient c(42);
  c.next_probe(100);
  std::vector<std::uint8_t> buf;
  wire::Header h;
  h.session = 41;
  h.echo_seq = 1;
  wire::encode_probe_echo(h, 0, 32, buf);
  CHECK(!c.on_datagram(buf.data(), buf.size(), 200));  // other session
  h.session = 42;
  h.echo_seq = 2;
  wire::encode_probe_echo(h, 0, 32, buf);
  CHECK(!c.on_datagram(buf.data(), buf.size(), 200));  // never sent
  h.echo_seq = 1;
  h.type = wire::PacketType::kProbe;
  wire::encode_padded(h, 32, buf);
  CHECK(!c.on_datagram(buf.data(), buf.size(), 200));  // not an echo
  CHECK(c.stray() == 3);
  wire::encode_probe_echo(h, 0, 32, buf);
  CHECK(c.on_datagram(buf.data(), buf.size(), 200));
  CHECK(c.on_datagram(buf.data(), buf.size(), 300));
  CHECK(c.records()[0].echoed && c.records()[0].duplicates == 1 && c.records()[0].t4 == 200);
}

TEST_CASE(probe_measures_a_lossy_skewed_path_exactly) {
  // 30 s at 1 kHz over a path with bursty loss in both directions,
  // reordering forward, and a reflector clock 2.5 s ahead and 40 ppm fast.
  auto fwd = lossy_path(from_millis(9), from_millis(1), false, 0.004, 1);
  auto bwd = lossy_path(from_millis(9), from_millis(2), true, 0.006, 2);
  const int n = 30000;
  const Run run = simulate(*fwd, *bwd, n, kNanosPerMilli, 2.5e9, 40e-6);
  const ProbeReport rep = analyze_probes(run.client);

  std::size_t true_fwd_lost = 0, true_bwd_lost = 0, true_fwd_reordered = 0;
  Nanos max_arrival = INT64_MIN;
  for (std::size_t k = 0; k < run.truth.size(); ++k) {
    const Truth& t = run.truth[k];
    if (t.fwd_lost) {
      ++true_fwd_lost;
      continue;
    }
    if (t.bwd_lost) ++true_bwd_lost;
    const Nanos arrival = static_cast<Nanos>(k) * kNanosPerMilli + static_cast<Nanos>(t.fwd);
    if (!t.bwd_lost) {
      if (arrival < max_arrival) ++true_fwd_reordered;
      max_arrival = std::max(max_arrival, arrival);
    }
  }
  REQUIRE(true_fwd_lost > 50 && true_bwd_lost > 50);

  // The receipt maps attribute every loss to its direction, except possibly
  // at the very end of the run where no later echo speaks for a probe.
  CHECK(rep.probes == static_cast<std::size_t>(n));
  CHECK(rep.ambiguous <= 3);
  CHECK(rep.fwd.lost >= true_fwd_lost && rep.fwd.lost <= true_fwd_lost + rep.ambiguous);
  CHECK(rep.bwd.lost + (rep.fwd.lost - true_fwd_lost) == true_bwd_lost);
  CHECK(rep.interpolated == rep.bwd.lost);
  CHECK(rep.fwd.reordered == true_fwd_reordered);
  CHECK(rep.fwd.reordered > 0);
  CHECK(rep.bwd.reordered == 0);  // FIFO backward path

  // Clock and delays.
  // The FIFO backward path bunches packets, so only about one in eight can
  // see an empty queue and the window minima scatter by several
  // microseconds; over 30 s that leaves a few tenths of a ppm.
  CHECK_NEAR(rep.clock.skew, 40e-6, 0.5e-6);
  REQUIRE(rep.fwd_trace.entries.size() == static_cast<std::size_t>(n));
  double worst_fwd = 0.0;
  for (std::size_t k = 0; k < run.truth.size(); ++k) {
    const Truth& t = run.truth[k];
    const TraceEntry& e = rep.fwd_trace.entries[k];
    CHECK(e.send_offset == static_cast<Nanos>(k) * kNanosPerMilli);
    if (t.fwd_lost) {
      CHECK(e.lost() || rep.ambiguous > 0);
      continue;
    }
    if (!t.bwd_lost) worst_fwd = std::max(worst_fwd, std::fabs(static_cast<double>(e.delay) - t.fwd));
  }
  // Minima of 1000 exponential samples (means 1 and 2 ms) per window sit
  // within a few microseconds of the floor, so the clock removal is good to
  // about that.
  CHECK(worst_fwd < 20e3);
  CHECK_NEAR(rep.fwd.delay.min, 9e6, 20e3);
  CHECK_NEAR(rep.bwd.delay.min, 9e6, 20e3);
  // Both directions near their floor in the same probe is rarer: the
  // minimum of the sum over 30000 probes is some 10-30 us above 18 ms.
  CHECK_NEAR(rep.rtt.min, 18e6, 50e3);
  CHECK(rep.rtt.min >= rep.fwd.delay.min + rep.bwd.delay.min - 1e3);
  // Exponential jitter: median = base + mean * ln 2.
  CHECK_NEAR(rep.fwd.delay.p50, 9e6 + 1e6 * std::log(2.0), 50e3);
  CHECK(rep.bwd.delay.p99 > rep.fwd.delay.p99);

  // The backward trace has one entry per probe the reflector answered.
  CHECK(rep.bwd_trace.entries.size() == static_cast<std::size_t>(n) - rep.fwd.lost);
  CHECK(rep.bwd.sent == rep.bwd_trace.entries.size());
  CHECK(std::is_sorted(rep.bwd_trace.entries.begin(), rep.bwd_trace.entries.end(),
                       [](const TraceEntry& a, const TraceEntry& b) {
                         return a.send_offset < b.send_offset;
                       }));
  CHECK(rep.bwd_trace.entries.front().send_offset == 0);
  CHECK(rep.fwd.longest_burst >= 2);
}

TEST_CASE(probe_traces_replay_the_measured_path) {
  // A trace measured on a fixed path replays as that path.
  FixedDelay fwd(from_millis(7)), bwd(from_millis(7));
  const Run run = simulate(fwd, bwd, 5000, kNanosPerMilli, -1.2e9, -15e-6);
  const ProbeReport rep = analyze_probes(run.client);
  CHECK(rep.fwd.lost == 0 && rep.bwd.lost == 0 && rep.ambiguous == 0);
  TraceChannel replay(rep.fwd_trace);
  for (Nanos t = 0; t < 4 * kNanosPerSecond; t += 777'777) {
    const auto d = replay.transit(t, 64);
    REQUIRE(d.has_value());
    CHECK(std::llabs(*d - from_millis(7)) <= 2);
  }
  CHECK_NEAR(rep.fwd.delay.rfc3550_jitter, 0.0, 2.0);
}
