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

TEST_CASE(probe_leaves_out_unsent_probes_and_the_time_after_a_cut) {
  // Twelve probes 1 ms apart over 2 ms each way. Probe 2 is lost on the way
  // out and probe 3's send fails. At 7.5 ms the client's network goes away:
  // the reflector still gets probes 5 to 8, but no echo comes back after
  // that, and from probe 9 (8 ms) every send fails. The run is cut at 8 ms.
  ProbeClient client(9);
  ProbeReflector reflector;
  std::vector<std::uint8_t> buf;
  const Nanos t0 = 2 * kNanosPerSecond, gone = t0 + 7'500'000;
  for (int k = 1; k <= 12; ++k) {
    const Nanos t1 = t0 + (k - 1) * kNanosPerMilli;
    wire::encode_padded(client.next_probe(t1), 68, buf);
    if (k == 3 || t1 >= gone) {
      client.mark_unsent();
      client.mark_unsent();  // once per probe
      continue;
    }
    if (k == 2) continue;
    const auto h = wire::decode_header(buf.data(), buf.size());
    const Nanos t2 = t1 + from_millis(2);
    const auto echo = reflector.on_probe(*h, t2, t2);
    REQUIRE(echo.has_value());
    const Nanos t4 = t2 + from_millis(2);
    if (t4 >= gone) continue;
    wire::encode_probe_echo(echo->header, echo->receipts, echo->payload, buf);
    CHECK(client.on_datagram(buf.data(), buf.size(), t4));
  }
  CHECK(client.sent() == 12 && client.unsent() == 5 && client.echoes() == 2);

  // Uncut, probes 5 to 8 read as forward losses with no receipt map to say
  // otherwise.
  const ProbeReport whole = analyze_probes(client);
  CHECK(whole.unsent == 5 && whole.in_flight == 0);
  CHECK(whole.probes == 7 && whole.fwd.sent == 7 && whole.fwd.lost == 5);
  CHECK(whole.ambiguous == 4 && whole.fwd.longest_burst == 4);

  // Probes 9 to 12 come after the cut. Probes 5 to 8, sent within 4 ms of
  // it (the longest round trip seen), may have been on their way; probe 2,
  // sent long before and covered by probe 4's receipt map, stays lost.
  const ProbeReport cut = analyze_probes(client, kNanosPerSecond, t0 + 8 * kNanosPerMilli);
  CHECK(cut.unsent == 1 && cut.in_flight == 4);
  CHECK(cut.probes == 3 && cut.fwd.sent == 3 && cut.fwd.lost == 1 && cut.ambiguous == 0);
  CHECK(cut.bwd.sent == 2 && cut.bwd.lost == 0 && cut.interpolated == 0);
  CHECK(cut.fwd.longest_burst == 1);
  REQUIRE(cut.fwd_trace.entries.size() == 3);
  // Probe 3 leaves no entry: the trace goes from 1 ms to 3 ms.
  CHECK(cut.fwd_trace.entries[0].send_offset == 0 && cut.fwd_trace.entries[0].delay == from_millis(2));
  CHECK(cut.fwd_trace.entries[1].send_offset == kNanosPerMilli && cut.fwd_trace.entries[1].lost());
  CHECK(cut.fwd_trace.entries[2].send_offset == 3 * kNanosPerMilli &&
        cut.fwd_trace.entries[2].delay == from_millis(2));
  CHECK(cut.bwd_trace.entries.size() == 2);
  CHECK_NEAR(cut.rtt.max, 4e6, 0.0);
}

TEST_CASE(one_clock_direction_measures_each_delay) {
  // Delays by hand: a loss, a duplicate, two probes that swap places, a
  // stray from another session and an echo, all through the wire format.
  OneClockDirection dir(0xAB);
  const std::vector<Nanos> delay = {5'000'000, 6'000'000, -1, 9'000'000, 3'500'000, 4'000'000};
  struct Arrival {
    Nanos time;
    std::vector<std::uint8_t> bytes;
  };
  std::vector<Arrival> arrivals;
  std::vector<std::uint8_t> buf;
  for (std::size_t k = 0; k < delay.size(); ++k) {
    const Nanos t = 1'000'000'000 + static_cast<Nanos>(k) * 1'000'000;
    wire::encode_padded(dir.next(t), 100 - wire::kHeaderSize, buf);
    if (delay[k] >= 0) arrivals.push_back({t + delay[k], buf});
    if (k == 1) arrivals.push_back({t + delay[k] + 300'000, buf});
  }
  wire::encode_padded(probe(0xAC, 1), 32, buf);
  arrivals.push_back({1'002'000'000, buf});
  wire::Header echo = probe(0xAB, 1);
  echo.type = wire::PacketType::kProbeEcho;
  wire::encode_probe_echo(echo, 0, 32, buf);
  arrivals.push_back({1'002'500'000, buf});
  std::sort(arrivals.begin(), arrivals.end(),
            [](const Arrival& a, const Arrival& b) { return a.time < b.time; });
  for (const auto& a : arrivals) dir.on_datagram(a.bytes.data(), a.bytes.size(), a.time);

  CHECK(dir.sent() == 6);
  CHECK(dir.received() == 5);
  CHECK(dir.stray() == 2);
  const OneClockReport rep = analyze_one_clock(dir);
  CHECK(rep.dir.sent == 6 && rep.dir.lost == 1);
  CHECK(rep.duplicates == 1);
  // Probe 4 (9 ms) arrives after probes 5 and 6 (3.5 and 4 ms, sent 1 and
  // 2 ms later), which count as reordered.
  CHECK(rep.dir.reordered == 2);
  CHECK(rep.dir.longest_burst == 1);
  REQUIRE(rep.trace.entries.size() == delay.size());
  for (std::size_t k = 0; k < delay.size(); ++k) {
    CHECK(rep.trace.entries[k].send_offset == static_cast<Nanos>(k) * 1'000'000);
    CHECK(rep.trace.entries[k].delay == delay[k]);
  }
  CHECK(rep.dir.delay.count == 5);
  CHECK_NEAR(rep.dir.delay.min, 3.5e6, 0.0);
  CHECK_NEAR(rep.dir.delay.max, 9e6, 0.0);
  CHECK(!rep.trace.comments.empty());
}

TEST_CASE(one_clock_leaves_out_unsent_probes_and_the_time_after_a_cut) {
  // Twelve probes 1 ms apart, each arriving 5 ms later if at all. Probe 2
  // is lost and probe 3's send fails. From probe 5 (sent at 4 ms) nothing
  // arrives: the far end went away, and the run is cut at 10 ms.
  OneClockDirection dir(7);
  std::vector<std::uint8_t> buf;
  const Nanos t0 = 2 * kNanosPerSecond;
  for (int k = 1; k <= 12; ++k) {
    const Nanos t = t0 + (k - 1) * kNanosPerMilli;
    wire::encode_padded(dir.next(t), 32, buf);
    if (k == 3) {
      dir.mark_unsent();
      dir.mark_unsent();  // once per probe
      continue;
    }
    if (k == 2 || k >= 5) continue;
    dir.on_datagram(buf.data(), buf.size(), t + from_millis(5));
  }
  CHECK(dir.sent() == 12 && dir.unsent() == 1 && dir.received() == 2);

  const OneClockReport whole = analyze_one_clock(dir);
  CHECK(whole.unsent == 1 && whole.in_flight == 0);
  CHECK(whole.dir.sent == 11 && whole.dir.lost == 9);
  CHECK(whole.dir.longest_burst == 8);
  CHECK(whole.trace.entries.size() == 11);

  // Probes 11 and 12 come after the cut. Probes 6 to 10, sent within 5 ms
  // of it (the longest delay seen), may have been on their way when the end
  // went; probe 5, sent 6 ms before, would have arrived, so its loss stays.
  const OneClockReport cut = analyze_one_clock(dir, t0 + 10 * kNanosPerMilli);
  CHECK(cut.unsent == 1 && cut.in_flight == 5);
  CHECK(cut.dir.sent == 4 && cut.dir.lost == 2);
  CHECK(cut.dir.longest_burst == 1);
  REQUIRE(cut.trace.entries.size() == 4);
  // Probe 3 leaves no entry: the trace goes from 1 ms to 3 ms.
  CHECK(cut.trace.entries[1].send_offset == kNanosPerMilli && cut.trace.entries[1].delay == -1);
  CHECK(cut.trace.entries[2].send_offset == 3 * kNanosPerMilli);
  CHECK(cut.trace.entries[3].send_offset == 4 * kNanosPerMilli && cut.trace.entries[3].delay == -1);
  CHECK(cut.dir.delay.count == 2);
  CHECK_NEAR(cut.dir.delay.p50, 5e6, 0.0);
}

TEST_CASE(one_clock_traces_replay_the_measured_path) {
  // Two directions of different constant delays through one clock come out
  // as measured, each on its own: no half of the round trip is assumed.
  FixedDelay ab(from_millis(20)), ba(from_millis(35));
  OneClockDirection dab(1), dba(2);
  std::vector<std::uint8_t> buf;
  for (int i = 0; i < 2000; ++i) {
    const Nanos t = 3 * kNanosPerSecond + i * kNanosPerMilli;
    for (auto* p : {&dab, &dba}) {
      wire::encode_padded(p->next(t), 64, buf);
      const auto d = (p == &dab ? static_cast<ChannelModel&>(ab) : ba).transit(t, buf.size());
      REQUIRE(d.has_value());
      p->on_datagram(buf.data(), buf.size(), t + *d);
    }
  }
  const OneClockReport rab = analyze_one_clock(dab), rba = analyze_one_clock(dba);
  CHECK_NEAR(rab.dir.delay.p50, 20e6, 0.0);
  CHECK_NEAR(rba.dir.delay.p50, 35e6, 0.0);
  CHECK(rab.dir.lost == 0 && rba.dir.reordered == 0);
  TraceChannel replay(rba.trace);
  for (Nanos t = 0; t < kNanosPerSecond; t += 333'333) {
    const auto d = replay.transit(t, 64);
    REQUIRE(d.has_value());
    CHECK(*d == from_millis(35));
  }
}
