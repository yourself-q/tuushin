#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <string>
#include <unistd.h>

#include "test.hpp"
#include "tuushin/net/channel.hpp"
#include "tuushin/net/trace.hpp"

using namespace tuushin;
using namespace tuushin::net;

TEST_CASE(channel_fixed_delay) {
  FixedDelay ch(from_millis(12));
  CHECK(*ch.transit(0, 64) == from_millis(12));
  CHECK(*ch.transit(from_seconds(3), 64) == from_millis(12));
}

TEST_CASE(channel_jitter_fifo_never_reorders) {
  JitterDelay ch(from_millis(10), from_millis(5), true, 7);
  Nanos last_arrival = INT64_MIN;
  for (int i = 0; i < 100000; ++i) {
    const Nanos send = i * kNanosPerMilli;
    const Nanos arrival = send + *ch.transit(send, 64);
    CHECK(arrival >= last_arrival);
    CHECK(arrival - send >= from_millis(10));
    last_arrival = arrival;
  }
}

TEST_CASE(channel_jitter_non_fifo_reorders_and_has_right_mean) {
  JitterDelay ch(from_millis(10), from_millis(5), false, 7);
  Nanos last_arrival = INT64_MIN;
  int reordered = 0;
  double sum = 0.0;
  const int n = 200000;
  for (int i = 0; i < n; ++i) {
    const Nanos send = i * kNanosPerMilli;
    const Nanos d = *ch.transit(send, 64);
    sum += to_millis(d);
    if (send + d < last_arrival) ++reordered;
    last_arrival = std::max(last_arrival, send + d);
  }
  CHECK(reordered > 0);
  CHECK_NEAR(sum / n, 15.0, 0.1);
}

TEST_CASE(channel_gilbert_elliott_stationary_loss) {
  GilbertElliottLoss::Params p;
  p.p_good_to_bad = 0.01;
  p.p_bad_to_good = 0.2;
  p.loss_good = 0.0;
  p.loss_bad = 0.5;
  GilbertElliottLoss ch(std::make_unique<FixedDelay>(from_millis(1)), p, 3);
  const int n = 1000000;
  int lost = 0;
  for (int i = 0; i < n; ++i) {
    if (!ch.transit(i * kNanosPerMilli, 64)) ++lost;
  }
  // Stationary P(bad) = p_gb / (p_gb + p_bg) = 0.01 / 0.21; loss = 0.5 * that.
  CHECK_NEAR(static_cast<double>(lost) / n, 0.5 * 0.01 / 0.21, 0.002);
}

TEST_CASE(trace_roundtrip_and_replay) {
  Trace t;
  t.comments.push_back("unit test");
  t.entries = {{0, from_millis(10)}, {kNanosPerMilli, -1}, {2 * kNanosPerMilli, from_millis(30)}};
  char path[] = "/tmp/tuushin_trace_XXXXXX";
  const int fd = mkstemp(path);
  REQUIRE(fd >= 0);
  close(fd);
  t.save_csv(path);
  const Trace u = Trace::load_csv(path);
  std::remove(path);
  REQUIRE(u.entries.size() == 3);
  CHECK(u.comments.size() == 1 && u.comments[0] == "unit test");
  CHECK(u.entries[1].lost());
  CHECK(u.entries[2].delay == from_millis(30));
  CHECK_NEAR(u.loss_rate(), 1.0 / 3.0, 1e-12);
  CHECK(u.duration() == 3 * kNanosPerMilli);

  TraceChannel ch(u, from_seconds(1));
  CHECK(*ch.transit(from_seconds(1), 64) == from_millis(10));
  CHECK(!ch.transit(from_seconds(1) + kNanosPerMilli + 500, 64));
  CHECK(*ch.transit(from_seconds(1) + 2 * kNanosPerMilli, 64) == from_millis(30));
  // Loops after duration() = 3 ms.
  CHECK(*ch.transit(from_seconds(1) + 3 * kNanosPerMilli, 64) == from_millis(10));
}

TEST_CASE(trace_grid_keeps_a_regular_trace) {
  const Nanos ms = kNanosPerMilli;
  Trace t;
  t.entries = {{0, 10 * ms}, {ms, 12 * ms}, {2 * ms, -1}, {3 * ms, 16 * ms}, {4 * ms, 11 * ms}};
  CHECK(t.median_interval() == ms);
  const TraceGrid g = t.grid(ms);
  REQUIRE(g.delay.size() == 5);
  // The lost packet's point takes its delay from the packets around it.
  const std::vector<Nanos> delay{10 * ms, 12 * ms, 14 * ms, 16 * ms, 11 * ms};
  const std::vector<bool> lost{false, false, true, false, false};
  CHECK(g.delay == delay);
  CHECK(g.lost == lost);
  CHECK_NEAR(g.loss_rate(), 0.2, 1e-12);
  CHECK(g.delay_at(0) == 10 * ms);
  CHECK(g.delay_at(ms / 2) == 11 * ms);
  CHECK(g.delay_at(2 * ms + ms / 2) == -1);
  // The last point leads back to the first: the trace loops.
  CHECK(g.delay_at(4 * ms + ms / 2) == 10 * ms + ms / 2);
  CHECK(g.delay_at(5 * ms) == 10 * ms);
  bool threw = false;
  try {
    (void)t.grid(0);
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  CHECK(threw);
}

TEST_CASE(trace_grid_resamples_wandering_sends) {
  // A probe whose sends wander up to 0.2 ms off a 1 ms grid, every tenth
  // lost, on a path whose delay is linear in the send time.
  const Nanos ms = kNanosPerMilli;
  Trace t;
  for (int i = 0; i < 1000; ++i) {
    const Nanos send = i * ms + (i % 5 - 2) * ms / 10;
    t.entries.push_back({send, i % 10 == 0 ? -1 : 10 * ms + send / 100});
  }
  const TraceGrid g = t.grid(ms);
  REQUIRE(g.delay.size() >= 999);
  // From the first received packet, the second sent, to the last.
  const Nanos first = t.entries.front().send_offset;
  for (std::size_t k = 2; k < 990; ++k) {
    CHECK(g.lost[k] == (k % 10 == 0));
    CHECK(std::llabs(g.delay[k] - (10 * ms + (first + static_cast<Nanos>(k) * ms) / 100)) <= 1);
  }
}

TEST_CASE(trace_grid_replays_in_order_what_arrived_in_order) {
  // A queue that fills slowly and drains fast: the delay falls by up to
  // 0.95 ms per 1 ms probe, so every probe still arrived after the one
  // before it.
  const Nanos ms = kNanosPerMilli;
  Trace t;
  Nanos d = 5 * ms;
  for (int i = 0; i < 3000; ++i) {
    t.entries.push_back({i * ms, d});
    d = (i / 40) % 2 == 0 ? d + ms / 5 : std::max<Nanos>(5 * ms, d - ms * 95 / 100);
  }
  for (std::size_t i = 1; i < t.entries.size(); ++i) {
    REQUIRE(t.entries[i].send_offset + t.entries[i].delay >=
            t.entries[i - 1].send_offset + t.entries[i - 1].delay);
  }
  const TraceGrid g = t.grid(ms);
  // Datagrams at 27 kHz, off the probes' grid, up to the last point (from
  // which the trace loops).
  Nanos last = 0, last_held = 0;
  int reordered = 0, reordered_held = 0;
  for (Nanos s = 0; s < 2999 * ms; s += 37'000) {
    const Nanos arrival = s + g.delay_at(s);
    if (arrival < last) ++reordered;
    last = std::max(last, arrival);
    // Holding each point's delay over its bin instead.
    const Nanos held = s + g.delay[static_cast<std::size_t>(s / ms)];
    if (held < last_held) ++reordered_held;
    last_held = std::max(last_held, held);
  }
  CHECK(reordered == 0);
  CHECK(reordered_held > 100);
}

TEST_CASE(bottleneck_idle_link_has_only_propagation_and_serialization) {
  BottleneckLink::Params p;
  p.rate_bps = 10e6;
  p.propagation = from_millis(5);
  BottleneckLink link(p);
  // 64 bytes at 10 Mbit/s = 51.2 us; 1 kHz leaves the queue empty each time.
  for (int i = 0; i < 1000; ++i) {
    const auto d = link.transit(i * kNanosPerMilli, 64);
    REQUIRE(d.has_value());
    CHECK_NEAR(to_millis(*d), 5.0512, 1e-6);
  }
}

TEST_CASE(bottleneck_cross_traffic_builds_queue_delay) {
  BottleneckLink::Params p;
  p.rate_bps = 10e6;
  p.buffer_bytes = 125000;  // 100 ms at 10 Mbit/s
  p.propagation = from_millis(5);
  p.cross_traffic = {{from_seconds(1), 12e6}, {from_seconds(3), 0.0}};
  BottleneckLink link(p);
  double max_delay = 0.0;
  int lost_overload = 0;
  int sent_overload = 0;
  double delay_after = 0.0;
  for (int i = 0; i < 5000; ++i) {
    const Nanos t = i * kNanosPerMilli;
    const auto d = link.transit(t, 64);
    if (t > from_seconds(2) && t < from_seconds(3)) {
      ++sent_overload;
      if (!d) ++lost_overload;
    }
    if (d) max_delay = std::max(max_delay, to_millis(*d));
    if (t == from_seconds(4.9) && d) delay_after = to_millis(*d);
  }
  // Full buffer: 5 ms propagation + 100 ms queue.
  CHECK_NEAR(max_delay, 105.0, 0.2);
  // Excess fraction 1 - 10/12 = 1/6 of our packets dropped while overloaded.
  CHECK_NEAR(static_cast<double>(lost_overload) / sent_overload, 1.0 / 6.0, 0.04);
  // The queue drains once the cross traffic stops.
  CHECK_NEAR(delay_after, 5.0512, 1e-3);
}
