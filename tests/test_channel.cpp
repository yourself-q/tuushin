#include <cstdio>
#include <memory>
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
