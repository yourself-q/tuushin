#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "tuushin/net/trace.hpp"
#include "tuushin/rng.hpp"
#include "tuushin/time.hpp"

namespace tuushin::net {

// A one-way network path. Used by the simulator and by the in-process
// impairment layer of the real transports. Calls must arrive in
// non-decreasing `send_time` order, as packets leave a host.
class ChannelModel {
 public:
  virtual ~ChannelModel() = default;
  // Transit time of a packet of `bytes` sent at `send_time`, or nullopt if it
  // is lost.
  virtual std::optional<Nanos> transit(Nanos send_time, std::size_t bytes) = 0;
};

// Constant delay, no loss.
class FixedDelay final : public ChannelModel {
 public:
  explicit FixedDelay(Nanos delay) : delay_(delay) {}
  std::optional<Nanos> transit(Nanos, std::size_t) override { return delay_; }

 private:
  Nanos delay_;
};

// Base delay plus exponentially distributed jitter.
//
// With `fifo` set, a packet never overtakes an earlier one, which is what a
// single queue on a single path does; jitter then shows up as bunching. With
// `fifo` cleared packets may be reordered, as on multipath or some radio links.
class JitterDelay final : public ChannelModel {
 public:
  JitterDelay(Nanos base, Nanos mean_jitter, bool fifo, std::uint64_t seed)
      : base_(base), mean_jitter_(mean_jitter), fifo_(fifo), rng_(seed) {}

  std::optional<Nanos> transit(Nanos send_time, std::size_t bytes) override;

 private:
  Nanos base_;
  Nanos mean_jitter_;
  bool fifo_;
  Rng rng_;
  Nanos last_arrival_ = INT64_MIN;
};

// Two-state Gilbert-Elliott burst loss on top of another path.
class GilbertElliottLoss final : public ChannelModel {
 public:
  struct Params {
    double p_good_to_bad = 0.0;  // per-packet transition probability
    double p_bad_to_good = 1.0;
    double loss_good = 0.0;      // loss probability in each state
    double loss_bad = 1.0;
  };

  GilbertElliottLoss(std::unique_ptr<ChannelModel> inner, Params params, std::uint64_t seed)
      : inner_(std::move(inner)), params_(params), rng_(seed) {}

  std::optional<Nanos> transit(Nanos send_time, std::size_t bytes) override;

 private:
  std::unique_ptr<ChannelModel> inner_;
  Params params_;
  Rng rng_;
  bool bad_ = false;
};

// Replays a recorded trace, looping when the simulation outlasts it.
//
// Each packet takes the delay (or loss) of the trace entry sent most recently
// at or before the same offset. The trace must have been recorded at a rate at
// least as high as the replayed stream for this to be faithful.
class TraceChannel final : public ChannelModel {
 public:
  explicit TraceChannel(Trace trace, Nanos start_time = 0);
  std::optional<Nanos> transit(Nanos send_time, std::size_t bytes) override;

 private:
  Trace trace_;
  Nanos start_time_;
  Nanos duration_;
};

// A single bottleneck link with a drop-tail FIFO buffer, shared with fluid
// cross traffic. Models the queueing delay ("bufferbloat") that dominates
// latency on congested access links.
class BottleneckLink final : public ChannelModel {
 public:
  struct Phase {
    Nanos start = 0;          // phase begins at this absolute time
    double cross_bps = 0.0;   // cross traffic offered to the link
  };
  struct Params {
    double rate_bps = 10e6;            // link capacity
    std::size_t buffer_bytes = 256 * 1024;
    Nanos propagation = from_millis(5);
    std::vector<Phase> cross_traffic;  // sorted by start; empty = idle link
    std::uint64_t seed = 1;
  };

  explicit BottleneckLink(Params params) : params_(std::move(params)), rng_(params_.seed) {}
  std::optional<Nanos> transit(Nanos send_time, std::size_t bytes) override;

  // Queue backlog right after the last packet was enqueued.
  double backlog_bytes() const { return backlog_; }

 private:
  double cross_rate_at(Nanos t) const;

  Params params_;
  Rng rng_;
  double backlog_ = 0.0;
  Nanos last_time_ = INT64_MIN;
};

}  // namespace tuushin::net
