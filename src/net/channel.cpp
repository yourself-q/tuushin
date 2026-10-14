#include "tuushin/net/channel.hpp"

#include <algorithm>
#include <stdexcept>

namespace tuushin::net {

std::optional<Nanos> JitterDelay::transit(Nanos send_time, std::size_t) {
  const double jitter = rng_.exponential(static_cast<double>(mean_jitter_));
  Nanos arrival = send_time + base_ + static_cast<Nanos>(jitter);
  if (fifo_ && arrival < last_arrival_) arrival = last_arrival_;
  last_arrival_ = arrival;
  return arrival - send_time;
}

std::optional<Nanos> GilbertElliottLoss::transit(Nanos send_time, std::size_t bytes) {
  // The inner path is always consulted so its own state (FIFO order, queue
  // backlog) advances whether or not this packet survives.
  const auto inner = inner_->transit(send_time, bytes);
  if (bad_) {
    if (rng_.bernoulli(params_.p_bad_to_good)) bad_ = false;
  } else {
    if (rng_.bernoulli(params_.p_good_to_bad)) bad_ = true;
  }
  const double p_loss = bad_ ? params_.loss_bad : params_.loss_good;
  if (rng_.bernoulli(p_loss)) return std::nullopt;
  return inner;
}

TraceChannel::TraceChannel(Trace trace, Nanos start_time)
    : trace_(std::move(trace)), start_time_(start_time), duration_(trace_.duration()) {
  if (trace_.entries.empty()) throw std::invalid_argument("TraceChannel: empty trace");
  // Rebase so the first entry sits at offset 0.
  const Nanos first = trace_.entries.front().send_offset;
  for (auto& e : trace_.entries) e.send_offset -= first;
}

std::optional<Nanos> TraceChannel::transit(Nanos send_time, std::size_t) {
  Nanos offset = (send_time - start_time_) % duration_;
  if (offset < 0) offset += duration_;
  // Last entry with send_offset <= offset.
  const auto it = std::upper_bound(
      trace_.entries.begin(), trace_.entries.end(), offset,
      [](Nanos value, const TraceEntry& e) { return value < e.send_offset; });
  const TraceEntry& e = (it == trace_.entries.begin()) ? trace_.entries.front() : *(it - 1);
  if (e.lost()) return std::nullopt;
  return e.delay;
}

double BottleneckLink::cross_rate_at(Nanos t) const {
  double rate = 0.0;
  for (const auto& p : params_.cross_traffic) {
    if (p.start > t) break;
    rate = p.cross_bps;
  }
  return rate;
}

std::optional<Nanos> BottleneckLink::transit(Nanos send_time, std::size_t bytes) {
  const double buffer = static_cast<double>(params_.buffer_bytes);
  if (last_time_ == INT64_MIN) last_time_ = send_time;

  // Advance the fluid queue from last_time_ to send_time, phase by phase.
  Nanos t = last_time_;
  while (t < send_time) {
    Nanos next = send_time;
    for (const auto& p : params_.cross_traffic) {
      if (p.start > t) {
        next = std::min(next, p.start);
        break;
      }
    }
    const double cross = cross_rate_at(t);
    backlog_ += (cross - params_.rate_bps) * to_seconds(next - t) / 8.0;
    if (backlog_ < 0.0) backlog_ = 0.0;
    if (backlog_ > buffer) backlog_ = buffer;
    t = next;
  }
  last_time_ = send_time;

  const double size = static_cast<double>(bytes);
  if (backlog_ + size > buffer) {
    // Drop-tail under sustained overload: the fluid model pins the queue at
    // the buffer size, but in a real queue space frees up packet by packet
    // and every flow loses roughly the excess fraction 1 - capacity/offered.
    const double offered = cross_rate_at(send_time);
    const double p_drop = offered > params_.rate_bps ? 1.0 - params_.rate_bps / offered : 1.0;
    if (rng_.bernoulli(p_drop)) return std::nullopt;
    // Admitted into the slot that just freed; it waits behind a full buffer.
    backlog_ = buffer - size;
  }
  backlog_ += size;
  const double queue_s = backlog_ * 8.0 / params_.rate_bps;
  return params_.propagation + from_seconds(queue_s);
}

}  // namespace tuushin::net
