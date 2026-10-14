#pragma once

#include <cstddef>
#include <cstdint>

#include "tuushin/time.hpp"
#include "tuushin/vec.hpp"

namespace tuushin::codec {

// Perceptual deadband parameters.
//
// The scheme follows Hinterseer, Hirche, Chaudhuri, Steinbach and Buss,
// "Perception-based data reduction and transmission of haptic data in
// telepresence and teleaction systems", IEEE Trans. Signal Processing 56(2),
// 2008: a sample is sent only when it differs from the last sent sample by
// more than a fixed fraction of that sample's magnitude (Weber's law). The
// receiver holds the last value. This is not a conformance claim for
// IEEE 1918.1.1; it is the classic scheme the standard builds on.
struct DeadbandParams {
  // Weber fraction. 0.1 (10 %) is the value used throughout the TUM papers.
  double k = 0.1;
  // Absolute threshold in signal units. Without it a signal near zero would
  // trigger on every bit of sensor noise, because k * |x| -> 0.
  double abs_floor = 0.0;
  // Upper bound on the time between two transmissions (0 disables it). A
  // periodic refresh lets the receiver tell "unchanged" from "link down" and
  // keeps piggybacked state such as passivity energies fresh.
  Nanos max_interval = 0;
};

// Decides which samples of a signal have to be transmitted.
template <std::size_t N>
class DeadbandEncoder {
 public:
  explicit DeadbandEncoder(DeadbandParams params = {}) : params_(params) {}

  // Feeds the sample taken at time `t`. Returns true when it must be sent;
  // in that case it becomes the new reference value.
  bool update(const Vec<N>& x, Nanos t) {
    if (!has_sent_ || exceeds(x) ||
        (params_.max_interval > 0 && t - last_time_ >= params_.max_interval)) {
      last_sent_ = x;
      last_time_ = t;
      has_sent_ = true;
      ++sent_count_;
      return true;
    }
    return false;
  }

  // Forgets the reference so the next sample is sent unconditionally.
  void reset() { has_sent_ = false; }

  const Vec<N>& last_sent() const { return last_sent_; }
  std::uint64_t sent_count() const { return sent_count_; }
  const DeadbandParams& params() const { return params_; }
  void set_params(const DeadbandParams& p) { params_ = p; }

 private:
  bool exceeds(const Vec<N>& x) const {
    const double threshold = params_.k * norm(last_sent_);
    const double limit = threshold > params_.abs_floor ? threshold : params_.abs_floor;
    return norm(x - last_sent_) > limit;
  }

  DeadbandParams params_;
  Vec<N> last_sent_{};
  Nanos last_time_ = 0;
  bool has_sent_ = false;
  std::uint64_t sent_count_ = 0;
};

}  // namespace tuushin::codec
