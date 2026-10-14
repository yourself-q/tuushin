#pragma once

#include <cstdint>

namespace tuushin {

// All timestamps and durations are signed nanoseconds. 64 bits cover about
// 292 years, so overflow is not a practical concern.
using Nanos = std::int64_t;

constexpr Nanos kNanosPerMilli = 1'000'000;
constexpr Nanos kNanosPerSecond = 1'000'000'000;

constexpr Nanos from_millis(double ms) { return static_cast<Nanos>(ms * 1e6); }
constexpr Nanos from_seconds(double s) { return static_cast<Nanos>(s * 1e9); }
constexpr double to_millis(Nanos ns) { return static_cast<double>(ns) / 1e6; }
constexpr double to_seconds(Nanos ns) { return static_cast<double>(ns) / 1e9; }

// Monotonic wall-clock time of this process. Not comparable across hosts.
Nanos monotonic_now();

}  // namespace tuushin
