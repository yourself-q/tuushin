#pragma once

#include <string>

#include "tuushin/time.hpp"

namespace tuushin::rt {

// Asks the operating system to treat the calling thread as a periodic,
// latency-critical one, best effort and without special privileges:
//  - macOS: the Mach time-constraint policy (what Core Audio uses for audio
//    threads) with the given period. Without it, the kernel adds timer
//    leeway: measured on an M-series Mac, a 1 ms select() or nanosleep()
//    wakes about 205 us late at the median with or without the
//    USER_INTERACTIVE QoS class, and about 5 us late (p99 10 us) with the
//    policy. A thread that computes for longer than its declared share
//    without blocking is demoted by the kernel, so the policy fails safe.
//  - Linux: a timer slack of 1 ns instead of the default 50 us
//    (PR_SET_TIMERSLACK). Real-time scheduling (SCHED_FIFO) needs privileges
//    and can starve the machine if a loop misbehaves, so it is left to the
//    caller.
// Returns a short description of what was applied, for logs.
std::string prefer_low_latency(Nanos period = kNanosPerMilli);

}  // namespace tuushin::rt
