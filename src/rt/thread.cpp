#include "tuushin/rt/thread.hpp"

#include <algorithm>
#include <cstdint>

#if defined(__APPLE__)
#include <mach/mach.h>
#include <mach/mach_time.h>
#include <mach/thread_policy.h>
#include <pthread.h>
#include <pthread/qos.h>
#elif defined(__linux__)
#include <sys/prctl.h>
#endif

namespace tuushin::rt {

std::string prefer_low_latency(Nanos period) {
#if defined(__APPLE__)
  if (period <= 0) period = kNanosPerMilli;
  mach_timebase_info_data_t tb{};
  mach_timebase_info(&tb);
  const auto ticks = [&](Nanos ns) {
    return static_cast<std::uint32_t>(static_cast<double>(ns) * tb.denom / tb.numer);
  };
  thread_time_constraint_policy_data_t p{};
  p.period = ticks(period);
  // Up to a fifth of the period of computation, finished within half of it.
  // The kernel refuses a computation under its least real-time quantum
  // (50 us accepted, 40 us refused on macOS 26), which a fifth of a short
  // period is: then that quantum, and a constraint no shorter.
  constexpr Nanos kLeastQuantum = 50 * 1000;
  const Nanos computation = std::max(period / 5, kLeastQuantum);
  p.computation = ticks(computation);
  p.constraint = ticks(std::max(period / 2, computation));
  p.preemptible = 1;
  const kern_return_t r =
      thread_policy_set(pthread_mach_thread_np(pthread_self()), THREAD_TIME_CONSTRAINT_POLICY,
                        reinterpret_cast<thread_policy_t>(&p), THREAD_TIME_CONSTRAINT_POLICY_COUNT);
  if (r == KERN_SUCCESS) return "mach time-constraint policy";
  if (pthread_set_qos_class_self_np(QOS_CLASS_USER_INTERACTIVE, 0) == 0) {
    return "qos user-interactive (time-constraint policy refused)";
  }
  return "scheduling unchanged";
#elif defined(__linux__)
  (void)period;
  if (prctl(PR_SET_TIMERSLACK, 1UL, 0UL, 0UL, 0UL) == 0) return "timer slack 1 ns";
  return "timer slack unchanged";
#else
  (void)period;
  return "scheduling unchanged";
#endif
}

}  // namespace tuushin::rt
