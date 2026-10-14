#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "tuushin/net/channel.hpp"
#include "tuushin/net/clock_sync.hpp"
#include "tuushin/net/probe.hpp"
#include "tuushin/net/udp.hpp"
#include "tuushin/teleop/controller.hpp"
#include "tuushin/teleop/device.hpp"
#include "tuushin/teleop/link.hpp"
#include "tuushin/time.hpp"

namespace tuushin::teleop {

// Real-time master and slave nodes: one process per end, each running its
// controller at a fixed rate against its device and exchanging samples with
// the other end over UDP (HapticLink). The ends run on their own clocks;
// nothing assumes they are synchronized.
//
// Session: the master picks a random session id and starts sending; the
// slave adopts the first session that reaches it while idle and answers the
// address it came from, which also works behind a NAT on the master's side.
// The master ends a session by setting wire::kFlagFinal; the slave answers
// with its own final samples, which carry its final energy books, and
// returns to idle. A slave that hears nothing for idle_timeout also returns
// to idle, with its actuator off.

// Fixed-rate schedule of a control loop.
class Ticker {
 public:
  Ticker(Nanos period, Nanos first) : period_(period), next_(first) {}
  Nanos due() const { return next_; }
  Nanos period() const { return period_; }
  // Records a tick taken at `now` and schedules the next one. A loop that
  // fell more than max_behind periods behind (the process was descheduled)
  // drops the ticks it missed instead of running them back to back: a
  // control sample is about the present, and a burst of them would only send
  // the same state several times.
  void taken(Nanos now);
  std::uint64_t ticks() const { return ticks_; }
  std::uint64_t skipped() const { return skipped_; }
  // Lateness of each tick (ns), for statistics; capped at about 4 M entries.
  const std::vector<double>& lateness() const { return lateness_; }

  static constexpr int kMaxBehind = 10;

 private:
  Nanos period_;
  Nanos next_;
  std::uint64_t ticks_ = 0;
  std::uint64_t skipped_ = 0;
  std::vector<double> lateness_;
};

struct MasterNodeConfig {
  MasterConfig control;
  Nanos duration = 10 * kNanosPerSecond;
  // 0 picks a random session id.
  std::uint32_t session = 0;
  // After the slave's final sample, keep running this long to collect the
  // rest of its final samples; give up waiting for it after final_timeout.
  Nanos final_linger = 30 * kNanosPerMilli;
  Nanos final_timeout = kNanosPerSecond;
  // Per-tick CSV log, empty for none.
  std::string log_path;
};

struct MasterNodeReport {
  std::uint32_t session = 0;
  std::uint64_t ticks = 0, skipped = 0;
  net::DelayStats lateness;  // ns
  HapticLink::Counters link;
  net::DelayStats rtt;       // ns, from the echo fields
  net::ClockModel clock;     // slave clock - master clock
  net::DelayStats forward, backward;  // one-way delays after the clock fit, ns
  bool slave_answered = false;
  bool slave_final = false;  // the slave's final books arrived
  double master_net = 0.0;   // J into the teleoperator at the master port
  double slave_net = 0.0;    // J at the slave port, last reported
  // Minimum over time of master_net + slave_net with the slave's reports
  // placed on the master's clock: the passivity margin of the whole
  // teleoperator as in sim::Teleop1DResult, up to the slave's report rate
  // and the clock fit's error.
  double min_margin = 0.0;
  double max_position = 0.0;  // m
  double max_force = 0.0;     // N
  double master_pc_active = 0.0;
};

// Runs a session against the slave at `slave` until cfg.duration elapses or
// `stop` becomes true. `device` is sampled every tick; `emulation`, if set,
// impairs the master-to-slave datagrams in-process.
MasterNodeReport run_master_node(net::UdpSocket& socket, const net::Endpoint& slave,
                                 Device1D& device, const MasterNodeConfig& cfg,
                                 std::unique_ptr<net::ChannelModel> emulation = nullptr,
                                 const std::atomic<bool>* stop = nullptr);

struct SlaveNodeConfig {
  // control.passivity is the scheme the slave runs; sessions announcing
  // another one are refused unless adopt_scheme is set.
  SlaveConfig control;
  bool adopt_scheme = false;
  Nanos idle_timeout = kNanosPerSecond;
  Nanos final_linger = 30 * kNanosPerMilli;
  // Serve one session, then return.
  bool once = false;
  // Per-tick CSV log for each session: the session id (hex) and ".csv" are
  // appended to this prefix. Empty for none.
  std::string log_prefix;
};

struct SlaveSessionReport {
  std::uint32_t session = 0;
  net::Endpoint master;
  Passivity scheme = Passivity::kNone;
  std::uint64_t ticks = 0, skipped = 0;
  net::DelayStats lateness;
  HapticLink::Counters link;
  bool ended_by_final = false;  // otherwise by idle timeout or stop
  double slave_net = 0.0;
  double max_position = 0.0;
  double max_force = 0.0;
  double slave_pc_active = 0.0;
};

struct SlaveNodeHooks {
  // A fresh impairment model for each session's slave-to-master datagrams,
  // given the session's start time; may return nullptr.
  std::function<std::unique_ptr<net::ChannelModel>(Nanos start)> emulation;
  // Called when a session ends.
  std::function<void(const SlaveSessionReport&)> on_session_end;
  // Called when a session is refused (scheme mismatch), with a reason.
  std::function<void(const std::string&)> on_refused;
};

// Serves sessions one after the other until `stop` becomes true, or after
// one if cfg.once. Returns the reports of all sessions.
std::vector<SlaveSessionReport> run_slave_node(net::UdpSocket& socket, Device1D& device,
                                               const SlaveNodeConfig& cfg,
                                               const SlaveNodeHooks& hooks = {},
                                               const std::atomic<bool>* stop = nullptr);

}  // namespace tuushin::teleop
