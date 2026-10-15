#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

#include "tuushin/net/clock_sync.hpp"
#include "tuushin/net/trace.hpp"
#include "tuushin/time.hpp"
#include "tuushin/wire/packet.hpp"

namespace tuushin::net {

// Path measurement with probes and a reflector, the logic behind
// tools/tuushin_probe.cpp, kept free of sockets so it can be tested
// in-process.
//
// The client sends kProbe datagrams at a fixed rate; the reflector answers
// each with a kProbeEcho that carries its own sequence number, its send time
// (t3), the hold time (t3 - t2) and a receipt map. From these the client
// gets, per probe, the four NTP timestamps, the one-way delays in both
// directions after clock-skew removal (clock_sync.hpp), and which direction
// lost what. The result is two traces that tuushin-sim replays.

// Reflector state for any number of concurrent sessions.
class ProbeReflector {
 public:
  // Sessions idle for longer than this are forgotten.
  explicit ProbeReflector(Nanos idle_timeout = 30 * kNanosPerSecond, std::size_t max_sessions = 256)
      : idle_timeout_(idle_timeout), max_sessions_(max_sessions) {}

  struct Echo {
    wire::Header header;
    std::uint64_t receipts = 0;
    std::size_t payload = 0;  // same as the probe's: an echo is never larger
  };

  // Builds the answer to `probe`, received at `rx_time` and to be sent at
  // `tx_time` (reflector clock). Returns nullopt if the session table is
  // full of live sessions.
  std::optional<Echo> on_probe(const wire::Header& probe, Nanos rx_time, Nanos tx_time);

  std::size_t sessions() const { return sessions_.size(); }

 private:
  static constexpr std::uint32_t kWindow = 1024;  // probe seqs remembered per session

  struct Session {
    std::uint32_t echo_seq = 0;  // last sequence number this reflector sent
    std::uint32_t highest = 0;   // highest probe seq received
    std::vector<std::uint64_t> seen = std::vector<std::uint64_t>(kWindow / 64, 0);
    Nanos last_rx = 0;
    bool test(std::uint32_t seq) const;
    void mark(std::uint32_t seq);
  };

  void expire(Nanos now);

  Nanos idle_timeout_;
  std::size_t max_sessions_;
  Nanos last_expiry_ = 0;
  std::unordered_map<std::uint32_t, Session> sessions_;
};

// One probe as the client saw it.
struct ProbeRecord {
  Nanos t1 = 0;  // client send time
  bool echoed = false;
  Nanos t2 = 0, t3 = 0;  // reflector receive and send (reflector clock)
  Nanos t4 = 0;          // client receive time of the first echo
  std::uint32_t reflector_seq = 0;
  int duplicates = 0;              // further echoes for the same probe
  bool reflector_received = false;  // known from an echo or a receipt map
  bool covered = false;             // some receipt map spoke about this probe
  bool unsent = false;              // the send call failed, so the probe never left
};

// Client side: hands out probe headers and collects echoes.
class ProbeClient {
 public:
  explicit ProbeClient(std::uint32_t session) : session_(session) {}

  // Header of the next probe, sent at `t1`.
  wire::Header next_probe(Nanos t1);

  // Marks the probe last handed out by next_probe() as never sent: its send
  // call failed. Such a probe counts as neither sent nor lost.
  void mark_unsent();

  // Feeds a received datagram. Returns true if it was an echo of this
  // session's probes.
  bool on_datagram(const std::uint8_t* data, std::size_t size, Nanos t4);

  std::uint32_t session() const { return session_; }
  // Probes handed out by next_probe(), unsent ones included.
  std::size_t sent() const { return records_.size(); }
  std::size_t unsent() const { return unsent_; }
  std::size_t echoes() const { return echoes_; }
  std::size_t stray() const { return stray_; }
  // Indexed by probe seq - 1.
  const std::vector<ProbeRecord>& records() const { return records_; }

 private:
  std::uint32_t session_;
  std::vector<ProbeRecord> records_;
  std::size_t unsent_ = 0;
  std::size_t echoes_ = 0;
  std::size_t stray_ = 0;
};

struct DelayStats {
  std::size_t count = 0;
  double min = 0, p50 = 0, p90 = 0, p99 = 0, p999 = 0, max = 0;  // ns
  // RFC 3550 interarrival jitter (smoothed |D|, gain 1/16), ns.
  double rfc3550_jitter = 0;
};

// Statistics of `delays` (ns, any order). `arrivals` pairs (arrival time,
// transit) in any order for the RFC 3550 jitter, which runs in order of
// arrival; leave it empty to skip the jitter.
DelayStats delay_stats(std::vector<double> delays,
                       std::vector<std::pair<Nanos, double>> arrivals = {});

struct DirectionReport {
  std::size_t sent = 0;       // probes (forward) or echoes the reflector sent (backward)
  std::size_t lost = 0;
  std::size_t reordered = 0;  // arrivals earlier than a packet sent before them
  // Longest run of consecutive losses.
  std::size_t longest_burst = 0;
  DelayStats delay;           // one-way, after clock correction
  double loss_rate() const { return sent == 0 ? 0.0 : static_cast<double>(lost) / static_cast<double>(sent); }
};

struct ProbeReport {
  ClockModel clock;
  std::size_t probes = 0;     // probes sent, less the unsent and left-out ones
  std::size_t unsent = 0;     // probes whose send call failed
  std::size_t in_flight = 0;  // unanswered probes just before `until`, left out
  std::size_t duplicates = 0;
  // Probes with neither an echo nor a receipt map that mentions them (a run
  // of more than 64 lost echoes, or the end of the run). Counted as forward
  // losses, the cautious choice for a replay.
  std::size_t ambiguous = 0;
  // Probes the reflector received whose echo was lost: their forward delay
  // is interpolated between neighbours in the forward trace.
  std::size_t interpolated = 0;
  DelayStats rtt;
  DirectionReport fwd, bwd;
  Trace fwd_trace, bwd_trace;
};

// Analyses everything the client collected. `clock_window` is the window of
// fit_clock. Probes sent at or after `until` are left out, for a run cut
// short where the client's network went away; so are the unanswered probes
// leading up to `until`, back to the longest round trip seen, as they or
// their echoes may have been on their way when it went.
ProbeReport analyze_probes(const ProbeClient& client, Nanos clock_window = kNanosPerSecond,
                           Nanos until = INT64_MAX);

// One direction of a path whose two ends read the same clock: two sockets
// of one host, on two of its networks (tuushin-probe pair). Each delay is a
// receive time minus a send time on that clock, with no clock model and no
// assumption about the other direction. The sender side hands out kProbe
// headers; the receiver side feeds what arrives.
class OneClockDirection {
 public:
  explicit OneClockDirection(std::uint32_t session) : session_(session) {}

  struct Record {
    Nanos sent = 0;
    Nanos received = -1;  // first arrival; -1 if none
    int duplicates = 0;   // further arrivals of the same probe
    bool unsent = false;  // the send call failed, so the probe never left
  };

  // Header of the next probe, sent at `t`.
  wire::Header next(Nanos t);

  // Marks the probe last handed out by next() as never sent: its send call
  // failed. Such a probe counts as neither sent nor lost.
  void mark_unsent();

  // Feeds a datagram received at `t`. Returns true if it was one of this
  // direction's probes.
  bool on_datagram(const std::uint8_t* data, std::size_t size, Nanos t);

  std::uint32_t session() const { return session_; }
  // Probes handed out by next(), unsent ones included.
  std::size_t sent() const { return records_.size(); }
  std::size_t unsent() const { return unsent_; }
  std::size_t received() const { return received_; }
  std::size_t stray() const { return stray_; }
  // Indexed by probe seq - 1.
  const std::vector<Record>& records() const { return records_; }

 private:
  std::uint32_t session_;
  std::vector<Record> records_;
  std::size_t unsent_ = 0;
  std::size_t received_ = 0;
  std::size_t stray_ = 0;
};

struct OneClockReport {
  DirectionReport dir;      // sent and lost leave out the unsent probes
  std::size_t duplicates = 0;
  std::size_t unsent = 0;   // probes whose send call failed
  std::size_t in_flight = 0;  // losses just before `until`, left out
  Trace trace;              // no entry for an unsent or left-out probe
};

// Probes sent at or after `until` are left out of the report and the trace,
// for a run cut short where one of its ends went away: what the other end
// sent after that is not the path's loss. So is the run of losses leading
// up to `until`, back to the longest delay seen, as those probes may have
// been in flight to the end when it went.
OneClockReport analyze_one_clock(const OneClockDirection& d, Nanos until = INT64_MAX);

}  // namespace tuushin::net
