#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "tuushin/crypto/noise.hpp"
#include "tuushin/net/channel.hpp"
#include "tuushin/net/clock_sync.hpp"
#include "tuushin/net/paths.hpp"
#include "tuushin/net/probe.hpp"
#include "tuushin/net/rendezvous.hpp"
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
// Either end ends a session by setting wire::kFlagFinal; from then on both
// ends switch their actuators off and stop their controllers, so that their
// energy books no longer change, and keep sending final samples carrying
// them for a short linger. A slave that hears nothing for idle_timeout also
// returns to idle, with its actuator off.

// Fixed-rate schedule of a control loop.
class Ticker {
 public:
  Ticker(Nanos period, Nanos first) : period_(period), next_(first) {}
  Nanos due() const { return next_; }
  Nanos period() const { return period_; }
  // Records a tick taken at `now` and schedules the next one on the grid. A
  // loop that fell a period or more behind (the process was descheduled)
  // drops the ticks it missed instead of running them back to back: a
  // control sample is about the present, a burst of them would only send
  // the same state several times, and samples microseconds apart would
  // look to the controllers like a device that stopped.
  void taken(Nanos now);
  std::uint64_t ticks() const { return ticks_; }
  std::uint64_t skipped() const { return skipped_; }
  // Lateness of each tick (ns), for statistics; capped at about 4 M entries.
  const std::vector<double>& lateness() const { return lateness_; }

 private:
  Nanos period_;
  Nanos next_;
  std::uint64_t ticks_ = 0;
  std::uint64_t skipped_ = 0;
  std::vector<double> lateness_;
};

// Meeting the other end through a rendezvous server (net/rendezvous.hpp),
// for two ends that are both behind NATs. It needs a pre-shared key: the
// handshake is what tells the slave's reply from anything else at its
// candidate addresses.
struct RendezvousConfig {
  net::Endpoint server;
  net::RendezvousId id{};  // crypto::rendezvous_id(psk)
  // Master: how long to wait for the server to announce a slave.
  Nanos wait = 30 * kNanosPerSecond;
  // Both: the hop limit of the datagrams that open this end's NAT towards
  // the other end's candidates without reaching them (net/rendezvous.hpp).
  // They pass the routers before the last one they reach: 3 passes a home
  // router and the next hop (a carrier-grade NAT, often), and expires
  // before the other end's NAT wherever the two are more than 3 hops apart,
  // as two homes nearly always are. Where the system cannot limit the hop
  // count, the first datagrams that go the whole way open the NAT instead,
  // as without priming. 0 primes nothing.
  int prime_hops = 3;
  // Both: before registering, find the hop limit instead (net/hops.hpp):
  // the hop count to the first router past this end's NATs on the way to
  // the server, which passes them all, however many there are, and stops
  // before the other end's NAT, however close that is (unless it lies on
  // the way to the server). prime_hops stands where nothing could be measured: on systems
  // other than Linux, towards a server at a private or an IPv6 address, or
  // where no router past the NATs answers. It takes a round trip to that
  // router, half a second at most.
  bool measure_hops = true;
  // Both: how long to wait for the other end's ready before sending
  // datagrams that go the whole way regardless.
  Nanos ready_wait = 2 * kNanosPerSecond;
  // Slave: how often, and for how long after the master is ready, to send
  // punches to all its candidates: datagrams that go the whole way, for
  // NATs further away than prime_hops. The master's hellos come every
  // hello_interval, so a punch interval well below it lets the second hello
  // through at the latest.
  Nanos punch_interval = 50 * kNanosPerMilli;
  Nanos punch_duration = 5 * kNanosPerSecond;
  // Both, during a session: after rejoin_after without an authentic sample
  // from the other end, register again at once, and every retry interval
  // while the silence lasts. If a NAT or a new network has moved an end,
  // the server sees its new address and announces it to the other end,
  // which punches towards it for rejoin_punch, so that its own NAT lets the
  // moved end's datagrams in; the link then follows the moved end
  // (HapticLink). Both ends look for a move because neither can tell which
  // one moved: each only hears nothing. The punches are copies of the
  // link's last datagram (HapticLink::send_copy), which authenticate: where
  // the punching end's NAT sends them from a port of their own (it gives
  // every destination one), the moved end's link follows the punching end
  // there in turn, and answers to that port.
  Nanos rejoin_after = 100 * kNanosPerMilli;
  Nanos rejoin_punch = kNanosPerSecond;
  // Master: how long after its first hellos to the slave's own candidates
  // it also sends hellos to the slave's relayed address (SlaveNodeConfig::
  // relay). A direct path that opens answers within a round trip or two of
  // the hellos, before the relay is tried; the master takes the first
  // reply that authenticates, and the session stays on its path.
  Nanos relay_delay = kNanosPerSecond;
};

// A relay on a TURN server (net/turn.hpp), with long-term credentials.
struct RelayConfig {
  net::Endpoint server;
  std::string username;
  std::string password;
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
  // Pre-shared key; without one the session is not authenticated.
  std::optional<crypto::Key> psk;
  Nanos hello_interval = 200 * kNanosPerMilli;
  // From the first hello; with a rendezvous, from when the server announced
  // the slave.
  Nanos handshake_timeout = 5 * kNanosPerSecond;
  // Find the slave through a rendezvous server; needs psk.
  std::optional<RendezvousConfig> rendezvous;
};

struct MasterNodeReport {
  std::uint32_t session = 0;
  net::Endpoint slave;            // where the session ran to
  bool authenticated = false;     // the handshake completed; datagrams sealed
  bool handshake_failed = false;  // no reply authenticated in time; no session ran
  bool rendezvous_failed = false;  // the server announced no slave in time
  net::Endpoint observed;          // this end's address as the server saw it
  std::vector<net::Endpoint> candidates;  // the slave's, as the server announced them
  std::vector<net::Endpoint> relays;      // the slave's relayed addresses, likewise
  int prime_hops = 0;      // the hop limit of the priming (RendezvousConfig::measure_hops)
  std::string prime_how;   // how it was found: measured (the routers), or why not
  bool relayed = false;  // the session ran to one of the relays
  std::uint64_t rejoins = 0;  // the slave announced anew during the session
  std::uint64_t ticks = 0, skipped = 0;
  net::DelayStats lateness;  // ns
  HapticLink::Counters link;
  net::DelayStats rtt;       // ns, from the echo fields
  net::ClockModel clock;     // slave clock - master clock
  net::DelayStats forward, backward;  // one-way delays after the clock fit, ns
  bool slave_answered = false;
  bool slave_final = false;     // the slave's final books arrived
  bool ended_by_slave = false;  // the slave ended the session (it was stopped)
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

// The handshake's prologue: what both ends must agree on, taken from the
// hello's header, so that a header changed on the way fails the handshake.
std::vector<std::uint8_t> handshake_prologue(std::uint32_t session, Passivity scheme);

// The payload of the hello (handshake message 1, which encrypts and
// authenticates it): the master's wall-clock time in ns since the Unix
// epoch, little-endian. A slave answers only a hello newer than the newest
// it has answered, so a recorded hello is useless once a newer one has
// arrived; WireGuard's initiation carries a TAI64N timestamp for the same
// reason (Donenfeld, NDSS 2017, section 5.1). A master whose clock is set
// back is refused until it passes the newest time the slave has seen.
constexpr std::size_t kHelloPayloadSize = 8;
std::vector<std::uint8_t> hello_payload(std::uint64_t wall_ns);
std::optional<std::uint64_t> hello_time(const std::vector<std::uint8_t>& payload);

// Runs a session against the slave at `slave` until cfg.duration elapses or
// `stop` becomes true. `device` is sampled every tick; `emulation`, if set,
// impairs the master-to-slave datagrams in-process. With cfg.rendezvous,
// `slave` may be left invalid: the hellos go to every candidate the server
// announces (and to `slave` if valid), and the session runs to wherever the
// first reply that authenticates came from.
MasterNodeReport run_master_node(net::UdpSocket& socket, const net::Endpoint& slave,
                                 Device1D& device, const MasterNodeConfig& cfg,
                                 std::unique_ptr<net::ChannelModel> emulation = nullptr,
                                 const std::atomic<bool>* stop = nullptr);

struct SlaveNodeConfig {
  // control.passivity is the scheme the slave runs; sessions announcing
  // another one are refused unless adopt_scheme is set. A slave that
  // reports the contact force refuses kReflective sessions, which cannot
  // work with it (sim::SlaveParams), and does not start with that scheme
  // unless it adopts the master's.
  SlaveConfig control;
  bool adopt_scheme = false;
  Nanos idle_timeout = kNanosPerSecond;
  Nanos final_linger = 30 * kNanosPerMilli;
  // Serve one session, then return.
  bool once = false;
  // Per-tick CSV log for each session: the session id (hex) and ".csv" are
  // appended to this prefix. Empty for none.
  std::string log_prefix;
  // Pre-shared key: only sessions that complete the handshake run.
  std::optional<crypto::Key> psk;
  // How long an answered hello waits for the master's first sealed sample.
  Nanos handshake_timeout = 5 * kNanosPerSecond;
  // Hellos at or before this wall-clock time (ns, as in hello_payload) are
  // not answered; the newest answered one raises it as the node runs.
  std::uint64_t newest_hello = 0;
  // Register with a rendezvous server while idle, and punch towards every
  // master it announces; needs psk.
  std::optional<RendezvousConfig> rendezvous;
  // Allocate a relay and announce its address through the rendezvous, for
  // masters that cannot reach this end directly (net/paths.hpp); needs
  // rendezvous. Every master the server announces gets a permission on the
  // relay for its addresses. The allocation is kept for as long as the
  // node runs.
  std::optional<RelayConfig> relay;
};

struct SlaveSessionReport {
  std::uint32_t session = 0;
  net::Endpoint master;
  bool relayed = false;  // the session started through this end's relay
  Passivity scheme = Passivity::kNone;
  bool authenticated = false;
  std::uint64_t ticks = 0, skipped = 0;
  net::DelayStats lateness;
  HapticLink::Counters link;
  bool ended_by_final = false;  // otherwise by idle timeout or stop
  double slave_net = 0.0;
  double max_position = 0.0;
  double max_force = 0.0;
  double slave_pc_active = 0.0;
  std::uint64_t rejoins = 0;  // the master announced anew during the session
};

struct SlaveNodeHooks {
  // A fresh impairment model for each session's slave-to-master datagrams,
  // given the session's start time; may return nullptr.
  std::function<std::unique_ptr<net::ChannelModel>(Nanos start)> emulation;
  // Called when a session ends.
  std::function<void(const SlaveSessionReport&)> on_session_end;
  // Called when a session or a handshake is refused (scheme mismatch, a key
  // on one end only, a hello that does not authenticate), with a reason.
  std::function<void(const std::string&)> on_refused;
  // Called with what the rendezvous server said (this end's address as it
  // saw it, and every master it announced) and what became of the relay.
  std::function<void(const std::string&)> on_rendezvous;
};

// Serves sessions one after the other until `stop` becomes true, or after
// one if cfg.once. Returns the reports of all sessions.
std::vector<SlaveSessionReport> run_slave_node(net::UdpSocket& socket, Device1D& device,
                                               const SlaveNodeConfig& cfg,
                                               const SlaveNodeHooks& hooks = {},
                                               const std::atomic<bool>* stop = nullptr);

}  // namespace tuushin::teleop
