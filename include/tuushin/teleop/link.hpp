#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <queue>
#include <vector>

#include "tuushin/crypto/noise.hpp"
#include "tuushin/net/channel.hpp"
#include "tuushin/net/clock_sync.hpp"
#include "tuushin/net/udp.hpp"
#include "tuushin/teleop/controller.hpp"
#include "tuushin/time.hpp"

namespace tuushin::teleop {

// Sequence-number bookkeeping of one receiving direction: duplicates,
// reordering and the count of distinct samples, over a sliding window.
class SeqWindow {
 public:
  enum class Verdict {
    kNew,        // first time seen, newer than everything before
    kLate,       // first time seen, but a higher seq arrived earlier
    kDuplicate,  // seen before
    kStale,      // older than the window; cannot tell, treated as a duplicate
  };
  Verdict accept(std::uint32_t seq);
  std::uint32_t highest() const { return highest_; }
  std::uint64_t distinct() const { return distinct_; }
  // Samples the sender must have sent but never arrived, assuming it numbers
  // from 1 without gaps.
  std::uint64_t missing() const { return highest_ > distinct_ ? highest_ - distinct_ : 0; }

 private:
  static constexpr std::uint32_t kWindow = 4096;
  bool test(std::uint32_t seq) const;
  void set(std::uint32_t seq);
  std::uint32_t highest_ = 0;
  std::uint64_t distinct_ = 0;
  std::vector<std::uint64_t> bits_ = std::vector<std::uint64_t>(kWindow / 64, 0);
};

// One end of a haptic session over UDP: frames teleop::Samples as kHaptic
// datagrams (wire/packet.hpp), numbers them, fills in the echo fields so the
// other end can measure the round trip from ordinary traffic, and collects
// the four timestamps of every exchange for a clock fit afterwards.
//
// With session keys (set_keys) every datagram is sealed (wire::kFlagSealed)
// and only datagrams that authenticate are accepted. The sequence window and
// the echo then only ever see authenticated sequence numbers, so a forged
// datagram can neither inject a sample nor, by claiming a sequence number
// far ahead, make every genuine one look stale.
//
// A sealed link also follows its peer to a new address, as WireGuard does
// (Donenfeld, NDSS 2017, section 2.1): a datagram from elsewhere that
// authenticates and is the newest yet makes its source the peer, so a
// session survives a NAT that moves the other end to another port, or an
// end that changes network. Being the newest keeps a replayed datagram
// from moving the link; a party on the path that forwards a fresh copy
// faster than the original moves it until the next genuine datagram, but
// could drop the traffic anyway. An open link stays with the address it
// was made with.
//
// An optional ChannelModel impairs outgoing datagrams in-process before they
// reach the socket, so that a path with a given delay, jitter, loss or a
// recorded trace can be tried on loopback or a LAN. The datagram keeps its
// original send time, so the impairment shows up in every measurement as if
// the network had caused it.
class HapticLink {
 public:
  // `scheme` goes into every datagram (wire::kSchemeMask).
  HapticLink(net::UdpSocket& socket, std::uint32_t session, const net::Endpoint& peer,
             Passivity scheme);

  void set_emulation(std::unique_ptr<net::ChannelModel> model) { emulation_ = std::move(model); }

  // Seals every datagram sent from now on with keys.send and accepts only
  // datagrams sealed with keys.recv. The sequence numbers are the nonces;
  // they never repeat within a link, and keys from a handshake are never
  // used by two links. Call it before the first send.
  void set_keys(const crypto::SessionKeys& keys);
  bool sealed() const { return sealed_; }

  // Sends a sample at `now`. `monitor` is the sender's net device-port energy
  // (J), carried for the other end's passivity bookkeeping across both
  // ports; `final` marks the end of the session.
  void send(Nanos now, const Sample& s, double monitor, bool final = false);

  // Releases emulated datagrams that are due at `now`.
  void pump(Nanos now);
  // When the next emulated datagram is due, if any is queued.
  std::optional<Nanos> next_release() const;

  struct Received {
    std::uint32_t seq = 0;
    Sample sample;
    double monitor = 0.0;
    Nanos send_time = 0;  // sender's clock
    Nanos rx_time = 0;
    bool final = false;
    SeqWindow::Verdict verdict = SeqWindow::Verdict::kNew;
  };
  // Parses a datagram from `from` received at `rx_time`. Returns the sample
  // if it belongs to this session and peer and is not a duplicate; on a
  // sealed link the peer may be at a new address (see above).
  std::optional<Received> on_datagram(const std::uint8_t* data, std::size_t size,
                                      const net::Endpoint& from, Nanos rx_time);

  struct Counters {
    std::uint64_t sent = 0;           // samples handed to send()
    std::uint64_t send_failed = 0;    // the kernel refused the datagram
    std::uint64_t emulated_lost = 0;  // dropped by the emulation
    std::uint64_t received = 0;       // distinct samples of this session
    std::uint64_t late = 0;           // arrived after a higher seq
    std::uint64_t duplicates = 0;
    std::uint64_t foreign = 0;        // malformed, other session or peer, wrong scheme
    std::uint64_t unauthentic = 0;    // this session and peer, but not sealed or
                                      // failing authentication (sealed links)
    std::uint64_t missing = 0;        // never arrived (so far)
    std::uint64_t moved = 0;          // the peer came from a new address
  };
  Counters counters() const;

  // Round-trip times measured from the echo fields (ns), one per sample sent
  // here that was echoed, and the exchanges behind them, in arrival order
  // (sort by t1 before fit_clock).
  const std::vector<double>& rtts() const { return rtts_; }
  const std::vector<net::TimestampSample>& exchanges() const { return exchanges_; }

  std::uint32_t session() const { return session_; }
  const net::Endpoint& peer() const { return peer_; }

  // Caps the number of exchanges kept (each 32 bytes), so that a run of
  // hours stays bounded; later exchanges are dropped.
  void set_exchange_limit(std::size_t n) { exchange_limit_ = n; }

 private:
  void transmit(const std::vector<std::uint8_t>& datagram);

  net::UdpSocket& socket_;
  std::uint32_t session_;
  net::Endpoint peer_;
  Passivity scheme_;
  std::uint32_t next_seq_ = 1;
  std::vector<std::uint8_t> buf_;
  bool sealed_ = false;
  crypto::SessionKeys keys_;
  std::vector<std::uint8_t> plain_;
  Counters counters_;
  SeqWindow window_;
  Nanos highest_rx_time_ = 0;  // arrival of the highest seq received

  // Send times of our recent samples, for the echo.
  static constexpr std::size_t kHistory = 8192;
  struct Sent {
    std::uint32_t seq = 0;
    Nanos t = 0;
    bool echoed = false;
  };
  std::vector<Sent> history_ = std::vector<Sent>(kHistory);
  std::vector<double> rtts_;
  std::vector<net::TimestampSample> exchanges_;
  std::size_t exchange_limit_ = std::size_t{1} << 21;

  struct Pending {
    Nanos release;
    std::uint64_t order;
    std::vector<std::uint8_t> bytes;
  };
  struct LaterRelease {
    bool operator()(const Pending& a, const Pending& b) const {
      if (a.release != b.release) return a.release > b.release;
      return a.order > b.order;
    }
  };
  std::unique_ptr<net::ChannelModel> emulation_;
  std::priority_queue<Pending, std::vector<Pending>, LaterRelease> pending_;
  std::uint64_t pending_order_ = 0;
};

}  // namespace tuushin::teleop
