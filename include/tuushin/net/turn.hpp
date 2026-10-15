#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "tuushin/crypto/legacy_digest.hpp"
#include "tuushin/net/stun.hpp"
#include "tuushin/net/udp.hpp"
#include "tuushin/time.hpp"

namespace tuushin::net {

// A TURN client (RFC 8656) over UDP: an address on a relay server that
// forwards datagrams between this end and its peers.
//
// Hole punching (rendezvous.hpp) fails where both ends sit behind NATs that
// pick a new outside port for every destination (endpoint-dependent
// mapping, "symmetric" NATs): the port the rendezvous server saw is not the
// one the other end would be let in on. A relay sidesteps the NATs'
// mappings. This end allocates an address on the relay (the relayed
// address) from its own socket; the relay forwards to this end whatever
// reaches the relayed address from a peer this end has given a permission,
// and sends from the relayed address what this end asks it to. To the peer
// the relayed address is an ordinary public address, so it needs no TURN
// client of its own: it sends its hello there as to any other candidate.
//
// Permissions are per IP address, without the port (RFC 8656, 9). That is
// what makes the relay work against a symmetric NAT at the peer: the port
// the peer's NAT picks for the relay is unknown in advance, but the address
// is the one the rendezvous server saw (NATs keep one outside address per
// inside host, RFC 4787 REQ-2).
//
// Data goes in Send and Data indications (36 bytes of overhead for IPv4)
// until a channel is bound to the peer (RFC 8656, 12), then in ChannelData
// messages (4 bytes). The client binds a channel to every peer it sends to.
//
// Like RendezvousClient it does no I/O: the caller sends what poll()
// returns to server() from the socket it keeps for the relay, hands every
// datagram from there to on_datagram(), and wraps what it sends to peers
// with wrap(). The socket must be used for nothing else: the relay knows
// the allocation by the socket's address as the server sees it.
//
// Authentication is the long-term credential mechanism (RFC 8489, 9.2)
// with MESSAGE-INTEGRITY (HMAC-SHA1), which is what coturn accepts: the
// first Allocate goes without credentials, the server answers 401 with a
// realm and a nonce, and every request after that carries them and a MAC
// keyed with MD5(username:realm:password). Responses to authenticated
// requests are accepted only with a valid MAC, except 401 and 438, which
// cannot carry one; a response that fails is dropped as if lost, and the
// request is repeated (RFC 8489, 9.2.5). The data path is not
// authenticated by TURN: indications and ChannelData carry no MAC, and the
// haptic handshake above authenticates the peer instead.
//
// Timers:
//  * Requests are repeated after 0.5, 1, 2, 4, 8 and 16 s and given up 8 s
//    after the seventh (RFC 8489, 6.2.1).
//  * A Refresh every kKeepalive, much more often than the allocation's
//    lifetime (10 minutes by default) requires, because the NAT in front
//    of this end may forget the mapping to the server within 30 s of
//    silence (RFC 8656, 9, note), and a NAT that maps the socket anew moves
//    the allocation out of reach. The Refresh is also how a lost
//    allocation shows: the server answers 437 when it no longer knows this
//    end's address (it restarted, the allocation expired, this end's NAT
//    moved it), and the client allocates again. allocations() counts the
//    allocations, so that the caller sees the relayed address change.
//  * Permissions last 5 minutes and channels 10 (RFC 8656, 9 and 12); the
//    client refreshes them after 4 and 5.
class TurnClient {
 public:
  TurnClient(const Endpoint& server, std::string username, std::string password);

  static constexpr Nanos kInitialRto = 500 * kNanosPerMilli;
  static constexpr int kMaxTransmissions = 7;
  static constexpr Nanos kKeepalive = 15 * kNanosPerSecond;
  static constexpr Nanos kPermissionRefresh = 240 * kNanosPerSecond;
  static constexpr Nanos kChannelRefresh = 300 * kNanosPerSecond;
  // After an Allocate that got no answer, or one the server could not
  // serve for now (437, 486, 508).
  static constexpr Nanos kRetryDelay = 10 * kNanosPerSecond;

  // The next message to send to server() at `now`, if one is due; call it
  // until it returns nullopt. Also due after on_datagram() and wrap().
  std::optional<std::vector<std::uint8_t>> poll(Nanos now);
  // When poll() will next have something to send.
  Nanos next_due() const;

  enum class Rx {
    kNotTurn,  // not from the server: the caller's to handle
    kControl,  // the server's, handled here (or dropped)
    kData,     // a peer's datagram, relayed: *peer and the payload are set
  };
  // Handles one datagram that arrived from `from`. For kData the payload is
  // data[*offset .. *offset + *length) and came from *peer.
  Rx on_datagram(const std::uint8_t* data, std::size_t size, const Endpoint& from, Nanos now,
                 Endpoint* peer, std::size_t* offset, std::size_t* length);

  // Wraps a datagram for `peer` into what to send to server(): ChannelData
  // once a channel to the peer is bound, a Send indication before. Asks for
  // a permission and a channel for the peer if it has none (permit(),
  // bind_channel()); the relay drops what is sent before the permission is
  // in place. Returns false, leaving `out` empty, if there is no allocation
  // or the server refused a permission for the peer's address.
  bool wrap(const Endpoint& peer, const std::uint8_t* data, std::size_t size,
            std::vector<std::uint8_t>& out);

  // Asks for a permission for `peer`'s IP address (its port does not
  // matter), and keeps it refreshed. Idempotent. The server may refuse
  // addresses it will not relay to (coturn refuses loopback by default,
  // and is often told to refuse private ranges); the client then stops
  // asking for that address until the next allocation.
  void permit(const Endpoint& peer);
  // Asks for a channel to `peer` (address and port), and its permission.
  // Idempotent.
  void bind_channel(const Endpoint& peer);
  // Finds out at once whether the server still knows the allocation: a
  // Refresh now, unless one is on its way, and an Allocate now if one is
  // waiting to be tried again. For when the caller has reason to think
  // this end's NAT has moved it (its peers fell silent, another server
  // sees this end at a new address), so that a lost allocation shows
  // within a round trip, not at the next keepalive.
  void check(Nanos now);
  // Deletes the allocation (a Refresh with lifetime 0); poll() sends it and
  // then nothing more.
  void release();

  const Endpoint& server() const { return server_; }
  bool allocated() const { return state_ == State::kAllocated; }
  // True once the server has refused the credentials or the allocation for
  // good; poll() then sends nothing.
  bool failed() const { return state_ == State::kFailed; }
  // Why the last allocation failed or was lost, for logs; empty if none.
  const std::string& last_error() const { return last_error_; }
  // The relayed address of the current or last allocation.
  const Endpoint& relayed() const { return relayed_; }
  // This end's address as the server sees it (XOR-MAPPED-ADDRESS).
  const Endpoint& mapped() const { return mapped_; }
  // Allocations made so far; a change means a new relayed address.
  std::uint64_t allocations() const { return allocations_; }
  // True once the server has confirmed a permission for `peer`'s address.
  bool permitted(const Endpoint& peer) const;
  // True once the server has confirmed a channel to `peer`.
  bool channel_bound(const Endpoint& peer) const;

 private:
  enum class State { kStart, kAllocating, kAllocated, kWaiting, kReleasing, kReleased, kFailed };
  enum class Kind { kAllocate, kRefresh, kCreatePermission, kChannelBind };

  struct Transaction {
    Kind kind;
    stun::TransactionId tid{};
    std::vector<std::uint8_t> bytes;
    bool authenticated = false;  // sent with MESSAGE-INTEGRITY
    Nanos due = 0;               // of the next transmission, or the timeout
    int sent = 0;
    int restarts = 0;  // after 401 or 438
    Endpoint peer;     // CreatePermission, ChannelBind
    std::uint16_t channel = 0;
    std::uint32_t lifetime = 0;  // Refresh: 0 deletes; kNoLifetime omits it
  };
  struct Permission {
    std::vector<std::uint8_t> ip;
    Endpoint peer;  // one address with the IP, for XOR-PEER-ADDRESS
    bool in_flight = false;
    bool confirmed = false;
    bool refused = false;
    Nanos due = 0;
  };
  struct Channel {
    Endpoint peer;
    std::uint16_t number = 0;
    bool in_flight = false;
    bool confirmed = false;
    bool refused = false;
    Nanos due = 0;
  };
  static constexpr std::uint32_t kNoLifetime = 0xffffffff;

  void start(Kind kind, const Endpoint& peer, std::uint16_t channel, std::uint32_t lifetime);
  void build(Transaction& t);
  std::vector<std::uint8_t> transmit(Transaction& t, Nanos now);
  void on_response(std::size_t index, const stun::Message& m, Nanos now);
  void on_success(const Transaction& t, const stun::Message& m, Nanos now);
  void on_error(Transaction t, const stun::Message& m, Nanos now);
  void on_timeout(const Transaction& t, Nanos now);
  void restart(Transaction t);
  void lose(const std::string& why);
  void fail(const std::string& why);
  bool has(Kind kind) const;
  Permission* find_permission(const Endpoint& peer);
  const Permission* find_permission(const Endpoint& peer) const;
  Channel* find_channel(const Endpoint& peer);
  const Channel* find_channel(const Endpoint& peer) const;

  Endpoint server_;  // canonical
  std::string username_;
  std::string password_;
  std::string realm_;
  std::string nonce_;
  crypto::Md5Digest key_{};
  bool have_credentials_ = false;  // realm_ and nonce_ known

  State state_ = State::kStart;
  Nanos retry_at_ = 0;
  Nanos refresh_at_ = 0;
  Nanos refresh_interval_ = kKeepalive;
  std::string last_error_;
  Endpoint relayed_;
  Endpoint mapped_;
  std::uint64_t allocations_ = 0;
  std::vector<Transaction> transactions_;
  std::vector<Permission> permissions_;
  std::vector<Channel> channels_;
  std::uint16_t next_channel_ = 0x4000;
};

}  // namespace tuushin::net
