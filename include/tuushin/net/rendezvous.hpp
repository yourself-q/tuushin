#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "tuushin/net/udp.hpp"
#include "tuushin/time.hpp"

namespace tuushin::net {

// Rendezvous: how a master and a slave that are both behind NATs find each
// other and open a direct path.
//
// A NAT forwards a datagram from outside only to an inside socket that has
// recently sent to that outside address (most home routers; RFC 4787 calls
// it address- and port-dependent filtering), and the outside address of an
// inside socket is known only to whoever it sends to. Two ends behind NATs
// therefore need a third party that both can reach: each sends it a
// registration from the socket it will use for the session, the server
// sees the address the NAT gave that socket (its server-reflexive address,
// in ICE's terms) and tells each end the other's. Then both send to each
// other at the same time. The first datagram of each end opens its own
// NAT for the other's address, so whichever arrives after the other end's
// first one gets through. This is UDP hole punching as B. Ford, P.
// Srisuresh and D. Kegel describe it ("Peer-to-peer communication across
// network address translators", USENIX ATC 2005; RFC 5128), and the core of
// ICE (RFC 8445): gather candidate addresses, exchange them through a
// signalling channel, check which pairs connect.
//
// Simplified against ICE:
//  * The candidates are the server-reflexive address and the host's own
//    addresses (for two ends on one LAN, whose router may not hairpin).
//    There are no relayed candidates (TURN): where both NATs pick a new
//    outside port for every destination ("symmetric" NATs, endpoint-
//    dependent mapping), the address the server saw is useless to the
//    other end and the ends do not connect.
//  * The connectivity check is the haptic handshake itself. The master
//    sends its hello to every candidate of the slave and takes the first
//    reply that authenticates, from wherever it comes; the slave sends
//    small datagrams (wire::PacketType::kPunch) to every candidate of the
//    master, only to open its own NAT, and answers hellos where they come
//    from, as without a rendezvous. Authentication is therefore required:
//    it is what tells the slave's reply from anything else at a candidate.
//
// What the server learns: an identifier that both ends derive from their
// pre-shared key (crypto::rendezvous_id), not the key; the two ends'
// addresses; when they meet. It carries no haptic traffic. A server that
// lies can only make the ends send hellos to wrong addresses, which do not
// authenticate.
//
// Messages are fixed-size, so that the server's replies are never larger
// than the requests: a registration with a forged source address cannot
// make the server send more to that address than the forger sent (QUIC
// pads its first datagram for the same reason, RFC 9000 section 8.1).
//
// Format, version 1, little-endian like wire/packet.hpp:
//
//   offset size field
//        0    4 magic       'T' 'S' 'R' 'V'
//        4    1 version     1
//        5    1 type        RendezvousMessage::Type
//        6    1 role        RendezvousRole: the sender's in a registration,
//                           the registrant's echoed in kWaiting, the peer's
//                           in kPeer
//        7    1 count       candidates, 0..kRendezvousMaxCandidates
//        8    1 flags       bit 0 ready: the sender's in a registration,
//                           the peer's in kPeer, zero in kWaiting;
//                           bit 1 in replies: the recipient's own ready
//                           as the server holds it; the other bits zero
//        9    3 reserved    zero
//       12   16 id          the meeting's identifier
//       28    8 txid        the registration's, echoed in the replies
//       36   20 observed    replies: the address the registration came from
//       56 20*8 candidates  registration: the sender's host addresses;
//                           kPeer: the peer's, its observed address first
//
// An address is 20 bytes: family (0 none, 4, 6), a zero byte, the port,
// and 16 address bytes (IPv4 in the first 4). Unused slots are zero.
//
// The ready flag orders the first datagrams. A NAT that sees a datagram
// arrive for one of its outside ports before the inside socket has sent
// to that sender may keep a state of its own for the pair: Linux's
// conntrack does when the NAT host accepts datagrams to itself, which it
// does without a firewall of its own. When the inside socket then sends to
// the same address, the NAT finds the pair taken and gives the socket
// another outside port; the address the server announced is no longer the
// one the datagrams come from, and the other end's NAT drops them. Each
// datagram that crosses the internet must therefore find the receiving NAT
// already expecting it. So each end first opens its own NAT with datagrams
// that expire on the way, before they reach the other end's NAT (a small
// TTL, UdpSocket::send_limited), and then registers ready; it sends
// datagrams that go the whole way (hellos, punches) only once the server
// has passed the other end's ready on. This is the priming that M.
// Trautwein et al. propose against the failures they measured
// ("Challenging Tribal Knowledge: Large Scale Measurement Campaign on
// Decentralized NAT Traversal", 2025, section 7.2).
//
// An end's ready is about the other end's current registration: the server
// forgets it when the other end moves or registers anew (a new txid), and
// says so to the end in the recipient-ready bit.

using RendezvousId = std::array<std::uint8_t, 16>;

enum class RendezvousRole : std::uint8_t { kMaster = 1, kSlave = 2 };

constexpr std::size_t kRendezvousMaxCandidates = 8;
constexpr std::size_t kRendezvousMessageSize = 56 + 20 * kRendezvousMaxCandidates;

struct RendezvousMessage {
  enum class Type : std::uint8_t {
    kRegister = 1,  // end -> server
    kWaiting = 2,   // server -> end: registered, no peer yet
    kPeer = 3,      // server -> end: the peer's candidates
  };
  Type type = Type::kRegister;
  RendezvousRole role = RendezvousRole::kMaster;
  bool ready = false;
  bool recipient_ready = false;
  RendezvousId id{};
  std::uint64_t txid = 0;
  Endpoint observed;
  std::vector<Endpoint> candidates;  // canonical (Endpoint::canonical) forms
};

// Encodes `m` into exactly kRendezvousMessageSize bytes; candidates beyond
// the limit are dropped. Decoding returns nullopt for anything malformed and
// never throws.
std::vector<std::uint8_t> encode_rendezvous(const RendezvousMessage& m);
std::optional<RendezvousMessage> decode_rendezvous(const std::uint8_t* data, std::size_t size);

// The server's logic without a socket: registrations in, replies out.
//
// A meeting holds one registration per role. A registration is answered
// at once, with kPeer if the other role is registered and kWaiting
// otherwise; when it is new or its addresses changed, the other end is
// also sent kPeer, so that both start sending to each other within one
// round trip of the later one. A change of the ready flag is passed on to
// the other end the same way; a new address of one end clears the other
// end's ready, which was about the old address. Registrations expire after
// `expiry` without a refresh. When `capacity` meetings are held, the one
// seen longest ago makes room.
class RendezvousServer {
 public:
  struct Out {
    Endpoint to;
    std::vector<std::uint8_t> bytes;
  };

  explicit RendezvousServer(Nanos expiry = 30 * kNanosPerSecond, std::size_t capacity = 4096);

  // Handles one datagram from `from` at `now` and appends what to send.
  // Returns true for a registration that was new or whose addresses or
  // ready flag changed.
  bool on_datagram(const std::uint8_t* data, std::size_t size, const Endpoint& from, Nanos now,
                   std::vector<Out>& out);

  std::size_t meetings() const { return meetings_.size(); }

 private:
  struct End {
    bool present = false;
    Endpoint from;  // canonical
    std::vector<Endpoint> host;
    std::uint64_t txid = 0;
    bool ready = false;
    Nanos seen = 0;
  };
  struct Meeting {
    End ends[2];  // indexed by role - 1
    Nanos seen = 0;
  };
  static RendezvousMessage peer_message(const RendezvousId& id, const End& to, const End& peer,
                                        RendezvousRole peer_role);

  Nanos expiry_;
  std::size_t capacity_;
  std::map<RendezvousId, Meeting> meetings_;
};

// One end's side: registers with the server and learns the peer's
// candidates. It does no I/O; the caller sends what poll() returns from the
// socket it uses for the session (the NAT maps each socket separately) and
// hands it every datagram from the server.
class RendezvousClient {
 public:
  // `host` are this end's own addresses (host_addresses()).
  RendezvousClient(const Endpoint& server, const RendezvousId& id, RendezvousRole role,
                   std::vector<Endpoint> host);

  // Unanswered registrations are repeated every retry_interval, answered
  // ones every refresh_interval: often enough to keep the NAT's mapping to
  // the server alive (some time out after 30 s) and the registration from
  // expiring at the server.
  static constexpr Nanos kRetryInterval = 500 * kNanosPerMilli;
  static constexpr Nanos kRefreshInterval = 10 * kNanosPerSecond;

  // The registration to send at `now`, if one is due.
  std::optional<std::vector<std::uint8_t>> poll(Nanos now);
  // When poll() will next return a registration.
  Nanos next_due() const { return next_; }

  // Sets this end's ready flag; a change makes the next poll() due at once,
  // and registrations are then retried until the server confirms it. An
  // announcement that counts in peer_changes() clears it.
  void set_ready(bool ready);
  bool ready() const { return ready_; }

  // True if the datagram is the server's answer to this end's registration
  // (and was taken into account); false for anything else, which the caller
  // handles as without a rendezvous.
  bool on_datagram(const std::uint8_t* data, std::size_t size, const Endpoint& from, Nanos now);

  const Endpoint& server() const { return server_; }
  bool answered() const { return answered_; }
  // This end's server-reflexive address, once answered.
  const Endpoint& observed() const { return observed_; }
  // The peer's candidates, its server-reflexive address first; empty until
  // the server has announced a peer.
  const std::vector<Endpoint>& peer() const { return peer_; }
  // Counts the announcements after which this end should open its NAT
  // towards the peer again: a changed peer(), or a peer that registered
  // anew (the server then no longer holds this end ready, after it had
  // confirmed it).
  std::uint64_t peer_changes() const { return peer_changes_; }
  // The peer's ready flag, as last announced.
  bool peer_ready() const { return peer_ready_; }

 private:
  Endpoint server_;  // canonical
  RendezvousId id_;
  RendezvousRole role_;
  std::vector<Endpoint> host_;
  std::uint64_t txid_;
  Nanos next_ = 0;
  bool started_ = false;
  bool answered_ = false;
  bool ready_ = false;
  bool confirmed_ = true;  // the server holds ready_ as it is
  Endpoint observed_;
  std::vector<Endpoint> peer_;
  std::uint64_t peer_changes_ = 0;
  bool peer_ready_ = false;
};

// `candidates` in the form `socket` can send to (Endpoint::for_family),
// without duplicates; those it cannot reach at all are left out.
std::vector<Endpoint> sendable(const std::vector<Endpoint>& candidates, const UdpSocket& socket);

// Serves meetings on `socket` until `stop` becomes true (checked every
// 100 ms). `log`, if set, gets a line for every registration that is new or
// whose addresses changed.
void run_rendezvous_server(UdpSocket& socket, const std::atomic<bool>* stop,
                           const std::function<void(const std::string&)>& log = {},
                           Nanos expiry = 30 * kNanosPerSecond);

}  // namespace tuushin::net
