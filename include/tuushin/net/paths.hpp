#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "tuushin/net/turn.hpp"
#include "tuushin/net/udp.hpp"
#include "tuushin/time.hpp"

namespace tuushin::net {

// Which way a datagram goes to a peer, or came from one: the peer's address,
// and whether the datagram passes through this end's TURN relay. In ICE's
// terms (RFC 8445) the flag names the local candidate of the pair: this
// end's own address or its relayed one. The address alone does not say: a
// peer behind a NAT that maps independently of the destination is at the
// same address both ways.
struct Route {
  Endpoint peer;
  bool relayed = false;

  friend bool operator==(const Route& a, const Route& b) {
    return a.relayed == b.relayed && a.peer == b.peer;
  }
  friend bool operator!=(const Route& a, const Route& b) { return !(a == b); }
  // The peer's canonical address, with " through the relay" if relayed.
  std::string to_string() const;
};

// This end's ways to its peers: its socket, and optionally a relay on a
// TURN server (turn.hpp), allocated from a second socket of its own, since
// the server knows an allocation by the address it sees the client at and
// takes every datagram from there for TURN's.
//
// What arrives either way is handed out with the Route it came by; what the
// TURN client receives for itself (its responses) it keeps. A datagram for
// a relayed route is wrapped for the server: a Send indication at first,
// ChannelData once a channel to the peer is bound, which the client asks
// for on the first datagram to the peer, together with a permission for
// the peer's address.
//
// A relay costs one hop through the server and 4 bytes per datagram, and is
// for where a direct path cannot be opened: both ends behind NATs that
// pick a new outside port for every destination (rendezvous.hpp).
class Paths {
 public:
  explicit Paths(UdpSocket& direct);
  Paths(const Paths&) = delete;
  Paths& operator=(const Paths&) = delete;

  // Allocates a relay on `server` with long-term credentials, from a new
  // socket bound to the direct socket's address with an ephemeral port and
  // marked with its DSCP, since it carries the same datagrams. Throws if
  // that socket cannot be bound or cannot send to the server.
  void enable_relay(const Endpoint& server, const std::string& username,
                    const std::string& password);

  UdpSocket& direct() { return direct_; }
  // The TURN client, if a relay is enabled.
  TurnClient* turn() { return turn_ ? &*turn_ : nullptr; }
  const TurnClient* turn() const { return turn_ ? &*turn_ : nullptr; }

  // Sends one datagram along `to`. False if it could not be handed to the
  // kernel, or, for a relayed route, if there is no allocation or the
  // server refused the peer (TurnClient::wrap).
  bool send(const Route& to, const std::uint8_t* data, std::size_t size);

  // Receives one datagram from either way if one is queued, as
  // UdpSocket::recv_from: its size (capacity + 1 if it did not fit) or
  // nullopt. The TURN client's own datagrams are taken on the way, and
  // what they make due is sent.
  std::optional<std::size_t> recv(std::uint8_t* buffer, std::size_t capacity, Route* from,
                                   Nanos* rx_time = nullptr);

  // Waits until either socket has a datagram queued or `timeout` passes.
  bool wait_readable(Nanos timeout);

  // Sends what the TURN client has due at `now`; call it by next_due().
  void service(Nanos now);
  // When service() next has something to send; far in the future without
  // a relay.
  Nanos next_due() const;

  // Gives the peers' addresses permissions on the relay (their ports do not
  // matter), so that what they send to the relayed address is passed on.
  // Does nothing without a relay.
  void permit(const std::vector<Endpoint>& peers);

 private:
  UdpSocket& direct_;
  std::optional<UdpSocket> relay_socket_;
  std::optional<TurnClient> turn_;
  Endpoint server_;  // in the form relay_socket_ sends to
  std::vector<std::uint8_t> wrapped_;
  std::vector<std::uint8_t> rx_;
};

}  // namespace tuushin::net
