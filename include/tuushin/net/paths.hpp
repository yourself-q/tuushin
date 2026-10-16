#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "tuushin/net/tcp.hpp"
#include "tuushin/net/tls.hpp"
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

// How this end reaches its TURN server: in datagrams, or over a TCP
// connection where UDP does not get out, or over TLS on TCP where only
// what looks like HTTPS does (RFC 8656, 3.1).
enum class RelayTransport { kUdp, kTcp, kTls };

// This end's ways to its peers: its socket, and optionally a relay on a
// TURN server (turn.hpp), allocated from a second socket of its own, since
// the server knows an allocation by the address it sees the client at and
// takes every datagram from there for TURN's; or over a TCP connection of
// its own (TLS on it, optionally), which the server knows the allocation
// by. The connection is
// opened again when it drops (TurnClient's timers), and what arrives on it
// is cut into TURN messages here.
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
// pick a new outside port for every destination (rendezvous.hpp). Over
// TCP a lost segment holds back what follows it until it is sent again
// (a retransmission timeout of 200 ms or more), and the arrival time of a
// datagram is when it was read, not when the kernel took it.
class Paths {
 public:
  explicit Paths(UdpSocket& direct);
  Paths(const Paths&) = delete;
  Paths& operator=(const Paths&) = delete;

  // Allocates a relay on `server` with long-term credentials, from a new
  // socket bound to the direct socket's address with an ephemeral port and
  // marked with its DSCP, since it carries the same datagrams; over TCP the
  // connections are made from such sockets, and over TLS they check the
  // server by `tls`. Throws if that socket cannot be bound or cannot send
  // to the server, or (std::runtime_error) if TLS cannot be set up
  // (TlsConnector).
  void enable_relay(const Endpoint& server, const std::string& username,
                    const std::string& password,
                    RelayTransport transport = RelayTransport::kUdp,
                    const TlsSettings& tls = {});

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

  // Waits until either socket has a datagram queued, the connection to the
  // TURN server has bytes or takes the ones waiting, or `timeout` passes.
  bool wait_readable(Nanos timeout);

  // Sends what the TURN client has due at `now`, and over TCP opens,
  // closes and writes the connection; call it by next_due() and after
  // every wait.
  void service(Nanos now);
  // When service() next has something to send; far in the future without
  // a relay.
  Nanos next_due() const;

  // Gives the peers' addresses permissions on the relay (their ports do not
  // matter), so that what they send to the relayed address is passed on.
  // Does nothing without a relay.
  void permit(const std::vector<Endpoint>& peers);

  // The connection to the TURN server, while one is open or opening.
  Stream* stream() { return stream_.get(); }
  // How the TURN server is reached, if a relay is enabled.
  RelayTransport relay_transport() const { return relay_transport_; }

 private:
  bool streamed() const { return turn_ && turn_->transport() == TurnClient::Transport::kStream; }
  void tend_stream(Nanos now);
  void close_stream();
  // The length of the TURN message at the front of in_: 0 while it has not
  // all arrived, nullopt if what is there is not one (a TLS alert from a
  // TLS port, say). Not all that is not TURN shows: an HTTP answer reads as
  // ChannelData of 20 KiB or so, and the requests that get no answer give
  // the connection up instead.
  std::optional<std::size_t> next_message() const;
  std::optional<std::size_t> recv_stream(std::uint8_t* buffer, std::size_t capacity, Route* from,
                                         Nanos* rx_time);

  UdpSocket& direct_;
  std::optional<UdpSocket> relay_socket_;
  std::optional<TurnClient> turn_;
  Endpoint server_;  // in the form relay_socket_ or a stream sends to
  RelayTransport relay_transport_ = RelayTransport::kUdp;
  std::vector<std::uint8_t> wrapped_;
  std::vector<std::uint8_t> rx_;
  std::function<std::unique_ptr<Stream>()> open_stream_;
  std::unique_ptr<Stream> stream_;
  bool stream_open_ = false;           // told the TURN client
  std::size_t in_begin_ = 0, in_end_ = 0;  // what was read and not handled, in rx_
  Nanos in_time_ = 0;                  // when it was read
};

}  // namespace tuushin::net
