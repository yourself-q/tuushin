#include "tuushin/net/paths.hpp"

#include <algorithm>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <system_error>

namespace tuushin::net {

std::string Route::to_string() const {
  return peer.canonical().to_string() + (relayed ? " through the relay" : "");
}

Paths::Paths(UdpSocket& direct) : direct_(direct) {}

void Paths::enable_relay(const Endpoint& server, const std::string& username,
                         const std::string& password, RelayTransport transport) {
  const Endpoint local = direct_.local_endpoint();
  const auto address = local.address();
  const auto bind = Endpoint::from_address(address.data(), address.size(), 0);
  if (!bind) throw std::runtime_error("Paths: the direct socket has no address to bind next to");
  const auto to = server.for_family(direct_.is_ipv6());
  if (!to) throw std::runtime_error("Paths: cannot reach the TURN server " + server.to_string() +
                                    " from an IPv4 socket");
  server_ = *to;
  if (transport == RelayTransport::kUdp) {
    UdpSocket s(*bind);
    if (direct_.dscp() >= 0) s.set_dscp(direct_.dscp());
    s.enable_rx_timestamps();
    relay_socket_.emplace(std::move(s));
    turn_.emplace(server, username, password);
    // A datagram of up to 64 KiB, which is what a TURN message can carry.
    rx_.assign(std::size_t{1} << 16, 0);
    return;
  }
  open_stream_ = [to = *to, from = *bind, dscp = direct_.dscp()]() -> std::unique_ptr<Stream> {
    return std::make_unique<TcpStream>(to, from, dscp);
  };
  turn_.emplace(server, username, password, TurnClient::Transport::kStream);
  // Room for the largest message (20 + 65535 bytes) after a partial one.
  rx_.assign(std::size_t{1} << 17, 0);
  service(monotonic_now());
}

bool Paths::send(const Route& to, const std::uint8_t* data, std::size_t size) {
  if (!to.relayed) return direct_.send_to(to.peer, data, size);
  if (!turn_ || !turn_->wrap(to.peer, data, size, wrapped_)) return false;
  // The permission and the channel that wrap() asked for go first: the
  // server drops what is sent to a peer before its permission is in place.
  service(monotonic_now());
  if (streamed()) return stream_ && stream_->write(wrapped_.data(), wrapped_.size());
  return relay_socket_->send_to(server_, wrapped_.data(), wrapped_.size());
}

std::optional<std::size_t> Paths::recv(std::uint8_t* buffer, std::size_t capacity, Route* from,
                                       Nanos* rx_time) {
  if (const auto n = direct_.recv_from(buffer, capacity, &from->peer, rx_time)) {
    from->relayed = false;
    return n;
  }
  if (streamed()) return recv_stream(buffer, capacity, from, rx_time);
  if (!relay_socket_) return std::nullopt;
  for (;;) {
    Endpoint source;
    Nanos rx = 0;
    const auto n = relay_socket_->recv_from(rx_.data(), rx_.size(), &source, &rx);
    if (!n) return std::nullopt;
    if (*n > rx_.size()) continue;
    Endpoint peer;
    std::size_t offset = 0, length = 0;
    const auto r = turn_->on_datagram(rx_.data(), *n, source, rx, &peer, &offset, &length);
    if (r != TurnClient::Rx::kData) {
      // A response may have made the next request due at once (after a
      // 401, the request with credentials).
      service(monotonic_now());
      continue;
    }
    std::memcpy(buffer, rx_.data() + offset, std::min(length, capacity));
    from->peer = peer;
    from->relayed = true;
    if (rx_time) *rx_time = rx;
    return length > capacity ? capacity + 1 : length;
  }
}

std::optional<std::size_t> Paths::next_message() const {
  const std::size_t have = in_end_ - in_begin_;
  if (have < 4) return 0;
  const std::uint8_t* p = rx_.data() + in_begin_;
  const std::size_t length = static_cast<std::size_t>((p[2] << 8) | p[3]);
  std::size_t whole = 0;
  if ((p[0] >> 6) == 0) {
    // STUN: the header, with a length that is a multiple of 4 and the magic
    // cookie (as far as it has arrived), then the attributes.
    if (length % 4 != 0) return std::nullopt;
    for (std::size_t i = 4; i < 8 && i < have; ++i) {
      if (p[i] != static_cast<std::uint8_t>(stun::kMagicCookie >> (8 * (7 - i)))) {
        return std::nullopt;
      }
    }
    whole = 20 + length;
  } else if ((p[0] & 0xf0) == 0x40) {
    // ChannelData on a channel 0x4000-0x4FFF, padded (RFC 8656, 12.5).
    whole = 4 + ((length + 3) & ~std::size_t{3});
  } else {
    return std::nullopt;
  }
  return have >= whole ? whole : 0;
}

std::optional<std::size_t> Paths::recv_stream(std::uint8_t* buffer, std::size_t capacity,
                                              Route* from, Nanos* rx_time) {
  for (;;) {
    const auto whole = next_message();
    if (!whole) {
      turn_->disconnected("the server sent what is not TURN", monotonic_now());
      close_stream();
      return std::nullopt;
    }
    if (*whole > 0) {
      const std::uint8_t* m = rx_.data() + in_begin_;
      in_begin_ += *whole;
      Endpoint peer;
      std::size_t offset = 0, length = 0;
      const auto r =
          turn_->on_datagram(m, *whole, turn_->server(), in_time_, &peer, &offset, &length);
      if (r != TurnClient::Rx::kData) {
        service(monotonic_now());
        continue;
      }
      std::memcpy(buffer, m + offset, std::min(length, capacity));
      from->peer = peer;
      from->relayed = true;
      if (rx_time) *rx_time = in_time_;
      return length > capacity ? capacity + 1 : length;
    }
    if (!stream_) return std::nullopt;
    if (in_begin_ > 0) {
      std::memmove(rx_.data(), rx_.data() + in_begin_, in_end_ - in_begin_);
      in_end_ -= in_begin_;
      in_begin_ = 0;
    }
    const long n = stream_->read(rx_.data() + in_end_, rx_.size() - in_end_);
    if (n == 0) return std::nullopt;
    if (n < 0) {
      turn_->disconnected(stream_->error(), monotonic_now());
      close_stream();
      return std::nullopt;
    }
    in_end_ += static_cast<std::size_t>(n);
    in_time_ = monotonic_now();
  }
}

bool Paths::wait_readable(Nanos timeout) {
  // A message read with an earlier one and not yet handed out.
  if (stream_ && next_message() != std::optional<std::size_t>{0}) return true;
  WaitFor waits[3];
  waits[0].fd = direct_.native_handle();
  if (relay_socket_) waits[1].fd = relay_socket_->native_handle();
  if (stream_) waits[2] = WaitFor{stream_->native_handle(), stream_->wants_write()};
  return wait_any(waits, 3, timeout);
}

void Paths::close_stream() {
  stream_.reset();
  stream_open_ = false;
  in_begin_ = in_end_ = 0;
}

void Paths::tend_stream(Nanos now) {
  // A connection the client gave up on (it stopped answering), or has no
  // more use for.
  if (stream_ && !turn_->needs_connection()) close_stream();
  if (stream_) {
    const Stream::State state = stream_->state();
    if (state == Stream::State::kClosed) {
      turn_->disconnected(stream_->error(), now);
      close_stream();
    } else if (state == Stream::State::kOpen && !stream_open_) {
      stream_open_ = true;
      turn_->connected(now);
    }
  }
  if (!stream_ && turn_->wants_connection(now)) {
    turn_->connecting(now);
    try {
      stream_ = open_stream_();
    } catch (const std::system_error& e) {
      turn_->disconnected(e.what(), now);
      return;
    }
    // A connection on the same host may be open, or refused, at once.
    tend_stream(now);
  }
}

void Paths::service(Nanos now) {
  if (!turn_) return;
  if (!streamed()) {
    while (const auto m = turn_->poll(now)) relay_socket_->send_to(server_, m->data(), m->size());
    return;
  }
  tend_stream(now);
  while (const auto m = turn_->poll(now)) {
    // Taken whole or not at all; a request that is not is answered by no
    // one, which gives the connection up.
    if (stream_) stream_->write(m->data(), m->size());
  }
  // poll() may have given the connection up, and may want a new one.
  tend_stream(now);
  if (stream_) stream_->flush();
}

Nanos Paths::next_due() const {
  return turn_ ? turn_->next_due() : std::numeric_limits<Nanos>::max();
}

void Paths::permit(const std::vector<Endpoint>& peers) {
  if (!turn_) return;
  for (const auto& p : peers) turn_->permit(p);
}

}  // namespace tuushin::net
