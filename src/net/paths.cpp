#include "tuushin/net/paths.hpp"

#include <algorithm>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace tuushin::net {

std::string Route::to_string() const {
  return peer.canonical().to_string() + (relayed ? " through the relay" : "");
}

Paths::Paths(UdpSocket& direct) : direct_(direct) {}

void Paths::enable_relay(const Endpoint& server, const std::string& username,
                         const std::string& password) {
  const Endpoint local = direct_.local_endpoint();
  const auto address = local.address();
  const auto bind = Endpoint::from_address(address.data(), address.size(), 0);
  if (!bind) throw std::runtime_error("Paths: the direct socket has no address to bind next to");
  UdpSocket s(*bind);
  if (direct_.dscp() >= 0) s.set_dscp(direct_.dscp());
  const auto to = server.for_family(s.is_ipv6());
  if (!to) throw std::runtime_error("Paths: cannot reach the TURN server " + server.to_string() +
                                    " from an IPv4 socket");
  s.enable_rx_timestamps();
  relay_socket_.emplace(std::move(s));
  server_ = *to;
  turn_.emplace(server, username, password);
  // A datagram of up to 64 KiB, which is what a TURN message can carry.
  rx_.assign(std::size_t{1} << 16, 0);
}

bool Paths::send(const Route& to, const std::uint8_t* data, std::size_t size) {
  if (!to.relayed) return direct_.send_to(to.peer, data, size);
  if (!turn_ || !turn_->wrap(to.peer, data, size, wrapped_)) return false;
  // The permission and the channel that wrap() asked for go first: the
  // server drops what is sent to a peer before its permission is in place.
  service(monotonic_now());
  return relay_socket_->send_to(server_, wrapped_.data(), wrapped_.size());
}

std::optional<std::size_t> Paths::recv(std::uint8_t* buffer, std::size_t capacity, Route* from,
                                       Nanos* rx_time) {
  if (const auto n = direct_.recv_from(buffer, capacity, &from->peer, rx_time)) {
    from->relayed = false;
    return n;
  }
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

bool Paths::wait_readable(Nanos timeout) {
  UdpSocket* const sockets[2] = {&direct_, relay_socket_ ? &*relay_socket_ : nullptr};
  return wait_any_readable(sockets, 2, timeout);
}

void Paths::service(Nanos now) {
  if (!turn_) return;
  while (const auto m = turn_->poll(now)) relay_socket_->send_to(server_, m->data(), m->size());
}

Nanos Paths::next_due() const {
  return turn_ ? turn_->next_due() : std::numeric_limits<Nanos>::max();
}

void Paths::permit(const std::vector<Endpoint>& peers) {
  if (!turn_) return;
  for (const auto& p : peers) turn_->permit(p);
}

}  // namespace tuushin::net
