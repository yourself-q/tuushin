#pragma once

#include <poll.h>

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "tuushin/net/stun.hpp"
#include "tuushin/net/udp.hpp"

namespace test {

namespace net = tuushin::net;
namespace stun = tuushin::net::stun;

// A TURN server (RFC 8656) in miniature, for tests on IPv4 loopback: UDP,
// long-term credentials with one user, Allocate, Refresh, CreatePermission,
// ChannelBind, Send and Data indications and ChannelData. Permissions and
// channels never expire, and the allocation's lifetime is not kept: tests
// that need expiry use drop_allocations(). What a client sees of coturn is
// modelled where tuushin depends on it: 401 with a realm and a nonce, 438
// for a nonce that is no longer current, 437 for a request other than
// Allocate from an address without an allocation, MESSAGE-INTEGRITY on
// every response to an authenticated request.
class SimTurn {
 public:
  SimTurn(std::string username, std::string password, std::string realm = "tuushin.test")
      : username_(std::move(username)), realm_(std::move(realm)),
        key_(stun::long_term_key(username_, realm_, password)),
        socket_(*net::Endpoint::resolve("127.0.0.1", 0)), endpoint_(socket_.local_endpoint()) {}
  ~SimTurn() {
    stop_.store(true);
    if (thread_.joinable()) thread_.join();
  }
  SimTurn(const SimTurn&) = delete;
  SimTurn& operator=(const SimTurn&) = delete;

  const net::Endpoint& endpoint() const { return endpoint_; }
  void start() {
    thread_ = std::thread([this] { run(); });
  }

  // A new nonce: requests with the old one get 438.
  void rotate_nonce() { nonce_generation_.fetch_add(1); }
  // Forgets every allocation, like a server that restarted; returns once
  // the server's thread has.
  void drop_allocations() {
    drop_.store(true);
    while (drop_.load()) std::this_thread::yield();
  }
  // Answers CreatePermission and ChannelBind with 403 from now on.
  void refuse_peers(bool refuse) { refuse_.store(refuse); }
  // Drops everything that arrives, like a path that went down.
  void pause(bool paused) { paused_.store(paused); }

  std::uint64_t allocations() const { return allocations_.load(); }
  std::uint64_t challenges() const { return challenges_.load(); }      // 401
  std::uint64_t stale_nonces() const { return stale_nonces_.load(); }  // 438
  std::uint64_t refreshes() const { return refreshes_.load(); }
  std::uint64_t permissions() const { return permissions_.load(); }
  std::uint64_t channel_binds() const { return channel_binds_.load(); }
  // Datagrams relayed to peers, from Send indications and from ChannelData.
  std::uint64_t sent_indications() const { return sent_indications_.load(); }
  std::uint64_t sent_channel_data() const { return sent_channel_data_.load(); }
  // Datagrams from peers relayed to clients, as Data indications and as
  // ChannelData.
  std::uint64_t data_indications() const { return data_indications_.load(); }
  std::uint64_t channel_data() const { return channel_data_.load(); }
  std::uint64_t live_allocations() const { return live_.load(); }

 private:
  struct Allocation {
    net::Endpoint client;
    stun::TransactionId tid{};
    net::UdpSocket relay;
    net::Endpoint relayed;
    std::vector<std::vector<std::uint8_t>> permitted;  // IPs
    std::vector<std::pair<std::uint16_t, net::Endpoint>> channels;
  };

  static std::uint16_t get16(const std::uint8_t* p) {
    return static_cast<std::uint16_t>((p[0] << 8) | p[1]);
  }

  std::string nonce() const { return "nonce-" + std::to_string(nonce_generation_.load()); }

  Allocation* find(const net::Endpoint& client) {
    for (auto& a : allocations_list_) {
      if (a->client == client) return a.get();
    }
    return nullptr;
  }
  static bool permitted(const Allocation& a, const net::Endpoint& peer) {
    const auto ip = peer.address();
    return std::find(a.permitted.begin(), a.permitted.end(), ip) != a.permitted.end();
  }
  static void permit(Allocation& a, const net::Endpoint& peer) {
    if (!permitted(a, peer)) a.permitted.push_back(peer.address());
  }

  void reply(const net::Endpoint& to, stun::Builder& b, bool integrity) {
    if (integrity) b.add_integrity(key_.data(), key_.size());
    b.add_fingerprint();
    socket_.send_to(to, b.bytes().data(), b.bytes().size());
  }
  void error(const net::Endpoint& to, const stun::Message& m, int code, bool integrity) {
    stun::Builder b(m.method(), stun::Class::kError, m.transaction());
    b.add_error(code, code == 401 ? "Unauthorized" : code == 438 ? "Stale Nonce" : "Error");
    if (code == 401 || code == 438) {
      b.add_string(stun::attr::kRealm, realm_);
      b.add_string(stun::attr::kNonce, nonce());
    }
    reply(to, b, integrity);
  }

  void on_request(const net::Endpoint& from, const stun::Message& m) {
    Allocation* a = find(from);
    if (m.method() == stun::method::kAllocate && a && a->tid != m.transaction()) {
      error(from, m, 437, false);  // as coturn: before authentication
      return;
    }
    if (!m.has(stun::attr::kMessageIntegrity)) {
      challenges_.fetch_add(1);
      error(from, m, 401, false);
      return;
    }
    if (m.string(stun::attr::kUsername) != username_ || m.string(stun::attr::kRealm) != realm_ ||
        !m.check_integrity(key_.data(), key_.size())) {
      challenges_.fetch_add(1);
      error(from, m, 401, false);
      return;
    }
    if (m.string(stun::attr::kNonce) != nonce()) {
      stale_nonces_.fetch_add(1);
      error(from, m, 438, false);
      return;
    }

    stun::Builder ok(m.method(), stun::Class::kSuccess, m.transaction());
    switch (m.method()) {
      case stun::method::kAllocate: {
        if (!a) {
          net::UdpSocket relay(*net::Endpoint::resolve("127.0.0.1", 0));
          const net::Endpoint relayed = relay.local_endpoint();
          allocations_list_.push_back(std::make_unique<Allocation>(
              Allocation{from, m.transaction(), std::move(relay), relayed, {}, {}}));
          a = allocations_list_.back().get();
          allocations_.fetch_add(1);
          live_.store(allocations_list_.size());
        }
        ok.add_address(stun::attr::kXorRelayedAddress, a->relayed);
        ok.add_address(stun::attr::kXorMappedAddress, from);
        ok.add_u32(stun::attr::kLifetime, 600);
        break;
      }
      case stun::method::kRefresh: {
        if (!a) return error(from, m, 437, true);
        refreshes_.fetch_add(1);
        const auto lifetime = m.u32(stun::attr::kLifetime);
        if (lifetime && *lifetime == 0) {
          allocations_list_.erase(
              std::find_if(allocations_list_.begin(), allocations_list_.end(),
                           [&](const auto& x) { return x.get() == a; }));
          live_.store(allocations_list_.size());
          ok.add_u32(stun::attr::kLifetime, 0);
        } else {
          ok.add_u32(stun::attr::kLifetime, 600);
        }
        break;
      }
      case stun::method::kCreatePermission: {
        if (!a) return error(from, m, 437, true);
        const auto peer = m.address(stun::attr::kXorPeerAddress);
        if (!peer) return error(from, m, 400, true);
        if (refuse_.load()) return error(from, m, 403, true);
        permit(*a, *peer);
        permissions_.fetch_add(1);
        break;
      }
      case stun::method::kChannelBind: {
        if (!a) return error(from, m, 437, true);
        const auto peer = m.address(stun::attr::kXorPeerAddress);
        const auto number = m.u32(stun::attr::kChannelNumber);
        if (!peer || !number) return error(from, m, 400, true);
        const std::uint16_t n = static_cast<std::uint16_t>(*number >> 16);
        if (n < 0x4000 || n > 0x4fff) return error(from, m, 400, true);
        if (refuse_.load()) return error(from, m, 403, true);
        bool found = false;
        for (const auto& c : a->channels) {
          if ((c.first == n) != (c.second == *peer)) return error(from, m, 400, true);
          found = found || c.first == n;
        }
        if (!found) a->channels.emplace_back(n, *peer);
        permit(*a, *peer);
        channel_binds_.fetch_add(1);
        break;
      }
      default:
        return error(from, m, 400, true);
    }
    reply(from, ok, true);
  }

  void on_client(const net::Endpoint& from, const std::uint8_t* data, std::size_t size) {
    if (size >= 4 && (data[0] & 0xf0) == 0x40) {
      Allocation* a = find(from);
      const std::uint16_t n = get16(data);
      const std::size_t len = get16(data + 2);
      if (!a || len > size - 4) return;
      for (const auto& c : a->channels) {
        if (c.first == n && permitted(*a, c.second)) {
          a->relay.send_to(c.second, data + 4, len);
          sent_channel_data_.fetch_add(1);
        }
      }
      return;
    }
    const auto m = stun::Message::parse(data, size);
    if (!m || !m->fingerprint_ok()) return;
    if (m->cls() == stun::Class::kRequest) return on_request(from, *m);
    if (m->cls() == stun::Class::kIndication && m->method() == stun::method::kSend) {
      Allocation* a = find(from);
      const auto peer = m->address(stun::attr::kXorPeerAddress);
      std::size_t n = 0;
      const std::uint8_t* payload = m->find(stun::attr::kData, &n);
      if (!a || !peer || !payload || !permitted(*a, *peer)) return;
      a->relay.send_to(*peer, payload, n);
      sent_indications_.fetch_add(1);
    }
  }

  void on_peer(Allocation& a, const net::Endpoint& peer, const std::uint8_t* data, std::size_t size) {
    if (!permitted(a, peer)) return;
    for (const auto& c : a.channels) {
      if (c.second == peer) {
        std::vector<std::uint8_t> out(4 + size);
        out[0] = static_cast<std::uint8_t>(c.first >> 8);
        out[1] = static_cast<std::uint8_t>(c.first);
        out[2] = static_cast<std::uint8_t>(size >> 8);
        out[3] = static_cast<std::uint8_t>(size);
        std::copy(data, data + size, out.begin() + 4);
        socket_.send_to(a.client, out.data(), out.size());
        channel_data_.fetch_add(1);
        return;
      }
    }
    stun::Builder b(stun::method::kData, stun::Class::kIndication, stun::random_transaction_id());
    b.add_address(stun::attr::kXorPeerAddress, peer);
    b.add(stun::attr::kData, data, size);
    socket_.send_to(a.client, b.bytes().data(), b.bytes().size());
    data_indications_.fetch_add(1);
  }

  void run() {
    std::vector<std::uint8_t> buf(65536);
    std::vector<pollfd> fds;
    while (!stop_.load()) {
      if (drop_.exchange(false)) {
        allocations_list_.clear();
        live_.store(0);
      }
      fds.clear();
      fds.push_back(pollfd{socket_.native_handle(), POLLIN, 0});
      for (const auto& a : allocations_list_) {
        fds.push_back(pollfd{a->relay.native_handle(), POLLIN, 0});
      }
      if (::poll(fds.data(), static_cast<nfds_t>(fds.size()), 1) <= 0) continue;
      net::Endpoint from;
      // Peers first: a Refresh with lifetime 0 below may remove allocations.
      const std::size_t count = allocations_list_.size();
      for (std::size_t i = 0; i < count; ++i) {
        if ((fds[1 + i].revents & POLLIN) == 0) continue;
        Allocation& a = *allocations_list_[i];
        while (const auto n = a.relay.recv_from(buf.data(), buf.size(), &from)) {
          if (!paused_.load()) on_peer(a, from.canonical(), buf.data(), *n);
        }
      }
      if (fds[0].revents & POLLIN) {
        while (const auto n = socket_.recv_from(buf.data(), buf.size(), &from)) {
          if (!paused_.load()) on_client(from.canonical(), buf.data(), *n);
        }
      }
    }
  }

  const std::string username_, realm_;
  const tuushin::crypto::Md5Digest key_;
  net::UdpSocket socket_;
  const net::Endpoint endpoint_;
  std::vector<std::unique_ptr<Allocation>> allocations_list_;
  std::atomic<bool> stop_{false}, drop_{false}, refuse_{false}, paused_{false};
  std::atomic<std::uint64_t> nonce_generation_{1};
  std::atomic<std::uint64_t> allocations_{0}, challenges_{0}, stale_nonces_{0}, refreshes_{0},
      permissions_{0}, channel_binds_{0}, sent_indications_{0}, sent_channel_data_{0},
      data_indications_{0}, channel_data_{0}, live_{0};
  std::thread thread_;
};

}  // namespace test
