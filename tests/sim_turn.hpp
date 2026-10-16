#pragma once

#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <deque>
#include <memory>
#include <stdexcept>
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
// and TCP on the same port, long-term credentials with one user, Allocate,
// Refresh, CreatePermission, ChannelBind, Send and Data indications and
// ChannelData. Permissions and channels never expire, and the allocation's
// lifetime is not kept: tests that need expiry use drop_allocations().
// What a client sees of coturn is modelled where tuushin depends on it: 401
// with a realm and a nonce, 438 for a nonce that is no longer current, 437
// for a request other than Allocate from an address without an
// allocation, MESSAGE-INTEGRITY on every response to an authenticated
// request; over TCP, an allocation per connection that goes when the
// connection closes, and ChannelData padded to a multiple of 4 bytes.
class SimTurn {
 public:
  SimTurn(std::string username, std::string password, std::string realm = "tuushin.test")
      : username_(std::move(username)), realm_(std::move(realm)),
        key_(stun::long_term_key(username_, realm_, password)),
        socket_(*net::Endpoint::resolve("127.0.0.1", 0)), endpoint_(socket_.local_endpoint()) {
    listener_ = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listener_ < 0 ||
        ::bind(listener_, static_cast<const sockaddr*>(endpoint_.data()),
               static_cast<socklen_t>(endpoint_.size())) != 0 ||
        ::listen(listener_, 8) != 0) {
      if (listener_ >= 0) ::close(listener_);
      throw std::runtime_error("SimTurn: cannot listen on TCP at the UDP port");
    }
    ::fcntl(listener_, F_SETFL, ::fcntl(listener_, F_GETFL, 0) | O_NONBLOCK);
  }
  ~SimTurn() {
    stop_.store(true);
    if (thread_.joinable()) thread_.join();
    for (const auto& c : conns_) ::close(c->fd);
    ::close(listener_);
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
  // Closes every TCP connection, and their allocations with them, like a
  // server that restarted or a middlebox that reset them; returns once the
  // server's thread has.
  void drop_connections() {
    drop_conns_.store(true);
    while (drop_conns_.load()) std::this_thread::yield();
  }
  // Drops everything that arrives, like a path that went down; over TCP the
  // connections stay up and nothing on them is answered.
  void pause(bool paused) { paused_.store(paused); }
  // Holds every datagram it relays, either way, for `delay` (to the
  // millisecond or so), like a relay off the direct path. Before start()
  // only.
  void set_delay(std::chrono::microseconds delay) { delay_ = delay; }

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
  // TCP connections accepted so far, and open now.
  std::uint64_t connections() const { return connections_.load(); }
  std::uint64_t open_connections() const { return open_connections_.load(); }

 private:
  // A client as the server knows it: its address and, over TCP, its
  // connection.
  struct Client {
    net::Endpoint address;
    int conn = -1;  // the connection's descriptor; -1 for UDP

    friend bool operator==(const Client& a, const Client& b) {
      return a.conn == b.conn && a.address == b.address;
    }
  };
  struct Conn {
    int fd;
    net::Endpoint address;
    std::vector<std::uint8_t> in;
  };
  struct Allocation {
    Client client;
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

  Allocation* find(const Client& client) {
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

  void send_client(const Client& to, const std::uint8_t* data, std::size_t size) {
    if (to.conn < 0) {
      socket_.send_to(to.address, data, size);
      return;
    }
#ifdef MSG_NOSIGNAL
    constexpr int kFlags = MSG_NOSIGNAL;
#else
    constexpr int kFlags = 0;  // SO_NOSIGPIPE is set on the connection
#endif
    // Blocking: a test's client reads all the time. A closed connection
    // shows on the next read.
    while (size > 0) {
      const ssize_t n = ::send(to.conn, data, size, kFlags);
      if (n <= 0) return;
      data += n;
      size -= static_cast<std::size_t>(n);
    }
  }
  void reply(const Client& to, stun::Builder& b, bool integrity) {
    if (integrity) b.add_integrity(key_.data(), key_.size());
    b.add_fingerprint();
    send_client(to, b.bytes().data(), b.bytes().size());
  }
  void error(const Client& to, const stun::Message& m, int code, bool integrity) {
    stun::Builder b(m.method(), stun::Class::kError, m.transaction());
    b.add_error(code, code == 401 ? "Unauthorized" : code == 438 ? "Stale Nonce" : "Error");
    if (code == 401 || code == 438) {
      b.add_string(stun::attr::kRealm, realm_);
      b.add_string(stun::attr::kNonce, nonce());
    }
    reply(to, b, integrity);
  }

  void on_request(const Client& from, const stun::Message& m) {
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
        ok.add_address(stun::attr::kXorMappedAddress, from.address);
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

  void on_client(const Client& from, const std::uint8_t* data, std::size_t size) {
    if (size >= 4 && (data[0] & 0xf0) == 0x40) {
      Allocation* a = find(from);
      const std::uint16_t n = get16(data);
      const std::size_t len = get16(data + 2);
      if (!a || len > size - 4) return;
      for (const auto& c : a->channels) {
        if (c.first == n && permitted(*a, c.second)) {
          relay(a->client, true, c.second, data + 4, len);
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
      relay(a->client, true, *peer, payload, n);
      sent_indications_.fetch_add(1);
    }
  }

  void on_peer(Allocation& a, const net::Endpoint& peer, const std::uint8_t* data, std::size_t size) {
    if (!permitted(a, peer)) return;
    for (const auto& c : a.channels) {
      if (c.second == peer) {
        // Padded over TCP (RFC 8656, 12.5).
        const std::size_t padded = a.client.conn < 0 ? size : (size + 3) / 4 * 4;
        std::vector<std::uint8_t> out(4 + padded, 0);
        out[0] = static_cast<std::uint8_t>(c.first >> 8);
        out[1] = static_cast<std::uint8_t>(c.first);
        out[2] = static_cast<std::uint8_t>(size >> 8);
        out[3] = static_cast<std::uint8_t>(size);
        std::copy(data, data + size, out.begin() + 4);
        relay(a.client, false, {}, out.data(), out.size());
        channel_data_.fetch_add(1);
        return;
      }
    }
    stun::Builder b(stun::method::kData, stun::Class::kIndication, stun::random_transaction_id());
    b.add_address(stun::attr::kXorPeerAddress, peer);
    b.add(stun::attr::kData, data, size);
    relay(a.client, false, {}, b.bytes().data(), b.bytes().size());
    data_indications_.fetch_add(1);
  }

  struct Held {
    std::chrono::steady_clock::time_point release;
    Client client;  // whose allocation
    bool to_peer;   // to `to` from its relayed address, else to the client
    net::Endpoint to;
    std::vector<std::uint8_t> bytes;
  };

  // Sends what the allocation of `client` relays, now or after the delay.
  void relay(const Client& client, bool to_peer, const net::Endpoint& to,
             const std::uint8_t* data, std::size_t size) {
    if (delay_.count() > 0) {
      held_.push_back(Held{std::chrono::steady_clock::now() + delay_, client, to_peer, to,
                           std::vector<std::uint8_t>(data, data + size)});
      return;
    }
    send_relayed(client, to_peer, to, data, size);
  }
  void send_relayed(const Client& client, bool to_peer, const net::Endpoint& to,
                    const std::uint8_t* data, std::size_t size) {
    // The allocation may be gone by now; over TCP its connection with it,
    // and the descriptor may be another connection's.
    Allocation* a = find(client);
    if (to_peer) {
      if (a) a->relay.send_to(to, data, size);
    } else if (a || client.conn < 0) {
      send_client(client, data, size);
    }
  }

  // Closes a connection and what it allocated.
  void close_conn(std::size_t i) {
    const int fd = conns_[i]->fd;
    allocations_list_.erase(std::remove_if(allocations_list_.begin(), allocations_list_.end(),
                                           [&](const auto& a) { return a->client.conn == fd; }),
                            allocations_list_.end());
    live_.store(allocations_list_.size());
    ::close(fd);
    conns_.erase(conns_.begin() + static_cast<std::ptrdiff_t>(i));
    open_connections_.store(conns_.size());
  }

  void accept_conns() {
    for (;;) {
      net::Endpoint from;
      socklen_t len = net::Endpoint::kCapacity;
      const int fd = ::accept(listener_, static_cast<sockaddr*>(from.data()), &len);
      if (fd < 0) return;
      from.set_size(static_cast<std::uint32_t>(len));
      // Blocking for writes (accepted sockets inherit O_NONBLOCK on BSD);
      // reads say MSG_DONTWAIT.
      ::fcntl(fd, F_SETFL, ::fcntl(fd, F_GETFL, 0) & ~O_NONBLOCK);
#ifdef SO_NOSIGPIPE
      const int one = 1;
      ::setsockopt(fd, SOL_SOCKET, SO_NOSIGPIPE, &one, sizeof one);
#endif
      conns_.push_back(std::make_unique<Conn>(Conn{fd, from.canonical(), {}}));
      connections_.fetch_add(1);
      open_connections_.store(conns_.size());
    }
  }

  // Reads what connection `i` has, and handles the messages in it; false
  // once it is closed, by the client or for bytes that are not TURN.
  bool read_conn(std::size_t i, std::vector<std::uint8_t>& buf) {
    Conn& c = *conns_[i];
    for (;;) {
      const ssize_t n = ::recv(c.fd, buf.data(), buf.size(), MSG_DONTWAIT);
      if (n == 0) return false;
      if (n < 0) return errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR;
      c.in.insert(c.in.end(), buf.begin(), buf.begin() + n);
      std::size_t at = 0;
      while (c.in.size() - at >= 4) {
        const std::uint8_t* p = c.in.data() + at;
        const std::size_t len = get16(p + 2);
        std::size_t whole = 0;
        if ((p[0] >> 6) == 0) {
          whole = 20 + len;
        } else if ((p[0] >> 6) == 1) {
          whole = 4 + (len + 3) / 4 * 4;
        } else {
          return false;
        }
        if (c.in.size() - at < whole) break;
        if (!paused_.load()) on_client(Client{c.address, c.fd}, p, whole);
        at += whole;
      }
      c.in.erase(c.in.begin(), c.in.begin() + static_cast<std::ptrdiff_t>(at));
    }
  }
  void release_held() {
    const auto now = std::chrono::steady_clock::now();
    while (!held_.empty() && held_.front().release <= now) {
      const Held& h = held_.front();
      send_relayed(h.client, h.to_peer, h.to, h.bytes.data(), h.bytes.size());
      held_.pop_front();
    }
  }

  void run() {
    std::vector<std::uint8_t> buf(65536);
    std::vector<pollfd> fds;
    while (!stop_.load()) {
      if (drop_.exchange(false)) {
        allocations_list_.clear();
        live_.store(0);
      }
      if (drop_conns_.load()) {
        while (!conns_.empty()) close_conn(conns_.size() - 1);
        drop_conns_.store(false);
      }
      fds.clear();
      fds.push_back(pollfd{socket_.native_handle(), POLLIN, 0});
      fds.push_back(pollfd{listener_, POLLIN, 0});
      for (const auto& a : allocations_list_) {
        fds.push_back(pollfd{a->relay.native_handle(), POLLIN, 0});
      }
      for (const auto& c : conns_) fds.push_back(pollfd{c->fd, POLLIN, 0});
      const int ready = ::poll(fds.data(), static_cast<nfds_t>(fds.size()), 1);
      release_held();
      if (ready <= 0) continue;
      net::Endpoint from;
      // Peers first: a Refresh with lifetime 0 below may remove allocations.
      const std::size_t count = allocations_list_.size();
      for (std::size_t i = 0; i < count; ++i) {
        if ((fds[2 + i].revents & POLLIN) == 0) continue;
        Allocation& a = *allocations_list_[i];
        while (const auto n = a.relay.recv_from(buf.data(), buf.size(), &from)) {
          if (!paused_.load()) on_peer(a, from.canonical(), buf.data(), *n);
        }
      }
      if (fds[0].revents & POLLIN) {
        while (const auto n = socket_.recv_from(buf.data(), buf.size(), &from)) {
          if (!paused_.load()) on_client(Client{from.canonical(), -1}, buf.data(), *n);
        }
      }
      // Connections from the last, so that closing one leaves the indices
      // of the ones before it.
      for (std::size_t i = conns_.size(); i-- > 0;) {
        if ((fds[2 + count + i].revents & (POLLIN | POLLERR | POLLHUP)) == 0) continue;
        if (!read_conn(i, buf)) close_conn(i);
      }
      if (fds[1].revents & POLLIN) accept_conns();
    }
  }

  const std::string username_, realm_;
  const tuushin::crypto::Md5Digest key_;
  net::UdpSocket socket_;
  const net::Endpoint endpoint_;
  int listener_ = -1;
  std::vector<std::unique_ptr<Conn>> conns_;
  std::vector<std::unique_ptr<Allocation>> allocations_list_;
  std::chrono::microseconds delay_{0};
  std::deque<Held> held_;
  std::atomic<bool> stop_{false}, drop_{false}, drop_conns_{false}, refuse_{false}, paused_{false};
  std::atomic<std::uint64_t> nonce_generation_{1};
  std::atomic<std::uint64_t> allocations_{0}, challenges_{0}, stale_nonces_{0}, refreshes_{0},
      permissions_{0}, channel_binds_{0}, sent_indications_{0}, sent_channel_data_{0},
      data_indications_{0}, channel_data_{0}, live_{0}, connections_{0}, open_connections_{0};
  std::thread thread_;
};

}  // namespace test
