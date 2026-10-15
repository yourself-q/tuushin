#pragma once

#include <poll.h>

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <memory>
#include <thread>
#include <utility>
#include <vector>

#include "tuushin/net/rendezvous.hpp"
#include "tuushin/net/udp.hpp"

namespace test {

namespace net = tuushin::net;

// A NAT in user space, for tests on IPv4 loopback, with RFC 4787's mapping
// and filtering behaviours. The sockets behind it (any loopback socket that
// sends to one of its proxies) reach each outside endpoint X through a
// proxy socket on the NAT, proxy(X): what arrives there from an inside
// socket I the NAT sends on to X from an outside socket of I's, so that X
// sees I at the NAT's outside address, and what an outside socket of I's
// receives from X it hands to I from proxy(X). The rendezvous server's
// messages have their candidates and relayed addresses rewritten to
// proxies (an application gateway), since the inside could not reach their
// real addresses through the NAT; the ends' own candidates are empty on
// loopback (host_addresses). Not modelled: hop limits (a primed datagram
// goes the whole way), and the conflicts between one NAT's mappings that
// priming avoids on Linux; tools/nat_lab.sh tests those against real NATs.
class SimNat {
 public:
  enum class Mapping {
    kEndpointIndependent,  // one outside port for every destination
    kAddressPortDependent  // one per destination ("symmetric")
  };
  enum class Filtering {
    kEndpointIndependent,  // anyone may send to a mapping ("full cone")
    kAddressPortDependent  // only the destinations it was used for ("port-restricted")
  };

  SimNat(const net::Endpoint& server, Mapping mapping, Filtering filtering)
      : server_(server.canonical()), mapping_(mapping), filtering_(filtering) {}
  ~SimNat() {
    stop_.store(true);
    if (thread_.joinable()) thread_.join();
  }
  SimNat(const SimNat&) = delete;
  SimNat& operator=(const SimNat&) = delete;

  // The address at which the inside reaches `outside`. Before start() only.
  net::Endpoint proxy(const net::Endpoint& outside) { return proxy_socket(outside).local; }
  void start() {
    thread_ = std::thread([this] { run(); });
  }
  // Forgets every mapping, like a NAT that rebooted or let them time out:
  // the next datagram of an inside socket to any X leaves from a new
  // outside port.
  void rebind() { rebind_.store(true); }

  // Datagrams from outside dropped by the filter.
  std::uint64_t filtered() const { return filtered_.load(); }
  // Datagrams from outside, other than the server's, handed to the inside.
  std::uint64_t delivered() const { return delivered_.load(); }

 private:
  struct Proxy {
    net::Endpoint outside;  // the X it stands for
    net::Endpoint local;
    net::UdpSocket socket;
  };
  struct Outside {
    net::Endpoint inside;       // the socket it maps
    net::Endpoint destination;  // the only one, for address-dependent mapping
    net::UdpSocket socket;
    std::vector<net::Endpoint> sent;  // the destinations used, for filtering
  };

  static net::UdpSocket loopback() { return net::UdpSocket(*net::Endpoint::resolve("127.0.0.1", 0)); }

  Proxy& proxy_socket(const net::Endpoint& outside) {
    const net::Endpoint x = outside.canonical();
    for (auto& p : proxies_) {
      if (p->outside == x) return *p;
    }
    net::UdpSocket s = loopback();
    const net::Endpoint local = s.local_endpoint();
    proxies_.push_back(std::make_unique<Proxy>(Proxy{x, local, std::move(s)}));
    return *proxies_.back();
  }

  Outside& outside_for(const net::Endpoint& inside, const net::Endpoint& x) {
    for (auto& o : outsides_) {
      if (o->inside == inside &&
          (mapping_ == Mapping::kEndpointIndependent || o->destination == x)) {
        return *o;
      }
    }
    outsides_.push_back(std::make_unique<Outside>(Outside{inside, x, loopback(), {}}));
    return *outsides_.back();
  }

  // The server's message with its candidates as the host must address them.
  void rewrite(std::vector<std::uint8_t>& bytes) {
    auto m = net::decode_rendezvous(bytes.data(), bytes.size());
    if (!m) return;
    for (auto& c : m->candidates) c = proxy_socket(c).local;
    for (auto& r : m->relays) r = proxy_socket(r).local;
    bytes = net::encode_rendezvous(*m);
  }

  void run() {
    std::vector<std::uint8_t> buf(4096);
    std::vector<pollfd> fds;
    while (!stop_.load()) {
      if (rebind_.exchange(false)) outsides_.clear();
      fds.clear();
      const std::size_t proxies = proxies_.size(), outsides = outsides_.size();
      for (const auto& p : proxies_) fds.push_back(pollfd{p->socket.native_handle(), POLLIN, 0});
      for (const auto& o : outsides_) fds.push_back(pollfd{o->socket.native_handle(), POLLIN, 0});
      if (::poll(fds.data(), static_cast<nfds_t>(fds.size()), 1) <= 0) continue;
      // Indices stay valid: new proxies and outside sockets are appended,
      // and only a rebind, above, removes any.
      for (std::size_t i = 0; i < proxies; ++i) {
        if ((fds[i].revents & POLLIN) == 0) continue;
        net::Endpoint from;
        while (const auto n = proxies_[i]->socket.recv_from(buf.data(), buf.size(), &from)) {
          const net::Endpoint x = proxies_[i]->outside;
          Outside& o = outside_for(from.canonical(), x);
          if (std::find(o.sent.begin(), o.sent.end(), x) == o.sent.end()) o.sent.push_back(x);
          o.socket.send_to(x, buf.data(), *n);
        }
      }
      for (std::size_t i = 0; i < outsides; ++i) {
        if ((fds[proxies + i].revents & POLLIN) == 0) continue;
        net::Endpoint from;
        while (const auto n = outsides_[i]->socket.recv_from(buf.data(), buf.size(), &from)) {
          const net::Endpoint y = from.canonical();
          const auto& sent = outsides_[i]->sent;
          if (filtering_ == Filtering::kAddressPortDependent &&
              std::find(sent.begin(), sent.end(), y) == sent.end()) {
            filtered_.fetch_add(1);
            continue;
          }
          std::vector<std::uint8_t> bytes(buf.begin(), buf.begin() + static_cast<std::ptrdiff_t>(*n));
          if (y == server_) {
            rewrite(bytes);
          } else {
            delivered_.fetch_add(1);
          }
          proxy_socket(y).socket.send_to(outsides_[i]->inside, bytes.data(), bytes.size());
        }
      }
    }
  }

  const net::Endpoint server_;
  const Mapping mapping_;
  const Filtering filtering_;
  std::vector<std::unique_ptr<Proxy>> proxies_;
  std::vector<std::unique_ptr<Outside>> outsides_;
  std::atomic<bool> stop_{false}, rebind_{false};
  std::atomic<std::uint64_t> filtered_{0}, delivered_{0};
  std::thread thread_;
};

}  // namespace test
