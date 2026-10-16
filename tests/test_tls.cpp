#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "paths_harness.hpp"
#include "sim_tls.hpp"
#include "sim_turn.hpp"
#include "test.hpp"
#include "tuushin/net/paths.hpp"
#include "tuushin/net/tls.hpp"

using namespace tuushin;
using namespace tuushin::net;
using test::PathsUnderTest;
using test::receive;

namespace {

const std::uint8_t* bytes(const char* s) { return reinterpret_cast<const std::uint8_t*>(s); }

const std::string kPrefix = "the server's certificate does not check out: ";

// Whether `done` holds within a second.
bool eventually(const std::function<bool()>& done) {
  const Nanos deadline = monotonic_now() + kNanosPerSecond;
  while (!done() && monotonic_now() < deadline) {
    WaitFor none;
    wait_any(&none, 1, kNanosPerMilli);
  }
  return done();
}

// Advances `s` until it is open or closed, or a second has gone.
Stream::State settle(Stream& s) {
  const Nanos deadline = monotonic_now() + kNanosPerSecond;
  while (s.state() == Stream::State::kConnecting && monotonic_now() < deadline) {
    WaitFor w{s.native_handle(), s.wants_write()};
    wait_any(&w, 1, kNanosPerMilli);
  }
  return s.state();
}

// A TCP listener on loopback that accepts only when asked.
struct Listener {
  Listener() {
    fd = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    at = *Endpoint::resolve("127.0.0.1", 0);
    socklen_t len = Endpoint::kCapacity;
    if (fd < 0 ||
        ::bind(fd, static_cast<const sockaddr*>(at.data()), static_cast<socklen_t>(at.size())) != 0 ||
        ::listen(fd, 4) != 0 || ::getsockname(fd, static_cast<sockaddr*>(at.data()), &len) != 0) {
      throw std::runtime_error("Listener: cannot listen");
    }
    at.set_size(static_cast<std::uint32_t>(len));
  }
  ~Listener() { ::close(fd); }
  int fd = -1;
  Endpoint at;
};

}  // namespace

TEST_CASE(tls_is_supported_in_this_build) { CHECK(tls_supported()); }

TEST_CASE(paths_reach_the_relay_over_tls) {
  UdpSocket direct(*Endpoint::resolve("127.0.0.1", 0));
  UdpSocket peer(*Endpoint::resolve("127.0.0.1", 0));
  const Endpoint peer_ep = peer.local_endpoint();
  test::SimTurn server("alice", "secret");
  server.start();
  test::TlsFront front(server.endpoint(), "DNS:turn.test,IP:127.0.0.1");
  PathsUnderTest t(direct);
  t.paths.enable_relay(front.endpoint(), "alice", "secret", RelayTransport::kTls,
                       TlsSettings{"turn.test", front.ca_file()});
  REQUIRE(t.pump([&] { return t.paths.turn()->allocated(); }));
  CHECK(front.handshakes() == 1 && front.failed_handshakes() == 0);
  CHECK(front.offered() == "stun.turn");  // RFC 7443
  CHECK(server.connections() == 1 && server.live_allocations() == 1);
  const Endpoint relayed = t.paths.turn()->relayed();

  // Through the relay, both ways: a Send indication, then ChannelData.
  Endpoint from;
  CHECK(t.paths.send(Route{peer_ep, true}, bytes("c"), 1));
  CHECK(receive(peer, &from) == "c");
  CHECK(from == relayed);
  REQUIRE(t.pump([&] { return t.paths.turn()->channel_bound(peer_ep); }));
  CHECK(t.paths.send(Route{peer_ep, true}, bytes("eee"), 3));
  CHECK(receive(peer, &from) == "eee");

  // A burst, which the front passes on in records of many messages: each
  // arrives whole and in order.
  std::vector<std::string> sent;
  for (std::size_t i = 0; i < 40; ++i) {
    sent.emplace_back(1 + i * 31, static_cast<char>('a' + i % 26));
    peer.send_to(relayed, bytes(sent.back().c_str()), sent.back().size());
  }
  REQUIRE(t.pump([&] { return t.received.size() == sent.size(); }));
  for (std::size_t i = 0; i < sent.size(); ++i) {
    CHECK(t.received[i].first == (Route{peer_ep, true}));
    CHECK(t.received[i].second == sent[i]);
  }

  // The connection closes (with a close_notify): a new one at once, with a
  // new handshake and a new allocation.
  front.drop_connections();
  REQUIRE(t.pump([&] { return t.paths.turn()->allocations() == 2; }));
  CHECK(t.paths.turn()->last_error() == "closed by the other end");
  CHECK(front.handshakes() == 2);
  CHECK(t.paths.send(Route{peer_ep, true}, bytes("g"), 1));
  CHECK(receive(peer, &from) == "g");
  CHECK(from == t.paths.turn()->relayed());

  t.paths.turn()->release();
  REQUIRE(t.pump([&] { return server.live_allocations() == 0; }));
  CHECK(t.pump([&] { return t.paths.stream() == nullptr; }));
}

TEST_CASE(tls_checks_the_servers_certificate) {
  test::SimTurn server("alice", "secret");
  server.start();
  test::TlsFront front(server.endpoint(), "DNS:turn.test");

  struct Case {
    TlsSettings settings;
    std::string error;  // empty: the relay is allocated
  };
  const Case cases[] = {
      {{"turn.test", front.ca_file()}, ""},
      {{"other.test", front.ca_file()}, kPrefix + "hostname mismatch"},
      // By the address connected to, which the certificate does not carry.
      {{"", front.ca_file()}, kPrefix + "IP address mismatch"},
      {{"127.0.0.1", front.ca_file()}, kPrefix + "IP address mismatch"},
      // The system's CAs, which did not sign it.
      {{"turn.test", ""}, kPrefix + "self-signed certificate"},
  };
  for (const auto& c : cases) {
    UdpSocket direct(*Endpoint::resolve("127.0.0.1", 0));
    PathsUnderTest t(direct);
    t.paths.enable_relay(front.endpoint(), "alice", "secret", RelayTransport::kTls, c.settings);
    if (c.error.empty()) {
      CHECK(t.pump([&] { return t.paths.turn()->allocated(); }));
      continue;
    }
    REQUIRE(t.pump([&] { return !t.paths.turn()->last_error().empty(); }));
    CHECK(t.paths.turn()->last_error() == c.error);
    if (t.paths.turn()->last_error() != c.error) {
      std::fprintf(stderr, "  for %s: %s\n", c.settings.host.c_str(),
                   t.paths.turn()->last_error().c_str());
    }
    CHECK(t.paths.stream() == nullptr && !t.paths.turn()->allocated());
    // Tried again after the delay, not at once.
    CHECK(t.paths.next_due() > monotonic_now() + TurnClient::kRetryDelay / 2);
  }
  // The front saw each refusal as a failed handshake.
  CHECK(eventually([&] { return front.failed_handshakes() == 4; }));
  CHECK(front.handshakes() == 1);
}

TEST_CASE(tls_refuses_a_ca_file_it_cannot_read) {
  bool threw = false;
  try {
    TlsConnector connector(TlsSettings{"turn.test", "/nonexistent/ca.pem"});
  } catch (const std::runtime_error& e) {
    threw = std::string(e.what()).find("/nonexistent/ca.pem") != std::string::npos;
  }
  CHECK(threw);
  // And so does enable_relay, before any connection.
  UdpSocket direct(*Endpoint::resolve("127.0.0.1", 0));
  Paths paths(direct);
  threw = false;
  try {
    paths.enable_relay(*Endpoint::resolve("127.0.0.1", 5349), "alice", "secret", RelayTransport::kTls,
                       TlsSettings{"turn.test", "/nonexistent/ca.pem"});
  } catch (const std::runtime_error&) {
    threw = true;
  }
  CHECK(threw);
}

TEST_CASE(tls_holds_back_at_most_the_backlog_and_sends_it_in_order) {
  // Behind the front, a server that does not read for now: the front stops
  // reading the client, and the client's socket fills.
  Listener backend;
  test::TlsFront front(backend.at, "DNS:turn.test");
  const TlsConnector connector(TlsSettings{"turn.test", front.ca_file()});
  auto s = connector.open(front.endpoint(), *Endpoint::resolve("127.0.0.1", 0), -1);
  REQUIRE(settle(*s) == Stream::State::kOpen);
  CHECK(!s->wants_write());

  // Writes of 1000 bytes, each byte numbered, until one is refused: none
  // is taken in part, and the refusal comes after the backlog is full (and
  // the kernel's buffers on the way, which grow to megabytes on loopback).
  std::vector<std::uint8_t> chunk(1000);
  std::size_t taken = 0;
  bool refused = false;
  for (int i = 0; i < 100000 && !refused; ++i) {
    for (std::size_t j = 0; j < chunk.size(); ++j) {
      chunk[j] = static_cast<std::uint8_t>((taken + j) % 251);
    }
    refused = !s->write(chunk.data(), chunk.size());
    if (!refused) taken += chunk.size();
    s->flush();
  }
  CHECK(refused);
  CHECK(taken >= Stream::kMaxBacklog);
  CHECK(s->wants_write());
  CHECK(s->state() == Stream::State::kOpen);

  // The server reads now: every byte arrives, in order, as what waited is
  // sent from where it moved to.
  const int conn = ::accept(backend.fd, nullptr, nullptr);
  REQUIRE(conn >= 0);
  std::size_t got = 0;
  bool in_order = true;
  const Nanos deadline = monotonic_now() + 5 * kNanosPerSecond;
  std::uint8_t buf[65536];
  while (got < taken && monotonic_now() < deadline) {
    s->flush();
    WaitFor w[2] = {{conn, false}, {s->native_handle(), s->wants_write()}};
    wait_any(w, 2, kNanosPerMilli);
    const ssize_t n = ::recv(conn, buf, sizeof buf, MSG_DONTWAIT);
    for (ssize_t k = 0; k < n; ++k) {
      in_order = in_order && buf[k] == (got + static_cast<std::size_t>(k)) % 251;
    }
    if (n > 0) got += static_cast<std::size_t>(n);
  }
  CHECK(got == taken);
  CHECK(in_order);
  CHECK(!s->wants_write());
  ::close(conn);
}
