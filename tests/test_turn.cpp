#include <cstring>
#include <functional>
#include <limits>
#include <string>
#include <vector>

#include "sim_turn.hpp"
#include "test.hpp"
#include "tuushin/net/paths.hpp"
#include "tuushin/net/turn.hpp"

using namespace tuushin;
using namespace tuushin::net;
namespace stun = tuushin::net::stun;

namespace {

const std::string kUser = "alice", kPass = "secret", kRealm = "example.org";

Endpoint ep(const char* host, std::uint16_t port) { return *Endpoint::resolve(host, port); }

const Endpoint kServer = ep("192.0.2.1", 3478);
const Endpoint kRelayed = ep("192.0.2.1", 50000);
const Endpoint kMapped = ep("203.0.113.5", 61000);
const Endpoint kPeer = ep("198.51.100.7", 4000);

crypto::Md5Digest key() { return stun::long_term_key(kUser, kRealm, kPass); }

stun::Message parse(const std::vector<std::uint8_t>& b) { return *stun::Message::parse(b.data(), b.size()); }

// The server's side, by hand: responses to `m`.
std::vector<std::uint8_t> respond(const stun::Message& m, stun::Class cls,
                                  const std::function<void(stun::Builder&)>& attrs, bool integrity,
                                  const crypto::Md5Digest& k = key()) {
  stun::Builder b(m.method(), cls, m.transaction());
  if (attrs) attrs(b);
  if (integrity) b.add_integrity(k.data(), k.size());
  b.add_fingerprint();
  return b.take();
}
std::vector<std::uint8_t> challenge(const stun::Message& m, int code, const std::string& nonce,
                                    const std::string& realm = kRealm) {
  return respond(
      m, stun::Class::kError,
      [&](stun::Builder& b) {
        b.add_error(code, "challenge");
        b.add_string(stun::attr::kRealm, realm);
        b.add_string(stun::attr::kNonce, nonce);
      },
      false);
}
std::vector<std::uint8_t> error(const stun::Message& m, int code, bool integrity = true) {
  return respond(
      m, stun::Class::kError, [&](stun::Builder& b) { b.add_error(code, "no"); }, integrity);
}
std::vector<std::uint8_t> success(const stun::Message& m, bool integrity = true,
                                  const Endpoint& relayed = kRelayed) {
  return respond(
      m, stun::Class::kSuccess,
      [&](stun::Builder& b) {
        if (m.method() == stun::method::kAllocate) {
          b.add_address(stun::attr::kXorRelayedAddress, relayed);
          b.add_address(stun::attr::kXorMappedAddress, kMapped);
          b.add_u32(stun::attr::kLifetime, 600);
        }
      },
      integrity);
}

TurnClient::Rx feed(TurnClient& c, const std::vector<std::uint8_t>& b, Nanos now,
                    Endpoint* peer = nullptr, std::size_t* offset = nullptr,
                    std::size_t* length = nullptr, const Endpoint& from = kServer) {
  Endpoint p;
  std::size_t o = 0, l = 0;
  return c.on_datagram(b.data(), b.size(), from, now, peer ? peer : &p, offset ? offset : &o,
                       length ? length : &l);
}

// Takes the client through the challenge to an allocation at `now`.
void allocate(TurnClient& c, Nanos now, const Endpoint& relayed = kRelayed) {
  const auto first = parse(*c.poll(now));
  if (!first.has(stun::attr::kMessageIntegrity)) {
    feed(c, challenge(first, 401, "n1"), now);
    feed(c, success(parse(*c.poll(now)), true, relayed), now);
  } else {
    feed(c, success(first, true, relayed), now);
  }
}

// Requests until nothing is due at `now`, each answered by `answer`.
void answer_all(TurnClient& c, Nanos now,
                const std::function<std::vector<std::uint8_t>(const stun::Message&)>& answer) {
  while (const auto out = c.poll(now)) feed(c, answer(parse(*out)), now);
}

const std::uint8_t* bytes(const char* s) { return reinterpret_cast<const std::uint8_t*>(s); }

}  // namespace

TEST_CASE(turn_allocates_after_the_challenge) {
  TurnClient c(kServer, kUser, kPass);
  CHECK(c.next_due() == 0);
  const auto first = parse(*c.poll(0));
  CHECK(first.method() == stun::method::kAllocate);
  CHECK(first.cls() == stun::Class::kRequest);
  CHECK(!first.has(stun::attr::kMessageIntegrity));
  CHECK(!first.has(stun::attr::kUsername));
  CHECK(first.u32(stun::attr::kRequestedTransport) == 17u << 24);
  CHECK(first.string(stun::attr::kSoftware) == "tuushin");
  CHECK(first.has(stun::attr::kFingerprint) && first.fingerprint_ok());
  CHECK(!c.poll(0));
  CHECK(c.next_due() == TurnClient::kInitialRto);

  CHECK(feed(c, challenge(first, 401, "n1"), 0) == TurnClient::Rx::kControl);
  const auto second = parse(*c.poll(0));
  CHECK(second.transaction() != first.transaction());
  CHECK(second.string(stun::attr::kUsername) == kUser);
  CHECK(second.string(stun::attr::kRealm) == kRealm);
  CHECK(second.string(stun::attr::kNonce) == "n1");
  CHECK(second.check_integrity(key().data(), key().size()));

  // A response without a valid MAC is dropped, and the request repeated.
  feed(c, success(second, false), 0);
  const auto wrong = stun::long_term_key(kUser, kRealm, "guess");
  feed(c, respond(second, stun::Class::kSuccess, nullptr, true, wrong), 0);
  CHECK(!c.allocated());
  CHECK(c.poll(TurnClient::kInitialRto).has_value());

  feed(c, success(second), 0);
  CHECK(c.allocated());
  CHECK(c.relayed() == kRelayed);
  CHECK(c.mapped() == kMapped);
  CHECK(c.allocations() == 1);
  CHECK(!c.poll(0));
  CHECK(c.next_due() == TurnClient::kKeepalive);

  const auto refresh = parse(*c.poll(TurnClient::kKeepalive));
  CHECK(refresh.method() == stun::method::kRefresh);
  CHECK(!refresh.has(stun::attr::kLifetime));
  CHECK(refresh.check_integrity(key().data(), key().size()));
}

TEST_CASE(turn_repeats_requests_on_rfc8489_schedule) {
  TurnClient c(kServer, kUser, kPass);
  const auto first = *c.poll(0);
  // RFC 8489, 6.2.1: at 0, 0.5, 1.5, 3.5, 7.5, 15.5 and 31.5 s; failed at 39.5 s.
  const Nanos at[] = {500, 1500, 3500, 7500, 15500, 31500};
  for (const Nanos ms : at) {
    CHECK(!c.poll(ms * kNanosPerMilli - 1));
    const auto again = c.poll(ms * kNanosPerMilli);
    CHECK(again && *again == first);
  }
  const Nanos given_up = 39500 * kNanosPerMilli;
  CHECK(c.next_due() == given_up);
  CHECK(!c.poll(given_up));
  CHECK(!c.failed());
  CHECK(c.last_error() == "no answer from the server");
  CHECK(c.next_due() == given_up + TurnClient::kRetryDelay);
  const auto retry = c.poll(given_up + TurnClient::kRetryDelay);
  REQUIRE(retry.has_value());
  CHECK(parse(*retry).method() == stun::method::kAllocate);
  CHECK(parse(*retry).transaction() != parse(first).transaction());
}

TEST_CASE(turn_stale_nonce_and_rejected_credentials) {
  TurnClient c(kServer, kUser, kPass);
  allocate(c, 0);
  const Nanos t = TurnClient::kKeepalive;
  const auto refresh = parse(*c.poll(t));
  feed(c, challenge(refresh, 438, "n2"), t);
  const auto again = parse(*c.poll(t));
  CHECK(again.method() == stun::method::kRefresh);
  CHECK(again.transaction() != refresh.transaction());
  CHECK(again.string(stun::attr::kNonce) == "n2");
  CHECK(again.check_integrity(key().data(), key().size()));
  feed(c, success(again), t);
  CHECK(c.allocated());
  CHECK(c.next_due() == 2 * t);

  // 401 to a request that had credentials, for the same realm: refused.
  feed(c, challenge(parse(*c.poll(2 * t)), 401, "n3"), 2 * t);
  CHECK(c.failed());
  CHECK(c.last_error().rfind("401", 0) == 0);
  CHECK(!c.poll(3 * t));
  CHECK(c.next_due() == std::numeric_limits<Nanos>::max());
}

TEST_CASE(turn_data_path_indications_then_channel) {
  TurnClient c(kServer, kUser, kPass);
  std::vector<std::uint8_t> out;
  CHECK(!c.wrap(kPeer, bytes("x"), 1, out));  // no allocation yet
  allocate(c, 0);

  // Before the channel is confirmed: a Send indication.
  CHECK(c.wrap(kPeer, bytes("hello"), 5, out));
  const auto send = parse(out);
  CHECK(send.method() == stun::method::kSend);
  CHECK(send.cls() == stun::Class::kIndication);
  CHECK(send.address(stun::attr::kXorPeerAddress) == kPeer);
  CHECK(send.string(stun::attr::kData) == "hello");

  // wrap() asked for the permission and the channel.
  const auto perm = parse(*c.poll(0));
  CHECK(perm.method() == stun::method::kCreatePermission);
  CHECK(perm.address(stun::attr::kXorPeerAddress) == kPeer);
  CHECK(perm.check_integrity(key().data(), key().size()));
  const auto bind = parse(*c.poll(0));
  CHECK(bind.method() == stun::method::kChannelBind);
  CHECK(bind.u32(stun::attr::kChannelNumber) == 0x4000u << 16);
  CHECK(bind.address(stun::attr::kXorPeerAddress) == kPeer);
  CHECK(!c.poll(0));

  // A Data indication from a permitted address (any port) is data; from
  // another address it is dropped.
  const Endpoint other_port = ep("198.51.100.7", 4001);
  stun::Builder data(stun::method::kData, stun::Class::kIndication, stun::random_transaction_id());
  data.add_address(stun::attr::kXorPeerAddress, other_port);
  data.add(stun::attr::kData, bytes("ping"), 4);
  Endpoint peer;
  std::size_t offset = 0, length = 0;
  CHECK(feed(c, data.bytes(), 0, &peer, &offset, &length) == TurnClient::Rx::kData);
  CHECK(peer == other_port);
  CHECK(length == 4 && std::memcmp(data.bytes().data() + offset, "ping", 4) == 0);
  stun::Builder stranger(stun::method::kData, stun::Class::kIndication, stun::random_transaction_id());
  stranger.add_address(stun::attr::kXorPeerAddress, ep("203.0.113.9", 4000));
  stranger.add(stun::attr::kData, bytes("ping"), 4);
  CHECK(feed(c, stranger.bytes(), 0) == TurnClient::Rx::kControl);
  // From anyone but the server it is not TURN's.
  CHECK(feed(c, data.bytes(), 0, nullptr, nullptr, nullptr, kPeer) == TurnClient::Rx::kNotTurn);

  // ChannelData on the channel asked for is data already; another is not.
  const std::vector<std::uint8_t> cd = {0x40, 0x00, 0x00, 0x03, 'a', 'b', 'c'};
  CHECK(feed(c, cd, 0, &peer, &offset, &length) == TurnClient::Rx::kData);
  CHECK(peer == kPeer && offset == 4 && length == 3);
  const std::vector<std::uint8_t> other = {0x40, 0x01, 0x00, 0x03, 'a', 'b', 'c'};
  CHECK(feed(c, other, 0) == TurnClient::Rx::kControl);
  const std::vector<std::uint8_t> truncated = {0x40, 0x00, 0x00, 0x09, 'a', 'b', 'c'};
  CHECK(feed(c, truncated, 0) == TurnClient::Rx::kControl);

  feed(c, success(perm), 0);
  feed(c, success(bind), 0);
  CHECK(c.permitted(ep("198.51.100.7", 1)));
  CHECK(c.channel_bound(kPeer));
  CHECK(!c.channel_bound(other_port));
  CHECK(c.wrap(kPeer, bytes("hello"), 5, out));
  const std::vector<std::uint8_t> want = {0x40, 0x00, 0x00, 0x05, 'h', 'e', 'l', 'l', 'o'};
  CHECK(out == want);

  // Refreshed after 240 s (permission) and 300 s (channel), besides the
  // Refreshes every 15 s.
  Nanos t = 0;
  const auto refreshes = [&](Nanos until) {
    int perms = 0, binds = 0;
    for (t += TurnClient::kKeepalive; t <= until; t += TurnClient::kKeepalive) {
      answer_all(c, t, [&](const stun::Message& m) {
        perms += m.method() == stun::method::kCreatePermission;
        binds += m.method() == stun::method::kChannelBind;
        return success(m);
      });
    }
    t -= TurnClient::kKeepalive;
    return std::make_pair(perms, binds);
  };
  CHECK(refreshes(225 * kNanosPerSecond) == std::make_pair(0, 0));
  CHECK(refreshes(240 * kNanosPerSecond) == std::make_pair(1, 0));
  CHECK(refreshes(300 * kNanosPerSecond) == std::make_pair(0, 1));
  CHECK(refreshes(480 * kNanosPerSecond) == std::make_pair(1, 0));
  CHECK(c.allocated());
}

TEST_CASE(turn_allocates_again_after_437) {
  TurnClient c(kServer, kUser, kPass);
  allocate(c, 0);
  c.bind_channel(kPeer);
  answer_all(c, 0, [](const stun::Message& m) { return success(m); });
  CHECK(c.channel_bound(kPeer));

  const Nanos t = TurnClient::kKeepalive;
  const auto refresh = parse(*c.poll(t));
  // Without a MAC, a 437 is dropped like a forgery; with one, the
  // allocation is gone.
  feed(c, error(refresh, 437, false), t);
  CHECK(c.allocated());
  feed(c, error(refresh, 437), t);
  CHECK(!c.allocated());
  CHECK(c.last_error().rfind("437", 0) == 0);
  CHECK(!c.channel_bound(kPeer) && !c.permitted(kPeer));

  // The credentials are known: the new Allocate carries them at once.
  const Endpoint moved = ep("192.0.2.1", 50002);
  const auto again = parse(*c.poll(t));
  CHECK(again.method() == stun::method::kAllocate);
  CHECK(again.check_integrity(key().data(), key().size()));
  feed(c, success(again, true, moved), t);
  CHECK(c.allocated() && c.allocations() == 2 && c.relayed() == moved);
  int perms = 0, binds = 0;
  answer_all(c, t, [&](const stun::Message& m) {
    perms += m.method() == stun::method::kCreatePermission;
    binds += m.method() == stun::method::kChannelBind;
    return success(m);
  });
  CHECK(perms == 1 && binds == 1);
  CHECK(c.channel_bound(kPeer));
}

TEST_CASE(turn_refused_peer_and_allocate_errors) {
  {
    TurnClient c(kServer, kUser, kPass);
    allocate(c, 0);
    c.permit(kPeer);
    feed(c, error(parse(*c.poll(0)), 403), 0);
    CHECK(!c.permitted(kPeer));
    std::vector<std::uint8_t> out;
    CHECK(!c.wrap(kPeer, bytes("x"), 1, out) && out.empty());
    CHECK(!c.poll(0));  // no ChannelBind for a refused address
    CHECK(c.allocated());
  }
  {
    TurnClient c(kServer, kUser, kPass);
    const auto first = parse(*c.poll(0));
    feed(c, challenge(first, 401, "n1"), 0);
    feed(c, error(parse(*c.poll(0)), 486), 0);  // quota: later
    CHECK(!c.failed() && !c.allocated());
    CHECK(c.next_due() == TurnClient::kRetryDelay);
    feed(c, error(parse(*c.poll(TurnClient::kRetryDelay)), 403), TurnClient::kRetryDelay);
    CHECK(c.failed());  // forbidden: for good
    CHECK(c.last_error().rfind("403", 0) == 0);
  }
}

TEST_CASE(turn_release_deletes_the_allocation) {
  TurnClient c(kServer, kUser, kPass);
  allocate(c, 0);
  c.release();
  const auto refresh = parse(*c.poll(0));
  CHECK(refresh.method() == stun::method::kRefresh);
  CHECK(refresh.u32(stun::attr::kLifetime) == 0u);
  feed(c, success(refresh), 0);
  CHECK(!c.allocated());
  CHECK(!c.poll(TurnClient::kKeepalive));
}

namespace {

// A client on a loopback socket, against SimTurn, on a clock that tests may
// move ahead.
struct Relayed {
  explicit Relayed(test::SimTurn& server)
      : socket(*Endpoint::resolve("127.0.0.1", 0)), client(server.endpoint(), "alice", "secret") {}

  Nanos now() const { return monotonic_now() + skew; }

  // Exchanges with the server until `done` holds or a second passes; data
  // relayed from peers goes to `received`.
  bool pump(const std::function<bool()>& done) {
    const Nanos deadline = monotonic_now() + kNanosPerSecond;
    std::uint8_t buf[2048];
    while (monotonic_now() < deadline) {
      while (const auto out = client.poll(now())) {
        socket.send_to(client.server(), out->data(), out->size());
      }
      if (done()) return true;
      socket.wait_readable(kNanosPerMilli);
      Endpoint from;
      while (const auto n = socket.recv_from(buf, sizeof buf, &from)) {
        Endpoint peer;
        std::size_t offset = 0, length = 0;
        if (client.on_datagram(buf, *n, from, now(), &peer, &offset, &length) ==
            TurnClient::Rx::kData) {
          received.emplace_back(peer, std::string(reinterpret_cast<char*>(buf) + offset, length));
        }
      }
    }
    return done();
  }

  bool send(const Endpoint& peer, const std::string& s) {
    std::vector<std::uint8_t> out;
    if (!client.wrap(peer, reinterpret_cast<const std::uint8_t*>(s.data()), s.size(), out)) {
      return false;
    }
    return socket.send_to(client.server(), out.data(), out.size());
  }

  UdpSocket socket;
  TurnClient client;
  Nanos skew = 0;
  std::vector<std::pair<Endpoint, std::string>> received;
};

// The next datagram at `socket` within a second, as text.
std::string receive(UdpSocket& socket, Endpoint* from) {
  std::uint8_t buf[2048];
  const Nanos deadline = monotonic_now() + kNanosPerSecond;
  while (monotonic_now() < deadline) {
    socket.wait_readable(kNanosPerMilli);
    if (const auto n = socket.recv_from(buf, sizeof buf, from)) {
      return std::string(reinterpret_cast<char*>(buf), *n);
    }
  }
  return {};
}

}  // namespace

TEST_CASE(turn_relays_through_a_server) {
  test::SimTurn server("alice", "secret");
  server.start();
  Relayed r(server);
  UdpSocket peer(*Endpoint::resolve("127.0.0.1", 0));
  const Endpoint peer_ep = peer.local_endpoint();

  REQUIRE(r.pump([&] { return r.client.allocated(); }));
  CHECK(server.challenges() == 1);
  CHECK(r.client.mapped() == r.socket.local_endpoint());
  const Endpoint relayed = r.client.relayed();

  r.client.permit(peer_ep);
  REQUIRE(r.pump([&] { return r.client.permitted(peer_ep); }));

  // Peer to client: a Data indication, there being no channel yet.
  peer.send_to(relayed, bytes("ping"), 4);
  REQUIRE(r.pump([&] { return !r.received.empty(); }));
  CHECK(r.received.back() == std::make_pair(peer_ep, std::string("ping")));
  CHECK(server.data_indications() == 1);

  // Client to peer: a Send indication, until the channel is bound.
  CHECK(r.send(peer_ep, "pong"));
  Endpoint from;
  CHECK(receive(peer, &from) == "pong");
  CHECK(from == relayed);
  CHECK(server.sent_indications() == 1);
  REQUIRE(r.pump([&] { return r.client.channel_bound(peer_ep); }));
  CHECK(r.send(peer_ep, "pong2"));
  CHECK(receive(peer, &from) == "pong2");
  CHECK(server.sent_channel_data() == 1);
  peer.send_to(relayed, bytes("ping2"), 5);
  REQUIRE(r.pump([&] { return r.received.size() == 2; }));
  CHECK(r.received.back().second == "ping2");
  CHECK(server.channel_data() == 1);

  // A nonce that went stale is replaced on the way.
  server.rotate_nonce();
  r.skew += TurnClient::kKeepalive;
  const auto refreshes = server.refreshes();
  REQUIRE(r.pump([&] { return server.refreshes() > refreshes; }));
  CHECK(server.stale_nonces() == 1);
  CHECK(r.client.allocated());

  // The server restarts: the next Refresh finds no allocation, and the
  // client allocates again and restores its permission and channel.
  server.drop_allocations();
  r.skew += TurnClient::kKeepalive;
  REQUIRE(r.pump([&] { return r.client.allocations() == 2 && r.client.channel_bound(peer_ep); }));
  CHECK(server.allocations() == 2);
  CHECK(r.client.relayed() != relayed);
  peer.send_to(r.client.relayed(), bytes("ping3"), 5);
  REQUIRE(r.pump([&] { return r.received.size() == 3; }));
  CHECK(r.received.back().second == "ping3");

  r.client.release();
  REQUIRE(r.pump([&] { return server.live_allocations() == 0; }));
}

TEST_CASE(paths_carry_datagrams_directly_and_through_the_relay) {
  UdpSocket direct(*Endpoint::resolve("127.0.0.1", 0));
  UdpSocket peer(*Endpoint::resolve("127.0.0.1", 0));
  const Endpoint peer_ep = peer.local_endpoint();
  Paths paths(direct);
  CHECK(paths.turn() == nullptr);
  CHECK(!paths.send(Route{peer_ep, true}, bytes("x"), 1));  // no relay yet
  CHECK((Route{peer_ep, true}).to_string() == peer_ep.to_string() + " through the relay");
  CHECK((Route{peer_ep, false}) != (Route{peer_ep, true}));

  test::SimTurn server("alice", "secret");
  server.start();
  paths.enable_relay(server.endpoint(), "alice", "secret");
  struct Received {
    Route from;
    std::size_t size;
    std::string text;
  };
  std::vector<Received> received;
  auto pump = [&](const std::function<bool()>& done) {
    const Nanos deadline = monotonic_now() + 2 * kNanosPerSecond;
    while (!done() && monotonic_now() < deadline) {
      paths.service(monotonic_now());
      paths.wait_readable(kNanosPerMilli);
      std::uint8_t buf[16];
      Route from;
      while (const auto n = paths.recv(buf, sizeof buf, &from)) {
        received.push_back({from, *n, std::string(reinterpret_cast<char*>(buf),
                                                  std::min(*n, sizeof buf))});
      }
    }
    return done();
  };
  REQUIRE(pump([&] { return paths.turn()->allocated(); }));
  const Endpoint relayed = paths.turn()->relayed();
  // What the TURN client received was its own: nothing was handed out.
  CHECK(received.empty());

  // Directly, both ways.
  Endpoint from;
  CHECK(paths.send(Route{peer_ep, false}, bytes("a"), 1));
  CHECK(receive(peer, &from) == "a");
  CHECK(from == direct.local_endpoint());
  peer.send_to(direct.local_endpoint(), bytes("b"), 1);
  REQUIRE(pump([&] { return received.size() == 1; }));
  CHECK(received[0].from == (Route{peer_ep, false}) && received[0].text == "b");

  // Through the relay: the first datagram asks for the permission and the
  // channel, which go ahead of it, so that the server passes it on.
  CHECK(paths.send(Route{peer_ep, true}, bytes("c"), 1));
  CHECK(receive(peer, &from) == "c");
  CHECK(from == relayed);
  // The server counts what it passed on once it has.
  REQUIRE(pump([&] { return server.sent_indications() == 1; }));
  CHECK(server.permissions() == 1);
  peer.send_to(relayed, bytes("d"), 1);
  REQUIRE(pump([&] { return received.size() == 2; }));
  CHECK(received[1].from == (Route{peer_ep, true}) && received[1].text == "d");
  REQUIRE(pump([&] { return paths.turn()->channel_bound(peer_ep); }));
  CHECK(paths.send(Route{peer_ep, true}, bytes("e"), 1));
  CHECK(receive(peer, &from) == "e");
  CHECK(pump([&] { return server.sent_channel_data() == 1; }));

  // Too long for the buffer, either way: its size is capacity + 1.
  const std::string longer(40, 'z');
  peer.send_to(relayed, bytes(longer.c_str()), longer.size());
  peer.send_to(direct.local_endpoint(), bytes(longer.c_str()), longer.size());
  REQUIRE(pump([&] { return received.size() == 4; }));
  CHECK(received[2].size == 17 && received[3].size == 17);
  CHECK(received[2].text == std::string(16, 'z'));

  // Permissions given ahead of any datagram, for a peer that sends first.
  UdpSocket other(*Endpoint::resolve("127.0.0.1", 0));
  paths.permit({other.local_endpoint()});
  REQUIRE(pump([&] { return paths.turn()->permitted(other.local_endpoint()); }));
  other.send_to(relayed, bytes("f"), 1);
  REQUIRE(pump([&] { return received.size() == 5; }));
  CHECK(received[4].from == (Route{other.local_endpoint(), true}));
}
