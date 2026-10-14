#include <cstdint>
#include <cstring>
#include <vector>

#include "test.hpp"
#include "tuushin/net/udp.hpp"
#include "tuushin/wire/packet.hpp"

using namespace tuushin;
using namespace tuushin::net;

namespace {

UdpSocket bind_loopback() { return UdpSocket(*Endpoint::resolve("127.0.0.1", 0)); }

Endpoint loopback_peer(const UdpSocket& s) {
  return *Endpoint::resolve("127.0.0.1", s.local_endpoint().port());
}

}  // namespace

TEST_CASE(udp_resolves_literals) {
  const auto v4 = Endpoint::resolve("127.0.0.1", 47000);
  REQUIRE(v4.has_value());
  CHECK(!v4->is_ipv6());
  CHECK(v4->port() == 47000);
  CHECK(v4->to_string() == "127.0.0.1:47000");

  const auto v6 = Endpoint::resolve("::1", 9);
  REQUIRE(v6.has_value());
  CHECK(v6->is_ipv6());
  CHECK(v6->to_string() == "[::1]:9");

  CHECK(*v4 == *Endpoint::resolve("127.0.0.1", 47000));
  CHECK(*v4 != *Endpoint::resolve("127.0.0.1", 47001));
  CHECK(*v4 != *v6);
  CHECK(!Endpoint().valid());
  CHECK(Endpoint() == Endpoint());
  CHECK(!Endpoint::resolve("no such host.invalid", 1).has_value());
}

TEST_CASE(udp_loopback_send_and_receive) {
  UdpSocket a = bind_loopback();
  UdpSocket b = bind_loopback();
  CHECK(a.local_endpoint().port() != 0);

  std::uint8_t buf[wire::kMaxDatagram];
  Endpoint from;
  CHECK(!b.recv_from(buf, sizeof buf, &from).has_value());  // nothing yet
  CHECK(!b.wait_readable(0));

  std::vector<std::uint8_t> out;
  wire::Header h;
  h.type = wire::PacketType::kProbe;
  h.seq = 42;
  wire::encode_padded(h, 64, out);
  REQUIRE(a.send_to(loopback_peer(b), out.data(), out.size()));
  REQUIRE(b.wait_readable(kNanosPerSecond));
  const auto n = b.recv_from(buf, sizeof buf, &from);
  REQUIRE(n.has_value());
  CHECK(*n == out.size());
  CHECK(std::memcmp(buf, out.data(), out.size()) == 0);
  CHECK(from == loopback_peer(a));
  const auto hd = wire::decode_header(buf, *n);
  REQUIRE(hd.has_value());
  CHECK(hd->seq == 42);
}

TEST_CASE(udp_reports_truncated_datagrams) {
  UdpSocket a = bind_loopback();
  UdpSocket b = bind_loopback();
  std::vector<std::uint8_t> big(100, 0xAB);
  REQUIRE(a.send_to(loopback_peer(b), big.data(), big.size()));
  REQUIRE(b.wait_readable(kNanosPerSecond));
  std::uint8_t small[10];
  const auto n = b.recv_from(small, sizeof small, nullptr);
  REQUIRE(n.has_value());
  CHECK(*n == sizeof small + 1);
  // The rest of the datagram is discarded, not delivered later.
  CHECK(!b.recv_from(small, sizeof small, nullptr).has_value());
}

TEST_CASE(udp_dual_stack_wildcard_accepts_ipv4) {
  // The passive wildcard is IPv6 where available and then serves IPv4 too.
  UdpSocket server(*Endpoint::resolve("", 0));
  UdpSocket client = bind_loopback();
  const std::uint8_t msg[3] = {1, 2, 3};
  REQUIRE(client.send_to(*Endpoint::resolve("127.0.0.1", server.local_endpoint().port()), msg, 3));
  REQUIRE(server.wait_readable(kNanosPerSecond));
  std::uint8_t buf[16];
  Endpoint from;
  const auto n = server.recv_from(buf, sizeof buf, &from);
  REQUIRE(n.has_value());
  CHECK(*n == 3);
  // The reply goes back to whatever address form the server saw (an
  // IPv4-mapped IPv6 address on a dual-stack socket).
  REQUIRE(server.send_to(from, buf, *n));
  REQUIRE(client.wait_readable(kNanosPerSecond));
  CHECK(client.recv_from(buf, sizeof buf, nullptr) == std::optional<std::size_t>(3));
}

TEST_CASE(udp_dscp_range) {
  UdpSocket s = bind_loopback();
  CHECK(s.set_dscp(46));
  CHECK(s.set_dscp(0));
  CHECK(!s.set_dscp(-1));
  CHECK(!s.set_dscp(64));
}

TEST_CASE(udp_socket_moves) {
  UdpSocket a = bind_loopback();
  const int fd = a.native_handle();
  UdpSocket b(std::move(a));
  CHECK(b.native_handle() == fd);
  CHECK(a.native_handle() == -1);  // NOLINT(bugprone-use-after-move)
  UdpSocket c = bind_loopback();
  c = std::move(b);
  CHECK(c.native_handle() == fd);
}
