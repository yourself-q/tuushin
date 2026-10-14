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

namespace {

// A sandwich `width` ns wide starting at monotonic `mono`, with the realtime
// clock `offset` ahead of the monotonic one at the sandwich's middle.
RxStampConverter::Sandwich sandwich(Nanos mono, Nanos offset, Nanos width = 100) {
  return {mono, mono + width / 2 + offset, mono + width};
}

constexpr Nanos kOffset = 1'760'000'000 * kNanosPerSecond;  // realtime - monotonic

}  // namespace

TEST_CASE(rx_stamps_are_converted_with_the_measured_offset) {
  RxStampConverter c;
  CHECK(c.to_monotonic(kOffset + 5'000, 9'000) == 9'000);  // no reading yet: read time
  c.observe(sandwich(10'000, kOffset));
  // Stamped 3 us before the read: arrived 3 us before it on the monotonic clock.
  CHECK(c.to_monotonic(kOffset + 10'050 - 3'000, 10'100) == 10'050 - 3'000);
  // Never after the read time.
  CHECK(c.to_monotonic(kOffset + 20'000, 10'100) == 10'100);
}

TEST_CASE(rx_stamps_ignore_a_reading_descheduled_between_the_clocks) {
  // The failure this guards against: a thread paused between reading the
  // monotonic and the realtime clock sees the realtime clock late by the
  // pause, and a datagram converted with that reading seems to arrive
  // before it was sent. Measured on a loaded Linux VM: a 2.5 ms pause.
  RxStampConverter c;
  c.observe(sandwich(1'000'000, kOffset));
  const Nanos pause = 2'500'000;
  const Nanos t = 2'000'000;
  c.observe({t, t + pause + kOffset, t + pause + 50});
  const Nanos arrival = t - 4'000;
  const Nanos rx = c.to_monotonic(arrival + kOffset, t + pause + 50);
  CHECK(rx == arrival);
  // What a single unchecked pair of readings gives instead.
  const Nanos age = (t + pause + kOffset) - (arrival + kOffset);
  CHECK(t - age < arrival - 2'000'000);
}

TEST_CASE(rx_stamps_from_before_a_step_get_the_read_time) {
  RxStampConverter c;
  c.observe(sandwich(1'000'000, kOffset));
  // The realtime clock is set forward by 50 ms between two reads; a datagram
  // that arrived just before the step carries an old stamp.
  const Nanos step = 50'000'000;
  const Nanos read = 2'000'000;
  const Nanos old_stamp = kOffset + read - 1'000;
  c.observe(sandwich(read, kOffset + step));
  CHECK(c.to_monotonic(old_stamp, read + 100) == read + 100);
  // A datagram stamped after the step was noticed converts normally.
  const Nanos later = 3'000'000;
  c.observe(sandwich(later, kOffset + step));
  CHECK(c.to_monotonic(kOffset + step + later - 2'000, later + 100) == later - 2'000);

  // Set back by 50 ms: old stamps would convert late, and are held to the
  // read time; new ones convert normally.
  RxStampConverter d;
  d.observe(sandwich(1'000'000, kOffset));
  d.observe(sandwich(read, kOffset - step));
  CHECK(d.to_monotonic(kOffset + read - 1'000, read + 100) == read + 100);
  d.observe(sandwich(later, kOffset - step));
  CHECK(d.to_monotonic(kOffset - step + later - 2'000, later + 100) == later - 2'000);
}

TEST_CASE(rx_stamps_tolerate_slew_without_seeing_a_step) {
  // Where the monotonic clock is not slewed with the realtime one (macOS),
  // the offset drifts: 500 ppm over 100 ms is 50 us, not a step.
  RxStampConverter c;
  c.observe(sandwich(0, kOffset));
  c.observe(sandwich(100'000'000, kOffset + 50'000));
  const Nanos read = 100'000'100;
  CHECK(c.to_monotonic(kOffset + 50'000 + read - 7'000, read) == read - 7'000);
  // The same change within 1 ms is a step.
  RxStampConverter d;
  d.observe(sandwich(0, kOffset));
  d.observe(sandwich(1'000'000, kOffset + 50'000));
  CHECK(d.to_monotonic(kOffset + 50'000 + 1'000'100 - 7'000, 1'000'100) == 1'000'100);
}

TEST_CASE(rx_stamps_of_loopback_datagrams_are_causal) {
  // On the real clocks: every datagram's receive time lies between the
  // sender's clock just before sending and the read.
  UdpSocket a = bind_loopback();
  UdpSocket b = bind_loopback();
  if (!b.enable_rx_timestamps()) return;
  std::uint8_t buf[64] = {};
  Endpoint from;
  int checked = 0;
  for (int i = 0; i < 200; ++i) {
    const Nanos before = monotonic_now();
    REQUIRE(a.send_to(loopback_peer(b), buf, 8));
    REQUIRE(b.wait_readable(kNanosPerSecond));
    Nanos rx = 0;
    REQUIRE(b.recv_from(buf, sizeof buf, &from, &rx).has_value());
    const Nanos after = monotonic_now();
    // Half a sandwich and the realtime clock's resolution are the bound.
    CHECK(rx >= before - RxStampConverter::kMaxWidth / 2 - RxStampConverter::kResolution);
    CHECK(rx <= after);
    ++checked;
  }
  CHECK(checked == 200);
  const auto s = RxStampConverter::read_clocks();
  CHECK(s.mono_after >= s.mono_before);
}
