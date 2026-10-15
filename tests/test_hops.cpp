#include <string>

#include "test.hpp"
#include "tuushin/net/hops.hpp"

using namespace tuushin;
using namespace tuushin::net;

namespace {

Endpoint at(const char* host) { return *Endpoint::resolve(host, 0); }

}  // namespace

TEST_CASE(hops_private_addresses) {
  for (const char* a : {"10.0.0.1", "172.16.0.1", "172.31.255.254", "192.168.1.1", "100.64.0.1",
                        "100.127.255.254", "192.0.0.1", "169.254.1.1", "127.0.0.1", "fd00::1",
                        "fc00::1", "fe80::1", "::1"}) {
    if (!is_private_address(at(a))) std::fprintf(stderr, "  %s\n", a);
    CHECK(is_private_address(at(a)));
  }
  // The documentation ranges stand for public addresses, in the lab too.
  for (const char* a : {"8.8.8.8", "172.15.255.255", "172.32.0.1", "100.63.255.255", "100.128.0.1",
                        "192.0.1.1", "192.0.2.10", "198.51.100.1", "203.0.113.1", "192.169.0.1",
                        "2001:db8::1", "2400:4050::1"}) {
    if (is_private_address(at(a))) std::fprintf(stderr, "  %s\n", a);
    CHECK(!is_private_address(at(a)));
  }
  CHECK(!is_private_address(Endpoint{}));
  // A dual-stack socket's view of an IPv4 router.
  CHECK(is_private_address(at("::ffff:192.168.1.1")));
  CHECK(!is_private_address(at("::ffff:198.51.100.1")));
}

TEST_CASE(hops_first_public_answer) {
  HopTrace t;
  CHECK(t.public_hop() == 0);
  CHECK(!t.settled());
  CHECK(t.to_string() == "no router answered");

  // A home router, then the ISP.
  t.routers = {at("192.168.1.1"), at("198.51.100.1"), at("198.51.100.9"), Endpoint{}};
  CHECK(t.public_hop() == 2);
  CHECK(t.settled());
  CHECK(t.to_string() == "192.168.1.1, 198.51.100.1");

  // A carrier-grade NAT behind it.
  t.routers = {at("192.168.1.1"), at("100.64.0.1"), at("203.0.113.1")};
  CHECK(t.public_hop() == 3);
  CHECK(t.settled());

  // A silent hop before the public one: counted, but not settled, for the
  // silent one may yet answer.
  t.routers = {at("192.168.1.1"), Endpoint{}, at("203.0.113.1")};
  CHECK(t.public_hop() == 3);
  CHECK(!t.settled());
  CHECK(t.to_string() == "192.168.1.1, *, 203.0.113.1");

  // Only the home router answered: a NAT that does not pass the errors for
  // its mappings back.
  t.routers = {at("10.0.0.1"), Endpoint{}, Endpoint{}};
  CHECK(t.public_hop() == 0);
  CHECK(!t.settled());
  CHECK(t.to_string() == "10.0.0.1");

  // A host with a public address of its own.
  t.routers = {at("198.51.100.1")};
  CHECK(t.public_hop() == 1);
  CHECK(t.settled());
}

TEST_CASE(hops_trace_needs_a_public_ipv4_target) {
  const Endpoint local = at("0.0.0.0");
  const Nanos t0 = monotonic_now();
  // Nothing past this host's NATs lies on a path to a private address.
  CHECK(!trace_hops(at("127.0.0.1"), local));
  CHECK(!trace_hops(at("192.168.1.1"), local));
  CHECK(!trace_hops(at("::1"), local));
  CHECK(!trace_hops(at("2001:db8::1"), local));
  // Without sending anything.
  CHECK(monotonic_now() - t0 < 50 * kNanosPerMilli);
}
