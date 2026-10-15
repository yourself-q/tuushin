#pragma once

#include <optional>
#include <string>
#include <vector>

#include "tuushin/net/udp.hpp"
#include "tuushin/time.hpp"

namespace tuushin::net {

// How far this host's own NATs reach, for the hop limit of the datagrams
// that open them without reaching the other end's (RendezvousConfig::
// prime_hops, rendezvous.hpp). The limit must be large enough to pass every
// NAT in front of this host (a home router, and often a carrier-grade NAT
// behind it) and small enough to expire before the other end's NAT. Guha
// and Francis (IMC 2005), on the low-TTL methods for TCP, name choosing it
// as a problem and leave it open.
//
// It is found here as traceroute finds a path: datagrams towards a host on
// the public internet (the rendezvous server) with hop limits 1, 2, ...,
// each from a socket of its own, so that an error that quotes only the
// UDP header still says which limit it answers, and the routers' ICMP
// time-exceeded errors read from the sockets' error queues (IP_RECVERR,
// Linux only). A NAT must send such errors itself and pass back those for
// its mappings (RFC 5508, section 7.2 and REQ-4). It answers from its
// inside address, as Linux does, which is private (RFC 1918) or, at a
// carrier-grade NAT, shared (RFC 6598); the first router that answers from
// a public address is then past this host's NATs, and a datagram with that
// hop count as its limit passes them all and expires there. A NAT that
// answers from its outside address instead gives itself away by answering
// from the address the rendezvous server sees this host at; the limit is
// then one more. Routers inside an ISP can have private addresses too,
// after its NAT or without one; the limit then passes them as well, which
// costs nothing unless the other end's NAT is among them.
//
// Silent hops (routers that send no errors, or rate-limit them) are
// skipped: the first public answer counts, whatever came before it.
struct HopTrace {
  // routers[i] answered for the hop limit i + 1; invalid where none did.
  std::vector<Endpoint> routers;

  // The first hop whose router answered from a public address; 0 if none
  // did.
  int public_hop() const;
  // Whether public_hop() can no longer change: every hop before it
  // answered, privately.
  bool settled() const;
  // The hop limit that passes this host's NATs and expires at the first
  // router past them: one past the last hop that answered from the address
  // of `outside` (where the rendezvous server sees this host: the
  // outermost NAT's outside address, any port), or else public_hop().
  int limit(const Endpoint& outside = {}) const;
  // "10.0.0.1, *, 198.51.100.1": the routers by hop, * for a silent one,
  // up to the one limit(outside) expires at, as far as they answered.
  std::string to_string(const Endpoint& outside = {}) const;
};

// True for an address that does not route on the public internet but may
// answer on the way to it: RFC 1918, RFC 6598 (100.64.0.0/10, shared by
// carrier-grade NATs), 192.0.0.0/24 (where DS-Lite's NAT, the AFTR, has its
// tunnel's end, RFC 6333), IPv6 unique local (fc00::/7), and the addresses
// Endpoint::is_local_only takes (loopback, link-local, ...).
bool is_private_address(const Endpoint& e);

// Probes the path to `target` (IPv4) from the address of `local` (the
// session's socket; any port) with hop limits 1 to max_hops, twice for the
// hops that stay silent, and waits for the routers' answers until the
// trace is settled, or the silent hops before the first public router have
// had twice its round trip to answer, or `timeout` passes. The probes that
// reach the target are small datagrams it ignores. nullopt where the system
// cannot report the errors (not Linux), for an IPv6 target or one at a
// private address (a path that stays inside has nothing past this host's
// NATs on it), or if the sockets cannot be made; the caller then keeps its
// configured limit.
std::optional<HopTrace> trace_hops(const Endpoint& target, const Endpoint& local,
                                   int max_hops = 10, Nanos timeout = 500 * kNanosPerMilli);

}  // namespace tuushin::net
