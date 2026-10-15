#include "tuushin/net/hops.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <algorithm>

#if defined(__linux__)
#include <linux/errqueue.h>
#include <netinet/ip_icmp.h>
#include <poll.h>
#include <sys/uio.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#endif

namespace tuushin::net {

namespace {

std::string address_string(const Endpoint& e) {
  const auto a = e.address();
  char buf[INET6_ADDRSTRLEN] = {};
  if (a.size() == 4) return inet_ntop(AF_INET, a.data(), buf, sizeof buf) ? buf : "?";
  if (a.size() == 16) return inet_ntop(AF_INET6, a.data(), buf, sizeof buf) ? buf : "?";
  return "?";
}

}  // namespace

int HopTrace::public_hop() const {
  for (std::size_t i = 0; i < routers.size(); ++i) {
    if (routers[i].valid() && !is_private_address(routers[i])) return static_cast<int>(i) + 1;
  }
  return 0;
}

bool HopTrace::settled() const {
  const int p = public_hop();
  if (p == 0) return false;
  for (int i = 0; i + 1 < p; ++i) {
    if (!routers[static_cast<std::size_t>(i)].valid()) return false;
  }
  return true;
}

namespace {

// The last hop that answered from the address of `outside`; 0 if none did.
int outside_hop(const std::vector<Endpoint>& routers, const Endpoint& outside) {
  if (!outside.valid()) return 0;
  const auto a = outside.address();
  int hop = 0;
  for (std::size_t i = 0; i < routers.size(); ++i) {
    if (routers[i].valid() && routers[i].address() == a) hop = static_cast<int>(i) + 1;
  }
  return hop;
}

}  // namespace

int HopTrace::limit(const Endpoint& outside) const {
  const int nat = outside_hop(routers, outside);
  return nat > 0 ? nat + 1 : public_hop();
}

std::string HopTrace::to_string(const Endpoint& outside) const {
  const int nat = outside_hop(routers, outside);
  std::size_t n = std::min(static_cast<std::size_t>(limit(outside)), routers.size());
  if (n == 0) n = routers.size();
  while (n > 0 && !routers[n - 1].valid()) --n;
  if (n == 0) return "no router answered";
  std::string s;
  for (std::size_t i = 0; i < n; ++i) {
    if (i > 0) s += ", ";
    s += routers[i].valid() ? address_string(routers[i]) : "*";
    if (static_cast<int>(i) + 1 == nat) s += " (the NAT's outside)";
  }
  return s;
}

bool is_private_address(const Endpoint& e) {
  if (!e.valid()) return false;
  if (e.is_local_only()) return true;
  const auto a = e.address();
  if (a.size() == 4) {
    return a[0] == 10 ||                             // RFC 1918
           (a[0] == 172 && (a[1] & 0xf0) == 16) ||   // RFC 1918
           (a[0] == 192 && a[1] == 168) ||           // RFC 1918
           (a[0] == 100 && (a[1] & 0xc0) == 64) ||   // RFC 6598, shared by CGNATs
           (a[0] == 192 && a[1] == 0 && a[2] == 0);  // RFC 6890; DS-Lite's AFTR (RFC 6333)
  }
  return a.size() == 16 && (a[0] & 0xfe) == 0xfc;  // RFC 4193
}

#if !defined(__linux__)

std::optional<HopTrace> trace_hops(const Endpoint&, const Endpoint&, int, Nanos) {
  // Without IP_RECVERR a socket hears nothing of the errors for its
  // datagrams; reading them otherwise takes a raw socket, and privileges.
  return std::nullopt;
}

#else

namespace {

// The probes' sockets, closed on every way out.
struct Sockets {
  std::vector<int> fds;
  ~Sockets() {
    for (int fd : fds) {
      if (fd >= 0) ::close(fd);
    }
  }
};

// Reads one error from fd's queue. True if there was one; `router` then
// receives the router that reported the probe's hop limit exceeded, if
// that is what it was.
bool read_error(int fd, Endpoint& router) {
  std::uint8_t data[64];
  iovec iov{data, sizeof data};
  sockaddr_in to{};
  union {
    cmsghdr align;
    unsigned char bytes[256];
  } control;
  msghdr msg{};
  msg.msg_name = &to;
  msg.msg_namelen = sizeof to;
  msg.msg_iov = &iov;
  msg.msg_iovlen = 1;
  msg.msg_control = control.bytes;
  msg.msg_controllen = sizeof control.bytes;
  if (::recvmsg(fd, &msg, MSG_ERRQUEUE | MSG_DONTWAIT) < 0) return false;
  for (cmsghdr* c = CMSG_FIRSTHDR(&msg); c != nullptr; c = CMSG_NXTHDR(&msg, c)) {
    if (c->cmsg_level != IPPROTO_IP || c->cmsg_type != IP_RECVERR) continue;
    sock_extended_err ee;
    std::memcpy(&ee, CMSG_DATA(c), sizeof ee);
    // Time exceeded in transit; code 1 is a fragment reassembly timing out
    // at the target.
    if (ee.ee_origin != SO_EE_ORIGIN_ICMP || ee.ee_type != ICMP_TIME_EXCEEDED || ee.ee_code != 0) {
      continue;
    }
    sockaddr_in offender;
    std::memcpy(&offender, CMSG_DATA(c) + sizeof ee, sizeof offender);
    if (offender.sin_family != AF_INET) continue;
    if (auto e = Endpoint::from_address(reinterpret_cast<const std::uint8_t*>(&offender.sin_addr),
                                        4, 0)) {
      router = *e;
    }
  }
  return true;
}

}  // namespace

std::optional<HopTrace> trace_hops(const Endpoint& target, const Endpoint& local, int max_hops,
                                   Nanos timeout) {
  const auto to = target.for_family(false);
  // A path to a private address stays inside, with nothing past this
  // host's NATs on it to find.
  if (!to || is_private_address(*to) || max_hops < 1 || max_hops > 64) return std::nullopt;
  const auto* dst = static_cast<const sockaddr*>(to->data());

  // The probes leave from the session socket's address, or where it is the
  // wildcard (also the IPv6 one of a dual-stack socket) from wherever the
  // routes send them, as the session's datagrams do.
  sockaddr_in from{};
  from.sin_family = AF_INET;
  const auto a = local.address();
  if (a.size() == 4) std::memcpy(&from.sin_addr, a.data(), 4);

  Sockets s;
  s.fds.assign(static_cast<std::size_t>(max_hops), -1);
  for (int i = 0; i < max_hops; ++i) {
    const int fd = ::socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    if (fd < 0) return std::nullopt;
    s.fds[static_cast<std::size_t>(i)] = fd;
    const int one = 1, ttl = i + 1;
    if (::setsockopt(fd, IPPROTO_IP, IP_RECVERR, &one, sizeof one) != 0 ||
        ::setsockopt(fd, IPPROTO_IP, IP_TTL, &ttl, sizeof ttl) != 0 ||
        ::bind(fd, reinterpret_cast<const sockaddr*>(&from), sizeof from) != 0) {
      return std::nullopt;
    }
  }

  static constexpr char kProbe[] = "tuushin hop probe";
  auto probe = [&](std::size_t i) {
    ::sendto(s.fds[i], kProbe, sizeof kProbe - 1, 0, dst, to->size());
  };
  HopTrace trace;
  trace.routers.resize(s.fds.size());
  for (std::size_t i = 0; i < s.fds.size(); ++i) probe(i);

  // An answer comes back after about the round trip to its router. Once
  // the first public router has answered, after t, the silent hops before
  // it, whose answers were due earlier, get a second probe each and as
  // long again to answer, with some slack for routers slow to make their
  // errors; whatever the hops past it say no longer matters. Until a
  // public router answers, the silent hops get their second probe halfway
  // through the timeout.
  static constexpr Nanos kSlack = 20 * kNanosPerMilli;
  std::vector<Nanos> answered(s.fds.size(), -1);
  std::vector<pollfd> polled(s.fds.size());
  const Nanos start = monotonic_now();
  bool resent = false;
  for (;;) {
    if (trace.settled()) break;
    const Nanos elapsed = monotonic_now() - start;
    const int p = trace.public_hop();
    const Nanos deadline =
        p > 0 ? std::min(timeout, 2 * answered[static_cast<std::size_t>(p) - 1] + kSlack) : timeout;
    if (elapsed >= deadline) break;
    if (!resent && (p > 0 || elapsed >= timeout / 2)) {
      const std::size_t below = p > 0 ? static_cast<std::size_t>(p) - 1 : s.fds.size();
      for (std::size_t i = 0; i < below; ++i) {
        if (!trace.routers[i].valid()) probe(i);
      }
      resent = true;
    }
    const Nanos until = resent ? deadline : timeout / 2;
    const int ms = static_cast<int>((until - elapsed + kNanosPerMilli - 1) / kNanosPerMilli);
    for (std::size_t i = 0; i < s.fds.size(); ++i) polled[i] = pollfd{s.fds[i], 0, 0};
    // The errors show as POLLERR, which poll reports whatever was asked.
    const int n = ::poll(polled.data(), polled.size(), ms);
    if (n < 0 && errno != EINTR) return std::nullopt;
    for (std::size_t i = 0; n > 0 && i < polled.size(); ++i) {
      if ((polled[i].revents & POLLERR) == 0) continue;
      Endpoint router;
      while (read_error(s.fds[i], router)) {
      }
      if (router.valid() && !trace.routers[i].valid()) {
        trace.routers[i] = router;
        answered[i] = monotonic_now() - start;
      }
    }
  }
  return trace;
}

#endif

}  // namespace tuushin::net
