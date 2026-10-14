#include "tuushin/net/udp.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/uio.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <system_error>
#include <utility>

namespace tuushin::net {

static_assert(Endpoint::kCapacity >= sizeof(sockaddr_storage), "Endpoint storage too small");

namespace {

[[noreturn]] void throw_errno(const char* what) {
  throw std::system_error(errno, std::generic_category(), what);
}

const sockaddr* as_sa(const Endpoint& e) { return static_cast<const sockaddr*>(e.data()); }

}  // namespace

std::optional<Endpoint> Endpoint::resolve(const std::string& host, std::uint16_t port,
                                          bool prefer_ipv6) {
  addrinfo hints{};
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_DGRAM;
  hints.ai_protocol = IPPROTO_UDP;
  if (host.empty()) hints.ai_flags = AI_PASSIVE;
  const std::string service = std::to_string(port);
  addrinfo* res = nullptr;
  if (getaddrinfo(host.empty() ? nullptr : host.c_str(), service.c_str(), &hints, &res) != 0) {
    return std::nullopt;
  }
  const addrinfo* pick = nullptr;
  for (const addrinfo* ai = res; ai != nullptr; ai = ai->ai_next) {
    if (ai->ai_family != AF_INET && ai->ai_family != AF_INET6) continue;
    if (pick == nullptr) pick = ai;
    if ((ai->ai_family == AF_INET6) == prefer_ipv6) {
      pick = ai;
      break;
    }
  }
  // A passive lookup prefers the IPv6 wildcard regardless, because a
  // dual-stack IPv6 socket serves both families.
  if (host.empty()) {
    for (const addrinfo* ai = res; ai != nullptr; ai = ai->ai_next) {
      if (ai->ai_family == AF_INET6) {
        pick = ai;
        break;
      }
    }
  }
  std::optional<Endpoint> out;
  if (pick != nullptr && pick->ai_addrlen <= kCapacity) {
    Endpoint e;
    std::memcpy(e.data(), pick->ai_addr, pick->ai_addrlen);
    e.set_size(static_cast<std::uint32_t>(pick->ai_addrlen));
    out = e;
  }
  freeaddrinfo(res);
  return out;
}

bool Endpoint::is_ipv6() const { return valid() && as_sa(*this)->sa_family == AF_INET6; }

std::uint16_t Endpoint::port() const {
  if (!valid()) return 0;
  if (is_ipv6()) return ntohs(static_cast<const sockaddr_in6*>(data())->sin6_port);
  return ntohs(static_cast<const sockaddr_in*>(data())->sin_port);
}

std::string Endpoint::to_string() const {
  if (!valid()) return "(none)";
  char buf[INET6_ADDRSTRLEN] = {};
  if (is_ipv6()) {
    const auto* a = static_cast<const sockaddr_in6*>(data());
    inet_ntop(AF_INET6, &a->sin6_addr, buf, sizeof buf);
    return "[" + std::string(buf) + "]:" + std::to_string(port());
  }
  const auto* a = static_cast<const sockaddr_in*>(data());
  inet_ntop(AF_INET, &a->sin_addr, buf, sizeof buf);
  return std::string(buf) + ":" + std::to_string(port());
}

bool operator==(const Endpoint& a, const Endpoint& b) {
  if (a.valid() != b.valid()) return false;
  if (!a.valid()) return true;
  if (a.is_ipv6() != b.is_ipv6() || a.port() != b.port()) return false;
  if (a.is_ipv6()) {
    const auto* x = static_cast<const sockaddr_in6*>(a.data());
    const auto* y = static_cast<const sockaddr_in6*>(b.data());
    return std::memcmp(&x->sin6_addr, &y->sin6_addr, sizeof x->sin6_addr) == 0 &&
           x->sin6_scope_id == y->sin6_scope_id;
  }
  const auto* x = static_cast<const sockaddr_in*>(a.data());
  const auto* y = static_cast<const sockaddr_in*>(b.data());
  return x->sin_addr.s_addr == y->sin_addr.s_addr;
}

UdpSocket::UdpSocket(const Endpoint& local) {
  if (!local.valid()) throw std::invalid_argument("UdpSocket: invalid local endpoint");
  ipv6_ = local.is_ipv6();
  fd_ = ::socket(ipv6_ ? AF_INET6 : AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (fd_ < 0) throw_errno("socket");
  if (ipv6_) {
    const int off = 0;
    // Dual stack where the system allows it; failure only loses IPv4.
    ::setsockopt(fd_, IPPROTO_IPV6, IPV6_V6ONLY, &off, sizeof off);
  }
  const int flags = ::fcntl(fd_, F_GETFL, 0);
  if (flags < 0 || ::fcntl(fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
    const int err = errno;
    ::close(fd_);
    errno = err;
    throw_errno("fcntl(O_NONBLOCK)");
  }
#ifdef SO_NOSIGPIPE
  const int one = 1;
  ::setsockopt(fd_, SOL_SOCKET, SO_NOSIGPIPE, &one, sizeof one);
#endif
  if (::bind(fd_, as_sa(local), static_cast<socklen_t>(local.size())) < 0) {
    const int err = errno;
    ::close(fd_);
    errno = err;
    throw_errno("bind");
  }
}

UdpSocket::~UdpSocket() {
  if (fd_ >= 0) ::close(fd_);
}

UdpSocket::UdpSocket(UdpSocket&& o) noexcept : fd_(std::exchange(o.fd_, -1)), ipv6_(o.ipv6_) {}

UdpSocket& UdpSocket::operator=(UdpSocket&& o) noexcept {
  if (this != &o) {
    if (fd_ >= 0) ::close(fd_);
    fd_ = std::exchange(o.fd_, -1);
    ipv6_ = o.ipv6_;
  }
  return *this;
}

Endpoint UdpSocket::local_endpoint() const {
  Endpoint e;
  socklen_t len = Endpoint::kCapacity;
  if (::getsockname(fd_, static_cast<sockaddr*>(e.data()), &len) == 0) {
    e.set_size(static_cast<std::uint32_t>(len));
  }
  return e;
}

bool UdpSocket::set_dscp(int dscp) {
  if (dscp < 0 || dscp > 63) return false;
  const int tos = dscp << 2;  // the ECN bits stay zero
  bool ok = true;
  if (ipv6_) {
    ok = ::setsockopt(fd_, IPPROTO_IPV6, IPV6_TCLASS, &tos, sizeof tos) == 0;
    // A dual-stack socket sends IPv4 with IP_TOS; not every system allows
    // setting it on an IPv6 socket, which only matters for IPv4 peers.
    ::setsockopt(fd_, IPPROTO_IP, IP_TOS, &tos, sizeof tos);
  } else {
    ok = ::setsockopt(fd_, IPPROTO_IP, IP_TOS, &tos, sizeof tos) == 0;
  }
  return ok;
}

bool UdpSocket::send_to(const Endpoint& to, const std::uint8_t* data, std::size_t size) {
  if (!to.valid()) return false;
  int flags = 0;
#ifdef MSG_NOSIGNAL
  flags |= MSG_NOSIGNAL;
#endif
  for (;;) {
    const ssize_t n = ::sendto(fd_, data, size, flags, as_sa(to), static_cast<socklen_t>(to.size()));
    if (n >= 0) return static_cast<std::size_t>(n) == size;
    if (errno == EINTR) continue;
    return false;
  }
}

std::optional<std::size_t> UdpSocket::recv_from(std::uint8_t* buffer, std::size_t capacity,
                                                Endpoint* from) {
  Endpoint src;
  for (;;) {
    iovec iov{buffer, capacity};
    msghdr msg{};
    msg.msg_name = src.data();
    msg.msg_namelen = static_cast<socklen_t>(Endpoint::kCapacity);
    msg.msg_iov = &iov;
    msg.msg_iovlen = 1;
    // recvmsg reports truncation in msg_flags on every POSIX system, whereas
    // MSG_TRUNC as an input flag of recvfrom is Linux-specific.
    const ssize_t n = ::recvmsg(fd_, &msg, 0);
    if (n >= 0) {
      src.set_size(static_cast<std::uint32_t>(msg.msg_namelen));
      if (from != nullptr) *from = src;
      if ((msg.msg_flags & MSG_TRUNC) != 0) return capacity + 1;
      return static_cast<std::size_t>(n);
    }
    if (errno == EINTR) continue;
    // EAGAIN: nothing queued. Anything else (e.g. ECONNREFUSED reported for
    // an earlier datagram) is also "nothing to read now".
    return std::nullopt;
  }
}

bool UdpSocket::wait_readable(Nanos timeout) {
  pollfd p{};
  p.fd = fd_;
  p.events = POLLIN;
  const int ms = timeout <= 0 ? 0 : static_cast<int>((timeout + kNanosPerMilli - 1) / kNanosPerMilli);
  for (;;) {
    const int r = ::poll(&p, 1, ms);
    if (r >= 0) return r > 0 && (p.revents & POLLIN) != 0;
    if (errno != EINTR) return false;
  }
}

}  // namespace tuushin::net
