#include "tuushin/net/udp.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <netdb.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/uio.h>
#include <time.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <system_error>
#include <utility>

namespace tuushin::net {

static_assert(Endpoint::kCapacity >= sizeof(sockaddr_storage), "Endpoint storage too small");

namespace {

[[noreturn]] void throw_errno(const char* what) {
  throw std::system_error(errno, std::generic_category(), what);
}

const sockaddr* as_sa(const Endpoint& e) { return static_cast<const sockaddr*>(e.data()); }

// Kernel receive stamps are on the realtime clock (CLOCK_REALTIME, which
// std::chrono::system_clock reads on POSIX systems).
Nanos realtime_now() {
  using namespace std::chrono;
  return duration_cast<nanoseconds>(system_clock::now().time_since_epoch()).count();
}

}  // namespace

RxStampConverter::Sandwich RxStampConverter::read_clocks() {
  Sandwich best;
  for (int i = 0; i < 3; ++i) {
    Sandwich s;
    s.mono_before = monotonic_now();
    s.realtime = realtime_now();
    s.mono_after = monotonic_now();
    if (i == 0 || s.mono_after - s.mono_before < best.mono_after - best.mono_before) best = s;
    if (best.mono_after - best.mono_before <= kResolution) break;
  }
  return best;
}

void RxStampConverter::observe(const Sandwich& s) {
  const Nanos width = s.mono_after - s.mono_before;
  if (width < 0 || width > kMaxWidth) return;
  const Nanos mid = s.mono_before + width / 2;
  const Nanos offset = s.realtime - mid;
  const Nanos half_width = width / 2 + 1;
  if (have_) {
    const Nanos elapsed = mid - at_;
    const Nanos moved = offset > offset_ ? offset - offset_ : offset_ - offset;
    const Nanos slack = half_width_ + half_width + kResolution +
                        static_cast<Nanos>(static_cast<double>(elapsed > 0 ? elapsed : 0) *
                                           (static_cast<double>(kSlewPerSecond) / kNanosPerSecond));
    if (moved > slack) {
      stepped_ = true;
      stepped_at_ = s.realtime;
    }
  }
  have_ = true;
  offset_ = offset;
  half_width_ = half_width;
  at_ = mid;
}

Nanos RxStampConverter::to_monotonic(Nanos stamp, Nanos read_time) const {
  if (!have_ || (stepped_ && stamp < stepped_at_)) return read_time;
  const Nanos rx = stamp - offset_;
  return rx < read_time ? rx : read_time;
}

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

std::optional<Endpoint> Endpoint::from_address(const std::uint8_t* addr, std::size_t len,
                                               std::uint16_t port) {
  Endpoint e;
  if (len == 4) {
    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_port = htons(port);
    std::memcpy(&a.sin_addr, addr, 4);
    std::memcpy(e.data(), &a, sizeof a);
    e.set_size(sizeof a);
    return e;
  }
  if (len == 16) {
    sockaddr_in6 a{};
    a.sin6_family = AF_INET6;
    a.sin6_port = htons(port);
    std::memcpy(&a.sin6_addr, addr, 16);
    std::memcpy(e.data(), &a, sizeof a);
    e.set_size(sizeof a);
    return e;
  }
  return std::nullopt;
}

namespace {

bool is_v4_mapped(const sockaddr_in6& a) {
  static const std::uint8_t kPrefix[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0xff, 0xff};
  return std::memcmp(&a.sin6_addr, kPrefix, sizeof kPrefix) == 0;
}

}  // namespace

std::vector<std::uint8_t> Endpoint::address() const {
  if (!valid()) return {};
  if (is_ipv6()) {
    const auto* a = static_cast<const sockaddr_in6*>(data());
    const auto* b = reinterpret_cast<const std::uint8_t*>(&a->sin6_addr);
    if (is_v4_mapped(*a)) return std::vector<std::uint8_t>(b + 12, b + 16);
    return std::vector<std::uint8_t>(b, b + 16);
  }
  const auto* a = static_cast<const sockaddr_in*>(data());
  const auto* b = reinterpret_cast<const std::uint8_t*>(&a->sin_addr);
  return std::vector<std::uint8_t>(b, b + 4);
}

Endpoint Endpoint::canonical() const {
  if (!is_ipv6()) return *this;
  const auto a = address();
  return a.size() == 4 ? *from_address(a.data(), 4, port()) : *this;
}

std::optional<Endpoint> Endpoint::for_family(bool ipv6) const {
  if (!valid()) return std::nullopt;
  const auto a = address();
  if (a.size() == 16) return ipv6 ? std::optional<Endpoint>(*this) : std::nullopt;
  if (!ipv6) return from_address(a.data(), 4, port());
  std::uint8_t mapped[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0xff, 0xff};
  std::memcpy(mapped + 12, a.data(), 4);
  return from_address(mapped, 16, port());
}

bool Endpoint::is_local_only() const {
  const auto a = address();
  if (a.size() == 4) {
    const bool zero = a[0] == 0 && a[1] == 0 && a[2] == 0 && a[3] == 0;
    return zero || a[0] == 127 || (a[0] == 169 && a[1] == 254) || a[0] >= 224;
  }
  if (a.size() == 16) {
    bool zero = true;
    for (int i = 0; i < 15; ++i) zero = zero && a[i] == 0;
    const bool unspecified_or_loopback = zero && (a[15] == 0 || a[15] == 1);
    const bool link_local = a[0] == 0xfe && (a[1] & 0xc0) == 0x80;
    return unspecified_or_loopback || link_local || a[0] == 0xff;
  }
  return true;
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

UdpSocket::UdpSocket(UdpSocket&& o) noexcept
    : fd_(std::exchange(o.fd_, -1)),
      ipv6_(o.ipv6_),
      rx_timestamps_(o.rx_timestamps_),
      stamps_(o.stamps_) {}

UdpSocket& UdpSocket::operator=(UdpSocket&& o) noexcept {
  if (this != &o) {
    if (fd_ >= 0) ::close(fd_);
    fd_ = std::exchange(o.fd_, -1);
    ipv6_ = o.ipv6_;
    rx_timestamps_ = o.rx_timestamps_;
    stamps_ = o.stamps_;
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

bool UdpSocket::send_limited(const Endpoint& to, const std::uint8_t* data, std::size_t size,
                             int hops) {
  if (!to.valid() || hops < 1 || hops > 255) return false;
  // The option that limits a datagram to `to`. An IPv6 socket sends to an
  // IPv4-mapped address as IPv4: Linux then takes the TTL from IP_TTL,
  // macOS from IPV6_UNICAST_HOPS (and refuses IP_TTL on an IPv6 socket).
  int level = IPPROTO_IP, name = IP_TTL;
  if (ipv6_) {
    level = IPPROTO_IPV6;
    name = IPV6_UNICAST_HOPS;
#if defined(__linux__)
    if (to.address().size() == 4) {
      level = IPPROTO_IP;
      name = IP_TTL;
    }
#endif
  }
  int saved = 0;
  socklen_t len = sizeof saved;
  if (::getsockopt(fd_, level, name, &saved, &len) != 0 || len != sizeof saved) return false;
  if (::setsockopt(fd_, level, name, &hops, sizeof hops) != 0) return false;
  const bool sent = send_to(to, data, size);
  const bool restored = ::setsockopt(fd_, level, name, &saved, sizeof saved) == 0;
  return sent && restored;
}

bool UdpSocket::enable_rx_timestamps() {
  const int one = 1;
#if defined(SO_TIMESTAMPNS)
  rx_timestamps_ = ::setsockopt(fd_, SOL_SOCKET, SO_TIMESTAMPNS, &one, sizeof one) == 0;
#elif defined(SO_TIMESTAMP)
  rx_timestamps_ = ::setsockopt(fd_, SOL_SOCKET, SO_TIMESTAMP, &one, sizeof one) == 0;
#endif
  if (rx_timestamps_) stamps_.observe(RxStampConverter::read_clocks());
  return rx_timestamps_;
}

std::optional<std::size_t> UdpSocket::recv_from(std::uint8_t* buffer, std::size_t capacity,
                                                Endpoint* from, Nanos* rx_time) {
  Endpoint src;
  union {
    cmsghdr align;
    unsigned char bytes[128];
  } control;
  for (;;) {
    iovec iov{buffer, capacity};
    msghdr msg{};
    msg.msg_name = src.data();
    msg.msg_namelen = static_cast<socklen_t>(Endpoint::kCapacity);
    msg.msg_iov = &iov;
    msg.msg_iovlen = 1;
    if (rx_timestamps_) {
      msg.msg_control = control.bytes;
      msg.msg_controllen = sizeof control.bytes;
    }
    // recvmsg reports truncation in msg_flags on every POSIX system, whereas
    // MSG_TRUNC as an input flag of recvfrom is Linux-specific.
    const ssize_t n = ::recvmsg(fd_, &msg, 0);
    if (n >= 0) {
      if (rx_time != nullptr) {
        Nanos stamp = -1;
        if (rx_timestamps_ && (msg.msg_flags & MSG_CTRUNC) == 0) {
          for (cmsghdr* c = CMSG_FIRSTHDR(&msg); c != nullptr; c = CMSG_NXTHDR(&msg, c)) {
            if (c->cmsg_level != SOL_SOCKET) continue;
#if defined(SCM_TIMESTAMPNS)
            if (c->cmsg_type == SCM_TIMESTAMPNS) {
              timespec ts;
              std::memcpy(&ts, CMSG_DATA(c), sizeof ts);
              stamp = static_cast<Nanos>(ts.tv_sec) * kNanosPerSecond + ts.tv_nsec;
            }
#endif
#if defined(SCM_TIMESTAMP)
            if (c->cmsg_type == SCM_TIMESTAMP) {
              timeval tv;
              std::memcpy(&tv, CMSG_DATA(c), sizeof tv);
              stamp = static_cast<Nanos>(tv.tv_sec) * kNanosPerSecond +
                      static_cast<Nanos>(tv.tv_usec) * 1000;
            }
#endif
          }
        }
        if (stamp >= 0) {
          const auto clocks = RxStampConverter::read_clocks();
          stamps_.observe(clocks);
          *rx_time = stamps_.to_monotonic(stamp, clocks.mono_after);
        } else {
          *rx_time = monotonic_now();
        }
      }
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
  UdpSocket* const self = this;
  return wait_any_readable(&self, 1, timeout);
}

bool wait_any_readable(UdpSocket* const* sockets, std::size_t count, Nanos timeout) {
  if (timeout < 0) timeout = 0;
  // An interrupted wait returns false like a timeout; callers loop on the
  // clock anyway, and retrying with the full timeout would overshoot.
  constexpr std::size_t kMax = 8;
  pollfd fds[kMax];
  nfds_t n = 0;
  int highest = -1;
  for (std::size_t i = 0; i < count && n < kMax; ++i) {
    if (sockets[i] == nullptr) continue;
    fds[n].fd = sockets[i]->native_handle();
    fds[n].events = POLLIN;
    fds[n].revents = 0;
    highest = std::max(highest, fds[n].fd);
    ++n;
  }
#if defined(__linux__)
  (void)highest;
  timespec ts{};
  ts.tv_sec = static_cast<time_t>(timeout / kNanosPerSecond);
  ts.tv_nsec = static_cast<long>(timeout % kNanosPerSecond);
  if (::ppoll(fds, n, &ts, nullptr) <= 0) return false;
#else
  if (highest < FD_SETSIZE) {
    fd_set set;
    FD_ZERO(&set);
    for (nfds_t i = 0; i < n; ++i) FD_SET(fds[i].fd, &set);
    const Nanos us = (timeout + 999) / 1000;
    timeval tv{};
    tv.tv_sec = static_cast<time_t>(us / 1'000'000);
    tv.tv_usec = static_cast<suseconds_t>(us % 1'000'000);
    return ::select(highest + 1, &set, nullptr, nullptr, &tv) > 0;
  }
  const int ms = static_cast<int>((timeout + kNanosPerMilli - 1) / kNanosPerMilli);
  if (::poll(fds, n, ms) <= 0) return false;
#endif
  for (nfds_t i = 0; i < n; ++i) {
    if ((fds[i].revents & POLLIN) != 0) return true;
  }
  return false;
}

std::vector<Endpoint> host_addresses(const UdpSocket& socket, std::size_t limit) {
  std::vector<Endpoint> v4, v6;
  const Endpoint local = socket.local_endpoint();
  const auto bound = local.address();
  bool wildcard = !bound.empty();
  for (const auto b : bound) wildcard = wildcard && b == 0;
  if (!wildcard) {
    const Endpoint c = local.canonical();
    if (c.valid() && !c.is_local_only()) (c.is_ipv6() ? v6 : v4).push_back(c);
  } else {
    ifaddrs* list = nullptr;
    if (::getifaddrs(&list) == 0) {
      for (const ifaddrs* i = list; i != nullptr; i = i->ifa_next) {
        if (i->ifa_addr == nullptr || (i->ifa_flags & IFF_UP) == 0 ||
            (i->ifa_flags & IFF_LOOPBACK) != 0) {
          continue;
        }
        std::optional<Endpoint> e;
        if (i->ifa_addr->sa_family == AF_INET) {
          const auto* a = reinterpret_cast<const sockaddr_in*>(i->ifa_addr);
          e = Endpoint::from_address(reinterpret_cast<const std::uint8_t*>(&a->sin_addr), 4,
                                     local.port());
        } else if (i->ifa_addr->sa_family == AF_INET6 && socket.is_ipv6()) {
          const auto* a = reinterpret_cast<const sockaddr_in6*>(i->ifa_addr);
          e = Endpoint::from_address(reinterpret_cast<const std::uint8_t*>(&a->sin6_addr), 16,
                                     local.port());
        }
        if (!e || e->is_local_only()) continue;
        auto& out = e->is_ipv6() ? v6 : v4;
        if (std::find(out.begin(), out.end(), *e) == out.end()) out.push_back(*e);
      }
      ::freeifaddrs(list);
    }
  }
  v4.insert(v4.end(), v6.begin(), v6.end());
  if (v4.size() > limit) v4.resize(limit);
  return v4;
}

}  // namespace tuushin::net
