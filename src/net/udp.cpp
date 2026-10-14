#include "tuushin/net/udp.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/uio.h>
#include <time.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
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
  if (timeout < 0) timeout = 0;
  // An interrupted wait returns false like a timeout; callers loop on the
  // clock anyway, and retrying with the full timeout would overshoot.
#if defined(__linux__)
  pollfd p{};
  p.fd = fd_;
  p.events = POLLIN;
  timespec ts{};
  ts.tv_sec = static_cast<time_t>(timeout / kNanosPerSecond);
  ts.tv_nsec = static_cast<long>(timeout % kNanosPerSecond);
  const int r = ::ppoll(&p, 1, &ts, nullptr);
  return r > 0 && (p.revents & POLLIN) != 0;
#else
  if (fd_ < FD_SETSIZE) {
    fd_set set;
    FD_ZERO(&set);
    FD_SET(fd_, &set);
    const Nanos us = (timeout + 999) / 1000;
    timeval tv{};
    tv.tv_sec = static_cast<time_t>(us / 1'000'000);
    tv.tv_usec = static_cast<suseconds_t>(us % 1'000'000);
    return ::select(fd_ + 1, &set, nullptr, nullptr, &tv) > 0;
  }
  pollfd p{};
  p.fd = fd_;
  p.events = POLLIN;
  const int ms = static_cast<int>((timeout + kNanosPerMilli - 1) / kNanosPerMilli);
  const int r = ::poll(&p, 1, ms);
  return r > 0 && (p.revents & POLLIN) != 0;
#endif
}

}  // namespace tuushin::net
