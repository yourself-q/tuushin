#include "tuushin/net/tcp.hpp"

#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <system_error>

namespace tuushin::net {

namespace {

[[noreturn]] void throw_errno(const char* what) {
  throw std::system_error(errno, std::generic_category(), what);
}

#ifdef MSG_NOSIGNAL
constexpr int kSendFlags = MSG_NOSIGNAL;
#else
constexpr int kSendFlags = 0;  // SO_NOSIGPIPE is set instead
#endif

}  // namespace

TcpStream::TcpStream(const Endpoint& to, const Endpoint& local, int dscp) {
  const bool ipv6 = to.is_ipv6();
  fd_ = ::socket(ipv6 ? AF_INET6 : AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (fd_ < 0) throw_errno("socket");
  auto fail = [&](const char* what) {
    const int err = errno;
    ::close(fd_);
    fd_ = -1;
    errno = err;
    throw_errno(what);
  };
  if (ipv6) {
    const int off = 0;
    // An IPv4-mapped server through a dual-stack socket's address.
    ::setsockopt(fd_, IPPROTO_IPV6, IPV6_V6ONLY, &off, sizeof off);
  }
  const int flags = ::fcntl(fd_, F_GETFL, 0);
  if (flags < 0 || ::fcntl(fd_, F_SETFL, flags | O_NONBLOCK) < 0) fail("fcntl(O_NONBLOCK)");
  ::fcntl(fd_, F_SETFD, FD_CLOEXEC);
  const int one = 1;
#ifdef SO_NOSIGPIPE
  ::setsockopt(fd_, SOL_SOCKET, SO_NOSIGPIPE, &one, sizeof one);
#endif
  ::setsockopt(fd_, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
  if (dscp >= 0 && dscp <= 63) {
    const int tos = dscp << 2;
    if (ipv6) ::setsockopt(fd_, IPPROTO_IPV6, IPV6_TCLASS, &tos, sizeof tos);
    ::setsockopt(fd_, IPPROTO_IP, IP_TOS, &tos, sizeof tos);
  }
  if (local.valid() &&
      ::bind(fd_, static_cast<const sockaddr*>(local.data()), static_cast<socklen_t>(local.size())) < 0) {
    fail("bind");
  }
  if (::connect(fd_, static_cast<const sockaddr*>(to.data()), static_cast<socklen_t>(to.size())) == 0) {
    state_ = State::kOpen;
  } else if (errno != EINPROGRESS && errno != EINTR) {
    close_with(std::string("connect: ") + std::strerror(errno));
  }
}

TcpStream::~TcpStream() {
  if (fd_ >= 0) ::close(fd_);
}

void TcpStream::close_with(const std::string& why) {
  if (state_ == State::kClosed) return;
  state_ = State::kClosed;
  error_ = why;
  out_.clear();
  head_ = 0;
}

Stream::State TcpStream::state() {
  if (state_ != State::kConnecting) return state_;
  pollfd p{fd_, POLLOUT, 0};
  if (::poll(&p, 1, 0) <= 0) return state_;
  int err = 0;
  socklen_t len = sizeof err;
  if (::getsockopt(fd_, SOL_SOCKET, SO_ERROR, &err, &len) != 0) err = errno;
  if (err != 0) {
    close_with(std::string("connect: ") + std::strerror(err));
    return state_;
  }
  state_ = State::kOpen;
  flush();
  return state_;
}

bool TcpStream::write(const std::uint8_t* data, std::size_t size) {
  if (state_ == State::kClosed) return false;
  if (out_.size() - head_ + size > kMaxBacklog) return false;
  std::size_t sent = 0;
  if (state_ == State::kOpen && head_ == out_.size()) {
    const ssize_t n = ::send(fd_, data, size, kSendFlags);
    if (n >= 0) {
      sent = static_cast<std::size_t>(n);
    } else if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
      close_with(std::string("send: ") + std::strerror(errno));
      return false;
    }
  }
  if (sent < size) {
    if (head_ == out_.size()) {
      out_.clear();
      head_ = 0;
    }
    out_.insert(out_.end(), data + sent, data + size);
  }
  return true;
}

void TcpStream::flush() {
  if (state_ != State::kOpen) return;
  while (head_ < out_.size()) {
    const ssize_t n = ::send(fd_, out_.data() + head_, out_.size() - head_, kSendFlags);
    if (n < 0) {
      if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
        close_with(std::string("send: ") + std::strerror(errno));
      }
      return;
    }
    head_ += static_cast<std::size_t>(n);
  }
  out_.clear();
  head_ = 0;
}

long TcpStream::read(std::uint8_t* buffer, std::size_t capacity) {
  if (state_ == State::kConnecting && state() != State::kOpen) return state_ == State::kClosed ? -1 : 0;
  if (state_ == State::kClosed) return -1;
  const ssize_t n = ::recv(fd_, buffer, capacity, 0);
  if (n > 0) return static_cast<long>(n);
  if (n == 0) {
    close_with("closed by the other end");
    return -1;
  }
  if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) return 0;
  close_with(std::string("recv: ") + std::strerror(errno));
  return -1;
}

Endpoint TcpStream::local_endpoint() const {
  Endpoint e;
  socklen_t len = Endpoint::kCapacity;
  if (::getsockname(fd_, static_cast<sockaddr*>(e.data()), &len) == 0) {
    e.set_size(static_cast<std::uint32_t>(len));
  }
  return e;
}

}  // namespace tuushin::net
