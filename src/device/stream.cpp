#include "tuushin/device/stream.hpp"

#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

#if defined(__APPLE__)
#include <IOKit/serial/ioss.h>
#endif

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <string>
#include <system_error>

namespace tuushin::device {

namespace {

[[noreturn]] void throw_errno(const std::string& what) {
  throw std::system_error(errno, std::generic_category(), what);
}

// A non-blocking file descriptor: a tty or a socket.
class FdStream : public ByteStream {
 public:
  FdStream(int fd, std::string name, bool tty) : fd_(fd), name_(std::move(name)), tty_(tty) {}
  ~FdStream() override {
    if (fd_ >= 0) ::close(fd_);
  }
  FdStream(const FdStream&) = delete;
  FdStream& operator=(const FdStream&) = delete;

  long read(std::uint8_t* buf, std::size_t cap) override {
    if (failed_) return -1;
    const ssize_t n = ::read(fd_, buf, cap);
    if (n > 0) return static_cast<long>(n);
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) return 0;
    // 0 is the end of a socket; on a tty it is the other end hanging up
    // (VMIN = 0 reads return -1 with EAGAIN while it is merely quiet).
    failed_ = true;
    return -1;
  }

  long write(const std::uint8_t* buf, std::size_t n) override {
    if (failed_) return -1;
#ifdef MSG_NOSIGNAL
    // A peer that went away must not kill the process with SIGPIPE.
    const ssize_t k = tty_ ? ::write(fd_, buf, n) : ::send(fd_, buf, n, MSG_NOSIGNAL);
#else
    const ssize_t k = ::write(fd_, buf, n);  // SO_NOSIGPIPE is set on sockets
#endif
    if (k >= 0) return static_cast<long>(k);
    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) return 0;
    failed_ = true;
    return -1;
  }

  bool wait_readable(Nanos timeout) override {
    if (failed_) return true;
    pollfd p{fd_, POLLIN, 0};
    const int ms = timeout <= 0 ? 0 : static_cast<int>((timeout + kNanosPerMilli - 1) / kNanosPerMilli);
    const int r = ::poll(&p, 1, ms);
    return r > 0;
  }

  void discard_input() override {
    if (tty_) {
      ::tcflush(fd_, TCIFLUSH);
      return;
    }
    std::uint8_t buf[512];
    while (read(buf, sizeof buf) > 0) {
    }
  }

  std::string name() const override { return name_; }

 private:
  int fd_;
  std::string name_;
  bool tty_;
  bool failed_ = false;
};

#if !defined(__APPLE__)
speed_t standard_speed(int baud) {
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
#ifdef B460800
    case 460800: return B460800;
#endif
#ifdef B921600
    case 921600: return B921600;
#endif
#ifdef B1000000
    case 1000000: return B1000000;
#endif
#ifdef B1500000
    case 1500000: return B1500000;
#endif
#ifdef B2000000
    case 2000000: return B2000000;
#endif
#ifdef B3000000
    case 3000000: return B3000000;
#endif
    default: return 0;
  }
}
#endif

}  // namespace

std::unique_ptr<ByteStream> open_serial(const std::string& path, int baud) {
  const int fd = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd < 0) throw_errno("open " + path);
  auto fail = [&](const std::string& what) {
    const int e = errno;
    ::close(fd);
    errno = e;
    throw_errno(what + " " + path);
  };
  // One controller per device: a second one would interleave its commands.
  if (::ioctl(fd, TIOCEXCL) != 0 && errno != ENOTTY) fail("TIOCEXCL");
  termios t;
  if (::tcgetattr(fd, &t) != 0) fail("tcgetattr");
  ::cfmakeraw(&t);
  t.c_cflag |= CLOCAL | CREAD;
  t.c_cflag &= ~static_cast<tcflag_t>(CRTSCTS);
  t.c_cc[VMIN] = 0;
  t.c_cc[VTIME] = 0;
#if defined(__APPLE__)
  ::cfsetspeed(&t, B115200);
  if (::tcsetattr(fd, TCSANOW, &t) != 0) fail("tcsetattr");
  // Any rate, after tcsetattr, which would reset it.
  speed_t speed = static_cast<speed_t>(baud);
  if (::ioctl(fd, IOSSIOSPEED, &speed) != 0 && errno != ENOTTY) fail("IOSSIOSPEED");
#else
  const speed_t s = standard_speed(baud);
  if (s == 0) {
    ::close(fd);
    throw std::runtime_error("unsupported baud rate " + std::to_string(baud) + " for " + path);
  }
  ::cfsetispeed(&t, s);
  ::cfsetospeed(&t, s);
  if (::tcsetattr(fd, TCSANOW, &t) != 0) fail("tcsetattr");
#endif
  ::tcflush(fd, TCIOFLUSH);
  return std::make_unique<FdStream>(fd, path, true);
}

std::unique_ptr<ByteStream> open_tcp(const std::string& host, std::uint16_t port, Nanos timeout) {
  addrinfo hints{};
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  addrinfo* res = nullptr;
  const std::string service = std::to_string(port);
  if (const int rc = ::getaddrinfo(host.c_str(), service.c_str(), &hints, &res); rc != 0) {
    throw std::runtime_error("cannot resolve " + host + ": " + ::gai_strerror(rc));
  }
  const std::string name = host + ":" + service;
  int err = 0;
  for (addrinfo* ai = res; ai; ai = ai->ai_next) {
    const int fd = ::socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
    if (fd < 0) {
      err = errno;
      continue;
    }
    ::fcntl(fd, F_SETFD, FD_CLOEXEC);
    ::fcntl(fd, F_SETFL, ::fcntl(fd, F_GETFL) | O_NONBLOCK);
    int ok = ::connect(fd, ai->ai_addr, ai->ai_addrlen) == 0 ? 0 : errno;
    if (ok == EINPROGRESS) {
      pollfd p{fd, POLLOUT, 0};
      const int r = ::poll(&p, 1, static_cast<int>(timeout / kNanosPerMilli));
      socklen_t len = sizeof ok;
      if (r <= 0) {
        ok = ETIMEDOUT;
      } else if (::getsockopt(fd, SOL_SOCKET, SO_ERROR, &ok, &len) != 0) {
        ok = errno;
      }
    }
    if (ok != 0) {
      err = ok;
      ::close(fd);
      continue;
    }
    const int one = 1;
    ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
#ifdef SO_NOSIGPIPE
    ::setsockopt(fd, SOL_SOCKET, SO_NOSIGPIPE, &one, sizeof one);
#endif
    ::freeaddrinfo(res);
    return std::make_unique<FdStream>(fd, name, false);
  }
  ::freeaddrinfo(res);
  errno = err;
  throw_errno("connect " + name);
}

}  // namespace tuushin::device
