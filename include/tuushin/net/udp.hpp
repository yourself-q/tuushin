#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

#include "tuushin/time.hpp"

namespace tuushin::net {

// A socket address (IPv4 or IPv6), stored opaquely so that system headers stay
// out of this header.
class Endpoint {
 public:
  Endpoint() = default;

  // Resolves `host` (name or literal; empty means the wildcard address) and
  // `port`. Returns nullopt if nothing resolves. With `prefer_ipv6` unset the
  // first IPv4 result wins, which is what most home routers handle best.
  static std::optional<Endpoint> resolve(const std::string& host, std::uint16_t port,
                                         bool prefer_ipv6 = false);

  bool valid() const { return len_ != 0; }
  bool is_ipv6() const;
  std::uint16_t port() const;
  std::string to_string() const;  // "1.2.3.4:5" or "[::1]:5"

  friend bool operator==(const Endpoint& a, const Endpoint& b);
  friend bool operator!=(const Endpoint& a, const Endpoint& b) { return !(a == b); }

  // Raw access for the socket implementation.
  const void* data() const { return storage_.data(); }
  void* data() { return storage_.data(); }
  std::uint32_t size() const { return len_; }
  void set_size(std::uint32_t n) { len_ = n; }
  static constexpr std::size_t kCapacity = 128;

 private:
  alignas(8) std::array<std::uint8_t, kCapacity> storage_{};
  std::uint32_t len_ = 0;
};

// Non-blocking UDP socket (POSIX). Setup errors throw std::system_error;
// per-datagram errors are reported through return values because a real-time
// loop must not stop on a transient network error.
class UdpSocket {
 public:
  // Binds to `local` (use Endpoint::resolve("", port) for the wildcard, port
  // 0 for an ephemeral port). An IPv6 wildcard socket also accepts IPv4.
  explicit UdpSocket(const Endpoint& local);
  ~UdpSocket();
  UdpSocket(const UdpSocket&) = delete;
  UdpSocket& operator=(const UdpSocket&) = delete;
  UdpSocket(UdpSocket&& o) noexcept;
  UdpSocket& operator=(UdpSocket&& o) noexcept;

  Endpoint local_endpoint() const;

  // Marks outgoing datagrams with a DSCP code point (RFC 2474), for example
  // 46 (Expedited Forwarding, RFC 3246) for haptic samples. Many networks
  // ignore or rewrite it; it costs nothing where they do. Returns false if
  // the system refused.
  bool set_dscp(int dscp);

  // Sends one datagram. Returns false if it could not be handed to the
  // kernel (buffer full, unreachable); the caller treats it as lost.
  bool send_to(const Endpoint& to, const std::uint8_t* data, std::size_t size);

  // Asks the kernel to stamp each datagram on arrival (SO_TIMESTAMPNS on
  // Linux, SO_TIMESTAMP elsewhere). The stamp then no longer depends on when
  // the application gets round to reading, so a loop may sleep between
  // packets without blurring delay measurements. Returns false if
  // unsupported; recv_from then stamps at read time.
  bool enable_rx_timestamps();

  // Receives one datagram if one is queued. Returns its size (datagrams
  // longer than `capacity` are truncated and reported as `capacity + 1`, so
  // callers can drop them) or nullopt if none is waiting. If `rx_time` is
  // given it receives the arrival time on monotonic_now()'s clock.
  std::optional<std::size_t> recv_from(std::uint8_t* buffer, std::size_t capacity, Endpoint* from,
                                       Nanos* rx_time = nullptr);

  // Waits until a datagram is queued or `timeout` passes; true if readable,
  // false on timeout or when interrupted by a signal.
  // Resolution is about a microsecond (ppoll on Linux, select elsewhere;
  // poll's millisecond timeout is too coarse for a 1 kHz loop).
  bool wait_readable(Nanos timeout);

  int native_handle() const { return fd_; }

 private:
  int fd_ = -1;
  bool ipv6_ = false;
  bool rx_timestamps_ = false;
};

}  // namespace tuushin::net
