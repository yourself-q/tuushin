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

// Converts the kernel's receive stamps, which are on the realtime clock, to
// monotonic_now()'s clock. The two clocks differ by an offset that changes
// only when the realtime clock is set (a step) or, where the monotonic clock
// is not slewed with it, slowly; so the offset is measured at each read and
// subtracted from the stamp.
//
// The offset is read as a sandwich, a realtime reading between two monotonic
// ones, as PTP_SYS_OFFSET_EXTENDED reads a hardware clock against the system
// clock: the offset is then known to within half the sandwich's width. A
// plain pair of readings has no such bound. A thread descheduled between the
// two (a loaded host, a virtual machine's vCPU) reads the realtime clock late
// by the whole pause, and the datagram it converts seems to have arrived that
// much earlier, before it was sent; one such reading, 2.5 ms wide, turned up
// in 5000 on a loaded Linux VM. Sandwiches wider than kMaxWidth are
// discarded and the last narrow one stands.
//
// A step of the realtime clock shows as an offset that moved by more than
// the two sandwiches' uncertainty allows. A datagram stamped before the step
// was noticed may be on either side of it, so it gets the read time instead,
// which is late but never early.
class RxStampConverter {
 public:
  struct Sandwich {
    Nanos mono_before = 0;  // monotonic_now()
    Nanos realtime = 0;     // ns since the Unix epoch
    Nanos mono_after = 0;
  };
  static constexpr Nanos kMaxWidth = 20'000;
  // Slack for a step: the realtime clock's resolution (a microsecond on
  // some systems) and a slew of up to 1000 ppm between readings.
  static constexpr Nanos kResolution = 1'000;
  static constexpr Nanos kSlewPerSecond = 1'000'000;

  // The narrowest of a few sandwiches of this process's clocks.
  static Sandwich read_clocks();
  // Takes a reading into account.
  void observe(const Sandwich& s);
  // The arrival time, on the monotonic clock, of a datagram stamped `stamp`
  // (realtime) and read at `read_time` (monotonic); never after read_time.
  Nanos to_monotonic(Nanos stamp, Nanos read_time) const;

 private:
  bool have_ = false;
  Nanos offset_ = 0;      // realtime - monotonic
  Nanos half_width_ = 0;  // uncertainty of offset_
  Nanos at_ = 0;          // monotonic time of the reading behind offset_
  bool stepped_ = false;
  Nanos stepped_at_ = 0;  // realtime when the last step was noticed
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
  RxStampConverter stamps_;
};

}  // namespace tuushin::net
