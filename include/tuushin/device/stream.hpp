#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "tuushin/time.hpp"

// The byte link to a device: a serial port (a USB CDC or a USB-UART bridge,
// or the pseudo-terminal of tuushin-devsim) or a TCP connection (an
// emulator's UART). Nothing here knows the protocol (device/proto.hpp).

namespace tuushin::device {

class ByteStream {
 public:
  virtual ~ByteStream() = default;
  // What has arrived, up to `cap`, without waiting: the count, 0 if
  // nothing has, -1 once the stream has closed or failed.
  virtual long read(std::uint8_t* buf, std::size_t cap) = 0;
  // Writes without waiting: how many bytes it took (fewer when the
  // outgoing buffer is full), -1 on failure.
  virtual long write(const std::uint8_t* buf, std::size_t n) = 0;
  // Waits until there is something to read, or the stream has failed, or
  // `timeout` has passed; false on timeout.
  virtual bool wait_readable(Nanos timeout) = 0;
  // Drops what has arrived and not been read: what a device sent before
  // anyone listened.
  virtual void discard_input() {}
  // For messages: the path or the address.
  virtual std::string name() const = 0;
};

// A serial port in raw mode at `baud` (ignored by USB CDC devices, which
// run at the bus's speed). macOS sets any rate (IOSSIOSPEED); elsewhere the
// standard ones. Throws std::runtime_error.
std::unique_ptr<ByteStream> open_serial(const std::string& path, int baud);

// A TCP connection with Nagle off. Throws std::runtime_error.
std::unique_ptr<ByteStream> open_tcp(const std::string& host, std::uint16_t port,
                                     Nanos timeout = 5 * kNanosPerSecond);

}  // namespace tuushin::device
