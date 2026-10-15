#pragma once

#include <cstddef>
#include <cstdint>

#include "sdkconfig.h"

namespace tuushin::esp32 {

// The host link: a UART behind the board's USB bridge, or the chip's own
// USB Serial/JTAG port. The servo loop calls it from its tick, so nothing
// here waits.
//
// The UART's driver can queue what is written for its interrupt to feed
// to the FIFO, but a write it cannot queue whole waits for room, and its
// queue's room is hard to know beforehand (each write is an item with a
// header). So the UART here has no driver queue for sending: writes go to
// a ring of our own, and flush() moves into the FIFO what fits, once a
// tick. At 921600 baud the line carries 18 bytes in a 200 us tick and
// the FIFO holds 128, so the line never idles while bytes are waiting.
//
// What the ring cannot take is refused, and the servo loop's core counts
// it as dropped (fw::DeviceCore::dropped_bytes).
class Link {
 public:
  // Installs the driver; false if it could not.
  bool begin();
  std::size_t read(std::uint8_t* buf, std::size_t cap);
  std::size_t write(const std::uint8_t* buf, std::size_t n);
  // Moves queued bytes on towards the host; call once a tick.
  void flush();

 private:
#if CONFIG_TUUSHIN_LINK_UART
  static constexpr std::size_t kRing = 4096;  // a power of two
  std::uint8_t ring_[kRing];
  std::size_t head_ = 0, tail_ = 0;  // free-running; head_ - tail_ bytes queued
#endif
};

}  // namespace tuushin::esp32
