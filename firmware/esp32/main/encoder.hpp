#pragma once

#include <cstdint>

#include "sdkconfig.h"

namespace tuushin::esp32 {

// The magnetic encoder on the motor's shaft, read once a tick by polling:
// an interrupt-driven transfer would cost more in scheduling than the
// transfer itself (some 3 us for 24 bits at 8 MHz).
//
//  MT6701 (SSI): 24-bit frames of 14 bits of angle, 4 of status and a
//    CRC-6 (x^6 + x + 1) over the other 18. The datasheet has the clock
//    idle high, the data driven on its rising edge and taken on its
//    falling one; SPI mode 1 takes it there with the first bit in place
//    (mode 2 takes the first bit before the chip has driven it).
//  AS5047P (SPI mode 1): a read's answer comes in the next frame, so each
//    reading is two frames, the second of which carries the angle at the
//    first; frames have even parity and an error flag, which stays up
//    until the error register is read.
//  AS5600 (I2C): the status and the raw angle in one read of three
//    registers, its slow filter set to the fastest (2x, 0.29 ms to
//    settle) at start: the default (16x) takes 2.2 ms, a lag the servo
//    loop would have to make up for.
//
// A reading that fails its check is retried once within the tick; two in
// a row make the tick's reading fail, which latches the core's encoder
// fault.
class Encoder {
 public:
  bool begin();
  bool read(std::uint32_t* raw);

#if CONFIG_TUUSHIN_ENCODER_AS5600
  static constexpr std::uint32_t kCountsPerRev = 4096;
#else
  static constexpr std::uint32_t kCountsPerRev = 16384;
#endif

  // Readings that failed a check (parity, CRC, error flags, no answer),
  // including those a retry recovered.
  std::uint32_t bad_frames() const { return bad_; }
  // MT6701 and AS5600: readings with the magnet too weak or too strong;
  // they still count, but are noisier.
  std::uint32_t weak_field() const { return weak_; }

 private:
  bool read_once(std::uint32_t* raw);
  std::uint32_t bad_ = 0;
  std::uint32_t weak_ = 0;
};

}  // namespace tuushin::esp32
