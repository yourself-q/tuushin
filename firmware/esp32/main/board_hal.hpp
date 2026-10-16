#pragma once

#include "bridge.hpp"
#include "calibration_store.hpp"
#include "encoder.hpp"
#include "link.hpp"
#include "supply.hpp"
#include "tuushin/fw/core.hpp"

namespace tuushin::esp32 {

// The board: the core's Hal over the encoder, the bridge, the supply's
// divider, the host link and NVS.
class BoardHal final : public fw::Hal {
 public:
  BoardHal(Link& link, CalibrationStore& store) : link_(link), store_(store) {}
  // Brings up the hardware; false (with the failing part named) if some
  // part did not come up. The core still runs then, and reports the
  // encoder's fault to the host, so that a host sees why.
  bool begin(const char** failed);

  std::uint32_t micros() override;
  bool read_encoder(std::uint32_t* raw) override { return encoder_.read(raw); }
  // Another transfer, if the menuconfig takes one (TUUSHIN_ENC_REREAD).
  // The new duties are out but take effect with the PWM's next period, up
  // to 50 us on at 20 kHz, and the current follows them over the windings'
  // L / R: no reading is at the change, and the core books the travel
  // between the two readings by the current's interval over it (see
  // "Reading again" in firmware/core/src/core.cpp).
  // A transfer that fails twice leaves that change booked by the tick's
  // reading, and counts in bad_frames().
  bool reread_encoder(std::uint32_t* raw) override {
#if CONFIG_TUUSHIN_ENC_REREAD
    return encoder_.read(raw);
#else
    (void)raw;
    return false;
#endif
  }
  void set_duties(float a, float b, float c) override { bridge_.set(a, b, c); }
  void enable_driver(bool on) override { bridge_.enable(on); }
  float supply_voltage() override { return supply_.voltage(); }
  std::size_t read_bytes(std::uint8_t* buf, std::size_t cap) override { return link_.read(buf, cap); }
  std::size_t write_bytes(const std::uint8_t* buf, std::size_t n) override { return link_.write(buf, n); }
  bool load_calibration(fw::Calibration* c) override { return store_.load(c); }
  void store_calibration(const fw::Calibration& c) override { store_.store(c); }

  const Encoder& encoder() const { return encoder_; }

 private:
  Link& link_;
  CalibrationStore& store_;
  Encoder encoder_;
  Bridge bridge_;
  Supply supply_;
};

// The device as the menuconfig describes it.
fw::DeviceConfig board_config();

}  // namespace tuushin::esp32
