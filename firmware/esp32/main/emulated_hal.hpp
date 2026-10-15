#pragma once

#include "calibration_store.hpp"
#include "link.hpp"
#include "tuushin/fw/core.hpp"
#include "tuushin/fw/plant.hpp"

namespace tuushin::esp32 {

// The board in an emulator: the core's Hal over the simulated gimbal motor
// of fw/plant.hpp instead of the encoder, the bridge and the supply, on
// the chip's own clock. The servo loop, its timer and tasks, the host link
// and NVS are the board's.
//
// The motor is integrated up to the clock's present whenever the core
// reads the encoder or changes the bridge, as tuushin-devsim's is. The
// chip has no double-precision FPU, so the integration step is coarse
// (CONFIG_TUUSHIN_EMULATED_STEP_US) and the forces are computed in float
// (fw::PlantParams::float_forces); the integration's cost is what bounds
// the tick an emulator keeps up with.
//
// An integration that falls behind the clock costs more the further behind
// it starts, and one that costs more than the time it covers never catches
// up: each tick's would take longer than the last. So the motor is never
// let fall more than a few ticks behind; past that it stands still while
// the clock runs on, and the time it stood is counted (lost_us()).
class EmulatedHal final : public fw::Hal {
 public:
  EmulatedHal(Link& link, CalibrationStore& store);

  std::uint32_t micros() override;
  bool read_encoder(std::uint32_t* raw) override;
  void set_duties(float a, float b, float c) override;
  void enable_driver(bool on) override;
  float supply_voltage() override { return static_cast<float>(plant_.params().supply); }
  std::size_t read_bytes(std::uint8_t* buf, std::size_t cap) override { return link_.read(buf, cap); }
  std::size_t write_bytes(const std::uint8_t* buf, std::size_t n) override { return link_.write(buf, n); }
  bool load_calibration(fw::Calibration* c) override;
  void store_calibration(const fw::Calibration& c) override { store_.store(c); }

  // After each tick: once the core has calibrated, puts the hand or the
  // object at the knob, as tuushin-devsim does (the calibration's sweeps
  // would run into them). True on the tick it does.
  bool after_tick(const fw::DeviceCore& core);

  const fw::Plant& plant() const { return plant_; }
  // The longest an integration took (us), and resets it.
  std::uint32_t take_worst_integration_us();
  // How much of the time the integration spent (us) against how much
  // motor time it covered (us), since begin; and the motor time skipped.
  std::int64_t busy_us() const { return busy_us_; }
  std::int64_t covered_us() const { return covered_us_; }
  std::int64_t lost_us() const { return lost_us_; }

  static fw::PlantParams plant_params();
  // The device as the simulated motor's datasheet would describe it.
  static fw::DeviceConfig config();

 private:
  void advance();

  Link& link_;
  CalibrationStore& store_;
  fw::Plant plant_;
  std::int64_t t0_us_;
  bool placed_ = false;
  std::uint32_t worst_us_ = 0;
  std::int64_t busy_us_ = 0, covered_us_ = 0, lost_us_ = 0;
};

}  // namespace tuushin::esp32
