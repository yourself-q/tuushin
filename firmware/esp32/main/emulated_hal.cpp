#include "emulated_hal.hpp"

#include <cstdio>

#include "esp_timer.h"

namespace tuushin::esp32 {

EmulatedHal::EmulatedHal(Link& link, CalibrationStore& store)
    : link_(link), store_(store), plant_(plant_params()), t0_us_(esp_timer_get_time()) {}

fw::PlantParams EmulatedHal::plant_params() {
  fw::PlantParams p;
  p.pole_pairs = CONFIG_TUUSHIN_POLE_PAIRS;
  p.phase_resistance = CONFIG_TUUSHIN_PHASE_RESISTANCE_MOHM * 1e-3;
  p.kt = CONFIG_TUUSHIN_TORQUE_CONSTANT_UNM_PER_A > 0 ? CONFIG_TUUSHIN_TORQUE_CONSTANT_UNM_PER_A * 1e-6 : 0.08;
  p.inertia = CONFIG_TUUSHIN_INERTIA_GMM2 * 1e-9;
  p.supply = CONFIG_TUUSHIN_SUPPLY_MV * 1e-3;
  p.encoder_noise = CONFIG_TUUSHIN_EMULATED_NOISE_MILLICOUNTS * 1e-3;
  p.phase_inductance = CONFIG_TUUSHIN_PHASE_INDUCTANCE_UH * 1e-6;
  p.pwm_period = 1.0 / 20000.0;  // the board's PWM's default
  p.step = CONFIG_TUUSHIN_EMULATED_STEP_US * 1e-6;
  p.float_forces = true;  // the chip's FPU is single precision
  return p;
}

fw::DeviceConfig EmulatedHal::config() {
  fw::DeviceConfig c = fw::device_config_for(plant_params(), CONFIG_TUUSHIN_NAME);
  c.max_torque = CONFIG_TUUSHIN_MAX_TORQUE_UNM * 1e-6f;
  c.voltage_limit = CONFIG_TUUSHIN_VOLTAGE_LIMIT_MV * 1e-3f;
  c.align_voltage = CONFIG_TUUSHIN_ALIGN_MV * 1e-3f;
  c.tick_us = CONFIG_TUUSHIN_TICK_US;
  return c;
}

void EmulatedHal::advance() {
  // How far behind the motor may fall before it stands still (us).
  constexpr std::int64_t kMaxLagUs = 4 * CONFIG_TUUSHIN_TICK_US;
  const std::int64_t start = esp_timer_get_time();
  std::int64_t target = start - t0_us_;
  const std::int64_t at = static_cast<std::int64_t>(plant_.time() * 1e6);
  if (target - at > kMaxLagUs) {
    const std::int64_t skip = target - at - kMaxLagUs;
    t0_us_ += skip;
    lost_us_ += skip;
    target -= skip;
  }
  if (target <= at) return;
  plant_.advance(static_cast<double>(target) * 1e-6);
  const std::int64_t took = esp_timer_get_time() - start;
  busy_us_ += took;
  covered_us_ += target - at;
  if (took > worst_us_) worst_us_ = static_cast<std::uint32_t>(took);
}

std::uint32_t EmulatedHal::take_worst_integration_us() {
  const std::uint32_t w = worst_us_;
  worst_us_ = 0;
  return w;
}

std::uint32_t EmulatedHal::micros() {
  return static_cast<std::uint32_t>(esp_timer_get_time());
}

bool EmulatedHal::read_encoder(std::uint32_t* raw) {
  advance();
  *raw = plant_.encoder_raw();
  return true;
}

void EmulatedHal::set_duties(float a, float b, float c) {
  advance();
  plant_.set_duties(a, b, c);
}

void EmulatedHal::enable_driver(bool on) {
  advance();
  plant_.enable(on);
}

bool EmulatedHal::load_calibration(fw::Calibration* c) {
#if CONFIG_TUUSHIN_EMULATED_CALIBRATED
  // How the simulated motor really sits under its encoder.
  const fw::PlantParams& p = plant_.params();
  c->valid = true;
  c->direction = static_cast<std::int8_t>(p.direction);
  c->zero_electric = static_cast<float>(p.electrical_offset);
  return true;
#else
  return store_.load(c);
#endif
}

bool EmulatedHal::after_tick(const fw::DeviceCore& core) {
  if (placed_ || !core.calibration().valid || (core.last_state().flags & device::kStateCalibrating)) {
    return false;
  }
  placed_ = true;
  const double now = plant_.time();
#if CONFIG_TUUSHIN_EMULATED_HAND
  fw::HandParams hand;
  hand.present = true;
  hand.ref_offset = plant_.angle();
  plant_.set_hand(hand, now);
#elif CONFIG_TUUSHIN_EMULATED_STOP
  fw::StopParams stop;
  stop.present = true;
  stop.angle = plant_.angle() + CONFIG_TUUSHIN_EMULATED_STOP_MRAD * 1e-3;
  plant_.set_stop(stop);
#endif
  (void)now;
  return true;
}

}  // namespace tuushin::esp32
