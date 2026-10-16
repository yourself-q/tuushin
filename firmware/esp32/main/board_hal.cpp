#include "board_hal.hpp"

#include <cstdio>

#include "esp_timer.h"

namespace tuushin::esp32 {

bool BoardHal::begin(const char** failed) {
  // The bridge first, so that the windings are open (or shorted, without
  // an enable pin) while the rest comes up.
  if (!bridge_.begin()) {
    *failed = "bridge";
    return false;
  }
  if (!supply_.begin()) {
    *failed = "supply ADC";
    return false;
  }
  if (!encoder_.begin()) {
    *failed = "encoder";
    return false;
  }
  return true;
}

std::uint32_t BoardHal::micros() {
  return static_cast<std::uint32_t>(esp_timer_get_time());
}

fw::DeviceConfig board_config() {
  fw::DeviceConfig c;
  c.motor.pole_pairs = CONFIG_TUUSHIN_POLE_PAIRS;
  c.motor.phase_resistance = CONFIG_TUUSHIN_PHASE_RESISTANCE_MOHM * 1e-3f;
  c.motor.torque_constant = CONFIG_TUUSHIN_TORQUE_CONSTANT_UNM_PER_A * 1e-6f;
  c.motor.kv = static_cast<float>(CONFIG_TUUSHIN_KV);
  c.counts_per_rev = Encoder::kCountsPerRev;
  c.max_torque = CONFIG_TUUSHIN_MAX_TORQUE_UNM * 1e-6f;
  c.inertia = CONFIG_TUUSHIN_INERTIA_GMM2 * 1e-9f;
  c.voltage_limit = CONFIG_TUUSHIN_VOLTAGE_LIMIT_MV * 1e-3f;
  c.supply_min = CONFIG_TUUSHIN_SUPPLY_MIN_MV * 1e-3f;
  c.supply_max = CONFIG_TUUSHIN_SUPPLY_MAX_MV * 1e-3f;
  c.align_voltage = CONFIG_TUUSHIN_ALIGN_MV * 1e-3f;
  c.tick_us = CONFIG_TUUSHIN_TICK_US;
  c.reading_noise_lo = CONFIG_TUUSHIN_ENC_NOISE_LO_MILLICOUNTS * 1e-3f;
  c.reading_noise_hi = CONFIG_TUUSHIN_ENC_NOISE_HI_MILLICOUNTS * 1e-3f;
  if (CONFIG_TUUSHIN_PHASE_INDUCTANCE_UH > 0 && c.motor.phase_resistance > 0.0f) {
    const float lag = CONFIG_TUUSHIN_PHASE_INDUCTANCE_UH * 1e-6f / c.motor.phase_resistance;
    const float spread = 1.0f + CONFIG_TUUSHIN_CURRENT_LAG_SPREAD_PCT * 0.01f;
    c.current_lag_lo = lag / spread;
    c.current_lag_hi = lag * spread;
  }
  c.current_lag_spread = CONFIG_TUUSHIN_CURRENT_LAG_SPREAD_PCT * 0.01f;
#if CONFIG_TUUSHIN_IDENTIFY_WINDINGS
  c.identify_windings = true;
#endif
  // The compare registers take a new duty where the timer next reaches
  // zero (Bridge), a period on at the most.
  c.duty_delay = 1.0f / static_cast<float>(CONFIG_TUUSHIN_PWM_HZ);
  c.kind = device::DeviceKind::kTorqueMotor;
  std::snprintf(c.name, sizeof c.name, "%s", CONFIG_TUUSHIN_NAME);
  return c;
}

}  // namespace tuushin::esp32
