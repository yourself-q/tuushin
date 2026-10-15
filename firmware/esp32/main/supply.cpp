#include "supply.hpp"

#if CONFIG_TUUSHIN_SUPPLY_ADC_GPIO >= 0
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#endif

namespace tuushin::esp32 {

#if CONFIG_TUUSHIN_SUPPLY_ADC_GPIO >= 0

namespace {

constexpr unsigned kEvery = 16;  // ticks between conversions
// The low-pass's weight per conversion: 3.2 ms between conversions at a
// 200 us tick, for a time constant of some 20 ms.
constexpr float kAlpha = 0.15f;
constexpr float kDivider = CONFIG_TUUSHIN_SUPPLY_DIVIDER_X1000 * 1e-3f;

adc_oneshot_unit_handle_t g_unit = nullptr;
adc_channel_t g_channel;
adc_cali_handle_t g_cali = nullptr;

bool read_mv(int* mv) {
  int raw = 0;
  if (adc_oneshot_read(g_unit, g_channel, &raw) != ESP_OK) return false;
  if (g_cali) return adc_cali_raw_to_voltage(g_cali, raw, mv) == ESP_OK;
  // Uncalibrated: 12 bits over some 3.1 V at 12 dB.
  *mv = raw * 3100 / 4095;
  return true;
}

}  // namespace

bool Supply::begin() {
  adc_unit_t unit;
  if (adc_oneshot_io_to_channel(CONFIG_TUUSHIN_SUPPLY_ADC_GPIO, &unit, &g_channel) != ESP_OK) return false;
  adc_oneshot_unit_init_cfg_t uc = {};
  uc.unit_id = unit;
  if (adc_oneshot_new_unit(&uc, &g_unit) != ESP_OK) return false;
  adc_oneshot_chan_cfg_t cc = {};
  cc.atten = ADC_ATTEN_DB_12;
  cc.bitwidth = ADC_BITWIDTH_DEFAULT;
  if (adc_oneshot_config_channel(g_unit, g_channel, &cc) != ESP_OK) return false;
  // The chip's calibration from eFuse, where it has one.
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
  adc_cali_curve_fitting_config_t cal = {};
  cal.unit_id = unit;
  cal.chan = g_channel;
  cal.atten = ADC_ATTEN_DB_12;
  cal.bitwidth = ADC_BITWIDTH_DEFAULT;
  if (adc_cali_create_scheme_curve_fitting(&cal, &g_cali) != ESP_OK) g_cali = nullptr;
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
  adc_cali_line_fitting_config_t cal = {};
  cal.unit_id = unit;
  cal.atten = ADC_ATTEN_DB_12;
  cal.bitwidth = ADC_BITWIDTH_DEFAULT;
#if CONFIG_IDF_TARGET_ESP32
  cal.default_vref = 1100;
#endif
  if (adc_cali_create_scheme_line_fitting(&cal, &g_cali) != ESP_OK) g_cali = nullptr;
#endif
  int mv;
  if (!read_mv(&mv)) return false;
  v_ = static_cast<float>(mv) * 1e-3f * kDivider;
  return true;
}

float Supply::voltage() {
  if (++n_ >= kEvery) {
    n_ = 0;
    int mv;
    if (read_mv(&mv)) v_ += kAlpha * (static_cast<float>(mv) * 1e-3f * kDivider - v_);
  }
  return v_;
}

#else

bool Supply::begin() { return true; }
float Supply::voltage() { return v_; }

#endif

}  // namespace tuushin::esp32
