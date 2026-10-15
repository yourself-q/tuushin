#include "bridge.hpp"

#include <cstdint>

#include "driver/gpio.h"

#if SOC_MCPWM_SUPPORTED
#include "driver/mcpwm_prelude.h"
#else
#include "driver/ledc.h"
#endif

namespace tuushin::esp32 {

namespace {

constexpr int kPins[3] = {CONFIG_TUUSHIN_PWM_A_GPIO, CONFIG_TUUSHIN_PWM_B_GPIO, CONFIG_TUUSHIN_PWM_C_GPIO};

#if SOC_MCPWM_SUPPORTED

constexpr std::uint32_t kResolutionHz = 40000000;
// Counting up and down, the timer peaks at half its period: 1000 steps of
// duty at 20 kHz.
constexpr std::uint32_t kPeriodTicks = kResolutionHz / CONFIG_TUUSHIN_PWM_HZ;
constexpr std::uint32_t kPeak = kPeriodTicks / 2;

mcpwm_cmpr_handle_t g_cmp[3] = {};

#else

constexpr ledc_mode_t kMode = LEDC_LOW_SPEED_MODE;
constexpr ledc_timer_bit_t kBits = LEDC_TIMER_11_BIT;
constexpr std::uint32_t kFull = 1u << 11;
constexpr ledc_channel_t kChannels[3] = {LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2};

#endif

constexpr bool kHaveEnable = CONFIG_TUUSHIN_DRIVER_EN_GPIO >= 0;
#if CONFIG_TUUSHIN_DRIVER_EN_ACTIVE_LOW
constexpr int kEnableLevel = 0;
#else
constexpr int kEnableLevel = 1;
#endif

float clamp01(float d) { return d < 0.0f ? 0.0f : (d > 1.0f ? 1.0f : d); }

}  // namespace

bool Bridge::begin() {
  if (kHaveEnable) {
    const gpio_num_t en = static_cast<gpio_num_t>(CONFIG_TUUSHIN_DRIVER_EN_GPIO);
    gpio_reset_pin(en);
    gpio_set_direction(en, GPIO_MODE_OUTPUT);
    gpio_set_level(en, 1 - kEnableLevel);
  }
#if SOC_MCPWM_SUPPORTED
  mcpwm_timer_handle_t timer = nullptr;
  mcpwm_timer_config_t tc = {};
  tc.group_id = 0;
  tc.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
  tc.resolution_hz = kResolutionHz;
  tc.count_mode = MCPWM_TIMER_COUNT_MODE_UP_DOWN;
  tc.period_ticks = kPeriodTicks;
  if (mcpwm_new_timer(&tc, &timer) != ESP_OK) return false;
  for (int i = 0; i < 3; ++i) {
    mcpwm_oper_handle_t oper = nullptr;
    mcpwm_operator_config_t oc = {};
    oc.group_id = 0;
    if (mcpwm_new_operator(&oc, &oper) != ESP_OK) return false;
    if (mcpwm_operator_connect_timer(oper, timer) != ESP_OK) return false;
    mcpwm_comparator_config_t cc = {};
    cc.flags.update_cmp_on_tez = true;  // a new duty from the next period on
    if (mcpwm_new_comparator(oper, &cc, &g_cmp[i]) != ESP_OK) return false;
    mcpwm_gen_handle_t gen = nullptr;
    mcpwm_generator_config_t gc = {};
    gc.gen_gpio_num = kPins[i];
    if (mcpwm_new_generator(oper, &gc, &gen) != ESP_OK) return false;
    // High from the compare on the way down, through the bottom, to the
    // compare on the way up: a pulse of 2 cmp ticks in 2 kPeak, centred on
    // the bottom.
    // (One action a call: the list form's terminator macro leaves fields
    // uninitialised, which C++ warns about.)
    mcpwm_gen_compare_event_action_t up = {};
    up.direction = MCPWM_TIMER_DIRECTION_UP;
    up.comparator = g_cmp[i];
    up.action = MCPWM_GEN_ACTION_LOW;
    mcpwm_gen_compare_event_action_t down = up;
    down.direction = MCPWM_TIMER_DIRECTION_DOWN;
    down.action = MCPWM_GEN_ACTION_HIGH;
    if (mcpwm_generator_set_action_on_compare_event(gen, up) != ESP_OK ||
        mcpwm_generator_set_action_on_compare_event(gen, down) != ESP_OK) {
      return false;
    }
  }
  set(0.0f, 0.0f, 0.0f);
  if (mcpwm_timer_enable(timer) != ESP_OK) return false;
  if (mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP) != ESP_OK) return false;
#else
  ledc_timer_config_t tc = {};
  tc.speed_mode = kMode;
  tc.duty_resolution = kBits;
  tc.timer_num = LEDC_TIMER_0;
  tc.freq_hz = CONFIG_TUUSHIN_PWM_HZ;
  tc.clk_cfg = LEDC_AUTO_CLK;
  if (ledc_timer_config(&tc) != ESP_OK) return false;
  for (int i = 0; i < 3; ++i) {
    ledc_channel_config_t cc = {};
    cc.gpio_num = kPins[i];
    cc.speed_mode = kMode;
    cc.channel = kChannels[i];
    cc.timer_sel = LEDC_TIMER_0;
    cc.duty = 0;
    cc.hpoint = 0;
    if (ledc_channel_config(&cc) != ESP_OK) return false;
  }
#endif
  return true;
}

void Bridge::set(float a, float b, float c) {
  const float d[3] = {clamp01(a), clamp01(b), clamp01(c)};
  for (int i = 0; i < 3; ++i) {
#if SOC_MCPWM_SUPPORTED
    // Kept off the ends, where the compare would meet the timer's turning
    // points and the generator's actions there would be ambiguous.
    std::uint32_t cmp = static_cast<std::uint32_t>(d[i] * static_cast<float>(kPeak) + 0.5f);
    if (cmp < 1) cmp = 1;
    if (cmp > kPeak - 1) cmp = kPeak - 1;
    mcpwm_comparator_set_compare_value(g_cmp[i], cmp);
#else
    const std::uint32_t duty = static_cast<std::uint32_t>(d[i] * static_cast<float>(kFull) + 0.5f);
    ledc_set_duty(kMode, kChannels[i], duty);
    ledc_update_duty(kMode, kChannels[i]);
#endif
  }
}

void Bridge::enable(bool on) {
  if (kHaveEnable) {
    gpio_set_level(static_cast<gpio_num_t>(CONFIG_TUUSHIN_DRIVER_EN_GPIO), on ? kEnableLevel : 1 - kEnableLevel);
  }
  if (!on) set(0.0f, 0.0f, 0.0f);
}

}  // namespace tuushin::esp32
