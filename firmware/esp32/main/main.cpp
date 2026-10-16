// The firmware of a one-axis tuushin device: the servo loop of
// firmware/core on a gimbal motor, driven at a steady tick by a hardware
// timer, and the host link (device/proto.hpp) on the serial port.
//
// The tick: a general-purpose timer's alarm notifies the servo task, which
// has the highest priority and a core to itself where the chip has two;
// the task runs one tick of the core and moves the bytes the tick queued
// towards the host. The timer's interrupt is allocated on the servo's
// core, so nothing on the other core delays it. A tick still running when
// the next alarm comes is a missed tick: the next one starts at once, and
// the core measures the time between ticks itself.

#include <cinttypes>
#include <cstdio>

#include "driver/gptimer.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "calibration_store.hpp"
#include "link.hpp"
#include "tuushin/fw/core.hpp"
#if CONFIG_TUUSHIN_EMULATED
#include "emulated_hal.hpp"
#else
#include "board_hal.hpp"
#endif

using namespace tuushin;
using namespace tuushin::esp32;

namespace {

#if CONFIG_TUUSHIN_EMULATED
using BoardHalT = EmulatedHal;
#else
using BoardHalT = BoardHal;
#endif

Link g_link;
CalibrationStore g_store;
BoardHalT* g_hal = nullptr;
fw::DeviceCore* g_core = nullptr;
TaskHandle_t g_servo = nullptr;

// What the servo task counts, for the reports.
struct Counters {
  std::uint32_t ticks = 0;
  std::uint32_t missed = 0;
  std::uint32_t worst_tick_us = 0;
  std::uint32_t rearmed = 0;  // times the timer had to be restarted
};
Counters g_counters;
portMUX_TYPE g_counters_lock = portMUX_INITIALIZER_UNLOCKED;

bool IRAM_ATTR on_alarm(gptimer_handle_t, const gptimer_alarm_event_data_t*, void*) {
  BaseType_t woken = pdFALSE;
  vTaskNotifyGiveFromISR(g_servo, &woken);
  return woken == pdTRUE;
}

#if CONFIG_TUUSHIN_EMULATED

// The simulated motor as the servo task last saw it, ten times a second.
struct Snapshot {
  double t, angle, torque, work, overrun;
  bool driving, calibrating, fault;
  std::uint32_t worst_integration_us;
  std::int64_t busy_us, covered_us, lost_us;
};
Snapshot g_snap = {};
double g_calibrated_at = -1.0, g_work_at_calibration = 0.0;
portMUX_TYPE g_snap_lock = portMUX_INITIALIZER_UNLOCKED;

void take_snapshot() {
  const fw::Plant& p = g_hal->plant();
  Snapshot s;
  s.t = p.time();
  s.angle = p.angle();
  s.torque = p.motor_torque();
  s.work = p.motor_work();
  s.overrun = g_core->worst_overrun();
  s.driving = g_core->enabled();
  s.calibrating = (g_core->last_state().flags & device::kStateCalibrating) != 0;
  s.fault = g_core->fault() != device::Fault::kNone;
  s.busy_us = g_hal->busy_us();
  s.covered_us = g_hal->covered_us();
  s.lost_us = g_hal->lost_us();
  s.worst_integration_us = 0;
  taskENTER_CRITICAL(&g_snap_lock);
  s.worst_integration_us = g_snap.worst_integration_us;
  const std::uint32_t w = g_hal->take_worst_integration_us();
  if (w > s.worst_integration_us) s.worst_integration_us = w;
  g_snap = s;
  taskEXIT_CRITICAL(&g_snap_lock);
}

#endif

void servo(void*) {
  g_servo = xTaskGetCurrentTaskHandle();
  gptimer_handle_t timer = nullptr;
  gptimer_config_t tc = {};
  tc.clk_src = GPTIMER_CLK_SRC_DEFAULT;
  tc.direction = GPTIMER_COUNT_UP;
  tc.resolution_hz = 1000000;
  ESP_ERROR_CHECK(gptimer_new_timer(&tc, &timer));
  gptimer_event_callbacks_t cbs = {};
  cbs.on_alarm = on_alarm;
  // The interrupt is allocated here, on the servo's core.
  ESP_ERROR_CHECK(gptimer_register_event_callbacks(timer, &cbs, nullptr));
  gptimer_alarm_config_t ac = {};
  ac.alarm_count = CONFIG_TUUSHIN_TICK_US;
  ac.reload_count = 0;
  ac.flags.auto_reload_on_alarm = true;
  ESP_ERROR_CHECK(gptimer_set_alarm_action(timer, &ac));
  ESP_ERROR_CHECK(gptimer_enable(timer));
  ESP_ERROR_CHECK(gptimer_start(timer));
#if CONFIG_TUUSHIN_EMULATED
  constexpr std::uint32_t kSnapshotEvery = 100000 / CONFIG_TUUSHIN_TICK_US;
  std::uint32_t until_snapshot = 0;
#endif
  // A few ticks, and at least two of the scheduler's.
  const TickType_t kStall = pdMS_TO_TICKS(4 * CONFIG_TUUSHIN_TICK_US / 1000) > 2
                                ? pdMS_TO_TICKS(4 * CONFIG_TUUSHIN_TICK_US / 1000)
                                : 2;
  for (;;) {
    std::uint32_t pending = ulTaskNotifyTake(pdTRUE, kStall);
    if (pending == 0) {
      // No alarm for several ticks: the timer stopped. (QEMU's timer drops
      // an alarm set to a count already passed, where the chip raises it
      // at once, and the driver re-enables an auto-reloading alarm only
      // from its interrupt, which an emulator can run a period late.)
      // Restarting the count re-arms it.
      gptimer_set_raw_count(timer, 0);
      pending = 1;
      taskENTER_CRITICAL(&g_counters_lock);
      ++g_counters.rearmed;
      taskEXIT_CRITICAL(&g_counters_lock);
    }
    const std::int64_t start = esp_timer_get_time();
    g_core->tick();
    g_link.flush();
    const std::uint32_t took = static_cast<std::uint32_t>(esp_timer_get_time() - start);
    taskENTER_CRITICAL(&g_counters_lock);
    ++g_counters.ticks;
    if (pending > 1) g_counters.missed += pending - 1;
    if (took > g_counters.worst_tick_us) g_counters.worst_tick_us = took;
    taskEXIT_CRITICAL(&g_counters_lock);
#if CONFIG_TUUSHIN_EMULATED
    if (g_hal->after_tick(*g_core)) {
      taskENTER_CRITICAL(&g_snap_lock);
      g_calibrated_at = g_hal->plant().time();
      g_work_at_calibration = g_hal->plant().motor_work();
      taskEXIT_CRITICAL(&g_snap_lock);
    }
    if (until_snapshot-- == 0) {
      until_snapshot = kSnapshotEvery;
      take_snapshot();
    }
#endif
  }
}

#if CONFIG_TUUSHIN_EMULATED

// Once a second on the console (UART1): what the simulated motor is doing,
// and how the servo loop keeps up: the integration's load is the time it
// took over the motor time it covered in the last second, and the motor
// stands still for what it could not cover (EmulatedHal).
void report() {
  bool told_calibration = false;
  std::int64_t last_busy = 0, last_covered = 0;
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(1000));
    taskENTER_CRITICAL(&g_snap_lock);
    const Snapshot s = g_snap;
    g_snap.worst_integration_us = 0;
    const double cal_at = g_calibrated_at, cal_work = g_work_at_calibration;
    taskEXIT_CRITICAL(&g_snap_lock);
    taskENTER_CRITICAL(&g_counters_lock);
    const Counters c = g_counters;
    g_counters.worst_tick_us = 0;
    taskEXIT_CRITICAL(&g_counters_lock);
    if (!told_calibration && cal_at >= 0.0) {
      told_calibration = true;
      std::printf("emu: calibrated at %.3f s, %.4f J of motor work so far\n", cal_at, cal_work);
    }
    const std::int64_t covered = s.covered_us - last_covered;
    const int load = covered > 0 ? static_cast<int>(100 * (s.busy_us - last_busy) / covered) : 0;
    last_busy = s.busy_us;
    last_covered = s.covered_us;
    std::printf("emu: t %.1f s  angle %+.3f rad  %+.4f N m  work %.4f J (cap overrun %.1f uJ at most)%s%s%s"
                "  ticks %" PRIu32 "  missed %" PRIu32 "  worst tick %" PRIu32 " us (integration %" PRIu32
                " us, load %d%%, lost %lld ms)  timer restarts %" PRIu32 "  stack spare %u B\n",
                s.t, s.angle, s.torque, s.work, s.overrun * 1e6, s.driving ? "  driving" : "",
                s.calibrating ? "  calibrating" : "", s.fault ? "  FAULT" : "", c.ticks, c.missed,
                c.worst_tick_us, s.worst_integration_us, load, static_cast<long long>(s.lost_us / 1000),
                c.rearmed, static_cast<unsigned>(uxTaskGetStackHighWaterMark(g_servo)));
  }
}

#endif

}  // namespace


extern "C" void app_main() {
  g_store.begin();
  g_link.begin();
#if CONFIG_TUUSHIN_EMULATED
  static EmulatedHal hal(g_link, g_store);
  static fw::DeviceCore core(hal, EmulatedHal::config());
  std::printf("emu: %s, tick %d us, integration step %d us\n", CONFIG_TUUSHIN_NAME, CONFIG_TUUSHIN_TICK_US,
              CONFIG_TUUSHIN_EMULATED_STEP_US);
#else
  static BoardHal hal(g_link, g_store);
  // A part that did not come up shows to the host as the core's fault
  // (the encoder not answering), with the driver off.
  const char* failed = nullptr;
  hal.begin(&failed);
  static fw::DeviceCore core(hal, board_config());
#endif
  g_hal = &hal;
  g_core = &core;
  core.begin();
  // The fit of the windings' L / R in calibration takes some 2.5 KB of the
  // stack below the tick's frame (-fstack-usage for the ESP32-S3:
  // fit_windings 1440 B, fit_at 400, identify 352). The emulator reports
  // what was never used ("stack spare").
  xTaskCreatePinnedToCore(servo, "servo", 8192, nullptr, configMAX_PRIORITIES - 1, nullptr,
                          CONFIG_TUUSHIN_SERVO_CORE);
#if CONFIG_TUUSHIN_EMULATED
  report();
#endif
}
