#include "calibration_store.hpp"

#include <cstring>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"

namespace tuushin::esp32 {

namespace {

constexpr const char* kNamespace = "tuushin";
constexpr const char* kKey = "cal";
constexpr std::uint8_t kVersion = 1;

// What is stored: a version, so that a later layout is not misread.
struct Stored {
  std::uint8_t version;
  std::int8_t direction;
  std::uint8_t pad[2];
  float zero_electric;
};

TaskHandle_t g_writer = nullptr;
portMUX_TYPE g_lock = portMUX_INITIALIZER_UNLOCKED;
fw::Calibration g_pending;

void write_now(const fw::Calibration& c) {
  nvs_handle_t h;
  if (nvs_open(kNamespace, NVS_READWRITE, &h) != ESP_OK) return;
  Stored s = {};
  s.version = kVersion;
  s.direction = c.direction;
  s.zero_electric = c.zero_electric;
  if (c.valid) {
    nvs_set_blob(h, kKey, &s, sizeof s);
  } else {
    nvs_erase_key(h, kKey);
  }
  nvs_commit(h);
  nvs_close(h);
}

void writer(void*) {
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    taskENTER_CRITICAL(&g_lock);
    const fw::Calibration c = g_pending;
    taskEXIT_CRITICAL(&g_lock);
    write_now(c);
  }
}

}  // namespace

bool CalibrationStore::begin() {
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    nvs_flash_erase();
    err = nvs_flash_init();
  }
  if (err != ESP_OK) return false;
  return xTaskCreate(writer, "cal-store", 3072, nullptr, 1, &g_writer) == pdPASS;
}

bool CalibrationStore::load(fw::Calibration* c) {
  nvs_handle_t h;
  if (nvs_open(kNamespace, NVS_READONLY, &h) != ESP_OK) return false;
  Stored s;
  std::size_t n = sizeof s;
  const bool ok = nvs_get_blob(h, kKey, &s, &n) == ESP_OK && n == sizeof s && s.version == kVersion;
  nvs_close(h);
  if (!ok) return false;
  c->valid = true;
  c->direction = s.direction;
  c->zero_electric = s.zero_electric;
  return true;
}

void CalibrationStore::store(const fw::Calibration& c) {
  if (!g_writer) return;
  taskENTER_CRITICAL(&g_lock);
  g_pending = c;
  taskEXIT_CRITICAL(&g_lock);
  xTaskNotifyGive(g_writer);
}

}  // namespace tuushin::esp32
