#pragma once

#include "tuushin/fw/core.hpp"

namespace tuushin::esp32 {

// The calibration in NVS. Loading happens once, before the servo loop
// starts. Storing is asked for from the servo loop, at the end of a
// calibration, and a flash write takes milliseconds during which code in
// flash cannot run, so a task of low priority does it: the servo loop only
// leaves a copy and a notification.
class CalibrationStore {
 public:
  // Opens NVS (erasing it if its layout is from another version) and
  // starts the writer task.
  bool begin();
  bool load(fw::Calibration* c);
  void store(const fw::Calibration& c);
};

}  // namespace tuushin::esp32
