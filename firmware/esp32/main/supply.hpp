#pragma once

#include "sdkconfig.h"

namespace tuushin::esp32 {

// The supply voltage, through a divider on an ADC pin, or the nominal one
// when there is none. A conversion takes tens of microseconds, so one is
// made every few ticks and the readings low-passed (a 20 ms time
// constant): the bridge's voltage changes slowly, and a single noisy
// conversion should not trip the core's supply check.
class Supply {
 public:
  bool begin();
  float voltage();

 private:
  float v_ = CONFIG_TUUSHIN_SUPPLY_MV * 1e-3f;
  unsigned n_ = 0;
};

}  // namespace tuushin::esp32
