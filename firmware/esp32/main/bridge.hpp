#pragma once

#include "sdkconfig.h"
#include "soc/soc_caps.h"

namespace tuushin::esp32 {

// The three half-bridges of a 3-PWM driver (DRV8313, L6234, the SimpleFOC
// Mini's): a duty cycle each, and an enable that opens all three.
//
// Where the chip has an MCPWM (ESP32, ESP32-S3), the three phases come
// from one timer counting up and down: their pulses are centred on the
// same instant, as space-vector modulation assumes, and a new duty takes
// effect at the period's start, never in the middle of a pulse. The
// ESP32-C3 has no MCPWM; its LEDC gives edge-aligned pulses from one
// timer, which puts a small common-mode ripple on the windings but the
// same mean voltages.
//
// Without an enable pin, off is all three phases low: no voltage across
// the windings, but they are shorted through the low sides, and a turning
// rotor is braked by its own back-EMF (kt^2 / R, some 1e-3 N m s/rad for
// a gimbal motor). The device's damping is then not zero when the host
// thinks it is.
class Bridge {
 public:
  bool begin();
  // Duty cycles in 0..1, held until the next call.
  void set(float a, float b, float c);
  void enable(bool on);
};

}  // namespace tuushin::esp32
