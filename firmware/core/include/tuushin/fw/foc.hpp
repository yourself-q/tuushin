#pragma once

#include <cstdint>

// Field-oriented control of a three-phase brushless motor in voltage mode,
// as gimbal motors are driven: their phase resistance is high (several
// ohms), so a voltage on the q axis sets the current, and with it the
// torque, closely enough without measuring it (the "estimated current"
// mode of SimpleFOC). Single precision throughout: the ESP32's FPU has no
// double.
//
// Conventions (amplitude-invariant Clarke and Park transforms):
//   theta_e = direction * pole_pairs * theta_m - zero_electric
//   u_alpha = u_d cos(theta_e) - u_q sin(theta_e)
//   u_beta  = u_d sin(theta_e) + u_q cos(theta_e)
// and a phase-to-neutral flux linkage of amplitude lambda gives
//   torque = 1.5 * pole_pairs * lambda * i_q = kt * i_q,
//   u_q = R i_q + lambda * pole_pairs * omega_m (inductance neglected),
// along the motor's positive electrical direction.

namespace tuushin::fw {

constexpr float kPi = 3.14159265358979f;
constexpr float kTwoPi = 6.28318530717959f;
constexpr float kSqrt3 = 1.73205080756888f;

// The angle in [0, 2 pi).
float wrap_two_pi(float a);
// The angle in [-pi, pi).
float wrap_pi(float a);

// Three half-bridge voltages (0..vbus each) that put the voltage vector
// (u_d, u_q) on the windings at electrical angle theta: inverse Park,
// inverse Clarke, and the min-max zero sequence of space-vector modulation,
// centred on vbus / 2. The vector's magnitude reaches vbus / sqrt(3) before
// a bridge saturates; beyond that the bridges are clamped.
struct BridgeVoltages {
  float a, b, c;
};
BridgeVoltages space_vector(float ud, float uq, float theta, float vbus);

struct MotorParams {
  int pole_pairs = 7;
  // Phase to neutral: half the resistance measured between two terminals.
  float phase_resistance = 5.0f;  // ohm
  // N m/A (amplitude-invariant, as above). 0: from kv.
  float torque_constant = 0.0f;
  // rpm/V of line-to-line peak back-EMF, as motors are rated. 0: unknown,
  // and with torque_constant 0 the motor cannot be driven.
  float kv = 0.0f;
  // Compensate the back-EMF on u_q. Without it the motor is also damped by
  // kt^2 / R (N m s/rad), which the device then has as physical damping.
  bool back_emf = true;

  float kt() const;
  // lambda * pole_pairs: u_q per rad/s of the shaft.
  float back_emf_constant() const;
};

// The q voltage for a torque along the motor's positive electrical
// direction at shaft speed omega (rad/s, same direction), clamped to
// |u_q| <= voltage_limit; `torque_out` gets the torque that voltage gives.
float q_voltage(const MotorParams& m, float torque, float omega, float voltage_limit,
                float* torque_out);

// First-order low-pass of a finite difference: a velocity estimate from an
// encoder read at irregular times.
class VelocityFilter {
 public:
  explicit VelocityFilter(float cutoff_hz = 300.0f) : cutoff_(cutoff_hz) {}
  // A displacement of dtheta (rad) over dt (s).
  float update(float dtheta, float dt);
  float value() const { return v_; }
  void reset() { v_ = 0.0f; }
  void set_cutoff(float hz) { cutoff_ = hz; }

 private:
  float cutoff_;
  float v_ = 0.0f;
};

// Accumulates a single-turn encoder (raw counts in [0, cpr)) into a count
// over turns, taking the shortest way between readings.
class TurnCounter {
 public:
  explicit TurnCounter(std::uint32_t counts_per_rev) : cpr_(static_cast<std::int32_t>(counts_per_rev)) {}
  // Returns the step since the last reading (counts).
  std::int32_t update(std::uint32_t raw);
  std::int32_t count() const { return count_; }
  void reset(std::uint32_t raw) {
    last_ = raw;
    count_ = 0;
    started_ = true;
  }

 private:
  std::int32_t cpr_;
  std::uint32_t last_ = 0;
  std::int32_t count_ = 0;
  bool started_ = false;
};

}  // namespace tuushin::fw
