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
  // The standard deviation of value() at rest, for readings whose errors
  // are independent with standard deviation `reading_sd` (rad), updated
  // every dt: a reading error enters one difference and leaves through
  // the next, and the filter's gain a passes a^2 * 2 / (2 - a) of the
  // variance of their sum.
  float spread(float reading_sd, float dt) const;
  void reset() { v_ = 0.0f; }
  void set_cutoff(float hz) { cutoff_ = hz; }

 private:
  float cutoff_;
  float v_ = 0.0f;
};

// A velocity observer with the motor's torque fed forward, after R. D.
// Lorenz and K. W. Van Patten, "High-resolution velocity estimation for
// all-digital, AC servo drives", IEEE T-IA 27(4), 1991: a rigid rotor of
// inertia J under the motor's torque and another it estimates (a hand, an
// object, friction, and whatever of the rotor's inertia J leaves out),
// corrected by the encoder. The motor's own torque moves the estimate
// within the tick it acts, where a filter of the readings lags it; the
// other torque is a constant to the model, which the readings correct
// with the observer's three poles at `bandwidth_hz`. Without an inertia
// (0) the model takes the acceleration for a constant instead.
//
// A hand holding the knob adds an inertia the device does not know, and
// an observer that takes the knob's alone sees a cut in the torque slow
// the shaft twice as fast as it does. The estimate is linear in the
// torque over the inertia, so the observer keeps the readings' part and
// the torque's part apart, and gives the estimate for any inertia J /
// share at once: share 1 is the rotor alone, 0 a shaft held by something
// without limit, which the torque does not move.
class VelocityObserver {
 public:
  VelocityObserver(float bandwidth_hz, float inertia, float nominal_dt);
  // The shaft moved dtheta (rad) over dt (s) under the motor's torque
  // (N m, constant over dt).
  void update(float dtheta, float dt, float torque);
  float velocity(float share = 1.0f) const { return w_ + share * wt_; }  // rad/s
  // The other torque's acceleration (rad/s^2).
  float acceleration(float share = 1.0f) const { return acc_ + share * acct_; }
  // The other torque (N m); 0 without an inertia.
  float disturbance() const { return acceleration() * inertia_; }
  // As VelocityFilter::spread, for the nominal tick. The torque's part
  // is free of the readings' errors.
  float spread(float reading_sd, float dt) const {
    return dt > 0.0f ? reading_sd / dt * noise_gain_ : 0.0f;
  }
  void reset() { r_ = w_ = acc_ = rt_ = wt_ = acct_ = 0.0f; }

 private:
  float hz_, inertia_;
  float noise_gain_;
  // The readings' part: as if the torque moved nothing.
  float r_ = 0.0f;    // the position estimate less the reading (rad)
  float w_ = 0.0f;    // rad/s
  float acc_ = 0.0f;  // rad/s^2
  // The torque's part, for the rotor alone, as if every reading were 0.
  float rt_ = 0.0f, wt_ = 0.0f, acct_ = 0.0f;
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
