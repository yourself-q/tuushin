#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <utility>
#include <vector>

#include "tuushin/fw/core.hpp"

// A simulated gimbal motor with a knob on its shaft, an encoder, and
// optionally a hand on the knob: what the firmware's core drives when there
// is no hardware (tuushin-devsim, the tests, an emulator). It is driven the
// way the board is, by three duty cycles, and read the way the board is, by
// a raw single-turn encoder count, so that the core's field-oriented
// control, its calibration and its checks are exercised as they will be on
// the motor.
//
// Electrically: the bridge voltages, referred to the windings' neutral,
// pass through the motor's true Clarke and Park transforms at its true
// electrical angle; the q current is (u_q - back-EMF) / R and makes the
// torque (inductance neglected: gimbal motors' L/R is a fraction of a
// millisecond). Mechanically: a rotor and knob with viscous and Coulomb
// friction, cogging, and the hand, integrated with RK4 under the held
// voltages, the torque recomputed at each stage as the rotor turns.

namespace tuushin::fw {

struct PlantParams {
  int pole_pairs = 7;
  double phase_resistance = 5.0;  // ohm, phase to neutral
  double kt = 0.08;               // N m/A, amplitude-invariant
  double inertia = 7e-6;          // kg m^2, rotor and knob
  double viscous = 2e-6;          // N m s/rad
  double coulomb = 1e-3;          // N m
  double cogging = 1.5e-3;        // N m, amplitude
  int cogging_per_rev = 84;       // lcm(slots, poles): 12N14P
  std::uint32_t counts_per_rev = 16384;
  double encoder_noise = 0.5;  // counts, standard deviation
  // How the motor really sits under the encoder, which the calibration has
  // to find: theta_e = direction * p * theta - electrical_offset.
  int direction = 1;
  double electrical_offset = 1.0;  // rad
  double supply = 12.0;  // V
  double start_angle = 0.3;  // rad, in the encoder's frame
};

// A hand holding the knob: a spring and a damper at the rim, pulling it
// towards a reference that moves as in teleop::OperatorParams.
struct HandParams {
  bool present = false;
  double k = 0.12;   // N m/rad (300 N/m at a 20 mm radius)
  double b = 2e-3;   // N m s/rad
  double inertia = 8e-6;  // kg m^2 the fingers add (some 20 g at the rim)
  double ref_offset = 0.0;     // rad, from where the hand takes hold
  double ref_amplitude = 2.0;  // rad
  double ref_freq = 0.5;       // Hz
};

class Plant {
 public:
  explicit Plant(const PlantParams& p, std::uint64_t seed = 1);
  // Bridge duty cycles, held until the next call; with the driver off the
  // windings float and carry no current.
  void set_duties(double a, double b, double c);
  void enable(bool on) { enabled_ = on; }
  // Advances to `t` (s).
  void advance(double t);
  // The encoder's reading now, with its noise.
  std::uint32_t encoder_raw();

  void set_hand(const HandParams& h, double t0);
  // An external torque (N m) besides the hand, held until changed.
  void set_load(double torque) { load_ = torque; }

  double time() const { return t_; }
  double angle() const { return theta_; }  // rad, over turns
  double velocity() const { return omega_; }
  // The motor's electromagnetic torque at the present state (N m).
  double motor_torque() const;
  // Work done on the shaft by the motor so far (J).
  double motor_work() const { return work_; }
  double hand_torque() const;
  const PlantParams& params() const { return p_; }

 private:
  double torque_at(double theta, double omega) const;
  double hand_at(double t, double theta, double omega) const;

  PlantParams p_;
  HandParams hand_;
  double hand_t0_ = 0.0;
  double load_ = 0.0;
  bool enabled_ = false;
  double va_ = 0.0, vb_ = 0.0, vc_ = 0.0;
  double t_ = 0.0, theta_, omega_ = 0.0;
  double work_ = 0.0;
  std::uint64_t rng_;
  double spare_ = 0.0;
  bool have_spare_ = false;
};

// The board, simulated: a Hal over a Plant, on a clock the caller sets, with
// the host link as two byte queues. A latency delays the bytes each way,
// as a USB serial link does (full-speed USB polls once per millisecond).
class SimHal : public Hal {
 public:
  SimHal(Plant& plant, double latency = 0.0) : plant_(plant), latency_(latency) {}

  // The board's clock (s); the plant advances to it on the next encoder read.
  void set_time(double t) { now_ = t; }
  double now() const { return now_; }

  std::uint32_t micros() override;
  bool read_encoder(std::uint32_t* raw) override;
  void set_duties(float a, float b, float c) override;
  void enable_driver(bool on) override;
  float supply_voltage() override { return static_cast<float>(plant_.params().supply); }
  std::size_t read_bytes(std::uint8_t* buf, std::size_t cap) override;
  std::size_t write_bytes(const std::uint8_t* buf, std::size_t n) override;
  bool load_calibration(Calibration* c) override;
  void store_calibration(const Calibration& c) override;

  // The host's end of the link.
  void host_write(const std::uint8_t* buf, std::size_t n);
  std::size_t host_read(std::uint8_t* buf, std::size_t cap);
  // Whether bytes for the host have arrived by now.
  bool host_readable() const { return !to_host_.empty() && to_host_.front().first <= now_; }

  bool driver_on() const { return driver_; }
  // Fails the encoder from now on, as a loose wire does.
  void break_encoder(bool broken) { encoder_broken_ = broken; }
  Calibration stored;

 private:
  Plant& plant_;
  double latency_;
  double now_ = 0.0;
  bool driver_ = false;
  bool encoder_broken_ = false;
  std::deque<std::pair<double, std::uint8_t>> to_device_, to_host_;
};

}  // namespace tuushin::fw
