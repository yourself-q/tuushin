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
// electrical angle; the q current makes the torque. Without an inductance
// it is (u_q - back-EMF) / R at once; with one, the windings' currents
// follow the voltages over L / R, and with the driver off they run down
// through the bridge's diodes against the supply. Mechanically: a rotor
// and knob with viscous and Coulomb friction, cogging, and the hand,
// integrated with RK4 under the held voltages, the torque recomputed at
// each stage as the rotor turns.

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
  // The windings' inductance, phase to neutral (H; 0: neglected). A
  // gimbal motor's L / R is some 0.3 ms: a GBM2804-100T was measured at
  // 2.0 mH (q axis) against 6.3 ohm.
  double phase_inductance = 0.0;
  // The PWM's period (s; 0: none): new duties take effect where the next
  // period starts, as a bridge's compare registers update at the counter's
  // zero.
  double pwm_period = 0.0;
  double start_angle = 0.3;  // rad, in the encoder's frame
  // The longest integration step (s). The knob against the stop rings at
  // some 85 Hz, so a coarse step still integrates it closely: driven into
  // the stop by a host at 1 kHz and ticked once a millisecond, a 250 us
  // step keeps the angle within 0.5 mrad rms (4 mrad at worst) of a 5 us
  // one, about as a 100 us step does, where a 500 us step strays 4 mrad
  // rms (14 at worst). The emulated board (firmware/esp32), which has to
  // keep up in real time without a double-precision FPU, takes 250 us.
  // With an inductance the step is at most half of L / R, which the
  // currents settle over, while they flow.
  double step = 20e-6;
  // Computes the forces in single precision, keeping the state, the clock
  // and the work in double. Where doubles are done in software (the
  // emulated board) the forces' sines and tanh are most of the cost, and
  // their float versions some five to ten times cheaper.
  bool float_forces = false;
};

// Something the knob runs into as it turns up past `angle`: a spring and
// a damper that only push (a robot's knob touching an object).
struct StopParams {
  bool present = false;
  double angle = 1.0;  // rad
  double k = 2.0;      // N m/rad (5000 N/m at a 20 mm radius)
  double b = 2e-3;     // N m s/rad
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
  // Bridge duty cycles, held until the next call, from where the PWM's
  // next period starts (PlantParams::pwm_period). With the driver off the
  // windings float; a current still in them runs down through the diodes.
  void set_duties(double a, double b, double c);
  void enable(bool on);
  // Advances to `t` (s).
  void advance(double t);
  // The encoder's reading now, with its noise.
  std::uint32_t encoder_raw();

  void set_hand(const HandParams& h, double t0);
  void set_stop(const StopParams& s) { stop_ = s; }
  // An external torque (N m) besides the hand, held until changed.
  void set_load(double torque) { load_ = torque; }

  double time() const { return t_; }
  double angle() const { return theta_; }  // rad, over turns
  double velocity() const { return omega_; }
  // The motor's electromagnetic torque at the present state (N m).
  double motor_torque() const;
  // The q current at the present state (A, along the motor's electrical
  // direction).
  double q_current() const;
  // Work done on the shaft by the motor so far (J).
  double motor_work() const { return work_; }
  double hand_torque() const;
  double stop_torque() const;
  const PlantParams& params() const { return p_; }

 private:
  // The windings' currents (A, alpha and beta) at `t` since the driver
  // went off, with an inductance: running down through the bridge's
  // diodes, which put the supply against them, and through the windings'
  // resistance. The supply is taken at the hexagon's inner circle, the
  // slowest the diodes ever take it, and the back-EMF is neglected (at
  // 10 rad/s a tenth of it), so the current is gone sooner than over L / R
  // alone, the slowest a driver that holds zero volts lets it go.
  void off_currents(double t, double* ia, double* ib) const;
  // When they are gone (s).
  double off_end() const;
  // The forces at a state, computed in F (PlantParams::float_forces).
  // The voltage on the windings, alpha and beta, referred to their neutral.
  template <class F>
  void voltage_ab(F* alpha, F* beta) const;
  // The q current at a state: (u_q - back-EMF) / R without an inductance,
  // the windings' currents ia, ib along q with one. Given `dia`, with an
  // inductance, also the currents' rates under the driver, from the same
  // sine and cosine.
  template <class F>
  F q_current_at(double theta, double omega, double ia, double ib, double* dia = nullptr,
                 double* dib = nullptr) const;
  template <class F>
  F hand_at(double t, double theta, double omega) const;
  template <class F>
  F stop_at(double theta, double omega) const;
  template <class F>
  void integrate(double t);

  PlantParams p_;
  HandParams hand_;
  double hand_t0_ = 0.0;
  StopParams stop_;
  double load_ = 0.0;
  bool enabled_ = false;
  double va_ = 0.0, vb_ = 0.0, vc_ = 0.0;
  // Duties set, and where they take effect (PlantParams::pwm_period).
  bool pending_ = false;
  double pending_at_ = 0.0;
  double pa_ = 0.0, pb_ = 0.0, pc_ = 0.0;
  double t_ = 0.0, theta_, omega_ = 0.0;
  double ia_ = 0.0, ib_ = 0.0;  // the windings' currents (A), with an inductance
  // Where the driver went off, and the currents then: magnitude and
  // direction.
  double off_t_ = 0.0, off_s_ = 0.0, off_a_ = 0.0, off_b_ = 0.0;
  double work_ = 0.0;
  std::uint64_t rng_;
  double spare_ = 0.0;
  bool have_spare_ = false;
};

// The firmware's view of a simulated motor: its parameters as a datasheet
// would give them, the knob's inertia, and kSimulated for its kind; the
// encoder's noise within 10 % of the plant's, as the host is told it in
// the teleoperation's simulation (sim::Teleop1DConfig).
DeviceConfig device_config_for(const PlantParams& p, const char* name = "sim");

// The board, simulated: a Hal over a Plant, on a clock the caller sets, with
// the host link as two byte queues. A latency delays the bytes each way,
// as a USB serial link does (full-speed USB polls once per millisecond).
class SimHal : public Hal {
 public:
  // `latency` (s) each way on the link, or with `latency_to_host` >= 0
  // only towards the device.
  SimHal(Plant& plant, double latency = 0.0, double latency_to_host = -1.0)
      : plant_(plant),
        latency_(latency),
        latency_to_host_(latency_to_host >= 0.0 ? latency_to_host : latency) {}

  // The board's clock (s); the plant advances to it on the next encoder read.
  void set_time(double t) { now_ = t; }
  double now() const { return now_; }
  // How fast the board's crystal runs (ppm): micros() counts the time set
  // that much fast, as a board's against its host's, while the plant and
  // the link keep to the time set.
  void set_clock_ppm(double ppm) { rate_ = 1.0 + ppm * 1e-6; }

  std::uint32_t micros() override;
  bool read_encoder(std::uint32_t* raw) override;
  bool reread_encoder(std::uint32_t* raw) override;
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
  // What reading the encoder again gives: a conversion of its own at the
  // same instant (the plant does not move within a tick), none, or the
  // first reading again, as an encoder that filters over the tick does.
  enum class Reread { kFresh, kNone, kSame };
  void set_reread(Reread r) { reread_ = r; }
  Calibration stored;

 private:
  Plant& plant_;
  double latency_;
  double latency_to_host_;
  double now_ = 0.0;
  double rate_ = 1.0;
  bool driver_ = false;
  bool encoder_broken_ = false;
  Reread reread_ = Reread::kFresh;
  std::uint32_t last_raw_ = 0;
  std::deque<std::pair<double, std::uint8_t>> to_device_, to_host_;
};

}  // namespace tuushin::fw
