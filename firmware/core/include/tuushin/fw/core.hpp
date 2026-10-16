#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>

#include "tuushin/control/noise_tax.hpp"
#include "tuushin/device/proto.hpp"
#include "tuushin/fw/foc.hpp"

// The servo loop of a one-axis force-feedback device on a microcontroller:
// encoder in, torque out by field-oriented control, the host link
// (device/proto.hpp), a watchdog, and the checks that keep a motor next to
// a hand from running away. It is plain C++ against the Hal below, so the
// same code runs on the board (firmware/esp32), on a simulated motor on a
// host (tuushin-devsim, the tests) and in an emulator (QEMU).
//
// Safety, in the order a tick applies it:
//  - the driver is off until the host enables it, and goes off again on a
//    command without kCmdEnable, on a fault, or when no command has come
//    for the link timeout (the host is gone); while enabled, it leaves the
//    windings open whenever there is no torque to make;
//  - the watchdog cuts the torque to 0 when no command has come for its
//    timeout, as haptic devices do when the host stalls, and the next
//    command restores it;
//  - the torque never exceeds the configured ceiling, whatever the host
//    asks, nor the voltage limit;
//  - with a work cap in the command, the torque does no more work on the
//    shaft than the cap allows, give or take what a tick's step does that
//    the readings could not foresee (keep_to_cap), the work counted with
//    the tax of every change of the torque, which bounds what the
//    encoder's resolution and noise hide (State::tax);
//  - an encoder that stops answering, a reading that jumps further than
//    the shaft can move in a tick, or a speed over the limit latches a
//    fault, which only a command without kCmdEnable can clear.

namespace tuushin::fw {

struct Calibration {
  bool valid = false;
  std::int8_t direction = 1;  // +1: the electrical angle grows with the encoder's
  float zero_electric = 0.0f;  // rad, theta_e = direction * p * theta_m - zero_electric
};

// What the core needs from the board. No call may block.
class Hal {
 public:
  virtual ~Hal() = default;
  // A free-running microsecond clock; it may wrap.
  virtual std::uint32_t micros() = 0;
  // The encoder's single-turn reading in [0, counts_per_rev); false if it
  // did not answer.
  virtual bool read_encoder(std::uint32_t* raw) = 0;
  // The encoder read again, after the duties set last took effect, from a
  // conversion of its own: noise independent of the first reading's. False
  // if the board cannot take one (an encoder that filters over longer than
  // the tick, as the AS5600 does, returns the first reading's error again,
  // which is no second reading). See "Reading again" in core.cpp.
  virtual bool reread_encoder(std::uint32_t* /*raw*/) { return false; }
  // The three half-bridges' duty cycles, 0..1.
  virtual void set_duties(float a, float b, float c) = 0;
  virtual void enable_driver(bool on) = 0;
  // The supply voltage (V): measured, or the nominal one.
  virtual float supply_voltage() = 0;
  // Bytes from the host that have arrived, up to `cap`.
  virtual std::size_t read_bytes(std::uint8_t* buf, std::size_t cap) = 0;
  // Queues bytes to the host; returns how many it took.
  virtual std::size_t write_bytes(const std::uint8_t* buf, std::size_t n) = 0;
  // Persistent storage of the calibration; by default nothing persists.
  virtual bool load_calibration(Calibration*) { return false; }
  virtual void store_calibration(const Calibration&) {}
};

struct DeviceConfig {
  MotorParams motor;
  std::uint32_t counts_per_rev = 16384;
  // The torque ceiling (N m), which the host can lower but not raise.
  float max_torque = 0.1f;
  float inertia = 0.0f;  // kg m^2, reported to the host
  // |u_q| limit (V); 0: what the supply allows, supply / sqrt(3).
  float voltage_limit = 0.0f;
  // The supply's acceptable range (V); a reading outside it is a fault. 0: unchecked.
  float supply_min = 0.0f;
  float supply_max = 0.0f;
  // The d-axis voltage that pulls the rotor onto an electrical angle during
  // calibration: enough to beat cogging and friction, little enough to
  // keep the current low. Gimbal motors take about 1-3 V.
  float align_voltage = 2.0f;
  std::uint32_t tick_us = 200;  // the nominal tick, reported to the host
  // Defaults of what the host can set (device::Param).
  std::uint32_t watchdog_us = 2000;
  std::uint32_t state_period_us = 1000;
  std::uint32_t link_timeout_us = 500000;
  float velocity_limit = 300.0f;  // rad/s
  float damping = 0.0f;           // N m s/rad
  float velocity_cutoff_hz = 300.0f;
  // The poles of the velocity observer the work cap predicts with.
  // Faster ones spread more with the readings' errors, slower ones lag a
  // stiff contact's push further: at a 1 ms tick, 300 Hz rather than 200
  // cut the slave's median overrun in device_lab.sh from 120 to about 30
  // uJ, and at 200 us neither showed (keep_to_cap).
  float observer_hz = 300.0f;
  // The encoder's noise before its rounding to a count: a standard
  // deviation within [lo, hi] counts, which the tax of the torque's changes
  // then bounds with probability 1 - noise_tax_delta (control::NoiseTax),
  // tightest for a change of about noise_tax_torque (N m). hi = 0: readings
  // off by at most a count, taxed a count per N m by which the torque
  // grows (control::quantization_tax), which bounds nothing once they
  // dither (control/noise_tax.hpp).
  float reading_noise_lo = 0.0f;
  float reading_noise_hi = 0.0f;
  float noise_tax_delta = 1e-3f;
  float noise_tax_torque = 0.005f;  // the host's 0.25 N at a 20 mm radius
  device::DeviceKind kind = device::DeviceKind::kTorqueMotor;
  std::uint32_t firmware = 0x000200;
  char name[16] = "tuushin";
};

class DeviceCore {
 public:
  DeviceCore(Hal& hal, const DeviceConfig& cfg);
  // Loads the calibration, switches the driver off, reads the encoder once.
  void begin();
  // One tick of the servo loop: the encoder, the host's messages, the
  // checks, the torque, and a State when one is due. Call it at a steady
  // rate (cfg.tick_us); it measures the time between calls itself.
  void tick();

  // Introspection, for tests and diagnostics.
  const device::State& last_state() const { return st_; }
  bool enabled() const { return enabled_; }
  const Calibration& calibration() const { return cal_; }
  device::Fault fault() const { return fault_; }
  // The host's torque as applied in the last tick (after limits and cuts),
  // without the damping the loop adds.
  float applied_torque() const { return applied_; }
  // The books as of the last tick: State::work and State::tax.
  double work() const { return work_; }
  double tax() const { return tax_; }
  std::int32_t count() const { return turns_.count() - zero_; }
  std::uint32_t dropped_bytes() const { return dropped_; }
  // The most a tick's torque and tax have run past the work cap it kept
  // to (J): what keep_to_cap did not foresee.
  double worst_overrun() const { return worst_overrun_; }
  // The noise tax's books, if the configuration gives the encoder's noise.
  const control::NoiseTax* noise_tax() const { return noise_ ? &*noise_ : nullptr; }
  // How many second readings the books were drawn at, and how independent
  // their errors look from the first readings': the mean square of their
  // difference, less the motion the velocity estimate predicts, over twice
  // the readings' error variance, which is 1 - their correlation (1:
  // independent, 0: the same error again).
  std::uint32_t rereads() const { return rereads_; }
  float reread_independence() const;

 private:
  enum class CalPhase { kNone, kSettle0, kSweepUp, kSettle1, kSweepDown, kSettle2 };

  void handle(const device::Message& m, std::uint32_t now);
  void on_command(const device::Command& c, std::uint32_t now);
  void on_set(const device::Set& s);
  void start_calibration(std::uint32_t now);
  void run_calibration(std::uint32_t now);
  void latch(device::Fault f);
  void set_enabled(bool on);
  // The bridge: driving the windings, or open (no current).
  void drive(bool on);
  void foc(float host, float omega, float vbus);
  float keep_to_cap(float host, float dt, float before);
  // The tax of a change of the torque applied (J), booked; and a line that
  // bounds what a change from `before`, the torque applied last, would be
  // booked, exact at `target` (control::NoiseTax::plan).
  double charge_change(float before, float to);
  control::ChangeTax change_line(float before, float target) const;
  void open_noise_books();
  void reread(float was, float omega, std::uint32_t now);
  void send(const std::uint8_t* frame, std::size_t n);
  void log(const char* text);
  void send_state(std::uint32_t now);
  float electrical_angle() const;
  float voltage_limit(float vbus) const;

  Hal& hal_;
  DeviceConfig cfg_;
  device::FrameReader reader_;
  Calibration cal_;
  TurnCounter turns_;
  VelocityFilter vel_;
  VelocityObserver obs_;
  std::int32_t zero_ = 0;  // turns_.count() at the host's zero
  std::uint32_t raw_ = 0;
  bool started_ = false;
  std::uint32_t last_us_ = 0;

  // The host's parameters.
  std::uint32_t watchdog_us_, state_period_us_, link_timeout_us_;
  float torque_limit_, velocity_limit_, damping_;

  bool enabled_ = false;  // the host's: the device applies its commands
  bool driving_ = false;
  device::Fault fault_ = device::Fault::kNone;
  bool have_cmd_ = false;
  device::Command cmd_;
  std::uint32_t cmd_us_ = 0;   // when the command in effect took effect
  bool cut_ = false;
  std::uint32_t cut_after_us_ = 0;
  float applied_ = 0.0f;
  float motor_torque_ = 0.0f;  // the host's and the damping's, as the motor makes them (N m)
  bool saturated_ = false;
  bool capped_ = false;  // in some tick since the last State
  double work_ = 0.0;
  // The count (turns_'s) the torque applied acts from in the books: the
  // reading it took effect on, or the second reading after it did.
  std::int32_t bound_ = 0;
  double tax_ = 0.0;  // State::tax
  bool noisy_ = false;  // the configuration gives the encoder's noise
  std::optional<control::NoiseTax> noise_;
  std::uint32_t rereads_ = 0;
  float reread_var_ = 0.0f;  // the mean square of second less first readings (counts^2)
  bool reread_told_ = false;
  // The work cap the last tick kept to, and work_ + tax_ as it decided.
  bool cap_held_ = false;
  double cap_used_ = 0.0;
  double booked_ = 0.0;
  double worst_overrun_ = 0.0;
  std::int32_t step_ = 0;  // the shaft's last step (counts)
  // The variance of the encoder's reading errors (counts^2), measured
  // (keep_to_cap); a cautious count until it is.
  float reading_var_ = 1.0f;

  CalPhase cal_phase_ = CalPhase::kNone;
  std::uint32_t cal_start_us_ = 0;
  float cal_theta_[3] = {0.0f, 0.0f, 0.0f};  // shaft angle at the three settles (rad, raw)
  float cal_turn_[3] = {0.0f, 0.0f, 0.0f};   // and over turns

  std::uint16_t state_seq_ = 0;
  std::uint32_t next_state_us_ = 0;  // the next slot on the State lattice
  device::State st_;
  bool resync_ = false;  // a frame went out cut short: send a delimiter first
  std::uint32_t dropped_ = 0;
};

}  // namespace tuushin::fw
