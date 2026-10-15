#pragma once

#include <cstddef>
#include <cstdint>

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
//    shaft than the cap allows, give or take the work of a count's step;
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
  float observer_hz = 200.0f;
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
  std::int32_t count() const { return turns_.count() - zero_; }
  std::uint32_t dropped_bytes() const { return dropped_; }

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
  float keep_to_cap(float host, float dt, float before, float taxed);
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
  float asked_ = 0.0f;  // the host's torque the tick asked for, after the work cap
  bool saturated_ = false;
  bool capped_ = false;  // in some tick since the last State
  double work_ = 0.0;
  double tax_ = 0.0;  // State::tax
  std::int32_t step_ = 0;  // the shaft's last step (counts)
  // The variance of the encoder's reading errors (counts^2), measured
  // (keep_to_cap); a cautious count until it is.
  float reading_var_ = 1.0f;
  float prev_cmd_ = 0.0f;  // the torque of the command before this one
  bool fresh_cmd_ = false;  // the command came since the last tick

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
