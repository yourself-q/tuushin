#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "tuushin/device/proto.hpp"
#include "tuushin/device/stream.hpp"
#include "tuushin/teleop/device.hpp"
#include "tuushin/time.hpp"

// A device running the firmware in firmware/ (a motor under field-oriented
// control on a microcontroller, or its simulation), driven over a byte
// stream with the protocol of device/proto.hpp, as the controllers'
// Device1D. The knob turns; the controllers think in metres and newtons:
// x = r * angle and F = torque / r at the rim, r the knob's radius.
//
// What the controllers cannot see through Device1D, and this keeps for the
// report: the link's round trip, the States lost on the way, and the work
// the device itself books for the host's torque.

namespace tuushin::device {

enum class Calibrate { kIfNeeded, kAlways, kNever };

struct RemoteConfig {
  double radius = 0.02;  // m, where the hand holds the knob
  bool reverse = false;  // x grows the other way round
  // What is set on connecting. 0 for torque_limit and velocity_limit keeps
  // the device's own.
  double torque_limit = 0.0;    // N m, at most the device's max_torque
  double damping = 0.0;         // N m s/rad, added on the device's ticks
  double velocity_limit = 0.0;  // rad/s
  Nanos watchdog = 2 * kNanosPerMilli;
  Nanos state_period = kNanosPerMilli;
  Nanos link_timeout = 500 * kNanosPerMilli;
  Calibrate calibrate = Calibrate::kIfNeeded;
  // Makes the present position 0 on connecting.
  bool zero = true;
  Nanos connect_timeout = 5 * kNanosPerSecond;
  Nanos calibrate_timeout = 15 * kNanosPerSecond;
  // With no State for this long the device counts as gone.
  Nanos lost_after = 100 * kNanosPerMilli;
  // Reads the stream on a thread of its own, which stamps each State when
  // it arrives instead of when the control loop next looks. Otherwise the
  // caller polls (poll(), or read_position, which polls).
  bool reader_thread = false;
  // The host's clock; monotonic_now when empty. Tests give a simulated one.
  std::function<Nanos()> clock;
  // The device's Log lines, and this end's own about connecting.
  std::function<void(const std::string&)> on_log;
};

// "serial:PATH[,opt=value...]" or "tcp:HOST:PORT[,opt=value...]", with the
// options baud=N (921600), radius=M, reverse, torque-limit=NM, damping=NMS,
// velocity-limit=RADS, state-us=US, calibrate=auto|always|never, nozero.
struct DeviceSpec {
  enum class Transport { kSerial, kTcp } transport = Transport::kSerial;
  std::string path;  // serial
  std::string host;  // tcp
  std::uint16_t port = 0;
  int baud = 921600;
  RemoteConfig config;
  bool damping_given = false;  // the spec set damping=, which a caller's default must not override
};
// Throws std::runtime_error on a malformed spec.
DeviceSpec parse_device_spec(const std::string& spec);
std::unique_ptr<ByteStream> open_stream(const DeviceSpec& spec);

struct RemoteStats {
  std::uint64_t states = 0;
  std::uint64_t states_lost = 0;  // gaps in their numbering
  std::uint64_t crc_errors = 0, bad_frames = 0;
  std::uint64_t commands = 0;
  std::uint64_t commands_short = 0;  // the stream took them only in part
  // Cuts of the latest command, reported to the controller; and cuts of a
  // command while the next one was on its way, which the link's jitter
  // makes and the controller is not told of (its next command undoes them).
  std::uint64_t cuts = 0, cuts_between = 0;
  std::uint64_t saturated = 0;  // States with the torque clamped
  // From sending a command to the arrival of the first State that applies
  // it, less how long it had been applied by then (ns): the link both
  // ways and the wait for the device's next tick, without the wait for
  // its next State.
  std::vector<double> round_trip;
  Nanos worst_state_age = 0;  // the stalest State a read used (ns, host time)
};

class RemoteDevice final : public teleop::Device1D {
 public:
  RemoteDevice(std::unique_ptr<ByteStream> stream, const RemoteConfig& cfg);
  // Disables the driver.
  ~RemoteDevice() override;
  RemoteDevice(const RemoteDevice&) = delete;
  RemoteDevice& operator=(const RemoteDevice&) = delete;

  // Asks for the device's Info, clears a fault left from before, sets the
  // parameters, calibrates when needed and zeroes. The driver stays off
  // until the first command. Throws std::runtime_error.
  void connect();

  double read_position(Nanos now) override;
  double velocity() const override;
  // A force f along +x is the torque f * r in x's sense of rotation. The
  // first command switches the driver on.
  void command_force(double f) override;
  Nanos force_cut_after() const override;
  const char* fault() const override;

  // Switches the driver off; the next command_force switches it on.
  void disable();
  // Reads what has arrived; for callers without the reader thread.
  void poll();

  const Info& info() const { return info_; }
  // The device as the controllers see it, at the rim.
  double model_mass() const;          // kg: J / r^2, 0 if the device does not say
  double encoder_resolution() const;  // m
  double max_force() const;           // N: the torque limit in effect / r
  double torque_limit() const { return torque_limit_; }
  double damping() const { return damping_; }  // N m s/rad on the device's ticks
  double radius() const { return cfg_.radius; }
  Nanos state_period() const { return cfg_.state_period; }
  // Work the device booked for the host's torque since connecting (J).
  double actuator_work() const;
  // The least round trip so far (ns), 0 before any.
  Nanos least_round_trip() const;
  // The least round trip of the parameters set while connecting (ns): the
  // link both ways and the wait for the device's next tick, known before
  // any command.
  Nanos connect_round_trip() const { return ack_rtt_; }
  RemoteStats stats() const;
  std::string name() const { return stream_->name(); }

 private:
  // The device's microsecond clock, unwrapped and mapped onto the host's.
  class Clock {
   public:
    // Extends a 32-bit device time to nanoseconds near the last one seen.
    std::int64_t extend(std::uint32_t us) const;
    void observe(std::uint32_t us, Nanos arrival);
    // The host time of a device time. The least (arrival - device time)
    // is the clocks' offset plus the least delay towards the host, which
    // no one-way observation separates; it is taken as half the least
    // round trip, as NTP takes it.
    Nanos to_host(std::int64_t device_ns, Nanos least_round_trip) const {
      return device_ns + offset_ - least_round_trip / 2;
    }
    bool valid() const { return valid_; }

   private:
    bool valid_ = false;
    std::uint32_t last_us_ = 0;
    std::int64_t last_ns_ = 0;
    Nanos last_arrival_ = 0;
    Nanos offset_ = 0;
  };

  Nanos now() const;
  void say(const std::string& what) const;
  void pump_locked(Nanos now);
  void on_state(const State& s, Nanos now);
  void send(const std::uint8_t* frame, std::size_t n);
  void send_command(double torque, std::uint8_t flags);
  // Waits until `done` (checked under the lock) or `deadline`, polling.
  bool wait_for(const std::function<bool()>& done, Nanos deadline);
  float set_param(Param p, double value);
  void reader_loop();

  std::unique_ptr<ByteStream> stream_;
  RemoteConfig cfg_;
  double sign_;

  mutable std::mutex mu_;
  FrameReader reader_;
  bool failed_ = false;  // the stream closed
  bool have_info_ = false;
  Info info_;
  std::vector<Ack> acks_;
  bool have_state_ = false;
  State state_;
  Nanos state_rx_ = 0;
  Clock clock_;
  RemoteStats stats_;
  std::uint16_t cut_seq_seen_ = 0;  // the last command whose cut was counted
  bool cut_seen_ = false;
  std::uint16_t echoed_ = 0;  // the newest command seen applied
  bool echoed_any_ = false;
  Nanos least_rtt_ = 0;

  // The control thread's.
  bool connected_ = false;
  bool resync_ = false;
  std::uint16_t seq_ = 0;
  bool sent_any_ = false;
  Nanos sent_at_[64] = {};
  float torque_limit_ = 0.0f;
  float damping_ = 0.0f;
  Nanos ack_rtt_ = 0;
  double work0_ = 0.0;
  State used_;  // the State the last read used
  mutable std::string fault_;

  std::thread thread_;
  std::atomic<bool> quit_{false};
};

}  // namespace tuushin::device
