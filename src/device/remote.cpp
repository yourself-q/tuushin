#include "tuushin/device/remote.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iterator>
#include <limits>
#include <stdexcept>

#include "tuushin/rt/thread.hpp"

namespace tuushin::device {

namespace {

constexpr double kTwoPi = 6.283185307179586;
// How fast the clock map may rise without a new least delay: above the
// drift between two crystals (tens of ppm), and small against the
// delays it maps.
constexpr double kClockLeak = 2e-4;
constexpr Nanos kHelloEvery = 250 * kNanosPerMilli;
constexpr Nanos kAckWait = 300 * kNanosPerMilli;
// The commands connect() measures the actuation delay with.
constexpr int kDelayProbes = 8;
constexpr std::size_t kRttCap = std::size_t{1} << 20;

std::string fmt(const char* format, double a, double b = 0.0, double c = 0.0) {
  char buf[160];
  std::snprintf(buf, sizeof buf, format, a, b, c);
  return buf;
}

const char* param_name(Param p) {
  switch (p) {
    case Param::kWatchdogUs: return "watchdog";
    case Param::kTorqueLimit: return "torque limit";
    case Param::kStatePeriodUs: return "state period";
    case Param::kDamping: return "damping";
    case Param::kVelocityLimit: return "velocity limit";
    case Param::kLinkTimeoutUs: return "link timeout";
  }
  return "parameter";
}

double parse_number(const std::string& s, const std::string& what) {
  char* end = nullptr;
  const double v = std::strtod(s.c_str(), &end);
  if (s.empty() || *end != '\0' || !std::isfinite(v)) {
    throw std::runtime_error("bad " + what + " in the device spec: " + s);
  }
  return v;
}

}  // namespace

DeviceSpec parse_device_spec(const std::string& spec) {
  DeviceSpec d;
  std::vector<std::string> parts;
  std::size_t from = 0;
  for (;;) {
    const std::size_t comma = spec.find(',', from);
    parts.push_back(spec.substr(from, comma == std::string::npos ? std::string::npos : comma - from));
    if (comma == std::string::npos) break;
    from = comma + 1;
  }
  const std::string& where = parts[0];
  if (where.rfind("tcp:", 0) == 0) {
    d.transport = DeviceSpec::Transport::kTcp;
    const std::string hp = where.substr(4);
    const std::size_t colon = hp.rfind(':');
    if (colon == std::string::npos || colon == 0) {
      throw std::runtime_error("the device spec needs tcp:HOST:PORT: " + where);
    }
    d.host = hp.substr(0, colon);
    if (d.host.size() >= 2 && d.host.front() == '[' && d.host.back() == ']') {
      d.host = d.host.substr(1, d.host.size() - 2);
    }
    const double port = parse_number(hp.substr(colon + 1), "port");
    if (port < 1 || port > 65535 || port != std::floor(port)) {
      throw std::runtime_error("bad port in the device spec: " + where);
    }
    d.port = static_cast<std::uint16_t>(port);
  } else {
    d.path = where.rfind("serial:", 0) == 0 ? where.substr(7) : where;
    if (d.path.empty()) throw std::runtime_error("the device spec names no serial port");
  }
  RemoteConfig& c = d.config;
  for (std::size_t i = 1; i < parts.size(); ++i) {
    const std::string& opt = parts[i];
    const std::size_t eq = opt.find('=');
    const std::string key = opt.substr(0, eq);
    const std::string value = eq == std::string::npos ? std::string() : opt.substr(eq + 1);
    if (key == "baud") {
      d.baud = static_cast<int>(parse_number(value, key));
      if (d.baud <= 0) throw std::runtime_error("bad baud in the device spec: " + value);
    } else if (key == "radius") {
      c.radius = parse_number(value, key);
      if (c.radius <= 0.0) throw std::runtime_error("the radius must be positive: " + value);
    } else if (key == "reverse") {
      c.reverse = true;
    } else if (key == "torque-limit") {
      c.torque_limit = parse_number(value, key);
    } else if (key == "damping") {
      c.damping = parse_number(value, key);
      d.damping_given = true;
    } else if (key == "velocity-limit") {
      c.velocity_limit = parse_number(value, key);
    } else if (key == "state-us") {
      c.state_period = static_cast<Nanos>(parse_number(value, key) * 1e3);
    } else if (key == "calibrate") {
      if (value == "auto") c.calibrate = Calibrate::kIfNeeded;
      else if (value == "always") c.calibrate = Calibrate::kAlways;
      else if (value == "never") c.calibrate = Calibrate::kNever;
      else throw std::runtime_error("calibrate must be auto, always or never: " + value);
      d.calibrate_given = true;
    } else if (key == "nozero") {
      c.zero = false;
    } else {
      throw std::runtime_error("unknown option in the device spec: " + opt);
    }
  }
  return d;
}

std::unique_ptr<ByteStream> open_stream(const DeviceSpec& spec) {
  if (spec.transport == DeviceSpec::Transport::kTcp) return open_tcp(spec.host, spec.port);
  return open_serial(spec.path, spec.baud);
}

std::int64_t RemoteDevice::Clock::extend(std::uint32_t us) const {
  return last_ns_ + static_cast<std::int64_t>(static_cast<std::int32_t>(us - last_us_)) * 1000;
}

void RemoteDevice::Clock::observe(std::uint32_t us, Nanos arrival) {
  const std::int64_t ns = valid_ ? extend(us) : static_cast<std::int64_t>(us) * 1000;
  const Nanos sample = arrival - ns;
  if (!valid_) {
    offset_ = sample;
    valid_ = true;
  } else {
    const auto leak = static_cast<Nanos>(kClockLeak * static_cast<double>(arrival - last_arrival_));
    offset_ = std::min(offset_ + std::max<Nanos>(leak, 0), sample);
  }
  last_us_ = us;
  last_ns_ = ns;
  last_arrival_ = arrival;
}

RemoteDevice::RemoteDevice(std::unique_ptr<ByteStream> stream, const RemoteConfig& cfg)
    : stream_(std::move(stream)), cfg_(cfg), sign_(cfg.reverse ? -1.0 : 1.0) {
  if (!stream_) throw std::invalid_argument("RemoteDevice: no stream");
  if (!(cfg_.radius > 0.0)) throw std::invalid_argument("RemoteDevice: the radius must be positive");
  std::fill(std::begin(caps_), std::end(caps_), std::numeric_limits<double>::infinity());
}

RemoteDevice::~RemoteDevice() {
  quit_.store(true);
  if (thread_.joinable()) thread_.join();
  if (connected_) {
    try {
      disable();
    } catch (...) {
    }
  }
}

Nanos RemoteDevice::now() const {
  return cfg_.clock ? cfg_.clock() : monotonic_now();
}

void RemoteDevice::say(const std::string& what) const {
  if (cfg_.on_log) cfg_.on_log(what);
}

void RemoteDevice::pump_locked(Nanos t) {
  std::uint8_t buf[512];
  for (;;) {
    const long n = stream_->read(buf, sizeof buf);
    if (n < 0) {
      failed_ = true;
      break;
    }
    for (long i = 0; i < n; ++i) {
      Message m;
      if (!reader_.push(buf[i], &m)) continue;
      State s;
      Ack a;
      char text[kMaxLogText + 1];
      if (decode(m, &s)) {
        on_state(s, t);
      } else if (decode(m, &a)) {
        acks_.push_back(a);
      } else if (decode(m, &info_)) {
        have_info_ = true;
      } else if (decode_log(m, text, sizeof text)) {
        say(std::string("firmware: ") + text);
      }
    }
    if (static_cast<std::size_t>(n) < sizeof buf) break;
  }
  stats_.crc_errors = reader_.crc_errors();
  stats_.bad_frames = reader_.bad_frames();
}

void RemoteDevice::on_state(const State& s, Nanos t) {
  if (have_state_) {
    const auto gap = static_cast<std::uint16_t>(s.seq - state_.seq);
    if (gap == 0) return;  // a repeat
    if (gap < 0x8000) stats_.states_lost += gap - 1u;
  }
  clock_.observe(s.time_us, t);
  ++stats_.states;
  if (s.flags & kStateSaturated) ++stats_.saturated;
  if (sent_any_) {
    // The first State that applies a command we sent, newer than any seen.
    const auto behind = static_cast<std::uint16_t>(seq_ - s.cmd_seq);
    const bool newer = !echoed_any_ || static_cast<std::int16_t>(s.cmd_seq - echoed_) > 0;
    if (newer && behind < 64) {
      echoed_ = s.cmd_seq;
      echoed_any_ = true;
      if (s.cmd_age_us < 0xFFFF) {  // not saturated
        const Nanos rtt = t - sent_at_[s.cmd_seq & 63] - Nanos{s.cmd_age_us} * 1000;
        if (rtt > 0 && (least_rtt_ == 0 || rtt < least_rtt_)) least_rtt_ = rtt;
        if (stats_.round_trip.size() < kRttCap) stats_.round_trip.push_back(static_cast<double>(rtt));
        // On the clock the positions are placed on (sampled_at): if the
        // clocks' mapping is off, by the link's asymmetry or the least
        // delay's error, both are off alike, and the force is still timed
        // right against the positions.
        const Nanos took_effect =
            clock_.to_host(clock_.extend(s.time_us - s.cmd_age_us), least_rtt_);
        const Nanos act = took_effect - sent_at_[s.cmd_seq & 63];
        if (stats_.actuation.size() < kRttCap) stats_.actuation.push_back(static_cast<double>(act));
        act_[act_next_] = act;
        act_next_ = (act_next_ + 1) % std::size(act_);
        act_n_ = std::min(act_n_ + 1, std::size(act_));
      }
    }
    if ((s.flags & kStateCut) && (!cut_seen_ || cut_seq_seen_ != s.cmd_seq) && behind < 64) {
      cut_seen_ = true;
      cut_seq_seen_ = s.cmd_seq;
      if (s.cmd_seq == seq_) {
        ++stats_.cuts;
      } else {
        ++stats_.cuts_between;
      }
    }
  }
  state_ = s;
  state_rx_ = t;
  have_state_ = true;
}

bool RemoteDevice::wait_for(const std::function<bool()>& done, Nanos deadline) {
  for (;;) {
    {
      std::lock_guard<std::mutex> lk(mu_);
      pump_locked(now());
      if (done()) return true;
      if (failed_) return false;
    }
    const Nanos t = now();
    if (t >= deadline) return false;
    stream_->wait_readable(std::min<Nanos>(deadline - t, 20 * kNanosPerMilli));
  }
}

void RemoteDevice::send(const std::uint8_t* frame, std::size_t n) {
  std::uint8_t buf[kMaxFrame + 1];
  std::size_t k = 0;
  // After a frame that went out cut short, a delimiter ends what the device
  // has of it, so that it costs only itself.
  if (resync_) buf[k++] = 0;
  std::copy(frame, frame + n, buf + k);
  k += n;
  const long w = stream_->write(buf, k);
  std::lock_guard<std::mutex> lk(mu_);
  if (w < 0) {
    failed_ = true;
  } else {
    resync_ = static_cast<std::size_t>(w) < k;
    if (resync_ && w > 0 && buf[w - 1] == 0) resync_ = false;
  }
}

void RemoteDevice::send_command(double torque, std::uint8_t flags, double work_cap) {
  Command c;
  c.torque = static_cast<float>(torque);
  c.flags = flags;
  c.work_cap = work_cap;
  {
    std::lock_guard<std::mutex> lk(mu_);
    c.seq = ++seq_;
    sent_at_[c.seq & 63] = now();
    constexpr double kInf = std::numeric_limits<double>::infinity();
    caps_[c.seq & 63] = !(flags & kCmdEnable) || c.torque == 0.0f ? -kInf
                        : (flags & kCmdWorkCap)                    ? work_cap
                                                                   : kInf;
    sent_any_ = true;
    ++stats_.commands;
  }
  std::uint8_t frame[kMaxFrame];
  const std::size_t n = encode(c, frame);
  send(frame, n);
  if (resync_) {
    std::lock_guard<std::mutex> lk(mu_);
    ++stats_.commands_short;
  }
}

float RemoteDevice::set_param(Param p, double value) {
  for (int attempt = 0; attempt < 3; ++attempt) {
    {
      std::lock_guard<std::mutex> lk(mu_);
      acks_.clear();
    }
    Set s;
    s.param = p;
    s.value = static_cast<float>(value);
    std::uint8_t frame[kMaxFrame];
    const Nanos sent = now();
    send(frame, encode(s, frame));
    Ack got;
    const bool ok = wait_for(
        [&] {
          for (const Ack& a : acks_) {
            if (a.of == MsgType::kSet && a.param == p) {
              got = a;
              return true;
            }
          }
          return false;
        },
        now() + kAckWait);
    if (ok) {
      const Nanos rtt = now() - sent;
      if (ack_rtt_ == 0 || rtt < ack_rtt_) ack_rtt_ = rtt;
      if (got.status != AckStatus::kOk) {
        throw std::runtime_error(std::string("the device refused its ") + param_name(p));
      }
      return got.value;
    }
    if (failed_) break;
  }
  throw std::runtime_error(std::string("no answer from the device to setting its ") + param_name(p));
}

void RemoteDevice::connect() {
  if (connected_) return;
  stream_->discard_input();
  {
    // Ends whatever a previous host left half-sent.
    const std::uint8_t z = 0;
    stream_->write(&z, 1);
  }
  const Nanos deadline = now() + cfg_.connect_timeout;
  for (;;) {
    std::uint8_t frame[kMaxFrame];
    send(frame, encode_bare(MsgType::kHello, frame));
    if (wait_for([&] { return have_info_; }, std::min(deadline, now() + kHelloEvery))) break;
    if (failed_) throw std::runtime_error("the link to the device on " + name() + " closed");
    if (now() >= deadline) {
      throw std::runtime_error("no answer from a device on " + name() + " within " +
                               fmt("%.0f", to_seconds(cfg_.connect_timeout)) +
                               " s (is it running tuushin's firmware, at this baud rate?)");
    }
  }
  if (info_.protocol != kProtocolVersion) {
    throw std::runtime_error("the device speaks protocol " + std::to_string(info_.protocol) +
                             ", this host " + std::to_string(kProtocolVersion));
  }
  if (info_.counts_per_rev == 0) throw std::runtime_error("the device reports no encoder resolution");
  say(std::string("device ") + info_.name + " on " + name() + ": " +
      (info_.kind == DeviceKind::kSimulated ? "simulated motor" : "torque motor") +
      fmt(", %.0f counts/rev, %.0f pole pairs", info_.counts_per_rev, info_.pole_pairs) +
      fmt(", max %.3f N m, firmware %.0f.", info_.max_torque, info_.firmware >> 16) +
      fmt("%.0f.%.0f", (info_.firmware >> 8) & 0xFF, info_.firmware & 0xFF));

  if (!wait_for([&] { return have_state_; }, now() + 500 * kNanosPerMilli)) {
    throw std::runtime_error("the device answers but sends no State");
  }
  bool faulted;
  {
    std::lock_guard<std::mutex> lk(mu_);
    faulted = state_.fault != Fault::kNone;
    if (faulted) say(std::string("clearing a fault left from before: ") + fault_name(state_.fault));
  }
  if (faulted) {
    send_command(0.0, kCmdClearFault);
    const std::uint64_t n0 = stats_.states;
    if (!wait_for([&] { return stats_.states > n0 + 1 && state_.fault == Fault::kNone; },
                  now() + 500 * kNanosPerMilli)) {
      throw std::runtime_error(std::string("the device's fault does not clear: ") +
                               fault_name(state_.fault));
    }
  }

  set_param(Param::kWatchdogUs, static_cast<double>(cfg_.watchdog) / 1e3);
  set_param(Param::kStatePeriodUs, static_cast<double>(cfg_.state_period) / 1e3);
  set_param(Param::kLinkTimeoutUs, static_cast<double>(cfg_.link_timeout) / 1e3);
  damping_ = set_param(Param::kDamping, cfg_.damping);
  if (cfg_.velocity_limit > 0.0) set_param(Param::kVelocityLimit, cfg_.velocity_limit);
  torque_limit_ = set_param(Param::kTorqueLimit,
                            cfg_.torque_limit > 0.0 ? cfg_.torque_limit : info_.max_torque);
  if (cfg_.torque_limit > torque_limit_ + 1e-9) {
    say(fmt("torque limit %.4f N m: the device allows no more than %.4f", cfg_.torque_limit,
            torque_limit_));
  }

  bool calibrated;
  {
    std::lock_guard<std::mutex> lk(mu_);
    calibrated = (state_.flags & kStateCalibrated) != 0;
  }
  if (cfg_.calibrate == Calibrate::kAlways || (!calibrated && cfg_.calibrate == Calibrate::kIfNeeded)) {
    say("calibrating: let go of the knob; it turns a little each way");
    {
      std::lock_guard<std::mutex> lk(mu_);
      acks_.clear();
    }
    std::uint8_t frame[kMaxFrame];
    send(frame, encode_bare(MsgType::kCalibrate, frame));
    Ack got;
    const bool ok = wait_for(
        [&] {
          for (const Ack& a : acks_) {
            if (a.of == MsgType::kCalibrate) {
              got = a;
              return true;
            }
          }
          return false;
        },
        now() + cfg_.calibrate_timeout);
    if (!ok) throw std::runtime_error("the device's calibration did not finish");
    if (got.status != AckStatus::kOk) {
      throw std::runtime_error(got.status == AckStatus::kRejected
                                   ? "the device refused to calibrate"
                                   : "the device's calibration failed (its log says why)");
    }
  } else if (!calibrated && cfg_.calibrate == Calibrate::kNever) {
    throw std::runtime_error("the device is not calibrated, and calibrate=never");
  }

  if (cfg_.zero) {
    {
      std::lock_guard<std::mutex> lk(mu_);
      acks_.clear();
    }
    std::uint8_t frame[kMaxFrame];
    send(frame, encode_bare(MsgType::kZero, frame));
    const bool ok = wait_for(
        [&] {
          for (const Ack& a : acks_) {
            if (a.of == MsgType::kZero) return true;
          }
          return false;
        },
        now() + kAckWait);
    if (!ok) throw std::runtime_error("no answer from the device to zeroing it");
  }
  // The actuation delay, before any force: commands that switch nothing
  // on, each once the one before was applied. Sent as a State arrives,
  // they meet the device's tick at much the same point each time, so a
  // whole tick is added to the longest, until the session's commands
  // have measured the waits for it (actuation_delay()).
  {
    std::lock_guard<std::mutex> lk(mu_);
    act_n_ = act_next_ = 0;
  }
  for (int i = 0; i < kDelayProbes; ++i) {
    send_command(0.0, 0);
    const std::uint16_t probe = seq_;
    if (!wait_for([&] { return echoed_any_ && echoed_ == probe; }, now() + kAckWait)) break;
  }
  {
    std::lock_guard<std::mutex> lk(mu_);
    if (act_n_ > 0) {
      const Nanos longest = *std::max_element(act_, act_ + act_n_);
      std::fill(std::begin(act_), std::end(act_), longest + Nanos{info_.tick_us} * 1000);
      act_n_ = std::size(act_);
      act_next_ = 0;
    } else {
      say("the device did not apply the commands that measure its actuation delay");
    }
  }

  // A State from after all of it.
  std::uint64_t n0;
  {
    std::lock_guard<std::mutex> lk(mu_);
    n0 = stats_.states;
  }
  if (!wait_for([&] { return stats_.states > n0 + 1; }, now() + 500 * kNanosPerMilli)) {
    throw std::runtime_error("the device stopped sending States");
  }
  {
    std::lock_guard<std::mutex> lk(mu_);
    work0_ = state_.work;
    tax0_ = state_.tax;
    used_ = state_;
    stats_ = RemoteStats{};
    stats_.crc_errors = reader_.crc_errors();
    stats_.bad_frames = reader_.bad_frames();
  }
  connected_ = true;
  if (cfg_.reader_thread) thread_ = std::thread(&RemoteDevice::reader_loop, this);
}

void RemoteDevice::reader_loop() {
  // The States' arrival stamps and the control loop's view of the device
  // are only as fresh as this thread: on a loaded Mac, at the default
  // priority it left the loop with States up to 40 ms old.
  rt::prefer_low_latency(cfg_.state_period);
  while (!quit_.load()) {
    stream_->wait_readable(10 * kNanosPerMilli);
    std::lock_guard<std::mutex> lk(mu_);
    pump_locked(now());
    if (failed_) break;
  }
}

void RemoteDevice::poll() {
  std::lock_guard<std::mutex> lk(mu_);
  pump_locked(now());
}

double RemoteDevice::read_position(Nanos) {
  std::lock_guard<std::mutex> lk(mu_);
  const Nanos t = now();
  if (!cfg_.reader_thread) pump_locked(t);
  if (!have_state_) return 0.0;
  used_ = state_;
  if (connected_) stats_.worst_state_age = std::max(stats_.worst_state_age, t - state_rx_);
  return sign_ * cfg_.radius * kTwoPi * static_cast<double>(used_.count) /
         static_cast<double>(info_.counts_per_rev);
}

Nanos RemoteDevice::sampled_at(Nanos now) const {
  std::lock_guard<std::mutex> lk(mu_);
  if (!have_state_ || !clock_.valid()) return now;
  return std::min(now, clock_.to_host(clock_.extend(used_.time_us), least_rtt_));
}

std::optional<Nanos> RemoteDevice::actuation_delay() const {
  std::lock_guard<std::mutex> lk(mu_);
  if (act_n_ == 0) return std::nullopt;
  return *std::max_element(act_, act_ + act_n_);
}

double RemoteDevice::velocity() const {
  return sign_ * cfg_.radius * static_cast<double>(used_.velocity);
}

void RemoteDevice::command_force(double f) {
  if (!connected_) return;
  send_command(sign_ * f * cfg_.radius, kCmdEnable);
}

bool RemoteDevice::limits_work() const {
  return connected_ && (info_.flags & kInfoWorkCap) != 0;
}

void RemoteDevice::command_force_capped(double f, double work_cap) {
  if (!connected_) return;
  if (!limits_work()) {
    command_force(f);
    return;
  }
  send_command(sign_ * f * cfg_.radius, kCmdEnable | kCmdWorkCap, work_cap + work0_ + tax0_);
}

std::optional<double> RemoteDevice::work_reserved() const {
  if (!limits_work()) return std::nullopt;
  std::lock_guard<std::mutex> lk(mu_);
  constexpr double kInf = std::numeric_limits<double>::infinity();
  // The commands from the one the State applied to the newest; the older
  // ones are behind the device. A State that applied none of the last 64
  // leaves them unknown.
  // None sent: a command left from before was cut by the watchdog.
  if (!sent_any_) return 0.0;
  const auto behind = static_cast<std::uint16_t>(seq_ - used_.cmd_seq);
  if (behind >= 64) return kInf;
  double cap = -kInf;
  for (std::uint16_t k = 0; k <= behind; ++k) {
    cap = std::max(cap, caps_[static_cast<std::uint16_t>(used_.cmd_seq + k) & 63]);
  }
  return std::max(0.0, cap - (used_.work + used_.tax));
}

void RemoteDevice::disable() {
  send_command(0.0, 0);
}

Nanos RemoteDevice::force_cut_after() const {
  std::lock_guard<std::mutex> lk(mu_);
  if (!connected_ || !(used_.flags & kStateCut) || used_.cmd_seq != seq_ || !clock_.valid()) return 0;
  // When the device cut it, on the host's clock, from when it took effect.
  const std::uint32_t took_effect = used_.time_us - used_.cmd_age_us;
  const std::int64_t cut = clock_.extend(took_effect) + std::int64_t{used_.cut_after_us} * 1000;
  return std::max<Nanos>(1, clock_.to_host(cut, least_rtt_) - sent_at_[seq_ & 63]);
}

const char* RemoteDevice::fault() const {
  std::lock_guard<std::mutex> lk(mu_);
  if (failed_) {
    fault_ = "the link to the device closed";
  } else if (have_state_ && state_.fault != Fault::kNone) {
    fault_ = std::string("device fault: ") + fault_name(state_.fault);
  } else if (connected_ && now() - state_rx_ > cfg_.lost_after) {
    fault_ = fmt("no State from the device for %.0f ms", to_millis(now() - state_rx_));
  } else {
    return nullptr;
  }
  return fault_.c_str();
}

double RemoteDevice::model_mass() const {
  return static_cast<double>(info_.inertia) / (cfg_.radius * cfg_.radius);
}

double RemoteDevice::encoder_resolution() const {
  return info_.counts_per_rev ? cfg_.radius * kTwoPi / static_cast<double>(info_.counts_per_rev) : 0.0;
}

double RemoteDevice::max_force() const {
  return static_cast<double>(torque_limit_) / cfg_.radius;
}

std::optional<double> RemoteDevice::port_work() const {
  std::lock_guard<std::mutex> lk(mu_);
  if (!connected_) return std::nullopt;
  // The device sums the torque times the angle; with x = sign r theta and
  // the torque sign f r, that is f times x as the controllers see them.
  // Its tax is work the books have to count as done, less the tax of the
  // rises it held back, which the books took when they were commanded.
  return used_.work - work0_ + (used_.tax - tax0_);
}

double RemoteDevice::actuator_work() const {
  std::lock_guard<std::mutex> lk(mu_);
  return connected_ ? used_.work - work0_ : 0.0;
}

Nanos RemoteDevice::least_round_trip() const {
  std::lock_guard<std::mutex> lk(mu_);
  return least_rtt_;
}

RemoteStats RemoteDevice::stats() const {
  std::lock_guard<std::mutex> lk(mu_);
  return stats_;
}

State RemoteDevice::latest_state() const {
  std::lock_guard<std::mutex> lk(mu_);
  return state_;
}

}  // namespace tuushin::device
