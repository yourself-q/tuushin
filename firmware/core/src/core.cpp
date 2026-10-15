#include "tuushin/fw/core.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>

namespace tuushin::fw {

namespace {

using namespace tuushin::device;

// Calibration: pull the rotor onto electrical angle 0, turn the field one
// electrical revolution forward, then back, settling at each end. Where the
// shaft settles gives the electrical zero, which way it turned the
// direction, and how far it turned checks the pole pairs (SimpleFOC's
// initFOC does the same, forward only).
constexpr std::uint32_t kSettle0Us = 700000;  // the first 200 ms ramp the voltage up
constexpr std::uint32_t kRampUs = 200000;
constexpr std::uint32_t kSweepUs = 800000;
constexpr std::uint32_t kSettleUs = 400000;

std::uint16_t saturate16(std::uint32_t v) {
  return v > 0xFFFF ? 0xFFFF : static_cast<std::uint16_t>(v);
}

float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

}  // namespace

DeviceCore::DeviceCore(Hal& hal, const DeviceConfig& cfg)
    : hal_(hal),
      cfg_(cfg),
      turns_(cfg.counts_per_rev),
      vel_(cfg.velocity_cutoff_hz),
      watchdog_us_(cfg.watchdog_us),
      state_period_us_(cfg.state_period_us),
      link_timeout_us_(cfg.link_timeout_us),
      torque_limit_(cfg.max_torque),
      velocity_limit_(cfg.velocity_limit),
      damping_(cfg.damping) {}

void DeviceCore::begin() {
  hal_.enable_driver(false);
  hal_.set_duties(0.0f, 0.0f, 0.0f);
  Calibration c;
  if (hal_.load_calibration(&c) && c.valid && (c.direction == 1 || c.direction == -1)) cal_ = c;
  std::uint32_t raw;
  if (hal_.read_encoder(&raw)) {
    turns_.reset(raw);
    raw_ = raw;
  } else {
    latch(Fault::kEncoder);
  }
  last_us_ = hal_.micros();
  last_state_us_ = last_us_;
  started_ = true;
}

float DeviceCore::electrical_angle() const {
  const float theta_m = static_cast<float>(raw_) * kTwoPi / static_cast<float>(cfg_.counts_per_rev);
  return wrap_two_pi(static_cast<float>(cal_.direction) * static_cast<float>(cfg_.motor.pole_pairs) *
                         theta_m -
                     cal_.zero_electric);
}

float DeviceCore::voltage_limit(float vbus) const {
  const float most = vbus / kSqrt3;
  return cfg_.voltage_limit > 0.0f && cfg_.voltage_limit < most ? cfg_.voltage_limit : most;
}

void DeviceCore::tick() {
  const std::uint32_t now = hal_.micros();
  const float dt = static_cast<float>(static_cast<std::uint32_t>(now - last_us_)) * 1e-6f;
  last_us_ = now;
  const float count_to_rad = kTwoPi / static_cast<float>(cfg_.counts_per_rev);

  // The encoder, and the work the torque held since the last tick did.
  std::uint32_t raw;
  if (!hal_.read_encoder(&raw)) {
    latch(Fault::kEncoder);
  } else {
    const std::int32_t step = turns_.update(raw);
    raw_ = raw;
    const float most = 2.0f * velocity_limit_ * dt / count_to_rad + 16.0f;
    if (std::fabs(static_cast<float>(step)) > most) latch(Fault::kEncoderJump);
    const float dtheta = static_cast<float>(step) * count_to_rad;
    work_ += static_cast<double>(applied_) * static_cast<double>(dtheta);
    vel_.update(dtheta, dt);
  }

  // The host's messages: a new command takes effect in this tick.
  std::uint8_t buf[64];
  for (int k = 0; k < 8; ++k) {
    const std::size_t n = hal_.read_bytes(buf, sizeof buf);
    for (std::size_t i = 0; i < n; ++i) {
      Message m;
      if (reader_.push(buf[i], &m)) handle(m, now);
    }
    if (n < sizeof buf) break;
  }

  const float vbus = hal_.supply_voltage();
  if (cfg_.supply_min > 0.0f && (vbus < cfg_.supply_min || vbus > cfg_.supply_max)) {
    latch(Fault::kSupply);
  }
  const float omega = vel_.value();
  if (enabled_) {
    if (std::fabs(omega) > velocity_limit_) latch(Fault::kOverspeed);
    const std::uint32_t since = now - cmd_us_;
    if (link_timeout_us_ > 0 && since > link_timeout_us_) {
      set_enabled(false);
      log("link timeout: driver off");
    } else if (!cut_ && watchdog_us_ > 0 && since > watchdog_us_) {
      cut_ = true;
      cut_after_us_ = since;
    }
  }

  applied_ = 0.0f;
  saturated_ = false;
  if (cal_phase_ != CalPhase::kNone) {
    run_calibration(now);
  } else if (enabled_) {
    const float host = cut_ ? 0.0f : clampf(cmd_.torque, -torque_limit_, torque_limit_);
    saturated_ = !cut_ && host != cmd_.torque;
    const float damp = -damping_ * omega;
    const float dir = static_cast<float>(cal_.direction);
    float out = 0.0f;
    const float uq = q_voltage(cfg_.motor, dir * (host + damp), dir * omega, voltage_limit(vbus), &out);
    const float total = dir * out;
    if (std::fabs(total - (host + damp)) > 1e-4f * cfg_.max_torque) saturated_ = true;
    // A clamp by the voltage takes from the host's torque first.
    applied_ = total - damp;
    const BridgeVoltages v = space_vector(0.0f, uq, electrical_angle(), vbus);
    hal_.set_duties(v.a / vbus, v.b / vbus, v.c / vbus);
  }

  if (static_cast<std::uint32_t>(now - last_state_us_) >= state_period_us_) {
    last_state_us_ = now;
    send_state(now);
  }
}

void DeviceCore::handle(const Message& m, std::uint32_t now) {
  std::uint8_t frame[kMaxFrame];
  switch (m.type) {
    case MsgType::kHello: {
      Info info;
      info.kind = cfg_.kind;
      info.pole_pairs = static_cast<std::uint8_t>(cfg_.motor.pole_pairs);
      info.flags = cal_.valid ? kInfoCalibrated : 0;
      info.counts_per_rev = cfg_.counts_per_rev;
      info.max_torque = cfg_.max_torque;
      info.inertia = cfg_.inertia;
      info.tick_us = cfg_.tick_us;
      info.firmware = cfg_.firmware;
      std::memcpy(info.name, cfg_.name, sizeof info.name);
      info.name[sizeof info.name - 1] = '\0';
      send(frame, encode(info, frame));
      break;
    }
    case MsgType::kCommand: {
      Command c;
      if (decode(m, &c)) on_command(c, now);
      break;
    }
    case MsgType::kSet: {
      Set s;
      if (decode(m, &s)) on_set(s);
      break;
    }
    case MsgType::kZero: {
      zero_ = turns_.count();
      Ack a;
      a.of = MsgType::kZero;
      send(frame, encode(a, frame));
      break;
    }
    case MsgType::kCalibrate: {
      if (enabled_ || cal_phase_ != CalPhase::kNone || fault_ != Fault::kNone) {
        Ack a;
        a.of = MsgType::kCalibrate;
        a.status = AckStatus::kRejected;
        send(frame, encode(a, frame));
        log(enabled_ ? "calibrate: disable first" : "calibrate: busy or faulted");
      } else {
        start_calibration(now);
      }
      break;
    }
    default:
      break;
  }
}

void DeviceCore::on_command(const Command& c, std::uint32_t now) {
  if (c.flags & kCmdEnable) {
    const bool asked_before = have_cmd_ && (cmd_.flags & kCmdEnable);
    if (fault_ == Fault::kNone && cal_.valid && cal_phase_ == CalPhase::kNone) {
      set_enabled(true);
    } else if (!asked_before) {
      // Once per request, not on every command that repeats it.
      log(!cal_.valid ? "not calibrated: send Calibrate with the shaft free"
          : fault_ != Fault::kNone ? "faulted: send a command without enable to clear"
                                   : "calibrating");
    }
  } else {
    set_enabled(false);
    if (c.flags & kCmdClearFault) fault_ = Fault::kNone;
  }
  cmd_ = c;
  have_cmd_ = true;
  cmd_us_ = now;
  cut_ = false;
  cut_after_us_ = 0;
}

void DeviceCore::on_set(const Set& s) {
  Ack a;
  a.of = MsgType::kSet;
  a.param = s.param;
  const float v = s.value;
  switch (s.param) {
    case Param::kWatchdogUs:
      watchdog_us_ = static_cast<std::uint32_t>(clampf(v, 0.0f, 1e6f));
      a.value = static_cast<float>(watchdog_us_);
      break;
    case Param::kTorqueLimit:
      torque_limit_ = clampf(v, 0.0f, cfg_.max_torque);
      a.value = torque_limit_;
      break;
    case Param::kStatePeriodUs:
      state_period_us_ = static_cast<std::uint32_t>(clampf(v, 100.0f, 1e5f));
      a.value = static_cast<float>(state_period_us_);
      break;
    case Param::kDamping:
      damping_ = clampf(v, 0.0f, 10.0f);
      a.value = damping_;
      break;
    case Param::kVelocityLimit:
      velocity_limit_ = clampf(v, 1.0f, 1e4f);
      a.value = velocity_limit_;
      break;
    case Param::kLinkTimeoutUs:
      link_timeout_us_ = static_cast<std::uint32_t>(clampf(v, 0.0f, 1e7f));
      a.value = static_cast<float>(link_timeout_us_);
      break;
    default:
      a.status = AckStatus::kRejected;
      break;
  }
  std::uint8_t frame[kMaxFrame];
  send(frame, encode(a, frame));
}

void DeviceCore::start_calibration(std::uint32_t now) {
  cal_phase_ = CalPhase::kSettle0;
  cal_start_us_ = now;
  hal_.enable_driver(true);
}

void DeviceCore::run_calibration(std::uint32_t now) {
  const std::uint32_t t = now - cal_start_us_;
  const float count_to_rad = kTwoPi / static_cast<float>(cfg_.counts_per_rev);
  auto record = [&](int i) {
    cal_theta_[i] = static_cast<float>(raw_) * count_to_rad;
    cal_turn_[i] = static_cast<float>(turns_.count()) * count_to_rad;
  };
  auto next = [&](CalPhase p) {
    cal_phase_ = p;
    cal_start_us_ = now;
  };
  float ud = cfg_.align_voltage, theta = 0.0f;
  switch (cal_phase_) {
    case CalPhase::kSettle0:
      ud *= t < kRampUs ? static_cast<float>(t) / static_cast<float>(kRampUs) : 1.0f;
      if (t >= kSettle0Us) {
        record(0);
        next(CalPhase::kSweepUp);
      }
      break;
    case CalPhase::kSweepUp:
      theta = kTwoPi * clampf(static_cast<float>(t) / static_cast<float>(kSweepUs), 0.0f, 1.0f);
      if (t >= kSweepUs) next(CalPhase::kSettle1);
      break;
    case CalPhase::kSettle1:
      theta = kTwoPi;
      if (t >= kSettleUs) {
        record(1);
        next(CalPhase::kSweepDown);
      }
      break;
    case CalPhase::kSweepDown:
      theta = kTwoPi * (1.0f - clampf(static_cast<float>(t) / static_cast<float>(kSweepUs), 0.0f, 1.0f));
      if (t >= kSweepUs) next(CalPhase::kSettle2);
      break;
    case CalPhase::kSettle2:
      if (t >= kSettleUs) {
        record(2);
        cal_phase_ = CalPhase::kNone;
      }
      break;
    case CalPhase::kNone:
      break;
  }
  if (cal_phase_ != CalPhase::kNone) {
    const float vbus = hal_.supply_voltage();
    const float u = ud < voltage_limit(vbus) ? ud : voltage_limit(vbus);
    const BridgeVoltages v = space_vector(u, 0.0f, theta, vbus);
    hal_.set_duties(v.a / vbus, v.b / vbus, v.c / vbus);
    return;
  }

  // Done: judge it.
  hal_.set_duties(0.0f, 0.0f, 0.0f);
  hal_.enable_driver(false);
  const float p = static_cast<float>(cfg_.motor.pole_pairs);
  const float moved = cal_turn_[1] - cal_turn_[0];
  const float back = cal_turn_[2] - cal_turn_[0];
  // One electrical turn moves the shaft 2 pi / p when p is right. The
  // tolerance is SimpleFOC's, 0.5 rad of electrical angle (8 %): room for
  // cogging and friction, too little for a motor with other pole pairs
  // (7 against 6 or 8, 11 against 12).
  const float error = std::fabs(std::fabs(moved) * p - kTwoPi);
  Ack a;
  a.of = MsgType::kCalibrate;
  char text[kMaxLogText + 1];
  if (error > 0.5f || std::fabs(back) * p > 0.25f * kPi) {
    const float seen = std::fabs(moved) > 1e-6f ? kTwoPi / std::fabs(moved) : 0.0f;
    std::snprintf(text, sizeof text, "calibration failed: pole pairs look like %.1f, back %.3f rad",
                  static_cast<double>(seen), static_cast<double>(back));
    log(text);
    a.status = AckStatus::kFailed;
    latch(Fault::kCalibration);
  } else {
    Calibration c;
    c.valid = true;
    c.direction = moved > 0.0f ? 1 : -1;
    const float z0 = static_cast<float>(c.direction) * p * cal_theta_[0];
    const float z2 = static_cast<float>(c.direction) * p * cal_theta_[2];
    c.zero_electric = wrap_two_pi(std::atan2(std::sin(z0) + std::sin(z2), std::cos(z0) + std::cos(z2)));
    cal_ = c;
    hal_.store_calibration(c);
    std::snprintf(text, sizeof text, "calibrated: direction %d, zero %.4f rad, off by %.3f rad",
                  c.direction, static_cast<double>(c.zero_electric), static_cast<double>(error));
    log(text);
    a.value = c.zero_electric;
  }
  std::uint8_t frame[kMaxFrame];
  send(frame, encode(a, frame));
}

void DeviceCore::latch(Fault f) {
  if (fault_ != Fault::kNone) return;
  fault_ = f;
  set_enabled(false);
  if (cal_phase_ != CalPhase::kNone) {
    cal_phase_ = CalPhase::kNone;
    hal_.set_duties(0.0f, 0.0f, 0.0f);
    hal_.enable_driver(false);
  }
  static const char* const kWhat[] = {"none", "encoder", "encoder jump", "overspeed",
                                      "calibration", "supply"};
  const auto i = static_cast<unsigned>(f);
  char text[kMaxLogText + 1];
  std::snprintf(text, sizeof text, "fault: %s; driver off", i < 6 ? kWhat[i] : "?");
  log(text);
}

void DeviceCore::set_enabled(bool on) {
  if (on == enabled_) return;
  enabled_ = on;
  if (!on) hal_.set_duties(0.0f, 0.0f, 0.0f);
  hal_.enable_driver(on);
  applied_ = 0.0f;
}

void DeviceCore::send(const std::uint8_t* frame, std::size_t n) {
  if (n == 0) return;
  if (resync_) {
    const std::uint8_t zero = 0;
    if (hal_.write_bytes(&zero, 1) != 1) {
      dropped_ += static_cast<std::uint32_t>(n);
      return;
    }
    resync_ = false;
  }
  const std::size_t k = hal_.write_bytes(frame, n);
  if (k < n) {
    dropped_ += static_cast<std::uint32_t>(n - k);
    // The host's reader would join this frame's head to the next frame;
    // a delimiter in between makes it lose this one only.
    if (k > 0) resync_ = true;
  }
}

void DeviceCore::log(const char* text) {
  std::uint8_t frame[kMaxFrame];
  send(frame, encode_log(text, frame));
}

void DeviceCore::send_state(std::uint32_t now) {
  st_.seq = state_seq_++;
  st_.time_us = now;
  st_.count = count();
  st_.velocity = vel_.value();
  st_.torque = applied_;
  st_.cmd_seq = cmd_.seq;
  st_.cmd_age_us = have_cmd_ ? saturate16(now - cmd_us_) : 0xFFFF;
  st_.cut_after_us = cut_ ? saturate16(cut_after_us_) : 0;
  std::uint8_t flags = 0;
  if (enabled_) flags |= kStateEnabled;
  if (cut_ && enabled_) flags |= kStateCut;
  if (fault_ != Fault::kNone) flags |= kStateFault;
  if (cal_.valid) flags |= kStateCalibrated;
  if (saturated_) flags |= kStateSaturated;
  if (cal_phase_ != CalPhase::kNone) flags |= kStateCalibrating;
  st_.flags = flags;
  st_.fault = fault_;
  st_.work = work_;
  std::uint8_t frame[kMaxFrame];
  send(frame, encode(st_, frame));
}

}  // namespace tuushin::fw
