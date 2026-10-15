#include "tuushin/fw/core.hpp"

#include <algorithm>
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

// How long a rise of the torque under a work cap may take to repay its
// tax (keep_to_cap): a host's command period, over which the host's own
// books trust their prediction of the shaft.
constexpr float kTaxHorizonUs = 1000.0f;
// How many standard deviations of the velocity estimate a rise takes off
// the work it predicts it takes out, and over how long the encoder's
// reading errors are measured (keep_to_cap).
constexpr float kRiseConfidence = 4.0f;
constexpr float kReadingNoiseUs = 100000.0f;

std::uint16_t saturate16(std::uint32_t v) {
  return v > 0xFFFF ? 0xFFFF : static_cast<std::uint16_t>(v);
}

float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

// How far |to| exceeds what of `from` it keeps (all of it in the same
// direction, none across zero): what control::quantization_tax charges.
float growth(float from, float to) {
  const float kept = from * to > 0.0f ? std::fabs(from) : 0.0f;
  const float g = std::fabs(to) - kept;
  return g > 0.0f ? g : 0.0f;
}

// The largest l >= 0 with b l^2 + a l <= r, for b >= 0 and r >= 0, in the
// form that does not cancel; 1 when every l is (b = 0, a <= 0).
float largest_root(float b, float a, float r) {
  const float d = std::sqrt(a * a + 4.0f * b * r);
  if (a <= 0.0f) return b > 0.0f ? (d - a) / (2.0f * b) : 1.0f;
  return 2.0f * r / (a + d);
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
  driving_ = false;
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
  next_state_us_ = last_us_ + state_period_us_;
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
    // The second difference of the readings: of their errors, 6 times
    // their variance, and of the shaft's motion, its acceleration.
    const float d2 = static_cast<float>(step - step_);
    step_ = step;
    reading_var_ += std::min(1.0f, dt * 1e6f / kReadingNoiseUs) * (d2 * d2 / 6.0f - reading_var_);
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

  // The torque the last tick asked for: the work cap's tax is on its
  // changes, as the host's books tax the commands'. (A voltage clamp
  // that comes and goes changes the torque applied besides, which the
  // State's saturated flag tells.)
  const float before = asked_;
  asked_ = 0.0f;
  const bool fresh = fresh_cmd_;
  fresh_cmd_ = false;
  applied_ = 0.0f;
  saturated_ = false;
  if (cal_phase_ != CalPhase::kNone) {
    run_calibration(now);
  } else if (enabled_) {
    float host = cut_ ? 0.0f : clampf(cmd_.torque, -torque_limit_, torque_limit_);
    saturated_ = !cut_ && host != cmd_.torque;
    const bool capped = (cmd_.flags & kCmdWorkCap) != 0;
    // The rise of the torque the host's books taxed: the command's over
    // the one before, in the tick it takes effect.
    const float taxed =
        fresh ? growth(clampf(prev_cmd_, -torque_limit_, torque_limit_), host) : 0.0f;
    if (host != 0.0f && capped) host = keep_to_cap(host, dt, before, taxed, omega);
    asked_ = host;
    if (host == 0.0f && damping_ == 0.0f) {
      // No torque to make: open the windings, which then carry no current
      // at all, instead of driving against an estimate of the back-EMF,
      // whose error (the velocity estimate's lag) would make some.
      drive(false);
    } else {
      drive(true);
      foc(host, omega, vbus);
    }
    if (capped) {
      const float g = growth(before, host) - taxed;
      if (g > 0.0f) tax_ += static_cast<double>(g) * static_cast<double>(count_to_rad);
    }
  }


  // States go out on a lattice of the period, not a period after the last
  // one went: a tick that came a little early would otherwise wait a whole
  // tick, and every gap would stretch by the jitter. A tick within half a
  // tick of its slot takes it; one that finds the next slot already passed
  // (a stall) starts the lattice again from now instead of sending a burst.
  const std::int32_t late = static_cast<std::int32_t>(now - next_state_us_);
  if (late >= -static_cast<std::int32_t>(cfg_.tick_us / 2)) {
    next_state_us_ += state_period_us_;
    if (static_cast<std::int32_t>(next_state_us_ - now) <= 0) {
      next_state_us_ = now + state_period_us_;
    }
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
      info.flags = static_cast<std::uint8_t>((cal_.valid ? kInfoCalibrated : 0) | kInfoWorkCap);
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
  prev_cmd_ = have_cmd_ && (cmd_.flags & kCmdEnable) ? cmd_.torque : 0.0f;
  fresh_cmd_ = true;
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
  drive(true);
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
  drive(false);
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
    drive(false);
  }
  char text[kMaxLogText + 1];
  std::snprintf(text, sizeof text, "fault: %s; driver off", fault_name(f));
  log(text);
}

void DeviceCore::foc(float host, float omega, float vbus) {
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

// The host's books learn of the work a torque did a State and a link's
// latency after it did it, and against a stiff object the shaft can turn
// round within that: a knob leaving a stop at 0.2 m/s under a torque the
// host allowed while the stop pushed it back took 0.9 mJ out of them
// before they knew (README, "Known issues"). The cap is the books' limit
// on the device's side, where the work is measured.
//
// The shaft is taken to turn on at its filtered velocity omega, plus what
// the torque itself adds from the moving parts' inertia J: a fraction l
// of the torque tau held for n ticks of dt then does the work
//
//   w_n(l) = l tau omega n dt + (l tau n dt)^2 / (2 J).
//
// The velocity, not the last step: at a few counts a tick the step is 0
// one tick and 2 the next, and a torque taking energy out of a knob a
// hand turns slowly looked, on the steps, as if it would push a shaft at
// rest, was cut, and came back on the next step: 400 cuts in 0.2 s at
// 1.5 rad/s, which the tax below made cost twice what the torque took out.
//
// Raising the torque costs besides: the work is counted in whole counts,
// and summed by parts the count is off by up to half a count q against
// every change of the torque, which the host's books tax for the changes
// it commands (control::quantization_tax) and the device has to tax for
// its own. Without that, a knob dithering across a count at a spent cap
// books a count's worth of work each time the device lets the torque back
// on for the step that went the torque's way: 0.5 mJ in a second, held by
// a hand, that the motor never did. So the device pays q times the rise
// beyond what the last tick kept and the host taxed.
//
// It then keeps two limits, as the host's predictive PC does
// (control/predictive_pc.hpp, "Repaying the tax"):
//  - the coming tick's work, w_1(l), within the cap's room (none, once it
//    is spent, however far): the torque held may stay while it does no
//    work. w_1 is convex and w_1(0) = 0, so every smaller fraction keeps
//    to it too, as when the voltage limit clamps the torque further;
//  - a rise repays its tax, and the work beyond the cap, from the work it
//    is predicted to take out over up to kTaxHorizonUs: the largest
//    -w_n(l), n within the horizon, which for a shaft the torque would
//    stop sooner is its kinetic energy J omega^2 / 2. Paid within the
//    tick it needs the shaft to move a count a tick against the torque,
//    1.9 rad/s, and a torque cut on a slower one never came back.
//    The prediction takes kRiseConfidence standard deviations of the
//    velocity estimate off: at rest, with an encoder dithering half a
//    count, the estimate strays 0.38 rad/s rms and up to 1.5 rad/s, and
//    each stray against the torque let it rise, pay its tax and be cut
//    again as the readings came back: 270 cuts in 0.2 s. The spread
//    follows from the readings' errors (VelocityFilter::spread), whose
//    variance the device measures from the second differences of its
//    counts, since a real encoder's noise grows with the bridge's.
// The hold is on the estimate itself: a stray cuts a held torque, which
// then stays off, rather than letting it do work.
// The prediction misses what the hand or a wall does to the shaft, so
// the cap can be overrun by about the torque times a few counts.
float DeviceCore::keep_to_cap(float tau, float dt, float before, float taxed, float omega) {
  const float left = static_cast<float>(cmd_.work_cap - (work_ + tax_));
  const float room = left > 0.0f ? left : 0.0f;
  const float q = kTwoPi / static_cast<float>(cfg_.counts_per_rev);
  const float mag = std::fabs(tau);
  const float a = tau * omega * dt;
  const float b = cfg_.inertia > 0.0f ? tau * tau * dt * dt / (2.0f * cfg_.inertia) : 0.0f;
  const float free = (before * tau > 0.0f ? std::fabs(before) : 0.0f) + taxed;
  const float l0 = free / mag;  // where the tax starts
  const float horizon = std::max(1.0f, kTaxHorizonUs * 1e-6f / dt);  // ticks

  // The tick's work.
  const float held = a + b <= room ? 1.0f : clampf(largest_root(b, a, room), 0.0f, 1.0f);
  if (held * mag <= free) {
    if (held < 1.0f) capped_ = true;
    return held * tau;
  }
  // The rise's tax less the most work it takes out within the horizon,
  // on the velocity less kRiseConfidence of its estimate's spread at rest
  // the torque's way. The readings' errors are rounding, at least, and
  // independent from one reading to the next.
  const float reading_sd = std::sqrt(std::max(reading_var_, 1.0f / 12.0f)) * q;
  const float sure = a + mag * kRiseConfidence * vel_.spread(reading_sd, dt) * dt;
  auto due = [&](float l) {
    float n = sure < 0.0f ? horizon : 1.0f;
    if (sure < 0.0f && b > 0.0f) n = clampf(-sure / (2.0f * b * l), 1.0f, horizon);
    return q * (l * mag - free) + (sure * n * l + b * n * n * l * l);
  };
  if (due(held) <= room) {
    if (held < 1.0f) capped_ = true;
    return held * tau;
  }
  capped_ = true;
  // due is falling and then rising over [l0, held]: the work taken out
  // over the whole horizon grows with l faster than the tax, at first,
  // up to where the torque would stop the shaft within it.
  float low = l0;
  if (sure < 0.0f && b > 0.0f) {
    const float fall = -(q * mag + sure * horizon) / (2.0f * b * horizon * horizon);
    low = clampf(fall, l0, std::max(l0, std::min(-sure / (2.0f * b * horizon), held)));
  }
  if (due(low) > room) return l0 * tau;
  float high = held;
  for (int k = 0; k < 20; ++k) {
    const float mid = 0.5f * (low + high);
    if (due(mid) <= room) {
      low = mid;
    } else {
      high = mid;
    }
  }
  return low * tau;
}

void DeviceCore::drive(bool on) {
  if (on == driving_) return;
  driving_ = on;
  if (!on) hal_.set_duties(0.0f, 0.0f, 0.0f);
  hal_.enable_driver(on);
}

void DeviceCore::set_enabled(bool on) {
  if (on == enabled_) return;
  enabled_ = on;
  // On: the next tick decides whether the bridge drives.
  if (!on) drive(false);
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
  if (capped_) flags |= kStateCapped;
  capped_ = false;
  st_.flags = flags;
  st_.fault = fault_;
  st_.work = work_;
  st_.tax = tax_;
  std::uint8_t frame[kMaxFrame];
  send(frame, encode(st_, frame));
}

}  // namespace tuushin::fw
