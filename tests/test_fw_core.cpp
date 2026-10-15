#include <cmath>
#include <cstring>
#include <functional>
#include <string>
#include <vector>

#include "test.hpp"
#include "tuushin/device/proto.hpp"
#include "tuushin/fw/core.hpp"
#include "tuushin/fw/foc.hpp"
#include "tuushin/fw/plant.hpp"

using namespace tuushin;
using namespace tuushin::device;
using namespace tuushin::fw;

namespace {

constexpr double kTwoPiD = 6.283185307179586;

// The firmware's view of the simulated motor: the same parameters, unless a
// test makes them differ.
DeviceConfig config_for(const PlantParams& p) {
  DeviceConfig c;
  c.motor.pole_pairs = p.pole_pairs;
  c.motor.phase_resistance = static_cast<float>(p.phase_resistance);
  c.motor.torque_constant = static_cast<float>(p.kt);
  c.counts_per_rev = p.counts_per_rev;
  c.max_torque = 0.1f;
  c.inertia = static_cast<float>(p.inertia);
  c.kind = DeviceKind::kSimulated;
  std::strcpy(c.name, "test");
  return c;
}

// A device on a simulated motor and a host end that reads its messages,
// stepped together on one simulated clock.
struct Rig {
  Plant plant;
  SimHal hal;
  DeviceCore core;
  double t = 0.0;
  double tick = 200e-6;
  FrameReader reader;
  std::vector<State> states;
  std::vector<Ack> acks;
  std::vector<std::string> logs;
  Info info;
  bool have_info = false;
  std::uint16_t seq = 0;

  Rig(const PlantParams& p, const DeviceConfig& c, double latency = 0.0)
      : plant(p), hal(plant, latency), core(hal, c) {
    hal.set_time(t);
    core.begin();
  }

  void send_frame(const std::uint8_t* f, std::size_t n) { hal.host_write(f, n); }
  void command(double torque, std::uint8_t flags = kCmdEnable) {
    Command c;
    c.seq = ++seq;
    c.torque = static_cast<float>(torque);
    c.flags = flags;
    std::uint8_t f[kMaxFrame];
    send_frame(f, encode(c, f));
  }
  void set(Param p, double v) {
    Set s;
    s.param = p;
    s.value = static_cast<float>(v);
    std::uint8_t f[kMaxFrame];
    send_frame(f, encode(s, f));
  }
  void bare(MsgType type) {
    std::uint8_t f[kMaxFrame];
    send_frame(f, encode_bare(type, f));
  }
  void pump() {
    std::uint8_t buf[256];
    for (;;) {
      const std::size_t n = hal.host_read(buf, sizeof buf);
      for (std::size_t i = 0; i < n; ++i) {
        Message m;
        if (!reader.push(buf[i], &m)) continue;
        State s;
        Ack a;
        char text[80];
        if (decode(m, &s)) states.push_back(s);
        else if (decode(m, &a)) acks.push_back(a);
        else if (decode(m, &info)) have_info = true;
        else if (decode_log(m, text, sizeof text)) logs.emplace_back(text);
      }
      if (n < sizeof buf) break;
    }
  }
  // Runs for `dur` (s); `each` is called before every tick with the time.
  void run(double dur, const std::function<void(double)>& each = nullptr) {
    const double end = t + dur;
    while (t < end - 1e-12) {
      t += tick;
      hal.set_time(t);
      if (each) each(t);
      core.tick();
      pump();
    }
  }
  // Calibrates with the shaft free; true if the device says it succeeded.
  bool calibrate() {
    bare(MsgType::kCalibrate);
    run(3.5);
    for (const Ack& a : acks) {
      if (a.of == MsgType::kCalibrate) return a.status == AckStatus::kOk;
    }
    return false;
  }
  // Sends a command every millisecond, as a host does.
  std::function<void(double)> host_at_1khz(const std::function<double(double)>& torque) {
    auto next = std::make_shared<double>(t);
    return [this, next, torque](double now) {
      if (now + 1e-12 >= *next) {
        command(torque(now));
        *next += 1e-3;
      }
    };
  }
  bool logged(const char* what) const {
    for (const auto& l : logs) {
      if (l.find(what) != std::string::npos) return true;
    }
    return false;
  }
};

PlantParams quiet_plant() {
  PlantParams p;
  p.cogging = 0.0;
  p.coulomb = 0.0;
  p.encoder_noise = 0.0;
  return p;
}

}  // namespace

TEST_CASE(fw_space_vector_puts_the_vector_on_the_windings) {
  const float vbus = 12.0f;
  for (float theta = 0.0f; theta < 6.3f; theta += 0.37f) {
    for (float uq : {-4.0f, -0.5f, 0.0f, 2.0f, 6.0f}) {
      const float ud = 0.3f * uq;
      const BridgeVoltages v = space_vector(ud, uq, theta, vbus);
      CHECK(v.a >= 0.0f && v.a <= vbus && v.b >= 0.0f && v.b <= vbus && v.c >= 0.0f && v.c <= vbus);
      // Back through Clarke and Park on the neutral-referred voltages.
      const float mean = (v.a + v.b + v.c) / 3.0f;
      const float alpha = v.a - mean, beta = ((v.b - mean) - (v.c - mean)) / kSqrt3;
      const float d = alpha * std::cos(theta) + beta * std::sin(theta);
      const float q = -alpha * std::sin(theta) + beta * std::cos(theta);
      CHECK_NEAR(d, ud, 1e-4);
      CHECK_NEAR(q, uq, 1e-4);
    }
  }
}

TEST_CASE(fw_q_voltage_and_limits) {
  MotorParams m;
  m.pole_pairs = 7;
  m.phase_resistance = 5.0f;
  m.kv = 100.0f;
  // kt from kv: (sqrt(3)/2) * 60 / (2 pi kv).
  CHECK_NEAR(m.kt(), 0.8269933 / 10.0, 1e-6);
  float out = 0.0f;
  const float uq = q_voltage(m, 0.02f, 10.0f, 6.0f, &out);
  CHECK_NEAR(uq, 5.0 * 0.02 / m.kt() + m.kt() / 1.5 * 10.0, 1e-5);
  CHECK_NEAR(out, 0.02, 1e-6);
  // Clamped: the torque that the voltage gives instead.
  const float uq2 = q_voltage(m, 0.5f, 0.0f, 6.0f, &out);
  CHECK_NEAR(uq2, 6.0, 1e-6);
  CHECK_NEAR(out, m.kt() * 6.0 / 5.0, 1e-6);
  MotorParams none;
  none.kv = 0.0f;
  CHECK(q_voltage(none, 0.1f, 0.0f, 6.0f, &out) == 0.0f && out == 0.0f);
}

TEST_CASE(fw_turn_counter_takes_the_short_way) {
  TurnCounter t(4096);
  CHECK(t.update(4000) == 0);
  CHECK(t.update(10) == 106);
  CHECK(t.update(4090) == -16);
  CHECK(t.count() == 90);
  for (int i = 0; i < 3 * 8; ++i) t.update(static_cast<std::uint32_t>((4090 + 512 * (i + 1)) % 4096));
  CHECK(t.count() == 90 + 3 * 4096);
}

TEST_CASE(fw_calibration_finds_how_the_motor_sits) {
  struct Case {
    int dir;
    double offset;
    double start;
  };
  for (const Case& c : {Case{1, 1.0, 0.3}, Case{-1, 4.0, 2.0}, Case{1, 5.9, -1.0}, Case{-1, 0.05, 5.5}}) {
    PlantParams p;  // with cogging, friction and encoder noise
    p.direction = c.dir;
    p.electrical_offset = c.offset;
    p.start_angle = c.start;
    Rig rig(p, config_for(p));
    REQUIRE(rig.calibrate());
    const Calibration& cal = rig.core.calibration();
    CHECK(cal.valid);
    CHECK(cal.direction == c.dir);
    // The zero, electrically: within a few degrees despite cogging.
    const double err = std::remainder(cal.zero_electric - c.offset, kTwoPiD);
    CHECK(std::fabs(err) < 0.1);
    CHECK(rig.hal.stored.valid);
    CHECK(!rig.hal.driver_on());
  }
}

TEST_CASE(fw_calibration_fails_on_wrong_pole_pairs) {
  PlantParams p = quiet_plant();
  p.pole_pairs = 11;
  DeviceConfig c = config_for(p);
  c.motor.pole_pairs = 7;
  Rig rig(p, c);
  CHECK(!rig.calibrate());
  CHECK(rig.core.fault() == Fault::kCalibration);
  CHECK(!rig.core.calibration().valid);
  CHECK(rig.logged("calibration failed"));
  CHECK(!rig.hal.driver_on());
}

TEST_CASE(fw_torque_holds_a_spring) {
  // A hand as a spring at rest: the shaft settles where the spring
  // balances the torque. Exact motor parameters, then a resistance 20 %
  // off, which the torque is then off by.
  for (double r_err : {1.0, 1.2}) {
    PlantParams p = quiet_plant();
    DeviceConfig c = config_for(p);
    c.motor.phase_resistance = static_cast<float>(p.phase_resistance * r_err);
    Rig rig(p, c);
    REQUIRE(rig.calibrate());
    HandParams h;
    h.present = true;
    h.k = 0.1;
    h.b = 5e-3;
    h.ref_offset = rig.plant.angle();
    h.ref_amplitude = 0.0;
    rig.plant.set_hand(h, rig.t);
    const double base = rig.core.count();
    rig.run(1.0, rig.host_at_1khz([](double) { return 0.02; }));
    const double turned = (rig.core.count() - base) * kTwoPiD / p.counts_per_rev;
    // The firmware believes R is r_err times larger: it applies r_err times the torque.
    CHECK_NEAR(turned, 0.02 * r_err / 0.1, 0.004 * r_err);
    CHECK(rig.core.enabled());
    CHECK_NEAR(rig.core.applied_torque(), 0.02, 1e-6);
  }
}

TEST_CASE(fw_reported_work_matches_the_motor) {
  // A hand moving the knob while the host commands a varying torque: the
  // work the device books for the host's torque is the work the motor did
  // on the shaft, up to its velocity estimate's lag in the back-EMF
  // compensation.
  PlantParams p;
  Rig rig(p, config_for(p));
  REQUIRE(rig.calibrate());
  HandParams h;
  h.present = true;
  h.ref_offset = rig.plant.angle();
  h.ref_amplitude = 3.0;
  h.ref_freq = 1.0;
  rig.plant.set_hand(h, rig.t);
  const double w0 = rig.plant.motor_work();
  rig.states.clear();
  rig.run(2.0, rig.host_at_1khz([](double t) { return 0.03 * std::sin(2 * 3.14159 * 3.0 * t); }));
  REQUIRE(!rig.states.empty());
  // Both from the start of the session: the device's books start at 0 and
  // calibration does no booked work.
  const double device = rig.states.back().work;
  const double motor = rig.plant.motor_work() - w0;
  double scale = 0.0;  // the work's own size, for the tolerance
  {
    // Rough magnitude: torque amplitude times the hand's travel.
    scale = 0.03 * 3.0 * 2 * 2;
  }
  CHECK(std::fabs(device - motor) < 0.02 * scale);
}

TEST_CASE(fw_watchdog_cuts_and_restores) {
  PlantParams p = quiet_plant();
  Rig rig(p, config_for(p));
  REQUIRE(rig.calibrate());
  // A hand holds the knob, or the torque would spin it up until the
  // back-EMF uses up the voltage.
  HandParams h;
  h.present = true;
  h.k = 0.1;
  h.b = 5e-3;
  h.ref_offset = rig.plant.angle();
  h.ref_amplitude = 0.0;
  rig.plant.set_hand(h, rig.t);
  double torque = 0.01;
  bool sending = true;
  auto host = rig.host_at_1khz([&](double) { return torque; });
  rig.run(0.1, [&](double t) {
    if (sending) host(t);
  });
  CHECK(rig.core.enabled());
  CHECK_NEAR(rig.core.applied_torque(), 0.01, 1e-6);
  // The host stalls: 2 ms after its last command the torque is cut.
  sending = false;
  rig.states.clear();
  rig.run(0.010);
  CHECK(rig.core.enabled());
  CHECK(rig.core.applied_torque() == 0.0f);
  bool saw_cut = false;
  for (const State& s : rig.states) {
    if (s.flags & kStateCut) {
      saw_cut = true;
      CHECK(s.torque == 0.0f);
      CHECK(s.cut_after_us >= 2000 && s.cut_after_us <= 2000 + 200 + 1);
    }
  }
  CHECK(saw_cut);
  CHECK(std::fabs(rig.plant.motor_torque()) < 1e-4);
  // The next command restores it.
  sending = true;
  rig.states.clear();
  rig.run(0.005, [&](double t) {
    if (sending) host(t);
  });
  CHECK_NEAR(rig.core.applied_torque(), 0.01, 1e-6);
  CHECK(!(rig.states.back().flags & kStateCut));
  // Silence for longer than the link timeout switches the driver off.
  rig.run(0.6);
  CHECK(!rig.core.enabled());
  CHECK(!rig.hal.driver_on());
  CHECK(rig.logged("link timeout"));
}

TEST_CASE(fw_faults_latch_until_cleared) {
  PlantParams p = quiet_plant();
  Rig rig(p, config_for(p));
  REQUIRE(rig.calibrate());
  auto host = rig.host_at_1khz([](double) { return 0.0; });
  rig.run(0.05, host);
  CHECK(rig.core.enabled());
  rig.hal.break_encoder(true);
  rig.run(0.01, host);
  CHECK(rig.core.fault() == Fault::kEncoder);
  CHECK(!rig.core.enabled());
  CHECK(!rig.hal.driver_on());
  CHECK(rig.states.back().flags & kStateFault);
  // Enabling again does nothing while it is latched; the wire is fixed,
  // and a command without enable clears it.
  rig.hal.break_encoder(false);
  rig.run(0.01, host);
  CHECK(!rig.core.enabled());
  rig.command(0.0, kCmdClearFault);
  rig.run(0.002);
  CHECK(rig.core.fault() == Fault::kNone);
  rig.run(0.01, host);
  CHECK(rig.core.enabled());
}

TEST_CASE(fw_overspeed_is_a_fault) {
  PlantParams p = quiet_plant();
  Rig rig(p, config_for(p));
  REQUIRE(rig.calibrate());
  rig.set(Param::kVelocityLimit, 20.0);
  rig.run(0.01);
  // A free knob under a large torque runs away.
  rig.run(0.5, rig.host_at_1khz([](double) { return 0.05; }));
  CHECK(rig.core.fault() == Fault::kOverspeed);
  CHECK(!rig.hal.driver_on());
  CHECK(std::fabs(rig.plant.velocity()) < 40.0);
}

TEST_CASE(fw_torque_limit_and_parameters) {
  PlantParams p = quiet_plant();
  Rig rig(p, config_for(p));
  REQUIRE(rig.calibrate());
  rig.acks.clear();
  rig.set(Param::kTorqueLimit, 0.01);
  rig.set(Param::kWatchdogUs, 5000);
  rig.set(static_cast<Param>(99), 1.0);
  rig.run(0.002);
  REQUIRE(rig.acks.size() == 3);
  CHECK(rig.acks[0].status == AckStatus::kOk && std::fabs(rig.acks[0].value - 0.01f) < 1e-7f);
  CHECK(rig.acks[1].value == 5000.0f);
  CHECK(rig.acks[2].status == AckStatus::kRejected);
  rig.states.clear();
  rig.run(0.02, rig.host_at_1khz([](double) { return -0.05; }));
  CHECK_NEAR(rig.core.applied_torque(), -0.01, 1e-7);
  CHECK(rig.states.back().flags & kStateSaturated);
  // The ceiling cannot be raised.
  rig.acks.clear();
  rig.set(Param::kTorqueLimit, 5.0);
  rig.run(0.002);
  REQUIRE(rig.acks.size() == 1);
  CHECK(std::fabs(rig.acks[0].value - 0.1f) < 1e-7f);
}

TEST_CASE(fw_refuses_to_drive_uncalibrated) {
  PlantParams p = quiet_plant();
  Rig rig(p, config_for(p));
  rig.bare(MsgType::kHello);
  rig.run(0.01, rig.host_at_1khz([](double) { return 0.02; }));
  REQUIRE(rig.have_info);
  CHECK(!(rig.info.flags & kInfoCalibrated));
  CHECK(rig.info.counts_per_rev == p.counts_per_rev);
  CHECK(std::strcmp(rig.info.name, "test") == 0);
  CHECK(!rig.core.enabled());
  CHECK(!rig.hal.driver_on());
  CHECK(rig.logged("not calibrated"));
  // Once, not on every command.
  int n = 0;
  for (const auto& l : rig.logs) n += l.find("not calibrated") != std::string::npos;
  CHECK(n == 1);
}

TEST_CASE(fw_zero_and_state_stream) {
  PlantParams p = quiet_plant();
  Rig rig(p, config_for(p));
  rig.run(0.1);
  // A State every millisecond, numbered.
  CHECK(rig.states.size() >= 99 && rig.states.size() <= 101);
  for (std::size_t i = 1; i < rig.states.size(); ++i) {
    CHECK(static_cast<std::uint16_t>(rig.states[i].seq - rig.states[i - 1].seq) == 1);
    const std::uint32_t gap = rig.states[i].time_us - rig.states[i - 1].time_us;
    CHECK(gap >= 1000 && gap <= 1200);
  }
  rig.plant.set_load(0.0);
  rig.bare(MsgType::kZero);
  rig.run(0.002);
  CHECK(rig.core.count() == 0);
  CHECK(rig.states.back().count == 0);
}
