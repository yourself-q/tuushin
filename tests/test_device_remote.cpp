#include <cmath>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "test.hpp"
#include "tuushin/device/remote.hpp"
#include "tuushin/fw/core.hpp"
#include "tuushin/fw/plant.hpp"

using namespace tuushin;
using namespace tuushin::device;
using namespace tuushin::fw;

namespace {

// The firmware on a simulated motor, as the byte stream a host opens. Time
// is simulated: it passes when the host waits on the stream, or when the
// test advances it, and the host reads it through RemoteConfig::clock.
class SimLink final : public ByteStream {
 public:
  SimLink(const PlantParams& p, const DeviceConfig& c, double latency)
      : plant_(p), hal_(plant_, latency), core_(hal_, c) {
    hal_.set_time(t_);
    core_.begin();
  }

  long read(std::uint8_t* buf, std::size_t cap) override {
    return static_cast<long>(hal_.host_read(buf, cap));
  }
  long write(const std::uint8_t* buf, std::size_t n) override {
    hal_.host_write(buf, n);
    return static_cast<long>(n);
  }
  bool wait_readable(Nanos timeout) override {
    const double end = t_ + static_cast<double>(timeout) * 1e-9;
    while (!hal_.host_readable() && t_ < end - 1e-12) step();
    return hal_.host_readable();
  }
  std::string name() const override { return "sim"; }

  Nanos clock() const { return static_cast<Nanos>(std::llround(t_ * 1e9)); }
  // Runs to `until` (s), letting `poll` read after each tick as a reader
  // thread would.
  void advance_to(double until, RemoteDevice* poll = nullptr) {
    while (t_ < until - 1e-12) {
      step();
      if (poll) poll->poll();
    }
  }
  double time() const { return t_; }
  // The device stops: no ticks, nothing sent.
  void freeze() { frozen_ = true; }

  Plant& plant() { return plant_; }
  SimHal& hal() { return hal_; }
  DeviceCore& core() { return core_; }

 private:
  void step() {
    t_ += 200e-6;
    hal_.set_time(t_);
    if (!frozen_) core_.tick();
  }

  Plant plant_;
  SimHal hal_;
  DeviceCore core_;
  double t_ = 0.0;
  bool frozen_ = false;
};

DeviceConfig sim_config(const PlantParams& p) {
  DeviceConfig c;
  c.motor.pole_pairs = p.pole_pairs;
  c.motor.phase_resistance = static_cast<float>(p.phase_resistance);
  c.motor.torque_constant = static_cast<float>(p.kt);
  c.counts_per_rev = p.counts_per_rev;
  c.max_torque = 0.1f;
  c.inertia = static_cast<float>(p.inertia);
  c.kind = DeviceKind::kSimulated;
  std::strcpy(c.name, "sim/test");
  return c;
}

PlantParams quiet() {
  PlantParams p;
  p.cogging = 0.0;
  p.coulomb = 0.0;
  p.encoder_noise = 0.0;
  return p;
}

// A device on a link with `latency` each way, and a host's view of it.
struct Bench {
  SimLink* link;
  std::vector<std::string> logs;
  std::unique_ptr<RemoteDevice> dev;

  explicit Bench(const PlantParams& p = quiet(), double latency = 0.5e-3,
                 RemoteConfig cfg = RemoteConfig{}) {
    auto owned = std::make_unique<SimLink>(p, sim_config(p), latency);
    link = owned.get();
    cfg.clock = [l = link] { return l->clock(); };
    cfg.on_log = [this](const std::string& s) { logs.push_back(s); };
    dev = std::make_unique<RemoteDevice>(std::move(owned), cfg);
  }
  // A hand that holds the knob where it is with a spring.
  void hold(double k = 0.1) {
    HandParams h;
    h.present = true;
    h.k = k;
    h.b = 5e-3;
    h.ref_offset = link->plant().angle();
    h.ref_amplitude = 0.0;
    link->plant().set_hand(h, link->time());
  }
  // A control loop at 1 kHz for `dur` seconds, commanding `force` (N).
  void run(double dur, double force) {
    const double end = link->time() + dur;
    while (link->time() < end - 1e-9) {
      link->advance_to(link->time() + 1e-3, dev.get());
      dev->read_position(link->clock());
      dev->command_force(force);
    }
  }
  bool logged(const char* what) const {
    for (const auto& l : logs) {
      if (l.find(what) != std::string::npos) return true;
    }
    return false;
  }
};

}  // namespace

TEST_CASE(remote_connects_calibrates_and_zeroes) {
  Bench b;
  CHECK(!b.link->core().calibration().valid);
  b.dev->connect();
  CHECK(b.link->core().calibration().valid);
  CHECK(b.link->hal().stored.valid);
  CHECK(b.logged("calibrating"));
  CHECK(b.logged("sim/test"));
  CHECK(std::strcmp(b.dev->info().name, "sim/test") == 0);
  CHECK(b.dev->info().kind == DeviceKind::kSimulated);
  // At the rim of a 20 mm knob.
  CHECK_NEAR(b.dev->model_mass(), 7e-6 / 4e-4, 1e-9);
  CHECK_NEAR(b.dev->encoder_resolution(), 0.02 * 6.283185307179586 / 16384, 1e-12);
  CHECK_NEAR(b.dev->max_force(), 5.0, 1e-5);
  CHECK(b.dev->read_position(b.link->clock()) == 0.0);
  CHECK(b.dev->fault() == nullptr);
  // Parameters as set: the watchdog of 2 ms, a State a millisecond.
  b.link->advance_to(b.link->time() + 0.05, b.dev.get());
  CHECK(b.dev->stats().states >= 48 && b.dev->stats().states <= 52);
  CHECK(!b.link->hal().driver_on());
}

TEST_CASE(remote_needs_calibration_or_permission) {
  RemoteConfig cfg;
  cfg.calibrate = Calibrate::kNever;
  Bench b(quiet(), 0.5e-3, cfg);
  bool threw = false;
  try {
    b.dev->connect();
  } catch (const std::runtime_error& e) {
    threw = std::string(e.what()).find("not calibrated") != std::string::npos;
  }
  CHECK(threw);
}

TEST_CASE(remote_force_turns_the_knob_against_a_hand) {
  for (bool reverse : {false, true}) {
    RemoteConfig cfg;
    cfg.reverse = reverse;
    Bench b(quiet(), 0.5e-3, cfg);
    b.dev->connect();
    b.hold(0.1);
    const double at = b.link->plant().angle();
    b.run(1.0, 1.0);
    // 1 N at 20 mm is 0.02 N m; the hand's 0.1 N m/rad gives 0.2 rad,
    // 4 mm at the rim, along +x whichever way x runs.
    const double x = b.dev->read_position(b.link->clock());
    CHECK_NEAR(x, 0.004, 1e-4);
    CHECK(std::fabs(b.dev->velocity()) < 1e-3);
    CHECK(b.link->hal().driver_on());
    CHECK_NEAR(b.link->plant().angle() - at, reverse ? -0.2 : 0.2, 0.005);
    // The device's books: the held force over the displacement.
    CHECK_NEAR(b.dev->actuator_work(), 1.0 * 0.004, 2e-4);
    CHECK(b.dev->stats().states_lost == 0 && b.dev->stats().crc_errors == 0);
  }
}

TEST_CASE(remote_measures_the_round_trip) {
  Bench b(quiet(), 0.5e-3);
  b.dev->connect();
  // Known from the parameters' acknowledgements before any command.
  CHECK(b.dev->connect_round_trip() >= 1.0e6 - 1e3);
  CHECK(b.dev->connect_round_trip() <= 1.4e6 + 1e3);
  b.hold();
  b.run(0.5, 0.2);
  const RemoteStats s = b.dev->stats();
  REQUIRE(s.round_trip.size() >= 400);
  double lo = 1e18, hi = 0.0;
  for (double r : s.round_trip) {
    lo = std::min(lo, r);
    hi = std::max(hi, r);
  }
  // 0.5 ms each way, and here the device's tick and the host's polling
  // both on 200 us ticks: 1.2 ms.
  CHECK(lo >= 1.0e6 - 1e3);
  CHECK(hi <= 1.4e6 + 1e3);
  CHECK(b.dev->least_round_trip() == static_cast<Nanos>(lo));
  CHECK(s.cuts == 0 && s.cuts_between == 0);
}

TEST_CASE(remote_reports_the_watchdog_cut) {
  Bench b(quiet(), 0.5e-3);
  b.dev->connect();
  b.hold();
  b.run(0.1, 1.0);
  CHECK(b.dev->force_cut_after() == 0);
  // The host stalls for 6 ms after its last command; the device cuts the
  // force 2 ms after it took effect, which was 0.5 ms (and up to a tick)
  // after it was sent.
  b.link->advance_to(b.link->time() + 6e-3, b.dev.get());
  b.dev->read_position(b.link->clock());
  const Nanos cut = b.dev->force_cut_after();
  CHECK(cut >= from_millis(2.45) && cut <= from_millis(2.95));
  CHECK(b.dev->stats().cuts == 1);
  // The windings are open: nothing at all.
  CHECK(b.link->plant().motor_torque() == 0.0);
  // The next command restores it, and the next read knows.
  b.dev->command_force(1.0);
  b.link->advance_to(b.link->time() + 2e-3, b.dev.get());
  b.dev->read_position(b.link->clock());
  CHECK(b.dev->force_cut_after() == 0);
  CHECK(b.dev->fault() == nullptr);
}

TEST_CASE(remote_reports_faults_and_a_silent_device) {
  {
    Bench b;
    b.dev->connect();
    b.hold();
    b.run(0.05, 0.5);
    CHECK(b.dev->fault() == nullptr);
    b.link->hal().break_encoder(true);
    b.run(0.01, 0.5);
    const char* f = b.dev->fault();
    REQUIRE(f != nullptr);
    CHECK(std::string(f) == "device fault: encoder");
    CHECK(!b.link->hal().driver_on());
    CHECK(b.logged("firmware: fault: encoder"));
  }
  {
    Bench b;
    b.dev->connect();
    b.run(0.05, 0.0);
    b.link->freeze();
    b.run(0.09, 0.0);
    CHECK(b.dev->fault() == nullptr);
    b.run(0.02, 0.0);
    const char* f = b.dev->fault();
    REQUIRE(f != nullptr);
    CHECK(std::string(f).find("no State") != std::string::npos);
  }
}

TEST_CASE(remote_clears_a_fault_left_from_before) {
  Bench c;
  c.dev->connect();
  c.hold();
  c.run(0.02, 0.5);
  c.link->hal().break_encoder(true);
  c.run(0.01, 0.5);
  c.link->hal().break_encoder(false);
  REQUIRE(c.link->core().fault() == Fault::kEncoder);
  // A new host on the same device: the fault is cleared, and the device,
  // still calibrated, is not calibrated again.
  struct Borrowed final : ByteStream {
    explicit Borrowed(ByteStream& s) : s_(s) {}
    long read(std::uint8_t* buf, std::size_t cap) override { return s_.read(buf, cap); }
    long write(const std::uint8_t* buf, std::size_t n) override { return s_.write(buf, n); }
    bool wait_readable(Nanos t) override { return s_.wait_readable(t); }
    std::string name() const override { return s_.name(); }
    ByteStream& s_;
  };
  RemoteConfig cfg;
  SimLink* l = c.link;
  cfg.clock = [l] { return l->clock(); };
  std::vector<std::string> logs;
  cfg.on_log = [&](const std::string& s) { logs.push_back(s); };
  RemoteDevice again(std::make_unique<Borrowed>(*c.link), cfg);
  again.connect();
  CHECK(c.link->core().fault() == Fault::kNone);
  bool cleared = false, recalibrated = false;
  for (const auto& s : logs) {
    cleared = cleared || s.find("clearing a fault") != std::string::npos;
    recalibrated = recalibrated || s.find("calibrating") != std::string::npos;
  }
  CHECK(cleared);
  CHECK(!recalibrated);
  CHECK(again.fault() == nullptr);
}

TEST_CASE(remote_device_spec) {
  const DeviceSpec a = parse_device_spec("serial:/dev/cu.usbmodem1101,baud=115200,radius=0.03,reverse");
  CHECK(a.transport == DeviceSpec::Transport::kSerial);
  CHECK(a.path == "/dev/cu.usbmodem1101" && a.baud == 115200);
  CHECK(a.config.radius == 0.03 && a.config.reverse);
  const DeviceSpec b = parse_device_spec("/dev/ttyACM0");
  CHECK(b.path == "/dev/ttyACM0" && b.baud == 921600 && b.config.zero);
  const DeviceSpec c =
      parse_device_spec("tcp:[::1]:5555,nozero,calibrate=always,torque-limit=0.05,state-us=500");
  CHECK(c.transport == DeviceSpec::Transport::kTcp && c.host == "::1" && c.port == 5555);
  CHECK(!c.config.zero && c.config.calibrate == Calibrate::kAlways);
  CHECK(c.config.torque_limit == 0.05 && c.config.state_period == 500 * 1000);
  const DeviceSpec d = parse_device_spec("tcp:localhost:4000");
  CHECK(d.host == "localhost" && d.port == 4000);
  for (const char* bad : {"tcp:localhost", "serial:", "/dev/x,radius=-1", "/dev/x,what=1",
                          "/dev/x,calibrate=maybe", "tcp:h:99999", "/dev/x,baud=fast"}) {
    bool threw = false;
    try {
      parse_device_spec(bad);
    } catch (const std::runtime_error&) {
      threw = true;
    }
    CHECK(threw);
  }
}
