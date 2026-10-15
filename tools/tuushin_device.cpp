// tuushin-device: a device running tuushin's firmware on its own, without
// a session, for bringing one up: what it reports, its encoder as a hand
// turns the knob, its calibration, a steady torque to weigh against a
// lever, and a spring to feel.
//
// Everything is at the shaft, in the encoder's sense of rotation: radians
// and N m (RemoteDevice at a radius of 1 m, not reversed).

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <thread>

#include "tuushin/device/proto.hpp"
#include "tuushin/device/remote.hpp"
#include "tuushin/net/probe.hpp"
#include "tuushin/rt/thread.hpp"
#include "tuushin/time.hpp"

using namespace tuushin;

namespace {

void usage() {
  std::fprintf(stderr,
               "usage: tuushin-device SPEC [COMMAND] [options]\n"
               "  SPEC as tuushin-teleop's --device: serial:PATH (or PATH) or tcp:HOST:PORT,\n"
               "  then ,option=value... (radius and reverse do not apply: everything here\n"
               "  is at the shaft, in radians and N m, along the encoder's count)\n"
               "commands\n"
               "  info        (the default) what the device reports, and its link; neither\n"
               "              calibrates nor drives\n"
               "  watch       the angle and the speed ten times a second, the driver off:\n"
               "              turn the knob by hand to check the encoder\n"
               "  calibrate   find the motor's electrical zero again (let go of the knob)\n"
               "  hold        a steady torque (--torque), to weigh against a lever on a scale\n"
               "  spring      a spring about the angle at the start, to feel\n"
               "options\n"
               "  --for S         how long watch, hold and spring run (10)\n"
               "  --torque T      hold: the torque in N m\n"
               "  --lever L       hold: the lever's length in m; prints what the scale should read\n"
               "  --k K           spring: N m/rad (what the device's damping and delay allow)\n"
               "  --b B           spring: its own damping in N m s/rad (0)\n");
}

double parse_double(const std::string& s, const char* what) {
  char* end = nullptr;
  const double v = std::strtod(s.c_str(), &end);
  if (end == s.c_str() || *end != '\0') throw std::runtime_error(std::string("bad ") + what + ": " + s);
  return v;
}

struct Options {
  std::string spec;
  std::string command = "info";
  double seconds = 10.0;
  double torque = 0.0;
  bool torque_given = false;
  double lever = 0.0;
  double k = -1.0;
  double b = 0.0;
};

// The damping the knob gets on the device's own ticks unless the spec sets
// one, as tuushin-teleop gives it: some 2 mN m of drag at 2 rad/s.
constexpr double kDeviceDamping = 1e-3;
// The speed past which a knob that hold or spring sends spinning faults
// (rad/s), unless the spec sets one: a free knob under a steady torque
// spins up until something stops it.
constexpr double kDriveVelocityLimit = 30.0;
constexpr double kGravity = 9.80665;
constexpr double kTwoPi = 6.283185307179586;
const Nanos kPeriod = kNanosPerMilli;
const Nanos kPrintEvery = 100 * kNanosPerMilli;

std::atomic<bool> g_stop{false};
void on_signal(int) { g_stop = true; }

void sleep_until(Nanos t) {
  const Nanos d = t - monotonic_now();
  if (d > 0) std::this_thread::sleep_for(std::chrono::nanoseconds(d));
}

std::string flags_of(const device::State& s) {
  std::string f;
  if (s.flags & device::kStateEnabled) f += " driving";
  if (s.flags & device::kStateCalibrating) f += " calibrating";
  if (s.flags & device::kStateCut) f += " cut";
  if (s.flags & device::kStateSaturated) f += " saturated";
  if (!(s.flags & device::kStateCalibrated)) f += " uncalibrated";
  if (s.fault != device::Fault::kNone) f += std::string(" FAULT ") + device::fault_name(s.fault);
  return f;
}

void print_rtt(const device::RemoteDevice& dev) {
  const net::DelayStats s = net::delay_stats(dev.stats().round_trip);
  if (s.count == 0) return;
  std::printf("round trip     min %.3f  p50 %.3f  p99 %.3f  max %.3f ms (command to the State "
              "applying it)\n",
              s.min / 1e6, s.p50 / 1e6, s.p99 / 1e6, s.max / 1e6);
}

void print_link(const device::RemoteDevice& dev, double seconds) {
  const device::RemoteStats s = dev.stats();
  std::printf("%llu States in %.1f s (%llu lost, %llu CRC errors, %llu bad frames), "
              "%llu commands",
              static_cast<unsigned long long>(s.states), seconds,
              static_cast<unsigned long long>(s.states_lost),
              static_cast<unsigned long long>(s.crc_errors),
              static_cast<unsigned long long>(s.bad_frames),
              static_cast<unsigned long long>(s.commands));
  if (s.commands > 0) {
    std::printf(", %llu States saturated, %llu watchdog cuts",
                static_cast<unsigned long long>(s.saturated),
                static_cast<unsigned long long>(s.cuts));
  }
  std::printf("\n");
  print_rtt(dev);
  if (const char* f = dev.fault()) std::printf("%s\n", f);
}

void info(device::RemoteDevice& dev) {
  const device::Info& in = dev.info();
  std::printf("name           %s\n", in.name);
  std::printf("kind           %s\n",
              in.kind == device::DeviceKind::kSimulated ? "a simulated motor" : "a torque motor");
  std::printf("firmware       %u.%u.%u, protocol %u\n", static_cast<unsigned>(in.firmware >> 16),
              static_cast<unsigned>((in.firmware >> 8) & 0xFF),
              static_cast<unsigned>(in.firmware & 0xFF), static_cast<unsigned>(in.protocol));
  std::printf("encoder        %u counts a turn (%.3g mrad a count)\n",
              static_cast<unsigned>(in.counts_per_rev), 1e3 * kTwoPi / in.counts_per_rev);
  std::printf("motor          %u pole pairs, %.4f N m at most (%.4f now)\n",
              static_cast<unsigned>(in.pole_pairs), in.max_torque, dev.torque_limit());
  if (in.inertia > 0.0f) {
    std::printf("inertia        %.4g g mm^2\n", 1e9 * in.inertia);
  } else {
    std::printf("inertia        not given\n");
  }
  std::printf("servo tick     %u us\n", static_cast<unsigned>(in.tick_us));
  std::printf("calibrated     %s\n",
              (dev.latest_state().flags & device::kStateCalibrated) ? "yes" : "no");
  std::printf("round trip     %.3f ms while connecting (the link both ways and the wait for a tick)\n",
              to_millis(dev.connect_round_trip()));
  // A second of States, to see that they keep coming at their period.
  const Nanos start = monotonic_now();
  for (Nanos t = start; !g_stop && t < start + kNanosPerSecond; t += kPrintEvery) {
    sleep_until(t + kPrintEvery);
    dev.read_position(monotonic_now());
  }
  print_link(dev, to_seconds(monotonic_now() - start));
}

void watch(device::RemoteDevice& dev, double seconds) {
  const Nanos start = monotonic_now();
  double lo = 0.0, hi = 0.0;
  std::printf("turn the knob by hand; one turn is %.4f rad\n", kTwoPi);
  for (Nanos t = start; !g_stop && t < start + from_seconds(seconds); t += kPrintEvery) {
    sleep_until(t + kPrintEvery);
    const double a = dev.read_position(monotonic_now());
    lo = std::min(lo, a);
    hi = std::max(hi, a);
    const device::State s = dev.latest_state();
    std::printf("%6.1f s  angle %+9.4f rad (%+8.2f deg, count %+7ld)  speed %+8.3f rad/s%s\n",
                to_seconds(t + kPrintEvery - start), a, a * 360.0 / kTwoPi,
                static_cast<long>(s.count), dev.velocity(), flags_of(s).c_str());
    std::fflush(stdout);
    if (dev.fault()) break;
  }
  std::printf("the angle ranged over %+.4f .. %+.4f rad\n", lo, hi);
  print_link(dev, to_seconds(monotonic_now() - start));
}

// Commands the torque `torque_at(angle, speed)` every period for `seconds`
// and prints the knob ten times a second. Returns the angle at the start
// and at the end.
template <class F>
std::pair<double, double> drive(device::RemoteDevice& dev, double seconds, F torque_at) {
  const Nanos start = monotonic_now();
  const double a0 = dev.read_position(start);
  double a = a0;
  Nanos next_print = start + kPrintEvery;
  for (Nanos t = start; !g_stop && t < start + from_seconds(seconds); t += kPeriod) {
    sleep_until(t + kPeriod);
    const Nanos now = monotonic_now();
    a = dev.read_position(now);
    dev.command_force(torque_at(a, dev.velocity()));
    if (now >= next_print) {
      next_print += kPrintEvery;
      const device::State s = dev.latest_state();
      std::printf("%6.1f s  angle %+9.4f rad  speed %+8.3f rad/s  torque %+.4f N m%s\n",
                  to_seconds(now - start), a, dev.velocity(), s.torque, flags_of(s).c_str());
      std::fflush(stdout);
    }
    if (const char* f = dev.fault()) {
      std::printf("stopped: %s\n", f);
      break;
    }
  }
  dev.disable();
  return {a0, a};
}

void hold(device::RemoteDevice& dev, const Options& o) {
  if (std::fabs(o.torque) > dev.torque_limit() + 1e-9) {
    throw std::runtime_error("--torque is over the device's limit of " +
                             std::to_string(dev.torque_limit()) + " N m");
  }
  if (o.lever > 0.0) {
    const double f = std::fabs(o.torque) / o.lever;
    std::printf("on a lever of %.3f m the scale should read %.3f N, %.1f g, if the firmware's "
                "torque constant and resistance are right\n",
                o.lever, f, 1e3 * f / kGravity);
  }
  drive(dev, o.seconds, [&](double, double) { return o.torque; });
  print_link(dev, o.seconds);
  if (o.lever > 0.0) {
    // Held still there is no back-EMF: the firmware puts R T / kt across the
    // winding and the torque comes out times (kt/R) true over (kt/R) set.
    std::printf("held still, the reading is that times the true kt/R over the one set: set the "
                "resistance to half a meter's phase-to-phase reading, then the torque constant "
                "to itself times reading / %.3f N\n",
                std::fabs(o.torque) / o.lever);
  }
}

void spring(device::RemoteDevice& dev, const Options& o) {
  // As tuushin-teleop's slave picks its coupling: a sampled spring K is
  // passive on a device with damping b when b > K T / 2 + |B| (Colgate and
  // Schenkel), T the period; the torque also waits for the link and the
  // State's age, counted with the half period, and half that stiffness
  // leaves a margin.
  const double horizon = to_seconds(kPeriod / 2 + dev.connect_round_trip() + dev.state_period() / 2);
  const double allowed = dev.damping() > 0.0 ? 0.5 * dev.damping() / horizon : 0.0;
  double k = o.k;
  if (k < 0.0) {
    if (allowed <= 0.0) throw std::runtime_error("a device without damping needs --k");
    k = allowed;
  }
  std::printf("spring %.4g N m/rad and %.3g N m s/rad (the device damps %.3g N m s/rad, the loop's "
              "delay %.2f ms: %.4g N m/rad is what that allows)\n",
              k, o.b, dev.damping(), 1e3 * horizon, allowed);
  if (allowed > 0.0 && k > allowed) {
    std::printf("over what the damping allows: the knob may buzz or swing up; Ctrl-C stops it\n");
  }
  const double w0 = dev.actuator_work();
  double centre = 0.0;
  bool first = true;
  const auto ends = drive(dev, o.seconds, [&](double a, double v) {
    if (first) {
      centre = a;
      first = false;
    }
    return -k * (a - centre) - o.b * v;
  });
  print_link(dev, o.seconds);
  // A spring gives back what it took: the work its torque did on the knob
  // is minus the change of its energy, less what its damping took. More
  // than that is energy the loop's delay made.
  const double work = dev.actuator_work() - w0;
  const double ideal = -0.5 * k * ((ends.second - centre) * (ends.second - centre));
  std::printf("the spring's work on the knob %.6f J; an ideal spring's %.6f J (to the angle at the "
              "end); %.6f J more %s\n",
              work, ideal, work - ideal,
              work - ideal > 0.0 ? "came from the loop's delay" : "went into damping and friction");
}

}  // namespace

int main(int argc, char** argv) {
  Options o;
  try {
    bool command_given = false;
    int i = 1;
    auto next = [&]() -> std::string {
      if (i + 1 >= argc) throw std::runtime_error(std::string("missing value after ") + argv[i]);
      return argv[++i];
    };
    for (; i < argc; ++i) {
      const std::string a = argv[i];
      if (a == "-h" || a == "--help") {
        usage();
        return 0;
      } else if (a == "--for") {
        o.seconds = parse_double(next(), "--for");
      } else if (a == "--torque") {
        o.torque = parse_double(next(), "--torque");
        o.torque_given = true;
      } else if (a == "--lever") {
        o.lever = parse_double(next(), "--lever");
      } else if (a == "--k") {
        o.k = parse_double(next(), "--k");
      } else if (a == "--b") {
        o.b = parse_double(next(), "--b");
      } else if (!a.empty() && a[0] == '-') {
        throw std::runtime_error("unknown option " + a);
      } else if (o.spec.empty()) {
        o.spec = a;
      } else if (!command_given) {
        o.command = a;
        command_given = true;
      } else {
        throw std::runtime_error("one command at a time: " + a);
      }
    }
    if (o.spec.empty()) {
      usage();
      return 2;
    }
    const std::string& c = o.command;
    if (c != "info" && c != "watch" && c != "calibrate" && c != "hold" && c != "spring") {
      throw std::runtime_error("unknown command " + c);
    }
    if (c == "hold" && !o.torque_given) throw std::runtime_error("hold needs --torque");
    if (o.seconds <= 0.0) throw std::runtime_error("--for must be positive");

    device::DeviceSpec spec = device::parse_device_spec(o.spec);
    device::RemoteConfig rc = spec.config;
    rc.radius = 1.0;
    rc.reverse = false;
    const bool drives = c == "hold" || c == "spring";
    if (!spec.calibrate_given) {
      rc.calibrate = c == "calibrate" ? device::Calibrate::kAlways
                     : drives         ? device::Calibrate::kIfNeeded
                                      : device::Calibrate::kSkip;
    }
    if (drives) {
      if (!spec.damping_given) rc.damping = kDeviceDamping;
      if (rc.velocity_limit <= 0.0) rc.velocity_limit = kDriveVelocityLimit;
    }
    rc.reader_thread = true;
    rc.on_log = [](const std::string& line) { std::fprintf(stderr, "device: %s\n", line.c_str()); };

    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);
    device::RemoteDevice dev(device::open_stream(spec), rc);
    dev.connect();
    if (drives) rt::prefer_low_latency(kPeriod);
    if (c == "info") info(dev);
    else if (c == "watch") watch(dev, o.seconds);
    else if (c == "calibrate") std::printf("calibrated\n");
    else if (c == "hold") hold(dev, o);
    else spring(dev, o);
    return dev.fault() ? 1 : 0;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "tuushin-device: %s\n", e.what());
    return 1;
  }
}
