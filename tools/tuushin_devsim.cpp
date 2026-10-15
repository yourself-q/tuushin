// tuushin-devsim: the firmware's core (firmware/core) driving a simulated
// gimbal motor in real time, on a pseudo-terminal or a TCP port: a device
// for tuushin-teleop --device before there is hardware, running the same
// code the board runs. The motor is the one in fw/plant.hpp: field-oriented
// control through its true electrical angle, cogging, friction, encoder
// noise, and the calibration has to find how it sits under the encoder.
//
//   tuushin-devsim --pty                    prints the terminal's path
//   tuushin-devsim --tcp 47300              then --device tcp:127.0.0.1:47300
//   tuushin-devsim --tcp 47300 --hand       a hand turning the knob (a master)
//   tuushin-devsim --tcp 47301 --stop 0.5   an object 0.5 rad on (a slave)

#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <stdexcept>
#include <string>

#include "tuushin/fw/core.hpp"
#include "tuushin/fw/plant.hpp"
#include "tuushin/rt/thread.hpp"
#include "tuushin/time.hpp"

using namespace tuushin;

namespace {

std::atomic<bool> g_stop{false};

void on_signal(int) { g_stop.store(true); }

void usage() {
  std::fprintf(stderr,
               "usage: tuushin-devsim (--pty | --tcp PORT) [options]\n"
               "  --pty                 serve on a new pseudo-terminal; its path is the first\n"
               "                        line on stdout\n"
               "  --path-file FILE      and write the path to FILE\n"
               "  --tcp PORT            serve one host at a time on a TCP port\n"
               "  --bind HOST           the TCP port's address (127.0.0.1)\n"
               "  --latency MS          added to the link each way (0.5)\n"
               "  --hand                a hand holds the knob and turns it to and fro\n"
               "  --hand-amplitude RAD  how far (2)\n"
               "  --hand-freq HZ        how often (0.5)\n"
               "  --hand-k K            the hand's grip in N m/rad (0.12)\n"
               "  --stop RAD            an object the knob runs into RAD on from where it\n"
               "                        is once calibrated\n"
               "  --stop-k K            its stiffness in N m/rad (2)\n"
               "  --calibrated          start with the calibration stored, as a board that\n"
               "                        has been calibrated before\n"
               "  --direction D         how the motor sits under the encoder: +1 or -1 (1)\n"
               "  --offset RAD          its electrical offset (1.0)\n"
               "  --counts N            encoder counts a turn (16384)\n"
               "  --noise COUNTS        encoder noise, standard deviation (0.5)\n"
               "  --seed N              of the encoder noise (1)\n"
               "  --duration S          exit after S seconds\n"
               "  --quiet               no status line every second\n");
}

double parse_double(const std::string& s, const char* what) {
  char* end = nullptr;
  const double v = std::strtod(s.c_str(), &end);
  if (end == s.c_str() || *end != '\0') throw std::runtime_error(std::string("bad ") + what + ": " + s);
  return v;
}

[[noreturn]] void throw_errno(const std::string& what) {
  throw std::runtime_error(what + ": " + std::strerror(errno));
}

void set_nonblocking(int fd) { ::fcntl(fd, F_SETFL, ::fcntl(fd, F_GETFL) | O_NONBLOCK); }

void make_raw(int fd) {
  termios t;
  if (::tcgetattr(fd, &t) != 0) return;
  ::cfmakeraw(&t);
  ::tcsetattr(fd, TCSANOW, &t);
}

// The device's end of the host link: bytes in, bytes out, and whether a
// host is there. Bytes the host is not there to take are dropped, as a
// board's USB stack drops them.
class Port {
 public:
  virtual ~Port() = default;
  // Reads what the host sent; -1 when nothing is connected.
  virtual long read(std::uint8_t* buf, std::size_t cap) = 0;
  virtual void write(const std::uint8_t* buf, std::size_t n) = 0;
  virtual void service() {}
  virtual bool connected() const = 0;
};

// A pseudo-terminal. This end holds the terminal open while no host has
// it, so that it stays raw and reads do not fail; once the host speaks it
// lets go, so that the host's close is the terminal's last, which hangs it
// up (that is how the host's leaving shows) and resets what the host set
// on it, its exclusive use among them.
class PtyPort final : public Port {
 public:
  PtyPort() {
    master_ = ::posix_openpt(O_RDWR | O_NOCTTY);
    if (master_ < 0) throw_errno("posix_openpt");
    if (::grantpt(master_) != 0 || ::unlockpt(master_) != 0) throw_errno("grantpt");
    const char* name = ::ptsname(master_);
    if (!name) throw_errno("ptsname");
    path_ = name;
    set_nonblocking(master_);
    ::fcntl(master_, F_SETFD, FD_CLOEXEC);
    hold();
  }
  ~PtyPort() override {
    if (held_ >= 0) ::close(held_);
    ::close(master_);
  }
  const std::string& path() const { return path_; }

  long read(std::uint8_t* buf, std::size_t cap) override {
    const ssize_t n = ::read(master_, buf, cap);
    if (n > 0) {
      if (held_ >= 0) {
        ::close(held_);
        held_ = -1;
        connected_ = true;
        std::fprintf(stderr, "a host opened %s\n", path_.c_str());
      }
      return static_cast<long>(n);
    }
    if (n < 0 && (errno == EAGAIN || errno == EINTR)) return connected_ ? 0 : -1;
    // Hung up: the host closed its end.
    if (held_ < 0) {
      std::fprintf(stderr, "the host closed %s\n", path_.c_str());
      connected_ = false;
      hold();
    }
    return -1;
  }
  void write(const std::uint8_t* buf, std::size_t n) override {
    // What the terminal cannot take is dropped.
    const ssize_t k = ::write(master_, buf, n);
    (void)k;
  }
  bool connected() const override { return connected_; }

 private:
  void hold() {
    held_ = ::open(path_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (held_ < 0) throw_errno("open " + path_);
    make_raw(held_);
    ::tcflush(held_, TCIOFLUSH);
  }

  int master_ = -1;
  int held_ = -1;
  bool connected_ = false;
  std::string path_;
};

class TcpPort final : public Port {
 public:
  TcpPort(const std::string& host, std::uint16_t port) {
    addrinfo hints{};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_flags = AI_PASSIVE;
    addrinfo* res = nullptr;
    const std::string service = std::to_string(port);
    if (const int rc = ::getaddrinfo(host.c_str(), service.c_str(), &hints, &res); rc != 0) {
      throw std::runtime_error("cannot resolve " + host + ": " + ::gai_strerror(rc));
    }
    listener_ = ::socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (listener_ < 0) {
      ::freeaddrinfo(res);
      throw_errno("socket");
    }
    const int one = 1;
    ::setsockopt(listener_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    if (::bind(listener_, res->ai_addr, res->ai_addrlen) != 0 || ::listen(listener_, 1) != 0) {
      ::freeaddrinfo(res);
      throw_errno("listen on " + host + ":" + service);
    }
    ::freeaddrinfo(res);
    set_nonblocking(listener_);
    ::fcntl(listener_, F_SETFD, FD_CLOEXEC);
  }
  ~TcpPort() override {
    if (conn_ >= 0) ::close(conn_);
    ::close(listener_);
  }

  void service() override {
    const int fd = ::accept(listener_, nullptr, nullptr);
    if (fd < 0) return;
    if (conn_ >= 0) {
      // One controller per device: a second would interleave its commands.
      ::close(fd);
      std::fprintf(stderr, "refused a second host\n");
      return;
    }
    conn_ = fd;
    set_nonblocking(conn_);
    ::fcntl(conn_, F_SETFD, FD_CLOEXEC);
    const int one = 1;
    ::setsockopt(conn_, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
#ifdef SO_NOSIGPIPE
    ::setsockopt(conn_, SOL_SOCKET, SO_NOSIGPIPE, &one, sizeof one);
#endif
    std::fprintf(stderr, "a host connected\n");
  }
  long read(std::uint8_t* buf, std::size_t cap) override {
    if (conn_ < 0) return -1;
    const ssize_t n = ::recv(conn_, buf, cap, 0);
    if (n > 0) return static_cast<long>(n);
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) return 0;
    drop();
    return -1;
  }
  void write(const std::uint8_t* buf, std::size_t n) override {
    if (conn_ < 0) return;
#ifdef MSG_NOSIGNAL
    const ssize_t k = ::send(conn_, buf, n, MSG_NOSIGNAL);
#else
    const ssize_t k = ::send(conn_, buf, n, 0);
#endif
    if (k < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) drop();
  }
  bool connected() const override { return conn_ >= 0; }

 private:
  void drop() {
    ::close(conn_);
    conn_ = -1;
    std::fprintf(stderr, "the host disconnected\n");
  }

  int listener_ = -1;
  int conn_ = -1;
};

void sleep_until(Nanos t) {
  const Nanos d = t - monotonic_now();
  if (d <= 0) return;
  timespec ts{static_cast<time_t>(d / kNanosPerSecond), static_cast<long>(d % kNanosPerSecond)};
  ::nanosleep(&ts, nullptr);
}

}  // namespace

int main(int argc, char** argv) {
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);
  std::signal(SIGPIPE, SIG_IGN);
  bool pty = false, quiet = false, calibrated = false;
  int tcp_port = 0;
  std::string bind = "127.0.0.1", path_file;
  double latency_ms = 0.5, duration = 0.0;
  std::uint64_t seed = 1;
  fw::PlantParams pp;
  fw::HandParams hand;
  fw::StopParams stop;
  try {
    for (int i = 1; i < argc; ++i) {
      const std::string a = argv[i];
      auto next = [&]() -> std::string {
        if (i + 1 >= argc) throw std::runtime_error("missing value for " + a);
        return argv[++i];
      };
      if (a == "--pty") pty = true;
      else if (a == "--path-file") path_file = next();
      else if (a == "--tcp") tcp_port = static_cast<int>(parse_double(next(), "port"));
      else if (a == "--bind") bind = next();
      else if (a == "--latency") latency_ms = parse_double(next(), "latency");
      else if (a == "--hand") hand.present = true;
      else if (a == "--hand-amplitude") hand.ref_amplitude = parse_double(next(), "hand-amplitude");
      else if (a == "--hand-freq") hand.ref_freq = parse_double(next(), "hand-freq");
      else if (a == "--hand-k") hand.k = parse_double(next(), "hand-k");
      else if (a == "--stop") {
        stop.present = true;
        stop.angle = parse_double(next(), "stop");
      }
      else if (a == "--stop-k") stop.k = parse_double(next(), "stop-k");
      else if (a == "--calibrated") calibrated = true;
      else if (a == "--direction") pp.direction = parse_double(next(), "direction") < 0 ? -1 : 1;
      else if (a == "--offset") pp.electrical_offset = parse_double(next(), "offset");
      else if (a == "--counts") pp.counts_per_rev = static_cast<std::uint32_t>(parse_double(next(), "counts"));
      else if (a == "--noise") pp.encoder_noise = parse_double(next(), "noise");
      else if (a == "--seed") seed = static_cast<std::uint64_t>(parse_double(next(), "seed"));
      else if (a == "--duration") duration = parse_double(next(), "duration");
      else if (a == "--quiet") quiet = true;
      else if (a == "-h" || a == "--help") { usage(); return 0; }
      else throw std::runtime_error("unknown option " + a);
    }
    if (pty == (tcp_port != 0)) throw std::runtime_error("give one of --pty and --tcp PORT");
    if (tcp_port < 0 || tcp_port > 65535) throw std::runtime_error("bad port");

    std::unique_ptr<Port> port;
    if (pty) {
      auto p = std::make_unique<PtyPort>();
      std::printf("%s\n", p->path().c_str());
      std::fflush(stdout);
      if (!path_file.empty()) {
        FILE* f = std::fopen(path_file.c_str(), "w");
        if (!f) throw_errno("open " + path_file);
        std::fprintf(f, "%s\n", p->path().c_str());
        std::fclose(f);
      }
      port = std::move(p);
    } else {
      port = std::make_unique<TcpPort>(bind, static_cast<std::uint16_t>(tcp_port));
    }

    fw::Plant plant(pp, seed);
    fw::SimHal hal(plant, latency_ms * 1e-3);
    const fw::DeviceConfig dc = fw::device_config_for(pp, "devsim");
    if (calibrated) {
      hal.stored.valid = true;
      hal.stored.direction = static_cast<std::int8_t>(pp.direction);
      hal.stored.zero_electric = static_cast<float>(pp.electrical_offset);
    }
    const Nanos tick = static_cast<Nanos>(dc.tick_us) * 1000;
    const std::string sched = rt::prefer_low_latency(tick);
    const Nanos t0 = monotonic_now();
    auto sim_time = [&](Nanos t) { return static_cast<double>(t - t0) * 1e-9; };
    hal.set_time(0.0);
    fw::DeviceCore core(hal, dc);
    core.begin();
    std::fprintf(stderr, "devsim: %s, %u counts a turn, tick %u us, link %.2f ms each way, %s%s%s\n",
                 pty ? "pseudo-terminal" : ("tcp " + bind + ":" + std::to_string(tcp_port)).c_str(),
                 pp.counts_per_rev, dc.tick_us, latency_ms, sched.c_str(),
                 hand.present ? ", a hand on the knob once calibrated" : "",
                 stop.present ? ", an object" : "");
    // The hand takes hold once the device is calibrated, as a person lets
    // go while it calibrates (the host says so), and turns the knob from
    // where it then is; the object is put there then too, so that the
    // calibration's sweeps do not run into it.
    bool placed = false;

    std::uint8_t buf[4096];
    Nanos due = t0;
    std::uint64_t ticks = 0, late = 0;
    Nanos worst = 0;
    Nanos next_status = t0 + kNanosPerSecond;
    const Nanos end = duration > 0.0 ? t0 + from_seconds(duration) : 0;
    while (!g_stop.load()) {
      due += tick;
      sleep_until(due);
      const Nanos now = monotonic_now();
      const Nanos behind = now - due;
      worst = std::max(worst, behind);
      if (behind > tick) {
        // Behind by a tick or more: no catching up, which would only run
        // ticks back to back; the core measures the time between them.
        ++late;
        due = now;
      }
      if (end && now >= end) break;
      hal.set_time(sim_time(now));
      port->service();
      for (;;) {
        const long n = port->read(buf, sizeof buf);
        if (n <= 0) break;
        hal.host_write(buf, static_cast<std::size_t>(n));
      }
      core.tick();
      ++ticks;
      if (!placed && core.calibration().valid &&
          !(core.last_state().flags & device::kStateCalibrating)) {
        placed = true;
        if (hand.present) {
          hand.ref_offset = plant.angle();
          plant.set_hand(hand, sim_time(now));
        }
        if (stop.present) {
          stop.angle += plant.angle();
          plant.set_stop(stop);
        }
      }
      for (;;) {
        const std::size_t n = hal.host_read(buf, sizeof buf);
        if (n == 0) break;
        port->write(buf, n);
      }
      if (!quiet && now >= next_status) {
        next_status += kNanosPerSecond;
        const device::State& st = core.last_state();
        std::fprintf(stderr,
                     "t %.0f s  angle %+.3f rad  %+.4f N m%s%s%s  late ticks %llu, worst %.0f us\n",
                     sim_time(now), plant.angle(), plant.motor_torque(),
                     hal.driver_on() ? "  driving" : "",
                     (st.flags & device::kStateCalibrating) ? "  calibrating" : "",
                     core.fault() != device::Fault::kNone ? "  FAULT" : "",
                     static_cast<unsigned long long>(late), static_cast<double>(worst) / 1e3);
        worst = 0;
      }
    }
    std::fprintf(stderr, "devsim: %llu ticks, %llu late; %.4f J of motor work\n",
                 static_cast<unsigned long long>(ticks), static_cast<unsigned long long>(late),
                 plant.motor_work());
    return 0;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "tuushin-devsim: %s\n", e.what());
    return 1;
  }
}
