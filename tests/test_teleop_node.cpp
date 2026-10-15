#include <atomic>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <thread>
#include <vector>

#include "test.hpp"
#include "tuushin/net/channel.hpp"
#include "tuushin/net/udp.hpp"
#include "tuushin/teleop/device.hpp"
#include "tuushin/teleop/link.hpp"
#include "tuushin/teleop/node.hpp"
#include "tuushin/wire/packet.hpp"

using namespace tuushin;
using namespace tuushin::teleop;

namespace {

net::UdpSocket bind_loopback() { return net::UdpSocket(*net::Endpoint::resolve("127.0.0.1", 0)); }

net::Endpoint peer_of(const net::UdpSocket& s) {
  return *net::Endpoint::resolve("127.0.0.1", s.local_endpoint().port());
}

// Receives one datagram within a second and hands it to `link`.
std::optional<HapticLink::Received> deliver(net::UdpSocket& sock, HapticLink& link,
                                            std::uint64_t* foreign_before = nullptr) {
  std::vector<std::uint8_t> buf(wire::kMaxDatagram + 1);
  if (!sock.wait_readable(kNanosPerSecond)) return std::nullopt;
  net::Endpoint from;
  Nanos rx = 0;
  const auto n = sock.recv_from(buf.data(), buf.size(), &from, &rx);
  if (!n) return std::nullopt;
  if (foreign_before) *foreign_before = link.counters().foreign;
  return link.on_datagram(buf.data(), *n, from, rx);
}

// A device that fails at a given time, as a link to a real one that closes.
class FailingDevice final : public Device1D {
 public:
  FailingDevice(Device1D& inner, Nanos fail_at) : inner_(inner), fail_at_(fail_at) {}
  double read_position(Nanos now) override {
    if (now >= fail_at_) failed_ = true;
    return inner_.read_position(now);
  }
  double velocity() const override { return inner_.velocity(); }
  void command_force(double f) override { inner_.command_force(f); }
  double contact_force() const override { return inner_.contact_force(); }
  Nanos force_cut_after() const override { return inner_.force_cut_after(); }
  const char* fault() const override { return failed_ ? "the link to the device closed" : nullptr; }

 private:
  Device1D& inner_;
  Nanos fail_at_;
  bool failed_ = false;
};

}  // namespace

TEST_CASE(seq_window_sorts_new_late_and_duplicate) {
  SeqWindow w;
  CHECK(w.accept(1) == SeqWindow::Verdict::kNew);
  CHECK(w.accept(2) == SeqWindow::Verdict::kNew);
  CHECK(w.accept(5) == SeqWindow::Verdict::kNew);
  CHECK(w.missing() == 2);  // 3 and 4
  CHECK(w.accept(4) == SeqWindow::Verdict::kLate);
  CHECK(w.accept(4) == SeqWindow::Verdict::kDuplicate);
  CHECK(w.accept(5) == SeqWindow::Verdict::kDuplicate);
  CHECK(w.accept(1) == SeqWindow::Verdict::kDuplicate);
  CHECK(w.highest() == 5);
  CHECK(w.distinct() == 4);
  CHECK(w.missing() == 1);
}

TEST_CASE(seq_window_forgets_slots_it_slides_over) {
  // A slot reused by a seq one window later must not look like a duplicate.
  SeqWindow w;
  CHECK(w.accept(10) == SeqWindow::Verdict::kNew);
  CHECK(w.accept(10 + 4096) == SeqWindow::Verdict::kNew);
  CHECK(w.accept(10) == SeqWindow::Verdict::kStale);
  CHECK(w.accept(11 + 4096) == SeqWindow::Verdict::kNew);
  CHECK(w.accept(4097) == SeqWindow::Verdict::kLate);  // within the window, unseen
  // A jump of more than a window clears everything.
  CHECK(w.accept(100000) == SeqWindow::Verdict::kNew);
  CHECK(w.accept(100000 - 4095) == SeqWindow::Verdict::kLate);
  CHECK(w.accept(100000 - 4096) == SeqWindow::Verdict::kStale);
  // Seqs near the top of the range do not loop.
  SeqWindow top;
  CHECK(top.accept(0xFFFFFFF0u) == SeqWindow::Verdict::kNew);
  CHECK(top.accept(0xFFFFFFFFu) == SeqWindow::Verdict::kNew);
  CHECK(top.accept(0xFFFFFFF5u) == SeqWindow::Verdict::kLate);
}

TEST_CASE(haptic_link_carries_samples_and_measures_the_round_trip) {
  net::UdpSocket sa = bind_loopback();
  net::UdpSocket sb = bind_loopback();
  sa.enable_rx_timestamps();
  sb.enable_rx_timestamps();
  HapticLink a(sa, 77, peer_of(sb), Passivity::kReflective);
  HapticLink b(sb, 77, peer_of(sa), Passivity::kReflective);

  a.send(monotonic_now(), Sample{0.25, -1.5, 0.003}, 0.002);
  const auto r1 = deliver(sb, b);
  REQUIRE(r1.has_value());
  CHECK(r1->seq == 1);
  CHECK(r1->sample.value == 0.25);
  CHECK(r1->sample.aux == -1.5);
  CHECK(r1->sample.energy == 0.003);
  CHECK(r1->monitor == 0.002);
  CHECK(!r1->final);
  CHECK(r1->verdict == SeqWindow::Verdict::kNew);
  CHECK(a.rtts().empty());

  // The answer echoes sample 1, so `a` learns the round trip and one exchange.
  b.send(monotonic_now(), Sample{4.0, 0.0, 0.0}, -0.001, true);
  const auto r2 = deliver(sa, a);
  REQUIRE(r2.has_value());
  CHECK(r2->final);
  CHECK(r2->monitor == -0.001);
  REQUIRE(a.rtts().size() == 1);
  CHECK(a.rtts()[0] > 0.0);
  CHECK(a.rtts()[0] < 1e9);
  REQUIRE(a.exchanges().size() == 1);
  const auto& x = a.exchanges()[0];
  CHECK(x.t1 <= x.t4);
  CHECK(x.t2 <= x.t3);  // the hold is not negative

  // A second answer echoing the same sample adds no exchange: the first echo
  // has the least waiting in it.
  b.send(monotonic_now(), Sample{4.5, 0.0, 0.0}, 0.0);
  REQUIRE(deliver(sa, a).has_value());
  CHECK(a.rtts().size() == 1);

  const auto ca = a.counters();
  CHECK(ca.sent == 1);
  CHECK(ca.received == 2);
  CHECK(ca.missing == 0);
  CHECK(ca.foreign == 0);
}

TEST_CASE(haptic_link_drops_foreign_and_duplicate_datagrams) {
  net::UdpSocket sa = bind_loopback();
  net::UdpSocket sb = bind_loopback();
  net::UdpSocket stranger = bind_loopback();
  HapticLink b(sb, 5, peer_of(sa), Passivity::kChannel);

  // Same session and scheme, but from another address.
  HapticLink impostor(stranger, 5, peer_of(sb), Passivity::kChannel);
  impostor.send(monotonic_now(), Sample{1.0, 0.0, 0.0}, 0.0);
  CHECK(!deliver(sb, b).has_value());
  // Right address, other session.
  HapticLink other_session(sa, 6, peer_of(sb), Passivity::kChannel);
  other_session.send(monotonic_now(), Sample{1.0, 0.0, 0.0}, 0.0);
  CHECK(!deliver(sb, b).has_value());
  // Right address and session, other scheme.
  HapticLink other_scheme(sa, 5, peer_of(sb), Passivity::kReflective);
  other_scheme.send(monotonic_now(), Sample{1.0, 0.0, 0.0}, 0.0);
  CHECK(!deliver(sb, b).has_value());
  CHECK(b.counters().foreign == 3);

  // The real peer, then the same datagram again as a duplicating network
  // would deliver it.
  HapticLink a(sa, 5, peer_of(sb), Passivity::kChannel);
  a.send(monotonic_now(), Sample{2.0, 0.0, 0.0}, 0.0);
  std::vector<std::uint8_t> buf(wire::kMaxDatagram + 1);
  REQUIRE(sb.wait_readable(kNanosPerSecond));
  net::Endpoint from;
  Nanos rx = 0;
  const auto n = sb.recv_from(buf.data(), buf.size(), &from, &rx);
  REQUIRE(n.has_value());
  CHECK(b.on_datagram(buf.data(), *n, from, rx).has_value());
  CHECK(!b.on_datagram(buf.data(), *n, from, rx).has_value());
  // Garbage.
  const std::uint8_t junk[] = {1, 2, 3};
  CHECK(!b.on_datagram(junk, sizeof junk, from, rx).has_value());
  const auto c = b.counters();
  CHECK(c.received == 1);
  CHECK(c.duplicates == 1);
  CHECK(c.foreign == 4);
}

TEST_CASE(haptic_link_emulation_delays_and_keeps_send_time) {
  net::UdpSocket sa = bind_loopback();
  net::UdpSocket sb = bind_loopback();
  HapticLink a(sa, 9, peer_of(sb), Passivity::kNone);
  HapticLink b(sb, 9, peer_of(sa), Passivity::kNone);
  a.set_emulation(std::make_unique<net::FixedDelay>(5 * kNanosPerMilli));

  const Nanos t0 = 1'000'000'000;
  a.send(t0, Sample{1.0, 0.0, 0.0}, 0.0);
  a.send(t0 + kNanosPerMilli, Sample{2.0, 0.0, 0.0}, 0.0);
  REQUIRE(a.next_release().has_value());
  CHECK(*a.next_release() == t0 + 5 * kNanosPerMilli);
  a.pump(t0 + 4 * kNanosPerMilli);
  CHECK(!sb.wait_readable(20 * kNanosPerMilli));  // nothing is due yet
  a.pump(t0 + 5 * kNanosPerMilli);
  const auto r = deliver(sb, b);
  REQUIRE(r.has_value());
  CHECK(r->seq == 1);
  CHECK(r->send_time == t0);  // as stamped by the sender, not at release
  CHECK(*a.next_release() == t0 + 6 * kNanosPerMilli);
  a.pump(t0 + 6 * kNanosPerMilli);
  CHECK(!a.next_release().has_value());
  REQUIRE(deliver(sb, b).has_value());
}

TEST_CASE(ticker_skips_after_a_long_stall) {
  Ticker t(kNanosPerMilli, 0);
  t.taken(0);
  CHECK(t.due() == kNanosPerMilli);
  t.taken(kNanosPerMilli + 200'000);  // a little late: keeps the grid
  CHECK(t.due() == 2 * kNanosPerMilli);
  CHECK(t.skipped() == 0);
  t.taken(3 * kNanosPerMilli + 500'000);  // a period and a half late: no catch-up tick
  CHECK(t.skipped() == 1);
  CHECK(t.due() == 4 * kNanosPerMilli);
  t.taken(50 * kNanosPerMilli + 300'000);  // descheduled for ~46 ms
  CHECK(t.skipped() == 47);
  CHECK(t.due() == 51 * kNanosPerMilli);  // still on the grid
  CHECK(t.ticks() == 4);
  CHECK(t.lateness().size() == 4);
}

TEST_CASE(nodes_run_a_session_over_loopback) {
  // The two ends in one process, on their own threads, as two machines with
  // 5 ms each way between them (emulated in-process).
  net::UdpSocket slave_sock = bind_loopback();
  net::UdpSocket master_sock = bind_loopback();
  const net::Endpoint slave_ep = peer_of(slave_sock);
  const Nanos period = kNanosPerMilli;
  const Nanos delay = 5 * kNanosPerMilli;

  WallParams wall;
  SimulatedSlave slave_dev(1.0, 2.0, wall, period, 10, monotonic_now(), 0.0);
  slave_dev.set_watchdog(2 * period);
  SlaveNodeConfig scfg;
  scfg.control.dt = period;
  scfg.control.passivity = Passivity::kReflective;
  scfg.control.hold_after = 150 * kNanosPerMilli;
  scfg.once = true;
  scfg.idle_timeout = 3 * kNanosPerSecond;
  SlaveNodeHooks hooks;
  hooks.emulation = [&](Nanos) { return std::make_unique<net::FixedDelay>(delay); };
  std::atomic<bool> stop{false};
  std::vector<SlaveSessionReport> slave_reports;
  std::thread slave([&] { slave_reports = run_slave_node(slave_sock, slave_dev, scfg, hooks, &stop); });

  const Nanos start = monotonic_now();
  OperatorParams op;
  SimulatedMaster master_dev(0.2, 0.5, op, period, 10, start);
  master_dev.set_watchdog(2 * period);
  MasterNodeConfig mcfg;
  mcfg.control.dt = period;
  mcfg.control.passivity = Passivity::kReflective;
  mcfg.control.model_mass = 0.2;
  mcfg.duration = 1500 * kNanosPerMilli;
  const auto rep = run_master_node(master_sock, slave_ep, master_dev, mcfg,
                                   std::make_unique<net::FixedDelay>(delay));
  stop.store(true);
  slave.join();

  CHECK(rep.slave_answered);
  CHECK(rep.slave_final);
  // The session ran its 1.5 s; ticks skipped on a loaded machine are the
  // scheduler's (see below), and up to a quarter of them were.
  CHECK(rep.ticks + rep.skipped > 1400);
  CHECK(rep.link.duplicates == 0);
  // The round trip is the two emulated delays plus a tick of waiting at most.
  REQUIRE(rep.rtt.count > 500);
  CHECK(rep.rtt.p50 > 1.9 * delay);
  CHECK(rep.rtt.p50 < 2.0 * delay + 3 * kNanosPerMilli);
  // Same process, same clock: the fit sees no offset and splits it evenly.
  CHECK(std::fabs(rep.clock.offset) < 2.0 * kNanosPerMilli);
  CHECK(std::fabs(rep.forward.p50 - static_cast<double>(delay)) < 2.0 * kNanosPerMilli);
  CHECK(std::fabs(rep.backward.p50 - static_cast<double>(delay)) < 2.0 * kNanosPerMilli);
  REQUIRE(slave_reports.size() == 1);
  const auto& s = slave_reports[0];
  // The teleoperator stays passive and moves, also when a thread was
  // descheduled long enough to skip ticks (no real-time priority here): the
  // devices' watchdogs cut the force a stalled loop would otherwise hold.
  CHECK(rep.min_margin > -1e-3);
  CHECK(rep.master_net + rep.slave_net > -1e-3);
  CHECK(rep.max_position > 0.005);

  CHECK(s.session == rep.session);
  CHECK(s.ended_by_final);
  // What one end sent reached the other.
  CHECK(rep.link.received > 0.95 * static_cast<double>(s.link.sent));
  CHECK(s.link.received > 0.95 * static_cast<double>(rep.link.sent));
  CHECK(s.max_position > 0.5 * rep.max_position);  // the slave followed
  CHECK(s.max_position < 2.0 * rep.max_position);
  CHECK(std::fabs(s.slave_net - rep.slave_net) < 1e-9);  // the final books arrived
}

TEST_CASE(a_device_fault_ends_the_session) {
  // The slave's device fails 400 ms into the session: the slave ends it as a
  // stop would, tells the master, and stops serving, since no later session
  // could go on; then the same with the master's device.
  for (const bool at_slave : {true, false}) {
    net::UdpSocket slave_sock = bind_loopback();
    net::UdpSocket master_sock = bind_loopback();
    const net::Endpoint slave_ep = peer_of(slave_sock);
    const Nanos period = kNanosPerMilli;
    const Nanos start = monotonic_now();
    const Nanos fail_at = start + 400 * kNanosPerMilli;
    const Nanos never = start + 3600 * kNanosPerSecond;

    WallParams wall;
    SimulatedSlave slave_sim(1.0, 2.0, wall, period, 10, start, 0.0);
    FailingDevice slave_dev(slave_sim, at_slave ? fail_at : never);
    SlaveNodeConfig scfg;
    scfg.control.dt = period;
    scfg.control.passivity = Passivity::kReflective;
    scfg.idle_timeout = 3 * kNanosPerSecond;
    std::atomic<bool> stop{false};
    std::vector<SlaveSessionReport> slave_reports;
    std::thread slave(
        [&] { slave_reports = run_slave_node(slave_sock, slave_dev, scfg, {}, &stop); });

    OperatorParams op;
    SimulatedMaster master_sim(0.2, 0.5, op, period, 10, start);
    FailingDevice master_dev(master_sim, at_slave ? never : fail_at);
    MasterNodeConfig mcfg;
    mcfg.control.dt = period;
    mcfg.control.passivity = Passivity::kReflective;
    mcfg.control.model_mass = 0.2;
    mcfg.duration = 3 * kNanosPerSecond;
    const auto rep = run_master_node(master_sock, slave_ep, master_dev, mcfg, nullptr);
    const Nanos master_done = monotonic_now();
    // A failed slave device stopped the slave node on its own.
    if (!at_slave) stop.store(true);
    slave.join();

    // Well before the 3 s the session was to run.
    CHECK(master_done - start < 1500 * kNanosPerMilli);
    REQUIRE(slave_reports.size() == 1);
    const auto& s = slave_reports[0];
    CHECK(rep.slave_final);
    CHECK(std::fabs(s.slave_net - rep.slave_net) < 1e-9);  // the final books arrived
    if (at_slave) {
      CHECK(s.device_fault == "the link to the device closed");
      CHECK(rep.device_fault.empty());
      CHECK(rep.ended_by_slave);
    } else {
      CHECK(rep.device_fault == "the link to the device closed");
      CHECK(s.device_fault.empty());
      CHECK(s.ended_by_final);
    }
  }
}
