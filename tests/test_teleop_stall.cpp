// The two real-time nodes (teleop/node.cpp) replayed in virtual time with
// scheduling stalls injected, to check what a descheduled control loop does
// to the energy books and to the physical energy, the work both actuators
// did on their devices.
#include <algorithm>
#include <cstdint>
#include <deque>
#include <random>
#include <vector>

#include "test.hpp"
#include "tuushin/teleop/controller.hpp"
#include "tuushin/teleop/device.hpp"
#include "tuushin/teleop/node.hpp"

using namespace tuushin;
using namespace tuushin::teleop;

namespace {

struct Stall {
  Nanos at, len;
};

struct Margins {
  double books = 0.0;     // minimum of the two ends' books, summed
  double physical = 0.0;  // minimum of minus the actuators' work
  std::uint64_t cuts = 0;  // samples during which a watchdog cut the force
};

// When a loop that wants to run at `due` actually runs: not before its last
// tick, and not while it is stalled.
Nanos run_time(const std::vector<Stall>& stalls, Nanos due, Nanos last) {
  Nanos t = std::max(due, last + 5000);
  for (const auto& s : stalls) {
    if (t >= s.at && t < s.at + s.len) t = s.at + s.len;
  }
  return t;
}

// The setup of tests/test_teleop_node.cpp: reflective scheme, 1 kHz, 5 ms
// each way, the slave holding after 150 ms without samples.
Margins run(Nanos watchdog, const std::vector<Stall>& master_stalls,
            const std::vector<Stall>& slave_stalls, Nanos duration = 3 * kNanosPerSecond) {
  struct Msg {
    Nanos arrive;
    std::uint32_t seq;
    Sample s;
  };
  const Nanos dt = kNanosPerMilli, delay = 5 * kNanosPerMilli;
  SimulatedMaster md(0.2, 0.5, OperatorParams{}, dt, 10, 0);
  SimulatedSlave sd(1.0, 2.0, WallParams{}, dt, 10, 0, 0.0);
  md.set_watchdog(watchdog);
  sd.set_watchdog(watchdog);
  MasterConfig mc;
  mc.dt = dt;
  SlaveConfig sc;
  sc.dt = dt;
  sc.hold_after = 150 * kNanosPerMilli;
  MasterController m(mc);
  SlaveController s(sc, sd.position());
  std::deque<Msg> to_slave, to_master;
  Ticker tm(dt, 0), ts(dt, 0);
  Nanos last_m = -dt, last_s = -dt;
  std::uint32_t seq_m = 0, seq_s = 0;
  Margins r;
  for (;;) {
    const Nanos nm = run_time(master_stalls, tm.due(), last_m);
    const Nanos ns = run_time(slave_stalls, ts.due(), last_s);
    const bool master_turn = nm <= ns;
    const Nanos now = master_turn ? nm : ns;
    if (now >= duration) break;
    if (master_turn) {
      for (; !to_master.empty() && to_master.front().arrive <= now; to_master.pop_front()) {
        m.receive(to_master.front().seq, to_master.front().s);
      }
      tm.taken(now);
      last_m = now;
      const double x = md.read_position(now);
      if (md.force_cut_after() > 0) ++r.cuts;
      const auto st = m.step(now, x, md.force_cut_after());
      md.command_force(-st.display_force);
      if (st.send) to_slave.push_back(Msg{now + delay, ++seq_m, st.sample});
    } else {
      for (; !to_slave.empty() && to_slave.front().arrive <= now; to_slave.pop_front()) {
        s.receive(to_slave.front().seq, to_slave.front().s);
      }
      ts.taken(now);
      last_s = now;
      const double x = sd.read_position(now);
      if (sd.force_cut_after() > 0) ++r.cuts;
      const auto st = s.step(now, x, sd.velocity(), sd.contact_force(), sd.force_cut_after());
      sd.command_force(st.force);
      if (st.send) {
        Sample out = st.sample;
        out.aux = x;
        to_master.push_back(Msg{now + delay, ++seq_s, out});
      }
    }
    md.read_position(now);
    sd.read_position(now);
    const double books =
        m.port().input_energy() - m.port().output_energy() + s.terminal_net();
    r.books = std::min(r.books, books);
    r.physical = std::min(r.physical, -(md.actuator_work() + sd.actuator_work()));
  }
  return r;
}

// Worst margins over stalls of `len` starting at 40 phases of the motion,
// at the master, the slave or both.
Margins worst_single_stalls(Nanos watchdog, Nanos len, bool master, bool slave) {
  Margins w;
  for (int k = 0; k < 40; ++k) {
    const std::vector<Stall> st{{300 * kNanosPerMilli + k * 50 * kNanosPerMilli + 137'000, len}};
    const auto r = run(watchdog, master ? st : std::vector<Stall>{}, slave ? st : std::vector<Stall>{});
    w.books = std::min(w.books, r.books);
    w.physical = std::min(w.physical, r.physical);
    w.cuts += r.cuts;
  }
  return w;
}

}  // namespace

TEST_CASE(stalled_loops_stay_passive_with_a_device_watchdog) {
  // A watchdog of two periods, the horizon the PC predicts over, bounds how
  // long a force acts unchecked; booking a cut force as predicted output and
  // waiting for the model to refill keeps the books on the safe side.
  const Nanos wd = 2 * kNanosPerMilli;
  for (const Nanos len : {12 * kNanosPerMilli, 50 * kNanosPerMilli, 100 * kNanosPerMilli}) {
    for (int who = 0; who < 3; ++who) {
      const auto w = worst_single_stalls(wd, len, who != 1, who != 0);
      CHECK(w.cuts > 0);
      CHECK(w.books > -2e-4);
      CHECK(w.physical > -2e-4);
    }
  }
  // Short random stalls at both ends, as on a loaded machine: 30 per second
  // and per end, up to 12 ms each. Physically the teleoperator never
  // generates energy; the books stay within tens of microjoules.
  for (int seed = 1; seed <= 20; ++seed) {
    std::mt19937_64 rng(seed);
    std::exponential_distribution<double> gap(30.0);
    std::uniform_real_distribution<double> length(0.2e-3, 12e-3);
    std::vector<Stall> ends[2];
    for (auto& v : ends) {
      for (double t = 0.05 + gap(rng); t < 3.0; t += gap(rng)) {
        const double l = length(rng);
        v.push_back(Stall{static_cast<Nanos>(t * 1e9), static_cast<Nanos>(l * 1e9)});
        t += l;
      }
    }
    const auto r = run(wd, ends[0], ends[1]);
    CHECK(r.books > -1e-4);
    CHECK(r.physical > -1e-6);
  }
}

TEST_CASE(stalled_loops_generate_energy_without_a_watchdog) {
  // Why the watchdog is needed: while a loop is descheduled nothing can take
  // its last force off the actuator, and that force does work no controller
  // checks. Over 50 ms stalls of the master the teleoperator generates
  // millijoules.
  const auto w = worst_single_stalls(0, 50 * kNanosPerMilli, true, false);
  CHECK(w.cuts == 0);
  CHECK(w.physical < -1e-3);
}
