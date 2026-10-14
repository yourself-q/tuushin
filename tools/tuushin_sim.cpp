// tuushin-sim: runs the 1-DoF teleoperation simulator over a modelled or
// recorded network path and prints stability / transparency metrics.
//
//   tuushin-sim --delay-ms 25 --jitter-ms 3 --loss 0.01
//   tuushin-sim --trace-fwd fwd.csv --trace-bwd bwd.csv --deadband 0.1
//   tuushin-sim --sweep 0,5,10,25,50,100      (one CSV row per one-way delay)

#include <cstdio>
#include <cstdlib>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "tuushin/net/channel.hpp"
#include "tuushin/net/trace.hpp"
#include "tuushin/sim/teleop1d.hpp"

using namespace tuushin;

namespace {

struct Options {
  sim::Teleop1DConfig cfg;
  double delay_ms = 0.0;
  double jitter_ms = 0.0;
  bool reorder = false;
  double loss = 0.0;
  bool burst = false;
  net::GilbertElliottLoss::Params ge;
  std::string trace_fwd, trace_bwd;
  std::vector<double> sweep;
  std::uint64_t seed = 1;
};

void usage() {
  std::fprintf(stderr,
               "usage: tuushin-sim [options]\n"
               "  --duration S          simulated time in seconds (10)\n"
               "  --delay-ms D          one-way delay, both directions (0)\n"
               "  --jitter-ms J         mean exponential jitter added to the delay (0)\n"
               "  --reorder             let jitter reorder packets (default: FIFO)\n"
               "  --loss P              independent packet loss probability (0)\n"
               "  --burst-loss a,b,c,d  Gilbert-Elliott p_gb,p_bg,loss_good,loss_bad\n"
               "  --trace-fwd FILE      replay a recorded master->slave path\n"
               "  --trace-bwd FILE      replay a recorded slave->master path\n"
               "  --passivity S         none, channel (conventional TDPA, default) or\n"
               "                        reflect (TDPA with energy reflection)\n"
               "  --no-tdpa             same as --passivity none\n"
               "  --constant-velocity-pc  PCs assume constant velocity over a sample\n"
               "  --model-mass M        master mass the PC believes in kg (true mass)\n"
               "  --deadband K          perceptual deadband with Weber fraction K, both ways\n"
               "  --wall-k K            wall stiffness in N/m (5000)\n"
               "  --contact-force       slave reports contact force, not coupling force\n"
               "  --drift-gain G        position-drift compensation gain in 1/s (0 = off)\n"
               "  --drift-ref R         drift measured against 'cmd' (default) or 'slave'\n"
               "  --seed N              random seed (1)\n"
               "  --log FILE            write a per-sample CSV log\n"
               "  --sweep D1,D2,...     run once per one-way delay (ms), print CSV\n");
}

std::vector<double> parse_list(const std::string& s) {
  std::vector<double> out;
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, ',')) out.push_back(std::stod(item));
  return out;
}

Options parse(int argc, char** argv) {
  Options o;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&]() -> std::string {
      if (i + 1 >= argc) throw std::invalid_argument("missing value for " + a);
      return argv[++i];
    };
    if (a == "--duration") o.cfg.duration = from_seconds(std::stod(next()));
    else if (a == "--delay-ms") o.delay_ms = std::stod(next());
    else if (a == "--jitter-ms") o.jitter_ms = std::stod(next());
    else if (a == "--reorder") o.reorder = true;
    else if (a == "--loss") o.loss = std::stod(next());
    else if (a == "--burst-loss") {
      const auto v = parse_list(next());
      if (v.size() != 4) throw std::invalid_argument("--burst-loss needs 4 values");
      o.burst = true;
      o.ge = {v[0], v[1], v[2], v[3]};
    }
    else if (a == "--trace-fwd") o.trace_fwd = next();
    else if (a == "--trace-bwd") o.trace_bwd = next();
    else if (a == "--no-tdpa") o.cfg.passivity = sim::Teleop1DConfig::Passivity::kNone;
    else if (a == "--constant-velocity-pc") o.cfg.predictive_pc = false;
    else if (a == "--model-mass") o.cfg.master_model_mass = std::stod(next());
    else if (a == "--passivity") {
      const std::string p = next();
      using P = sim::Teleop1DConfig::Passivity;
      if (p == "none") o.cfg.passivity = P::kNone;
      else if (p == "channel") o.cfg.passivity = P::kChannel;
      else if (p == "reflect") o.cfg.passivity = P::kReflective;
      else throw std::invalid_argument("--passivity must be none, channel or reflect");
    }
    else if (a == "--deadband") {
      const double k = std::stod(next());
      o.cfg.forward_deadband = o.cfg.backward_deadband = true;
      o.cfg.forward_deadband_params.k = k;
      o.cfg.backward_deadband_params.k = k;
    }
    else if (a == "--wall-k") o.cfg.wall.k = std::stod(next());
    else if (a == "--contact-force") o.cfg.slave.report_contact_force = true;
    else if (a == "--drift-gain") o.cfg.slave.drift_gain = std::stod(next());
    else if (a == "--drift-ref") {
      const std::string r = next();
      if (r == "cmd") o.cfg.slave.drift_reference = sim::SlaveParams::DriftReference::kCommand;
      else if (r == "slave") o.cfg.slave.drift_reference = sim::SlaveParams::DriftReference::kSlavePosition;
      else throw std::invalid_argument("--drift-ref must be cmd or slave");
    }
    else if (a == "--seed") o.seed = std::stoull(next());
    else if (a == "--log") o.cfg.log_path = next();
    else if (a == "--sweep") o.sweep = parse_list(next());
    else if (a == "-h" || a == "--help") { usage(); std::exit(0); }
    else throw std::invalid_argument("unknown option " + a);
  }
  return o;
}

std::unique_ptr<net::ChannelModel> make_channel(const Options& o, double delay_ms,
                                                const std::string& trace, std::uint64_t seed) {
  std::unique_ptr<net::ChannelModel> ch;
  if (!trace.empty()) {
    ch = std::make_unique<net::TraceChannel>(net::Trace::load_csv(trace));
  } else if (o.jitter_ms > 0.0) {
    ch = std::make_unique<net::JitterDelay>(from_millis(delay_ms), from_millis(o.jitter_ms),
                                            !o.reorder, seed);
  } else {
    ch = std::make_unique<net::FixedDelay>(from_millis(delay_ms));
  }
  if (o.burst) {
    ch = std::make_unique<net::GilbertElliottLoss>(std::move(ch), o.ge, seed + 100);
  } else if (o.loss > 0.0) {
    net::GilbertElliottLoss::Params iid{0.0, 1.0, o.loss, 1.0};
    ch = std::make_unique<net::GilbertElliottLoss>(std::move(ch), iid, seed + 100);
  }
  return ch;
}

void print_result(const sim::Teleop1DResult& r) {
  std::printf("diverged               %s", r.diverged ? "yes" : "no");
  if (r.diverged) std::printf(" (at %.3f s)", r.diverged_at);
  std::printf("\n");
  std::printf("min_passivity_margin   %.6g J\n", r.min_passivity_margin);
  std::printf("min_channel_margin     %.6g J\n", r.min_channel_margin);
  std::printf("max_master_pos         %.4f m\n", r.max_master_pos);
  std::printf("max_display_force      %.3f N\n", r.max_display_force);
  std::printf("displayed_stiffness    %.1f N/m\n", r.displayed_stiffness);
  std::printf("pos_rmse               %.5f m\n", r.pos_rmse);
  std::printf("final_drift            %.5f m\n", r.final_drift);
  std::printf("force_rmse             %.3f N\n", r.force_rmse);
  std::printf("forward_packet_rate    %.1f /s (loss %.2f %%)\n", r.forward_packet_rate,
              100.0 * r.forward_loss);
  std::printf("backward_packet_rate   %.1f /s (loss %.2f %%)\n", r.backward_packet_rate,
              100.0 * r.backward_loss);
  std::printf("master_pc_active       %.2f %%\n", 100.0 * r.master_pc_active);
  std::printf("slave_pc_active        %.2f %%\n", 100.0 * r.slave_pc_active);
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Options o = parse(argc, argv);
    if (o.sweep.empty()) {
      auto fwd = make_channel(o, o.delay_ms, o.trace_fwd, o.seed);
      auto bwd = make_channel(o, o.delay_ms, o.trace_bwd, o.seed + 1);
      print_result(sim::run_teleop1d(o.cfg, *fwd, *bwd));
      return 0;
    }
    std::printf("one_way_ms,diverged,min_margin_J,max_force_N,displayed_k_Npm,pos_rmse_m,"
                "drift_m,force_rmse_N,fwd_rate,bwd_rate,pc_m,pc_s\n");
    for (const double d : o.sweep) {
      auto fwd = make_channel(o, d, o.trace_fwd, o.seed);
      auto bwd = make_channel(o, d, o.trace_bwd, o.seed + 1);
      const auto r = sim::run_teleop1d(o.cfg, *fwd, *bwd);
      std::printf("%g,%d,%.4g,%.3f,%.1f,%.5f,%.5f,%.3f,%.1f,%.1f,%.4f,%.4f\n", d,
                  r.diverged ? 1 : 0, r.min_passivity_margin, r.max_display_force,
                  r.displayed_stiffness, r.pos_rmse, r.final_drift, r.force_rmse,
                  r.forward_packet_rate, r.backward_packet_rate, r.master_pc_active,
                  r.slave_pc_active);
    }
    return 0;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "tuushin-sim: %s\n", e.what());
    usage();
    return 2;
  }
}
