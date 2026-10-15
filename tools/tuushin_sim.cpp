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
  std::vector<double> master_compliance, slave_compliance;
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
               "  --slave-model-mass M  slave mass the PC believes in kg (true mass)\n"
               "  --master-friction C[,S]  Coulomb and stiction friction of the master in N\n"
               "  --slave-friction C[,S]   the same for the slave\n"
               "  --master-compliance F,HZ[,Z]  a fraction F of the master's mass sits\n"
               "                        beyond a spring at HZ Hz, damping ratio Z (0.05)\n"
               "  --slave-compliance F,HZ[,Z]   the same for the slave\n"
               "  --master-actuation-delay-ms D  delay before a master force takes effect\n"
               "  --slave-actuation-delay-ms D   the same for the slave\n"
               "  --actuation-delay-unknown  controllers take every force to act at once\n"
               "  --estimate-actuation-delay MAX  controllers estimate their devices' delays\n"
               "                        online, from 0 to MAX ms\n"
               "  --sample-jitter-ms J  each end samples up to J off its 1 ms grid (< 0.5)\n"
               "  --pc-hold N           samples a PC force must stay affordable if held (2)\n"
               "  --master-encoder Q    master encoder resolution in m (0 = exact)\n"
               "  --slave-encoder Q     slave encoder resolution in m (0 = exact)\n"
               "  --master-encoder-noise N  Gaussian noise of N counts on the master's\n"
               "                        readings before rounding (0)\n"
               "  --slave-encoder-noise N   the same for the slave (0)\n"
               "  --encoders-unknown    controllers treat the encoder readings as exact\n"
               "  --force-rate R        with encoders: operator's least force rate in N/s (20)\n"
               "  --fixed-force-rate    with encoders: the filters keep the force rate\n"
               "  --fixed-reading-noise with encoders: the filters take a reading's error to\n"
               "                        be the quantization's, not what the readings show\n"
               "  --pc-confidence K     PC predictions' confidence factor (1)\n"
               "  --pc-tax N            samples a force rise may take to repay the books'\n"
               "                        deficit and, with encoders, its quantization tax;\n"
               "                        0: no repayment, the tax at once (4)\n"
               "  --pc-rise N           with encoders: samples a master force rise must be\n"
               "                        affordable for on the expected motion, 0 = off (64)\n"
               "  --slave-pc-rise N     the same at the slave (0)\n"
               "  --initial-energy E    reflect: energy in the store at start in J (0)\n"
               "  --deadband K          perceptual deadband with Weber fraction K, both ways\n"
               "  --wall-k K            wall stiffness in N/m (5000)\n"
               "  --contact-force       slave reports contact force, not coupling force\n"
               "                        (none and channel only)\n"
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

teleop::FrictionParams parse_friction(const std::string& name, const std::string& s) {
  const auto v = parse_list(s);
  if (v.empty() || v.size() > 2) throw std::invalid_argument(name + " needs 1 or 2 values");
  teleop::FrictionParams f;
  f.coulomb = v[0];
  f.stiction = v.size() > 1 ? v[1] : 0.0;
  return f;
}

// Two masses, a fraction `v[0]` of `mass` beyond the spring, resonating
// against each other at v[1] Hz with damping ratio v[2].
teleop::ComplianceParams compliance(const std::string& name, double mass,
                                    const std::vector<double>& v) {
  if (v.size() < 2 || v.size() > 3 || !(v[0] > 0.0 && v[0] < 1.0) || !(v[1] > 0.0)) {
    throw std::invalid_argument(name + " needs F in (0, 1), HZ > 0 and optionally Z");
  }
  const double m2 = v[0] * mass, mu = (mass - m2) * m2 / mass;
  const double w = 2.0 * 3.14159265358979323846 * v[1], zeta = v.size() > 2 ? v[2] : 0.05;
  return teleop::ComplianceParams{m2, mu * w * w, 2.0 * zeta * mu * w};
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
    else if (a == "--slave-model-mass") o.cfg.slave_model_mass = std::stod(next());
    else if (a == "--master-friction") o.cfg.master.friction = parse_friction(a, next());
    else if (a == "--slave-friction") o.cfg.slave.friction = parse_friction(a, next());
    else if (a == "--master-compliance") o.master_compliance = parse_list(next());
    else if (a == "--slave-compliance") o.slave_compliance = parse_list(next());
    else if (a == "--master-actuation-delay-ms") {
      o.cfg.master.actuation_delay = from_millis(std::stod(next()));
    }
    else if (a == "--slave-actuation-delay-ms") {
      o.cfg.slave.actuation_delay = from_millis(std::stod(next()));
    }
    else if (a == "--sample-jitter-ms") o.cfg.sample_jitter = from_millis(std::stod(next()));
    else if (a == "--pc-hold") o.cfg.pc_hold_samples = std::stoi(next());
    else if (a == "--master-encoder") o.cfg.master_encoder_resolution = std::stod(next());
    else if (a == "--slave-encoder") o.cfg.slave_encoder_resolution = std::stod(next());
    else if (a == "--master-encoder-noise") o.cfg.master_encoder_noise = std::stod(next());
    else if (a == "--slave-encoder-noise") o.cfg.slave_encoder_noise = std::stod(next());
    else if (a == "--encoders-unknown") o.cfg.encoder_resolution_known = false;
    else if (a == "--actuation-delay-unknown") {
      o.cfg.master_model_actuation_delay = 0;
      o.cfg.slave_model_actuation_delay = 0;
    }
    else if (a == "--estimate-actuation-delay") {
      o.cfg.master_actuation_delay_max = o.cfg.slave_actuation_delay_max =
          from_millis(std::stod(next()));
    }
    else if (a == "--force-rate") o.cfg.force_rate = std::stod(next());
    else if (a == "--fixed-force-rate") o.cfg.adapt_force_rate = false;
    else if (a == "--fixed-reading-noise") o.cfg.estimate_reading_noise = false;
    else if (a == "--pc-confidence") o.cfg.pc_confidence = std::stod(next());
    else if (a == "--pc-tax") o.cfg.pc_tax_samples = std::stoi(next());
    else if (a == "--pc-rise") o.cfg.master_pc_rise_samples = std::stoi(next());
    else if (a == "--slave-pc-rise") o.cfg.slave_pc_rise_samples = std::stoi(next());
    else if (a == "--initial-energy") o.cfg.initial_energy = std::stod(next());
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
  if (!o.master_compliance.empty()) {
    o.cfg.master.compliance =
        compliance("--master-compliance", o.cfg.master.mass, o.master_compliance);
  }
  if (!o.slave_compliance.empty()) {
    o.cfg.slave.compliance = compliance("--slave-compliance", o.cfg.slave.mass, o.slave_compliance);
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
  std::printf("min_true_margin        %.6g J\n", r.min_true_margin);
  std::printf("max_book_excess        %.6g J\n", r.max_book_excess);
  std::printf("master_actuation_delay %.4f ms\n", to_seconds(r.master_actuation_delay) * 1e3);
  std::printf("slave_actuation_delay  %.4f ms\n", to_seconds(r.slave_actuation_delay) * 1e3);
  std::printf("master_reading_noise   %.3f um\n", r.master_reading_noise * 1e6);
  std::printf("slave_reading_noise    %.3f um\n", r.slave_reading_noise * 1e6);
  std::printf("min_channel_margin     %.6g J\n", r.min_channel_margin);
  std::printf("max_master_pos         %.4f m\n", r.max_master_pos);
  std::printf("max_display_force      %.3f N\n", r.max_display_force);
  std::printf("displayed_stiffness    %.1f N/m\n", r.displayed_stiffness);
  std::printf("pos_rmse               %.5f m\n", r.pos_rmse);
  std::printf("final_drift            %.5f m\n", r.final_drift);
  std::printf("force_rmse             %.3f N\n", r.force_rmse);
  std::printf("pc_force_rmse          %.3f N\n", r.pc_force_rmse);
  std::printf("display_variation      %.1f N/s\n", r.display_force_variation);
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
    std::printf("one_way_ms,diverged,min_margin_J,min_true_margin_J,max_force_N,"
                "displayed_k_Npm,pos_rmse_m,drift_m,force_rmse_N,fwd_rate,bwd_rate,pc_m,pc_s,"
                "pc_force_rmse_N,display_variation_Nps\n");
    for (const double d : o.sweep) {
      auto fwd = make_channel(o, d, o.trace_fwd, o.seed);
      auto bwd = make_channel(o, d, o.trace_bwd, o.seed + 1);
      const auto r = sim::run_teleop1d(o.cfg, *fwd, *bwd);
      std::printf("%g,%d,%.4g,%.4g,%.3f,%.1f,%.5f,%.5f,%.3f,%.1f,%.1f,%.4f,%.4f,%.3f,%.1f\n", d,
                  r.diverged ? 1 : 0, r.min_passivity_margin, r.min_true_margin,
                  r.max_display_force,
                  r.displayed_stiffness, r.pos_rmse, r.final_drift, r.force_rmse,
                  r.forward_packet_rate, r.backward_packet_rate, r.master_pc_active,
                  r.slave_pc_active, r.pc_force_rmse, r.display_force_variation);
    }
    return 0;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "tuushin-sim: %s\n", e.what());
    usage();
    return 2;
  }
}
