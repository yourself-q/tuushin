#include <cmath>
#include <deque>
#include <vector>

#include "test.hpp"
#include "tuushin/control/predictive_pc.hpp"
#include "tuushin/rng.hpp"

using tuushin::Rng;
using tuushin::control::DevicePredictor;
using tuushin::control::quantization_tax;
using tuushin::control::quantization_tax_line;
using tuushin::control::TaxLine;

namespace {

constexpr double kPi = 3.14159265358979323846;

// A point mass under an external force H and a port force phi, both held
// over each sample, integrated in closed form; read by an encoder of
// resolution q that rounds to the nearest count.
struct QuantizedMass {
  double m, dt, q;
  double x, v = 0.0;
  double reading() const { return q * std::round(x / q); }
  // u = (the upper edge of the count read) - x, in [0, q].
  double u() const { return reading() + 0.5 * q - x; }
  void step(double H, double phi) {
    const double a = (H - phi) / m;
    x += v * dt + 0.5 * a * dt * dt;
    v += a * dt;
  }
};

// Drives a QuantizedMass around a slow sine with a port force that tracks
// it on the true state and chatters, under an H that drifts at up to
// 12 N/s, and feeds its readings to a predictor; calls `check` after each
// sample with the true u at every reading since the window's start, the
// last one last.
template <typename Check>
void run_quantized(Rng& rng, double q, int samples, Check check) {
  const double m = 0.2, dt = 1e-3;
  QuantizedMass dev{m, dt, q, 0.3 * q};
  DevicePredictor pred(m, dt, q);
  std::deque<double> truth;
  double y = dev.reading();
  for (int k = 0; k < samples; ++k) {
    const double t = k * dt;
    const double H = 2.0 + 1.5 * std::sin(2.0 * kPi * 1.3 * t);
    const double target = 5e-4 * std::sin(2.0 * kPi * 2.0 * t);
    const double phi = H + 100.0 * (dev.x - target) + 1.0 * dev.v + 0.2 * (rng.uniform() - 0.5);
    dev.step(H, phi);
    const double y_next = dev.reading();
    pred.observe(y_next - y, phi);
    y = y_next;
    truth.push_back(dev.u());
    while (truth.size() > pred.count_window_readings()) truth.pop_front();
    check(pred, truth);
  }
}

}  // namespace

// Summed by parts, the work the books miss over a force history that starts
// and ends at zero is -sum_k u_k (phi_k - phi_{k-1}), u_k where the device
// lay in its count at the change k: the tax lines at the true u add up to
// exactly that, and their worst over [0, q] to the tax that assumes nothing.
TEST_CASE(quantization_tax_line_is_the_misattributed_work_at_u) {
  Rng rng(11);
  const double q = 55e-6;
  for (int trial = 0; trial < 200; ++trial) {
    std::vector<double> phi(300);
    double f = 0.0;
    for (auto& p : phi) {
      const double r = rng.uniform();
      if (r < 0.3) f += (rng.uniform() - 0.5) * 4.0;
      else if (r < 0.45) f = -f;
      else if (r < 0.5) f = 0.0;
      p = f;
    }
    phi.push_back(0.0);
    double prev = 0.0, lines = 0.0, missed = 0.0;
    for (double p : phi) {
      const double u = q * rng.uniform();
      const TaxLine line = quantization_tax_line(q, prev, p);
      CHECK_NEAR(line.worst(0.0, q), quantization_tax(q, prev, p), 1e-18);
      CHECK(line.worst(0.0, q) >= line.alpha + line.beta * u - 1e-18);
      lines += line.alpha + line.beta * u;
      missed -= u * (p - prev);
      prev = p;
    }
    CHECK_NEAR(lines, missed, 1e-15);
  }
  // A change of sign is the fall to zero and the rise from it.
  const TaxLine flip = quantization_tax_line(q, 3.0, -2.0);
  const TaxLine fall = quantization_tax_line(q, 3.0, 0.0), rise = quantization_tax_line(q, 0.0, -2.0);
  CHECK_NEAR(flip.alpha, fall.alpha + rise.alpha, 1e-18);
  CHECK_NEAR(flip.beta, fall.beta + rise.beta, 1e-18);
}

// Whatever the readings in the window say of where the device lay at each
// of them holds of the true device, and says something most of the time.
TEST_CASE(count_bounds_hold_the_true_place_in_the_count) {
  Rng rng(5);
  const double q = 55e-6;
  long checked = 0, informative = 0;
  double width = 0.0;
  run_quantized(rng, q, 3000, [&](const DevicePredictor& pred, const std::deque<double>& truth) {
    for (std::size_t lag = 0; lag < truth.size(); ++lag) {
      const auto b = pred.count_bounds_at(lag);
      const double u = truth[truth.size() - 1 - lag];
      CHECK(b.lo <= u + 1e-12 && u <= b.hi + 1e-12);
      ++checked;
      width += b.hi - b.lo;
      if (b.hi - b.lo < 0.9 * q) ++informative;
    }
  });
  CHECK(checked > 50000);
  CHECK(informative > checked / 2);
  CHECK(width / static_cast<double>(checked) < 0.7 * q);
}

// The joint worst of several taxes in the window bounds their true sum, and
// over a chatter of the force it is far below the sum of their termwise
// worsts, since the device cannot lie at the worst place for each at once.
TEST_CASE(worst_tax_bounds_the_true_sum_and_undercuts_termwise_bounds) {
  Rng rng(9);
  const double q = 55e-6;
  double joint_chatter = 0.0, termwise_chatter = 0.0, true_chatter = 0.0;
  int sample = 0;
  run_quantized(rng, q, 3000, [&](const DevicePredictor& pred, const std::deque<double>& truth) {
    if (++sample % 7 != 0 || truth.size() < 12) return;
    const std::size_t n = truth.size();
    auto u_at = [&](std::size_t lag) { return truth[n - 1 - lag]; };
    // Random taxes at random lags.
    std::vector<DevicePredictor::LaggedTax> taxes;
    double truth_sum = 0.0;
    for (int i = 0; i < 6; ++i) {
      const auto lag = static_cast<std::size_t>(rng.uniform() * static_cast<double>(n)) % n;
      const TaxLine line =
          quantization_tax_line(q, (rng.uniform() - 0.5) * 8.0, (rng.uniform() - 0.5) * 8.0);
      taxes.push_back({lag, line});
      truth_sum += line.alpha + line.beta * u_at(lag);
    }
    CHECK(pred.worst_tax(taxes) >= truth_sum - 1e-15);
    // A force that rises by 0.2 N at one reading and falls back at the next,
    // over the last twelve readings.
    taxes.clear();
    double termwise = 0.0, chatter_truth = 0.0;
    for (std::size_t lag = 0; lag < 12; ++lag) {
      const TaxLine line = lag % 2 == 1 ? quantization_tax_line(q, 2.0, 2.2)
                                        : quantization_tax_line(q, 2.2, 2.0);
      taxes.push_back({lag, line});
      const auto b = pred.count_bounds_at(lag);
      termwise += line.worst(b.lo, b.hi);
      chatter_truth += line.alpha + line.beta * u_at(lag);
    }
    const double joint = pred.worst_tax(taxes);
    CHECK(joint >= chatter_truth - 1e-15);
    joint_chatter += joint;
    termwise_chatter += termwise;
    true_chatter += chatter_truth;
  });
  CHECK(termwise_chatter > 0.0);
  CHECK(joint_chatter < 0.5 * termwise_chatter);
  CHECK(joint_chatter >= true_chatter);
}
