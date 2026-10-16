#pragma once

namespace tuushin::control {

// The quantization tax on readings that are noisy as well as quantized.
//
// Why: quantization_tax() (predictive_pc.hpp) makes the books a lower bound
// on the true work for every reading error confined to an interval of
// width q. A magnetic encoder (MT6701, AS5047P) dithers by about half a
// count, and its reading error is then unbounded: the books' error
//
//   B_N - W_N = sum_{k<N} phi_k (e_{k+1} - e_k)
//             = -sum_{k<N} dphi_k e_k + phi_{N-1} e_N,
//
// with e_k the error of the reading at t_k, phi_k the force over
// (t_k, t_{k+1}) and dphi_k = phi_k - phi_{k-1}, has no bound that holds
// for every error, and the tax that assumes one bounds nothing. What noise
// does allow is a bound that holds with a stated probability.
//
// Split each error into what the device's place in its count makes of it
// and the rest: e_k = b(x_k) + xi_k with b(x) = E[e | x]. For a reading
// Q(x + n), Q rounding to the count and n Gaussian of standard deviation
// sigma, b is periodic and |b| <= bias(sigma), and xi_k given x_k is
// sub-Gaussian: E[exp(t xi_k) | x_k] <= exp(t^2 s^2 / 2) with s =
// proxy(sigma), for every x (dither_bounds()). Then:
//
//  * The bias part, -sum dphi_k b(x_k) + phi_{N-1} b(x_N), is at most
//    bias (sum |dphi_k| + |phi_{N-1}|), which is 2 bias per newton by which
//    |phi| grows: quantization_tax() with 2 bias for q. Noise shrinks it
//    fast, as the Gaussian smooths the sawtooth's Fourier series: |b| <=
//    sum_k exp(-2 pi^2 k^2 sigma^2) / (pi k), 0.0023 q at half a count.
//  * The noise part. If each change dphi_k is decided without the reading
//    it pairs with, M_N = -sum_{k<N} dphi_k xi_k is a martingale whose
//    increments are sub-Gaussian with variance proxy s^2 dphi_k^2, so
//    exp(t M_N - t^2 V_N / 2) with V_N = sum s^2 dphi_k^2 is a
//    supermartingale for every t. Mixing t over a normal of variance
//    1 / rho and applying Ville's inequality (H. Robbins, "Statistical
//    methods related to the law of the iterated logarithm", Ann. Math.
//    Statist. 41(5), 1970; S. R. Howard, A. Ramdas, J. McAuliffe and
//    J. Sekhon, "Time-uniform, nonparametric, nonasymptotic confidence
//    sequences", Ann. Statist. 49(2), 2021) gives, at every N at once,
//
//      P(exists N: M_N >= U(V_N)) <= delta,
//      U(V) = sqrt((V + rho) (log((V + rho) / rho) + 2 log(1 / delta))).
//
//    U grows as the square root of the sum of squared changes, not as the
//    sum of their sizes: many small changes, or many changes over a long
//    run, cost far less than q per newton, as the errors they pair with
//    cancel. A single rise from rest costs more: with the fall reserved
//    for below, about 6 s |dphi| at delta = 1e-3 for 0.5-5 N, where
//    quantization_tax() takes q |dphi|, 1.7 s |dphi| at half a count.
//  * The end term phi_{N-1} xi_N, the newest reading's noise times the
//    force now, is not charged: it does not accumulate, as the next
//    reading replaces it, and no finite charge bounds it for every N. The
//    books then bound the true work from below, at every reading with
//    probability 1 - delta, up to the newest reading's noise times the
//    force now.
//
// When a change is decided without its reading. A device that applies a
// command a whole number of samples after it is issued (the lead, see
// DevicePredictor::set_lead) changes its force at a later reading than
// any the command saw: dphi_k depends on the readings before t_k only.
// Without a delay the force that acts from t_k is chosen from the reading
// at t_k, which can steer it into the noise; a delay that is not a whole
// number of samples changes the force within a sample, whose split the
// model makes from readings the command saw. Neither is covered.
//
// Reserving for the fall. A fall adds to V as much as a rise does, so with
// V alone a fall would be charged, and a controller with an empty budget
// could neither hold its force nor drop it. The charge is instead made
// against V* = V + s^2 phi^2, which is never below V: a change from prev
// to phi moves V* by 2 s^2 phi (phi - prev), which is positive for a rise
// of |phi| or a change of sign and not for a fall towards zero. The books
// are charged U(V*) less what they were charged so far whenever V* reaches
// a new peak, so a force that rises pays in advance for falling back, and
// falls are free, as with quantization_tax(): zero force stays affordable
// whatever the budget. U(0) = sqrt(2 rho log(1 / delta)) is charged when
// the books open.
//
// Planning a change. Since U is concave, it lies below each of its
// tangents, and a controller may count on a change costing at most the
// bias part plus a fixed part and 2 slope phi (phi - prev) where that is
// positive (ChangeTax): with the tangent at the peak, slope = s^2 U'(peak)
// and no fixed part, which is tight for small changes; with the tangent at
// the V* a change to a force `target` would reach, a smaller slope and a
// fixed part, which is exact at the target. A first rise from rest, the
// costliest change, is where the two differ most: to 2 N from rest at
// half a count, the tangent at the peak takes five times the charge.
//
// What the bound assumes: Gaussian noise before the rounding, of a
// standard deviation within the range given; independent from reading to
// reading (white at the sampling rate) and of the motion; and the lead the
// controller books with. Noise correlated over samples breaks the
// martingale: a slow drift of the readings is a bias the table does not
// cover.
struct DitherBounds {
  double bias = 0.5;   // in counts
  double proxy = 0.5;  // in counts
};

// The bounds for noise whose standard deviation lies in [lo, hi] counts:
// the bias at lo, as it falls with the noise, and the proxy at hi, as it
// grows with it. Both are suprema over where the device lies in its count,
// from a table every 0.05 counts up to one count that was computed from the
// exact distribution of the rounded reading (the bias also from its
// Fourier series), interpolated linearly, which bounds both from above
// between the grid points; beyond one count the proxy is the reading
// error's standard deviation, sqrt(sigma^2 + 1 / 12), with a margin.
DitherBounds dither_bounds(double lo, double hi);

// What a change of the port force from prev to phi pays at most: `rise`
// per newton by which |phi| grows (the whole of |phi| when the sign
// flips), plus `fixed` and 2 slope phi (phi - prev) when that is positive.
struct ChangeTax {
  double rise = 0.0;   // J/N
  double slope = 0.0;  // J/N^2
  double fixed = 0.0;  // J
  double most(double prev, double phi) const {
    const double kept = phi * prev > 0.0 ? (prev > 0.0 ? prev : -prev) : 0.0;
    const double grow = (phi > 0.0 ? phi : -phi) - kept;
    const double reserve = phi * (phi - prev);
    return rise * (grow > 0.0 ? grow : 0.0) +
           (reserve > 0.0 ? fixed + 2.0 * slope * reserve : 0.0);
  }
};

class NoiseTax {
 public:
  // `bias` and `proxy` in m (the resolution times dither_bounds()).
  // `delta` is the probability that the noise ever takes more from the
  // books than they were charged. The bound is tightest for a single
  // change of about `force_scale` N: rho = (proxy force_scale)^2.
  NoiseTax(double bias, double proxy, double delta = 1e-3, double force_scale = 0.25);

  // Opens the books: returns U(0), to book as an output, the first time.
  double open();
  // Books a change of the port force from `prev` to `phi`, decided without
  // the reading it takes effect on, and returns what to book as an output
  // (J). Opens the books if they are not.
  double charge(double prev, double phi);
  // What charge(prev, phi) would return, apart from opening the books.
  double cost(double prev, double phi) const;
  // The most charge() takes from now on, apart from opening the books: on
  // the tangent at the peak (no fixed part), or on the tangent at the V* a
  // change from `prev`, the force last charged, to `target` would reach,
  // which is exact there. Each bounds every change.
  ChangeTax plan() const;
  ChangeTax plan(double prev, double target) const;

  bool opened() const { return opened_; }
  // U(v) for v in J^2.
  double bound(double v) const;
  double variance() const { return v_; }   // V so far, J^2
  double charged() const { return paid_; } // the noise part charged so far, J

 private:
  double slope(double v) const;  // U'(v)
  double bias_, proxy_, rho_, log_delta2_;
  bool opened_ = false;
  double v_ = 0.0;     // V
  double peak_ = 0.0;  // the largest V* so far
  double paid_ = 0.0;  // U(peak_) once opened
};

}  // namespace tuushin::control
