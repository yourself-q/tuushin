#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <optional>
#include <vector>

#include "tuushin/control/bounded_lp.hpp"

#include "tuushin/control/tdpa.hpp"

namespace tuushin::control {

// Impedance-type passivity control for a force-controlled device, with the
// device's response to the force itself included in the energy prediction.
//
// Why: a force held over a sample (zero-order hold) does the work
// phi * (x(k+1) - x(k)) on the device, and a passivity controller has to
// decide phi before that displacement is known. The classical controllers
// (tdpa.hpp, and J.-H. Ryu, D.-S. Kwon and B. Hannaford, IEEE T-RA 2002)
// assume the velocity stays constant over the sample. On a light device that
// fails exactly when it matters: a 10 N force switched onto a 0.2 kg master
// that creeps at 1 mm/s reverses it within the 1 ms sample, so a sample the
// controller expected to be an input turns out to be an output. Booked from
// the realized displacement (EnergyPort::account_energy) the deficit is
// visible, but a controller that lets predicted inputs through keeps adding
// to it, and the force chatters between zero and full. J.-H. Ryu, Y. S. Kim
// and B. Hannaford, "Sampled- and continuous-time passivity and stability of
// virtual environments", IEEE T-RA 20(4), 2004, likewise drop the
// constant-velocity assumption.
//
// Model: the device is a mass m driven by the port force and by everything
// else (the operator's hand, gravity, friction), which changes little over
// two samples. With F_j = H - phi_j the net force during sample j, a mass
// under zero-order hold moves
//
//   dx_j - dx_{j-1} = (F_j + F_{j-1}) * dt^2 / (2 m),
//
// and subtracting the same relation one sample earlier eliminates both the
// unknown H and the unknown velocity:
//
//   dx_k = 2 dx_{k-1} - dx_{k-2} - (phi_k - phi_{k-2}) * dt^2 / (2 m)
//        = P - c * phi_k,          c = dt^2 / (2 m).
//
// The work done on the port over the coming sample is then the concave
// quadratic W(phi) = phi * (P - c * phi). Scaling the requested force by
// lambda in [0, 1] keeps W(lambda phi) >= -budget for every lambda up to the
// positive root, which is computed in closed form.
//
// Consequences worth knowing:
//  * lambda = 0 is always feasible and does exactly zero work, so a deficit
//    can only grow through model error, not through the controller's logic.
//  * With no budget the force may still be held while the device is at rest
//    (W = 0) and may rise by about m * v / dt per sample while the device
//    moves into it, so the force ramps up instead of jumping. This is the
//    behaviour gradient-limited passivity controllers are designed to have;
//    here it falls out of the device dynamics.
//  * The mass is a model parameter. Underestimating it overestimates the
//    device's response and makes the controller more conservative;
//    overestimating it lets the controller expect inputs that do not come.
//    The model error margin (see "Model error") covers either once the
//    misses show: in the 1-DoF simulator (sim/teleop1d.hpp, reflective
//    scheme, default scenario, exact readings, 10-100 ms) half to twice
//    the true mass keep the deficit below 1 uJ, where without the margin
//    half the true mass and +-20 % left 0.1 mJ and twice the true mass at
//    the master 0.14 J. When in doubt, err light.
//
// Holding horizon. The largest force with W >= -budget over one sample is,
// without budget, the force that brings the device's mean displacement over
// the sample to zero: the device enters the sample moving into the force and
// leaves it moving out at the same speed. The next sample then has to drop
// the force to zero, the hand pushes the device back in, and the force
// chatters between zero and full at half the sample rate while the books
// stay balanced. The classical TDPA has the same on/off behaviour; H. Singh,
// A. Jafari and J.-H. Ryu, "Enhancing the force transparency of time domain
// passivity approach: observer-based gradient controller", ICRA 2019, attack
// it with an observer as well (their method is not reproduced here).
// Requiring in addition that the force stays affordable when held for n
// samples removes the reversal. With the same model the displacement over n
// held samples is
//
//   D_n = n * v * dt + n^2 * c * (H - phi),
//
// where v and H are recovered from the history like P above, and the work is
// again a concave quadratic in phi. For n = 2 and no budget the bound is
// H + m * v / dt, the force that stops the device within one sample: contact
// without stored energy becomes perfectly inelastic instead of bouncing, and
// at rest the force follows the operator's push. Larger n only soften it.
//
// Samples of unequal length. A real-time loop that is a fraction of a period
// late shortens the next sample by as much, and the recursion above, which
// assumes every sample lasts dt, then misreads the velocity in proportion:
// a 0.68 ms sample read as 1 ms underestimates it by a third, and so the
// output a large force will draw from a device moving into it. With the
// actual lengths h_1 (last sample) and h_2 (the one before) the same model,
// solved for the two unknowns, gives
//
//   H = (2 m (dx_1 / h_1 - dx_2 / h_2) + phi_1 h_1 + phi_2 h_2) / (h_1 + h_2),
//   v = dx_1 / h_1 + (H - phi_1) h_1 / (2 m),
//
// which reduce to the expressions above when h_1 = h_2 = dt. The coming
// sample is predicted at the nominal length; if it turns out longer, the
// device's watchdog bounds how long the force acts (teleop::Device1D).
//
// Irregular samples. The model takes everything except the port force to be
// constant over the history; that fails for a sample much longer than dt
// (the loop was descheduled: after a 50 ms stall dx_1 is fifty samples'
// worth of motion under a hand that has moved on), and the recursion fails
// for a sample during which the device's watchdog cut the force. Samples
// further than half a period from dt, and samples observed as irregular, put
// the predictor out of ready() until two regular samples have refilled the
// history; until then limit_force_predictive applies no force, which does
// exactly zero work whatever the device does. At 1 kHz that is 2 ms without
// force after a stall of tens of milliseconds. Samples much shorter than dt
// count as irregular too: on a real encoder dx_1 / h_1 amplifies the
// quantization by dt / h_1.
//
// Actuation delay. A device that applies each command some time d after it
// is issued (a servo loop that picks commands up on its own ticks, a bus
// that carries them) breaks two assumptions above: the force held over a
// sample is no longer the one commanded at its start, and the force being
// chosen does not act over the coming sample but over one that starts d
// later. Booked as if each force acted at once, the work of one force is
// credited to the next; in the 1-DoF simulator (reflective scheme, nine
// hand motions, 10-100 ms) a master delay of 1 ms let the actuators
// generate 0.14 J with exact readings, and 2 ms 1.2 J, while the books
// showed nothing. Both are fixed by knowing d (teleop::ActuatorSchedule):
//
//  * A sample whose force changed within it is observed as segments
//    phi_i over r_i samples. The mass model is still exact: the segments
//    enter the recursion as two held forces, phi_v, which changes the
//    velocity as much (phi_v r = sum phi_i r_i), and phi_d, which moves the
//    device as far (phi_d r^2 = sum phi_i r_i (2 (r - s_i) - r_i) for the
//    segment starting s_i samples in). The Kalman filter's transition takes
//    them the same way.
//  * The encoder sees only the sample's total displacement. Each segment
//    but the last is credited with the displacement the model predicts for
//    it from the state at the sample's start, and the last with the rest
//    (realized_work), so that the books still add up to what was read.
//  * The forces already commanded but not yet in effect, the lead, act
//    before the new one. Their predicted work, less `confidence` times its
//    standard deviation, is added to the budget, and the new force's
//    horizons start where the lead ends: hold_base() from the velocity the
//    lead leaves, hold_sd() over samples R to R + n, (R + n)^2 - R^2 instead
//    of n^2 for the model error.
//
// With d known the same runs leave a few microjoules: the split misplaces
// up to 16 uJ against what the model leaves out within a sample (the
// hand's impedance, the damping), and with 2 ms a prediction four samples
// ahead overshoots by up to 0.12 mJ with exact readings. Coarse encoders,
// whose books already allow for misses, generate nothing (3 mJ to 0.6 J
// unaware). The delay has to be known: believed 0.1 ms off, exact readings
// let the actuators generate up to 0.9 mJ, and half a millisecond off up
// to 35 mJ.
//
// Encoder quantization. Everything above takes the encoder to be exact. A
// real one reads the position to a resolution q (about 55 um on a
// 3D Systems Touch, 10 um on a Force Dimension omega), and that breaks two
// things.
//
//  * The books. Work booked from readings y = x + e, phi_k (y_{k+1} - y_k),
//    differs from the work done, phi_k (x_{k+1} - x_k), by phi_k times the
//    change of the reading error e. Summed by parts the difference is
//    sum_k e_k (phi_{k-1} - phi_k) plus the end terms: every change of the
//    force is booked against a position error of up to q/2, and nothing
//    makes those errors cancel. A force that chatters between 0 and 10 N
//    while the reading dithers between two counts draws energy the books
//    show as entering. quantization_tax() bounds the term sample by sample,
//    and the controllers book it as an output, which makes their books a
//    lower bound on the true work for any reading error confined to an
//    interval of width q (rounding, truncation, a constant offset). The
//    tax charges only increases of |phi| (and the whole force when its sign
//    flips), so a force that rises once and is held pays q |phi| once.
//    J. J. Abbott and A. M. Okamura, "Effects of position quantization and
//    sampling rate on virtual-wall passivity", IEEE T-RO 21(5), 2005, and
//    N. Diolaiti et al., "Stability of haptic rendering: discretization,
//    quantization, time delay, and Coulomb effects", IEEE T-RO 22(2), 2006,
//    show the same mechanism for virtual walls: quantization alone can make
//    a sampled wall non-passive.
//  * The prediction. The recursion above differentiates the readings twice:
//    one count in dx_1 moves the estimate of H by up to 2 m q / dt^2, which
//    is 22 N for a 0.2 kg device with a 55 um encoder at 1 kHz. With a
//    resolution given, the predictor estimates v and H with a Kalman filter
//    on the same model instead, as P. R. Belanger et al., "Estimation of
//    angular velocity and acceleration from shaft-encoder measurements",
//    IJRR 17(11), 1998, do for encoders: the state is the position, the
//    velocity and H; the reading error is taken as uniform over a count
//    (variance q^2 / 12), and H as a random walk whose change over a sample
//    has standard deviation force_rate * dt. The predicted displacement is
//    still that of the device, not of its readings; the difference between
//    the two is what the tax pays for. Smoother estimates also mean fewer
//    and smaller force changes, and so a smaller tax.
//
// Repaying the tax. Without a budget the tax of a rise has to come from the
// input the new force draws. Paid within the coming sample, it needs the
// device to move more than q per sample into the force, 5.5 cm/s for a
// Touch at 1 kHz: on a device pressed more slowly a force cannot start from
// zero, so the master displays nothing, takes in nothing and never earns
// the budget that would let it. With tax_samples = N the one-sample and
// hold constraints leave the tax out, and a rise must instead repay it,
// with any deficit of the books, from the work predicted over N samples
// with the force held:
//
//   lambda * G_N - lambda^2 * C_N - q * (lambda |phi| - kept) >= -budget,
//
// with G_N and C_N the coefficients of the work over N samples above and
// kept the part of the held force with phi's sign. That lowers the
// threshold to about q / (N dt), 1.4 cm/s for N = 4. The books still take
// the tax at once; if the device does not come, they go into deficit by at
// most about one tax and block further rises until inputs repay it.
//
// Trusting the prediction. A horizon of N samples trusts the prediction
// over N samples, so its error matters. limit_force_predictive takes
// `confidence` standard deviations (hold_sd) off the predicted work: a
// lower confidence bound. That bound is only as good as the filter's idea
// of its own uncertainty, which rests on force_rate. Told 5 N/s while the
// hand's push changes faster, the filter misses the device's displacement
// by about fifteen of its standard deviations, told 20 N/s by about four,
// and a horizon of four samples on such predictions let the actuators
// generate up to 7 mJ at 50 ms. With adapt_force_rate the filter tunes its
// random walk to its innovations, the covariance matching of R. K. Mehra,
// "On the identification of variances and adaptive Kalman filtering",
// IEEE T-AC 15(2), 1970: a consistent filter's normalized innovation
// squared nu^2 / S averages 1, so the log of the walk's variance moves by
// 3/s times (min(nu^2 / S, 9) - 1), with the rate kept between force_rate
// and 16 force_rate. Its misses are then about one standard deviation.
// It raises the rate quickly and lowers it slowly: a rate too low lets the
// innovations exceed their spread many times over, while one too high
// barely widens the spread, which the reading error's q^2 / 12 dominates,
// so nu^2 / S stays just under 1. After a push that swung by 3 N at 2 Hz
// the rate stands at 66 N/s; with the device kept oscillating by a spring
// on its readings it is still 38 N/s five seconds later and 9 N/s after
// twenty. Reading errors replaced by white noise of the same variance give
// the same, so it is not their correlation. The slow side is the safe one:
// a rate too high costs transparency, not passivity. Passivity no longer
// rests on the adaptation either: with the model error margin below, the
// rate fixed at 5 N/s generates nothing.
//
// Model error. The model is a rigid mass under a slowly changing push, and
// a real device is not one: its links bend, putting part of its mass behind
// a resonance; it has friction; its mass is known only roughly. Where it
// moves otherwise than predicted, a horizon of several samples spends
// energy on inputs that do not come. In the simulator, before the margin, a
// master with 80 % of its mass beyond a 50 Hz resonance let the
// teleoperator generate 66 J on coarse encoders and 3.9 J on exact ones.
// hold_sd() therefore adds what the model has lately missed over a sample,
// model_error(), grown as n^2 over n samples as an unmodeled force would.
// Its estimate depends on the readings:
//
//  * With a resolution the misses hide among the reading errors, and the
//    margin is what the innovations show beyond the filter's own spread:
//    the running mean of nu^2 - S over 50 ms, its positive part. This is
//    Mehra's covariance matching again, applied to the residual instead of
//    the random walk; a consistent filter keeps it at zero.
//  * With exact readings each miss is known exactly, after its sample, and
//    there is nothing to average out: the margin is the envelope of the
//    squared misses, decaying with a time constant of 50 ms. It covers
//    misses like those already seen, not one larger than any before it.
//
// The tax horizon then serves exact readings too: a rise must repay any
// deficit of the books from the work predicted, less the margin, over
// tax_samples samples. Without that, a deficit the margin came too late to
// prevent stays and the next rise may add to it.
//
// The margin costs stiffness where the model is right but the motion is
// hard to predict. On the default scenario with exact readings the
// displayed stiffness of the reflective scheme fell by 3 to 9 % at 5 and
// 10 ms, 11 % at 25 ms, half at 50 ms and 6 to 22 % at 100 and 200 ms,
// where residual deficits of 0.04 to 0.09 mJ went to zero. A margin kept
// per direction of the force recovered the stiffness but let the worst
// misdescribed device generate eight times as much; a shorter memory (20,
// 10 or 5 ms), or a margin on one side of the prediction only, did not
// recover it.
//
// Rises that cannot last. With a coarse encoder the master's force chatters
// with every constraint met: a count transition shows the hand moving into
// the force, the PC lets the force rise and books its tax, and before the
// next transition, which shows nothing until the hand has moved another
// count, the force has decayed again. Each cycle pays the tax anew, so the
// budget stays at zero. The readings, not the tax, cause it: with exact
// readings and the filter and the tax unchanged, the force's total
// variation at 50 ms is a fifth. With rise_samples = M a rise goes only as
// far as the force could be held over M samples on the expected motion,
// lambda * G0_M - lambda^2 * C_M >= -budget with G0_M the mean prediction,
// which for long M is about the hand's estimated push H: the force the hand
// is known to sustain. The held force may stay, so this declines only rises
// that would have to be taken back. Passivity does not rest on it, so it
// takes the mean; with the lower bound a force could rise only to H less
// its uncertainty, and with M = 64 the master conveyed 11 % of the contact
// force at 50 ms instead of 16 %. A limit on the rate of rise is no
// substitute: at 50 N/s it halved the variation at 50 ms but cut the force
// conveyed from 17 to 11 %.
//
// Measured in the 1-DoF simulator with the tax that assumes nothing
// (count_window 0, see "Crossings"; reflective scheme, 55 um master and
// 1 um slave encoders, 20 s over nine hand motions): with the filter, the tax,
// the model error margin and the ports paying their own excess first
// (control/tdpa.hpp), the actuators never generate energy at 10, 50 or
// 100 ms each way, on the nominal devices and on the eight misdescribed
// ones of tests/test_teleop1d.cpp (half the model's mass, half or 80 % of
// the mass beyond a resonance, Coulomb friction). Controllers that take the
// readings as exact let the teleoperator generate up to 0.5 mJ, 0.8 J and
// 1.1 J before the margin; with it, the quantization shows as model error,
// and they generate 7.5 uJ, once, with the first force, and display
// nothing after. The price of the resolution is transparency: with
// adaptation, confidence 1, the tax repaid over four samples and rises
// held over 64 the master conveys 92, 17 and 10 % of the contact force at
// 10, 50 and 100 ms, against 84, 26 and 12 % with exact encoders, and its
// force varies 0.5, 1.8 and 2.3 times as much. Without the rise hold it
// conveyed 28, 13 and 7 % and its force varied 1.2, 3.5 and 4.0 times as
// much. With the tax prepaid as well it conveyed 32, 12 and 4 %, and
// nothing at all on the three motions too slow to pay it.
//
// Crossings. The tax above assumes nothing about where in its count the
// device lies, and charges every rise as if the reading erred by a whole
// count against it. A sampled device tells more: when the reading changes,
// the device has just crossed the edge between two counts, and lies past it
// by no more than it moved over the sample. With u = (the upper edge of the
// count read) - x, in [0, q], a reading's error is u plus a constant, which
// drops out of the sum by parts, and the work the books miss up to sample
// N is
//
//   phi_N u_{N+1} - sum_{k <= N} u_k (phi_k - phi_{k-1}).
//
// With u_k known to lie in [lo, hi] the tax of the change at sample k is,
// with phi > 0, (q - lo) per newton of rise and a credit of (q - hi) per
// newton of fall (the first term is at most q phi_N throughout); with
// phi < 0, hi per newton by which |phi| rises and a credit of lo per newton
// by which it falls; a change of sign passes through zero. lo = 0 and
// hi = q give the tax above. Just after the reading went up across an
// edge, u lies in [q - D, q]; just after it went down, in [0, D]; D is how
// far the device can have moved past that edge. A device that dithers
// across an edge by d into a spring of stiffness K pays (2 D - q) K q for a
// round trip, a credit where D < q / 2: the spring does -2 d K q of work on
// it, which the tax with D = d books exactly, while the tax above books
// -2 K q^2.
//
// Bounds from the window. Where the device lay at each reading the
// readings tell together, not one by one. Over the last count_window
// readings the device moved from each to the next as a mass under the port
// force and an H that changes by at most force_drift N/s, and each reading
// put it within its count: constraints linear in the state at the window's
// start and the changes of H, so the bounds on u at any reading in the
// window are two linear programs, which BoundedLp solves (the
// bounded-variable primal simplex of G. B. Dantzig, "Linear programming and
// extensions", 1963, with R. G. Bland's rule, "New finite pivoting rules
// for the simplex method", Math. Oper. Res. 2(2), 1977). This is
// set-membership estimation with quantized measurements, as in M. Casini,
// A. Garulli and A. Vicino, "Adaptive threshold selection for set
// membership state estimation with quantized measurements", IEEE T-AC,
// 2025, but over a window, a smoother ("Optimal set-membership smoothing",
// arXiv 2410.09857): later readings narrow the bounds at earlier ones.
// Within a sample H may stray from its mean by the drift, which the model
// leaves out; each bound is widened by the most that moves the device. The
// books then bound the true work from below as far as the device is a mass
// of the mass assumed, pushed by an H that drifts no faster than
// force_drift and does not jump (Coulomb friction's does), no longer for
// every motion. Readings that no such motion explains leave the program
// infeasible, and the window starts again from the next.
//
// Refunds. A force acts from the reading it was commanded on, and the
// readings after it narrow where the device lay at that one. The books
// charge each change at once at the bounds the readings so far give
// (count_bounds()), and the PC chooses the force against that charge, a
// fall's credit included. refund_lag readings later they settle it: at most
// the worst of its line over the bounds then known and, since the u in a
// window are not independent, at most what bounding it jointly with the
// changes settled before it within the window allows (worst_tax(): the
// largest sum of their lines the readings allow, less what those were
// settled at). Over a chatter of the force the joint bound is far below
// the sum of the termwise ones, since the device cannot lie at the worst
// place for every change at once. The books are refunded the difference.
//
// Nothing is known, and the tax is the one above, after an irregular
// sample, on readings that change by a fraction of a count or are noisier
// than their quantization (set_reading_noise: a variance above q^2 / 12,
// as on a magnetic encoder with half a count of noise), for a force that
// takes effect only after the next reading (a lead of a sample or more),
// and for a second command on the same reading, which acts from later than
// its bounds hold.
//
// Measured in the 1-DoF simulator (model-mediated scheme, 100 ms each way,
// 55 um encoders on both sides, 20 s over twelve hand motions, no rise
// hold): with exact encoders the master's force breaks a contact into 1.01
// per hand cycle on average, varies by 22.9 N/s and falls short of the
// rendered force by 0.25 N RMS. With the tax above it breaks it into 3.59,
// varies by 125 N/s and falls short by 5.7 N; with the window's bounds and
// refunds settled term by term, 1.56, 37 N/s and 0.68 N; settled jointly,
// 1.59, 36 N/s and 0.54 N; with the PC counting on a fall's credit as well,
// 1.15, 35 N/s and 0.58 N. Taxed at the true u, which no controller knows,
// the master gives 1.28, 29 N/s and 0.26 N: what is left is the width of
// the bounds near rest, not the use made of them. On the same runs the
// joint settlement overcharged the true tax by 3.0 mJ, against 6.3 mJ term
// by term, and never undercharged it.
//
// Sign convention: power into the controlled two-port is phi * dx, and the
// device accelerates by -phi / m. At a haptic master phi is the displayed
// force when it pushes against the positive direction; at a slave robot phi
// is minus the force the controller applies.
// A port force held for `samples` samples, which need not be whole: a piece
// of a force that changed within a sample, as when the device applies
// commands some time after they are issued.
struct ForceSegment {
  double phi = 0.0;
  double samples = 0.0;
};

// The quantization tax of a change of the port force as a function of u,
// where the device lies in its count (see "Crossings"): alpha + beta * u.
// worst() is its largest value for u in [lo, hi]; with lo = 0 and hi = q
// that is quantization_tax(). Negative for a refund.
struct TaxLine {
  double alpha = 0.0, beta = 0.0;
  double worst(double lo, double hi) const { return alpha + beta * (beta > 0.0 ? hi : lo); }
};

class DevicePredictor {
 public:
  // `resolution` is the encoder's resolution in m, 0 for an exact encoder
  // (the three-point recursion). `force_rate` is how fast the forces other
  // than the port force are expected to change, in N/s; it only matters
  // with a resolution, as the random walk of the Kalman filter's H. With
  // `adapt_force_rate` it is the least rate, and the filter raises it up to
  // 16 times as far as its innovations show it to be overconfident.
  DevicePredictor(double mass, double dt, double resolution = 0.0, double force_rate = 20.0,
                  bool adapt_force_rate = false);

  // Call once per sample with the displacement realized over the last sample,
  // the port force that was held throughout it, and its length h in seconds
  // (dt if omitted). A sample further than dt / 2 from dt is irregular.
  void observe(double dx, double phi_held) { observe(dx, phi_held, dt_); }
  void observe(double dx, double phi_held, double h);
  // A sample during which the port force changed: `segments` in order, their
  // lengths adding up to h / dt. The model is still exact for a mass.
  void observe(double dx, const std::vector<ForceSegment>& segments, double h);
  // A sample during which the port force was not held throughout, e.g. the
  // device's watchdog cut it.
  void observe_irregular(double dx, double phi_held);
  // The work the segments' forces did over a sample in which the encoder saw
  // the displacement dx, to be called before that sample is observed: each
  // segment but the last over the displacement the model predicts for it
  // from the state at the sample's start, the last over the rest. One
  // segment did phi * dx.
  double realized_work(double dx, const std::vector<ForceSegment>& segments) const;
  // The mean velocity over a coming sample of length h (s), in which the
  // port force is `segments`, less that over the last sample, as the model
  // expects it from its estimate of H (m/s): how far the forces it knows
  // of bend the readings' second difference (ReadingNoise).
  double expected_velocity_change(const std::vector<ForceSegment>& segments, double h) const;
  // True when the last two samples were regular. A new predictor is ready:
  // its history is that of a device at rest.
  bool ready() const { return regular_ >= 2; }

  double resolution() const { return q_; }
  // The variance of a reading's error the Kalman filter assumes, m^2: q^2 /
  // 12, or with set_reading_noise what an estimate (ReadingNoise) says, if
  // larger. Only used with a resolution.
  double reading_noise() const { return noise_; }
  void set_reading_noise(double variance) { noise_ = std::max(q_ * q_ / 12.0, variance); }
  // The rate the filter's random walk currently assumes: force_rate, or
  // with adapt_force_rate what the innovations have raised it to.
  double force_rate() const;
  // Forces already commanded that take effect after the last sample and
  // before the next one commanded: on a device that applies its commands
  // some time after they are issued, the forces in flight. The predictions
  // below are then for the samples that follow them. Observing a sample
  // clears the lead.
  void set_lead(const std::vector<ForceSegment>& lead);
  double lead_samples() const { return lead_samples_; }
  // The work the lead is predicted to do (J), and its standard deviation.
  double lead_work() const;
  double lead_sd() const;

  // Where the device lies in the count its encoder reads, as bounds on
  // u = (the count's upper edge) - x (m, see "Crossings"), [0, q] when
  // nothing is known: count_bounds_at(lag) at the reading `lag` readings
  // before the last, from all the readings in the window, those after it
  // included; count_bounds() at the last reading, for a force that takes
  // effect before the next one (nothing is known for a lead of a sample or
  // more).
  struct CountBounds {
    double lo = 0.0, hi = 0.0;
  };
  CountBounds count_bounds() const;
  CountBounds count_bounds_at(std::size_t lag) const;
  // The largest sum of taxes, each `line` at u at the reading `lag`
  // readings before the last, that the readings in the window allow: the
  // u are not independent, since the device moved from one to the next as
  // the model says. +infinity when nothing is known of some u.
  struct LaggedTax {
    std::size_t lag;
    TaxLine line;
  };
  double worst_tax(const std::vector<LaggedTax>& taxes) const;
  // How many readings the window holds now.
  std::size_t count_window_readings() const { return window_.size(); }
  // The bounds rest on the last `readings` readings, and on the forces other
  // than the port force changing by at most `force_drift` N/s.
  void set_count_window(std::size_t readings, double force_drift);

  // The port force held before the next one: the last of the lead, or the
  // force observed at the end of the last sample, or 0 after
  // observe_irregular(), whose force was cut. The quantization tax of the
  // next force is measured from it.
  double held_force() const { return lead_.empty() ? held_ : lead_.back().phi; }

  // Coefficients of the predicted displacement dx = base() - compliance() * phi.
  double base() const;
  double compliance() const { return c_; }
  double displacement(double phi) const { return base() - c_ * phi; }
  double work(double phi) const { return phi * displacement(phi); }

  // The same over n samples with phi held: D_n = hold_base(n) - n^2 c phi.
  // hold_base(1) equals base(). n need not be an integer: the model is a
  // mass under a constant net force, so it predicts any horizon. With a
  // lead, the n samples start where it ends.
  double hold_base(double n) const;
  double hold_compliance(double n) const { return n * n * c_; }
  double hold_displacement(double phi, double n) const {
    return hold_base(n) - hold_compliance(n) * phi;
  }
  double hold_work(double phi, double n) const { return phi * hold_displacement(phi, n); }
  // Standard deviation of hold_displacement(): with a resolution, from the
  // Kalman filter's uncertainty about v and H; either way, from the model
  // error, n^2 model_error(), or (R + n)^2 - R^2 times it after a lead of R
  // samples.
  double hold_sd(double n) const;
  // How far the model has lately missed the displacement over a sample,
  // beyond what the reading error explains, in m (see "Model error").
  double model_error() const { return std::sqrt(miss_); }

  // Estimates recovered from the history (or the Kalman filter, with a
  // resolution): everything acting on the device except the port force (the
  // operator's hand, gravity, friction) in N, and the velocity at the end of
  // the last sample times dt.
  double external_force() const;
  double velocity_dt() const;

 private:
  // A sample's port force by what a mass feels of it, as two held forces:
  // phi_d, which moves it as far over the sample, and phi_v, which changes
  // its velocity as much. Both are phi for a held force. For segments
  // phi_i over r_i samples, the one starting s_i samples into a sample of
  // r: phi_v r = sum phi_i r_i, and phi_d r^2 = sum phi_i r_i (2 (r - s_i) - r_i).
  void push(double dx, double phi_d, double phi_v, double r, bool regular);
  // phi_d and phi_v of `segments` over a sample of r (their lengths are
  // scaled to it).
  static void held_forces(const std::vector<ForceSegment>& segments, double r, double& phi_d,
                          double& phi_v);
  void filter(double dx, double phi_d, double phi_v, double r, bool regular);
  // Keeps a sample for count_bounds_at(), or forgets them all after one
  // the model does not describe or that moved by a fraction of a count.
  void remember(double dx, double phi_d, double phi_v, double r, bool regular);
  // Builds the linear program over the window and finds a feasible point;
  // forgets the window if there is none. True if there is one.
  bool solve_window() const;
  // How far the device may lie from where the model puts it at the reading
  // `lag` before the last, in counts: what H's drift within the samples
  // until then can add up to.
  double model_slack(std::size_t lag) const;
  // Standard deviation of a quantity whose error is gv times that of the
  // velocity, k c times that of H, and k times the model error.
  double spread(double gv, double k) const;

  double dt_;
  double mass_;
  double c_;
  double dx1_ = 0.0, dx2_ = 0.0;      // last two realized displacements
  double phid1_ = 0.0, phid2_ = 0.0;  // the port force during them (push())
  double phiv1_ = 0.0, phiv2_ = 0.0;
  double r1_ = 1.0, r2_ = 1.0;        // their lengths in units of dt
  int regular_ = 2;                   // regular samples since an irregular one, up to 2
  double held_ = 0.0;
  std::vector<ForceSegment> lead_;
  double lead_samples_ = 0.0;
  double lead_imp_ = 0.0;  // sum phi_i r_i over the lead
  // Kalman filter, used when q_ > 0. The position is kept relative to the
  // last reading (e_ = estimate - reading), so that it never grows large.
  double q_;
  double noise_;     // variance of a reading, at least q^2 / 12
  double h_walk_;    // variance of the change of H over a nominal sample
  double force_rate_;
  bool adapt_;
  double log_walk_ = 0.0;  // log of the factor adaptation applies to h_walk_
  double e_ = 0.0, v_ = 0.0, h_ = 0.0;
  double p_[3][3] = {};
  // Model error, in m^2: with exact readings the decaying envelope of the
  // squared miss; with a resolution the positive part of excess_, the
  // running mean of nu^2 - S.
  double miss_ = 0.0;
  double excess_ = 0.0;
  // The readings count_bounds_at() rests on, oldest first: the counts each
  // moved by and the sample's port force and length, as push() takes them.
  struct Reading {
    double counts, phi_d, phi_v, r;
  };
  // A window that turns out infeasible is forgotten when found so.
  mutable std::deque<Reading> window_;
  std::size_t window_size_;
  double force_drift_;
  // The window's linear program, built on demand, and the bounds found,
  // per lag (lo > hi where not yet found).
  mutable std::optional<BoundedLp> lp_;
  mutable bool solved_ = false;
  mutable std::vector<CountBounds> found_;
};

// Energy (J) to book as an output when the port force goes from `prev`, the
// force held until now, to `phi`, on an encoder of resolution `q` (m):
// q * (|phi| - |prev|) when the force grows in the same direction, q * |phi|
// when it changes sign or starts from zero, and 0 when it shrinks. Booked
// for every new force, it bounds the work the readings misattribute (see
// DevicePredictor): with phi_{-1} = 0 the sum over samples 0..N-1 is
// (q / 2) (sum_k |phi_k - phi_{k-1}| + |phi_{N-1}|), the largest
// |sum_k phi_k (e_{k+1} - e_k)| can be when every reading error lies in one
// interval of width q.
double quantization_tax(double q, double prev, double phi);
// The same as a function of u, where the device lies in its count (see
// "Crossings"), as TaxLine.
TaxLine quantization_tax_line(double q, double prev, double phi);
// Its worst value for u known to lie in [lo, hi] within [0, q].
double quantization_tax(double q, double prev, double phi, double lo, double hi);
// The tax of moving `device`'s held force to phi, with its resolution and
// count_bounds().
double quantization_tax(const DevicePredictor& device, double phi);

// Scales `phi` towards zero so that the predicted work over the coming
// sample, and over `hold_samples` samples if that is larger than one, less
// `confidence` standard deviations of it (DevicePredictor::hold_sd), stays
// above -budget (budget in J; negative values count as zero). With an
// encoder resolution the quantization tax of moving from the held force to
// the result counts as well, as quantization_tax(device, result) charges
// it: a rise's tax and a fall's credit at the device's count_bounds()
// (later refunds are booked, not counted on). The credit counts against
// each of those predictions; the tax does when `tax_samples` is 0, and
// otherwise only for a rise beyond the held force. With `tax_samples` > 0,
// with or without a resolution, such a rise must repay its tax and any
// deficit (a negative budget) from the work predicted over `tax_samples`
// samples, less `confidence` standard deviations. With a resolution and
// `rise_samples` > 0 such a rise also goes no further than the expected
// work over `rise_samples` samples can pay for. The result's gain is the
// fraction of the requested force that was removed, 1 - lambda. While the
// device is not ready() the result is zero force.
PcResult limit_force_predictive(double budget, double phi, const DevicePredictor& device,
                                int hold_samples = 2, double confidence = 0.0,
                                int tax_samples = 0, int rise_samples = 0);

}  // namespace tuushin::control
