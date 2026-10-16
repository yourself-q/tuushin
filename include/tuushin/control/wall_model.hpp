#pragma once

#include <cstddef>
#include <deque>

namespace tuushin::control {

// Model-mediated teleoperation (MMT): the slave sends a model of what it
// touches instead of the force it feels, and the master renders the model
// itself, so that no delay is inside the operator's haptic loop. Hannaford
// proposed it (IEEE T-RA 5(4), 1989); P. Mitra and G. Niemeyer,
// "Model-mediated telemanipulation", IJRR 27(2), 2008, made the slave
// execute only commands consistent with the model; X. Xu, B. Cizmeci,
// A. Al-Nuaimi and E. Steinbach (IEEE TIM 63(11), 2014) estimate a spring
// from the slave's last 100 samples every millisecond. With the delay out
// of the loop a stiff object is not displayed softer the longer the delay,
// which energy-based schemes (tdpa.hpp, reflection.hpp) cannot avoid; in
// X. Xu, Q. Liu and E. Steinbach's user study (arXiv 1705.05613, 2017) MMT
// rated better than TDPA from about 50 ms round trip on. Its price is that
// the operator feels the model, not the object: a model that is wrong, or
// late, is displayed as if it were right.
//
// The model here is a wall along +x: from `surface` on, it pushes back with
// stiffness times the penetration plus damping times its rate, and it only
// pushes. That is the simulator's wall (teleop::WallParams); a nonlinear or
// moving object is fitted by it locally and the fit follows it.
struct WallModel {
  double stiffness = 0.0;  // N/m; 0 while nothing has been touched
  double surface = 0.0;    // m
  double damping = 0.0;    // N s/m, while penetrating
  bool known() const { return stiffness > 0.0; }
  // The force with which the wall pushes a point at x moving at v back
  // towards -x (N, at least 0).
  double force(double x, double v) const;
};

// Estimates the wall at the slave from its positions and its force sensor.
//
// Contact. The sensor reads noise in free space, and a model fitted to
// noise is a wall where there is none, which the master then renders and
// the slave's commands are then held behind: with 0.2 N of noise and a
// threshold of 0.05 N the simulated slave was driven 10 cm out of its
// workspace and into the wall at 80 N. Contact therefore begins when
// `debounce` samples in a row read more than `contact_sigmas` standard
// deviations of the noise (and `contact_force`), and ends when as many read
// less than half that. The noise is `force_noise` if known, from a data
// sheet or the tare at rest; else it is estimated from the median absolute
// difference of the last `noise_window` readings, which a smooth contact
// force barely moves (sigma = 1.4826 median|df| / sqrt 2).
//
// Stiffness. In contact, a least-squares line through the last `window`
// samples of the contact force over the position (and the velocity, with
// damping) gives the stiffness as its slope. It is taken only if its
// standard error is within `max_relative_error` of it and the samples span
// `min_span`: at the start of a contact, a few samples a few micrometres
// apart put the slope anywhere (Xu et al.'s estimate overshot at every
// first contact), and noise puts it anywhere at all. Once a stiffness is
// known, a new one replaces it only if it differs by more than
// `stiffness_deadband` of it and by twice its standard error: X. Xu et al.'s
// perception-based model updating (IEEE TIM 63(11), 2014) sends a new model
// only past the Weber fraction of stiffness, which humans discriminate at
// 10 to 20 % at best. Each new stiffness moves the surface the master
// renders, so that its force stays the same (WallRenderer), and a fit
// jittering by a few per cent around a wall whose damping it does not
// model moved it every sample, by a little more one way than the other: in
// simulation the operator felt 4000 of 5000 N/m. The surface takes up what
// the stiffness leaves out.
//
// Surface. With a stiffness, the surface is where the last `surface_window`
// samples say it is (x - (f - b v) / k, averaged), so that a wall that moved
// is found again within a few samples whatever the stiffness fit lags. Out
// of contact, a slave beyond the surface by more than the force that ends a
// contact would push there is evidence that the wall is not there any more:
// the surface moves on to where that force would be read. Without this a
// wall that moved away stays in the model, the master renders it and the
// slave's commands stop at it.
struct WallEstimatorParams {
  double contact_force = 0.05;  // N: the least force taken as contact
  double force_noise = 0.0;     // N, standard deviation; 0: estimate it
  double contact_sigmas = 4.0;
  std::size_t debounce = 3;        // samples
  std::size_t noise_window = 200;  // samples
  std::size_t window = 100;        // samples
  std::size_t surface_window = 10; // samples
  double min_span = 20e-6;         // m
  double max_relative_error = 0.1;
  double stiffness_deadband = 0.1;
  bool estimate_damping = false;
  double max_stiffness = 50000.0;  // N/m; fits beyond it are not trusted
};

class WallEstimator {
 public:
  explicit WallEstimator(const WallEstimatorParams& params = {}) : p_(params) {}

  // One sample of the slave: its position (m), velocity (m/s) and the force
  // the wall pushes it back with (N).
  void observe(double x, double v, double f);
  const WallModel& model() const { return model_; }
  bool in_contact() const { return contact_; }
  // The force noise taken (N, standard deviation).
  double force_noise() const;
  // The force above which contact begins (N).
  double contact_threshold() const;

 private:
  struct Point {
    double x, v, f;
  };
  void fit();
  void track_surface();
  WallEstimatorParams p_;
  std::deque<Point> window_;
  std::deque<double> diffs_;  // |df| for the noise
  double f_last_ = 0.0;
  bool has_last_ = false;
  bool contact_ = false;
  std::size_t run_ = 0;  // samples in a row past the threshold to switch
  WallModel model_;
};

// Renders a WallModel at the master. When a new model arrives while the
// master is inside the wall, its surface is not taken at once: the
// rendered surface is placed so that the force stays what it was, as far
// as the reserve below pays for a softer spring storing more. Placed at
// the new model's surface, a wall the master learns of a round trip after
// it reached it would push with the stiffness times how far the master went
// on meanwhile, 6 cm/s for 0.2 s into 5000 N/m being 60 N: the jump of a
// model update, which Xu, Schuwerk and Steinbach ("Passivity-based model
// updating for model-mediated teleoperation", ICMEW 2015) dissipate with an
// adaptive damper instead.
//
// In contact the rendered surface then moves to the model's. Moving it
// towards the master, so that it penetrates less, releases energy the wall
// stores, and is always allowed, at `relax_rate`. Moving it back, so that
// it penetrates more, stores energy the operator did not put in; it is
// allowed at `active_relax_rate` as far as a reserve pays for it, into
// which every release goes, as does what a new stiffness releases and what
// the model's damping dissipates. As TDPA's drift compensation spends what
// its passivity controller dissipated (C. Artigas, J.-H. Ryu and
// C. Preusche, IROS 2010), the rendering then never gives back more than it
// took. Allowed only one way, a surface that the model's noise moved back
// and forth would end up where it went furthest towards the master, and
// the wall would feel softer: 4000 of 5000 N/m in simulation. Where the
// reserve cannot pay, as at a first contact the master went through before
// the model came, the surface stays until the master leaves, and then
// follows it back to the model's without touching it. The master's
// passivity controller keeps the device port passive whatever the
// rendering does; the rendering only decides how it feels, and how often
// that controller has to step in.
//
// Where that controller displays less than the rendering asked for,
// yield() moves the surface in until the wall gives what was displayed.
// The cut is then not a force the wall lags behind, which the next request,
// from the old surface, would try to catch up on: with dithering encoders
// the controller held the force back while the master pushed in and let it
// through while the master came back out, more force out than in, and the
// master held contact in a limit cycle of 1 mm at 33 Hz, 2.1 contacts per
// press at 100 ms each way. Moved, the surface keeps the wall a spring of
// the master's position, as Lee and Huang's passive set-position
// modulation keeps a spring's set position where the energy available
// allows (IEEE T-RO 26(2), 2010). What moving it in releases is what the
// controller dissipated; it does not go into the reserve, which would spend
// it at once moving the surface back out. In simulation, 1.04 contacts per
// press, with the controller active in 0.9 % of samples rather than 21.5 %.
//
// offset() is how far the rendered surface lies beyond the model's: the
// master's position less the offset is where the slave should be for the
// force the master displays (Mitra and Niemeyer's commands consistent with
// the model).
struct WallRendererParams {
  double relax_rate = 0.02;         // m/s
  double active_relax_rate = 0.02;  // m/s
  // Whether the master calls yield() when its passivity controller cuts the
  // force.
  bool yield_to_display = true;
};

class WallRenderer {
 public:
  explicit WallRenderer(const WallRendererParams& params = {}) : p_(params) {}

  // Renders `m` from now on, with the master at x.
  void update(const WallModel& m, double x);
  // The force to display with the master at x moving at v, positive
  // towards -x, after moving the rendered surface over the sample of dt s.
  double force(double x, double v, double dt);
  // The master displayed `displayed` rather than the force(x, v, dt) just
  // rendered, which was larger: the surface moves in so that the wall gives
  // what was displayed.
  void yield(double x, double v, double displayed);

  const WallModel& model() const { return model_; }
  double surface() const { return surface_; }
  double offset() const { return model_.known() ? surface_ - model_.surface : 0.0; }
  // Energy released and not yet spent moving the surface back or on a
  // softer spring (J).
  double reserve() const { return reserve_; }

 private:
  WallRendererParams p_;
  WallModel model_;
  double surface_ = 0.0;  // rendered
  double reserve_ = 0.0;  // J
};

}  // namespace tuushin::control
