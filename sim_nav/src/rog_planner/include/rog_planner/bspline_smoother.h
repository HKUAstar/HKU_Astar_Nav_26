// Phase 3: rog_planner — back-end B-spline + L-BFGS smoother.
//
// Trajectory: uniform cubic B-spline (order k=3) with uniform knot interval
// dt. Control points are Q_0..Q_{N-1} in R^2 (xy only; z is constant at
// chassis height). Curve segment i is defined for t in [i*dt, (i+1)*dt]
// using control points Q_i .. Q_{i+3}. Standard de Boor matrix form.
//
// Cost: J = w_s * J_smooth + w_o * J_obs + w_d * J_dyn
//   J_smooth = sum_i ||Q_{i+1} - 2 Q_i + Q_{i-1}||^2     (jerk approximation)
//   J_obs    = sum_t max(0, d_safe - ESDF(p(t)))^2       (ESDF gradient via SentryMap)
//   J_dyn    = sum_t max(0, |v(t)| - v_max)^2
//             + sum_t max(0, |a(t)| - a_max)^2
//
// Optimizer: hand-rolled limited-memory BFGS (m=8) with backtracking
// line search (Armijo). Fixed iter cap (50) and wall-time cap (200 ms).
// On non-convergence: returns the best iterate found so far.
//
// License: MIT — pure original implementation, no GPL dependency.

#pragma once

#include <vector>
#include <Eigen/Core>
#include <rog_planner/sentry_map.h>
#include <rog_planner/static_map_2d.h>

namespace rog_planner {

struct BSplineParams {
  int    order        = 3;
  double dt           = 0.4;       // knot spacing (s)
  double v_max        = 2.5;
  double a_max        = 4.0;
  double safe_dist    = 0.4;
  double static_safe_dist = 0.25;
  double w_smooth     = 10.0;
  double w_obs        = 10000.0;
  double w_static_obs = 50000.0;
  double w_dyn        = 100.0;
  double sample_dt    = 0.05;      // cost-eval sample step (s)
  int    max_iter     = 50;
  double max_time_s   = 0.20;
  double chassis_height = 0.3;
  // First two and last two control points are "endpoint-fixed" so that
  // start position and (approx) start/end velocity are preserved.
  bool   pin_endpoints = true;
};

struct BSplineTrajectory {
  // xy control points; size N >= order+1.
  std::vector<Eigen::Vector2d> Q;
  double dt;
  int    order;
  double z_height;

  // Total trajectory time span (excludes the order knots at each end).
  double tEnd() const {
    int N = static_cast<int>(Q.size());
    return std::max(0.0, (N - order) * dt);
  }

  // Position p(t), velocity v(t). t in [0, tEnd()].
  Eigen::Vector3d pos(double t) const;
  Eigen::Vector3d vel(double t) const;
};

class BSplineSmoother {
public:
  BSplineSmoother(SentryMap* map, const BSplineParams& p,
                  StaticMap2D* static_map = nullptr)
      : map_(map), static_map_(static_map), p_(p) {}

  // Initialize from front-end waypoints (xy plane) and run L-BFGS.
  // Returns true if optimizer converged or hit iter/time cap with valid
  // (collision-free at samples) trajectory; returns the best iterate.
  bool smooth(const std::vector<Eigen::Vector3d>& front_path,
              BSplineTrajectory& out);

  double lastCost() const { return last_cost_; }

private:
  SentryMap*    map_;
  StaticMap2D*  static_map_;
  BSplineParams p_;
  double last_cost_ = 0.0;

  // Cost + analytic gradient w.r.t. control points.
  // x is a flat vector of (Qx_0, Qy_0, Qx_1, Qy_1, ...).
  double evalCostAndGrad(const Eigen::VectorXd& x, Eigen::VectorXd& grad,
                         int N);
};

}  // namespace rog_planner
