// Phase 3: rog_planner — B-spline + L-BFGS smoother. See header.
#include <rog_planner/bspline_smoother.h>

#include <chrono>
#include <deque>
#include <cmath>
#include <functional>
#include <ros/console.h>

namespace rog_planner {

// -------------------- Cubic uniform B-spline evaluation -------------------
// Standard form: p(t) = (1/6) [u^3 u^2 u 1] M [Q_i Q_{i+1} Q_{i+2} Q_{i+3}]^T
// with M = [[-1, 3,-3, 1],
//           [ 3,-6, 3, 0],
//           [-3, 0, 3, 0],
//           [ 1, 4, 1, 0]]
// where i = floor(t/dt), u = t/dt - i, valid for u in [0,1].
// Velocity: derivative w.r.t. t of the same expression.

static inline void cubicBasis(double u, double b[4]) {
  double u2 = u * u, u3 = u2 * u;
  b[0] = (-u3 + 3*u2 - 3*u + 1) / 6.0;
  b[1] = ( 3*u3 - 6*u2       + 4) / 6.0;
  b[2] = (-3*u3 + 3*u2 + 3*u + 1) / 6.0;
  b[3] = (u3) / 6.0;
}

static inline void cubicBasisDeriv(double u, double dt, double bd[4]) {
  // d/dt of basis = (1/dt) * d/du of basis
  double u2 = u * u;
  bd[0] = (-3*u2 +  6*u - 3) / 6.0 / dt;
  bd[1] = ( 9*u2 - 12*u    ) / 6.0 / dt;
  bd[2] = (-9*u2 +  6*u + 3) / 6.0 / dt;
  bd[3] = ( 3*u2           ) / 6.0 / dt;
}

Eigen::Vector3d BSplineTrajectory::pos(double t) const {
  int N = static_cast<int>(Q.size());
  if (N < order + 1) return Eigen::Vector3d(0,0,z_height);
  double tt = std::max(0.0, std::min(t, tEnd() - 1e-6));
  int i = static_cast<int>(tt / dt);
  if (i > N - order - 1) i = N - order - 1;
  double u = tt / dt - i;
  double b[4]; cubicBasis(u, b);
  Eigen::Vector2d p = b[0]*Q[i] + b[1]*Q[i+1] + b[2]*Q[i+2] + b[3]*Q[i+3];
  return Eigen::Vector3d(p.x(), p.y(), z_height);
}

Eigen::Vector3d BSplineTrajectory::vel(double t) const {
  int N = static_cast<int>(Q.size());
  if (N < order + 1) return Eigen::Vector3d::Zero();
  double tt = std::max(0.0, std::min(t, tEnd() - 1e-6));
  int i = static_cast<int>(tt / dt);
  if (i > N - order - 1) i = N - order - 1;
  double u = tt / dt - i;
  double bd[4]; cubicBasisDeriv(u, dt, bd);
  Eigen::Vector2d v = bd[0]*Q[i] + bd[1]*Q[i+1] + bd[2]*Q[i+2] + bd[3]*Q[i+3];
  return Eigen::Vector3d(v.x(), v.y(), 0.0);
}

// -------------------- Cost and analytic gradient ----------------------------
// x layout: [Qx_0, Qy_0, Qx_1, Qy_1, ..., Qx_{N-1}, Qy_{N-1}]
double BSplineSmoother::evalCostAndGrad(const Eigen::VectorXd& x,
                                        Eigen::VectorXd& grad, int N) {
  grad.setZero(2 * N);
  double J = 0.0;

  // -------- Smoothness: sum_i ||Q_{i+1} - 2*Q_i + Q_{i-1}||^2 ----------
  // Gradient w.r.t. Q_{i-1}: +2*w*dx ; Q_i: -4*w*dx ; Q_{i+1}: +2*w*dx
  for (int i = 1; i <= N - 2; ++i) {
    double dx = x(2*(i+1)+0) - 2*x(2*i+0) + x(2*(i-1)+0);
    double dy = x(2*(i+1)+1) - 2*x(2*i+1) + x(2*(i-1)+1);
    J += p_.w_smooth * (dx*dx + dy*dy);
    grad(2*(i-1)+0) += 2 * p_.w_smooth * dx;
    grad(2*(i  )+0) += -4 * p_.w_smooth * dx;
    grad(2*(i+1)+0) += 2 * p_.w_smooth * dx;
    grad(2*(i-1)+1) += 2 * p_.w_smooth * dy;
    grad(2*(i  )+1) += -4 * p_.w_smooth * dy;
    grad(2*(i+1)+1) += 2 * p_.w_smooth * dy;
  }

  // -------- Sample-based costs (obs, dyn) ------------------------------
  double tEnd = std::max(0.0, (N - p_.order) * p_.dt);
  int nSamples = std::max(2, static_cast<int>(tEnd / p_.sample_dt));
  for (int s = 0; s < nSamples; ++s) {
    double t = s * p_.sample_dt;
    if (t >= tEnd - 1e-9) break;
    int i = static_cast<int>(t / p_.dt);
    if (i > N - p_.order - 1) i = N - p_.order - 1;
    double u = t / p_.dt - i;
    double b[4]; cubicBasis(u, b);
    double bd[4]; cubicBasisDeriv(u, p_.dt, bd);

    Eigen::Vector2d ps =
        b[0]*Eigen::Vector2d(x(2*(i  )), x(2*(i  )+1)) +
        b[1]*Eigen::Vector2d(x(2*(i+1)), x(2*(i+1)+1)) +
        b[2]*Eigen::Vector2d(x(2*(i+2)), x(2*(i+2)+1)) +
        b[3]*Eigen::Vector2d(x(2*(i+3)), x(2*(i+3)+1));
    Eigen::Vector2d vs =
        bd[0]*Eigen::Vector2d(x(2*(i  )), x(2*(i  )+1)) +
        bd[1]*Eigen::Vector2d(x(2*(i+1)), x(2*(i+1)+1)) +
        bd[2]*Eigen::Vector2d(x(2*(i+2)), x(2*(i+2)+1)) +
        bd[3]*Eigen::Vector2d(x(2*(i+3)), x(2*(i+3)+1));

    // ----- ESDF obstacle cost -----
    Eigen::Vector3d p3(ps.x(), ps.y(), p_.chassis_height);
    double d = 0.0;
    Eigen::Vector3d g3 = Eigen::Vector3d::Zero();
    try {
      map_->distAndGrad(p3, d, g3);
    } catch (...) { d = 0.0; }
    if (d < p_.safe_dist) {
      double slack = p_.safe_dist - d;
      J += p_.w_obs * slack * slack;
      // d/dQ_{i+k} of slack^2 = -2*slack * dDist/dp * basis_k
      for (int k = 0; k < 4; ++k) {
        grad(2*(i+k)+0) += -2.0 * p_.w_obs * slack * g3.x() * b[k];
        grad(2*(i+k)+1) += -2.0 * p_.w_obs * slack * g3.y() * b[k];
      }
    }

    // ----- Static PGM/OccupancyGrid wall barrier cost -----
    if (static_map_) {
      Eigen::Vector2d g2 = Eigen::Vector2d::Zero();
      double d2 = static_map_->clearance(ps, p_.static_safe_dist, &g2);
      if (d2 < p_.static_safe_dist) {
        double slack = p_.static_safe_dist - d2;
        J += p_.w_static_obs * slack * slack;
        for (int k = 0; k < 4; ++k) {
          grad(2*(i+k)+0) += -2.0 * p_.w_static_obs * slack * g2.x() * b[k];
          grad(2*(i+k)+1) += -2.0 * p_.w_static_obs * slack * g2.y() * b[k];
        }
      }
    }

    // ----- Dynamics: velocity penalty -----
    double vmag = vs.norm();
    if (vmag > p_.v_max && vmag > 1e-9) {
      double slack = vmag - p_.v_max;
      J += p_.w_dyn * slack * slack;
      Eigen::Vector2d dv_dQk = vs / vmag;  // d|v|/dv
      for (int k = 0; k < 4; ++k) {
        grad(2*(i+k)+0) += 2.0 * p_.w_dyn * slack * dv_dQk.x() * bd[k];
        grad(2*(i+k)+1) += 2.0 * p_.w_dyn * slack * dv_dQk.y() * bd[k];
      }
    }
  }

  return J;
}

// -------------------- L-BFGS (m=8) with backtracking ------------------------
static bool lbfgs_optimize(int N, Eigen::VectorXd& x,
                           std::function<double(const Eigen::VectorXd&,
                                                Eigen::VectorXd&)> eval,
                           int max_iter, double max_time_s,
                           double tol_g = 1e-3) {
  const int m = 8;
  std::deque<Eigen::VectorXd> s_hist, y_hist;
  std::deque<double>          rho_hist;
  Eigen::VectorXd grad(x.size()), grad_new(x.size()), q(x.size()), z(x.size());
  Eigen::VectorXd x_new(x.size()), s(x.size()), y(x.size());

  double f = eval(x, grad);
  auto t0 = std::chrono::steady_clock::now();

  for (int it = 0; it < max_iter; ++it) {
    if (grad.norm() < tol_g) return true;

    // Two-loop recursion: q = grad
    q = grad;
    int K = static_cast<int>(s_hist.size());
    std::vector<double> alpha(K);
    for (int i = K - 1; i >= 0; --i) {
      alpha[i] = rho_hist[i] * s_hist[i].dot(q);
      q -= alpha[i] * y_hist[i];
    }
    // Initial Hessian approx: gamma * I
    double gamma = 1.0;
    if (K > 0) {
      gamma = s_hist.back().dot(y_hist.back()) /
              std::max(1e-12, y_hist.back().squaredNorm());
    }
    z = gamma * q;
    for (int i = 0; i < K; ++i) {
      double beta = rho_hist[i] * y_hist[i].dot(z);
      z += s_hist[i] * (alpha[i] - beta);
    }
    // Search direction: d = -z
    Eigen::VectorXd d = -z;
    if (d.dot(grad) > 0) d = -grad;  // safeguard

    // Backtracking line search (Armijo, c1=1e-4)
    double step  = (it == 0) ? std::min(1.0, 1.0 / grad.norm()) : 1.0;
    double f_new = f;
    bool   ok    = false;
    for (int ls = 0; ls < 20; ++ls) {
      x_new = x + step * d;
      f_new = eval(x_new, grad_new);
      if (f_new <= f + 1e-4 * step * grad.dot(d)) { ok = true; break; }
      step *= 0.5;
    }
    if (!ok) return false;

    s = x_new - x;
    y = grad_new - grad;
    double sy = s.dot(y);
    if (sy > 1e-10) {
      if ((int)s_hist.size() == m) {
        s_hist.pop_front(); y_hist.pop_front(); rho_hist.pop_front();
      }
      s_hist.push_back(s);
      y_hist.push_back(y);
      rho_hist.push_back(1.0 / sy);
    }
    x = x_new; grad = grad_new; f = f_new;

    auto dt = std::chrono::duration<double>(
                  std::chrono::steady_clock::now() - t0).count();
    if (dt > max_time_s) return true;
  }
  return true;
}

// -------------------- Line-of-sight path pruning ---------------------------
// Greedy simplification of the A* front-end output. The 8-connected A* on
// a 0.1 m grid produces alternating diagonal/cardinal moves which, when
// arc-length-sampled into B-spline control points, fight the smoothness
// term and produce visible zigzag in /trajectory. We collapse runs of
// LOS-feasible waypoints into straight segments; the optimizer then pulls
// those segments taut against ESDF and static_map clearances.
//
// LOS check is ESDF-based (not isLineFree-on-inflated-occ): we sample the
// candidate segment at 0.05 m and require ESDF >= shortcut_clear at every
// sample, plus the static map check. ESDF-based is more permissive for
// diagonal corner shortcuts than isLineFree against the 1-cell halo, which
// is exactly the staircase pattern that produces the zigzag.
static std::vector<Eigen::Vector3d> prunePath(
    const std::vector<Eigen::Vector3d>& in,
    SentryMap* map,
    StaticMap2D* static_map,
    double chassis_h,
    double shortcut_clear,
    double static_clear) {
  if (in.size() <= 2) return in;

  auto segmentOk = [&](const Eigen::Vector3d& p0,
                       const Eigen::Vector3d& p1) -> bool {
    Eigen::Vector3d d = p1 - p0;
    double L = d.norm();
    int steps = std::max(2, static_cast<int>(std::ceil(L / 0.05)));
    for (int s = 0; s <= steps; ++s) {
      double t = static_cast<double>(s) / steps;
      Eigen::Vector3d ps = p0 + t * d;
      ps.z() = chassis_h;
      double esdf_d = 0.0;
      try { esdf_d = map->dist(ps); } catch (...) { esdf_d = 0.0; }
      if (esdf_d < shortcut_clear) return false;
      if (static_map &&
          static_map->isOccupied(Eigen::Vector2d(ps.x(), ps.y()))) {
        return false;
      }
      // additional static_clear check via clearance() not done here; the
      // smoother's static-map cost will pull the optimum away from walls.
      (void)static_clear;
    }
    return true;
  };

  // Iterate the greedy shortcut to fixed point so chains of small staircase
  // segments collapse over multiple passes.
  std::vector<Eigen::Vector3d> cur = in;
  for (int pass = 0; pass < 4; ++pass) {
    std::vector<Eigen::Vector3d> out;
    out.reserve(cur.size());
    out.push_back(cur.front());
    size_t anchor = 0;
    while (anchor + 1 < cur.size()) {
      size_t best = anchor + 1;
      for (size_t j = cur.size() - 1; j > anchor + 1; --j) {
        if (segmentOk(cur[anchor], cur[j])) { best = j; break; }
      }
      out.push_back(cur[best]);
      anchor = best;
    }
    if (out.size() == cur.size()) { cur.swap(out); break; }
    cur.swap(out);
  }
  return cur;
}

// -------------------- Public smooth() API ----------------------------------
bool BSplineSmoother::smooth(const std::vector<Eigen::Vector3d>& front_path_in,
                             BSplineTrajectory& out) {
  if (front_path_in.size() < 2) return false;

  // Prune A* output before B-spline initialization. This drastically reduces
  // the number of control points and removes the 8-connected staircase that
  // otherwise locks the smoother into a zigzag attractor.
  // shortcut_clear is the ESDF threshold for accepting a straight shortcut;
  // we set it slightly below safe_dist so the prune is permissive (the
  // smoother's obstacle term still pulls the optimum back to safe_dist).
  const double shortcut_clear = std::max(0.05, p_.safe_dist * 0.5);
  std::vector<Eigen::Vector3d> front_path =
      prunePath(front_path_in, map_, static_map_,
                p_.chassis_height, shortcut_clear, p_.static_safe_dist);

  // Initialize control points by uniformly sampling the front-end path.
  // Need >= order+1 control points; aim for ~one CP per dt of motion at vmax.
  double path_len = 0.0;
  for (size_t i = 1; i < front_path.size(); ++i) {
    path_len += (front_path[i] - front_path[i-1]).norm();
  }
  double T_est = path_len / std::max(0.1, p_.v_max * 0.6);  // assume ~60% vmax avg
  int    nSeg  = std::max(3, static_cast<int>(std::ceil(T_est / p_.dt)));
  int    N     = nSeg + p_.order;  // N control points
  if (N > 100) N = 100;

  std::vector<Eigen::Vector2d> Q(N);
  for (int k = 0; k < N; ++k) {
    double s = (N <= 1) ? 0.0
                        : static_cast<double>(k) / (N - 1) * path_len;
    // walk along front_path to arc-length s
    double acc = 0.0;
    Eigen::Vector3d p = front_path.front();
    for (size_t i = 1; i < front_path.size(); ++i) {
      double seg = (front_path[i] - front_path[i-1]).norm();
      if (acc + seg >= s) {
        double t = (seg < 1e-9) ? 0.0 : (s - acc) / seg;
        p = front_path[i-1] + t * (front_path[i] - front_path[i-1]);
        break;
      }
      acc += seg;
      p = front_path[i];
    }
    Q[k] = Eigen::Vector2d(p.x(), p.y());
  }

  // Pack into x.
  Eigen::VectorXd x(2 * N);
  for (int k = 0; k < N; ++k) { x(2*k) = Q[k].x(); x(2*k+1) = Q[k].y(); }

  // Hold ESDF mutex across the entire optimize() call (plan §back-end thread-safety).
  std::lock_guard<std::mutex> lck(map_->getEsdfMutex());

  auto eval_fn = [&](const Eigen::VectorXd& xx, Eigen::VectorXd& gg) {
    Eigen::VectorXd xv = xx;
    if (p_.pin_endpoints) {
      // Re-pin endpoints by overwriting the first/last two entries.
      xv(0) = Q.front().x(); xv(1) = Q.front().y();
      xv(2) = Q.front().x(); xv(3) = Q.front().y();
      xv(2*(N-1))   = Q.back().x();  xv(2*(N-1)+1) = Q.back().y();
      xv(2*(N-2))   = Q.back().x();  xv(2*(N-2)+1) = Q.back().y();
    }
    double f = evalCostAndGrad(xv, gg, N);
    if (p_.pin_endpoints) {
      gg(0) = gg(1) = 0.0;
      gg(2) = gg(3) = 0.0;
      gg(2*(N-1))   = gg(2*(N-1)+1) = 0.0;
      gg(2*(N-2))   = gg(2*(N-2)+1) = 0.0;
    }
    return f;
  };

  bool ok = lbfgs_optimize(N, x, eval_fn, p_.max_iter, p_.max_time_s);
  Eigen::VectorXd grad_dummy(2*N);
  last_cost_ = evalCostAndGrad(x, grad_dummy, N);

  out.Q.resize(N);
  for (int k = 0; k < N; ++k) {
    out.Q[k] = Eigen::Vector2d(x(2*k), x(2*k+1));
  }
  out.dt       = p_.dt;
  out.order    = p_.order;
  out.z_height = p_.chassis_height;

  if (static_map_) {
    for (double t = 0.0; t < out.tEnd(); t += p_.sample_dt) {
      Eigen::Vector3d p = out.pos(t);
      if (static_map_->isOccupied(Eigen::Vector2d(p.x(), p.y()))) {
        ROS_WARN("[rog_planner/smoother] trajectory intersects static map wall");
        return false;
      }
    }
  }

  if (!ok) {
    ROS_WARN("[rog_planner/lbfgs] line search failed; emitting best iterate");
  }
  return true;
}

}  // namespace rog_planner
