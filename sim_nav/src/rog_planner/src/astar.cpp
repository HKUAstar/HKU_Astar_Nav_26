// Phase 3: rog_planner — A* implementation. See astar.h.
#include <rog_planner/astar.h>

#include <queue>
#include <unordered_map>
#include <cmath>
#include <ros/console.h>

namespace rog_planner {

// Hash a 2-D integer cell index (ix, iy) into a 64-bit key.
static inline uint64_t cellKey(int ix, int iy) {
  return (static_cast<uint64_t>(static_cast<uint32_t>(ix)) << 32) |
          static_cast<uint64_t>(static_cast<uint32_t>(iy));
}

bool AStar3D::snapToFree(const Vec3& from, const Vec3& pos,
                         double radius, Vec3& out) {
  // Walk back along (pos - from) in resolution steps until we find a
  // non-inflated-occupied cell. Used when a goal lies inside an obstacle
  // halo or just outside the local map.
  Vec3 dir = pos - from;
  double L = dir.norm();
  if (L < 1e-6) return false;
  dir /= L;
  int n = static_cast<int>(std::min(L, radius) / p_.resolution);
  for (int i = 0; i <= n; ++i) {
    Vec3 q = pos - dir * (i * p_.resolution);
    q.z()  = p_.chassis_height;
    if (!isBlocked(q)) {
      out = q;
      return true;
    }
  }
  return false;
}

bool AStar3D::isBlocked(const Vec3& p) const {
  if (static_map_ && static_map_->isOccupied(Eigen::Vector2d(p.x(), p.y()))) {
    return true;
  }
  return map_->isOccupiedInflate(p);
}

bool AStar3D::transitionBlocked(const Vec3& from, const Vec3& to,
                                int dx, int dy) const {
  if (isBlocked(to)) return true;

  // Prevent diagonal corner-cutting through two touching occupied cells.
  if (dx != 0 && dy != 0) {
    Vec3 side_x(from.x() + dx * p_.resolution, from.y(), p_.chassis_height);
    Vec3 side_y(from.x(), from.y() + dy * p_.resolution, p_.chassis_height);
    if (isBlocked(side_x) || isBlocked(side_y)) return true;
  }

  const double length = (to - from).head<2>().norm();
  const int samples = std::max(1, static_cast<int>(std::ceil(length / 0.025)));
  for (int i = 1; i <= samples; ++i) {
    double a = static_cast<double>(i) / samples;
    Vec3 p = (1.0 - a) * from + a * to;
    p.z() = p_.chassis_height;
    if (isBlocked(p)) return true;
  }
  return false;
}

bool AStar3D::plan(const Vec3& start_in, const Vec3& goal_in, Path& out_path) {
  out_path.clear();
  last_expansions_ = 0;
  last_snapped_    = false;

  // Hold ESDF writer mutex for the entire search: the clearance penalty
  // and isOccupiedInflate both read shared buffers that the 1 kHz
  // updateESDF3D() / inf_map sliding writer mutates.
  std::lock_guard<std::mutex> elck(map_->getEsdfMutex());

  Vec3 start = start_in;  start.z() = p_.chassis_height;
  Vec3 goal  = goal_in;   goal.z()  = p_.chassis_height;

  // Snap start/goal if blocked.
  if (isBlocked(start)) {
    Vec3 s2;
    if (snapToFree(goal, start, p_.snap_radius, s2)) {
      start = s2; last_snapped_ = true;
    } else {
      ROS_WARN("[rog_planner/astar] start in occupied cell, no nearby free");
      return false;
    }
  }
  if (isBlocked(goal)) {
    Vec3 g2;
    if (snapToFree(start, goal, p_.snap_radius, g2)) {
      goal = g2; last_snapped_ = true;
    } else {
      ROS_WARN("[rog_planner/astar] goal in occupied cell, no nearby free");
      return false;
    }
  }

  const double res = p_.resolution;
  auto toCell = [&](const Vec3& p) {
    return std::make_pair(static_cast<int>(std::round(p.x() / res)),
                          static_cast<int>(std::round(p.y() / res)));
  };
  auto toPos = [&](int ix, int iy) {
    return Vec3(ix * res, iy * res, p_.chassis_height);
  };

  auto sc = toCell(start);  int sx = sc.first, sy = sc.second;
  auto gc = toCell(goal);   int gx = gc.first, gy = gc.second;

  struct Node { double f; int ix, iy; };
  struct Cmp { bool operator()(const Node& a, const Node& b) const {
    return a.f > b.f;
  }};
  std::priority_queue<Node, std::vector<Node>, Cmp> open;
  std::unordered_map<uint64_t, double>   g_score;
  std::unordered_map<uint64_t, uint64_t> came_from;
  std::unordered_map<uint64_t, bool>     closed;

  uint64_t s_key = cellKey(sx, sy);
  uint64_t g_key = cellKey(gx, gy);
  g_score[s_key] = 0.0;
  open.push({0.0, sx, sy});

  // 8-connected neighbors (planar)
  const int dx[8] = {-1,-1,-1, 0, 0, 1, 1, 1};
  const int dy[8] = {-1, 0, 1,-1, 1,-1, 0, 1};

  while (!open.empty() && last_expansions_ < p_.max_expansions) {
    Node n = open.top(); open.pop();
    uint64_t k = cellKey(n.ix, n.iy);
    if (closed[k]) continue;
    closed[k] = true;
    last_expansions_++;

    if (n.ix == gx && n.iy == gy) {
      // Reconstruct path.
      std::vector<uint64_t> rev;
      uint64_t cur = k;
      while (cur != s_key) {
        rev.push_back(cur);
        auto it = came_from.find(cur);
        if (it == came_from.end()) break;
        cur = it->second;
      }
      rev.push_back(s_key);
      out_path.reserve(rev.size());
      for (auto it = rev.rbegin(); it != rev.rend(); ++it) {
        int cx = static_cast<int>(static_cast<int32_t>(*it >> 32));
        int cy = static_cast<int>(static_cast<int32_t>(*it & 0xFFFFFFFF));
        out_path.push_back(toPos(cx, cy));
      }
      return true;
    }

    for (int i = 0; i < 8; ++i) {
      int nx = n.ix + dx[i];
      int ny = n.iy + dy[i];
      Vec3 cp = toPos(n.ix, n.iy);
      Vec3 np = toPos(nx, ny);
      if (transitionBlocked(cp, np, dx[i], dy[i])) continue;
      double step = (dx[i] != 0 && dy[i] != 0) ? std::sqrt(2.0) * res : res;

      // Clearance penalty: prefer wider corridors.
      double esdf_d = map_->dist(np);
      double clearance_pen = 0.0;
      if (esdf_d < p_.safe_dist && esdf_d > 0.0) {
        clearance_pen = p_.clearance_w * (p_.safe_dist - esdf_d);
      }
      if (static_map_) {
        double d2 = static_map_->clearance(Eigen::Vector2d(np.x(), np.y()),
                                           p_.static_safe_dist, nullptr);
        if (d2 < p_.static_safe_dist) {
          clearance_pen += p_.static_clearance_w * (p_.static_safe_dist - d2);
        }
      }

      uint64_t nk = cellKey(nx, ny);
      double tentative_g = g_score[k] + step + clearance_pen;
      auto it = g_score.find(nk);
      if (it == g_score.end() || tentative_g < it->second) {
        g_score[nk] = tentative_g;
        came_from[nk] = k;
        double dxh = (gx - nx) * res;
        double dyh = (gy - ny) * res;
        double h   = p_.heuristic_w * std::sqrt(dxh*dxh + dyh*dyh);
        open.push({tentative_g + h, nx, ny});
      }
    }
  }
  ROS_WARN("[rog_planner/astar] no path found after %d expansions", last_expansions_);
  return false;
}

}  // namespace rog_planner
