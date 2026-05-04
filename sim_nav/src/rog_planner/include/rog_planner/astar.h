// Phase 3: rog_planner — front-end 3-D voxel A*
//
// Search runs on ROG-Map's INFLATED occupancy via isOccupiedInflate().
// Z is hard-clamped to chassis_height because (a) the sentry doesn't fly
// and (b) virtual_ceil_height / virtual_ground_height return FREE for
// cells outside the band — so we MUST enforce the z-clamp explicitly,
// otherwise A* would consider above-ceiling expansions as legal.
//
// State: 2-D grid index (ix, iy) at z = chassis_height (constant).
// Neighbors: 8-connected at inflation_resolution (0.2 m by default).
// Heuristic: weighted Euclidean. Tiebreak prefers wider corridors via
// ESDF distance — cells with larger getDistance() get a small bonus.

#pragma once

#include <vector>
#include <Eigen/Core>
#include <rog_planner/sentry_map.h>
#include <rog_planner/static_map_2d.h>

namespace rog_planner {

struct AStarParams {
  double chassis_height = 0.3;   // z-clamp (m)
  double resolution     = 0.2;   // step size; should match inflation_resolution
  double safe_dist      = 0.4;   // soft buffer; cells with esdf<safe_dist get penalized
  double clearance_w    = 0.5;   // tiebreak weight on ESDF clearance
  double static_safe_dist = 0.25;
  double static_clearance_w = 0.5;
  double heuristic_w    = 1.001; // weighted A* (slightly inflated for speed)
  int    max_expansions = 20000;
  double snap_radius    = 1.5;   // m — radius for nearest-free fallback when goal blocked
};

class AStar3D {
public:
  using Vec3 = Eigen::Vector3d;
  using Path = std::vector<Vec3>;

  AStar3D(SentryMap* map, const AStarParams& p,
          StaticMap2D* static_map = nullptr)
      : map_(map), static_map_(static_map), p_(p) {}

  // Runs A*. On success returns true and fills `out_path` (start..goal,
  // map frame, z=chassis_height). On failure returns false and `out_path`
  // is empty. If goal is OCCUPIED, snaps to nearest free cell within
  // snap_radius along the goal direction; if still infeasible, fails.
  bool plan(const Vec3& start, const Vec3& goal, Path& out_path);

  // Last-call diagnostics
  int  expansions() const { return last_expansions_; }
  bool snapped()    const { return last_snapped_; }

private:
  SentryMap*  map_;
  StaticMap2D* static_map_;
  AStarParams p_;
  int  last_expansions_ = 0;
  bool last_snapped_    = false;

  // Try to find a free voxel near `pos` within `radius` along ray from `from`.
  bool snapToFree(const Vec3& from, const Vec3& pos, double radius, Vec3& out);
  bool isBlocked(const Vec3& p) const;
  bool transitionBlocked(const Vec3& from, const Vec3& to,
                         int dx, int dy) const;
};

}  // namespace rog_planner
