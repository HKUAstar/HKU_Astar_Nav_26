# ROG-Planner Live-Test Bug-Fix Session Notes

**Workspace:** `rogmap/Old_nav_backup/sim_nav`  
**Build command:**
```bash
cd /home/sentry/AstarTraining/rogmap/Old_nav_backup/sim_nav
source /opt/ros/noetic/setup.bash
catkin_make --only-pkg-with-deps rog_planner -j4
```
**Launch command:**
```bash
roslaunch /home/sentry/AstarTraining/rogmap/Old_nav_backup/3DNavUL_Test.launch planner:=rog
# Note: static_pcd arg is now hard-disabled inside rog_planner.launch (see §4)
```
**Build target:** `rog_planner_node` (only needs rog_map + rog_planner).  
**Status at session end:** All builds clean, system live (`/rog_planner_node` confirmed in `rosnode list`), code committed and pushed.

---

## 1. Original Bug Reports

| # | Symptom | User description |
|---|---------|-----------------|
| 1 | Real-time obstacles not avoided | "the inf/occ is not updating when the robot moves" |
| 2 | Stale obstacle trace (ghost trail) | "when obstacles move, they leave a trace of inflated obstacle cells and this trace does not disappear" |
| 3 | Zigzag trajectory | "/trajectory rugged and the robot moves in a zigzag way" |
| 4 | Self-detection blind spot | "the robot did not consider itself when calculating the inf map. check whether the robot blinded obstacles within 0.3m" |
| 5 | cmd_vel corner-cutting | "cmd_vel kept pointing to a point a few steps ahead, resulting in a behavior that it always tries to cut corners and crash into an obstacle" |

---

## 2. Root-Cause Analysis

### Bug 1 + 2: Stale Map / Ghost Trails (Core ROG-Map Limitation)

**Root cause:** ROG-Map's `raycastProcess` only fires free-rays along **currently-returning beams**. When a moving obstacle vacates a cell, no subsequent return lies along that exact beam direction, so the cell receives zero miss updates and stays at `l_max` (fully occupied) **forever**. This is a structural limitation of standard probabilistic raycasting.

**Contributing factors that worsened it:**
- Original `inflation_step: 0` → no inflation halo at all (bug in config, set to `1`).
- Original map size was large (30 m) → old ghost cells from the far side of the map never got overwritten.
- Static PCD (`innowing.pcd`) was being baked into the live `prob_map` at startup. Any cell in the PCD that wasn't covered by current lidar returns could never clear.
- Aggressive `p_miss = 0.45` (original, too high) meant cells needed many consecutive misses to clear even when rayed.
- `map_sliding.threshold` was large, so the map didn't slide aggressively to flush stale regions.

**Structural fix:** Visibility-aware stale-occupied decay (`clearStaleOccupied`) — see §4.

### Bug 3: Zigzag Trajectory

**Root cause:** A* on a 2D grid with 8-connectivity produces staircase paths. The original path pruner used a `isLineFree` check that was too conservative (single-point queries) and degenerated on dense obstacle fields. B-spline smoother had knot spacing `dt=0.4` (short, many knots → over-constrained) and `w_smooth=10` (too low). The optimizer had insufficient budget (`max_iter=50`, `max_time_s=0.20`) to converge.

**Fix:** ESDF-based 4-pass iterative path pruner + increased smoother weights + larger A* resolution.

### Bug 4: Self-Detection Blind Spot

**Root cause:** `ray_range[0]` (inner radius filter = start-of-free-ray position) was 0.3 m in the YAML, but the actual **hit filter** was missing. A lidar point closer than 0.3 m from the sensor origin (e.g., the robot chassis) was being used as a hit point, inflating the occupancy around the robot permanently. The robot couldn't find a free start for A*.

**Fix:** Inner-radius filter in `raycastProcess`: drop any hit where `(p - cur_odom).squaredNorm() < sqr_raycast_range_min`.

### Bug 5: cmd_vel Corner-Cutting

**Root cause:** The tracker was purely time-indexed. It evaluated the trajectory at `t_now` and followed the current point. When the robot fell behind (e.g., due to corner drag or replanning), `t_now` advanced past the actual robot position on the path, so the tracker was always pointing to a future point → geometric corner-cut → crash into wall.

**Fix:** Nearest-point trajectory tracker — scans the entire trajectory at fine dt steps, finds the minimum-distance point, then follows that (with optional small lookahead).

---

## 3. File-by-File Changes

### 3.1 `src/bot_sim/config/rog_planner_map.yaml`

Full final content:
```yaml
resolution: 0.05
inflation_resolution: 0.05
inflation_step: 1              # was 0 (no halo) — fixed
unk_inflation_en: false
unk_inflation_step: 1
map_size: [14.0, 14.0, 4.0]   # was 30 m — shrunk to flush stale ghosts faster
virtual_ceil_height: 1
virtual_ground_height: -0.5
load_pcd_en: false             # was true — disabled (see §4 Static PCD)
pcd_name: ""
point_filt_num: 1
intensity_thresh: -1
map_sliding:
  enable: true
  threshold: 0.05              # was large — now slides on every 5 cm
ros_callback:
  enable: true
  cloud_topic: /aligned_points
  odom_topic: /odom
  odom_timeout: 2.0
raycasting:
  enable: true
  batch_update_size: 1
  local_update_box: [12.0, 12.0, 3.0]
  ray_range: [0.30, 8.0]       # [0] = inner-radius filter / free-ray start
  p_hit: 0.70
  p_miss: 0.05                 # was 0.45 — very aggressive miss → 1 frame clearance
  p_min: 0.05
  p_max: 0.80
  p_occ: 0.65
  p_free: 0.35
  unk_thresh: 0.70
  stale_decay_en: true         # NEW: visibility-aware ghost-trail fix
  stale_decay_min_range: 0.32
  stale_decay_max_range: 8.0
  stale_decay_threshold: 8    # frames cooldown before firing decay
esdf:
  enable: true
  resolution: 0.1
  local_update_box: [12.0, 12.0, 3.0]
visualization:
  enable: true
  time_rate: 10
  range: [12.0, 12.0, 3.0]
  frame_id: map
  pub_unknown_map_en: false
  use_dynamic_reconfigure: false
```

### 3.2 `src/rog_map/include/rog_map/rog_map_core/config.hpp`

**Added** in struct fields:
```cpp
// Visibility-aware stale-occupied decay parameters.
bool stale_decay_en{true};
double stale_decay_min_range{0.4};
double stale_decay_max_range{8.0};
double sqr_stale_decay_min_range{0.16};
double sqr_stale_decay_max_range{64.0};
int stale_decay_threshold{6};
```

**Added** in constructor (under `name_space + "/raycasting/"` block):
```cpp
LoadParam(name_space + "/raycasting/stale_decay_en", stale_decay_en, true);
LoadParam(name_space + "/raycasting/stale_decay_min_range",
          stale_decay_min_range, raycast_range_min * 1.2);
LoadParam(name_space + "/raycasting/stale_decay_max_range",
          stale_decay_max_range, raycast_range_max);
LoadParam(name_space + "/raycasting/stale_decay_threshold",
          stale_decay_threshold, 6);
sqr_stale_decay_min_range = stale_decay_min_range * stale_decay_min_range;
sqr_stale_decay_max_range = stale_decay_max_range * stale_decay_max_range;
```

### 3.3 `src/rog_map/include/rog_map/prob_map.h`

**Added** to `ProbMap` class:
```cpp
// Public:
void clearStaleOccupied(const Vec3f &cur_odom);

// Protected (RaycastData struct):
std::vector<uint16_t> stale_frames_;   // per-cell cooldown counter

// Protected (class member):
std::unordered_set<int> occupied_hash_set_;  // hash IDs of OCCUPIED cells
```

**Added** `#include <unordered_set>` at top.

### 3.4 `src/rog_map/src/rog_map/prob_map.cpp`

#### `initProbMap()`
```cpp
raycast_data_.stale_frames_.resize(map_size, 0);
```

#### `resetLocalMap()`
```cpp
std::fill(raycast_data_.stale_frames_.begin(), raycast_data_.stale_frames_.end(), 0);
occupied_hash_set_.clear();
```

#### `resetCell()` (on OCCUPIED→evict path)
```cpp
occupied_hash_set_.erase(hash_id);
```

#### `hitPointUpdate()` (after `from_type != to_type` block)
```cpp
if (to_type == GridType::OCCUPIED) {
    occupied_hash_set_.insert(hash_id);
} else if (from_type == GridType::OCCUPIED) {
    occupied_hash_set_.erase(hash_id);
}
```

#### `missPointUpdate()` (after `from_type != to_type` block)
```cpp
if (from_type == GridType::OCCUPIED && to_type != GridType::OCCUPIED) {
    occupied_hash_set_.erase(hash_id);
}
```

#### `updateProbMap()` (before `probabilisticMapFromCache()`)
```cpp
clearStaleOccupied(pos);
```
Called in the `batch_update_counter == 0` branch, immediately before `probabilisticMapFromCache()`.

#### `raycastProcess()` — inner-radius filter (Bug 4 fix)
Added after intensity filter and temporal filter:
```cpp
// inner-radius filter: drop returns inside the sensor's footprint
if ((p - cur_odom).squaredNorm() < cfg_.sqr_raycast_range_min) {
    continue;
}
```

#### `clearStaleOccupied()` — NEW METHOD (full implementation)
```cpp
void ProbMap::clearStaleOccupied(const Vec3f& cur_odom) {
    if (!cfg_.stale_decay_en || occupied_hash_set_.empty()) return;

    raycaster::RayCaster rc;
    rc.setResolution(cfg_.resolution);
    Vec3f ray_pt;

    // Snapshot set to avoid invalidation during iteration
    std::vector<int> snapshot(occupied_hash_set_.begin(), occupied_hash_set_.end());
    for (int hash_id : snapshot) {
        // Real obstacle: got a hit this frame → reset cooldown, keep it
        if (raycast_data_.hit_cnt[hash_id] > 0) {
            raycast_data_.stale_frames_[hash_id] = 0;
            continue;
        }
        // Normal miss processing this frame → skip (normal flow handles it)
        if (raycast_data_.operation_cnt[hash_id] > 0) continue;

        Vec3i id_g;
        Vec3f cell_pos;
        hashIdToGlobalIndex(hash_id, id_g);
        globalIndexToPos(id_g, cell_pos);

        // Range gate
        Vec3f delta = cell_pos - cur_odom;
        double dsq = delta.squaredNorm();
        if (dsq < cfg_.sqr_stale_decay_min_range ||
            dsq > cfg_.sqr_stale_decay_max_range) {
            raycast_data_.stale_frames_[hash_id] = 0;
            continue;
        }

        // Virtual ceiling/ground guard
        if (cell_pos.z() > cfg_.virtual_ceil_height ||
            cell_pos.z() < cfg_.virtual_ground_height) {
            raycast_data_.stale_frames_[hash_id] = 0;
            continue;
        }

        // Raycast from sensor (just outside inner radius) toward cell.
        // If another OCCUPIED cell blocks line-of-sight → cell is occluded
        // → reset cooldown (cannot conclude it's stale).
        const double dist = std::sqrt(dsq);
        Vec3f start = cur_odom + delta * (cfg_.raycast_range_min / dist);
        rc.setInput(start, cell_pos);
        bool occluded = false;
        while (rc.step(ray_pt)) {
            Vec3i ray_id_g;
            posToGlobalIndex(ray_pt, ray_id_g);
            if (ray_id_g == id_g) break;             // arrived at target
            if (!insideLocalMap(ray_id_g)) { occluded = true; break; }
            const int rh = getHashIndexFromGlobalIndex(ray_id_g);
            if (occupancy_buffer_[rh] >= cfg_.l_occ) { occluded = true; break; }
        }
        if (occluded) {
            raycast_data_.stale_frames_[hash_id] = 0;
            continue;
        }

        // Cell is visible, not hit → increment cooldown
        if (raycast_data_.stale_frames_[hash_id] < 0xFFFF)
            raycast_data_.stale_frames_[hash_id]++;
        if (raycast_data_.stale_frames_[hash_id] >= cfg_.stale_decay_threshold) {
            insertUpdateCandidate(id_g, false);   // queue a miss
            raycast_data_.stale_frames_[hash_id] = 0;
        }
    }
}
```

**How it works:**  
Every frame, after `raycastProcess`, iterate over all currently-OCCUPIED cells. If a cell is **visible from the sensor** (no other occupied cell blocks the LOS) **and received no return**, increment its stale counter. Once the counter hits `stale_decay_threshold` (8 frames), enqueue a miss update. With `p_miss = 0.05` and `l_min` deep, a single miss drives the cell well below `l_occ` → it clears in one shot. Real walls and obstacles get re-hit every frame → counter stays at 0.

**Why the cooldown (threshold=8)?**  
Sparse lidar (Livox MID360) does not hit every wall cell on every frame due to finite angular density. Without a cooldown, real walls would flicker. 8 frames ≈ 400 ms at 20 Hz cloud rate — sufficient to let the lidar sweep hit the cell at least once if it's real.

### 3.5 `src/rog_planner/include/rog_planner/astar.h`

**Added** to `AStarParams`:
```cpp
double hard_safe_dist = 0.3;
double static_hard_safe_dist = 0.25;
```

### 3.6 `src/rog_planner/src/astar.cpp`

#### `isBlocked()` — added ESDF hard-margin check
```cpp
// ESDF hard footprint margin
if (map_->dist(p) > 1e-6 && map_->dist(p) < p_.hard_safe_dist)
    return true;
// Static map: occupied OR too close
if (static_map_ && static_map_->isOccupied(...)) return true;
if (static_map_ && static_map_->clearance(...) < p_.static_hard_safe_dist) return true;
```

#### `snapToFree()` — radial fallback
Added spiral/radial search up to `snap_radius=1.5 m` when the original line-back snap fails. Prevents A* from giving up when the computed start is slightly inside the inflation halo.

### 3.7 `src/rog_planner/include/rog_planner/bspline_smoother.h`

**Changed** `BSplineParams` defaults:
| Param | Old | New |
|-------|-----|-----|
| `dt` | 0.4 | 0.6 |
| `w_smooth` | 10 | 800 |
| `w_obs` | 10000 | 3000 |
| `max_iter` | 50 | 120 |
| `max_time_s` | 0.20 | 0.30 |

### 3.8 `src/rog_planner/src/bspline_smoother.cpp`

#### `prunePath()` — complete rewrite
Old: `isLineFree` single-point check.  
New: ESDF-based 4-pass iterative pruner.

```
segmentOk(a, b):
  - sample at 0.05 m intervals
  - require ESDF >= shortcut_clear = max(0.05, safe_dist * 0.5) at every sample
  - require !static_map_->isOccupied at every sample
  - if any sample fails → segment is not OK

prunePath(path):
  for up to 4 passes:
    for i in path:
      if segmentOk(path[i], path[i+2]):   // skip intermediate node
        remove path[i+1]
    if no change this pass: break (fixed point)
```

This collapses the 8-connected A* staircase into diagonal shortcuts and removes redundant waypoints that cause the B-spline to oscillate.

#### `smooth()` — signature change
Now takes `front_path_in` directly, calls `prunePath` internally. Old blend-tail code removed.

### 3.9 `src/rog_planner/src/rog_planner_node.cpp`

#### Nearest-point tracker (Bug 5 fix)
```cpp
// Scan entire trajectory, find minimum-distance point
double best_t = 0.0, best_d2 = inf;
for (double tt = 0.0; tt < t_end; tt += track_search_dt_) {
    double d2 = (traj_active_.pos(tt).head<2>() - rs.p.head<2>()).squaredNorm();
    if (d2 < best_d2) { best_d2 = d2; best_t = tt; }
}
double t_ref = min(t_end - 1e-3, best_t + track_lookahead_time_);
pos_des = traj_active_.pos(t_ref);
vel_des = traj_active_.vel(t_ref);
```
`track_lookahead_time_ = 0.0` (no lookahead by default).  
`track_kp_ = 0.45` (cross-track proportional gain).

#### `commandSegmentIsSafe()` — new method
```cpp
bool commandSegmentIsSafe(pos, v_cmd):
  horizon = min(0.45, max(0.15, speed * 0.25))
  for samples along dir * horizon:
    if ESDF dist < tracker_safe_dist_: return false
    if static_map clearance < static_map_safe_dist_: return false
  return true
```
- `tracker_safe_dist_ = 0.18 m` — **relaxed** relative to planner's `hard_safe_dist = 0.30 m`.  
- Why separate threshold: the planner plans with 0.30 m margin so paths legitimately hug the 0.30 m clearance line. If the tracker used the same threshold it would reject every command issued near a wall. 0.18 m catches genuine new closer obstacles without killing valid plans.

#### Debounce
```cpp
if (++unsafe_consecutive_ >= tracker_unsafe_ticks_to_clear_) {  // 10 ticks = 100 ms
    clearActiveTrajectory("tracker command segment unsafe");
    unsafe_consecutive_ = 0;
}
```
Single-frame map glitches (ghost flicker) do not nuke the active plan.

#### Trajectory blending removed
Old code blended `traj_pending_` into `traj_active_` over `blend_time_`. Removed entirely — new replans install directly as `traj_active_`. This eliminated the oscillation caused by the blend interpolation.

### 3.10 `src/rog_planner/launch/rog_planner.launch`

**Key params set:**
```xml
<arg name="w_smooth"               default="800.0"/>
<arg name="w_obs"                  default="3000.0"/>
<arg name="astar_resolution"       default="0.2"/>
<arg name="hard_safe_dist"         default="0.30"/>
<arg name="collision_d_min"        default="0.30"/>
<arg name="track_kp"               default="0.45"/>
<arg name="track_lookahead_time"   default="0.0"/>
<arg name="track_search_dt"        default="0.02"/>
<arg name="tracker_safe_dist"      default="0.18"/>
<!-- Static PCD disabled — bakes stale indoor map into live prob_map -->
<param name="~rog_map/load_pcd_en" value="false"/>
<param name="~rog_map/pcd_name"    value=""/>
```

---

## 4. Static PCD Pollution Fix (Critical)

The original launch file passed `static_pcd:=…/innowing.pcd` to ROG-Map. On startup, ROG-Map loaded the entire indoor scene as occupied cells in `prob_map`. These cells never received lidar returns during live operation (robot moves, lidar angle changes), so they could never clear. This caused:
- Permanent phantom obstacles from empty/cleared indoor features.
- A* start-snap failures (robot center appeared occupied).
- Planner thrashing and stale-trajectory-invalid loops.

**Fix:** Force `load_pcd_en = false` and `pcd_name = ""` hard inside `rog_planner.launch`. The `static_pcd` launch arg still exists but is ignored by the planner node. Wall avoidance now comes exclusively from `/map` (2D costmap → `StaticMap2D`).

---

## 5. Parameter Tuning Guide

### Ghost Trail Persistence

If ghosts from moving obstacles still linger:
- Decrease `stale_decay_threshold` (e.g., 4–5). Faster decay.
- Confirm `p_miss: 0.05` is in YAML. With this aggressive miss, a single queued miss clears a fully-saturated cell.

If real walls flicker (disappear and reappear):
- Increase `stale_decay_threshold` (e.g., 10–12). More tolerance for sparse lidar coverage.

### Planner Safety Margins

If A* cannot find paths in tight corridors:
- Reduce `hard_safe_dist` from 0.30 to 0.20 m.
- Reduce `inflation_step` from 1 to 0 (no halo — not recommended unless tight space absolutely requires it).

If robot still clips walls:
- Increase `hard_safe_dist` to 0.35 m.
- Increase `safe_dist` (smoother's ESDF repulsion range).

### Tracker Tuning

If `cmd_vel = 0` frequently (trajectory being cleared):
- Reduce `tracker_safe_dist` from 0.18 to 0.12 m.
- Increase debounce from 10 to 20 ticks.

If robot oscillates near trajectory:
- Reduce `track_kp` from 0.45 to 0.30.
- Add small `track_lookahead_time` (e.g., 0.05 s) to smooth out near-zero-distance instability.

### Smoother Quality

If trajectory still has residual oscillation:
- Increase `w_smooth` (e.g., 1200) and `max_iter` (e.g., 150).
- Increase `dt` (e.g., 0.8 s) to space control points further apart → smoother curve.

If smoother is too slow (> 150 ms per replan):
- Reduce `max_iter` to 80 or `max_time_s` to 0.15 s.
- Increase `astar_resolution` to 0.3 m to reduce waypoint count.

---

## 6. Architecture Overview

```
Livox MID360
     │  /aligned_points (PointCloud2)
     ▼
hdl_localization ──► /odom
     │                │
     └──────────────┬─┘
                    ▼
              ROG-Map (ProbMap)
              ┌─────────────────────────────────┐
              │ raycastProcess()                │
              │   ├─ inner-radius filter (0.3m) │ ← Bug 4 fix
              │   ├─ hit update (prob_map)       │
              │   └─ miss update (free-rays)     │
              │ clearStaleOccupied()             │ ← Bug 2 fix
              │   └─ visibility-aware decay      │
              │ probabilisticMapFromCache()       │
              │ updateESDF3D()                   │
              └─────────────────────────────────┘
                    │
              inf_map_ ──► /rog_map/occ_cloud  (viz)
              esdf_map_ ──► ESDF queries
                    │
              rog_planner_node
              ┌─────────────────────────────────┐
              │ 5 Hz planner timer              │
              │   ├─ A* (ESDF hard-margin)      │ ← Bug 4 fix (footprint)
              │   ├─ prunePath (4-pass ESDF)    │ ← Bug 3 fix
              │   └─ BSpline + L-BFGS smoother  │ ← Bug 3 fix
              │ 100 Hz tracker timer            │
              │   ├─ nearest-point search       │ ← Bug 5 fix
              │   ├─ commandSegmentIsSafe       │ ← safety guard
              │   │   └─ debounce (10 ticks)    │
              │   └─ /cmd_vel (virtual_frame)   │
              └─────────────────────────────────┘
```

---

## 7. Log-Odds Probability Math

With `p_miss = 0.05`:
```
l_miss = logit(0.05) = log(0.05/0.95) ≈ -2.944
l_min  = logit(0.05) ≈ -2.944
l_max  = logit(0.80) ≈  1.386
l_occ  = logit(0.65) ≈  0.619
```

A fully-saturated cell (at `l_max = 1.386`) needs:
```
N = ceil((l_occ - l_max) / l_miss) = ceil((0.619 - 1.386) / -2.944) = ceil(0.26) = 1
```
→ **One miss update drops a fully-occupied cell below l_occ immediately.**

This is why stale-decay with `stale_decay_threshold=8` is effective even though only one miss is fired: the cell clears in the same frame the decay fires.

---

## 8. Known Remaining Issues / Watchlist

1. **Lidar occlusion vs. stale-decay:** If a real obstacle is completely behind a wall for > 8 frames (e.g., robot makes a tight turn), the cells briefly behind the wall may get stale-decayed. Recovery: robot sees them again on the next scan → re-occupied in one hit frame. Minor visual flicker only; A* replans on the next 5 Hz tick.

2. **A* tight-corridor failures:** `hard_safe_dist = 0.30 m` combined with `inflation_step = 1` at `resolution = 0.05 m` → effective clearance envelope = ~0.35 m per side. Corridors narrower than ~0.80 m may block A*. Reduce `hard_safe_dist` to 0.20 m if this occurs.

3. **ESDF update latency:** ESDF is updated at 1 kHz (ROGMap timer), but the `prob_map` updates at cloud rate (~20 Hz). There is a ~50 ms ESDF lag after new obstacles appear. Planner collision checks use ESDF, so very fast obstacles (< 0.5 m in 50 ms) could slip through. Mitigation: `commandSegmentIsSafe` uses ESDF directly in the tracker loop at 100 Hz, catching fast-moving obstacles before `cmd_vel` is issued.

4. **cmd_vel = 0 on every tick near walls:** If this returns, lower `tracker_safe_dist` from 0.18 to 0.12 m or increase debounce to 20 ticks.

5. **`track_lookahead_time = 0.0`:** The nearest-point tracker with zero lookahead can oscillate if robot position is exactly on the trajectory (zero cross-track error → no directional bias from `vel_des`). Add `track_lookahead_time = 0.05` if oscillation near waypoints is observed.

---

## 9. Files Modified (Summary)

| File | What changed |
|------|-------------|
| `src/bot_sim/config/rog_planner_map.yaml` | `inflation_step`, `map_size`, `p_miss`, `map_sliding.threshold`, `load_pcd_en`, added stale-decay params |
| `src/rog_map/include/rog_map/rog_map_core/config.hpp` | Added stale-decay config fields and loading code |
| `src/rog_map/include/rog_map/prob_map.h` | Added `clearStaleOccupied()`, `stale_frames_`, `occupied_hash_set_` |
| `src/rog_map/src/rog_map/prob_map.cpp` | Inner-radius filter; `clearStaleOccupied()` implementation; hash-set maintenance in hit/miss/reset; stale_frames_ init |
| `src/rog_planner/include/rog_planner/astar.h` | Added `hard_safe_dist`, `static_hard_safe_dist` to `AStarParams` |
| `src/rog_planner/src/astar.cpp` | ESDF hard-margin in `isBlocked()`; radial fallback in `snapToFree()` |
| `src/rog_planner/include/rog_planner/bspline_smoother.h` | Smoother param defaults (`dt`, `w_smooth`, `w_obs`, `max_iter`, `max_time_s`) |
| `src/rog_planner/src/bspline_smoother.cpp` | Complete rewrite of `prunePath()` (ESDF 4-pass); `smooth()` signature |
| `src/rog_planner/src/rog_planner_node.cpp` | Nearest-point tracker; `commandSegmentIsSafe()`; debounce; blend removal; `#include <limits>` |
| `src/rog_planner/launch/rog_planner.launch` | Added launch args for all new params; disabled static PCD |

---

*End of session notes.*
