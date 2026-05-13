# Phase 3 — `rog_planner` Deployment Manual

> Reference manual for the ROG-Map-aware 3-D planner that replaces D\*-Lite in
> the sentry navigation stack. Implementation lives in
> `/home/sentry/AstarTraining/rogmap/Old_nav_backup/` (sandbox); production
> deployment will copy the package to `Old_nav/` after live validation.

---

## 1. Overview

`rog_planner` is a single ROS Noetic node that
1. owns its own `rog_map::ROGMap` instance with **ESDF enabled** (subclass
   `rog_planner::SentryMap`);
2. runs front-end **3-D voxel A\*** on the inflated occupancy grid,
   z-clamped to chassis height;
3. runs back-end **uniform cubic B-spline + L-BFGS** smoothing using the
   ESDF gradient as the obstacle force;
4. emits `/cmd_vel` (Twist, virtual_frame, **100 Hz**) and `/dstar_status`
   (Bool, **100 Hz**) — byte-for-byte compatible with the legacy
   `dstarlite` consumer interface (BehaviorTree + MCU bridge).

D\*-Lite is **not removed**. Selection is via launch arg `planner:=dstar|rog`,
default `dstar`.

| | D\*-Lite (legacy) | `rog_planner` (Phase 3) |
|--|--|--|
| Map dim | 2-D OccupancyGrid | 3-D voxel ESDF (chassis-height slice) |
| Search  | D\* on `/grid` or `/rog_grid` | A\* on `isOccupiedInflate()` |
| Smoothing | Local link tree, no-smoothing | B-spline + L-BFGS |
| Replan trigger | Edge cost change | Goal-change, expiry, projected-collision |
| Output | `/cmd_vel`@100Hz, `/dstar_status`@100Hz | identical |

---

## 2. Source layout

```
sim_nav/src/rog_planner/
├── CMakeLists.txt
├── package.xml
├── include/rog_planner/
│   ├── sentry_map.h          # ROGMap subclass; exposes ESDF grad + mutex
│   ├── astar.h               # 3-D voxel A* declarations
│   └── bspline_smoother.h    # B-spline trajectory + L-BFGS smoother
├── src/
│   ├── astar.cpp             # A* impl
│   ├── bspline_smoother.cpp  # cubic B-spline basis, cost+grad, L-BFGS
│   └── rog_planner_node.cpp  # main node: planner@5Hz + tracker@100Hz
└── launch/
    └── rog_planner.launch    # standalone launch (called by 3DNavUL_Test.launch)
```

Modified files outside the package:

| File | Change |
|--|--|
| `sim_nav/src/rog_map/include/rog_map/esdf_map.h` | Added public `getUpdateMtx()` returning `std::mutex&`. Single-line addition; private members untouched. |
| `sim_nav/src/bot_sim/config/rog_planner_map.yaml` | **NEW** — ROG-Map config with `esdf.enable: true` and optional `load_pcd_en` for static-PCD bootstrap. |
| `3DNavUL_Test.launch` | Added `<arg name="planner" default="dstar"/>`, conditional include of `rog_planner.launch`, auto-disable of `rog_map_observability.launch` when `planner==rog`. |

---

## 3. ROS interface

### 3.1 Subscriptions

| Topic | Type | Note |
|--|--|--|
| `/clicked_point` | `geometry_msgs/PointStamped` | Goal in `map` frame, published by `decision_node` BehaviorTree (`strategy_node.cpp PublishGoalPoint`). New goal accepted only if displacement > 0.05 m. |
| `/aligned_points` | `sensor_msgs/PointCloud2` | Consumed *internally* by SentryMap via ROGMap's `ros_callback`. Frame: `map`. |
| `/odom` | `nav_msgs/Odometry` | Consumed internally by SentryMap. From `hdl_localization`. |

### 3.2 Publications

| Topic | Type | Rate | Note |
|--|--|--|--|
| `/cmd_vel` | `geometry_msgs/Twist` | 100 Hz | In `virtual_frame`. `linear.{x,y}` populated; `linear.z = angular.* = 0`. **MCU consumes from `mcu_communicator.cpp`**. |
| `/dstar_status` | `std_msgs/Bool` | 100 Hz | `true` when `‖robot_xy − goal_xy‖ < arrival_radius`. **BehaviorTree consumes via `CheckArrived`**. |
| `~rog_planner_node/path` | `nav_msgs/Path` | per replan | Front-end A\* output, viz only. |
| `~rog_planner_node/trajectory` | `nav_msgs/Path` | per replan | Sampled smoothed B-spline. |
| `~rog_planner_node/rog_map/...` | various | per ROGMap config | ESDF / occupancy viz (same set as Stage 1 observer). |
| static TF | `map → world` | latch | Identity TF; ROG-Map hardcodes `world` in viz headers. |

### 3.3 Parameters

Set via `<rosparam>` in `rog_planner.launch`. ROGMap-specific keys live
under `~rog_map/...` (loaded from `rog_planner_map.yaml`).

| Param | Default | Purpose |
|--|--|--|
| `chassis_height` | `0.3` | z-clamp in A\* and ESDF queries. |
| `safe_dist` | `0.4` | Soft buffer; A\* cells with `esdf<safe_dist` get clearance penalty; smoother penalty `(safe_dist - esdf)²`. |
| `v_max` | `1.5` (launch) / `2.5` (planner default) | Linear-velocity cap. **Live arg defaults to 1.5 for first test**; raise after bench. |
| `a_max` | `4.0` | Acceleration cap (used for time allocation; full a-penalty pending). |
| `arrival_radius` | `0.25` | xy-only arrival threshold. Matches dstarlite. |
| `replan_rate` | `5.0` | Hz |
| `tracker_rate` | `100.0` | Hz |
| `w_smooth` | `10.0` | Jerk-approx weight. |
| `w_obs` | `10000.0` | ESDF obstacle weight. |
| `w_dyn` | `100.0` | Velocity-cap soft weight. |
| `blend_time` | `0.1` | Linear lerp window between active and pending trajectories. |
| `collision_d_min` | `0.05` | Projected-collision trigger threshold. |
| `rog_map/esdf/enable` | `true` | **Must be true** — planner depends on ESDF. |
| `rog_map/esdf/resolution` | `0.2` | ESDF voxel size. |
| `rog_map/load_pcd_en` | overridden by launch | Driven by `static_pcd:=` arg. |

---

## 4. Architecture

### 4.1 Threading model

```
┌────────────────────── rog_planner_node ──────────────────────┐
│                                                              │
│  AsyncSpinner(4)  ──┬──> ROGMap update_timer @ 1 kHz         │
│                     │      (writes esdf_map_->distance_buffer)│
│                     ├──> /aligned_points cb                  │
│                     ├──> /odom cb                            │
│                     ├──> planner_timer @ 5 Hz                │
│                     │      A* + L-BFGS (~100-200 ms)         │
│                     └──> tracker_timer @ 100 Hz              │
│                            B-spline eval + cmd_vel           │
└──────────────────────────────────────────────────────────────┘
```

### 4.2 Mutex ownership

- `getEsdfMutex()` — held by:
  - ROG-Map writer inside `ESDFMap::updateESDF3D()` (~1 kHz).
  - `AStar3D::plan()` for the entire search duration.
  - `BSplineSmoother::smooth()` around the L-BFGS optimize loop.
  - Planner-tick's projected-collision sweep.
- `traj_mtx_` — guards `traj_active_`, `traj_pending_`, `t_active_start_`, `t_blend_start_`, `has_active_`, `has_pending_`.
- `goal_mtx_` — guards `goal_`, `has_goal_`, `goal_dirty_`.

Lock ordering: `goal_mtx_` → `traj_mtx_` → `getEsdfMutex()`. Never reverse.

### 4.3 Replan triggers

| Trigger | Source |
|--|--|
| New goal (Δ > 5 cm) | `goalCallback` sets `goal_dirty_=true` |
| No active trajectory | `has_active_ == false` |
| Trajectory expired | `t_now > traj_active_.tEnd()` |
| Projected collision | Any sample on remaining traj has `dist < collision_d_min` |
| Stale `RobotState` (>0.5 s) | Skip replan; tracker emits `cmd_vel=0`, `status=false` |

`goal_dirty_` is only cleared **after** a successful trajectory install; if A\* or smoother fails, the next planner tick retries.

---

## 5. Cost model (back-end)

$$ J = w_s \, J_{\text{smooth}} + w_o \, J_{\text{obs}} + w_d \, J_{\text{dyn}} $$

with control points $Q_0, \dots, Q_{N-1} \in \mathbb{R}^2$ and uniform knot $dt$.

- **Smoothness (jerk approx):** $J_{\text{smooth}} = \sum_i \lVert Q_{i+1} - 2 Q_i + Q_{i-1} \rVert^2$
- **Obstacle (one-sided):** $J_{\text{obs}} = \sum_t \max(0, d_{\text{safe}} - \text{ESDF}(p(t)))^2$, sampled every `sample_dt = 0.05 s`.
- **Dynamics (velocity cap):** $J_{\text{dyn}} = \sum_t \max(0, \lVert v(t) \rVert - v_{\max})^2$.
  Acceleration penalty is reserved (`a_max` parsed; cost term TBD next iteration).
- **Endpoints pinned**: `Q_0=Q_1=` start, `Q_{N-2}=Q_{N-1}=` goal; gradients zeroed at pinned indices each iteration so the optimizer cannot drift them.

Optimizer: hand-rolled limited-memory BFGS (`m=8`) with backtracking
Armijo line search. Caps: 50 iters or 200 ms wall time, whichever first.

---

## 6. Build & install

### 6.1 First-time build

```bash
cd /home/sentry/AstarTraining/rogmap/Old_nav_backup/sim_nav
source /opt/ros/noetic/setup.bash
catkin_make --only-pkg-with-deps rog_planner -j4
```

`--only-pkg-with-deps` is required because the workspace contains
livox-dependent packages (FAST_LIO, Point-LIO, LiDAR_IMU_Init) whose
prebuilt `livox_ros_driver2Config.cmake` files reference a stale absolute
path. Building only `{rog_map, rog_planner}` skips them cleanly.

After first build, `source devel/setup.bash` to expose
`rog_planner_node` to `rosrun` / `roslaunch`.

### 6.2 Incremental rebuilds

```bash
catkin_make --only-pkg-with-deps rog_planner -j4
```

### 6.3 Verifying the build

```bash
ls devel/lib/rog_planner/rog_planner_node     # binary should exist
roscd rog_planner                              # package found
roscat rog_planner launch/rog_planner.launch   # launch file readable
```

---

## 7. Launching

### 7.1 Default (legacy D\*-Lite)

```bash
roslaunch <bot_sim or top-level> 3DNavUL_Test.launch
# planner:=dstar (default), behavior identical to today's stack.
```

### 7.2 ROG-Map planner — no static PCD

```bash
roslaunch <top-level> 3DNavUL_Test.launch planner:=rog
```

The planner's SentryMap builds the ESDF live from `/aligned_points`. **First
~5 s after launch the ESDF is empty** (`distance_buffer = 0` everywhere)
which would make the obstacle cost saturate. Operator must wait until
`/rog_planner_node/rog_map/esdf` shows non-zero data before publishing the
first `/clicked_point`.

### 7.3 ROG-Map planner — with static PCD bootstrap

```bash
roslaunch <top-level> 3DNavUL_Test.launch \
    planner:=rog \
    static_pcd:=/abs/path/to/innowing.pcd
```

The PCD is loaded once at startup and baked into the ESDF before the first
replan. **The file must exist** — ROG-Map's `loadPCDFile` failure path is
`exit(-1)`, which kills `rog_planner_node` immediately (look at the screen
log; there's no graceful fallback).

`innowing.pcd` is **not currently in the workspace**. Generate via:
- `roslaunch <top-level> hdl_graph_slam_mapping.launch` and rename the
  result of `save_map_on_update.py`, **OR**
- Reverse-convert `innowing.pgm` via `pcd2pgm_package` PCD output mode.

Pre-downsample to 0.1 m voxels if size > 1 M points (startup latency).

---

## 8. Validation gates (do not skip)

1. **Sim/unit** — gtest harness (TBD); confirm A\* finds reachable goal in synthetic ROGMap, smoother converges on toy obstacle.
2. **Bag replay** — `Old_nav/logs/rosbag_20260418_141634_0.bag` headless; assert no trajectory sample has `getDistance(p) < 0.05`.
3. **Bench, motors disconnected** — full launch, `planner:=rog`, motor power off. Cycle goals 60 s; CPU < 70 %.
4. **Live with kill-switch** — physical robot, `v_max:=1.5`. Tests:
   - Static-only goals.
   - Person walks across path → reroute < 1 s.
   - Goal switch mid-trajectory → no chassis jerk (blend window 100 ms).
   - Edge-of-map goal → snap-to-nearest works.
   - MCU `/nav_received` events steady throughout.
5. **A/B vs D\*-Lite** — same goal sequence; `rog` should match-or-beat time-to-goal, ≤ 1.1× path length, strictly higher min-clearance.
6. **Service-time gate** — do **not** flip `planner:=rog` in `run_3DNavUL_Test_with_decision.sh` until all above pass. Default `dstar` for at least one match cycle after first live success.

---

## 9. Known limitations / pending work

| ID | Description | Priority |
|--|--|--|
| L1 | Acceleration penalty `J_a = max(0, ‖a‖−a_max)²` not yet implemented (`a_max` is parsed but unused in cost). Velocity cap alone may emit spikes through tight corners. | high |
| L2 | Snap-to-free in A\* walks along the line `goal→start`; for an occluded *start*, this samples toward goal which is geometrically reasonable but not radial. | medium |
| L3 | First ~5 s after launch the ESDF is empty; smoother sees `dist=0` everywhere. Operator workaround: wait for ESDF viz to populate before sending first goal. Future: gate replan on first cloud frame. | medium |
| L4 | Z-tilt scaling `(s_x, s_y) ∈ [0.7, 1.7]` can multiply commanded speed up to `1.7 × v_max` on negative tilt — matches dstarlite parity, intentional. Operator should derate `v_max` accordingly. | low (parity) |
| L5 | `static_pcd:=<missing_path>` causes silent `exit(-1)` from inside ROG-Map. Add pre-flight check or clearer error. | low |
| L6 | L-BFGS does not track the best iterate; on line-search failure the last x is emitted. Worst case: smoother degenerates to passthrough of the front-end CPs. | low |

---

## 10. Rollback plan

`rog_planner` is opt-in. To revert any system to legacy behavior:

1. Restart any active launch with `planner:=dstar` (or omit the arg).
2. The unmodified `dstarlite.launch` includes are reached, identical to
   the pre-Phase-3 stack.

The only persistent change to `rog_map` is the addition of `getUpdateMtx()`
which is a non-breaking public accessor; all dependents of `rog_map` continue
to compile and run.

---

## 11. File checksum reference (sandbox state at deployment)

To diff against your local copy:

```
sim_nav/src/rog_map/include/rog_map/esdf_map.h         (1 line added)
sim_nav/src/bot_sim/config/rog_planner_map.yaml        (NEW)
sim_nav/src/rog_planner/                               (NEW package)
3DNavUL_Test.launch                                    (planner arg + 2 includes)
```
