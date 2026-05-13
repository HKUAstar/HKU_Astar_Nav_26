# Phase 3 — ROG-Map-Aware 3-D Planner with ESDF
## Technical Design Plan (v1.0, 2026-04-26)

> **Status**: Design complete — implementation not started.
> **Authors**: Sentry Navigation Team
> **Predecessor**: Phase 1 (ROG-Map observer) + Phase 2 (dynamic costmap injection into D\*-Lite)
> **Related files**: [ROGMap_Guide.md](ROGMap_Guide.md) · [ROGMap_Stage1_Deployment_Manual.md](ROGMap_Stage1_Deployment_Manual.md) · [ROGMap_Stage2_Deployment_Manual.md](ROGMap_Stage2_Deployment_Manual.md)

---

## TL;DR

Replace `dstarlite` with a new ROS package **`rog_planner`** that performs:

- **(a) Front-end**: 3-D voxel A\* on ROG-Map's `InfMap` (`isOccupiedInflate`)
- **(b) Back-end**: L-BFGS trajectory smoothing driven by ROG-Map's sliding ESDF gradient

A 100 Hz tracker turns the resulting B-spline trajectory into `/cmd_vel`.  
**The BehaviorTree (`/clicked_point` in, `/dstar_status` out) and MCU bridge (`/cmd_vel` + `/dstar_status`) contracts are preserved byte-for-byte.**  
The static field map (`innowing.pcd`) is bootstrapped into ROG-Map via `load_pcd_en` so wall priors are part of the same ESDF — no separate static-map fusion code path.

---

## System Architecture

```
BehaviorTree ──/clicked_point──► rog_planner_node
                                     │  ├─ SentryMap (ROGMap subclass)
                                     │  │     ├─ isOccupiedInflate (A*)
                                     │  │     └─ evaluateFirstGrad (L-BFGS)
                                     │  ├─ KinoAstar (front-end)
                                     │  ├─ BsplineOptimizer (back-end)
                                     │  └─ Tracker (100 Hz)
                                     │        ├─ /cmd_vel   → MCU bridge → MCU
                                     └────────└─ /dstar_status → BT CheckArrived
```

**Existing pipeline nodes untouched**: `livox_ros_driver2`, `hdl_localization`, `rog_map_node` (Stage 1), `decision_node`, `mcu_communicator`.

---

## Implementation Phases

### Phase A — ROG-Map Preparation *(parallel with B)*

**Goal**: enable ESDF and bootstrap static map. All changes are in YAML only.

1. **Enable ESDF** in `Old_nav/sim_nav/src/bot_sim/config/rog_map_passive.yaml`:
   - `esdf.enable: true`
   - `esdf.resolution: 0.2`
   - `esdf.local_update_box: [12, 12, 4]`  ← must be ≤ `raycasting/local_update_box` = `[14,14,8]`
   - `frontier_extraction_en: false` (not needed for planning)

2. **Generate `innowing.pcd`** (the file is NOT in the workspace today — must be created):
   - **Option A** (preferred): re-run `hdl_graph_slam_mapping.launch` in the real environment; `save_map_on_update.py` writes `mapping_result.pcd` → rename to `innowing.pcd`.
   - **Option B**: reverse-convert the existing `innowing.pgm` via `pcd2pgm_package` (PCD output mode) — produces 2-D extruded cloud.
   - Pre-downsample to 0.1 m voxels with a PCL voxel filter if the PCD has > 1 M points (reduces startup latency).

3. **Enable PCD bootstrap** in `rog_map_passive.yaml`:
   - `load_pcd_en: true`
   - `pcd_name: <absolute path to innowing.pcd>`
   - Note: `rog_map.cpp` calls `updateESDF3D` immediately after PCD load, so static walls appear in the ESDF at startup — no extra code needed.

4. **Smoke test** (before writing any planner code):
   - Launch Stage 1 + a temporary ad-hoc node that prints `map.getDistance({rx, ry, 0.3})`.
   - Verify: sane positive values (≥ `safe_dist`) in open space, negative values inside walls.
   - Visualize `/rog_map_node/rog_map/esdf` in RViz.

---

### Phase B — Package Skeleton *(parallel with A)*

**Goal**: create the `rog_planner` catkin package with compile-clean stubs.

1. **Directory**: `Old_nav/sim_nav/src/rog_planner/` with structure:
   ```
   rog_planner/
     CMakeLists.txt
     package.xml
     config/rog_planner.yaml
     include/rog_planner/sentry_map.h
     include/rog_planner/kino_astar.h
     include/rog_planner/bspline_optimizer.h
     include/rog_planner/tracker.h
     src/rog_planner_node.cpp
     src/sentry_map.cpp
     src/kino_astar.cpp
     src/bspline_optimizer.cpp
     src/tracker.cpp
     test/unit_sentrymap.cpp
     test/unit_kino_astar.cpp
     test/unit_lbfgs_smooth.cpp
     launch/rog_planner.launch
   ```

2. **`package.xml` deps**: `roscpp`, `rog_map`, `tf2_ros`, `tf2_geometry_msgs`, `nav_msgs`, `geometry_msgs`, `Eigen3`, `pcl_ros`.

3. **`SentryMap` subclass** (`sentry_map.h`):
   - Subclasses `rog_map::ROGMap`.
   - `esdf_map_` is `protected` in `ROGMap`; its methods (`getDistance`, `evaluateFirstGrad`, `evaluateSecondGrad`) are public on `ESDFMap`.
   - Add forwarding wrappers: `dist(p)`, `grad(p, out)`, `hess(p, out)`.
   - Expose the mutex `update_esdf_mtx` from `ESDFMap` via a `getMutex()` accessor for the optimizer's snapshot acquisition.

4. **Three-timer structure** in `rog_planner_node.cpp`:
   - **Map thread**: ROG-Map's internal `ros_callback` (driven by `rog_map_node` — Stage 1 handles this separately, so the planner subscribes to the map object, not raw topics).
   - **Planner timer** @ 5 Hz: replan if goal changed, trajectory expired, or collision detected ahead.
   - **Tracker timer** @ 100 Hz: evaluate active trajectory at `t = now − t_start`; publish `/cmd_vel` + `/dstar_status`.

5. **Vendor dependencies** (both single-header, copy into `include/rog_planner/third_party/`):
   - `lbfgs.hpp` — from **libLBFGS** by Naoaki Okazaki (https://github.com/chokkan/liblbfgs), MIT license. **Do NOT use the copy from EGO-Planner-v2** (that package is GPL-3.0; vendoring from upstream avoids license contamination).
   - `bspline_utils.h` — hand-roll or vendor from a permissive source (~150 LOC); provides `pos(t)`, `vel(t)`, `acc(t)` eval over uniform order-3 B-spline control points.

---

### Phase C — Front-End: Voxel A\* on InfMap *(depends on B)*

**Goal**: find a coarse collision-free 3-D path from robot pose to goal.

1. **State**: `(x, y, z)` at z **hard-clamped** to chassis height (`chassis_height ≈ 0.3 m`). The sentry doesn't fly; constraining z eliminates a full search dimension. The clamp must be **explicit in code** — do NOT rely on `virtual_ceil_height` / `virtual_ground_height` to bound the search, because ROG-Map returns `KNOWN_FREE` (not occupied) for cells outside those planes.

2. **Connectivity**: 8-connected neighbors in xy at `inflation_resolution` (0.2 m) step.

3. **Collision check**: `map.isOccupiedInflate(p)`.

4. **Heuristic**: weighted Euclidean distance. Tiebreak prefers cells with larger `getDistance` (prefers wider corridors).

5. **Edge cases** (must handle gracefully):
   - Start or goal is `OUT_OF_MAP`: snap to nearest in-window free voxel along the goal-direction ray.
   - Start or goal is `OCCUPIED` after snap: publish `/dstar_status=false`, skip trajectory emit, wait for next replan tick.
   - Goal outside local window: log warning, snap to nearest free voxel at `local_window_radius − 1 m` in goal direction.

6. **Output**: coarse 3-D waypoints `q_0 … q_N` in map frame → feed to Phase D.

---

### Phase D — Back-End: ESDF-Driven Trajectory Optimization *(depends on C)*

**Goal**: smooth the A\* waypoints into a dynamically-feasible, obstacle-clearing trajectory.

1. **Parameterization**: uniform order-3 B-spline with control points `Q` initialized from front-end waypoints. The convex hull property guarantees that if all control points are in free space and clearance margins are satisfied, the entire spline is feasible.

2. **Cost function**:
   $$J = w_s J_\text{smooth} + w_o J_\text{obs} + w_d J_\text{dyn}$$
   - $J_\text{smooth} = \sum_i \|Q_{i+1} - 2Q_i + Q_{i-1}\|^2$ (jerk approximation)
   - $J_\text{obs} = \sum_t \max(0,\ d_\text{safe} - \text{ESDF}(p(t)))^2$, $t$ sampled every 0.05 s
   - $J_\text{dyn} = \sum_t \bigl[\max(0, \|v(t)\| - v_\text{max})^2 + \max(0, \|a(t)\| - a_\text{max})^2\bigr]$

3. **ESDF gradient**: $\nabla J_\text{obs}$ uses `evaluateFirstGrad` (chain rule through B-spline basis).

4. **Solver**: L-BFGS from vendored `lbfgs.hpp`. Cap: 50 iterations, 200 ms wall time. On non-convergence: emit raw front-end path with conservative (trapezoidal) time allocation.

5. **Thread-safety — CRITICAL**:
   - `ESDFMap::getDistance()` and `evaluateFirstGrad()` are **NOT** internally locked.
   - The 1 kHz internal ESDF update writes to `distance_buffer` while the planner reads it.
   - **Required mitigation**: at replan start, acquire `esdf_map_->update_esdf_mtx` (exposed via `SentryMap::getMutex()`) and hold it for the **entire** `lbfgs_optimize()` call. This is a **single-snapshot blocking approach**.
   - The "lock per outer L-BFGS iteration" approach is **insufficient** because L-BFGS line-search makes tens of gradient evaluations per outer iteration, all racing against ESDF updates between locks.
   - Expected cost: ≤ 2 ms blocking per replan (ESDF update is short; it blocks the map update during optimization). Acceptable at 5 Hz replanning rate.

6. **Time allocation**: trapezoidal velocity profile per segment, capped by `v_max`, `a_max`.

7. **Output**: `Trajectory` struct with `pos(t)`, `vel(t)`, `t_end` — stored as the active trajectory for Phase E.

---

### Phase E — Tracker / cmd_vel Publisher *(depends on D)*

**Goal**: 100 Hz conversion from active trajectory to `/cmd_vel` + `/dstar_status`, matching D\*-Lite's contract exactly.

1. **Time tracking**: `t_now = (ros::Time::now() − t_start).toSec()`. If `t_now > t_end` and `||robot − goal|| < arrival_radius` → set `arrived = true`.

2. **Velocity command**:
   - `vel_des = traj.vel(t_now) + Kp * (traj.pos(t_now) − robot_pos)` (cross-track correction)
   - TF lookup `map → virtual_frame` (same lookup as `dstarlite.cpp`); rotate `vel_des` (xy only) into `virtual_frame`.
   - Fill `cmd_vel.linear.{x, y}`; set `linear.z = angular.* = 0` (heading controlled by MCU/gimbal separately).

3. **Z-tilt safety scaling** — port from `dstarlite.cpp:523-525`:
   ```cpp
   double z_angle = atan2(vel_des.z, vel_des.head<2>().norm());
   double scale_x = std::clamp(1.0 - z_angle / (15.0 * M_PI / 180.0), 0.7, 1.7);
   cmd_vel.linear.x *= scale_x;
   ```
   Keep this ramp behaviour identical to the old planner.

4. **Publish rates** — must match existing MCU/BT contract exactly:
   - `/cmd_vel` (Twist, `virtual_frame`): 100 Hz
   - `/dstar_status` (Bool): 100 Hz
   - MCU bridge latches `/cmd_vel` at 50 Hz — publishing at 100 Hz is correct (bridge takes latest).

5. **Trajectory handover** when a new plan replaces the active one:
   - Keep both `traj_active` and `traj_pending`.
   - Linear blend reference position + velocity over 100 ms (10 ticks): `alpha = clamp((t − t_blend_start) / 0.1, 0, 1)`, output = lerp(active, pending, alpha).
   - When `alpha = 1`, drop `traj_active`. This prevents the cmd_vel discontinuity that causes chassis jerk on replans.

6. **Replan triggers**:
   | Condition | Action |
   |---|---|
   | New `/clicked_point` AND `||new_goal − old_goal|| > 0.05 m` | Trigger replan |
   | `t_now > t_end` and not arrived | Trigger replan |
   | Any trajectory sample in next 1 s has `dist < 0.05 m` (sampled at 0.1 s intervals) | Trigger replan immediately |

7. **Safety stops**:
   | Condition | Action |
   |---|---|
   | `now − getRobotState().rcv_time > 0.5 s` | Publish `cmd_vel=0`, `/dstar_status=false`, STOP |
   | Stage 1 `rog_map_node` not heard from in 1 s | Publish `cmd_vel=0`, `/dstar_status=false`, STOP |
   | Goal outside local window after snap attempt fails | Log warning, hold position |

   Note on `/clicked_point` delivery: `strategy_node.cpp PublishGoalPoint` (line 875) uses exact-equality comparison before publishing — any BT goal change is always forwarded. The `0.05 m` threshold in the planner is a guard against numerical noise only.

---

### Phase F — Wiring & Launch Integration *(depends on A–E)*

**Goal**: integrate `rog_planner` into the main launch system, keeping D\*-Lite as the default.

1. **New launch file**: `Old_nav/sim_nav/src/bot_sim/launch_real/rog_planner.launch`
   - Parameters (all exposed as launch args with defaults):
     ```
     safe_dist=0.4        (m)
     v_max=2.5            (m/s) — bench-test BEFORE using this; start at 1.5 m/s for first live test
     a_max=4.0            (m/s²)
     replan_rate=5        (Hz)
     arrival_radius=0.25  (m)
     chassis_height=0.3   (m)
     w_smooth=10
     w_obs=10000
     w_dyn=100
     ```

2. **Edit `3DNavUL_Test.launch`**: add a `planner` arg (`dstar` | `rog`) with default `dstar`:
   ```xml
   <arg name="planner" default="dstar"/>
   <!-- D*-Lite (default) -->
   <include if="$(eval arg('planner') == 'dstar')"
            file="$(find bot_sim)/launch_real/dstarlite.launch"/>
   <!-- ROG planner -->
   <include if="$(eval arg('planner') == 'rog')"
            file="$(find bot_sim)/launch_real/rog_planner.launch"/>
   ```
   Launches without the `planner` arg are **unaffected** (D\*-Lite runs as today).

3. **Stage 1 auto-enable**: when `planner:=rog`, `enable_rog_map_observer:=true` is set automatically (ROG planner requires the map node).

4. **Keep `dstarlite.cpp` and `dstarlite.launch`** intact — A/B test capability must be preserved.

---

### Phase G — Validation (Staged Deployment) *(depends on F)*

> **Do NOT skip levels.** Complete each level before proceeding to the next.

#### Level 1 — Unit tests (dev machine, no hardware)
- `SentryMap` subclass exposes ESDF accessors correctly.
- A\* finds a reachable goal in a synthetic ROG-Map.
- A\* fails gracefully (returns false, publishes `/dstar_status=false`) on an unreachable goal.
- Trajectory `eval` matches B-spline ground truth at sample times.
- L-BFGS converges on a 5-waypoint test; output stays ≥ `safe_dist − 0.05 m` from synthetic obstacles.

Run: `rostest rog_planner unit_sentrymap.test && rostest rog_planner unit_kino_astar.test && rostest rog_planner unit_lbfgs_smooth.test`

#### Level 2 — Bag replay (no MCU, no motors)
- Use `Old_nav/logs/rosbag_20260418_141634_0.bag` (confirmed present).
- If bag has `/aligned_points` + `/odom`: replay and run `rog_planner_node` headless.
- If bag has only raw `/livox/lidar` + `/livox/imu`: run `hdl_localization` in the loop.
- Assert: no trajectory sample has `getDistance(p) < 0.05 m` at any point in playback.

Run: `roslaunch rog_planner replay_validate.launch bag:=Old_nav/logs/rosbag_20260418_141634_0.bag`

#### Level 3 — Bench on NUC, motors disabled
- Full launch: `roslaunch 3DNavUL_Test.launch planner:=rog enable_rog_map_observer:=true`
- Motor power **disconnected**.
- Visualize `/cmd_vel` via `cmd_vel_marker` node.
- Cycle goals for 60 s.
- Performance gate: `top -p $(pgrep rog_planner)` shows < 70% CPU.

#### Level 4 — Live with kill-switch (physical robot, motors enabled)
- Human operator must be on tele-op kill-switch throughout.
- Required sub-tests:
  1. **Static navigation**: navigate to each occupy point in normal environment.
  2. **Dynamic obstacle**: person walks across path → reroute in < 1 s; remove obstacle → return to original path within 2 replans.
  3. **Mid-trajectory goal switch**: smooth handover (no chassis jerk).
  4. **Edge-of-map goal**: snap-to-nearest works, no crash or freeze.
  5. **MCU frame continuity**: `/nav_received` events remain steady throughout (50 Hz unbroken).
  6. **Speed check**: start at `v_max=1.5 m/s`; bench-test physical sentry limits before raising to 2.5 m/s.

#### Level 5 — Regression
- Relaunch with default args: `roslaunch 3DNavUL_Test.launch` (no `planner` arg).
- Behavior must be **identical to pre-Phase-3 baseline** (D\*-Lite path, timing, BT interaction).

#### Level 6 — A/B Comparison
- Same goal sequence, same starting pose, both planners on same recorded environment.
- Pass criteria (rog vs dstar):
  - Time-to-goal: `rog ≤ dstar × 1.1`
  - Path length: `rog ≤ dstar × 1.1`
  - Minimum clearance: `rog ≥ dstar` (strictly)
  - Replan rate: `rog ≤ 5 Hz` (within budget)

#### Rate gates (must pass before production)
| Topic | Required rate |
|---|---|
| `/cmd_vel` | 100 Hz |
| `/dstar_status` | 100 Hz |
| MCU nav frames | 50 Hz |
| `/rog_planner/trajectory` | ≥ 4 Hz |
| `/rog_map_node/rog_map/esdf` | ≥ 5 Hz |

> **Service gate**: do NOT switch `run_3DNavUL_Test_with_decision.sh` to `planner:=rog` until ALL above pass. Keep `planner:=dstar` as the production default for at least one full match cycle after the first live test.

---

## Decisions & Design Boundaries

| Topic | Decision |
|---|---|
| **ESDF thread safety** | Expose `update_esdf_mtx` via `SentryMap`; hold lock around entire `lbfgs_optimize()` call (snapshot approach, ~2 ms per replan). Per-iteration locking is insufficient. |
| **Static map fusion** | Bootstrap `innowing.pcd` via `load_pcd_en`. Verified: `rog_map.cpp` propagates PCD points to ESDF at startup. Caveat: `innowing.pcd` must be generated first. |
| **Search dimension** | A\* constrained to chassis-height layer (sentry doesn't fly). 3-D ESDF still used for clearance + gradient. |
| **L-BFGS license** | Vendor `lbfgs.hpp` from **libLBFGS** (Naoaki Okazaki, MIT). Do NOT use copy from EGO-Planner-v2 (GPL-3.0). |
| **Speed limits** | Default `v_max=2.5 m/s`. First live test: `v_max=1.5 m/s`. Physical limits unverified — bench-test required before raising. |
| **Heading** | `cmd_vel.angular.z = 0` always. Chassis is omnidirectional; gimbal yaw controlled separately. |
| **Frame contract** | Planner internal math: `map` frame. `/cmd_vel`: `virtual_frame`. `/clicked_point`: `map`. `/dstar_status`: bool. Unchanged from D\*-Lite. |
| **Fallback** | D\*-Lite NOT deleted. Kept as A/B fallback via `planner:=dstar` launch arg. |
| **Out of scope** | BT changes, MCU changes, static-map regeneration, dynamic-obstacle prediction, multi-goal queueing, formation/swarm. |

---

## Key File References

| File | Role |
|---|---|
| [Old_nav/sim_nav/src/bot_sim/src/dstarlite.cpp](../../Old_nav/sim_nav/src/bot_sim/src/dstarlite.cpp) | Reference: TF lookup, z-tilt scaling (L523–525), arrival logic (L753, 0.25 m), cmd_vel frame conversion (L512–524), 100 Hz publish |
| [Old_nav/sim_nav/src/bot_sim/launch_real/dstarlite.launch](../../Old_nav/sim_nav/src/bot_sim/launch_real/dstarlite.launch) | Launch arg template |
| [Old_nav/sim_nav/src/bot_sim/config/rog_map_passive.yaml](../../Old_nav/sim_nav/src/bot_sim/config/rog_map_passive.yaml) | ESDF + PCD-bootstrap flags to set |
| [Old_nav/3DNavUL_Test.launch](../../Old_nav/3DNavUL_Test.launch) | Top-level launch; add `planner` arg |
| [rogmap/ROG-Map/rog_map/include/rog_map/esdf_map.h](../../rogmap/ROG-Map/rog_map/include/rog_map/esdf_map.h) | ESDF API, mutex location |
| [rogmap/ROG-Map/rog_map/include/rog_map/prob_map.h](../../rogmap/ROG-Map/rog_map/include/rog_map/prob_map.h) | `getDistance(Vec3f)` |
| [rogmap/ROG-Map/rog_map/include/rog_map/rog_map.h](../../rogmap/ROG-Map/rog_map/include/rog_map/rog_map.h) | `isOccupiedInflate`, `getRobotState`, `boxSearchInflate` |
| [DecisionNode/src/decision_node/config/strategy_tree.xml](../../DecisionNode/src/decision_node/config/strategy_tree.xml) | BT — verify NO changes required |
| [DecisionNode/src/decision_node/src/mcu_communicator.cpp](../../DecisionNode/src/decision_node/src/mcu_communicator.cpp) | MCU bridge — verify `/cmd_vel` + `/dstar_status` consumer unchanged |
| `Old_nav/sim_nav/src/rog_planner/` | New package (to be created) |

---

## Verification Checklist

- [ ] `catkin_make --pkg rog_planner` compiles clean (warnings ≤ existing baseline)
- [ ] `rostest rog_planner unit_sentrymap.test` — ESDF accessor
- [ ] `rostest rog_planner unit_kino_astar.test` — front-end correctness
- [ ] `rostest rog_planner unit_lbfgs_smooth.test` — back-end converges, output ≥ `safe_dist − 0.05` from obstacles
- [ ] `roslaunch rog_planner replay_validate.launch bag:=...` — min clearance ≥ 0.05 m throughout
- [ ] `rostopic hz /cmd_vel` = 100, `/dstar_status` = 100, MCU nav = 50
- [ ] `/rog_planner/trajectory` ≥ 4 Hz during goal cycling
- [ ] A/B comparison: rog planner meets pass criteria on identical goal sequence
- [ ] Regression: `planner:=dstar` behavior identical to pre-Phase-3 baseline
- [ ] Service gate cleared: switch `run_3DNavUL_Test_with_decision.sh` only after all above
