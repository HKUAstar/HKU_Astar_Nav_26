# Phase 3 — ROG Planner Deployment Guide
## Operator Reference (v1.0, 2026-04-26)

> **Purpose**: Step-by-step guide for building, running, testing, and rolling back the `rog_planner` package. Written for the sentry operator / integration engineer.
> **Related**: [Phase3_RogPlanner_Plan.md](Phase3_RogPlanner_Plan.md) (technical design) · [ROGMap_Stage1_Deployment_Manual.md](ROGMap_Stage1_Deployment_Manual.md)

---

## Prerequisites

Before touching any code, verify these conditions:

| Check | Command | Expected |
|---|---|---|
| Stage 1 (ROG-Map observer) working | `rostopic echo /rog_map_node/rog_map/inf_occ` during a live run | Occupied cells visible in RViz |
| Stage 2 (dynamic costmap) working | `rostopic hz /grid` during a live run | ~10 Hz |
| ROS Noetic sourced | `echo $ROS_DISTRO` | `noetic` |
| Conda deactivated | `conda info --envs` | No `*` on any env other than `base` (or deactivate) |
| libdw-dev installed | `dpkg -l libdw-dev` | Installed |
| Eigen symlink exists | `ls /usr/include/Eigen` | Directory listing |
| Build workspace known | `echo $CATKIN_WS` or note it is `Old_nav/sim_nav` | Set correctly |

---

## Part 1 — YAML Preparation (Phase A)

These are **YAML-only** changes to `rog_map_passive.yaml`. Make them **before** writing any C++ code; the smoke test in Step 4 validates them independently.

### 1.1 Enable ESDF

File: `Old_nav/sim_nav/src/bot_sim/config/rog_map_passive.yaml`

Change (under `esdf:` key):
```yaml
esdf:
  enable: true           # was: false
  resolution: 0.2
  local_update_box: [12.0, 12.0, 4.0]   # must be ≤ raycasting/local_update_box = [14,14,8]
```

### 1.2 Generate `innowing.pcd`

The static field PCD **is not in the workspace** — it must be generated before `load_pcd_en` can be used.

**Option A — from a fresh mapping run (recommended):**
```bash
# Terminal 1: start mapping
roslaunch Old_nav/hdl_graph_slam_mapping.launch

# Drive the robot around the full field until the map stabilizes.
# Terminal 2: save map
rosservice call /hdl_graph_slam/save_map "resolution: 0.1" "destination: /home/sentry/AstarTraining/innowing.pcd"
# Or, if save_map_on_update.py is running, it writes mapping_result.pcd automatically.
cp ~/mapping_result.pcd ~/AstarTraining/innowing.pcd
```

**Option B — from existing `innowing.pgm` (quick, 2-D only):**
```bash
# Use pcd2pgm_package in PCD output mode (see that package's README).
# Result is a 2-D extruded cloud — adequate for ground-floor planning.
```

**Downsample if > 1 M points** (avoids slow startup):
```bash
cd /home/sentry/AstarTraining
# Use pcl_octree_compression or a simple voxel filter node.
# Alternatively, during hdl_graph_slam: set resolution=0.1 in the save_map call.
```

### 1.3 Enable PCD Bootstrap

```yaml
load_pcd_en: true
pcd_name: "/home/sentry/AstarTraining/innowing.pcd"   # absolute path
```

### 1.4 Smoke-Test ESDF

Build and launch Stage 1 only:
```bash
cd Old_nav/sim_nav && catkin_make -DBUILD_TYPE=Release
source devel/setup.bash
roslaunch 3DNavUL_Test.launch enable_rog_map_observer:=true
```

In a second terminal, check ESDF is publishing:
```bash
rostopic echo /rog_map_node/rog_map/esdf | head -20
```

In RViz: add `PointCloud2` display on `/rog_map_node/rog_map/esdf`. Cells near walls should show short distances (red/orange in distance colormap); open-field cells should be green/blue.

If ESDF topic is absent → check `esdf.enable: true` is saved. If PCD fails to load → check `pcd_name` is an absolute path and the file exists.

---

## Part 2 — Building `rog_planner` (Phases B–E)

> This section is a reference for the developer implementing the package. The operator runs these commands after the developer delivers the source code.

### 2.1 First Build

```bash
conda deactivate   # CRITICAL — conda breaks catkin_make

cd Old_nav/sim_nav
catkin_make --pkg rog_planner -DBUILD_TYPE=Release 2>&1 | tee /tmp/build_rog.log

# Expected: zero errors; warnings ≤ existing dstarlite baseline
grep -c "error:" /tmp/build_rog.log   # should be 0
```

If `rog_map` headers not found → ensure `rog_map` is in the same catkin workspace (`Old_nav/sim_nav/src/`). If it is in a different workspace, source that workspace's `setup.bash` first.

### 2.2 Incremental Rebuild

```bash
catkin_make --pkg rog_planner -DBUILD_TYPE=Release
```

---

## Part 3 — Running the Planner

### 3.1 Planner-only launch (for testing, not production)

```bash
source Old_nav/sim_nav/devel/setup.bash
roslaunch bot_sim rog_planner.launch \
  v_max:=1.5 \
  safe_dist:=0.4 \
  arrival_radius:=0.25
```

Key launch arguments and their defaults:

| Argument | Default | Notes |
|---|---|---|
| `safe_dist` | `0.4` | Minimum clearance for ESDF cost (m) |
| `v_max` | `2.5` | **Start at 1.5 for first live test** |
| `a_max` | `4.0` | Max acceleration (m/s²) |
| `replan_rate` | `5` | Hz — planner timer frequency |
| `arrival_radius` | `0.25` | Goal-reached threshold (m) |
| `chassis_height` | `0.3` | A\* z-clamp (m) |
| `w_smooth` | `10` | Smoothness cost weight |
| `w_obs` | `10000` | Obstacle cost weight |
| `w_dyn` | `100` | Dynamics cost weight |

### 3.2 Full stack — ROG planner active

```bash
roslaunch 3DNavUL_Test.launch \
  planner:=rog \
  enable_rog_map_observer:=true
```

### 3.3 Full stack — D\*-Lite (default, production)

```bash
roslaunch 3DNavUL_Test.launch
# or explicitly:
roslaunch 3DNavUL_Test.launch planner:=dstar
```

No behavioural change from pre-Phase-3.

---

## Part 4 — Unit Tests

Run all unit tests before any hardware testing:

```bash
source Old_nav/sim_nav/devel/setup.bash

# Test 1: SentryMap ESDF accessor
rostest rog_planner unit_sentrymap.test
# Pass: ESDF dist > 0 in free space, < 0 inside wall; no crash on out-of-bounds query.

# Test 2: Kino A* correctness
rostest rog_planner unit_kino_astar.test
# Pass: finds path to reachable goal; returns false + no crash on unreachable goal.

# Test 3: L-BFGS optimizer
rostest rog_planner unit_lbfgs_smooth.test
# Pass: converges within 50 iters; all samples ≥ safe_dist - 0.05 m from obstacles.
```

If any test fails → **stop; do not proceed to bag replay or hardware**.

---

## Part 5 — Bag Replay Validation

```bash
source Old_nav/sim_nav/devel/setup.bash
roslaunch rog_planner replay_validate.launch \
  bag:=$(pwd)/Old_nav/logs/rosbag_20260418_141634_0.bag
```

This launch:
1. Replays the bag (providing `/aligned_points` + `/odom`).
2. Runs `rog_planner_node` headless.
3. Logs minimum ESDF distance along all generated trajectories to `/tmp/min_dist.log`.

**Pass gate**: all values in `/tmp/min_dist.log` ≥ 0.05 m.

```bash
awk 'NF && $1 < 0.05 {print "FAIL:", $0}' /tmp/min_dist.log
# Should print nothing (no output = pass)
```

---

## Part 6 — Bench Test (NUC, Motors Disabled)

Before any live robot test, bench on the NUC with motor power disconnected:

```bash
# Disconnect motor power cable before this step.
roslaunch 3DNavUL_Test.launch \
  planner:=rog \
  enable_rog_map_observer:=true

# In a second terminal — monitor CPU
top -p $(pgrep rog_planner)
```

Pass criteria:
- No crashes over 60 s of goal cycling.
- `rog_planner` CPU < 70%.
- `rostopic hz /cmd_vel` ≈ 100 Hz.
- `rostopic hz /dstar_status` ≈ 100 Hz.
- `rostopic hz /rog_planner/trajectory` ≥ 4 Hz.
- No jerk in `/cmd_vel` visualization on goal switch.

---

## Part 7 — Live Robot Test

> **SAFETY**: Human operator must be on tele-op kill-switch throughout. **Do NOT skip bench test.**

### 7.1 First live run — conservative speed

```bash
roslaunch 3DNavUL_Test.launch \
  planner:=rog \
  enable_rog_map_observer:=true \
  v_max:=1.5
```

Observe:
- Sentry navigates to first goal without collision.
- `/cmd_vel` continuous at 100 Hz (`rostopic hz`).
- MCU nav frames steady at 50 Hz.

### 7.2 Dynamic obstacle test

- Send robot toward a goal.
- Walk a person across the path mid-trajectory.
- Verify: reroute within 1 s; after person clears, robot returns to path within 2 replans.

### 7.3 Goal switch test

- While robot is moving, issue a new goal via BT/`/clicked_point`.
- Verify: smooth transition — no chassis jerk visible or felt.

### 7.4 Edge-of-map goal

- Issue a goal at the edge of the field (near walls).
- Verify: planner snaps to nearest free voxel; no crash or freeze; `/dstar_status=true` when arrived.

### 7.5 Rate monitoring (run in parallel terminal)

```bash
rostopic hz /cmd_vel /dstar_status &
rostopic hz /rog_planner/trajectory &
```

### 7.6 Speed increase

Once 7.1–7.5 pass at `v_max=1.5`:
- Run bench test with `v_max=2.0`, then `v_max=2.5`.
- Verify MCU can track at higher speed (no timing gaps in nav frames).
- Physical sentry chassis limits must be measured — if chassis oscillates or tracks poorly, **reduce `v_max` or `a_max`**.

---

## Part 8 — Regression Test

After each live test session, verify the default stack is still working:

```bash
roslaunch 3DNavUL_Test.launch
# (no planner arg = dstar default)
```

Run the same goal sequence used in Phase 7. The behaviour must be identical to the pre-Phase-3 baseline.

---

## Part 9 — A/B Comparison

Once both planners are validated individually, run a direct comparison:

```bash
# Run 1: D*-Lite
roslaunch 3DNavUL_Test.launch planner:=dstar
# Record: time_to_goal per waypoint, path length, min clearance (log from rog_map ESDF), replan events

# Run 2: ROG planner (same goal sequence, same start pose)
roslaunch 3DNavUL_Test.launch planner:=rog enable_rog_map_observer:=true v_max:=1.5
# Record same metrics
```

Pass criteria (`rog` vs `dstar`):

| Metric | Gate |
|---|---|
| Time to goal | `rog ≤ dstar × 1.1` |
| Path length | `rog ≤ dstar × 1.1` |
| Min clearance | `rog ≥ dstar` (strictly better) |
| Replan rate | ≤ 5 Hz |

---

## Part 10 — Production Switch

> **Only after ALL above pass.**

Edit `run_3DNavUL_Test_with_decision.sh`:
```bash
# Change:
roslaunch 3DNavUL_Test.launch ...
# To:
roslaunch 3DNavUL_Test.launch planner:=rog enable_rog_map_observer:=true v_max:=1.5 ...
```

Keep `planner:=dstar` as the commented-out fallback line in the script for at least one full match cycle.

---

## Part 11 — Rollback Procedure

If `rog_planner` misbehaves at any point:

### Immediate (during a run)
1. Operator activates kill-switch.
2. In launch terminal: `Ctrl+C`.
3. Relaunch with default: `roslaunch 3DNavUL_Test.launch` (D\*-Lite, no planner arg).

### Between sessions
```bash
# Revert 3DNavUL_Test.launch change (remove planner arg):
# Edit the file to remove <arg name="planner" .../> — or just don't pass it.
# D*-Lite is the default; no code deletion needed.

# Revert rog_map_passive.yaml ESDF changes (if ESDF causes instability):
# esdf.enable: false
# load_pcd_en: false
# catkin_make (no source changes; YAML is loaded at runtime)
```

The `rog_planner` package can be left in the workspace — it is only activated by `planner:=rog`. Removing or ignoring it has no effect on the D\*-Lite stack.

---

## Quick Reference — Topic Checklist

| Topic | Type | Publisher | Consumer | Required rate |
|---|---|---|---|---|
| `/clicked_point` | `PointStamped` (map frame) | BT `strategy_node` | `rog_planner` | Event |
| `/cmd_vel` | `Twist` (virtual_frame) | `rog_planner` | MCU bridge | 100 Hz |
| `/dstar_status` | `Bool` | `rog_planner` | BT `CheckArrived`, MCU | 100 Hz |
| `/rog_planner/trajectory` | custom | `rog_planner` | RViz debug | ≥ 4 Hz |
| `/rog_map_node/rog_map/esdf` | `PointCloud2` | `rog_map_node` | `rog_planner` | ≥ 5 Hz |
| `/aligned_points` | `PointCloud2` | `hdl_localization` | `rog_map_node` | 10 Hz |
| `/odom` | `Odometry` | `hdl_localization` | `rog_map_node`, `rog_planner` | 100 Hz |

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `rog_map/esdf` not publishing | `esdf.enable` still false | Check `rog_map_passive.yaml`, rebuild |
| PCD not loading | `pcd_name` is relative path | Use absolute path in YAML |
| ESDF values all -1 or 0 | PCD not triggering `updateESDF3D` | Check `rog_map.cpp` PCD load log; ensure file > 0 bytes |
| A\* returns no path | Start/goal outside window or occupied | Increase `map_size`; check inflation settings |
| Replan rate < 4 Hz | L-BFGS timeout too short or too many control points | Reduce `w_obs` (less iterations), or reduce waypoint density |
| `/cmd_vel` drops below 100 Hz | Tracker timer blocked by planner mutex | Separate tracker and planner threads; verify mutex is only in planner, not tracker |
| Chassis jerk on goal switch | Trajectory blend not active | Verify `traj_pending` / `traj_active` handover in `tracker.cpp` |
| MCU nav frames stall | `cmd_vel` rate dropped | Check `rog_planner` CPU; `replan_rate` too high |
| Build error: `lbfgs.hpp` not found | Not vendored yet | Copy `lbfgs.hpp` from libLBFGS (MIT) into `include/rog_planner/third_party/` |
| Build error: `esdf_map_` inaccessible | Missing subclass | Ensure planner uses `SentryMap` (subclass of `ROGMap`), not `ROGMap` directly |
