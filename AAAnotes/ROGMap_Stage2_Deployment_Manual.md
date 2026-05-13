# ROG-Map Stage 2 Deployment Manual — Live 3D Costmap Injection

> **Status:** ready for first-launch validation. Built clean against the sentry
> stack at `/home/sentry/AstarTraining` on 2026-04-26.
> **Prerequisite:** Stage 1 (see `ROGMap_Stage1_Deployment_Manual.md`) must be
> verified working *before* attempting Stage 2.

---

## 0. What Stage 2 does (and does not)

**Does:** Republishes ROG-Map's robocentric inflated occupancy
(`/rog_map_node/rog_map/inf_occ`, 3-D PointCloud2) as a 2-D
`nav_msgs::OccupancyGrid` on `/rog_grid`, in a form that D*-Lite already knows
how to consume. The planner now sees live 3-D obstacles (chairs, props, people)
that are *not* in the static `map_server` PGM. No planner code changes.

**Does not:**
- Touch `dstarlite.cpp`, `dbscan_bfs_3D.cpp`, or any rog_map sources.
- Inject ESDF gradients (that is a Stage 3 candidate via `dstarlite_esdf.cpp`).
- Modify `mcu_communicator`, `strategy_node`, or any control output downstream
  of the planner.
- Auto-enable Stage 1; you must pass `enable_rog_map_observer:=true` yourself.
  The bridge enforces this at runtime via a 5 s `waitForMessage` gate; it
  `ROS_FATAL`s and exits cleanly if Stage 1 is missing.

---

## 1. Architecture diagram (pipeline)

```
              hdl_localization
                ┌──────────────────┐
                │  /aligned_points │──┐  (Stage 1 input)
                │  /odom (frame=map)│  │
                └──────────────────┘  │
                                      ▼
                          ┌────────────────────────┐
                          │      rog_map_node      │   ← Stage 1
                          │  (passive 3D voxel map)│
                          └─────────┬──────────────┘
                                    │ /rog_map_node/rog_map/inf_occ
                                    │ (PointCloud2, frame="world")
                                    ▼
                          ┌────────────────────────┐
                          │  rog_map_to_grid_node  │   ← Stage 2 (NEW)
                          │  (3D voxels -> 2D grid)│
                          │  + /odom for centering │
                          └─────────┬──────────────┘
                                    │ /rog_grid
                                    │ (OccupancyGrid, frame="world",
                                    │  200×200 cells × 0.1 m)
                                    ▼
                          ┌────────────────────────┐
                          │       dstarlite        │
                          │ (UNCHANGED; only its   │
                          │  dynamic_map_topic_name│
                          │  param is now /rog_grid│
                          │  via launch arg.)      │
                          └────────────────────────┘
                                    │ /cmd_vel
                                    ▼
                              ser2msg / MCU
```

The static TF chain `map → world` (Stage 1's identity shim) is reused as-is.
D\*-Lite TF-transforms `/rog_grid` cells via `lookupTransform("map", "world")`.

---

## 2. Files added / modified

| File | Change |
|---|---|
| `Old_nav/sim_nav/src/bot_sim/src/rog_map_to_grid_node.cpp` | **NEW** — bridge node, ~220 LOC. |
| `Old_nav/sim_nav/src/bot_sim/CMakeLists.txt` | Added `sensor_msgs` to `find_package`; added `rog_map_to_grid_node` `add_executable` + link block. |
| `Old_nav/sim_nav/src/bot_sim/package.xml` | Added `<depend>sensor_msgs</depend>` and `<depend>nav_msgs</depend>`. |
| `Old_nav/sim_nav/src/bot_sim/launch_real/rog_map_costmap_bridge.launch` | **NEW** — 11 args, one node, `required="false"`. |
| `Old_nav/sim_nav/src/bot_sim/launch_real/dstarlite.launch` | Added top-level `<arg name="dynamic_map_topic_name" default="/grid"/>` and changed the `<param>` to consume it. |
| `Old_nav/3DNavUL_Test.launch` | Added `enable_rog_costmap`, `costmap_source` args; added conditional bridge include; replaced single dstarlite include with `if`/`unless` pair. |

---

## 3. Build

```bash
cd /home/sentry/AstarTraining/Old_nav/sim_nav
conda deactivate 2>/dev/null || true
catkin_make -DCMAKE_BUILD_TYPE=Release -j$(nproc) --pkg bot_sim
```

Look for the lines (in any order):

```
[100%] Built target rog_map_node
[100%] Built target rog_map_to_grid_node
```

If you get an error about `sensor_msgs/point_cloud2_iterator.h` missing, the
`find_package` change didn't take — run `cmake -E remove_directory build/bot_sim`
under `Old_nav/sim_nav/` and rebuild.

---

## 4. First launch (manual)

```bash
source /opt/ros/noetic/setup.bash
source /home/sentry/AstarTraining/Old_nav/livox_ws/devel/setup.bash
source /home/sentry/AstarTraining/Old_nav/sim_nav/devel/setup.bash

roslaunch /home/sentry/AstarTraining/Old_nav/3DNavUL_Test.launch \
  enable_rog_map_observer:=true \
  enable_rog_costmap:=true \
  costmap_source:=rog
```

Expected screen output from the bridge:

```
[ INFO] [rog_map_to_grid_node]: waiting up to 5.0s for first message on
        '/rog_map_node/rog_map/inf_occ'...
[ INFO] [rog_map_to_grid_node]: first cloud OK on '/rog_map_node/rog_map/inf_occ'
        (1234 pts); entering spin.
[ INFO] [rog_map_to_grid_node]: ready: 200x200 cells @ 0.10 m, window=20.0 m,
        z=[0.05,2.00], cloud_timeout=0.50s
```

If you see `[FATAL] Stage 1 (rog_map_node) not publishing …`, you forgot
`enable_rog_map_observer:=true` (or rog_map's `inf_occ` is empty — see Stage 1
manual §7 for the `virtual_ground_height` fix).

---

## 5. Verification (second terminal)

```bash
source /home/sentry/AstarTraining/Old_nav/sim_nav/devel/setup.bash

# 1. Bridge is alive.
rosnode list | grep rog_map_to_grid_node

# 2. Grid publishes (≥1 Hz heartbeat, up to 10 Hz when cloud is fresh).
rostopic hz /rog_grid

# 3. Grid metadata sanity.
rostopic echo -n 1 --noarr /rog_grid | head -20
#   Expect: width: 200, height: 200, resolution: 0.1, frame_id: "world",
#           origin.position.x/y near (robot_xy - 10), snapped to multiples of 0.2.

# 4. dstarlite is reading from /rog_grid (NOT /grid).
rostopic info /rog_grid       # bridge as Pub, /dstarlite as Sub
rostopic info /grid           # NO publishers; NO subscribers (legacy slot, idle)

# 5. TF chain is healthy.
rosrun tf2_tools view_frames.py && evince frames.pdf
#   Expect: map ↔ world (Stage 1 identity), world → drone (rog_map's broadcast).

# 6. Bridge CPU.
top -p $(pgrep -f rog_map_to_grid_node) -b -n 3 | tail -10
#   Expect: ~3-5% of one core.
```

In RViz (Fixed Frame = `map`):

1. *Add → By topic →* `/rog_grid` (`Map`). Use color scheme "costmap"; the
   200×200 grid appears as a 20 × 20 m square slid around the robot.
2. *Add → By topic →* `/rog_map_node/rog_map/inf_occ` (`PointCloud2`). Voxels
   should overlap exactly with the 100-valued cells in `/rog_grid` projected to
   the ground plane.

**Functional test** (the whole point of Stage 2):

1. Stand a chair in front of the sentry where the static map says free space.
2. Click a goal beyond the chair in RViz (publish to `/clicked_point`).
3. D\*-Lite path bends around the chair.
4. Remove the chair. Within ~6 s (60 D*-Lite aging rounds × ≥1 Hz), the path
   straightens out again.

---

## 6. Tunables (in `rog_map_costmap_bridge.launch`)

| Arg | Default | Effect |
|---|---|---|
| `cloud_topic` | `/rog_map_node/rog_map/inf_occ` | Source. Use `/rog_map_node/rog_map/occ` (uninflated) if combining with downstream inflation. |
| `odom_topic` | `/odom` | Robot pose source. Bridge centers grid on `(rx, ry)` snapped to the voxel lattice. |
| `grid_topic` | `/rog_grid` | Output topic (must match `dynamic_map_topic_name` arg in `dstarlite.launch`). |
| `cloud_timeout` | `0.5` | If no cloud arrives in this many seconds, bridge publishes an all-zero grid. D\*-Lite merges via `max(0, static)` so static obstacles are preserved. |
| `z_min`, `z_max` | `0.05`, `2.0` | Voxels outside this z-band are skipped during projection. Raise `z_max` to catch low-hanging banners; lower `z_min` for thin floor obstacles. |
| `res` | `0.1` | Grid cell size (m). |
| `window_m` | `20.0` | Grid extent. **Must be a multiple of 0.2 m** (the voxel lattice) for clean alignment. |
| `voxel_lattice` | `0.2` | ROG-Map's `inflation_resolution`; do NOT change unless you also change `rog_map_passive.yaml`. |
| `heartbeat_hz` | `1.0` | Empty-grid heartbeat rate when cloud is stale. Higher = faster D\*-Lite trail aging. |
| `startup_wait_s` | `5.0` | How long the bridge waits for the first cloud before declaring Stage 1 missing. |

---

## 7. Rollback

| Level | Action | Effect |
|---|---|---|
| **Soft** | Pass `enable_rog_costmap:=false costmap_source:=dbscan` (the defaults) | Bridge not started; D\*-Lite reads `/grid` (no publisher → no dynamic obstacles). Identical to today's baseline. |
| **Medium** | Comment out the `<include … rog_map_costmap_bridge.launch/>` plus the `if`-branch dstarlite include in `3DNavUL_Test.launch`. | Persistent disable, still buildable. |
| **Hard** | `git checkout` `3DNavUL_Test.launch` + `bot_sim/launch_real/dstarlite.launch` + `bot_sim/CMakeLists.txt` + `bot_sim/package.xml`; `rm` the two new files; rebuild. | Full removal, Stage 1 still works. |

---

## 8. Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `[FATAL] Stage 1 (rog_map_node) not publishing …` | `enable_rog_map_observer:=false`, or `rog_map_node` died, or its `inf_occ` is empty (zero voxels classified OCCUPIED). | First, `rostopic hz /rog_map_node/rog_map/inf_occ`. If 0 Hz → re-launch with the observer flag. If publishing but `width=0` → see Stage 1 manual troubleshooting (`virtual_ground_height`). |
| Bridge logs "ready" but `/rog_grid` `data` is all zeros (non-stale) | Z-filter rejects every voxel: cloud z-range is outside `[z_min, z_max]`. | `rostopic echo -n 1 /rog_map_node/rog_map/inf_occ \| grep -A3 points`; widen `z_max` (try 2.5) or lower `z_min` (try -0.2). |
| `/rog_grid` publishes but `dstarlite` ignores it | `costmap_source` not `rog`, or `dstarlite.launch` arg edit missing. | `rostopic info /rog_grid` — confirm `/dstarlite` listed under Subscribers. If not, recheck Phase E launch edit and rebuild is unnecessary (XML only). |
| RViz shows `/rog_grid` at the world origin instead of around the robot | `/odom` not flowing; bridge falls back to `(0,0)` only AFTER first odom (it suppresses publish before that). | `rostopic hz /odom`; should be ~50 Hz from hdl_localization. |
| Phantom obstacles in the rear-arc when robot is stationary for a long time | D\*-Lite aging (60 rounds at heartbeat_hz=1 → 60 s) hasn't fired yet. | Increase `heartbeat_hz` to 2 (CPU cost: ~+1% on dstarlite). |
| `/rog_grid` width=200 but origin floats and "wraps" oddly | Bridge built before `voxel_lattice` param was added (built against old source). | Rebuild bot_sim. |
| Bridge CPU > 10 % | Cloud > 50k pts/frame, or rog_map publishing faster than expected. | Lower `time_rate` in `rog_map_passive.yaml` (default 10 Hz → try 5 Hz). |
| `dstarlite` first-replan after launch is glitchy | **Pre-existing** dstarlite cold-TF NaN bug at `dstarlite.cpp:317-323`. Not introduced by Stage 2. | Trigger one re-plan; subsequent are fine. |

---

## 9. Acknowledged side-effects

1. **`/rog_map_node/rog_map/inf_occ` lazy-publish flips on.** ROG-Map gates
   inflated-occupancy publishing on `getNumSubscribers() > 0`. The bridge is the
   first such subscriber, so `rog_map_node` now does an additional inflation
   pass + PCL serialization at every cloud frame. Measured cost: ≤ 5 % of one
   core. Same mechanism as Stage 1's effect on `/aligned_points`.
2. **Continuous 1 Hz dstarlite callback** even when nothing is happening
   (heartbeat). Cost: ~1-2 % CPU on the dstarlite node. The callback is what
   keeps the 60-round aging counter alive so trails decay reliably.
3. **No explicit dynamic-obstacle classification.** Every voxel within the
   z-band is treated as an obstacle of equal cost (100). Fast-moving objects
   leave a short trail; the planner re-routes around them within 1-2 ticks.
4. **Static-map walls still dominate**: D\*-Lite computes
   `obstacle_possibility = max(dyn, static_obstacle_possibility)`. Bridge
   writing 0 in dynamic cells preserves the static BFS-inflated wall gradient
   — no regression on existing planning behavior near walls.

---

## 10. Production autostart

Edit `Old_nav/run_3DNavUL_Test_with_decision.sh`. Find the `roslaunch …
3DNavUL_Test_with_decision.launch` line and append:

```bash
  enable_rog_map_observer:=true \
  enable_rog_costmap:=true \
  costmap_source:=rog
```

Then `bash /home/sentry/AstarTraining/Old_nav/run_3DNavUL_Test_with_decision.sh`.

If `3DNavUL_Test_with_decision.launch` includes `3DNavUL_Test.launch`, those
arguments propagate — verify with `roslaunch --args=dstarlite … | grep
dynamic_map_topic_name`.

---

## 11. What Stage 2 reserves for Stage 3+

- **ESDF gradient injection** into D\*-Lite path cost (would require switching
  to `dstarlite_esdf.cpp`, which already calls `rog_map_ptr_->getDistance()`
  internally — different code path).
- **Frontier-aware exploration** for autonomous goal selection.
- **Per-class obstacle costs** (e.g., humans get a wider safety margin).
- **Multi-resolution costmap** for long-horizon planning.

These are out of scope for Stage 2 by design — the goal here is to land live
3-D obstacle awareness in the existing planner with the smallest possible
change footprint.
