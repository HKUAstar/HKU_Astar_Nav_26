# Phase 3 — `rog_planner` Hands-On Tutorial

> A step-by-step walkthrough for an operator who has never touched
> `rog_planner` before. Companion to **Phase3_RogPlanner_Manual.md**
> (which is the reference); this file is the "do this in order" guide.

---

## 0. What you will achieve

By the end of this tutorial:
1. You will have built the new package and the (slightly-patched) `rog_map`.
2. You will have launched the sentry stack with `planner:=rog`.
3. You will have published a goal via RViz and watched the robot navigate
   under the new planner (motors disconnected, then on a kill-switch).
4. You will know how to flip back to `dstarlite` instantly.
5. You will know what each visualization topic in RViz means.

---

## 1. Prerequisites

- Ubuntu 20.04, ROS Noetic.
- A working `Old_nav` stack — i.e. you can already run `dstarlite` today.
- Live LiDAR (Livox MID-360) attached, or a recorded bag.
- The sandbox at `/home/sentry/AstarTraining/rogmap/Old_nav_backup/`.

If you haven't synced the backup folder recently:

```bash
rsync -av --exclude='*.pcd' --exclude='*.bag' \
    /home/sentry/AstarTraining/Old_nav/ \
    /home/sentry/AstarTraining/rogmap/Old_nav_backup/
```

---

## 2. Quick mental model (60 seconds)

Today's stack:
```
LiDAR ─▶ hdl_localization ─▶ /odom + /aligned_points
                                  │
                                  ▼
                            (optional) ROG-Map observer ─▶ viz
                                  │
                            /grid (legacy) or /rog_grid (Stage 2 bridge)
                                  │
                                  ▼
                              dstarlite ─▶ /cmd_vel + /dstar_status
```

After Phase 3 with `planner:=rog`:
```
LiDAR ─▶ hdl_localization ─▶ /odom + /aligned_points
                                  │
                                  ▼
                          rog_planner_node
                            (own ROGMap+ESDF inside)
                                  │
                              ─▶ /cmd_vel + /dstar_status   (same as before)
                              ─▶ ~rog_planner/{path,trajectory}  (new viz)
                              ─▶ ~rog_planner/rog_map/esdf  (new viz)
```

The ROG-Map *observer* node is automatically suppressed when
`planner:=rog`, because the planner's own internal ROGMap covers the same
viz topics.

---

## 3. Build it

### 3.1 First-time build of the sandbox

```bash
cd /home/sentry/AstarTraining/rogmap/Old_nav_backup/sim_nav
source /opt/ros/noetic/setup.bash
catkin_make --only-pkg-with-deps rog_planner -j4
```

Why `--only-pkg-with-deps`? The sandbox has FAST_LIO / Point-LIO etc.
that depend on a `livox_ros_driver2` whose CMake config files contain a
stale absolute path from a build on another machine
(`/home/sentry_train_test/...`). Building only the planner and `rog_map`
is enough — none of the LIO packages are needed at run time when you use
`hdl_localization` (which is the project default).

Expected last lines of output:

```
[100%] Built target rog_map
[100%] Built target rog_planner_node
```

If you see `Could not find a package configuration file provided by
"livox_ros_driver2"`, you forgot the `--only-pkg-with-deps` flag.

### 3.2 Source the workspace

```bash
source devel/setup.bash
which rog_planner_node    # should point inside devel/lib/rog_planner/
```

### 3.3 Sanity check

```bash
roscat rog_planner launch/rog_planner.launch | head -30
```

If `roscat` fails with "package 'rog_planner' not found", the `setup.bash`
sourcing step did not happen.

---

## 4. Smoke test — no robot, no LiDAR

The point of this step is to confirm the node *starts* without crashing
even with no inputs. We bring up just the planner and check that it
publishes zero `/cmd_vel` and `/dstar_status=false` because there's no
odom yet.

Terminal 1:

```bash
roscore
```

Terminal 2 (still in `Old_nav_backup/sim_nav` with `setup.bash` sourced):

```bash
roslaunch rog_planner rog_planner.launch
```

You should see (within a few seconds):

```
[ INFO] [rog_planner] initialized (planner@5.0Hz, tracker@100.0Hz)
 -- [ROGMap] Init successfully -- .
 -- [ESDFMap] Init successfully -- .
[ WARN] [rog_planner] stale RobotState; skip plan   (every 2 s; expected)
```

Terminal 3:

```bash
rostopic hz /cmd_vel       # should print ~100 Hz
rostopic hz /dstar_status  # should print ~100 Hz
rostopic echo -n 1 /cmd_vel        # all zero
rostopic echo -n 1 /dstar_status   # data: False
```

Kill all three terminals once those checks pass.

---

## 5. Real bring-up — full sentry stack

### 5.1 Default behavior (legacy D\*-Lite)

Sanity-check that nothing regressed:

```bash
roslaunch <top-level path> 3DNavUL_Test.launch
```

(no `planner:=` arg, so `dstar` default applies). RViz should come up,
hdl_localization should converge, dstarlite should respond to RViz
"Publish Point" events on `/clicked_point` with a `/cmd_vel` arrow. If
this works exactly as before, your Phase 3 patch did not break the
default path.

Kill it.

### 5.2 ROG-Map planner — first launch

```bash
roslaunch <top-level path> 3DNavUL_Test.launch planner:=rog
```

Watch the screen log. Expected milestones:

| Time after launch | What you should see |
|--|--|
| 0 s   | `[ROGMap] Init successfully` |
| 0 s   | `[ESDFMap] Init successfully` |
| ~1 s  | `[rog_planner] initialized (planner@5.0Hz, tracker@100.0Hz)` |
| ~1 s  | hdl_localization status messages |
| 2-5 s | First `/aligned_points` arrives — ROG-Map starts populating ESDF |
| 5+ s  | `/rog_planner_node/rog_map/esdf` topic starts publishing |

**Do not click a goal until the ESDF viz is populated** — for the first
few seconds the distance field is identically 0 everywhere, which makes
the smoother think the robot is buried inside an obstacle.

### 5.3 RViz set-up

Add these displays:

| Display | Topic | Color |
|--|--|--|
| PointCloud2 | `/rog_planner_node/rog_map/inf_occ` | red |
| PointCloud2 | `/rog_planner_node/rog_map/esdf` | rainbow (z-axis) |
| Path | `/rog_planner_node/path` | yellow |
| Path | `/rog_planner_node/trajectory` | green |
| Marker | `/cmd_vel_marker` | blue arrow |

Fixed Frame: `map`.

You should see the inflated occupancy (red dots) update as the robot
moves, and the ESDF (a horizontal slab of colored dots) gradient from
walls outward.

### 5.4 Send a goal

In RViz, use the "Publish Point" tool to drop a goal somewhere a few
meters in front of the robot. Watch:

1. **Yellow path** appears within ~200 ms (front-end A\*).
2. **Green trajectory** appears just after (smoothed B-spline; smoother
   than the yellow A\* polyline).
3. **Blue cmd_vel arrow** points in the direction of motion.
4. After the robot reaches the goal, `/dstar_status` flips to `True`
   (`rostopic echo /dstar_status` will show `data: True`).

If the robot does not move (motors connected), but `/cmd_vel` shows
non-zero numbers — something downstream (MCU bridge or motors) is at
fault, not the planner.

### 5.5 Drop test (dynamic obstacle)

While following a goal, walk across the planned path. Expected:

- The green trajectory should curve away within < 1 s (one or two
  replan ticks).
- `cmd_vel` should not zero out — the planner reroutes, it doesn't stop.
- After you step away, the trajectory straightens within 1-2 replan
  ticks.

If the robot stops dead (`cmd_vel` zeros), something locked up the
projected-collision check; file a bug.

---

## 6. With static-PCD bootstrap

If you have an `innowing.pcd` ready (see Manual §7.3 for how to make
one), pass it as a launch arg:

```bash
roslaunch <top-level path> 3DNavUL_Test.launch \
    planner:=rog \
    static_pcd:=/home/sentry/AstarTraining/Old_nav/maps/innowing.pcd
```

Effect: the ESDF is *pre-populated with wall priors at startup*, so you
can publish a goal immediately — no need to wait the 5 s for the first
LiDAR sweep to color in the field.

**Caveat**: if the path is wrong, ROG-Map prints `Load pcd file failed!`
in red and the node exits with status -1. The launch screen is the only
place you'll see this — `roslaunch` will then re-spawn the node forever
(the default respawn behavior of stuck nodes), which looks like a
healthy boot. Always check the screen log on first launch with a new
`static_pcd:=` argument.

---

## 7. Tuning knobs

These are exposed as launch args on `rog_planner.launch`:

```bash
roslaunch <top-level path> 3DNavUL_Test.launch planner:=rog \
    v_max:=1.5 a_max:=4.0 safe_dist:=0.4 \
    arrival_radius:=0.25 \
    w_smooth:=10 w_obs:=10000 w_dyn:=100
```

Quick-reference effects:

| Knob | If you raise it | If you lower it |
|--|--|--|
| `v_max` | faster robot (cap higher) | slower, safer |
| `safe_dist` | bigger buffer to walls | tighter cornering |
| `w_obs` | obstacle avoidance is hard-er | smoother through narrow gaps but riskier |
| `w_smooth` | smoother trajectory | tracks A\* polyline more closely |
| `w_dyn` | sharper velocity cap enforcement | velocity may briefly exceed v_max |
| `arrival_radius` | "arrived" earlier | requires getting closer |
| `replan_rate` | reacts to dynamics faster, more CPU | less CPU, slower reaction |

For the **first live test, do NOT raise `v_max` above 1.5 m/s**. Bench
the chassis dynamics at 1.5, then 2.0, then 2.5.

---

## 8. Rollback

If anything seems wrong, get back to the legacy stack instantly:

```bash
# Just relaunch without planner:=rog
roslaunch <top-level path> 3DNavUL_Test.launch
```

That's it — nothing in the legacy code path was modified. Default arg is
`dstar`, and the wiring in `3DNavUL_Test.launch` only includes
`rog_planner.launch` when `planner == 'rog'`.

If you suspect the `rog_map` patch (`getUpdateMtx()`) broke something:

```bash
cd /home/sentry/AstarTraining/rogmap/Old_nav_backup/sim_nav/src/rog_map
git diff include/rog_map/esdf_map.h     # see exactly what changed
```

Should be a 1-line addition. Revert with `git checkout` if needed; no
other downstream code references `getUpdateMtx()` outside of
`rog_planner`.

---

## 9. Troubleshooting cheat-sheet

| Symptom | Likely cause | Fix |
|--|--|--|
| `Could not find package 'livox_ros_driver2'` during build | catkin_make tried to build FAST_LIO etc. | Use `--only-pkg-with-deps rog_planner` |
| Node exits immediately, screen log shows `Load pcd file failed!` | `static_pcd:=` path is wrong or file missing | Use absolute path; verify `ls -l <path>` |
| `/dstar_status` never becomes `True` even at goal | (was a bug; fixed in this drop) Arrival used 3-D norm with z-mismatch | Confirm you're running the patched code; arrival is now xy-only |
| Yellow A\* path appears, green trajectory does not | Smoother failed (e.g. timeout, infeasible) | Look for `[rog_planner] smoother failed` warn; raise `safe_dist` or lower `w_obs` |
| ESDF viz never populates | LiDAR not arriving, or ROGMap raycast box too small | Check `rostopic hz /aligned_points`; verify `raycasting/local_update_box` in yaml |
| Robot jerks when goal switches | Blend window too short | Raise `blend_time` (default 0.1 s) |
| Tracker `/cmd_vel` rate drops below 100 Hz | AsyncSpinner thread starvation | Already raised to 4 threads; if still bad, profile with `top -H -p $(pgrep rog_planner_node)` |
| Plan looks correct but robot drifts off it | Cross-track gain `Kp` (hardcoded 1.0) too low for chassis lag | Increase in source (no launch arg today) and rebuild |
| `[rog_planner] stale RobotState; skip plan` repeats | `/odom` not arriving or hdl_localization not converged | Check hdl_localization, IMU bias initialization |
| TF warning about `world` frame | Two static publishers fighting | Verify the observer launch is **not** included (`enable_rog_map_observer:=false` or default) |

---

## 10. What success looks like (acceptance demo)

For the deployment review, run:

```bash
roslaunch <top-level path> 3DNavUL_Test.launch planner:=rog v_max:=1.5
```

Then in another terminal:

```bash
# Capture rate stability
rostopic hz /cmd_vel /dstar_status /rog_planner_node/trajectory
```

Drive a 3-goal sequence (RViz "Publish Point" three times to opposite
corners of your test arena), with one human walking across the path each
leg. Acceptance criteria:

- Each goal reached, `/dstar_status` flips to `True` momentarily.
- `/cmd_vel` rate stays at 100 ± 2 Hz, no gaps > 50 ms.
- No `[rog_planner] smoother failed` messages during the run.
- CPU < 70 % on a single core (`top -p $(pgrep rog_planner_node)`).
- min `getDistance` along executed path > 0.1 m (read off ESDF viz; or
  add a logger).

If all four pass, you're ready to schedule a full A/B comparison vs
D\*-Lite on the same goal sequence.

---

## 11. Where to look next

- **Manual** (`Phase3_RogPlanner_Manual.md`) — full reference, parameter
  table, cost equations, file diff list.
- **Plan** (`Phase3_RogPlanner_Plan.md`) — original design doc, decisions
  and out-of-scope list.
- **Deployment guide** (`Phase3_RogPlanner_Deployment_Guide.md`) — original
  operator-facing onboarding doc that predates this implementation.
- **Source** — start from
  [sim_nav/src/rog_planner/src/rog_planner_node.cpp](rogmap/Old_nav_backup/sim_nav/src/rog_planner/src/rog_planner_node.cpp);
  the `PlannerNode` class is small and orchestrates everything.
