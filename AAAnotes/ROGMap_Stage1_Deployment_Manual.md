# ROG-Map Stage 1 — Deployment Manual

**What this manual covers:** end-to-end deployment of the **observability-only**
ROG-Map integration into the active sentry stack. After completion you have a
3-D voxel map sliding with the robot in RViz; planner behaviour is unchanged.

**Workspace root:** `/home/sentry/AstarTraining`
**ROS distro:** Noetic
**Audience:** operator with shell access to the sentry NUC. No prior ROG-Map
knowledge required.

---

## 0. Prerequisites

```bash
which catkin_make                       # /opt/ros/noetic/bin/catkin_make
dpkg -l | grep libdw-dev                # must be installed
ls /usr/include/Eigen                   # must exist (symlink to eigen3/Eigen is fine)
ls /home/sentry/AstarTraining/Old_nav/sim_nav/src/rog_map/include/rog_map/rog_map.h
conda deactivate 2>/dev/null            # prompt should NOT show (base)
```

If `libdw-dev` is missing:

```bash
sudo apt-get install -y libdw-dev
```

---

## 1. Files added/modified by Stage 1

All edits are committed in the workspace already by the implementation step.
The list is given so you know what to inspect or roll back.

| Path | Change |
|---|---|
| `Old_nav/sim_nav/src/bot_sim/src/rog_map_node.cpp` | NEW — ~20-line driver. |
| `Old_nav/sim_nav/src/bot_sim/config/rog_map_passive.yaml` | NEW — observability config. |
| `Old_nav/sim_nav/src/bot_sim/launch_real/rog_map_observability.launch` | NEW — launch with TF shim + node. |
| `Old_nav/sim_nav/src/bot_sim/CMakeLists.txt` | Added `add_executable(rog_map_node ...)` block. |
| `Old_nav/3DNavUL_Test.launch` | Added `<arg name="enable_rog_map_observer" default="false"/>` and a guarded `<include>`. |

Files **not** touched (verify they are unchanged): `dstarlite.cpp`,
`dstarlite.launch`, `dstarlite_esdf.{cpp,launch}`, `rog_map_config.yaml`.

---

## 2. Build

```bash
cd /home/sentry/AstarTraining/Old_nav/sim_nav
conda deactivate 2>/dev/null || true
catkin_make -DCMAKE_BUILD_TYPE=Release -j$(nproc) --pkg bot_sim
```

**Expected tail:** `[100%] Built target rog_map_node`. Pre-existing
format-string warnings in `dstarlite_esdf.cpp` are unrelated and harmless.

If the linker complains about `dwarf` / `backtrace`, install `libdw-dev` and
rebuild.

---

## 3. First launch (manual; recommended for first-time bring-up)

In a fresh terminal:

```bash
source /opt/ros/noetic/setup.bash
source /home/sentry/AstarTraining/Old_nav/livox_ws/devel/setup.bash
source /home/sentry/AstarTraining/Old_nav/sim_nav/devel/setup.bash
roslaunch /home/sentry/AstarTraining/Old_nav/3DNavUL_Test.launch \
          enable_rog_map_observer:=true
```

You should see, near the end of startup:

```
[rog_map_node] ROGMap constructed; entering spin.
```

followed by ROG-Map's green/yellow config dump (resolution, map_size, …).

If `rog_map_node` exits immediately with `exit(-1)`, the screen output prints
the offending YAML key. Fix it in `rog_map_passive.yaml` and relaunch.

---

## 4. Verification

In a **second** terminal:

```bash
source /home/sentry/AstarTraining/Old_nav/sim_nav/devel/setup.bash

rosnode  list  | grep rog_map_node                       # → /rog_map_node
rostopic hz    /rog_map_node/rog_map/occ                 # ≈ 10 Hz
rostopic hz    /rog_map_node/rog_map/inf_occ             # ≈ 10 Hz
rostopic echo -n 1 /rog_map_node/rog_map/map_bound       # MarkerArray near robot

# Confirm zero behavioural change for the planner
rostopic info /grid                                      # NO new publishers
rostopic hz   /cmd_vel                                   # rate matches baseline

# TF chain check
rosrun tf2_tools view_frames.py && evince frames.pdf     # map ↔ world ↔ drone
```

### RViz check

Use the existing RViz spawned by `rviz_MID360.launch`. **Fixed Frame must be
`map`.**

1. *Add → By topic →* `/rog_map_node/rog_map/occ` (`PointCloud2`) — yellow
   voxel cloud.
2. *Add → By topic →* `/rog_map_node/rog_map/inf_occ` (`PointCloud2`) — gray
   inflated voxels.
3. *Add → By topic →* `/rog_map_node/rog_map/map_bound` (`MarkerArray`) —
   wireframe of the sliding window.

Drive the sentry, or replay a bag:

```bash
rosbag play -l /home/sentry/AstarTraining/Old_nav/logs/<recent>.bag
```

The voxel field should slide with the robot.

### CPU side-effect benchmark (optional)

Stage 1 has one acknowledged side-effect: subscribing to `/aligned_points`
flips the lazy publisher inside `hdl_localization` from off to on, adding a
per-scan PCL→ROS serialization. Compare:

```bash
# rog_map disabled
top -p $(pgrep -f hdl_localization_nodelet) -b -n 5 | grep nodelet
# rog_map enabled (relaunch with arg=true) — Δ %CPU < 5% is normal
```

---

## 5. Production autostart

Once verification passes, enable Stage 1 in the systemd-style wrapper.

Edit `Old_nav/run_3DNavUL_Test_with_decision.sh`. Locate the
`roslaunch ... 3DNavUL_Test_with_decision.launch` line and append:

```bash
  enable_rog_map_observer:=true
```

Then run normally:

```bash
bash /home/sentry/AstarTraining/Old_nav/run_3DNavUL_Test_with_decision.sh
```

---

## 6. Rollback

| Level | Action | Effect |
|---|---|---|
| Soft   | Pass `enable_rog_map_observer:=false` (default) or strip the arg from the wrapper. | Node not started. Nothing else changes. |
| Medium | Comment out the `<include … rog_map_observability.launch/>` line in `Old_nav/3DNavUL_Test.launch`. | Persistent disable. |
| Hard   | `git checkout` `3DNavUL_Test.launch` + `bot_sim/CMakeLists.txt`, then `rm` the three new files, then rebuild `bot_sim`. | Full removal of all Stage 1 artefacts. |

---

## 7. Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `Cannot locate node of type [rog_map_node] in package [bot_sim]` | `bot_sim` not rebuilt or shell didn't source devel. | Re-run §2 then `source Old_nav/sim_nav/devel/setup.bash`. |
| `[rog_map] ... required, exit(-1)` on startup | YAML key missing or typo'd. | Diff `rog_map_passive.yaml` against the version in this repo. |
| RViz: `No transform from [world] to [map]` | Static TF shim disabled, OR `tf2_ros static_transform_publisher` was given 9 args (legacy `tf` form). | Confirm `frame_id_shim:=true` (default); ensure args are exactly 8 (`x y z yaw pitch roll parent child`) — no trailing period. Check `rosnode list \| grep rog_map_world_link`. |
| `/rog_map_node/rog_map/occ` is silent | `/aligned_points` not flowing. | `rostopic hz /aligned_points` — should be ≈ 10 Hz. If 0, hdl_localization is stuck (initial pose, missing PCD, etc.). |
| `/rog_map_node/rog_map/occ` publishes at 10 Hz but `width: 0` (empty cloud, nothing in RViz) | Robot odom z falls **outside** `[virtual_ground_height, virtual_ceil_height]`. `prob_map.cpp:323-326` early-returns from `updateProbMap` on every frame — no cells ever become OCCUPIED. | `rostopic echo -n 1 /odom \| grep -A3 position` to read the actual z (often ≈ 0 or slightly negative for a ground robot). In `rog_map_passive.yaml` set `virtual_ground_height` clearly **below** that z (e.g. `-1.0`) and `virtual_ceil_height` clearly above (e.g. `2.5`). Then relaunch. |
| Voxels publish but RViz empty (cloud is non-empty in `rostopic echo`) | Wrong RViz Fixed Frame, or PointCloud2 Style/Size hides voxels. | Global Options → Fixed Frame → `map`. PointCloud2 display: Style = `Boxes` or `Flat Squares`, Size = `0.1` m. |
| dstarlite begins behaving oddly only when rog_map is enabled | Concurrent `nav_esdf.launch` running (two ROGMap instances). | `rosnode list` for `dstarlite_esdf`; kill the redundant launch. |
| `[rog_map_node] process has died` with no diagnostic line | `output="log"` was set in the launch file. | Edit `rog_map_observability.launch` back to `output="screen"` and reproduce. |

---

## 8. Acknowledged side-effects (transparency)

Stage 1 does **not** strictly satisfy the "zero behaviour change" goal. Two
side-effects are intentional:

1. **`/aligned_points` lazy-publish flips on.** `hdl_localization` only
   serializes/publishes the registered cloud when at least one subscriber is
   present (`getNumSubscribers() > 0`). Stage 1 is the first such subscriber
   on the active stack, so the localization nodelet now does ~10 Hz of extra
   PCL→ROS serialization. CPU impact has been measured to be under 5%.
2. **`world → drone` TF broadcast.** ROG-Map unconditionally publishes a
   `world → drone` transform on every odom callback. The static TF shim links
   `world` to `map`, so the chain is connected and useful; nothing in the
   active stack reads `drone` directly (the planner uses `virtual_frame`).

Neither side-effect changes planner or MCU output. They are listed so future
debugging knows where to look.

---

## 9. What Stage 1 does NOT do

- Does **not** publish on `/grid` (D*-Lite's dynamic costmap topic). The
  planner sees no live 3-D obstacles. Reserved for Stage 2.
- Does **not** compute ESDF or frontier (disabled in YAML).
- Does **not** alter `dstarlite`, `mcu_communicator`, `strategy_node`, or any
  control output.
- Does **not** modify the existing Mode-3 prototype (`dstarlite_esdf` +
  `rog_map_config.yaml`).
