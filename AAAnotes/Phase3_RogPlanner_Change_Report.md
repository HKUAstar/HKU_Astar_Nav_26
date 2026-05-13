# Phase 3 ROG Planner Change Report

## 1. Purpose of this report

This report explains what changed in the navigation system during and after Phase 3, why those changes were made, and how the updated system now works from sensor input to final `/cmd_vel` output.

The goal is to make the new system understandable for both developers and operators:

- Developers should be able to see the code structure, ownership boundaries, safety logic, and planning pipeline.
- Operators should be able to understand what each launch option does, what RViz displays mean, and how to fall back to the old planner.
- Future maintainers should be able to see why the final design uses both ROG-Map and the old static `/map` layer.

The implementation currently lives in the Phase 3 sandbox:

```text
/home/sentry/AstarTraining/rogmap/Old_nav_backup
```

The report files live in:

```text
/home/sentry/AstarTraining/AAAnotes
```

---

## 2. Executive summary

Phase 3 adds a new planner package named `rog_planner`. It is a drop-in alternative to the legacy D*-Lite planner.

The old planner is still available and remains the default:

```bash
roslaunch 3DNavUL_Test.launch planner:=dstar
```

The new Phase 3 planner is selected explicitly:

```bash
roslaunch 3DNavUL_Test.launch planner:=rog
```

At a high level, the change is this:

```text
Old system:
  map / grid input -> D*-Lite -> /cmd_vel + /dstar_status

Phase 3 system:
  Livox + odom -> internal ROG-Map ESDF
  static /map  -> hard 2-D wall layer
  both maps    -> A* front end -> B-spline smoother -> 100 Hz tracker
              -> /cmd_vel + /dstar_status
```

The new planner keeps the same external control interface as D*-Lite:

- It subscribes to `/clicked_point` for goals.
- It publishes `/cmd_vel` for motion control.
- It publishes `/dstar_status` for BehaviorTree arrival checking.
- It works in the existing `map` / `virtual_frame` TF setup.

The most important final design decision is that ROG-Map is not used alone. The old static PGM map, published as `/map`, is now used as a hard wall barrier. ROG-Map and ESDF handle live obstacles and distance costs, while the static `/map` prevents paths from crossing walls that are missing from the live point cloud or static PCD.

---

## 3. Why Phase 3 was needed

The legacy D*-Lite planner worked, but it had several limitations:

1. It depended on a 2-D grid view of the world.
2. It did not use ROG-Map's ESDF distance field directly.
3. Its path output was not smoothed into a continuous velocity-friendly trajectory.
4. It could not easily use 3-D obstacle information from the Livox/ROG-Map pipeline.
5. It was difficult to reason about dynamic replanning and trajectory safety once obstacles changed.

Phase 3 was designed to improve this without breaking the rest of the robot stack. The new planner keeps the same outside interface, so the BehaviorTree, MCU bridge, `/clicked_point` goal flow, `/cmd_vel`, and `/dstar_status` all continue to work.

The major internal changes are:

- Add an internal ESDF-enabled ROG-Map instance inside the planner.
- Add A* search over the robot's chassis-height planning slice.
- Add a B-spline + L-BFGS smoother for continuous trajectories.
- Add static `/map` hard-wall checking.
- Add segment-level collision validation to prevent wall crossing.
- Add active trajectory safety behavior so the robot stops if a currently-followed trajectory becomes unsafe and replanning fails.

---

## 4. Scope of the updated navigation system

### 4.1 In scope

Phase 3 updates the local/global motion planner part of the sentry navigation stack.

It covers:

- Goal handling from `/clicked_point`.
- Map ingestion from live point clouds and `/odom` through ROG-Map.
- Static map ingestion from `/map` OccupancyGrid.
- Front-end path search.
- Back-end trajectory smoothing.
- Collision validation.
- Velocity tracking.
- RViz visualization for planner outputs.
- Runtime launch selection between legacy D*-Lite and new ROG planner.

### 4.2 Out of scope

Phase 3 does not replace the full robot stack.

It does not replace:

- Livox driver bring-up.
- HDL localization.
- map_server.
- BehaviorTree decision logic.
- MCU communication.
- IMU filtering.
- Real robot TF publication.
- The old D*-Lite code path.

It also does not remove the old planner. D*-Lite remains the default and can be selected at launch time.

---

## 5. Main files changed or added

### 5.1 New package: `rog_planner`

Location:

```text
rogmap/Old_nav_backup/sim_nav/src/rog_planner
```

Structure:

```text
rog_planner/
├── CMakeLists.txt
├── package.xml
├── include/rog_planner/
│   ├── astar.h
│   ├── bspline_smoother.h
│   ├── sentry_map.h
│   └── static_map_2d.h
├── launch/
│   └── rog_planner.launch
└── src/
    ├── astar.cpp
    ├── bspline_smoother.cpp
    ├── rog_planner_node.cpp
    └── static_map_2d.cpp
```

### 5.2 Planner node

File:

```text
rogmap/Old_nav_backup/sim_nav/src/rog_planner/src/rog_planner_node.cpp
```

Responsibilities:

- Owns the internal ROG-Map instance.
- Owns the static map layer.
- Receives goals from `/clicked_point`.
- Runs planner timer at about 5 Hz.
- Runs tracker timer at 100 Hz.
- Publishes `/cmd_vel`.
- Publishes `/dstar_status`.
- Publishes visualization paths.
- Checks current trajectory safety.
- Clears active trajectory when replanning fails after a safety violation.

### 5.3 ROG-Map wrapper

File:

```text
rogmap/Old_nav_backup/sim_nav/src/rog_planner/include/rog_planner/sentry_map.h
```

`SentryMap` is a small subclass of `rog_map::ROGMap`. It exposes the ESDF distance, ESDF gradient, and ESDF update mutex to the planner code.

This exists because the planner needs safe access to ESDF data while ROG-Map's internal update timer is also writing to that data.

### 5.4 A* front end

Files:

```text
rogmap/Old_nav_backup/sim_nav/src/rog_planner/include/rog_planner/astar.h
rogmap/Old_nav_backup/sim_nav/src/rog_planner/src/astar.cpp
```

Responsibilities:

- Search at fixed chassis height.
- Use ROG-Map inflated occupancy.
- Use static `/map` walls as hard obstacles.
- Penalize low-clearance cells.
- Snap blocked start or goal points toward nearby free space when possible.
- Prevent diagonal corner cutting.
- Check each motion segment at fine resolution so the planner cannot jump through thin walls.

### 5.5 B-spline smoother

Files:

```text
rogmap/Old_nav_backup/sim_nav/src/rog_planner/include/rog_planner/bspline_smoother.h
rogmap/Old_nav_backup/sim_nav/src/rog_planner/src/bspline_smoother.cpp
```

Responsibilities:

- Convert the A* waypoint path into a uniform cubic B-spline.
- Optimize control points with L-BFGS.
- Penalize unsmooth curves.
- Penalize low ESDF clearance.
- Penalize proximity to static map walls.
- Penalize velocities above `v_max`.
- Reject trajectories that intersect static map walls.

### 5.6 Static map layer

Files:

```text
rogmap/Old_nav_backup/sim_nav/src/rog_planner/include/rog_planner/static_map_2d.h
rogmap/Old_nav_backup/sim_nav/src/rog_planner/src/static_map_2d.cpp
```

Responsibilities:

- Subscribe to the old map_server `/map` OccupancyGrid.
- Interpret occupied cells above a threshold as walls.
- Inflate those walls slightly.
- Answer `isOccupied(x, y)` queries.
- Answer clearance and gradient queries for smoothing.
- Treat positions outside the map as occupied by default.

This is the final fix for the wall-crossing issue. ROG-Map and PCD data may miss walls if the point cloud does not contain enough wall geometry, so the old PGM map is now the reliable hard-wall prior.

### 5.7 ROG planner map config

File:

```text
rogmap/Old_nav_backup/sim_nav/src/bot_sim/config/rog_planner_map.yaml
```

Important final values:

```yaml
rog_map:
  resolution: 0.05
  inflation_resolution: 0.05
  inflation_step: 0
  virtual_ceil_height: 1.5
  virtual_ground_height: -0.5
  esdf:
    enable: true
    resolution: 0.2
```

The most important value here is:

```yaml
inflation_step: 0
```

This means ROG-Map no longer adds extra hard inflation cells. That avoids closing narrow corridors. Static `/map` provides the hard wall layer, while ESDF and smoother costs provide soft clearance behavior.

### 5.8 Top-level launch file

File:

```text
rogmap/Old_nav_backup/3DNavUL_Test.launch
```

Important changes:

- Adds `planner:=dstar|rog`.
- Keeps `dstar` as the default.
- Includes `rog_planner.launch` only when `planner:=rog`.
- Automatically disables the passive ROG-Map observer when `planner:=rog` to avoid double-processing `/aligned_points`.
- Keeps the existing map_server, Livox, HDL localization, IMU, TF, and decision bridge flow.

### 5.9 RViz config

File:

```text
rogmap/Old_nav_backup/rog_planner.rviz
```

Purpose:

- Preserve the useful old visualization groups.
- Add Phase 3 planner displays.
- Show A* path.
- Show smoothed B-spline trajectory.
- Show ROG-Map ESDF / inflated occupancy outputs.
- Show robot frames such as `virtual_frame` and `gimbal_frame`.

### 5.10 TF fix for `gimbal_frame`

Files:

```text
rogmap/Old_nav_backup/sim_nav/src/bot_sim/src/real_robot_transform.cpp
Old_nav/sim_nav/src/bot_sim/src/real_robot_transform.cpp
```

Runtime issue fixed:

```text
For frame [gimbal_frame]: Frame [gimbal_frame] does not exist
```

The fix makes `real_robot_transform` publish `gimbal_frame` even before localization fully succeeds, using identity or the last known rotation as needed. This keeps RViz and downstream TF consumers from losing that frame during startup.

---

## 6. Updated navigation pipeline

### 6.1 Whole-system pipeline

The updated system can be understood as this pipeline:

```text
Livox MID-360
  -> livox_ros_driver2
  -> point cloud preprocessing / alignment
  -> /aligned_points

IMU + localization stack
  -> hdl_localization
  -> /odom
  -> map-related TF

map_server
  -> /map OccupancyGrid from old PGM

BehaviorTree / decision node
  -> /clicked_point goal

rog_planner_node
  -> internal ROG-Map + ESDF
  -> static map hard wall layer
  -> A* path search
  -> B-spline trajectory smoothing
  -> final safety validation
  -> trajectory tracker
  -> /cmd_vel
  -> /dstar_status
```

### 6.2 Planner-internal pipeline

Inside `rog_planner_node`, one planning cycle looks like this:

```text
1. Read current goal
2. Read current robot pose from ROG-Map RobotState
3. Check whether static /map is ready
4. Decide whether replanning is needed
5. Run A* front end
6. Publish front-end path for RViz
7. Run B-spline smoother
8. Validate final trajectory
9. Install trajectory
10. Tracker converts trajectory into /cmd_vel at 100 Hz
```

A simplified version:

```text
/clicked_point + robot pose
        |
        v
  A* at chassis height
        |
        v
  waypoint path
        |
        v
  B-spline + L-BFGS
        |
        v
  safe smooth trajectory
        |
        v
  100 Hz tracker
        |
        v
      /cmd_vel
```

---

## 7. Mapping and obstacle model

The final planner uses two map sources at the same time.

### 7.1 Dynamic/live map: ROG-Map

ROG-Map receives:

- `/aligned_points`
- `/odom`

It builds:

- Occupancy information.
- Inflated occupancy information.
- ESDF distance field.
- ROG-Map visualization topics.

The planner uses ROG-Map for:

- Live obstacle awareness.
- ESDF distance queries.
- ESDF gradient queries used by smoothing.
- Projected collision checks along the active trajectory.

### 7.2 Static hard wall map: `/map`

The old PGM map is published by map_server as `/map`.

The planner's `StaticMap2D` subscribes to this topic and uses it for:

- Hard wall blocking in A*.
- Wall clearance cost in the smoother.
- Final trajectory validation.
- Active trajectory collision checks.

This layer is important because real point clouds or static PCD files can miss walls. If a wall is missing from point cloud data, a pure ROG-Map planner may think that wall is free space. The PGM map prevents this.

### 7.3 Why both maps are needed

ROG-Map is good at live 3-D obstacle information. Static `/map` is good at guaranteed field-wall structure.

Using only ROG-Map caused paths to cross walls when the wall was absent from point cloud/PCD data.

Using only static `/map` would ignore live obstacles.

The final design uses both:

```text
ROG-Map / ESDF:
  dynamic obstacles, live sensed geometry, smooth clearance gradients

Static /map:
  reliable known walls, field boundary, hard no-crossing layer
```

---

## 8. A* front-end behavior

The A* planner searches in a 2-D grid at constant chassis height.

Although the package describes the planner as ROG-Map-aware 3-D planning, the actual sentry robot does not fly. Therefore, the search state is:

```text
(ix, iy, z = chassis_height)
```

This is intentional. It lets the planner use 3-D map and ESDF information while producing a ground robot path.

### 8.1 Blocking rules

A cell or point is blocked if either condition is true:

```text
static /map says occupied
OR
ROG-Map inflated occupancy says occupied
```

In code terms, the A* blocker is conceptually:

```text
blocked = static_map.isOccupied(x, y) || rog_map.isOccupiedInflate(x, y, z)
```

### 8.2 Clearance preference

A* does not only avoid occupied cells. It also prefers cells with more clearance.

It adds a small penalty when:

- ESDF distance is below `safe_dist`.
- Static map clearance is below `static_map_safe_dist`.

This helps paths prefer wider corridors instead of scraping close to walls.

### 8.3 Start and goal snapping

If the start or goal lies inside an occupied or inflated cell, the planner tries to snap it to nearby free space along the start-goal direction.

This is useful when:

- The clicked goal is slightly inside an inflation band.
- Localization places the robot barely inside a mapped obstacle.
- The goal is close to a wall but still operationally intended.

If snapping fails, A* fails and the planner does not install a trajectory.

### 8.4 Segment-level transition checking

A major safety fix was added after wall-crossing behavior was observed.

A grid planner can appear safe at grid nodes but still cross through a wall between nodes. This is especially common with diagonals or thin obstacles.

The final A* transition check now:

- Rejects the target cell if blocked.
- Prevents diagonal corner cutting by checking the two side cells.
- Samples the movement segment every 0.025 m.
- Rejects the transition if any sample is blocked.

This prevents paths from slipping through wall corners or thin map features.

---

## 9. B-spline smoothing behavior

The A* result is a waypoint path. Waypoint paths are usually jagged, so Phase 3 smooths the result into a continuous trajectory.

The smoother uses:

```text
uniform cubic B-spline + L-BFGS optimization
```

### 9.1 Inputs and outputs

Input:

```text
A* waypoints in map frame
```

Output:

```text
B-spline trajectory at fixed chassis height
```

The trajectory can be evaluated for:

- Position at time `t`.
- Velocity at time `t`.

The 100 Hz tracker uses these values to generate `/cmd_vel`.

### 9.2 Cost terms

The smoother optimizes a cost with four main ideas:

```text
1. Smoothness
2. ESDF obstacle clearance
3. Static map wall clearance
4. Velocity limit
```

In plain language:

- Smoothness makes the curve less jagged.
- ESDF cost pushes the trajectory away from live obstacles.
- Static map cost pushes the trajectory away from known walls.
- Velocity cost discourages speeds above `v_max`.

### 9.3 Endpoint pinning

The first two and last two B-spline control points are pinned.

This keeps the trajectory attached to the intended start and goal and reduces endpoint drift during optimization.

### 9.4 No unsafe fallback

An important safety change was made here.

Earlier versions could fall back to a raw or partially smoothed trajectory if the smoother rejected the optimized result. That is unsafe because a raw spline can cut corners through obstacles.

The final behavior is:

```text
If smoother rejects the trajectory, do not install a fallback.
```

If the robot was already following a trajectory that has now become unsafe, the active trajectory is cleared so the tracker publishes zero velocity.

---

## 10. Runtime tracker behavior

The tracker runs at 100 Hz.

Its job is not to search. Its job is to follow the currently installed B-spline trajectory and publish velocity commands.

### 10.1 Tracker inputs

The tracker uses:

- Current robot state from ROG-Map.
- Current active trajectory.
- Current goal.
- TF from `map` to `virtual_frame`.

### 10.2 Tracker output

The tracker publishes:

```text
/cmd_vel geometry_msgs/Twist
```

Only planar velocity is used:

```text
linear.x
linear.y
```

The output is transformed into `virtual_frame`, matching the old D*-Lite/MCU interface.

### 10.3 Arrival status

The tracker publishes:

```text
/dstar_status std_msgs/Bool
```

It publishes `true` when the robot is within `arrival_radius` of the goal in XY distance.

The check is XY-only because clicked goals often have `z = 0`, while localization and chassis height may place the robot at a different z value. A full 3-D distance check would incorrectly prevent arrival from triggering.

### 10.4 Stale odom behavior

If robot state is stale for more than about 0.5 seconds:

```text
/cmd_vel = zero
/dstar_status = false
```

This is safer than continuing to drive without a reliable pose.

---

## 11. Replanning behavior

The planner timer runs at about 5 Hz.

Replanning is triggered when any of these conditions is true:

- A new goal is received.
- There is no active trajectory.
- The active trajectory has expired.
- The active trajectory is predicted to collide with ROG-Map obstacles.
- The active trajectory is predicted to collide with static `/map` walls.

### 11.1 Projected collision checking

The planner samples the remaining active trajectory.

If a future point is too close to an ESDF obstacle or inside a static wall, the active trajectory is marked invalid and replanning is requested.

### 11.2 Successful replan

If the current trajectory is still valid, the new trajectory can be installed as pending and blended over a short time window.

If the current trajectory is already invalid, the new trajectory replaces it immediately. This avoids blending through an unsafe old path.

### 11.3 Failed replan after active trajectory becomes unsafe

This is one of the most important safety behaviors.

If the current trajectory is unsafe and A* or smoothing fails, the planner clears the active trajectory.

Then the tracker has no trajectory to follow and publishes zero velocity.

This prevents the robot from continuing along a path that the planner already knows is unsafe.

---

## 12. Static map hard-wall integration

This section deserves special attention because it was the biggest correction after initial Phase 3 testing.

### 12.1 Original problem

The planner sometimes crossed walls.

The root cause was not simply A* or smoothing. The deeper issue was map representation:

- ROG-Map can only block geometry it knows about.
- Static PCD files can miss walls.
- Live point clouds can fail to observe some wall surfaces.
- ESDF cannot create a wall where the input map has no wall.

So a path could look valid to ROG-Map even though it crossed a wall in the old field PGM map.

### 12.2 Final fix

The old `/map` OccupancyGrid is now a hard obstacle source.

The planner uses it in three places:

1. A* node and segment blocking.
2. B-spline static wall cost.
3. Final trajectory validation and active trajectory checks.

### 12.3 Inflation choice

Static map inflation is small:

```text
static_map_inflation_radius = 0.05 m
```

ROG-Map extra hard inflation is disabled:

```text
inflation_step = 0
```

This keeps narrow corridors open while still preventing actual wall crossing.

The safety model becomes:

```text
Hard no-crossing:
  static /map occupied cells + small inflation
  ROG occupied cells

Soft preference:
  ESDF clearance cost
  static map clearance cost
```

---

## 13. Launch behavior

### 13.1 Legacy default

Running without planner args keeps old behavior:

```bash
roslaunch 3DNavUL_Test.launch
```

Equivalent to:

```bash
roslaunch 3DNavUL_Test.launch planner:=dstar
```

This launches D*-Lite and does not activate the new planner.

### 13.2 Phase 3 planner

To run the new planner:

```bash
roslaunch 3DNavUL_Test.launch planner:=rog
```

This launches:

- map_server
- imu_filter
- Livox driver
- hdl_localization
- real_robot_transform
- rog_planner_node
- ser2msg decision/goal bridge

It does not launch D*-Lite.

### 13.3 Static PCD argument

The launch file supports:

```bash
roslaunch 3DNavUL_Test.launch planner:=rog static_pcd:=/absolute/path/to/file.pcd
```

This can preload a static PCD into ROG-Map.

However, the final wall safety does not depend only on this PCD. The static `/map` layer remains the hard wall authority.

### 13.4 ROG-Map observer interaction

When `planner:=rog`, the old passive ROG-Map observer is disabled automatically.

Reason:

- `rog_planner_node` already owns its own ROG-Map instance.
- Running both would double-process `/aligned_points`.
- Running both could create confusing duplicate visualization and TF behavior.

---

## 14. ROS interface summary

### 14.1 Inputs

| Topic | Type | Purpose |
|---|---|---|
| `/clicked_point` | `geometry_msgs/PointStamped` | Goal from BehaviorTree/RViz |
| `/aligned_points` | `sensor_msgs/PointCloud2` | Live cloud input to internal ROG-Map |
| `/odom` | `nav_msgs/Odometry` | Robot pose input to internal ROG-Map |
| `/map` | `nav_msgs/OccupancyGrid` | Static hard-wall layer |
| TF `map -> virtual_frame` | TF | Velocity transform for output frame |

### 14.2 Outputs

| Topic | Type | Purpose |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity command, compatible with old MCU bridge |
| `/dstar_status` | `std_msgs/Bool` | Arrival status, compatible with old BehaviorTree |
| `~path` | `nav_msgs/Path` | A* path visualization |
| `~trajectory` | `nav_msgs/Path` | Smoothed trajectory visualization |
| ROG-Map visualization topics | various | Occupancy/ESDF visualization |
| `/cmd_vel_marker` | marker | Operator RViz velocity arrow |

---

## 15. Important parameters

### 15.1 Planner parameters

| Parameter | Typical value | Meaning |
|---|---:|---|
| `chassis_height` | `0.3` | Fixed z height for planning |
| `safe_dist` | `0.4` | Desired ESDF clearance |
| `v_max` | `1.5` launch default | Velocity cap for first live tests |
| `a_max` | `4.0` | Acceleration-related limit, parsed for timing/future dynamics |
| `arrival_radius` | `0.25` | XY goal arrival threshold |
| `replan_rate` | `5.0` | Planner timer rate |
| `tracker_rate` | `100.0` | Command publishing rate |
| `astar_resolution` | `0.1` | A* grid step |
| `collision_d_min` | `0.05` | Active trajectory collision threshold |

### 15.2 Static map parameters

| Parameter | Typical value | Meaning |
|---|---:|---|
| `static_map_enable` | `true` | Enable `/map` hard-wall layer |
| `static_map_topic` | `/map` | OccupancyGrid source |
| `static_map_occ_threshold` | `65` | Occupancy value considered wall |
| `static_map_inflation_radius` | `0.05` | Small hard-wall inflation |
| `static_map_safe_dist` | `0.25` | Soft clearance around static walls |
| `static_map_unknown_as_occupied` | `false` | Unknown cells are not blocked |
| `static_map_outside_as_occupied` | `true` | Outside map bounds is blocked |
| `w_static_obs` | `50000.0` | Static wall smoothing cost weight |

### 15.3 ROG-Map parameters

| Parameter | Final value | Meaning |
|---|---:|---|
| `resolution` | `0.05` | Occupancy resolution |
| `inflation_resolution` | `0.05` | Inflated occupancy resolution |
| `inflation_step` | `0` | No extra hard ROG inflation |
| `virtual_ground_height` | `-0.5` | Ignore deep floor noise below this |
| `virtual_ceil_height` | `1.5` | Ignore high points above this |
| `esdf.enable` | `true` | Required for planner |
| `esdf.resolution` | `0.2` | ESDF query resolution |

---

## 16. Runtime issues resolved during Phase 3

### 16.1 Missing `gimbal_frame`

Error:

```text
For frame [gimbal_frame]: Frame [gimbal_frame] does not exist
```

Cause:

`gimbal_frame` was not always published during startup or before localization became valid.

Fix:

`real_robot_transform` now broadcasts `gimbal_frame` continuously, using identity or the last valid rotation until localization data is ready.

### 16.2 Missing `bot_sim` launch nodes in backup

Problem:

The backup sandbox did not initially have every built `bot_sim` executable available in `devel`.

Fix/workaround:

The needed `bot_sim` binaries were copied from the working `Old_nav` devel space into the backup devel space so the backup launch could run.

### 16.3 Livox `bind failed`

Problem:

Livox driver reported bind failure.

Diagnosis:

This was caused by a stale process or socket still holding the Livox port, not by the planner implementation.

Operational fix:

Stop the old process or restart the driver/network binding cleanly.

### 16.4 `aft_mapped` disconnected from `map`

Problem:

TF involving `aft_mapped` and `map` could be disconnected.

Diagnosis:

Localization was not receiving or producing the expected LiDAR/localization data path.

The planner depends on valid odom/TF, so if localization is stale, it stops rather than driving blindly.

### 16.5 PCL missing normal/curvature warnings

Problem:

PCL warnings appeared about missing fields such as normal or curvature.

Diagnosis:

These warnings were not planner blockers. The relevant navigation flow uses point positions and occupancy/ESDF behavior, not those specific fields.

### 16.6 Excessive inflation blocked A*

Problem:

The robot start or usable corridors could become occupied because hard inflation was too large.

Fix:

ROG hard inflation was reduced to no extra cells:

```yaml
inflation_step: 0
```

Static `/map` now provides reliable hard walls, and soft costs handle clearance.

### 16.7 Wall crossing

Problem:

A* and/or smoothing could produce paths that crossed walls.

Final fixes:

- Static `/map` hard-wall layer.
- A* diagonal corner-cut prevention.
- A* segment sampling every 0.025 m.
- Smoother static map cost.
- Smoother static collision rejection.
- Final trajectory segment validation.
- Active trajectory clearing if emergency replanning fails.

---

## 17. Safety improvements after initial implementation

The final implementation includes several safety hardening changes beyond the first Phase 3 version.

### 17.1 Static map hard barrier

The old PGM map is now treated as a hard collision source. This prevents wall crossing even if point cloud data is incomplete.

### 17.2 Fine transition checks

A* now checks motion segments, not only grid nodes. This closes the gap where a diagonal or long step could pass through an obstacle.

### 17.3 Diagonal corner-cut prevention

For diagonal moves, the planner checks the adjacent side cells. If either side cell is blocked, the diagonal move is rejected.

### 17.4 No raw smoother fallback

If smoothing produces an unsafe result or is rejected, the planner does not install a fallback trajectory.

### 17.5 Final trajectory validation

Before a trajectory becomes active, it is sampled in time and by segment. If any point or segment is unsafe, the trajectory is rejected.

### 17.6 Active trajectory clearing

If the active trajectory becomes unsafe and replanning fails, the active trajectory is cleared. The tracker then publishes zero velocity.

### 17.7 Immediate replacement when old trajectory is unsafe

If the old trajectory is invalid but a new safe trajectory is found, the planner replaces immediately instead of blending from the unsafe old trajectory.

---

## 18. Build and validation status

The planner package was built successfully with:

```bash
cd /home/sentry/AstarTraining/rogmap/Old_nav_backup/sim_nav
source /opt/ros/noetic/setup.bash
catkin_make --only-pkg-with-deps rog_planner -j4
```

Final build result observed:

```text
[100%] Built target rog_planner_node
```

The targeted package build is important because the full sandbox workspace contains unrelated Livox/LIO packages with stale CMake paths. Building only `rog_planner` and its dependencies avoids those unrelated failures.

A final review pass reported no blocking issues after the static map and trajectory safety fixes.

---

## 19. How to operate the updated system

### 19.1 Build

```bash
cd /home/sentry/AstarTraining/rogmap/Old_nav_backup/sim_nav
source /opt/ros/noetic/setup.bash
catkin_make --only-pkg-with-deps rog_planner -j4
source devel/setup.bash
```

### 19.2 Run old planner

```bash
cd /home/sentry/AstarTraining/rogmap/Old_nav_backup
source sim_nav/devel/setup.bash
roslaunch 3DNavUL_Test.launch planner:=dstar
```

### 19.3 Run new planner

```bash
cd /home/sentry/AstarTraining/rogmap/Old_nav_backup
source sim_nav/devel/setup.bash
roslaunch 3DNavUL_Test.launch planner:=rog
```

### 19.4 Run new planner with static PCD preload

```bash
cd /home/sentry/AstarTraining/rogmap/Old_nav_backup
source sim_nav/devel/setup.bash
roslaunch 3DNavUL_Test.launch planner:=rog static_pcd:=/absolute/path/to/static_map.pcd
```

### 19.5 Visualize Phase 3

Open the Phase 3 RViz config:

```bash
rviz -d /home/sentry/AstarTraining/rogmap/Old_nav_backup/rog_planner.rviz
```

Useful displays:

- A* Path: front-end grid path.
- B-Spline Trajectory: final smooth trajectory.
- ESDF / inflated occupancy: ROG-Map obstacle model.
- `/map`: static PGM wall layer.
- `/cmd_vel_marker`: direction and magnitude of commanded velocity.
- `virtual_frame`: robot command frame.
- `gimbal_frame`: gimbal transform availability.

---

## 20. How to read common symptoms

### 20.1 Planner says it is waiting for static `/map`

Meaning:

`static_map_enable` is true, but no OccupancyGrid has arrived yet.

Check:

```bash
rostopic echo -n 1 /map
```

The map_server launch must be running.

### 20.2 Planner reports stale RobotState

Meaning:

ROG-Map has not received fresh odometry, or localization is stale.

Check:

```bash
rostopic hz /odom
rostopic echo -n 1 /odom
```

Also check that HDL localization is receiving the needed LiDAR data.

### 20.3 A* fails

Likely causes:

- Start is inside a hard wall and cannot be snapped out.
- Goal is inside a hard wall and cannot be snapped out.
- Static map blocks the route.
- ROG-Map marks too much space occupied.
- Map/TF alignment is wrong.

Useful checks:

```bash
rostopic echo -n 1 /map
rostopic hz /aligned_points
rostopic hz /odom
rosrun tf tf_echo map virtual_frame
```

### 20.4 Smoother rejects trajectory

Likely causes:

- A* path is too close to a wall.
- Static map collision occurs after smoothing.
- ESDF obstacle cost pushes the curve into an invalid shape.

Expected behavior:

The planner does not install an unsafe fallback.

### 20.5 Robot stops after an obstacle appears

This can be correct behavior.

If the active trajectory becomes unsafe and no safe replacement is found, the planner clears the active trajectory and the tracker publishes zero velocity.

---

## 21. Current limitations and future work

### 21.1 Acceleration penalty is not fully implemented

`a_max` is parsed and part of the parameter set, but the current smoother primarily enforces velocity through the dynamic cost. A full acceleration penalty can be added later.

### 21.2 Planning is fixed-height

The planner uses 3-D map information but searches at a fixed chassis-height slice. This is appropriate for the current ground sentry platform, but it is not a flying or full 3-D kinodynamic planner.

### 21.3 Static map quality matters

The final hard-wall safety depends on `/map` being aligned with the real world and localization frame.

If `/map` is shifted relative to localization, the planner may incorrectly block free space or allow unsafe space.

### 21.4 Dynamic obstacle prediction is simple

The planner replans from current map observations and projected collision checks. It does not yet predict future moving obstacles with a learned or tracked dynamic model.

### 21.5 Full hardware validation is still required before production replacement

The code builds and the safety issues found during review were fixed, but final production deployment should still include:

- Bench test with motors disabled.
- Low-speed live test.
- Narrow corridor test.
- Goal near wall test.
- Obstacle insertion test.
- Localization dropout test.
- D*-Lite rollback test.

---

## 22. Rollback plan

Rollback is simple because D*-Lite is preserved.

Use:

```bash
roslaunch 3DNavUL_Test.launch planner:=dstar
```

Or omit the planner arg entirely:

```bash
roslaunch 3DNavUL_Test.launch
```

The default remains D*-Lite.

No code deletion is needed to roll back operationally.

---

## 23. Final system mental model

The easiest way to understand the final Phase 3 system is:

```text
D*-Lite was a grid planner that directly produced velocity-compatible behavior.

rog_planner is a layered planner:

  1. ROG-Map builds live obstacle and ESDF information.
  2. Static /map supplies reliable known walls.
  3. A* finds a collision-free coarse route.
  4. B-spline smoothing turns that route into a smoother trajectory.
  5. Validation rejects any route that crosses walls or obstacles.
  6. The tracker follows the safe trajectory at 100 Hz.
  7. The outside robot interface remains /cmd_vel and /dstar_status.
```

The most important practical result is that the robot can now use ROG-Map/ESDF-aware planning while keeping the old system's control contract and rollback path.
