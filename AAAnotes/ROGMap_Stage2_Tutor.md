# ROG-Map Stage 2 — Code Tutor

> **Purpose of this document.** A read-along walkthrough of every file
> Stage 2 introduced or modified, written so a future maintainer (or future
> you) can answer "why is this line here?" without re-deriving the design.
> The deployment manual answers *how to run it*; this file answers *how it
> works and why it was built this way*.

The companion files are:
- [ROGMap_Stage1_Deployment_Manual.md](ROGMap_Stage1_Deployment_Manual.md) — Stage 1 ops.
- [ROGMap_Stage2_Deployment_Manual.md](ROGMap_Stage2_Deployment_Manual.md) — Stage 2 ops.
- This file — Stage 2 internals.

---

## 0. Big picture in one paragraph

D\*-Lite already had a "dynamic obstacle" socket: a `nav_msgs::OccupancyGrid`
topic configured by the param `dynamic_map_topic_name` (default `/grid`).
That socket was **unused** in production — the original publisher
(`dbscan_bfs_3D`) was commented out of the launch file. Stage 1 added a
passive ROG-Map node that builds a 3-D voxel map but does not feed the
planner. Stage 2 plugs the gap with a tiny **bridge node**: it subscribes
to ROG-Map's inflated occupancy cloud, projects the 3-D voxels to a 2-D
grid, and republishes on a new topic `/rog_grid`. A launch arg flips
D\*-Lite's `dynamic_map_topic_name` from `/grid` to `/rog_grid`. **Zero
edits** to the planner.

---

## 1. Topic and node inventory (after Stage 2)

### 1.1 Nodes

| Node | Source | Role | Stage |
|---|---|---|---|
| `/velodyne_nodelet_manager` | Livox driver + hdl_localization (loaded as nodelets) | Drives the LiDAR; publishes `/livox/lidar` and is the host process for the localization nodelet → `/odom`, `/aligned_points`. | Pre-existing |
| `/hdl_localization_nodelet` (inside the manager) | `hdl_localization` package | NDT-based map-frame localizer. Outputs `/odom` (frame=`map`) and lazily `/aligned_points` (frame=`map`). | Pre-existing |
| `/map_server` | `map_server` (ROS) | Loads the static PGM into `/map`. | Pre-existing |
| `/dstarlite` | `bot_sim/src/dstarlite.cpp` | The motion planner. Reads `/map` (static), `/<dynamic_map_topic_name>` (live obstacles), and `/clicked_point` (goal); publishes `/cmd_vel` and `/dstar_status`. | Pre-existing — **only its launch param changes in Stage 2.** |
| `/cmd_vel_marker` | `bot_sim/src/cmd_vel_marker.cpp` | Visualizes `/cmd_vel` in RViz. | Pre-existing |
| `/ser2msg_decision_givepoint` | `bot_sim/src/ser2msg_decision_givepoint.cpp` | Talks to the chassis MCU on `/dev/ttyUSB0`; broadcasts `virtual_frame` TF used by D\*-Lite. | Pre-existing |
| **`/rog_map_node`** | `bot_sim/src/rog_map_node.cpp` (Stage 1) | Hosts a `rog_map::ROGMap` instance; consumes `/aligned_points` + `/odom`; publishes voxel viz topics and broadcasts `world → drone` TF. | **Stage 1** |
| **`/rog_map_to_grid_node`** | `bot_sim/src/rog_map_to_grid_node.cpp` (Stage 2) | The new bridge. Sub: `/rog_map_node/rog_map/inf_occ` (3D PointCloud2) + `/odom`. Pub: `/rog_grid` (2D OccupancyGrid). | **Stage 2** |
| `rog_map_world_link` (Stage 1) | `tf2_ros/static_transform_publisher` | Publishes the identity `map → world` static TF that links rog_map's hardcoded `world` frame to the rest of the stack. | Stage 1 |

### 1.2 Topics introduced or repurposed by Stage 2

| Topic | Type | Pub → Sub | Notes |
|---|---|---|---|
| `/rog_grid` | `nav_msgs/OccupancyGrid` | `rog_map_to_grid_node` → `dstarlite` | New. 200×200 cells × 0.1 m, frame=`world`, robocentric, snapped to 0.2 m lattice. Stage 2's only public output. |
| `/rog_map_node/rog_map/inf_occ` | `sensor_msgs/PointCloud2` | `rog_map_node` → `rog_map_to_grid_node` (+ optionally RViz) | **Lazy-published** — only emitted while at least one subscriber exists. The bridge being a subscriber flips it on. |
| `/odom` | `nav_msgs/Odometry` | `hdl_localization_nodelet` → many | Pre-existing. Bridge uses `pose.pose.position.{x,y}` for grid centering. |
| `/grid` | `nav_msgs/OccupancyGrid` | (none) | Legacy slot. No publisher today. Default for `dynamic_map_topic_name`. |

### 1.3 The two D\*-Lite socket states

```
costmap_source:=dbscan  (default)        costmap_source:=rog        (Stage 2 ON)
─────────────────────────────────         ─────────────────────────────────
                                           
   /map ── map_server                       /map ── map_server
   /grid ── (no publisher)                  /rog_grid ── rog_map_to_grid_node
        │                                          │
        ▼                                          ▼
   ┌─────────┐                               ┌─────────┐
   │dstarlite│  reads /grid                  │dstarlite│  reads /rog_grid
   └─────────┘  (no live obstacles)          └─────────┘  (live 3D obstacles)
```

The planner code is identical. Only one launch param differs.

---

## 2. The bridge node — file by file, line by line

Source: [Old_nav/sim_nav/src/bot_sim/src/rog_map_to_grid_node.cpp](../../../Old_nav/sim_nav/src/bot_sim/src/rog_map_to_grid_node.cpp)

The whole file is ~220 lines. We walk it top-down.

### 2.1 Includes

```cpp
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <mutex>
#include <string>
```

We deliberately do **not** include any PCL headers. ROG-Map's `inf_occ`
PointCloud2 is constructed with `pcl::toROSMsg(pcl::PointCloud<pcl::PointXYZ>, …)`
upstream, but on the wire it is just a `sensor_msgs::PointCloud2` with `x,y,z`
fields. `sensor_msgs::PointCloud2Iterator<float>` reads those fields by name
without any PCL dependency — keeping the bridge's compile time and binary
size small.

### 2.2 The class skeleton

```cpp
class RogMapToGridBridge {
public:
  RogMapToGridBridge(ros::NodeHandle& nh, ros::NodeHandle& pnh) { … }
private:
  void cloudCb(...);     // subscriber: ~cloud_in
  void odomCb(...);      // subscriber: ~odom_in
  void timerCb(...);     // 1 Hz heartbeat
  void publishGrid(bool cloud_driven);
  void paintCloud(...) const;

  // … members …
};
```

The constructor reads params, sizes the grid, advertises and subscribes.
**Three callbacks** all funnel into one method `publishGrid(bool cloud_driven)`,
which is the only place an OccupancyGrid is constructed. This single
funnel is what makes the dedup logic and origin-snapping logic trivial to
audit.

### 2.3 Parameters

```cpp
pnh.param("cloud_timeout", cloud_timeout_, 0.5);
pnh.param("z_min",         z_min_,         0.05);
pnh.param("z_max",         z_max_,         2.0);
pnh.param("res",           res_,           0.1);
pnh.param("window_m",      window_m_,      20.0);
pnh.param("voxel_lattice", voxel_lattice_, 0.2);   // ROG-Map inflation_resolution
pnh.param("heartbeat_hz",  heartbeat_hz_,  1.0);
```

The defaults are not arbitrary:

- `cloud_timeout = 0.5 s` — twice the slowest expected ROG-Map publish
  period (`time_rate=10` in YAML → 0.1 s nominal). One missed frame is
  tolerated; two consecutive misses are treated as a stall.
- `z_min = 0.05` — drops any voxel at or below the floor that survives
  ROG-Map's `virtual_ground_height` filter (which is set to -1.0 to keep
  hdl /odom's slightly-negative z above the kill threshold). 5 cm is high
  enough to ignore floor speckle.
- `z_max = 2.0` — slightly above sentry's gimbal apex (~1.0 m). Banners
  and cable trays at 1.6–1.8 m are caught.
- `res = 0.1, window_m = 20.0` → 200 × 200 cells. Fits comfortably in the
  D\*-Lite per-callback iteration budget (~40 ms per call, see §4 below).
- **`voxel_lattice = 0.2`** — this **must equal** ROG-Map's
  `inflation_resolution` in `rog_map_passive.yaml`. It is the lattice the
  3-D voxel centers live on. Used for origin snapping (§2.7).
- `heartbeat_hz = 1.0` — empty-grid publish when cloud is stale. D\*-Lite
  ages a "changed" cell out after 60 callbacks; at 1 Hz that's a 60 s
  decay. Cloud-driven callbacks (5–10 Hz) accelerate aging when active.

### 2.4 Subscribers and publisher

```cpp
cloud_sub_ = nh.subscribe("cloud_in", 1, &RogMapToGridBridge::cloudCb, this);
odom_sub_  = nh.subscribe("odom_in",  10, &RogMapToGridBridge::odomCb,  this);
grid_pub_  = nh.advertise<nav_msgs::OccupancyGrid>("grid_out", 1);
```

Note **`nh`** (public node handle) — we subscribe to **relative names**
`cloud_in`, `odom_in`, `grid_out`. The launch file remaps them to the
real topics. Why relative? Because remapping is the canonical way to
swap data sources without recompiling. If you want to point the bridge
at a *simulated* cloud later, `<remap from="cloud_in" to="/sim/cloud"/>`
is enough.

Queue sizes:
- Cloud queue = 1 → only ever process the latest cloud; old ones are
  dropped by ROS automatically. The bridge has nothing to do with stale
  history.
- Odom queue = 10 → odom is small and arrives at ~50 Hz. Buffer absorbs
  scheduler hiccups; no real cost.
- Grid pub queue = 1 → matches D\*-Lite's sub queue; latest-grid
  semantics.

### 2.5 The startup gate (in `main`, NOT the constructor)

```cpp
const std::string cloud_topic = ros::names::resolve("cloud_in");
…
auto first = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(
    cloud_topic, nh, ros::Duration(startup_wait_s));
if (!first) {
  ROS_FATAL("[rog_map_to_grid_node] Stage 1 (rog_map_node) not publishing %s; "
            "refusing to start. Re-launch with enable_rog_map_observer:=true",
            cloud_topic.c_str());
  return 1;
}
```

Two non-obvious choices:

1. **`ros::names::resolve("cloud_in")`** — this honours the launch-file
   `<remap from="cloud_in" to="/rog_map_node/rog_map/inf_occ"/>` and
   returns the final topic the bridge will subscribe to. Logging the
   resolved name (not the raw `cloud_in`) makes the FATAL message useful.
2. **`waitForMessage` happens BEFORE `spinner.start()`**. This is
   deliberate: if we constructed `RogMapToGridBridge` first, its
   subscriber would already be registered, and `waitForMessage` would
   race with the cloudCb. By doing the gate in `main` with a separate
   single-shot subscription, the construction path is clean.

If Stage 1 isn't running, the bridge `return 1`s. Because the launch
file declares `required="false"`, this does not bring down dstarlite or
hdl_localization — only the bridge dies, with a loud explanation.

### 2.6 The publish funnel

```cpp
void publishGrid(bool cloud_driven) {
  // 1. Snapshot under lock.
  …
  if (cloud_driven && cloud_ptr && cloud_stamp == last_published_stamp_) {
    return;        // Dedup: same cloud as last time.
  }
  …
  if (!have_odom) return;   // We don't know where the robot is yet.

  const ros::Time now = ros::Time::now();
  const bool stale = (!cloud_ptr) ||
                     ((now - cloud_stamp).toSec() > cloud_timeout_);
  …
}
```

Three things happen here that you should remember:

1. **Always read the latest robot xy.** The snapshot pulls
   `(last_robot_x_, last_robot_y_)` whether the trigger was a cloud, a
   timer, or anything else. This was iter-3 review item #20: the
   heartbeat path must NOT use a robot pose frozen at the time of the
   last cloud. Otherwise a long stall + robot motion would publish empty
   grids mis-located on the robot.
2. **Cloud-driven dedup.** If the same cloud arrives twice (queue=1 and
   timing jitter occasionally lets ROS deliver duplicates), we don't
   build a new grid — `last_published_stamp_` filters them out.
3. **Stale path.** When `cloud_timeout` has elapsed since the last
   cloud, we publish an **all-zero grid** at the heartbeat rate. D\*-Lite
   merges grid cells via `obstacle_possibility = max(dyn,
   static_obstacle_possibility)`, so a zero from us preserves whatever
   the static map says — the static walls remain in place, dynamic
   obstacles fade after 60 rounds × heartbeat period.

### 2.7 Origin snapping — the fix that broke v1

```cpp
const double half = window_m_ * 0.5;
const double origin_x = std::floor((rx - half) / voxel_lattice_) * voxel_lattice_;
const double origin_y = std::floor((ry - half) / voxel_lattice_) * voxel_lattice_;
```

This is the single most important paragraph in the bridge. Here is why
every character matters.

**The problem.** ROG-Map publishes voxel centers at multiples of
`inflation_resolution = 0.2 m` *in world coordinates* (verified in
`rog_map/sliding_map.cpp:175-189`: `posToGlobalIndex` uses `round(pos/res)`
with **no offset by `local_map_origin`**). The bridge's grid has
resolution `res = 0.1 m`. Without snapping, the grid origin floats with
the robot and lands on arbitrary fractional values. A voxel center at
world `x = 1.0` would map to a different grid cell index every tick:
sometimes cell 9, sometimes cell 10, sometimes 9.5 → depending on
fractional origin, the 2×2 paint would systematically miss boundary
voxels and produce **Swiss-cheese** holes in obstacle outlines.

**The fix.** Snap the origin to the same 0.2 m world-anchored lattice
as the voxels themselves. After snapping, the quantity
`(voxel_center − origin) / 0.1` is *always an integer* — the 2×2 paint
covers exactly the world rectangle `[vc − 0.1, vc + 0.1]` on each axis,
which is the voxel's footprint. No gaps; no double-painting issues.

Worked example with the test results we got: robot at ~`(-0.4, -0.2)`,
window 20 m, voxel_lattice 0.2:

```
origin_x = floor((-0.4 - 10.0) / 0.2) * 0.2
         = floor(-52.0) * 0.2
         = -52 * 0.2
         = -10.4         ← matches the rostopic echo output exactly.

origin_y = floor((-0.2 - 10.0) / 0.2) * 0.2
         = floor(-51.0) * 0.2
         = -51 * 0.2
         = -10.2         ← matches.
```

The grid covers `[-10.4, +9.6] × [-10.2, +9.8]`, robot is inside, and
all voxel centers in the cloud sit on `0.2`-multiples relative to the
origin → integer cell indices guaranteed.

### 2.8 The grid header and `data` allocation

```cpp
nav_msgs::OccupancyGrid g;
g.header.frame_id = "world";
g.header.stamp = stale ? now : cloud_stamp;
g.info.map_load_time = g.header.stamp;
g.info.resolution = static_cast<float>(res_);
g.info.width  = static_cast<uint32_t>(N_);
g.info.height = static_cast<uint32_t>(N_);
g.info.origin.position.x = origin_x;
g.info.origin.position.y = origin_y;
g.info.origin.position.z = 0.0;
g.info.origin.orientation.w = 1.0;
g.data.assign(N_ * N_, 0);
```

- **Frame `world`, not `map`.** ROG-Map's hardcoded viz frame is
  `world`; Stage 1 added an identity `map ↔ world` static TF. By
  publishing in `world`, the bridge stays honest about the cloud's
  native frame, and dstarlite handles the (identity) transform via
  its existing `lookupTransform("map", header.frame_id)` call. If
  someone later changes the static TF to a non-identity, the bridge
  still works correctly because TF-transform happens downstream.
- **Stamp.** Cloud's stamp when fresh (so RViz, rosbag and other
  consumers get accurate timing); `now()` when stale (the data is
  old by definition; using the cached stamp would be misleading).
- **`data.assign(N*N, 0)`** — start with a clean zero grid. The
  zero values are the "free / unknown to me" baseline that
  `max(0, static)` preserves the static map across.

### 2.9 The paint loop

```cpp
sensor_msgs::PointCloud2ConstIterator<float> ix(cloud, "x"), iy(cloud, "y"), iz(cloud, "z");
for (; ix != ix.end(); ++ix, ++iy, ++iz) {
  if (!std::isfinite(*ix) || !std::isfinite(*iy) || !std::isfinite(*iz)) continue;
  if (*iz < z_min_ || *iz > z_max_) continue;
  const int cx = static_cast<int>(std::lround((*ix - origin_x) / res));
  const int cy = static_cast<int>(std::lround((*iy - origin_y) / res));
  for (int dy = -1; dy <= 0; ++dy) {
    const int y = cy + dy;
    if (y < 0 || y >= N) continue;
    for (int dx = -1; dx <= 0; ++dx) {
      const int x = cx + dx;
      if (x < 0 || x >= N) continue;
      data[static_cast<size_t>(y) * N + x] = 100;
    }
  }
}
```

- `std::isfinite` guard — defensive against any NaN that ever slips
  through. Cheap and worth it.
- `std::lround` (round-half-away-from-zero), not `floor`. With the
  origin snapped, the fractional part of `(*ix - origin_x)/res` is
  exactly 0 for every voxel center; `lround` gives the integer.
- The `[-1, 0]` paint pattern: cell index `cx` covers world
  `[origin + cx·res, origin + (cx+1)·res]`. Painting cells `cx-1` and
  `cx` covers `[origin + (cx-1)·res, origin + (cx+1)·res]` — exactly
  one voxel-lattice spacing centered on the voxel center. No off-by-one.
- **Row-major index `y*N + x`** matches the dstarlite consumer's
  unpacking at `dstarlite.cpp:327`: `index = i + j*width` (with
  `i=column, j=row`). Critical to get this right; transposing the
  grid here would silently ghost obstacles 90° rotated.

### 2.10 Threading

```cpp
ros::AsyncSpinner spinner(1);
spinner.start();
ros::waitForShutdown();
```

`AsyncSpinner(1)` means one worker thread services all callback queues
(cloud, odom, timer). Callbacks are *serialized* — at most one runs at
a time. The mutex `mtx_` is therefore strictly defensive: it costs
nothing at single-thread, and protects future maintainers who might
crank the spinner up to 2+ threads without realizing the implications.

---

## 3. The launch wiring

### 3.1 `rog_map_costmap_bridge.launch`

[Old_nav/sim_nav/src/bot_sim/launch_real/rog_map_costmap_bridge.launch](../../../Old_nav/sim_nav/src/bot_sim/launch_real/rog_map_costmap_bridge.launch)

Eleven `<arg>` declarations + one `<node>`. The pattern:

```xml
<arg name="cloud_topic" default="/rog_map_node/rog_map/inf_occ"/>
…
<node pkg="bot_sim" type="rog_map_to_grid_node" name="rog_map_to_grid_node"
      output="screen" required="false">
  <remap from="cloud_in" to="$(arg cloud_topic)"/>
  …
  <param name="cloud_timeout" value="$(arg cloud_timeout)"/>
  …
</node>
```

Two pieces of philosophy here:

1. **`required="false"`.** If the bridge `ROS_FATAL`s and exits (Stage 1
   missing), only this one node dies. dstarlite, hdl_localization, etc.
   keep running. With `required="true"`, a Stage-1 omission would
   cascade into a full-stack outage — much worse UX.
2. **`output="screen"`.** All bring-up errors (param parse failures,
   FATAL exits) appear in the terminal where you ran `roslaunch`,
   instead of disappearing into `~/.ros/log`. Keep this.

### 3.2 The `dstarlite.launch` patch

[Old_nav/sim_nav/src/bot_sim/launch_real/dstarlite.launch](../../../Old_nav/sim_nav/src/bot_sim/launch_real/dstarlite.launch)

Two-line change:

```xml
<launch>
    <arg name="dynamic_map_topic_name" default="/grid"/>
    <node pkg="bot_sim" type="dstarlite" name="dstarlite" >
        …
        <param name="dynamic_map_topic_name" value="$(arg dynamic_map_topic_name)" />
```

The `<arg>` is declared at `<launch>` scope (NOT inside `<node>`) so a
parent launch can override it via `<include><arg name="…" value="…"/></include>`.
The `<param>` line still writes the param to the dstarlite node's
private namespace, but the value now comes from the arg, defaulting to
the legacy `/grid`.

This is **PARAM substitution, NOT a topic remap.** D\*-Lite at
`dstarlite.cpp:646` does `nh.subscribe(<param string>, …)` — it builds
the subscriber using the param value as the literal topic name. A
`<remap>` would not help here; the param has to carry the right name.

### 3.3 The top-level `3DNavUL_Test.launch` patch

[Old_nav/3DNavUL_Test.launch](../../../Old_nav/3DNavUL_Test.launch)

Three additions:

```xml
<arg name="enable_rog_costmap" default="false"/>
<arg name="costmap_source"     default="dbscan"/>
…
<include file="$(find bot_sim)/launch_real/rog_map_costmap_bridge.launch"
         if="$(arg enable_rog_costmap)"/>
…
<include file="$(find bot_sim)/launch_real/dstarlite.launch"
         if="$(eval costmap_source == 'rog')">
    <arg name="dynamic_map_topic_name" value="/rog_grid"/>
</include>
<include file="$(find bot_sim)/launch_real/dstarlite.launch"
         unless="$(eval costmap_source == 'rog')"/>
```

Why **two** `<include>` for `dstarlite.launch`? Because roslaunch does
not allow conditionally inserting a child `<arg>` inside an `<include>`;
the `<arg>` either exists (and is passed) or doesn't. The canonical
ROS 1 pattern for "include with extra args only when X" is the
`if`/`unless` pair you see here. Both `<include>`s are mutually
exclusive (the conditions are negations) so dstarlite is started exactly
once per launch.

The `$(eval costmap_source == 'rog')` is a Python-syntax expression
evaluated by roslaunch at parse time. Single quotes inside double-quoted
attributes are mandatory.

### 3.4 The CMakeLists changes

[Old_nav/sim_nav/src/bot_sim/CMakeLists.txt](../../../Old_nav/sim_nav/src/bot_sim/CMakeLists.txt)

Two diffs:

1. Added `sensor_msgs` to the `find_package(catkin REQUIRED COMPONENTS …)`
   list. Without this, `<sensor_msgs/point_cloud2_iterator.h>` would
   not be on the include path.
2. Right after the Stage 1 `rog_map_node` block:
   ```cmake
   # --- Stage 2 ROG-Map -> /grid bridge node ---
   add_executable(rog_map_to_grid_node src/rog_map_to_grid_node.cpp)
   add_dependencies(rog_map_to_grid_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
   target_link_libraries(rog_map_to_grid_node ${catkin_LIBRARIES})
   ```
   `add_dependencies` is the catkin idiom for "wait for any in-package
   message generation before compiling this exe". Even though the bridge
   doesn't generate or consume custom msgs, copying the idiom prevents
   surprises if `bot_sim` ever adds them.

### 3.5 The `package.xml` changes

[Old_nav/sim_nav/src/bot_sim/package.xml](../../../Old_nav/sim_nav/src/bot_sim/package.xml)

Two `<depend>` tags added next to `<depend>rog_map</depend>`:

```xml
<depend>sensor_msgs</depend>
<depend>nav_msgs</depend>
```

`nav_msgs` was already pulled in via `find_package` of catkin
(transitively via `tf2_ros`), but explicit `<depend>` keeps `rosdep`
honest about install requirements on a fresh machine.

---

## 4. Why these design choices and not the obvious alternatives?

This section is the post-mortem from three rounds of subagent review.

### 4.1 Why a bridge node, not a patch to dstarlite?

A direct rewire of `dstarlite.cpp` to subscribe to `inf_occ` was
rejected because:

- D\*-Lite expects an `OccupancyGrid` (2-D, indexable, with a stable
  `info.origin`); it has no code path for sparse 3-D point clouds.
- Adding a code path to dstarlite means adding test surface to a
  battle-tested planner. The bridge is ~220 LOC of pure data
  reformatting; it can be deleted in a single git revert.
- The `dynamic_map_topic_name` socket already existed and was *unused*.
  Filling an existing socket is a smaller change than adding a new one.

### 4.2 Why `world` frame on the grid, not `map`?

Two reasons:

1. **Honesty.** The cloud is in `world`; pretending it's in `map` would
   create silent bugs the day someone changes the Stage 1 static TF
   from identity to a non-trivial offset. By publishing in `world`,
   the bridge is correct under any future shim.
2. **No bridge-side TF dependency.** Doing the world→map transform
   inside the bridge would require a `tf2_ros::Buffer` + listener.
   That listener has its own thread (or needs the bridge to multi-thread),
   complicating the threading model. Letting dstarlite do its existing
   `lookupTransform` is free — it was already on that code path for the
   legacy `dbscan_bfs_3D` setup, which used `frame_id="odom"` or
   `"laser"` and went through the same TF call.

### 4.3 Why robot pose from `/odom`, not TF?

D\*-Lite uses `virtual_frame` as the robot frame. `virtual_frame` is
only broadcast by `ser2msg_decision_givepoint` when the chassis MCU
serial port is connected. Bench testing, rosbag replay, or any
hardware-disconnected dev workflow → `virtual_frame` doesn't exist →
TF lookup throws → bridge can't compute origin → bridge can't publish.

`/odom` from hdl_localization is *always* available whenever the
LiDAR pipeline is alive; it has no MCU dependency. Stage 1's identity
TF makes the xy values valid in `world`. Net result: the bridge works
on bench, in rosbag, on hardware — same code, same launch.

### 4.4 Why grid resolution 0.1 m and voxel lattice 0.2 m, painted 2×2?

The first plan (v1) had 0.05 m grid with 1 cell per voxel. That
created Swiss-cheese obstacle outlines because rog_map voxels are
0.2 m apart on a fixed lattice — there were 4-cell gaps between
adjacent voxels. The current 0.1 m + 2×2 paint provides:

- One 2×2 footprint per voxel = exactly 0.2 × 0.2 m on the ground = no gaps
- Half the per-cell budget of the v1 design (0.1 vs 0.05 m) = 4× fewer
  cells = cheaper for dstarlite to iterate
- Cleanly snappable to the same lattice as the voxels (0.1 = 0.2 / 2)

### 4.5 Why heartbeat at 1 Hz?

D\*-Lite's "changed dynamic obstacle" cells age out after **60
callbacks** (hard-coded constant in the planner). The heartbeat keeps
the callback firing even when no cloud is arriving, so trails decay in
~60 s. The cloud-driven path runs at 5–10 Hz when active, so trails
decay in ≤6 s under normal operation. The heartbeat exists for the
edge case where rog_map stalls — without it, dstarlite would freeze
the last live obstacles in place forever.

### 4.6 Why a runtime guard, not a launch-time auto-promotion?

We considered:

```xml
<arg name="enable_rog_map_observer" default="$(eval enable_rog_costmap)"/>
```

This *almost* works, but fails subtly:

- Inside `$(eval …)`, roslaunch coerces arg names to typed values. A
  bare `enable_rog_costmap` becomes the Python `bool` value, which
  stringifies to `"True"` (capitalized).
- Some roslaunch versions compare `if=` against lowercase
  `"true"`/`"false"` only; capitalized `True` is then treated as
  falsy, silently breaking the auto-promotion.
- Hard to test across machines without a deployment regression hunt.

The runtime guard (`waitForMessage` + `ROS_FATAL`) is **trivially
correct** across roslaunch versions and gives a clear, actionable
error message instead of a silent misconfiguration. Tradeoff: the
operator must remember to pass two args. Acceptable; the manual is
explicit.

---

## 5. Reading the test output you ran

You captured this:

```
average rate: 11.109     ← cloud-driven publish; faster than the 1 Hz heartbeat
                            because rog_map's inf_occ is publishing at ~10 Hz
                            and we publish on every fresh cloud.

origin: x: -10.4, y: -10.200000000000001
                         ← snapped origins. -10.4 = -52 * 0.2. -10.2 = -51 * 0.2.
                            The trailing ...01 is double-precision representation
                            noise; harmless.

resolution: 0.10…01
                         ← float32 representation of 0.1; expected.

width: 200, height: 200, frame_id: "world"
                         ← matches design.

Publishers: /rog_map_to_grid_node    Subscribers: /dstarlite
                         ← the entire purpose of Stage 2: dstarlite is now
                            consuming the bridge's output.

%CPU 0.3 / 0.7
                         ← bridge essentially free.
```

The high system CPU (`74.5 us / 21.2 sy`) in your `top` snapshot is
NOT the bridge — that's the rest of the stack (LiDAR driver,
hdl_localization NDT, RViz, etc.). The bridge process itself is at
<1%. Fine.

---

## 6. What to read next when something breaks

1. **Stage 2 deployment manual** — operational checklist + troubleshooting table.
2. **Stage 1 deployment manual** — the `virtual_ground_height` gotcha
   that left `inf_occ` empty during initial bring-up.
3. The bridge source itself, `rog_map_to_grid_node.cpp` — comments are
   intentionally dense around the origin-snap math and the threading
   contract.
4. `dstarlite.cpp:300-358` — the consumer-side. Specifically the line
   `obstacle_possibility = max(data[index], static_obstacle_possibility)`
   that explains why our zero-cells preserve static obstacles.
5. `rog_map.cpp:407-417` — the upstream side. Specifically the
   `vm_.occ_inf_pub.getNumSubscribers() >= 1` lazy-publish gate that
   the bridge silently flips on.
