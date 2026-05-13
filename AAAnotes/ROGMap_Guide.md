# ROG-Map — What It Is, What It Does, How to Plug It into 哨兵

**Source in this workspace:** [ROG-Map/](../ROG-Map/) · **Upstream:** https://github.com/hku-mars/ROG-Map · **Paper:** *IROS 2024*, Ren et al., arXiv:2302.14819
**Target pipeline:** the sentry stack documented in [Pipeline_Report.md](Pipeline_Report.md).

This guide is written in two halves:
- **Part A (plain-English):** what ROG-Map is and what you get out of it.
- **Part B (integrator-level):** how to wire it into the existing 哨兵 `livox_ros_driver2 → hdl_localization → bot_sim/dstarlite → decision_node` pipeline.

---

## Part A — What ROG-Map Is

### A.1 One-paragraph summary

ROG-Map (**R**obocentric **O**ccupancy **G**rid **Map**) is a ROS/C++ **local** voxel map library from HKU MaRS Lab, designed so that a motion planner can answer questions like *"is this point free?"*, *"how far is the nearest obstacle?"*, and *"where is the frontier of known space?"* in constant time, at 10 cm resolution, over tens of metres, **at the rate that a LiDAR produces frames**. It is not a SLAM system — it consumes an *already-registered* point cloud + odometry and maintains a dense grid that **follows the robot** as it moves.

Think of it as a more modern, planning-oriented replacement for `octomap_server` / `costmap_2d`:

| Need | Classical tool | ROG-Map |
|---|---|---|
| Build a map from LiDAR | OctoMap | ✔ (local window only) |
| Inflate obstacles by robot radius | `costmap_2d` | ✔, **incrementally**, multi-resolution |
| Query ESDF (distance to nearest obstacle + gradient) | `dynamicEDT3D` / Fast-Planner's ESDF | ✔, **sliding**, trilinear with analytic gradient/Hessian |
| Find frontiers (unknown↔free boundary) for exploration | External packages | ✔, O(1) per cell, incremental |
| Run at 10+ Hz on 0.1 m / 30×30×5 m window | Struggles | Designed for it |

### A.2 The four sub-maps inside one container

All four share the same robocentric **sliding** voxel grid, re-indexed (not memcopied) as the robot translates:

1. **ProbMap** — probabilistic occupancy (log-odds Bayesian update). Cells are classified `UNKNOWN`, `KNOWN_FREE`, or `OCCUPIED`. Resolution typically 0.05–0.1 m. Updated by a LiDAR-specific raycaster.
2. **InfMap** — the ProbMap **inflated by the robot radius**. Can run at a *coarser* resolution (e.g. 0.2 m for a 0.1 m ProbMap) so that collision checks for the planner are cheap. "Incremental" means: whenever one fine cell flips occupied/free, only the `(2r+1)³` affected coarse cells bump a counter; no full pass.
3. **ESDFMap** — signed distance transform (positive outside obstacles, negative inside) computed on the same sliding window. Exposes `getDistance(p)`, `evaluateEDT`, `evaluateFirstGrad`, `evaluateSecondGrad` for gradient-based trajectory optimizers.
4. **FreeCntMap / Frontier** — each unknown cell knows how many of its 27 neighbours are `KNOWN_FREE`; if ≥1, it is a **frontier**. Used for exploration / active SLAM.

### A.3 Key features (why choose it over OctoMap + Costmap)

| Feature | Why it matters |
|---|---|
| **Zero-copy sliding** | The grid's memory is never memmoved; only an integer origin index shifts and the "ring" that leaves the window is reset. Memory and update cost are bounded by window size, **not** by how far the robot has travelled. |
| **Bounded memory** | Pick your window (e.g. 30×30×5 m @ 0.1 m = ~4.5 M cells). Runtime complexity is independent of explored area. |
| **Multi-resolution inflation** | Planner checks collision against a coarser grid (1 cell = robot radius), while perception stays fine. Huge speedup for A*/RRT*/MPC. |
| **Incremental inflation updates** | Only cells whose fine-grid state changed this tick trigger inflation updates. No per-frame re-inflation pass. |
| **Sliding ESDF with gradient** | Trajectory optimizers (EGO-Planner, IPOPT-based MPC) can directly ∇ through the map. |
| **Frontier in O(1) / cell** | Exploration planners can just iterate over `isFrontier(p)`. |
| **Unknown handling is first-class** | Explicit `UNKNOWN` / `FRONTIER` enum values. Optional "inflate unknown too" for conservative planning. |
| **Virtual ceil/ground** | Forces everything above a plane (ceiling) or below a plane (ground) to OCCUPIED. Useful for 2-D-ish robots (like the 哨兵) to keep obstacles on the ground plane. |
| **PCD preload** | Bootstrap the window from a saved point-cloud map (perfect match for the `innowing.pcd` we already use). |
| **ROS-optional** | You can either let it subscribe to cloud + odom, or manually call `updateMap(cloud, pose)` every frame. |

### A.4 What ROG-Map does *not* do

- It is **not a SLAM system**. Pose must come from somewhere (FAST-LIO, `hdl_localization`, Point-LIO, …).
- It does **not** read `/tf`. The incoming cloud is expected to already be in the same frame as the odometry (the typical "`/cloud_registered`" output of LIO pipelines).
- It does **not** provide a global, persistent map. The window follows the robot; cells leaving the window are forgotten. If you want persistence, save periodically or keep a separate global map.
- It does **not** come with a planner. The A*/RRT* in `examples/` are reference implementations, not production-grade planners.
- It does **not** process raw Livox `CustomMsg`; feed it `PointCloud2` (`pcl::PointXYZINormal`).

### A.5 Visual overview

ProbMap (yellow, fine) + InfMap (gray, coarser inflated):

![multi-res](../ROG-Map/misc/image-20240830181904520.png)

Frontiers (red) within a 5 m sensing range:

![frontier](../ROG-Map/misc/image-20240830183455023-17250143906771.png)

Sliding ESDF slice:

![esdf](../ROG-Map/misc/image-20240830183740886.png)

---

## Part B — Integrator Reference

### B.1 Public C++ API (summary)

Header: [ROG-Map/rog_map/include/rog_map/rog_map.h](../ROG-Map/rog_map/include/rog_map/rog_map.h)

```cpp
#include "rog_map/rog_map.h"

namespace rog_map {
  typedef std::pair<Vec3f, Eigen::Quaterniond> Pose;
  typedef pcl::PointCloud<pcl::PointXYZINormal> PointCloud;

  class ROGMap : public ProbMap {
  public:
    typedef std::shared_ptr<ROGMap> Ptr;
    explicit ROGMap(const ros::NodeHandle& nh);

    // Feed one LiDAR frame (use when ros_callback.enable: false)
    void updateMap(const PointCloud& cloud, const Pose& pose);

    // Collision-free line check (fine or inflated grid)
    bool isLineFree(const Vec3f& start, const Vec3f& end,
                    double max_dis = 1e9) const;
    bool isLineFree(const Vec3f& start, const Vec3f& end,
                    bool use_inf_map, bool use_unk_as_occ) const;

    RobotState getRobotState() const;
  };
}
```

Inherited from `ProbMap` ([prob_map.h](../ROG-Map/rog_map/include/rog_map/prob_map.h)):

```cpp
// Point-wise state tests (fine grid)
bool isOccupied      (const Vec3f& p) const;
bool isUnknown       (const Vec3f& p) const;
bool isKnownFree     (const Vec3f& p) const;
// Inflated grid (cheap; use for planning)
bool isOccupiedInflate(const Vec3f& p) const;
bool isUnknownInflate (const Vec3f& p) const;
bool isKnownFreeInflate(const Vec3f& p) const;
// Frontier
bool isFrontier(const Vec3f& p) const;
// Categorical
GridType getGridType    (const Vec3f& p) const;   // UNKNOWN / OCCUPIED / KNOWN_FREE / FRONTIER / OUT_OF_MAP
GridType getInfGridType (const Vec3f& p) const;
// Bulk region query
void boxSearch        (const Vec3f& bmin, const Vec3f& bmax, GridType gt, vec_E<Vec3f>& out) const;
void boxSearchInflate (const Vec3f& bmin, const Vec3f& bmax, GridType gt, vec_E<Vec3f>& out) const;
// Meta
Vec3f getLocalMapOrigin() const;
Vec3f getLocalMapSize()   const;
double getResolution()    const;
```

ESDF (through `esdf_map_`, currently `protected` — expose via subclass or friend if you need it):

```cpp
double getDistance(const Vec3f& pos) const;
void   evaluateEDT        (const Eigen::Vector3d& p, double& dist);
void   evaluateFirstGrad  (const Eigen::Vector3d& p, Eigen::Vector3d& grad);
void   evaluateSecondGrad (const Eigen::Vector3d& p, Eigen::Vector3d& grad);
```

`GridType` enum ([utils/common_lib.hpp](../ROG-Map/rog_map/include/utils/common_lib.hpp)):

```cpp
enum GridType { UNDEFINED=0, UNKNOWN, OUT_OF_MAP, OCCUPIED, KNOWN_FREE, FRONTIER };
```

### B.2 ROS interface

**ROG-Map is TF-free.** Pose comes from the odom topic, cloud is assumed world-frame.

**Subscriptions** (only when `ros_callback.enable: true`):

| Topic (default) | Type | Purpose |
|---|---|---|
| `/cloud_registered` | `sensor_msgs/PointCloud2` (PCL `XYZINormal`) | Already-registered world-frame cloud |
| `/lidar_slam/odom`  | `nav_msgs/Odometry` | Robot pose in same frame |

**Publications** (under node's private namespace):

| Topic | Type | Gate |
|---|---|---|
| `rog_map/occ` | `PointCloud2` | visualization.enable |
| `rog_map/unk` | `PointCloud2` | pub_unknown_map_en |
| `rog_map/inf_occ` | `PointCloud2` | visualization |
| `rog_map/inf_unk` | `PointCloud2` | unk_inflation_en & viz |
| `rog_map/frontier` | `PointCloud2` | frontier_extraction_en |
| `rog_map/esdf`, `esdf/neg`, `esdf/occ` | `PointCloud2` | esdf.enable |
| `rog_map/map_bound` | `MarkerArray` | always |

### B.3 Full configuration reference

All parameters live under the node's private namespace at `rog_map/…`.

#### Geometry / sliding

| Key | Default | Meaning |
|---|---|---|
| `rog_map/resolution` | 0.1 m | Fine (ProbMap) voxel size |
| `rog_map/inflation_resolution` | 0.1 m | Coarse (InfMap) voxel size; **must be ≥ resolution** |
| `rog_map/inflation_step` | 1 | Inflation radius in InfMap cells (spherical) |
| `rog_map/unk_inflation_en` / `unk_inflation_step` | false / 1 | Also inflate unknown |
| `rog_map/map_size` | [10,10,0] m | Extent of local window (symmetric around origin) |
| `rog_map/map_sliding/enable` | true | Robocentric sliding |
| `rog_map/map_sliding/threshold` | -1 (always) m | Min robot displacement before re-sliding |
| `rog_map/virtual_ceil_height` / `virtual_ground_height` | -0.1 | Force OCCUPIED outside slab. Use `9999` / `-9999` to disable |

#### Input

| Key | Default | Meaning |
|---|---|---|
| `rog_map/ros_callback/enable` | false | If true, ROG-Map subscribes cloud+odom itself |
| `rog_map/ros_callback/cloud_topic` | `/cloud_registered` | Registered PointCloud2 |
| `rog_map/ros_callback/odom_topic` | `/lidar_slam/odom` | Odometry |
| `rog_map/ros_callback/odom_timeout` | 0.05 s | Stale-odom guard |
| `rog_map/point_filt_num` | 2 | Temporal down-sample factor |
| `rog_map/intensity_thresh` | -1 | Drop points below intensity |
| `rog_map/load_pcd_en` / `rog_map/pcd_name` | false / "map.pcd" | Bulk-load PCD at startup |

#### Raycasting / probabilistic update

| Key | Default | Meaning |
|---|---|---|
| `rog_map/raycasting/enable` | true | If false, only hits are marked occupied; rest stays UNKNOWN |
| `rog_map/raycasting/batch_update_size` | 1 | Accumulate N frames before pass |
| `rog_map/raycasting/local_update_box` | [999,999,999] | Raycasting box around robot |
| `rog_map/raycasting/ray_range` | [0.3, 10] | [min,max] (m) |
| `rog_map/raycasting/p_hit / p_miss / p_min / p_max / p_occ / p_free` | 0.70 / 0.70 / 0.12 / 0.97 / 0.80 / 0.30 | Log-odds bands |
| `rog_map/raycasting/unk_thresh` | 0.70 | Fraction of sub-cells unknown → coarse cell unknown |

#### Frontier / ESDF

| Key | Default | Meaning |
|---|---|---|
| `rog_map/frontier_extraction_en` | false | Enable incremental frontier counting |
| `rog_map/esdf/enable` | false | Compute sliding ESDF |
| `rog_map/esdf/resolution` | 0.2 | ESDF voxel size |
| `rog_map/esdf/local_update_box` | — | Box around robot for ESDF refresh |

#### Visualization

| Key | Default | Meaning |
|---|---|---|
| `rog_map/visualization/enable` | false | Master switch |
| `rog_map/visualization/time_rate` | 0 Hz | Periodic viz rate |
| `rog_map/visualization/range` | [0,0,0] | Viz box extent (m) |
| `rog_map/visualization/frame_id` | `world` | Header frame — **must match odom/cloud frame** |
| `rog_map/visualization/pub_unknown_map_en` | false | Publish unknown cells |

### B.4 Preset YAMLs (starting points)

| File | Use when… |
|---|---|
| [marsim_example.yaml](../ROG-Map/examples/rog_map_example/config/marsim_example.yaml) | Full stack: raycast + frontier + ESDF + viz |
| [astar_example.yaml](../ROG-Map/examples/rog_map_example/config/astar_example.yaml) | Loads a PCD; A* search on InfMap |
| [rrt_example.yaml](../ROG-Map/examples/rog_map_example/config/rrt_example.yaml) | Loads a PCD; RRT* sampling |
| [no_raycast.yaml](../ROG-Map/examples/rog_map_example/config/no_raycast.yaml) | Use a pre-built PCD; no free-space carving |
| [pure_ogm.yaml](../ROG-Map/examples/rog_map_example/config/pure_ogm.yaml) | Bare probabilistic OGM, no frontier/ESDF |

### B.5 Minimal user code

Based on [examples/rog_map_example/Apps/marsim_example_node.cpp](../ROG-Map/examples/rog_map_example/Apps/marsim_example_node.cpp):

```cpp
#include "rog_map/rog_map.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "rog_map_node");
  ros::NodeHandle nh("~");
  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

  auto map = std::make_shared<rog_map::ROGMap>(nh);   // reads ~rog_map/* params

  // Option A — topic driven: set rog_map/ros_callback/enable: true, then just spin.
  // Option B — manual:
  //   rog_map::Pose pose{{x,y,z}, q};
  //   rog_map::PointCloud cloud;  // already in world frame
  //   map->updateMap(cloud, pose);

  // Query example
  Eigen::Vector3d q(1.2, 0.3, 1.0);
  if (map->isOccupiedInflate(q)) { /* avoid */ }

  ros::AsyncSpinner s(0); s.start();
  ros::waitForShutdown();
}
```

### B.6 Dependencies and build

```bash
# apt
sudo apt-get install ros-noetic-rosfmt libglfw3-dev libglew-dev \
                     libeigen3-dev libdw-dev
sudo ln -s /usr/include/eigen3/Eigen /usr/include/Eigen   # once

# build (inside any catkin ws)
cd <ws>/src
ln -s /path/to/哨兵/ROG-Map/rog_map .                  # or copy
cd .. && catkin_make -DBUILD_TYPE=Release
```

### B.7 Known gotchas

- **`libdw-dev`** is a hard linker dep (backward-cpp stack traces). Missing → link errors.
- **Conda**: `conda deactivate` before `catkin_make`, or delete `build/` `devel/` and retry.
- **`VizConfig` not generated** → `catkin_make -DCATKIN_DEVEL_PREFIX:PATH=${YOUR_WS}/devel`.
- **`ORIGIN_AT_CORNER` vs `ORIGIN_AT_CENTER`** is a compile-time flag ([rog_map/CMakeLists.txt](../ROG-Map/rog_map/CMakeLists.txt)). Keep consistent with downstream code.
- **`inflation_resolution ≥ resolution`** required at startup, else abort.
- **`virtual_ceil_height` / `virtual_ground_height`** force OCCUPIED outside the slab — set to ±9999 for full 3-D.
- **Frame consistency**: `visualization/frame_id` must be the frame in which odom + cloud live.
- **ROG-Map does not use `/tf`**; the cloud must already be in that frame (use an LIO's `/cloud_registered` or run a pre-transformer).
- Don't enable `ros_callback` and call `updateMap()` at the same time.
- `raycasting.enable: false` leaves everything UNKNOWN except hits → frontier/ESDF near useless. Combine with PCD preload (`load_pcd_en: true`).

---

## Part C — Integrating ROG-Map into 哨兵

The sentry stack today is described in [Pipeline_Report.md](Pipeline_Report.md). The relevant slice is:

```
livox_ros_driver2  ──► /livox/lidar  ──► hdl_localization  ──► TF map→aft_mapped + /aligned_points
                                                                │
                       map_server (innowing.yaml, 2-D)  ──► dstarlite ──► /cmd_vel
```

The planner (`dstarlite`) is **2-D only**, fed by a static OccupancyGrid; it has no runtime awareness of 3-D obstacles (dust baffles, ramps, projectiles, moving robots). The goal of integration is to add **live 3-D obstacle perception** while leaving the static map, localization, and control loop untouched.

### C.1 Three recommended integration modes

| Mode | What you gain | Work required |
|---|---|---|
| **1. Observability only** | RViz visualization + rosbag of ROG-Map occupancy, inflation, ESDF, frontier — no behavioural change. | ~30 min. |
| **2. Dynamic costmap injection** | Publish ROG-Map's inflated-occupancy as a 2-D `/grid` that `dstarlite` already subscribes to → planner avoids live 3-D obstacles in real time. | Half-day. |
| **3. Replace D*-Lite with an ROG-Map-aware 3-D planner** | A*/RRT* on ROG-Map's `InfMap`, or use ESDF for trajectory optimization. | Multi-day. |

Mode 2 is the recommended first step — maximum benefit, minimum change.

---

### C.2 Mode 1 — Observability (drop-in, no behaviour change)

**Goal:** run ROG-Map alongside the current stack, purely as a monitoring/research tool.

**Step 1. Add `rog_map` as a package source.**
Place or symlink [ROG-Map/rog_map/](../ROG-Map/rog_map/) into `Old_Nav/sim_nav/src/`:
```bash
cd Old_Nav/sim_nav/src
ln -s ../../../ROG-Map/rog_map .
```
Rebuild: `cd ../.. && catkin_make -DBUILD_TYPE=Release` (after `conda deactivate`).

**Step 2. Feed it a registered point cloud.**
`hdl_localization` publishes `/aligned_points` — the live LiDAR cloud transformed into the `map` frame. That is exactly what ROG-Map wants.

**Step 3. Create a config YAML.** Example `sim_nav/src/bot_sim/config/rog_map_passive.yaml`:

```yaml
rog_map:
  resolution: 0.1
  inflation_resolution: 0.2
  inflation_step: 2                  # ~robot radius
  map_size: [30.0, 30.0, 3.0]
  map_sliding:
    enable: true
    threshold: 0.5
  virtual_ceil_height: 2.0           # sentry is < 2 m tall
  virtual_ground_height: 0.05        # 5 cm above floor → drop floor clutter
  load_pcd_en: false
  point_filt_num: 2
  intensity_thresh: -1
  ros_callback:
    enable: true
    cloud_topic: /aligned_points     # <-- from hdl_localization
    odom_topic: /odom                # <-- from hdl_localization
    odom_timeout: 0.2
  raycasting:
    enable: true
    batch_update_size: 1
    local_update_box: [30.0, 30.0, 3.0]
    ray_range: [0.3, 15.0]
    p_hit: 0.70
    p_miss: 0.35
    p_min: 0.12
    p_max: 0.97
    p_occ: 0.80
    p_free: 0.30
    unk_thresh: 0.70
  frontier_extraction_en: false
  esdf:
    enable: false
  visualization:
    enable: true
    time_rate: 10
    range: [20.0, 20.0, 3.0]
    frame_id: map                    # same as hdl_localization TF
    pub_unknown_map_en: false
```

**Step 4. Write a tiny driver node.** Create `sim_nav/src/bot_sim/src/rog_map_node.cpp` with the 10-line `main()` from Section B.5. Add to `CMakeLists.txt`:

```cmake
find_package(catkin REQUIRED COMPONENTS roscpp rog_map pcl_ros)
add_executable(rog_map_node src/rog_map_node.cpp)
target_link_libraries(rog_map_node ${catkin_LIBRARIES})
```

**Step 5. Launch alongside `3DNavUL_Test.launch`.** `sim_nav/src/bot_sim/launch_real/rog_map.launch`:

```xml
<launch>
  <node name="rog_map_node" pkg="bot_sim" type="rog_map_node" output="log">
    <rosparam command="load" file="$(find bot_sim)/config/rog_map_passive.yaml"/>
  </node>
</launch>
```

Add `<include file="$(find bot_sim)/launch_real/rog_map.launch"/>` to [3DNavUL_Test.launch](../Old_Nav/3DNavUL_Test.launch).

**Verify.** `rostopic echo /rog_map_node/rog_map/inf_occ` during a run shows occupied cells; RViz on the `map` frame renders the yellow/gray multi-res grid.

---

### C.3 Mode 2 — Dynamic costmap injection into D*-Lite

**Goal:** `dstarlite` already subscribes to a dynamic map on the topic named by `dynamic_map_topic_name` (default `/grid`). We want ROG-Map to publish an `OccupancyGrid` on that topic representing **currently-observed 3-D obstacles projected to 2-D**.

**Extra node: `rog_map_to_grid`** — a thin post-processor that:
1. Gets ROG-Map occupied cells inside a 2-D box around the robot via `boxSearchInflate(bmin, bmax, OCCUPIED, cells)`.
2. Projects each `(x,y,z)` down to the `(x,y)` plane if `z ∈ [floor_clip, ceiling_clip]`.
3. Publishes a `nav_msgs/OccupancyGrid` on `/grid` with 100 for occupied, 0 for free, origin aligned to the robot.

Skeleton:

```cpp
#include "rog_map/rog_map.h"
#include <nav_msgs/OccupancyGrid.h>

class Rog2Grid {
  rog_map::ROGMap::Ptr map_;
  ros::Publisher pub_;
  ros::Timer timer_;
  double res_ = 0.1, size_ = 20.0;
  double z_lo_ = 0.05, z_hi_ = 1.8;
public:
  Rog2Grid(ros::NodeHandle& nh) {
    map_ = std::make_shared<rog_map::ROGMap>(nh);
    pub_ = nh.advertise<nav_msgs::OccupancyGrid>("/grid", 1);
    timer_ = nh.createTimer(ros::Duration(0.1), &Rog2Grid::tick, this);
  }
  void tick(const ros::TimerEvent&) {
    auto state = map_->getRobotState();
    Eigen::Vector3d c = state.p;
    Eigen::Vector3d bmin(c.x()-size_/2, c.y()-size_/2, z_lo_);
    Eigen::Vector3d bmax(c.x()+size_/2, c.y()+size_/2, z_hi_);
    rog_map::vec_E<Eigen::Vector3d> occ;
    map_->boxSearchInflate(bmin, bmax, rog_map::OCCUPIED, occ);

    nav_msgs::OccupancyGrid g;
    g.header.stamp = ros::Time::now();
    g.header.frame_id = "map";
    g.info.resolution = res_;
    g.info.width  = (unsigned)(size_/res_);
    g.info.height = (unsigned)(size_/res_);
    g.info.origin.position.x = bmin.x();
    g.info.origin.position.y = bmin.y();
    g.info.origin.orientation.w = 1.0;
    g.data.assign(g.info.width*g.info.height, 0);

    for (auto& p : occ) {
      int ix = (int)((p.x()-bmin.x())/res_);
      int iy = (int)((p.y()-bmin.y())/res_);
      if (ix>=0 && iy>=0 && ix<(int)g.info.width && iy<(int)g.info.height)
        g.data[iy*g.info.width + ix] = 100;
    }
    pub_.publish(g);
  }
};
```

With `dynamic_map_topic_name: /grid` (already the default in [dstarlite.launch](../Old_Nav/sim_nav/src/bot_sim/launch_real/dstarlite.launch)), this is **fully plug-and-play**: the planner will avoid whatever ROG-Map currently classifies as inflated-occupied.

**Tuning knobs that matter most for the sentry:**

- `virtual_ceil_height: 1.8` — matches sentry height; suppresses ceiling dust / rafters.
- `virtual_ground_height: 0.05` — ignores ground-hugging LIDAR noise.
- `inflation_step` — keep ≈ robot_radius / `inflation_resolution`. For ~0.4 m radius + 0.2 m res → `inflation_step: 2`.
- `raycasting/local_update_box: [25,25,2.0]` — wide enough to see enemies before the planner re-plan horizon.
- `raycasting/batch_update_size: 1` at 10 Hz LiDAR keeps latency ≤100 ms.
- `map_sliding/threshold: 0.5` m — re-slide only after 0.5 m chassis movement (avoids constant re-indexing when stationary).

**Keep the static map.** `dstarlite` fuses `/map` (static, from `map_server`) and `/grid` (dynamic) — ROG-Map only supplies the latter; the pre-built `innowing.pgm` still defines the field walls / permanent structure.

---

### C.4 Mode 3 — 3-D planning on ROG-Map's InfMap

If/when the team decides to replace or augment D*-Lite:

- The MaRS Lab A* example at [examples/rog_map_example/Apps/astar_example_node.cpp](../ROG-Map/examples/rog_map_example/Apps/) expands neighbours using `map->isOccupiedInflate(p)` — drop-in replacement for a 3-D voxel A*.
- The RRT* example does informed sampling in the inflated free space (`isKnownFreeInflate`).
- ESDF enables smoothing / optimization: any EGO-Planner-style trajectory optimizer can pull gradients via `evaluateFirstGrad`.

To expose the ESDF, add a subclass:

```cpp
class SentryMap : public rog_map::ROGMap {
public:
  using rog_map::ROGMap::ROGMap;
  double dist(const Eigen::Vector3d& p) const { return esdf_map_.getDistance(p); }
};
```

(or add a `friend` declaration to your wrapper).

---

### C.5 Topic/frame bindings for 哨兵 (cheat-sheet)

| ROG-Map param | Set to | Why |
|---|---|---|
| `ros_callback/cloud_topic` | `/aligned_points` | hdl_localization publishes cloud in `map` frame |
| `ros_callback/odom_topic` | `/odom` | hdl_localization odometry (`map → aft_mapped`) |
| `visualization/frame_id` | `map` | Same frame as cloud + odom |
| `virtual_ceil_height` | 1.8 | Sentry height |
| `virtual_ground_height` | 0.05 | Floor clip |
| `map_size` | [30,30,2.5] | Comfortably larger than plan horizon |
| `resolution` / `inflation_resolution` | 0.1 / 0.2 | Matches `innowing.yaml` 2-D map (0.05–0.1 m) |
| `inflation_step` | 2 | ≈ robot radius (2 × 0.2 m = 0.4 m) |
| ROG-Map output topic for planner | `/grid` (via converter) | `dstarlite`'s `dynamic_map_topic_name` default |

### C.6 Integration checklist

- [ ] Symlink/copy `rog_map` into `Old_Nav/sim_nav/src/`.
- [ ] `conda deactivate`, install `libdw-dev` + `Eigen` symlink, rebuild.
- [ ] Create `config/rog_map_passive.yaml` (Section C.2).
- [ ] Create `rog_map_node` driver (Section B.5).
- [ ] Launch alongside `3DNavUL_Test.launch`; verify RViz renders the sliding grid.
- [ ] (Mode 2) Add `rog_map_to_grid` converter on `/grid`.
- [ ] Confirm `dstarlite` re-plans around a moving obstacle introduced into the LiDAR FOV.
- [ ] Tune `virtual_ceil/ground`, `inflation_step`, and `raycasting.local_update_box`.
- [ ] Record a rosbag with `/rog_map/*` for post-match analysis.

---

## Appendix — Files to read for deeper customization

- [rog_map/include/rog_map/rog_map.h](../ROG-Map/rog_map/include/rog_map/rog_map.h)
- [rog_map/include/rog_map/prob_map.h](../ROG-Map/rog_map/include/rog_map/prob_map.h)
- [rog_map/include/rog_map/inf_map.h](../ROG-Map/rog_map/include/rog_map/inf_map.h)
- [rog_map/include/rog_map/esdf_map.h](../ROG-Map/rog_map/include/rog_map/esdf_map.h)
- [rog_map/include/rog_map/free_cnt_map.h](../ROG-Map/rog_map/include/rog_map/free_cnt_map.h)
- [rog_map/include/rog_map/rog_map_core/sliding_map.h](../ROG-Map/rog_map/include/rog_map/rog_map_core/sliding_map.h)
- [rog_map/include/rog_map/rog_map_core/counter_map.h](../ROG-Map/rog_map/include/rog_map/rog_map_core/counter_map.h)
- [rog_map/include/rog_map/rog_map_core/config.hpp](../ROG-Map/rog_map/include/rog_map/rog_map_core/config.hpp)
- [rog_map/include/utils/common_lib.hpp](../ROG-Map/rog_map/include/utils/common_lib.hpp)
- Source: `rog_map.cpp`, `prob_map.cpp`, `inf_map.cpp`, `esdf_map.cpp`, `sliding_map.cpp`, `counter_map.cpp` under [rog_map/src/rog_map/](../ROG-Map/rog_map/src/rog_map/).
