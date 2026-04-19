# Understanding `graph_slam.cpp` — A Comprehensive Guide

## Table of Contents

1. [What Is This File?](#1-what-is-this-file)
2. [Prerequisites — What You Need to Know First](#2-prerequisites)
3. [The Big Picture: Graph-Based SLAM](#3-the-big-picture)
4. [Key Concepts](#4-key-concepts)
5. [Code Walkthrough](#5-code-walkthrough)
   - [5.1 Includes & Registration](#51-includes--registration)
   - [5.2 Constructor & Solver Setup](#52-constructor--solver-setup)
   - [5.3 Adding Nodes (Vertices)](#53-adding-nodes-vertices)
   - [5.4 Adding Edges (Constraints)](#54-adding-edges-constraints)
   - [5.5 Robust Kernels](#55-robust-kernels)
   - [5.6 Optimization](#56-optimization)
   - [5.7 Save & Load](#57-save--load)
6. [How It Fits Into hdl_graph_slam](#6-how-it-fits-into-hdl_graph_slam)
7. [Parameter Reference](#7-parameter-reference)
8. [Common Scenarios](#8-common-scenarios)
9. [Debugging Tips](#9-debugging-tips)

---

## 1. What Is This File?

`graph_slam.cpp` implements the `GraphSLAM` class — a **wrapper around the g2o library** that provides a clean, SLAM-specific API for building and optimizing pose graphs.

Think of it as the **mathematical engine** of hdl_graph_slam. While other nodelets handle LiDAR processing, scan matching, and floor detection, this class takes all those measurements and finds the best set of robot poses that are consistent with ALL measurements simultaneously.

**It does NOT:**
- Process point clouds
- Perform scan matching
- Detect loop closures
- Interface with ROS topics

**It DOES:**
- Store robot poses and landmarks as graph nodes
- Store measurements as graph edges (constraints)
- Run optimization to find globally consistent poses
- Save/load the graph to/from disk

---

## 2. Prerequisites

### What is g2o?

**g2o** (General Graph Optimization) is a C++ library for optimizing graph-based nonlinear error functions. It is the mathematical solver behind many SLAM systems.

Website: [https://github.com/RainerKuemmerle/g2o](https://github.com/RainerKuemmerle/g2o)

### What is Eigen?

**Eigen** is a C++ linear algebra library. You'll see types like:
- `Eigen::Isometry3d` — A 4×4 transformation matrix (rotation + translation) representing a 6-DOF pose
- `Eigen::Vector3d` — A 3D vector (x, y, z)
- `Eigen::Vector4d` — A 4D vector (used for plane coefficients: ax + by + cz + d = 0)
- `Eigen::MatrixXd` — A dynamically-sized matrix (used for information matrices)
- `Eigen::Quaterniond` — A unit quaternion representing rotation

### What is an Information Matrix?

The **information matrix** (Ω) is the **inverse of the covariance matrix**:

$$\Omega = \Sigma^{-1}$$

- **High values** on the diagonal = high confidence in that measurement = strong constraint
- **Low values** = low confidence = weak constraint (optimizer can easily "stretch" it)

For a 6-DOF pose (x, y, z, roll, pitch, yaw), the information matrix is 6×6:

$$\Omega = \begin{bmatrix} \omega_x & & & & & \\ & \omega_y & & & & \\ & & \omega_z & & & \\ & & & \omega_{roll} & & \\ & & & & \omega_{pitch} & \\ & & & & & \omega_{yaw} \end{bmatrix}$$

---

## 3. The Big Picture

### The SLAM Problem

A robot moves through an environment, taking LiDAR scans. Over time, small scan matching errors **accumulate** (drift). After 100m of driving, the robot's estimated position might be off by several meters.

### The Graph Solution

```
  Keyframe 1 ---[odometry]--> Keyframe 2 ---[odometry]--> Keyframe 3
       |                                                        |
       +------------------[loop closure]------------------------+
```

1. Each keyframe becomes a **node** (vertex) in a graph
2. Each odometry measurement becomes an **edge** between consecutive nodes
3. When the robot revisits a location, a **loop closure edge** connects distant nodes
4. **Optimization** adjusts all node positions to satisfy all edges simultaneously

This is like a spring system:
- Each edge is a spring with a "rest length" (the measurement)
- Optimization finds the configuration where springs have minimum total energy

### Before vs After Optimization

```
BEFORE (with drift):              AFTER (optimized):
   1--2--3--4--5                     1--2--3--4--5
                 \                   |             |
                  6                  6             |
                   \                 |             |
                    7--8--9          7--8--9-------+
                         (gap!)         (loop closed!)
```

---

## 4. Key Concepts

### Nodes (Vertices)

| Node Type | Represents | Dimensions | Used For |
|-----------|-----------|------------|----------|
| `VertexSE3` | Robot pose (position + orientation) | 6-DOF | Every keyframe |
| `VertexPlane` | A planar surface | 4D (a,b,c,d) | Floor detection |
| `VertexPointXYZ` | A 3D point landmark | 3D (x,y,z) | Point features |

### Edges (Constraints)

| Edge Type | Connects | Measurement | Used For |
|-----------|----------|-------------|----------|
| `EdgeSE3` | Pose ↔ Pose | Relative transform (6-DOF) | Odometry, loop closure |
| `EdgeSE3Plane` | Pose ↔ Plane | Plane coefficients | Floor constraint |
| `EdgeSE3PointXYZ` | Pose ↔ Point | 3D observation | Point landmarks |
| `EdgeSE3PriorXY` | Pose → fixed | 2D position | GPS (no altitude) |
| `EdgeSE3PriorXYZ` | Pose → fixed | 3D position | GPS (with altitude) |
| `EdgeSE3PriorVec` | Pose → fixed | Direction vector | IMU gravity |
| `EdgeSE3PriorQuat` | Pose → fixed | Quaternion | IMU compass |
| `EdgePlanePriorNormal` | Plane → fixed | Normal vector | Floor should be "up" |
| `EdgePlanePriorDistance` | Plane → fixed | Scalar distance | Floor height |
| `EdgePlaneIdentity` | Plane ↔ Plane | Identity | Same physical plane |
| `EdgePlaneParallel` | Plane ↔ Plane | Normal | Parallel planes |
| `EdgePlanePerpendicular` | Plane ↔ Plane | Normal | Perpendicular planes |

**Binary edges** connect two nodes. **Unary edges** (priors) connect one node to a fixed reference.

### Robust Kernels

Outlier measurements (e.g., a false loop closure) can catastrophically distort the map. Robust kernels limit the influence of large errors:

```
Standard quadratic cost:        Huber cost:
   cost                            cost
    |        /                      |      /
    |       /                       |     / (linear for outliers)
    |      /                        |    /
    |     /                         |   /-------
    |    /                          |  / 
    |   /                           | /  (quadratic for inliers)
    |  /                            |/
    | /                             +
    |/                              
    +---------- error               +---------- error
```

---

## 5. Code Walkthrough

### 5.1 Includes & Registration

```cpp
G2O_USE_OPTIMIZATION_LIBRARY(pcg)
G2O_USE_OPTIMIZATION_LIBRARY(cholmod)
G2O_USE_OPTIMIZATION_LIBRARY(csparse)
```

These macros **register** sparse linear solvers with g2o. Without them, g2o wouldn't know how to solve the linear systems during optimization.

| Solver | Speed | License | Notes |
|--------|-------|---------|-------|
| PCG | Slow | Free | Iterative, no external deps |
| CHOLMOD | **Fast** | **GPL** | Direct Cholesky factorization |
| CSPARSE | Medium | LGPL* | Direct solver |

```cpp
namespace g2o {
G2O_REGISTER_TYPE(EDGE_SE3_PLANE, EdgeSE3Plane)
// ... more registrations
}
```

These register custom edge types so g2o can serialize them (save/load to file). Each type gets a unique string name.

### 5.2 Constructor & Solver Setup

```cpp
GraphSLAM::GraphSLAM(const std::string& solver_type) {
  graph.reset(new g2o::SparseOptimizer());
  // ...
  g2o::OptimizationAlgorithm* solver = solver_factory->construct(solver_type, solver_property);
  graph->setAlgorithm(solver);
}
```

The solver type string follows this naming pattern:

```
<algorithm>_<block_structure>[_<linear_solver>]
```

| Part | Options | Meaning |
|------|---------|---------|
| Algorithm | `gn` / `lm` | Gauss-Newton / Levenberg-Marquardt |
| Block structure | `var` / `fix6_3` / `fix3_2` | Variable / Fixed 6×3 / Fixed 3×2 |
| Linear solver | `cholmod` / `csparse` / `pcg` | Sparse solver backend |

**Common choices:**

| Solver String | Description | When to Use |
|--------------|-------------|-------------|
| `lm_var_cholmod` | **Default & recommended.** LM with Cholmod. | General use |
| `gn_var_cholmod` | Faster convergence near optimum. | If LM is too slow |
| `lm_var` | LM with default solver (PCG). | If Cholmod unavailable |

**Levenberg-Marquardt vs Gauss-Newton:**
- **LM** is more robust — it works even when the initial guess is far from optimal (adds damping)
- **GN** is faster but can diverge if the initial guess is poor

### 5.3 Adding Nodes (Vertices)

All node-adding methods follow the same pattern:

```cpp
g2o::VertexSE3* GraphSLAM::add_se3_node(const Eigen::Isometry3d& pose) {
  g2o::VertexSE3* vertex(new g2o::VertexSE3());     // 1. Create vertex
  vertex->setId(static_cast<int>(graph->vertices().size()));  // 2. Auto-assign ID
  vertex->setEstimate(pose);                          // 3. Set initial value
  graph->addVertex(vertex);                           // 4. Add to graph
  return vertex;                                      // 5. Return pointer
}
```

The returned pointer is stored externally (in `HdlGraphSlamNodelet`) so edges can reference it later.

**Important:** The `setEstimate()` value is the **initial guess**. The optimizer will modify it during optimization. After optimization, call `vertex->estimate()` to get the corrected pose.

### 5.4 Adding Edges (Constraints)

All edge-adding methods follow the same pattern:

```cpp
g2o::EdgeSE3* GraphSLAM::add_se3_edge(v1, v2, relative_pose, information_matrix) {
  g2o::EdgeSE3* edge(new g2o::EdgeSE3());
  edge->setMeasurement(relative_pose);      // What we measured
  edge->setInformation(information_matrix);  // How confident we are
  edge->vertices()[0] = v1;                  // First connected node
  edge->vertices()[1] = v2;                  // Second connected node
  graph->addEdge(edge);
  return edge;
}
```

**The error function** for an SE3 edge is:
$$e = \log(Z^{-1} \cdot v_1^{-1} \cdot v_2)$$

Where:
- $Z$ = measurement (relative_pose)
- $v_1$, $v_2$ = current estimates of the two nodes
- $\log()$ = SE3 logarithmic map (converts transform to 6D error vector)

If $v_1$ and $v_2$ are perfectly consistent with measurement $Z$, the error is zero.

### 5.5 Robust Kernels

```cpp
void GraphSLAM::add_robust_kernel(edge, kernel_type, kernel_size) {
  g2o::RobustKernel* kernel = robust_kernel_factory->construct(kernel_type);
  kernel->setDelta(kernel_size);
  edge_->setRobustKernel(kernel);
}
```

In hdl_graph_slam, robust kernels are typically applied to:
- **Loop closure edges** with `Huber` kernel (default) — to reject false loop closures
- **Odometry edges** with `NONE` (default) — odometry is generally reliable

The `kernel_size` (δ) parameter:
- **Smaller δ** → more aggressive outlier rejection (errors > δ treated as outliers)
- **Larger δ** → less aggressive (only very large errors are down-weighted)

### 5.6 Optimization

```cpp
int GraphSLAM::optimize(int num_iterations) {
  if(graph->edges().size() < 10) return -1;     // Skip if too few constraints
  graph->initializeOptimization();                // Build solver matrices
  graph->setVerbose(true);                        // Print iteration details
  int iterations = graph->optimize(num_iterations);
  return iterations;
}
```

**What happens inside `graph->optimize()`:**

```
For each iteration:
  1. Compute error for every edge:  e_i = measurement_i - predicted_i
  2. Build the Hessian matrix H and gradient b from all edges:
     H = Σ J_i^T * Ω_i * J_i
     b = Σ J_i^T * Ω_i * e_i
     (where J_i is the Jacobian of edge i)
  3. Solve H * Δx = -b  (using Cholmod/CSparse/PCG)
  4. Update all node estimates:  x ← x + Δx
  5. Check convergence: if |Δx| < epsilon, stop early
```

**The chi2 (χ²) value** is the total weighted squared error:

$$\chi^2 = \sum_i e_i^T \cdot \Omega_i \cdot e_i$$

- χ² should **decrease** after optimization
- If it increases, something is wrong (bad initial guess, conflicting constraints)

### 5.7 Save & Load

```cpp
void GraphSLAM::save(const std::string& filename) {
  graph->save(ofs);                                    // Save graph in g2o text format
  g2o::save_robust_kernels(filename + ".kernels", graph);  // Save kernel info separately
}
```

The `.g2o` file format is human-readable text:
```
VERTEX_SE3:QUAT 0 0.0 0.0 0.0 0.0 0.0 0.0 1.0
VERTEX_SE3:QUAT 1 1.0 0.0 0.0 0.0 0.0 0.0 1.0
EDGE_SE3:QUAT 0 1  1.0 0.0 0.0 0.0 0.0 0.0 1.0  500 0 0 0 0 0 500 0 0 0 0 500 0 0 0 500 0 0 500 0 500
```

---

## 6. How It Fits Into hdl_graph_slam

```
┌─────────────────────┐
│   LiDAR Driver      │ /livox/lidar
└──────────┬──────────┘
           ▼
┌─────────────────────┐
│ PrefilteringNodelet  │ Downsample + denoise
└──────────┬──────────┘
           ▼  /filtered_points
    ┌──────┴──────┐
    ▼             ▼
┌────────┐  ┌──────────┐
│  Scan  │  │  Floor   │
│Matching│  │Detection │
│Odometry│  │(optional)│
└───┬────┘  └────┬─────┘
    │             │
    ▼ /odom_hdl   ▼ /floor_coeffs
┌─────────────────────────────────┐
│    HdlGraphSlamNodelet          │
│  ┌───────────────────────┐      │
│  │  GraphSLAM instance   │◄─────── THIS FILE
│  │  (graph_slam.cpp)     │      │
│  │                       │      │
│  │  • add_se3_node()     │      │  ← Called for each keyframe
│  │  • add_se3_edge()     │      │  ← Called for odometry + loop closure
│  │  • add_se3_prior_*()  │      │  ← Called for GPS / IMU data
│  │  • add_se3_plane_*()  │      │  ← Called for floor constraints
│  │  • optimize()         │      │  ← Called every graph_update_interval
│  │  • save() / load()    │      │  ← Called via ROS services
│  └───────────────────────┘      │
└──────────┬──────────────────────┘
           ▼
    /hdl_graph_slam/map_points (optimized map)
    TF: map → camera_init (corrected transform)
```

**Call sequence during runtime:**

1. New keyframe arrives → `add_se3_node()` creates a pose node
2. Odometry measurement → `add_se3_edge()` between consecutive nodes
3. GPS arrives → `add_se3_prior_xyz_edge()` on the nearest node
4. IMU arrives → `add_se3_prior_vec_edge()` (gravity) or `add_se3_prior_quat_edge()` (compass)
5. Floor detected → `add_se3_plane_edge()` + `add_plane_node()`
6. Loop detected → `add_se3_edge()` between distant nodes + `add_robust_kernel(Huber)`
7. Timer fires → `optimize()` runs, correcting all poses

---

## 7. Parameter Reference

These parameters from the launch file directly affect `GraphSLAM`:

| Parameter | Default | Effect on graph_slam.cpp |
|-----------|---------|--------------------------|
| `g2o_solver_type` | `lm_var_cholmod` | Passed to `GraphSLAM()` constructor |
| `g2o_solver_num_iterations` | `512` | Passed to `optimize()` |
| `odometry_edge_robust_kernel` | `NONE` | Passed to `add_robust_kernel()` |
| `odometry_edge_robust_kernel_size` | `1.0` | Kernel delta for odometry edges |
| `loop_closure_edge_robust_kernel` | `Huber` | Passed to `add_robust_kernel()` |
| `loop_closure_edge_robust_kernel_size` | `1.0` | Kernel delta for loop closure edges |
| `gps_edge_robust_kernel` | `NONE` | Kernel for GPS edges |
| `gps_edge_stddev_xy` | `20.0` | Becomes diagonal of GPS info matrix |
| `gps_edge_stddev_z` | `5.0` | Z component of GPS info matrix |
| `imu_orientation_edge_stddev` | `1.0` | IMU orientation info matrix |
| `imu_acceleration_edge_stddev` | `1.0` | IMU gravity info matrix |
| `floor_edge_stddev` | `10.0` | Floor plane info matrix |
| `use_const_inf_matrix` | `false` | Use constant vs. fitness-based info matrices |
| `fix_first_node` | `true` | Whether to anchor the first pose node |
| `fix_first_node_stddev` | `10 10 10 1 1 1` | Anchor strength (lower = more fixed) |

---

## 8. Common Scenarios

### Scenario 1: Simple Corridor Mapping

```
Nodes: [1]--[2]--[3]--[4]--[5]
Edges: odometry between each pair
Result: A chain of poses. No loop closure, so drift accumulates.
```

### Scenario 2: Loop Closure

```
Nodes: [1]--[2]--[3]--[4]--[5]
Edges: odometry + loop closure edge [5]→[1]
Result: Optimization distributes the loop closure correction across ALL poses.
         Poses 2-4 shift slightly so the loop connects cleanly.
```

### Scenario 3: GPS-Aided Mapping

```
Nodes: [1]--[2]--[3]--[4]--[5]
Edges: odometry + GPS priors on [1], [3], [5]
Result: Poses are pulled toward GPS coordinates. The strength depends on
         gps_edge_stddev_xy (lower = stronger pull toward GPS).
```

### Scenario 4: Floor-Constrained Indoor Mapping

```
Nodes: [1]--[2]--[3]--[4]--[5]--[Floor]
Edges: odometry + floor plane edges from each pose to [Floor]
Result: All poses are corrected so they agree on where the floor is.
         This prevents Z-axis and tilt drift in flat environments.
```

---

## 9. Debugging Tips

### Check optimization health

Watch the console output during optimization:
```
--- pose graph optimization ---
nodes: 45   edges: 89
chi2: (before)1234.5 -> (after)12.3    ← Good! Large reduction
time: 0.045[sec]
```

**Warning signs:**
- χ² increases → conflicting constraints or bad initial guess
- χ² barely changes → already optimal, or constraints are too weak
- "failed to allocate solver" → missing solver library (install cholmod)
- Very high χ² after optimization → possible outlier edge (enable robust kernels)

### Inspect the graph

Use the dump service to save the full graph:
```bash
rosservice call /hdl_graph_slam/dump "destination: '/tmp/graph_dump'"
```

This saves:
- The g2o graph file (viewable with g2o_viewer)
- All keyframe point clouds
- Floor coefficients and odometry data

### Tune constraint weights

If the map looks wrong, consider:
- **Map is "wobbly"**: Increase `g2o_solver_num_iterations` or lower `graph_update_interval`
- **Loop closures cause jumps**: Lower `loop_closure_edge_robust_kernel_size`
- **GPS pulling poses to wrong locations**: Increase `gps_edge_stddev_xy` (weaker GPS)
- **Floor warped**: Decrease `floor_edge_stddev` (stronger floor constraint)
- **Z-axis drift**: Enable `enable_floor_detection` or `enable_imu_acc`
