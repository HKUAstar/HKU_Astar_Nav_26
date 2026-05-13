# Navigation Pipeline Explainer — Static Walls vs Live Obstacles

> How the original (dbscan) and current (ROG-Map Stage 2) pipelines handle
> wall avoidance and real-time obstacle avoidance, derived from reading
> `dstarlite.cpp` and `dbscan_bfs_3D.cpp` in full.

---

## Stage A — Static wall avoidance (identical in both pipelines)

### 1. Map loading at startup

`dstarlite`'s constructor blocks on
`waitForMessage<OccupancyGrid>(static_map_topic)` — it does not start until
`map_server` has published the PGM. PGM binary values: 0 = free, 100 = wall.

```cpp
// dstarlite.cpp constructor
for(int i = 0; i < max_x; i++) {
    map[i] = new Nodeptr[max_y];
    for(int j = 0; j < max_y; j++) {
        map[i][j] = new Node(i, j);
        if(msg->data[i + j * max_x] == 100) {
            map[i][j]->static_obstacle_possibility =
            map[i][j]->obstacle_possibility = 100;
            q.push(map[i][j]);       // seed BFS from every wall cell
        }
    }
}
```

### 2. Static BFS inflation at startup

```cpp
double decrease_rate = 5;
while(!q.empty()) {
    Nodeptr cur = q.front(); q.pop();
    for(int i = 0; i < 8; i++) {
        if(map[nx][ny]->obstacle_possibility <
           cur->obstacle_possibility - (i<4 ? 1.0 : 1.414) * decrease_rate) {
            map[nx][ny]->static_obstacle_possibility =
            map[nx][ny]->obstacle_possibility =
                cur->obstacle_possibility - (i<4 ? 1.0 : 1.414) * decrease_rate;
            q.push(map[nx][ny]);
        }
    }
}
```

A cost gradient radiates outward from every wall cell: value drops 5 per
4-connected step, 5×1.414 per diagonal step. A cell 2 cells from a wall → 90;
10 cells away → 50; ~20 cells away → 0. This creates a soft repulsion band.

Both `static_obstacle_possibility` and `obstacle_possibility` are set to this
value and **frozen**. `static_obstacle_possibility` is the permanent floor that
can never be overwritten by any dynamic data.

### 3. Sigmoid edge cost function

D\*-Lite does not use binary free/occupied. The cost of traversing an edge is:

```cpp
double dstarlite::calculate_edge_value(Nodeptr cur) {
    return std::max(
        1 + L / (1 + exp(-k * (cur->static_obstacle_possibility - x0))),   // static floor
        (1 + L / (1 + exp(-k * (cur->obstacle_possibility       - x0)))) * 0.5  // dyn half-weight
    );
}
```

Edge cost between two nodes = `edge_value(A) × edge_value(B)`. The formula is
a **logistic sigmoid** on obstacle value, tuned by params `k`, `x0`, `L`:

| `obstacle_possibility` | edge multiplier |
|---|---|
| 0 (open space) | ≈ 1.0 |
| 50 (BFS gradient) | rises steeply (controlled by `k`) |
| 100 (at wall) | `1 + L` (very expensive) |

The `max()` ensures the dynamic term (half-weighted) can only raise cost above
the static floor, never reduce it. Near-wall cells stay expensive regardless of
what any dynamic sensor reports.

### 4. D\*-Lite path finding with Link-Cut Tree

D\*-Lite is a backwards-Dijkstra replanner. Each node maintains:

- `rhs` — 1-step lookahead cost (min over all 8 neighbors of `neighbor.dis_to_goal + edge_cost`)
- `dis_to_goal` — settled shortest-path distance
- `succ` — pointer to next node toward the goal

The planner uses a **Link-Cut Tree (LCT)** to maintain the `succ` pointer
forest. Checking "is the current path still valid?" is O(log n):

```cpp
bool dstarlite::old_path_still_work(int sx, int sy) {
    return lct->find_root(map[sx][sy]) == lct->find_root(final_goal_node);
}
```

If the LCT says the path is still connected, `dstar_main` returns immediately
with zero replanning work. Only broken paths trigger `dstar_update`.

### 5. Velocity output

`publish()` looks **12 `succ` hops ahead** on the path to compute a lookahead
waypoint, computes the direction vector, applies a TF rotation into
`robot_frame`, and scales by velocity from `calculate_velocity`. Output is a
continuous `cmd_vel` — the robot tracks the direction of the 12-step lookahead
vector, not individual waypoints.

---

## Stage B — Original dynamic obstacle handling: dbscan pipeline

The original designed path. **Commented out in production** before Stage 1.

### Data flow

```
/test_scan (PointCloud2)  →  dbscan_bfs_3D  →  /grid (OccupancyGrid, gimbal_frame)
```

### Per-frame processing in `dbscan_bfs_3D`

**Step 1 — DBSCAN clustering.** Raw 3D points clustered with `eps=0.2 m`,
`minPts=7`. Points in clusters smaller than `minPts` are marked `noise=true`
and discarded. Coherent physical objects survive; isolated sensor speckle does
not.

**Step 2 — Grid projection.** Surviving cluster points projected to a
robot-centric 2D grid: `MAXN=100 × 100` cells, `resolution=0.05 m` → 5 × 5 m
window. Origin is `(-2.5, -2.5)` in `gimbal_frame` (rotates with robot).

**Step 3 — BFS inflation.** Each cluster point paints value=100; BFS spreads
outward decrementing by `dfs_decrease=5` per step — same gradient pattern as
the static startup BFS.

**Step 4 — `obscured_point_filter`.** The most sophisticated part. Uses a
**segment tree on polar angle sweep** to identify which grid cells lie
geometrically behind obstacles from the robot's viewpoint. Those cells are
marked `-1` (unknown/occluded). D\*-Lite's `when_receive_new_dynamic_map`
skips `data[index] == -1` cells, preventing phantom free-space behind opaque
objects from clearing real obstacles. Stage 2 does not need this because
ROG-Map's raycasting handles occlusion physically (see below).

### D\*-Lite integration of the dbscan grid

`when_receive_new_dynamic_map()` does three things on each `/grid` message:

**TF transform**: `gimbal_frame` rotates with the robot body. Each cell's world
coordinate is computed via `tfBuffer.lookupTransform("map", "gimbal_frame")`.
As the robot rotates, the dynamic costmap is re-aligned into map frame.

**Merge with static floor**:
```cpp
map[x][y]->obstacle_possibility = std::max(
    (double)dynamic_map_msg->data[index],
    map[x][y]->static_obstacle_possibility
);
```
Dynamic data can only raise cost above the static floor; it cannot clear a wall.

**Register change and trigger replanning**:
```cpp
map[x][y]->round_for_dynamic_map = for_dynamic_map_round;
changed_obstacle_nodes.insert(map[x][y]);
dstar_update_node(map[x][y]);
```
`dstar_update_node` recalculates `rhs` and if `rhs ≠ dis_to_goal`, puts the
node back in the open set. The next `dstar_main` call propagates cost changes
until the LCT reconnects start to goal.

**60-round aging decay**: `changed_obstacle_nodes` is sorted by
`round_for_dynamic_map`. On every callback, nodes whose
`round_for_dynamic_map + 60 <= for_dynamic_map_round` are popped from the
front — their `obstacle_possibility` resets to `static_obstacle_possibility`
and `dstar_update_node` is called again. At 1 Hz → 60 s decay. At 10 Hz
(per-frame) → 6 s decay.

---

## Stage C — Current dynamic obstacle handling: ROG-Map Stage 2 pipeline

D\*-Lite code is **byte-for-byte identical**. `when_receive_new_dynamic_map`,
the aging logic, the edge cost function — nothing changed. Only the source of
`dynamic_map_msg` changed from `/grid` → `/rog_grid`.

### What changes in the sensor-to-grid path

| | dbscan pipeline | ROG pipeline |
|---|---|---|
| **Source** | Raw `/test_scan` per frame | ROG-Map accumulated Bayesian voxels |
| **Obstacle detection** | DBSCAN: geometric clustering, per-frame | Probabilistic log-odds raycasting; cell needs multiple hits to exceed `p_occ` threshold |
| **Noise rejection** | DBSCAN `minPts` filter (require N nearby points) | Bayesian accumulation — single-frame noise dissipates; real obstacles accumulate |
| **Occlusion handling** | Explicit `obscured_point_filter` marks `-1` | ROG-Map `raycasting.enable=true` — free-space ray actively clears previously-occupied voxels |
| **Coordinate frame** | `gimbal_frame` (robot-body, rotates with robot) | `world` (fixed, map-anchored) |
| **Grid origin** | Always centred on robot per-frame | Centred on robot, snapped to 0.2 m world lattice |
| **Inflation** | BFS gradient per-frame in 2D | ROG-Map 3D inflation (`inflation_step` × 0.2 m radius), bridge paints 2×2 at 0.1 m |
| **Vertical filtering** | None — all z projected flat | `z_min=0.05 / z_max=2.0` band; floor and ceiling excluded |
| **Persistence** | Not remembered between frames (object must be visible) | Voxels persist; robot can turn away, obstacle stays until raycasting clears it |

### Why ROG-Map's raycasting handles occlusion differently

Each LiDAR ray updates log-odds **along its entire length**:

- Every voxel the ray **passes through** before the endpoint:
  `l += log(p_miss / (1 - p_miss))` → negative (free evidence)
- The voxel at the **endpoint** (the obstacle surface):
  `l += log(p_hit / (1 - p_hit))` → positive (occupied evidence)

Voxels are clamped between `[p_min, p_max]`. Classification:
- `probability > p_occ` → **OCCUPIED**
- `probability < p_free` → **FREE**

When an obstacle moves away, subsequent rays pass through where it was and
actively decrement its log-odds. After enough free-space rays, it drops below
`p_occ` and is reclassified. Clearing speed is governed by `p_miss`, `p_max`,
and the LiDAR scan rate.

### How D\*-Lite sees the ROG grid

From D\*-Lite's perspective `/rog_grid` is indistinguishable from `/grid`.
`when_receive_new_dynamic_map`:

1. Looks up `tfBuffer.lookupTransform("map", "world")` — Stage 1 identity TF,
   zero translation and rotation. Coordinates pass through unchanged.
2. For each cell: `obstacle_possibility = max(rog_grid[cell], static_obstacle_possibility)`.
3. 60-round aging runs identically. At 11 Hz cloud-driven publish rate, a
   removed obstacle fades in ≈ 60/11 = 5.5 s. At 1 Hz heartbeat (cloud
   stalled), 60 s.

### Complete signal path end to end

```
LiDAR (Livox MID-360)
    │ /livox/lidar  (raw PointCloud2)
    ▼
hdl_localization_nodelet
    │ /aligned_points  (registered, map frame)
    │ /odom            (robot pose, map frame, ~50 Hz)
    ▼
rog_map_node  [Stage 1]
    │ Bayesian log-odds raycasting per ray
    │ 3D inflation in voxel space
    │ /rog_map_node/rog_map/inf_occ  (PointCloud2, world frame, ~10 Hz)
    ▼
rog_map_to_grid_node  [Stage 2 bridge]
    │ z-filter [z_min, z_max]
    │ origin snap to 0.2 m world lattice
    │ 2×2 cell paint at 0.1 m resolution
    │ /rog_grid  (OccupancyGrid, world frame, 200×200)
    ▼
dstarlite
    │ TF transform world→map (identity)
    │ merge: obstacle_possibility = max(rog_grid[cell], static_obstacle_possibility)
    │ dstar_update_node → edge cost recalc → LCT path reconnect
    │ 60-round aging → auto-clear stale obstacles
    │ 12-step lookahead velocity command
    ▼
/cmd_vel → ser2msg → chassis MCU
```

### Summary: what each layer is responsible for

| Layer | Responsible for |
|---|---|
| **map_server PGM** | Permanent wall geometry — never changes at runtime |
| **dstarlite BFS inflation** (startup) | Soft repulsion gradient around every static wall — baked into `static_obstacle_possibility` |
| **`calculate_edge_value` sigmoid** | Translates obstacle value to traversal cost — robot prefers paths far from obstacles, not just avoids 100-value cells |
| **ROG-Map raycasting** | Per-ray probabilistic occupancy + free-space clearing — produces stable, noise-free 3D voxel map |
| **Bridge z-filter** | Drops floor speckle and overhead objects; keeps obstacle-height voxels only |
| **Bridge 2×2 paint** | Converts 0.2 m voxel footprint into 0.1 m grid cells without Swiss-cheese gaps |
| **`when_receive_new_dynamic_map`** | Merges dynamic grid into planner cost field, respects static floor, triggers incremental replanning |
| **60-round aging** | Auto-decays dynamic obstacles no longer being reported |
| **LCT `find_root`** | Makes "is path still valid?" O(log n) — enables 100 Hz planning loop |
