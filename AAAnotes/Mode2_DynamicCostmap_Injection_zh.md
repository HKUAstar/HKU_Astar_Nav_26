# 模式 2 — ROG-Map 动态代价地图注入 D*-Lite 详细实施报告

**目标读者：** 负责将 ROG-Map 接入哨兵导航栈的开发者
**前置阅读：** [ROGMap_Guide_zh.md](ROGMap_Guide_zh.md)、[Pipeline_Report_zh.md](Pipeline_Report_zh.md)
**本报告讨论的模块路径：**
- [Old_Nav/sim_nav/src/bot_sim/src/dstarlite.cpp](../Old_Nav/sim_nav/src/bot_sim/src/dstarlite.cpp)
- [Old_Nav/sim_nav/src/bot_sim/launch_real/dstarlite.launch](../Old_Nav/sim_nav/src/bot_sim/launch_real/dstarlite.launch)
- [ROG-Map/rog_map/include/rog_map/rog_map.h](../ROG-Map/rog_map/include/rog_map/rog_map.h)

---

## 0. 为什么是模式 2？

哨兵现有的规划器 `dstarlite` 是 **纯 2-D** 的：
- **静态层：** 订阅 `/map`（由 `map_server` 从 `innowing.pgm` 加载），表示比赛场地永久墙壁。
- **动态层：** 订阅话题 `/grid`（`nav_msgs/OccupancyGrid`），用于*运行时*感知到的障碍物。

在现有代码里（见 [dstarlite.launch](../Old_Nav/sim_nav/src/bot_sim/launch_real/dstarlite.launch) 第 7 行），`dynamic_map_topic_name` 已经固定为 `/grid`；在 `dstarlite.cpp` 的 `when_receive_new_dynamic_map()` 中（约第 310 行），每次收到 `/grid` 后会做以下事情：

```cpp
map[x][y]->obstacle_possibility
    = std::max((double)dynamic_map_msg->data[index],
               map[x][y]->static_obstacle_possibility);
dstar_update_node(map[x][y]);
```

即：把动态图的占据值与静态图的占据值取**最大**，再触发 D*-Lite 的增量更新。因此：

> **只要把 ROG-Map 感知到的 3-D 障碍投影回 2-D 的 `nav_msgs/OccupancyGrid`，并以 `/grid` 话题发布出去，规划器就会自动避让它——不需要动 `dstarlite.cpp` 一行代码。**

这就是模式 2 的核心杠杆：**在规划器外部**增加一个薄的"桥接节点"，把 ROG-Map 的 3-D 膨胀占据层"降维打击"成 D*-Lite 认识的 2-D 代价图。

---

## 1. 数据流总览

```
           ┌──────────────────────┐
           │  hdl_localization    │
           │  /aligned_points     │     (map 坐标系下的点云)
           │  /odom               │     (map → aft_mapped 里程计)
           └─────────┬────────────┘
                     │
                     ▼
           ┌──────────────────────┐
           │   rog_map_node       │  ← 模式 1 已经起来的节点
           │  维护 3-D 膨胀占据层 │
           │  boxSearchInflate()  │
           └─────────┬────────────┘
                     │ C++ 共享指针 /
                     │ 或 ROS 话题
                     ▼
           ┌──────────────────────┐
           │ rog_map_to_grid 节点 │  ← 本模式新增
           │  3-D → 2-D 投影       │
           │  发布 OccupancyGrid   │
           └─────────┬────────────┘
                     │  /grid  (nav_msgs/OccupancyGrid, map 坐标系)
                     ▼
           ┌──────────────────────┐
           │     dstarlite        │  ← 订阅 /grid，原封不动
           │  静态 /map ⨁ 动态 /grid
           └─────────┬────────────┘
                     │  /cmd_vel
                     ▼
                 底盘运动
```

**关键点：**
1. 模式 2 建立在模式 1（观察模式）之上——`rog_map_node` 必须已经在 `map` 坐标系下正常吃点云 + 里程计。
2. 新增的桥接节点 `rog_map_to_grid` **不是 ROG-Map 内置的**，需要我们自己写（约 80 行 C++）。
3. 不允许任何其他节点在同一话题上再发布 `/grid`，否则两路数据会被相互覆盖。

---

## 2. 前提条件清单

在开始写桥接节点之前，请逐项确认：

| # | 检查项 | 验证方法 |
|---|---|---|
| 1 | 模式 1 已跑通，`rog_map_node` 发布了 `/rog_map_node/rog_map/inf_occ` | `rostopic hz /rog_map_node/rog_map/inf_occ` > 5 Hz |
| 2 | ROG-Map 的 `visualization/frame_id` = `map` | 读 `config/rog_map_passive.yaml` |
| 3 | `hdl_localization` 输出 `/aligned_points` 和 `/odom` | `rostopic list | grep -E '/aligned_points|/odom'` |
| 4 | `dstarlite.launch` 里 `dynamic_map_topic_name == /grid` | 见 [dstarlite.launch](../Old_Nav/sim_nav/src/bot_sim/launch_real/dstarlite.launch#L7) |
| 5 | 机器人半径、高度已知（决定膨胀步数、天花板剪裁） | 哨兵 ≈ 0.4 m 半径，1.8 m 高 |
| 6 | 静态 `/map` 的分辨率一致或接近（`innowing.yaml`，0.05 – 0.1 m） | `rosparam get /map_server` 或看 `innowing.yaml` |

如果第 1 或第 2 项未过，**必须先回到模式 1 把数据链路调通**，不要跳过。

---

## 3. 桥接节点 `rog_map_to_grid` 的职责

这个节点只做四件事：

1. **持有一个 `rog_map::ROGMap` 实例**（读取同一份 YAML 参数），直接利用 ROG-Map 的内部数据结构，避免跨话题序列化的延迟。
2. **周期性地（建议 10 Hz）** 从 ROG-Map 中取出机器人周围一个 2-D 包围盒内的、**已经膨胀过**的占据栅格。
3. **做 Z 轴切片**：只保留 $z \in [z_\text{lo}, z_\text{hi}]$ 范围内的栅格（哨兵身高区间）。
4. **投影并打包成 `nav_msgs/OccupancyGrid`**（占据值 100，未观测 0），`header.frame_id = map`，以 `/grid` 话题发出。

### 3.1 为什么用 `boxSearchInflate()` 而不是订阅 ROG-Map 的话题？

ROG-Map 的 `ROGMap` 类暴露了一个 O(1) 查询、O(k) 遍历的接口：

```cpp
void boxSearchInflate(const Eigen::Vector3d& bmin,
                      const Eigen::Vector3d& bmax,
                      GridType type,               // OCCUPIED / KNOWN_FREE / UNKNOWN
                      vec_E<Eigen::Vector3d>& out);
```

相比之下，如果通过订阅 `/rog_map_node/rog_map/inf_occ`（`PointCloud2`）再反投影：
- 会引入一次额外的序列化/反序列化（几个毫秒）。
- 无法享受 ROG-Map 内部的滑动索引——每帧都要扫描整云。
- 话题中是**可视化用的抽样**，不一定覆盖整个感兴趣区域。

因此模式 2 推荐的做法是**进程内**共享 `ROGMap` 对象：要么把 `rog_map_to_grid` 与 `rog_map_node` 合二为一（同一个可执行文件），要么两者之间通过 `std::shared_ptr<rog_map::ROGMap>` 在同一进程里共享。下面给出的代码采用后者——更简单，也不影响节点解耦。

### 3.2 为什么要做 Z 轴切片？

哨兵高度 ≤ 1.8 m，场地天花板/灯架可能在 2.5 m 以上。如果把 3-D 体素一股脑投影下来：
- **天花板、灯具、横梁**会被当作地面障碍，规划器误以为整条走廊堵死。
- **地面激光噪声**（尘埃、反光）会让机器人"以为"自己脚下有障碍。

因此要保留 $z_\text{lo} = 0.05$ m 到 $z_\text{hi} = 1.8$ m 之间的体素，即：
- **高于地面** 5 cm：丢弃地噪。
- **低于哨兵肩膀** 1.8 m：丢弃天花板/横梁——反正哨兵头顶过不去。

注意：这两个值与 ROG-Map YAML 里的 `virtual_ground_height` / `virtual_ceil_height` 含义**不同**：
- ROG-Map 参数在地图*内部*把超出范围的体素强制当 OCCUPIED 处理。
- 本节点参数在*投影阶段*决定哪些 3-D 占据要"贡献"到 2-D。
两者可以互为冗余保险。

### 3.3 为什么用"膨胀"占据而不是原始占据？

`boxSearchInflate(..., OCCUPIED, ...)` 返回的是 `InfMap` 层的结果，即**每个原始占据体素已经按 `inflation_step` 向周围扩散**过。这样：
- D*-Lite 得到的 `/grid` 自带安全半径（≈ `inflation_step × inflation_resolution`）。
- 不需要在 `dstarlite` 侧再做一次形态学膨胀。
- 机器人半径 0.4 m + `inflation_resolution` 0.2 m → `inflation_step: 2` 就够了。

---

## 4. 代码实现

### 4.1 节点骨架 `rog_map_to_grid.cpp`

将下面的文件放到 `Old_Nav/sim_nav/src/bot_sim/src/rog_map_to_grid.cpp`：

```cpp
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <Eigen/Dense>
#include "rog_map/rog_map.h"

class RogMapToGrid {
public:
    RogMapToGrid(ros::NodeHandle& nh)
    : nh_(nh)
    {
        // 读取桥接节点自己的参数（命名空间 ~grid_bridge/*）
        ros::NodeHandle pnh("~grid_bridge");
        pnh.param("resolution",   res_,   0.1);   // 2-D 栅格分辨率（米）
        pnh.param("size_xy",      size_,  20.0);  // 输出栅格总边长（米），以机器人为中心
        pnh.param("z_lo",         z_lo_,  0.05);  // 投影下界
        pnh.param("z_hi",         z_hi_,  1.8);   // 投影上界
        pnh.param("publish_rate", rate_,  10.0);  // Hz

        // ROG-Map 本体的参数仍走 ~rog_map/* 命名空间
        map_ = std::make_shared<rog_map::ROGMap>(nh_);

        pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/grid", 1);
        timer_ = nh_.createTimer(
            ros::Duration(1.0 / rate_),
            &RogMapToGrid::onTimer, this);
    }

private:
    void onTimer(const ros::TimerEvent&) {
        if (!map_->isMapReady()) return;           // ROG-Map 还没收到足够的数据

        // 1) 取当前机器人位姿（map 坐标系）
        const auto state = map_->getRobotState();
        const Eigen::Vector3d c = state.p;

        // 2) 构造以机器人为中心的 3-D 查询盒
        const double half = size_ * 0.5;
        Eigen::Vector3d bmin(c.x() - half, c.y() - half, z_lo_);
        Eigen::Vector3d bmax(c.x() + half, c.y() + half, z_hi_);

        // 3) 查询所有被标记为"膨胀占据"的体素中心坐标
        rog_map::vec_E<Eigen::Vector3d> occ;
        map_->boxSearchInflate(bmin, bmax, rog_map::OCCUPIED, occ);

        // 4) 准备输出栅格
        nav_msgs::OccupancyGrid g;
        g.header.stamp    = ros::Time::now();
        g.header.frame_id = "map";
        g.info.resolution = res_;
        g.info.width      = static_cast<unsigned>(size_ / res_);
        g.info.height     = static_cast<unsigned>(size_ / res_);
        g.info.origin.position.x = bmin.x();
        g.info.origin.position.y = bmin.y();
        g.info.origin.orientation.w = 1.0;           // 无旋转
        g.data.assign(g.info.width * g.info.height, 0);   // 默认"空闲"

        // 5) 3-D → 2-D 投影（Z 轴已经被查询盒切掉）
        for (const auto& p : occ) {
            const int ix = static_cast<int>((p.x() - bmin.x()) / res_);
            const int iy = static_cast<int>((p.y() - bmin.y()) / res_);
            if (ix < 0 || iy < 0 ||
                ix >= static_cast<int>(g.info.width) ||
                iy >= static_cast<int>(g.info.height))
                continue;
            g.data[iy * g.info.width + ix] = 100;    // 占据
        }

        pub_.publish(g);
    }

    ros::NodeHandle nh_;
    ros::Publisher  pub_;
    ros::Timer      timer_;
    std::shared_ptr<rog_map::ROGMap> map_;
    double res_, size_, z_lo_, z_hi_, rate_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "rog_map_to_grid");
    ros::NodeHandle nh("~");
    RogMapToGrid node(nh);
    ros::AsyncSpinner sp(2);
    sp.start();
    ros::waitForShutdown();
}
```

**关键约束再次提醒：**
- `g.info.origin` 是栅格**左下角**在 `map` 坐标系下的真实坐标。`dstarlite` 的 `when_receive_new_dynamic_map()` 会用 `x = i * resolution + origin.x` 复原世界坐标，然后再走 TF 变换到 `map_frame_name`。由于我们本就发在 `map` 帧下，这一步 TF 是恒等变换，不会引入误差。
- `data[index] == -1` 会被 `dstarlite` 视为"未知"并跳过（见 [dstarlite.cpp#L341](../Old_Nav/sim_nav/src/bot_sim/src/dstarlite.cpp)）。我们全部填 0（free）或 100（occupied）即可，避免让规划器误判。
- `dstarlite` 对每个栅格只做一次 `max(dynamic, static)`，所以**不要**发送 0 之外的"低代价"值——那些会被 max 掉，浪费带宽。

### 4.2 `CMakeLists.txt` 追加项

在 [Old_Nav/sim_nav/src/bot_sim/CMakeLists.txt](../Old_Nav/sim_nav/src/bot_sim/CMakeLists.txt) 里追加：

```cmake
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rog_map
  nav_msgs
)

add_executable(rog_map_to_grid src/rog_map_to_grid.cpp)
target_link_libraries(rog_map_to_grid ${catkin_LIBRARIES})
```

并确保 `package.xml` 增加 `<depend>rog_map</depend>` 与 `<depend>nav_msgs</depend>`。

### 4.3 配置文件

新建 `Old_Nav/sim_nav/src/bot_sim/config/rog_map_bridge.yaml`：

```yaml
# rog_map 本身的参数：沿用模式 1 的 rog_map_passive.yaml 内容
rog_map:
  resolution: 0.1
  inflation_resolution: 0.2
  inflation_step: 2                 # ≈ 机器人半径 0.4 m
  map_size: [30.0, 30.0, 3.0]
  map_sliding:
    enable: true
    threshold: 0.5
  virtual_ceil_height: 2.0
  virtual_ground_height: 0.05
  point_filt_num: 2
  ros_callback:
    enable: true
    cloud_topic: /aligned_points
    odom_topic:  /odom
    odom_timeout: 0.2
  raycasting:
    enable: true
    batch_update_size: 1
    local_update_box: [25.0, 25.0, 2.0]
    ray_range: [0.3, 15.0]
    p_hit: 0.70
    p_miss: 0.35
    p_min: 0.12
    p_max: 0.97
    p_occ: 0.80
    p_free: 0.30
  frontier_extraction_en: false
  esdf:
    enable: false
  visualization:
    enable: true
    time_rate: 5
    range: [20.0, 20.0, 2.0]
    frame_id: map

# 桥接节点独有参数
grid_bridge:
  resolution: 0.1                   # 与静态 /map 分辨率尽量一致
  size_xy: 20.0                     # 规划器关心的视野范围
  z_lo: 0.05
  z_hi: 1.8
  publish_rate: 10.0
```

### 4.4 launch 文件

新建 `Old_Nav/sim_nav/src/bot_sim/launch_real/rog_map_bridge.launch`：

```xml
<launch>
  <!-- ROG-Map + 桥接节点合为一个进程，共享 ROGMap 实例 -->
  <node pkg="bot_sim" type="rog_map_to_grid" name="rog_map_to_grid"
        output="screen">
    <rosparam command="load"
              file="$(find bot_sim)/config/rog_map_bridge.yaml"/>
  </node>
</launch>
```

在顶层 [3DNavUL_Test_with_decision.launch](../Old_Nav/3DNavUL_Test_with_decision.launch) 的**`dstarlite.launch` 之前**插入：

```xml
<include file="$(find bot_sim)/launch_real/rog_map_bridge.launch"/>
```

顺序很重要：`dstarlite` 启动时要能立刻收到一帧 `/grid`，否则会打印 `Failed to get dynamic map` 的警告。

---

## 5. 参数调优指南

下面每一个参数都会直接影响避障行为或延迟，按**优先级**排列：

| # | 参数 | 建议 | 失配后果 |
|---|---|---|---|
| 1 | `grid_bridge/z_lo` = 0.05 m | 略高于地面 | 过低 → 地面噪声变"伪障碍"；过高 → 矮桩（弹药箱）漏检 |
| 2 | `grid_bridge/z_hi` = 1.8 m | 哨兵最高点 | 过低 → 悬挂物漏检；过高 → 天花板被当障碍 |
| 3 | `rog_map/inflation_step` = 2 | ≈ 机器人半径 / 膨胀分辨率 | 过小 → 贴墙受伤；过大 → 窄门过不去 |
| 4 | `rog_map/raycasting/local_update_box` = [25, 25, 2.0] | 覆盖规划视野 | 过小 → 远处敌人看不到；过大 → 每帧计算量暴涨 |
| 5 | `grid_bridge/publish_rate` = 10 Hz | 与 LiDAR 帧率对齐 | 过低 → 避障滞后；过高 → 占 CPU 但 ROG-Map 本身更新不了那么快 |
| 6 | `grid_bridge/size_xy` = 20 m | ≥ 规划器最长搜索半径 | 过小 → 远离机器人的动态障碍被截断 |
| 7 | `grid_bridge/resolution` = 0.1 m | 与 `innowing.yaml` 一致或整倍数 | 不一致 → `dstarlite` 里从动态栅格映射到内部地图时出现锯齿/漏格 |
| 8 | `rog_map/map_sliding/threshold` = 0.5 m | 底盘移动阈值 | 过小 → 静止时反复重索引，CPU 抖动；过大 → 刚移动一段距离地图没跟上 |

**经验法则：** 先把 `z_lo / z_hi / inflation_step` 三个参数调到正常工作，再调 `local_update_box` 和 `publish_rate` 做性能优化。

---

## 6. 联调与验证步骤

### 6.1 分阶段点亮

**阶段 A — 单独启动桥接节点：**
```bash
roslaunch bot_sim rog_map_bridge.launch
```
- `rostopic hz /grid` 应 ≈ 10 Hz。
- `rostopic echo -n1 /grid/info` 检查 `width × height`、`resolution`、`origin` 是否符合预期。
- RViz 加 `Map` 显示 `/grid`，底色灰 = 空闲，黑色 = 占据。

**阶段 B — 与静态地图并联：**
```bash
roslaunch bot_sim dstarlite.launch   # 同时依赖 map_server 发 /map
```
- 日志里应出现 `dynamic_map_topic_name: /grid`（见 [dstarlite.cpp#L647](../Old_Nav/sim_nav/src/bot_sim/src/dstarlite.cpp)）。
- 手持一块纸板在 LiDAR 视野内前后移动，RViz 里 `/grid` 的对应位置应亮起黑色方块。
- 在 RViz 点击目标点，观察规划出的路径是否绕开纸板。

**阶段 C — 全栈联调：**
```bash
bash Old_Nav/run_3DNavUL_Test_with_decision.sh
```
- 决策节点 `decision_node` 发目标，规划器自动避让动态障碍。
- 通过 `rosbag record /grid /map /cmd_vel /odom /rog_map_node/rog_map/inf_occ` 留全量日志。

### 6.2 Quick-Check 命令

```bash
# 检查 /grid 话题在线、频率健康
rostopic hz /grid

# 检查栅格左下角坐标跟随机器人
rostopic echo /grid/info/origin/position -n 5

# 检查占据栅格数量（非零字节数应随障碍出现而增加）
rostopic echo /grid/data -n1 | tr ',' '\n' | grep -c 100
```

### 6.3 常见故障 & 排查

| 症状 | 可能原因 | 定位方法 |
|---|---|---|
| `/grid` 没有发布 | `rog_map_to_grid` 没拿到 `/aligned_points` 或 `/odom` | `rosnode info /rog_map_to_grid` 看订阅量 |
| `/grid` 全部为 0（空闲） | Z 切片过严 / 机器人位姿跳变 / raycasting 未启用 | 临时把 `z_lo=-0.5, z_hi=3.0` 验证 |
| `dstarlite` 认为到处都是障碍 | `frame_id` 不是 `map`，TF 变换后坐标爆掉 | 在 `onTimer` 里打印 `bmin/bmax` 与 `c`，与 `tf_echo map base_link` 对比 |
| 规划器避障有"鬼影" | 障碍离开后 `/grid` 没回落 | 降低 `p_miss`、减小 `ray_range` 上限；或者 `dstarlite` 里 `for_dynamic_map_round` 的 60 帧老化机制会自动擦除（见 [dstarlite.cpp#L354-L361](../Old_Nav/sim_nav/src/bot_sim/src/dstarlite.cpp)） |
| 规划频繁抖动 | `publish_rate` 太高、机器人静止时栅格仍在"微颤" | 提高 `map_sliding/threshold` 至 0.8 m，或在 `onTimer` 里加一层"只有栅格内容变化时才发布"的去抖 |
| `dstarlite` 日志里反复 `tf2::TransformException` | 节点启动顺序错，`map → map` TF 尚未建立 | 在 launch 里给 `rog_map_to_grid` 加 `launch-prefix="bash -c 'sleep 2; exec $0 \"$@\"'"` 延迟启动，或使用 `<node ... respawn="true">` |

---

## 7. 静态图与动态图的融合逻辑（必读）

`dstarlite` 在 [dstarlite.cpp#L346](../Old_Nav/sim_nav/src/bot_sim/src/dstarlite.cpp) 行的融合公式是：

$$
\text{obstacle\_possibility}(x,y) \;=\; \max\bigl(\text{dynamic}_{x,y},\ \text{static}_{x,y}\bigr)
$$

这意味着：

1. **动态图只能"加"障碍，不能"减"静态障碍。** 比赛中用不到打破墙壁的场景，这个语义是合理的。
2. **动态障碍的消失由 `dstarlite` 内部的"老化机制"处理**（`changed_obstacle_nodes` 60 帧清除，见 [dstarlite.cpp#L354](../Old_Nav/sim_nav/src/bot_sim/src/dstarlite.cpp)）。因此当一个动态障碍物离开 LiDAR 视野后：
   - ROG-Map 的概率衰减会把对应体素降到 free。
   - 我们发的 `/grid` 对应位置变 0。
   - `dstarlite` 在 60 帧内不再看到 100，于是把 `obstacle_possibility` 回退到 `static_obstacle_possibility`。
3. **不要擅自把 ROG-Map 的未知区域填成 100。** 否则每当机器人背后的"未观测区"滑过视野边缘，都会被当作障碍——规划器将彻底僵住。

---

## 8. 实施核对表（打钩执行）

- [ ] 模式 1 已跑通，`/rog_map_node/rog_map/inf_occ` 正常。
- [ ] 新建 [rog_map_to_grid.cpp](../Old_Nav/sim_nav/src/bot_sim/src/) 并编译通过。
- [ ] `CMakeLists.txt` / `package.xml` 追加依赖，`catkin_make -DBUILD_TYPE=Release` 无错。
- [ ] 新建 `config/rog_map_bridge.yaml` 和 `launch_real/rog_map_bridge.launch`。
- [ ] 在顶层 launch 中，`rog_map_bridge.launch` 在 `dstarlite.launch` 之前被 include。
- [ ] 单独启动桥接节点，`rostopic hz /grid` 稳定在 10 Hz。
- [ ] 手举纸板动态测试，规划路径实时绕开。
- [ ] 录 rosbag 并离线回放，检查 `/grid` 与 `/rog_map_node/rog_map/inf_occ` 时间戳差 < 100 ms。
- [ ] 调 `z_lo / z_hi` 保证天花板/地噪都不进入代价图。
- [ ] 长时间运行（≥ 10 分钟）看 CPU 占用稳定，无内存泄漏。

---

## 9. 后续可选增强

完成模式 2 后，如果还想进一步提升：

1. **置信度分级：** `dstarlite` 支持 0–100 的占据值。可以按"刚观测到 / 膨胀邻居 / 原始占据"输出不同占据值（例如 70 / 90 / 100），让 D*-Lite 的代价梯度更平滑。
2. **帧率自适应：** 检测到动态障碍时 15 Hz，静止场景降到 5 Hz 节省 CPU。
3. **多层 Z 切片：** 发两路 `/grid_low`（脚下 0–0.3 m）与 `/grid_mid`（胸前 0.5–1.5 m），让决策层区分"卡底盘"与"被击中"。
4. **向模式 3 演进：** 如果 D*-Lite 的 2-D 降维无法处理斜坡/跳板，可以用 ROG-Map 的 `isOccupiedInflate(Eigen::Vector3d)` 直接做 3-D A*（见 [ROGMap_Guide_zh.md](ROGMap_Guide_zh.md) C.4 节）。

---

**至此，模式 2 的完整实施路径已经列出。** 整个工作量约为半天（编码 1–2 h + 联调 2–3 h + 调参 1 h），不改变现有规划器与决策节点的任何逻辑，风险与回退成本都很低，是把 ROG-Map 落地到哨兵最推荐的第一步。
