# ROG-Map — 简介、功能及其在“哨兵”中的集成指南

**本地工作区源码：** [ROG-Map/](../ROG-Map/) · **上游仓库：** https://github.com/hku-mars/ROG-Map · **论文：** *IROS 2024*, Ren et al., arXiv:2302.14819
**目标管线：** 记录在 [Pipeline_Report.md](Pipeline_Report.md) 中的哨兵技术栈。

本指南分为两部分：
- **A部分（通俗介绍）：** ROG-Map 是什么，以及它能为你带来什么。
- **B部分（集成视角）：** 如何将其接入现有的哨兵 `livox_ros_driver2 → hdl_localization → bot_sim/dstarlite → decision_node` 架构管线。

---

## A部分 — ROG-Map 是什么

### A.1 一段话总结

ROG-Map（以机器人为中心的占据栅格地图，**R**obocentric **O**ccupancy **G**rid **Map**）是香港大学 MaRS 实验室开发的一个基于 ROS/C++ 的**局部**体素地图库。其设计初衷是让运动规划器能够“以**激光雷达的帧率**”在数十米的范围内、以 10 cm 的分辨率、以常数时间复杂度回答诸如*“这个点是否空闲？”*、*“距离最近的障碍物有多远？”*，以及*“已知空间的边界（Frontier）在哪里？”*等问题。它不是一个 SLAM 系统——它接收*已经完成配准*的点云 + 里程计，并在其四周维护一个**跟随机器人移动**的稠密栅格地图。

你可以将其视为一个更现代的、面向规划的 `octomap_server` / `costmap_2d` 替代方案：

| 需求 | 传统工具 | ROG-Map |
|---|---|---|
| 从激光雷达构建地图 | OctoMap | ✔（仅限局部窗口） |
| 根据机器人半径膨胀障碍物 | `costmap_2d` | ✔，**增量式**，多分辨率 |
| 查询 ESDF（到最近障碍物的距离 + 梯度） | `dynamicEDT3D` / Fast-Planner 的 ESDF | ✔，**滑动式**，支持三线性插值及解析解梯度/Hessian矩阵 |
| 寻找未知与空闲交界的边界（Frontier）以进行探索 | 外部扩展包 | ✔，每个栅格 $O(1)$，增量式更新 |
| 在 0.1 m分辨率 / 30×30×5 m 的窗口下以 10+ Hz 运行 | 勉强/吃力 | 专为此设计 |

### A.2 共享同一容器的四重子地图

这四种子地图共享同一个以机器人为中心的**滑动**体素栅格（在机器人平移时只重新索引，不进行内存拷贝）：

1. **ProbMap (概率地图)** — 概率占据栅格（对数几率贝叶斯更新）。单元格被分类为 `UNKNOWN` (未知)、`KNOWN_FREE` (已知空闲) 或 `OCCUPIED` (占据)。分辨率通常为 0.05–0.1 m。由专门针对激光雷达的射线投射器 (Raycaster) 更新。
2. **InfMap (膨胀地图)** — 将 ProbMap **按机器人半径膨胀**后的地图。能够以比 ProbMap *更粗*的分辨率运行（例如 0.1 m的ProbMap对应 0.2 m的InfMap），从而降低规划器的碰撞检测计算成本。“增量式”意味着：每当一个精细网格在占据/空闲之间翻转时，只有受影响的 `(2r+1)³` 个粗网格会增减计数器；无需全图遍历。
3. **ESDFMap (欧氏符号距离场地图)** — 在同一滑动窗口上计算的符号距离变换（障碍物外为正，内部为负）。为基于梯度的轨迹优化器公开了 `getDistance(p)`, `evaluateEDT`, `evaluateFirstGrad`, `evaluateSecondGrad` 接口。
4. **FreeCntMap / Frontier (前沿地图)** — 每个未知单元格记录其 27 个邻居中有多少个是 `KNOWN_FREE` 的；如果是 $\ge 1$，则它是一个**前沿 (Frontier)**。可用于主动探索 / 主动 SLAM。

### A.3 核心特性（为什么选择它而不是 OctoMap + Costmap）

| 特性 | 意义 |
|---|---|
| **零拷贝滑动 (Zero-copy sliding)** | 栅格的内存永远不会使用 memmove 移动；只有一个整数原点索引发生平移，离开窗口的“环带”会被重置。内存和更新开销由窗口大小决定，与机器人的行驶距离**无关**。 |
| **有界内存** | 设置你的窗口大小（例如 30×30×5 m @ 0.1 m = 约 450万 个单元格）。运行时时空复杂度独立于已探索区域的面积。 |
| **多分辨率膨胀** | 规划器可以使用较粗糙的栅格检查碰撞（1个单元格 = 机器人半径），而感知层保持高精细。对 A*/RRT*/MPC 等算法有巨大的加速效果。 |
| **增量式膨胀更新** | 本次 tick 中只有精细网格状态发生改变的单元格，才会触发膨胀更新。不需要每帧都进行全局重膨胀。 |
| **自带梯度的滑动 ESDF** | 轨迹优化器（如 EGO-Planner、基于 IPOPT 的 MPC）可以直接通过该地图获取梯度 $\nabla$。 |
| **O(1) 复杂度获取前沿节点** | 探索规划算法直接迭代 `isFrontier(p)` 即可。 |
| **原生的未知空间处理** | 包含显式的 `UNKNOWN` / `FRONTIER` 枚举值。支持可选的“对未知区域也进行膨胀”，以保障保守安全规划。 |
| **虚拟天花板/地面** | 将任意高于设定平面（天花板）或低于设定平面（地面）的区域强制设为 OCCUPIED。对于类似“哨兵”这样近似二维移动的机器人非常实用，有助于将感知约束在贴地障碍上。 |
| **PCD 预加载** | 支持从提前保存的点云地图启动窗口构建（和我们目前使用的 `innowing.pcd` 完美契合）。 |
| **与 ROS 松耦合** | 你可以听任它其订阅接收点云底+里程计话题，也可以选择每帧手动调用 `updateMap(cloud, pose)` 来脱离ROS订阅。 |

### A.4 ROG-Map *不*具备的功能

- 它**不是 SLAM 系统**。位姿必须由其他系统（FAST-LIO, `hdl_localization`, Point-LIO 等）提供。
- 它**不**监听 `/tf`。输入的点云需假设为“已经变换到里程计坐标系”下的格式（即典型的 LIO 管道输出的 "`/cloud_registered`"）。
- 它**不**提供全局的持久化地图。窗口跟着机器人走；脱离窗口离开视野的栅格将会被遗忘丢弃。如果需要持久化，可以定期保存或维护单独的全局图。
- 它**不**附带生产级规划器。源码 `examples/` 中的 A*/RRT* 仅供参考，不适合工业运用。
- 它**不**处理原始的 Livox `CustomMsg`，仅接收标准格式 `PointCloud2` (`pcl::PointXYZINormal`)。

### A.5 可视化展示

ProbMap（黄色，精细） + InfMap（灰色，多分辨率膨胀粗化）：

![multi-res](../ROG-Map/misc/image-20240830181904520.png)

在 5 m 感知范围内的前沿边界 (红色)：

![frontier](../ROG-Map/misc/image-20240830183455023-17250143906771.png)

滑动的 ESDF 切片：

![esdf](../ROG-Map/misc/image-20240830183740886.png)

---

## B部分 — 集成开发者参考手册

### B.1 公共 C++ API (摘要)

头文件: [ROG-Map/rog_map/include/rog_map/rog_map.h](../ROG-Map/rog_map/include/rog_map/rog_map.h)

```cpp
#include "rog_map/rog_map.h"

namespace rog_map {
  typedef std::pair<Vec3f, Eigen::Quaterniond> Pose;
  typedef pcl::PointCloud<pcl::PointXYZINormal> PointCloud;

  class ROGMap : public ProbMap {
  public:
    typedef std::shared_ptr<ROGMap> Ptr;
    explicit ROGMap(const ros::NodeHandle& nh);

    // 馈入一帧激光雷达数据 (当 ros_callback.enable: false 时使用)
    void updateMap(const PointCloud& cloud, const Pose& pose);

    // 无碰撞连线检测 (支持高精或者膨胀地图验证)
    bool isLineFree(const Vec3f& start, const Vec3f& end,
                    double max_dis = 1e9) const;
    bool isLineFree(const Vec3f& start, const Vec3f& end,
                    bool use_inf_map, bool use_unk_as_occ) const;

    RobotState getRobotState() const;
  };
}
```

继承自 `ProbMap` ([prob_map.h](../ROG-Map/rog_map/include/rog_map/prob_map.h)):

```cpp
// 逐点位状态检测 (精细网格)
bool isOccupied      (const Vec3f& p) const;
bool isUnknown       (const Vec3f& p) const;
bool isKnownFree     (const Vec3f& p) const;
// 膨胀网格检测 (计算开销低；推荐规划模块使用)
bool isOccupiedInflate(const Vec3f& p) const;
bool isUnknownInflate (const Vec3f& p) const;
bool isKnownFreeInflate(const Vec3f& p) const;
// 前沿节点判断
bool isFrontier(const Vec3f& p) const;
// 枚举类别分类
GridType getGridType    (const Vec3f& p) const;   // UNKNOWN / OCCUPIED / KNOWN_FREE / FRONTIER / OUT_OF_MAP
GridType getInfGridType (const Vec3f& p) const;
// 容积区间批量查询搜索
void boxSearch        (const Vec3f& bmin, const Vec3f& bmax, GridType gt, vec_E<Vec3f>& out) const;
void boxSearchInflate (const Vec3f& bmin, const Vec3f& bmax, GridType gt, vec_E<Vec3f>& out) const;
// 元数据提取
Vec3f getLocalMapOrigin() const;
Vec3f getLocalMapSize()   const;
double getResolution()    const;
```

ESDF接口（通过 `esdf_map_`，目前受 `protected` 保护修饰 — 如果你需要调用，则需使用子类或通过 friend 声明暴露）：

```cpp
double getDistance(const Vec3f& pos) const;
void   evaluateEDT        (const Eigen::Vector3d& p, double& dist);
void   evaluateFirstGrad  (const Eigen::Vector3d& p, Eigen::Vector3d& grad);
void   evaluateSecondGrad (const Eigen::Vector3d& p, Eigen::Vector3d& grad);
```

`GridType` 枚举定义 ([utils/common_lib.hpp](../ROG-Map/rog_map/include/utils/common_lib.hpp)):

```cpp
enum GridType { UNDEFINED=0, UNKNOWN, OUT_OF_MAP, OCCUPIED, KNOWN_FREE, FRONTIER };
```

### B.2 ROS 接口定义

**ROG-Map 是零 TF 依赖的！** 位姿来源于里程计话题，点云必须已经在世界坐标系下。

**订阅的话题** (仅需要开启 `ros_callback.enable: true` 时生效)：

| 默认话题名 | 消息类型 | 作用 |
|---|---|---|
| `/cloud_registered` | `sensor_msgs/PointCloud2` (PCL 格式：`XYZINormal`) | 已经在世界坐标系转换好的受配准后点云 |
| `/lidar_slam/odom`  | `nav_msgs/Odometry` | 在同一坐标系下的机器人的里程计预估位姿 |

**发布的话题** (在节点的 Private namespace 下):

| 话题名 | 消息类型 | 控制开关 |
|---|---|---|
| `rog_map/occ` | `PointCloud2` | visualization.enable |
| `rog_map/unk` | `PointCloud2` | pub_unknown_map_en |
| `rog_map/inf_occ` | `PointCloud2` | visualization |
| `rog_map/inf_unk` | `PointCloud2` | unk_inflation_en & viz |
| `rog_map/frontier` | `PointCloud2` | frontier_extraction_en |
| `rog_map/esdf`, `esdf/neg`, `esdf/occ` | `PointCloud2` | esdf.enable |
| `rog_map/map_bound` | `MarkerArray` | 永远开启 |

### B.3 全面配置参阅指南

所有的参数都需要放在节点私有命名空间（即 `rog_map/…`）下。

#### 几何 / 滑动相关设置

| 键名 | 默认值 | 含义 |
|---|---|---|
| `rog_map/resolution` | 0.1 m | 精细（核心地图 ProbMap）体素尺寸 |
| `rog_map/inflation_resolution` | 0.1 m | 粗略（膨胀图 InfMap）的体素尺寸；**必须 $\ge$ resolution** |
| `rog_map/inflation_step` | 1 | 膨胀半径（在 InfMap 单元格步数的刻度下），球形域 |
| `rog_map/unk_inflation_en` / `unk_inflation_step` | false / 1 | 是否同样对此处设为未知的网格区域膨胀 |
| `rog_map/map_size` | [10,10,0] m | 局部滑动窗口的物理范围（以坐标原点对称向四周延伸计算） |
| `rog_map/map_sliding/enable` | true | 开启机器中心的窗口滑动机制 |
| `rog_map/map_sliding/threshold` | -1 (实时跟踪) m | 控制在发生最小多少距离平移后再响应重滑动 |
| `rog_map/virtual_ceil_height` / `virtual_ground_height` | -0.1 | 强制将这层厚壁外的区域当作为 OCCUPIED 墙壁处理。设置 `9999` / `-9999` 代表禁用。 |

#### 输入机制相关设置

| 键名 | 默认值 | 含义 |
|---|---|---|
| `rog_map/ros_callback/enable` | false | 开启后，ROG-Map 自助订阅云和 Odom |
| `rog_map/ros_callback/cloud_topic` | `/cloud_registered` | 配准后的 PointCloud2 数据源节点话题 |
| `rog_map/ros_callback/odom_topic` | `/lidar_slam/odom` | 里程计估计位姿话题 |
| `rog_map/ros_callback/odom_timeout` | 0.05 s | Odom 超时判断阈值 |
| `rog_map/point_filt_num` | 2 | 降采样降频因子 |
| `rog_map/intensity_thresh` | -1 | 若无反射强度过滤，丢弃低于指定阈值反射率的数据点 |
| `rog_map/load_pcd_en` / `rog_map/pcd_name` | false / "map.pcd" | 初始化时从外部一次性导入预渲染的 PCD 点云建图 |

#### 光线投射 / 概率更新更新参数

| 键名 | 默认值 | 含义 |
|---|---|---|
| `rog_map/raycasting/enable` | true | 万一关了这一条，地图中所有检测到的撞击只会是占据物；非占据全都会变为未知（无掏空效果） |
| `rog_map/raycasting/batch_update_size` | 1 | 累计投射扫描多少个局部帧后运行 |
| `rog_map/raycasting/local_update_box` | [999,999,999] | 以机器人所在点位原点的探测光线范围界限 |
| `rog_map/raycasting/ray_range` | [0.3, 10] | 有效扫描线范围约束 [min,max] (单位m) |
| `rog_map/raycasting/p_hit / p_miss / p_min / p_max / p_occ / p_free` | 0.70/0.70/0.12/0.97/0.80/0.30 | 贝叶斯似然函数的概率与对数几率的设定值区间 |
| `rog_map/raycasting/unk_thresh` | 0.70 | 多少粗网格里的细网格子胞元如果占比未达未知比例将被直接断定 |

#### 前沿 / ESDF 参数功能

| 键名 | 默认值 | 含义 |
|---|---|---|
| `rog_map/frontier_extraction_en` | false | 启用增量性前沿边界边缘提取功能 |
| `rog_map/esdf/enable` | false | 计算滑动的实时 ESDF 场 |
| `rog_map/esdf/resolution` | 0.2 | 设置 ESDF 计算场的体素网格大小 |
| `rog_map/esdf/local_update_box` | — | 限定机器人周边的 ESDF 的刷新距离框 |

#### 可视化支持参数配置

| 键名 | 默认值 | 含义 |
|---|---|---|
| `rog_map/visualization/enable` | false | RViz 等可视主开关 |
| `rog_map/visualization/time_rate` | 0 Hz | 主动周期刷新发布的频率 |
| `rog_map/visualization/range` | [0,0,0] | 可视范围显示框界定 (单位 m) |
| `rog_map/visualization/frame_id` | `world` | 头部发布数据参考坐标系标识 — **它需要和odom/cloud对应** |
| `rog_map/visualization/pub_unknown_map_en` | false | 是否要一并把未知空白的栅格也发送上去 |

### B.4 实用的 YAML 配置文件方案（启动模板库）

| 文件名 | 何时试用… |
|---|---|
| [marsim_example.yaml](../ROG-Map/examples/rog_map_example/config/marsim_example.yaml) | 全栈火力全开：使用 投射线（raycast） + 前沿发现（frontier） + 欧氏场 （ESDF） + 允许图传（viz） |
| [astar_example.yaml](../ROG-Map/examples/rog_map_example/config/astar_example.yaml) | 直接用现成的 PCD；用于运行在自带 InfMap 膨胀地图体系里的 A* 全局路径搜寻 |
| [rrt_example.yaml](../ROG-Map/examples/rog_map_example/config/rrt_example.yaml) | 同样采用现成点云格式（PCD）；用于做采样性质寻路：RRT* |
| [no_raycast.yaml](../ROG-Map/examples/rog_map_example/config/no_raycast.yaml) | 单纯从磁盘读死 PCD 图即可; 不在图上掏空/切分可行区域空间 |
| [pure_ogm.yaml](../ROG-Map/examples/rog_map_example/config/pure_ogm.yaml) | 裸的单纯跑一遍概率地形地图(OGM)，什么拓展场 (Frontier / ESDF ) 前沿之类的都不需要算 |

### B.5 超袖珍纯净代码引用样例

从 [examples/rog_map_example/Apps/marsim_example_node.cpp](../ROG-Map/examples/rog_map_example/Apps/marsim_example_node.cpp) 改编而得:

```cpp
#include "rog_map/rog_map.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "rog_map_node");
  ros::NodeHandle nh("~");
  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

  auto map = std::make_shared<rog_map::ROGMap>(nh);   // 读取 ~rog_map/* 这些节点的配置内容

  // 选项 A — 话题订阅驱动模式: 把参数设为 rog_map/ros_callback/enable: true, 随后 spin 挂起监听即可.
  // 选项 B — 人肉填入代码模式:
  //   rog_map::Pose pose{{x,y,z}, q};
  //   rog_map::PointCloud cloud;  // 请确保这里的数据已经位列于在世界（World）系了！
  //   map->updateMap(cloud, pose);

  // 查表举证实例
  Eigen::Vector3d q(1.2, 0.3, 1.0);
  if (map->isOccupiedInflate(q)) { /* 在规划上需要绕路避障的情景响应 */ }

  ros::AsyncSpinner s(0); s.start();
  ros::waitForShutdown();
}
```

### B.6 依赖项和安装构建指令

```bash
# 执行 apt 的包装任务
sudo apt-get install ros-noetic-rosfmt libglfw3-dev libglew-dev \
                     libeigen3-dev libdw-dev
sudo ln -s /usr/include/eigen3/Eigen /usr/include/Eigen   # 只需要运行这一次就好

# 进行构建 (可以位于你自选建立好的任意一 catkin ws 空间之中)
cd <ws>/src
ln -s /path/to/哨兵/ROG-Map/rog_map .                  # 你也可以选择复制而不是建立硬/软短链链接
cd .. && catkin_make -DBUILD_TYPE=Release
```

### B.7 容易踩坑的部分

- **`libdw-dev`** 这是一个底层的链接依赖项目（针对 backward-cpp 的异常堆栈打印调用）。如果是缺失状态这会导致抛出大量连接报文错误。
- **Conda冲突环境配置**: 在 `catkin_make` 编译环境之前务必要执行 `conda deactivate` 命令；或者如果是遇到异常直接一不做二不休把编译输出里的 `build/` 和 `devel/` 文件夹全都铲干净再试一次。
- **如遇 `VizConfig` 未正常生成** → 可以考虑采用这方法 `catkin_make -DCATKIN_DEVEL_PREFIX:PATH=${YOUR_WS}/devel`。
- **关于 `ORIGIN_AT_CORNER`（边角坐标主辅）还是 `ORIGIN_AT_CENTER`（坐标中心为主辅参数）** 这是一个涉及到编译时选项的参数项（具体参照 [rog_map/CMakeLists.txt](../ROG-Map/rog_map/CMakeLists.txt)）。你需要将它保持为和你的系统上游接口设定完全一致的数据模式设定原则方案，切记！。
- **关于要求 `inflation_resolution >= resolution`** 这作为开机初始化阶段要求的绝对不可违项。如果没有满足直接导致异常Abort挂起退出运行状态。
- **关于虚拟地板厚度概念设定 `virtual_ceil_height` / `virtual_ground_height`** 如果该区间范围限定外的坐标轴内，所有一切都将被暴力强行塞成占据状态 OCCUPIED - 故而想维持完全 3-D 三维自由场效果的情况只需要配定设置区间值为 ±9999 就行了。
- **坐标系同一与兼容性问题**: `visualization/frame_id` 务必请确保 odom （里程计参考系） + cloud （数据云点坐标源坐标指向）二位数据头所在的坐标保持百分之百的一致同源化！
- **ROG-Map本身代码没有包含 `/tf` 使用读取模块**；因此输入的云数据必须之前事先就通过位姿转换转换好了才行；（比如通过 LIO 内嵌产生的 `/cloud_registered` 类型，或者在它进入之前做一堆专门跑这种脏活累活的转换接口程序）！
- 千万不要在你运行着开启配置了参数 `ros_callback` 后台服务的时候还犯傻同时向里乱手动调用传 `updateMap()` 代码数据。
- 开启配置参数项 `raycasting.enable: false` 后如果不用 PCD 初始化（`load_pcd_en: true`）将会产生灾难一样的数据结果！你只有这两种使用选择：要么开启要么直接挂静态PCD——所有那些没被主动探测感知区域扫描碰到的空旷都将会沦丧定格变成纯一片 UNKNOWN （毫无使用探索及ESDF意义和用处！）; 解决这两种二致状态情况最好用办法就是和预加载 PCD 参数（`load_pcd_en: true` 配合）作为打底的混合方案并存。

---

## C部分 — 在“哨兵”系统中集成 ROG-Map

目前记录总结在这篇指南 [Pipeline_Report.md](Pipeline_Report.md) 中的哨兵体系下目前的相关切面现状大抵为如下所示：

```
livox_ros_driver2  ──► /livox/lidar  ──► hdl_localization  ──► TF map→aft_mapped + /aligned_points
                                                                │
                       map_server (innowing.yaml, 2-D)  ──► dstarlite ──► /cmd_vel
```

当前这套规划器 (`dstarlite`) **仅支持 2-D 地形运算**，其由静态地图构建供给出的占据栅格 `OccupancyGrid` 来提供场景视野—— 这表明实际上它完全没有能在运行的时候应对具备空间 3-D 本征特性的任何动态/真实障碍感知规避能力！(如：落体碎片挡板材料，倾斜爬升的坡道、高空抛落流体飞射投弹体或者走位移动之中的对家战车对手等等）。所以目前在此引入与实施做整合化集成的最大初心使命主轴就是在——维持保持静态建图、实时预估定位以及核心控制循环基本层不变的基础面提要之上去进一步额外为其并联加上一层**实时热更新叠加的 3-D 被动障碍感知响应网**。

### C.1 推荐选用的三种实施路径建议方案

| 实施方向模式 | 选用其你可以收获的体验与效果反馈 | 大致开发量消耗 |
|---|---|---|
| **1. 获取可视化侦查监听支持 （Observability only）** | RViz 提供可化面板界面调式 + 获取能够完整覆盖记录 ROG-Map 生成的占用地图区域, ESDF , frontier等所有内部参报机制的数据rosbag 包——没有任何主动进行执行逻辑修正和操控反应能力改变影响！ | 约 30 分钟。 |
| **2. 动态代价引入注能降维打击 (Dynamic costmap injection)** | 将 ROG-Map 计算膨化过后的动态高危障碍占用地图重新进行平面映射回二维并在主题 `/grid` 当中按 2-D 数据发送——它刚好就可以借且此数据总线与 `dstarlite` 并线通讯——从而引导指挥其躲避原有的三维路障盲区陷阱！ | 半个工作日左右。 |
| **3. 使用原版基于附带有 3-D 感知的 ROG-Map 取代 D*-Lite** | 通过 `InfMap` 上的原生 A*/RRT* 进行规避绕障求解生成线路，或者引入 ESDF 用在其本身的优化参数轨迹约束生成中。 | 需要至少大几天及更久的长线工程搭建测试。 |

对于初次探索融合来讲，强烈建议方案路径优先实施配置模式 2 作为初始切入点：它属于效益获益成本比回报最充沛也是对现有架构代码触痛最低下手的上选决断思路路径。

---

### C.2 模式 1 — 获取纯数据追踪监控感知权支持 (即插即用的只读非侵入接轨方式)

**主导思路:** 完全仅以用来用作监督监控观测和科研理论比对搜刮样本用地的立场和现有堆栈框架下同级伴生运行跑跑这一套 ROG-Map。

**步骤 1. 取道加载 `rog_map` 作为本地功能包子项目。**
把此项目或者其下文件目录的符号系统链接直接置入下放归档给 [ROG-Map/rog_map/](../ROG-Map/rog_map/) 到这级下面去 `Old_Nav/sim_nav/src/`:
```bash
cd Old_Nav/sim_nav/src
ln -s ../../../ROG-Map/rog_map .
```
最后做一次编译清理和重新生成大整合: 在上级工作区重编译（`cd ../.. && catkin_make -DBUILD_TYPE=Release` ) ——请再次牢记一定要先保证把你那 `conda deactivate` 执行了关掉虚拟化！)。

**步骤 2. 保证输送上对齐好了过后的配准云。**
系统原有的组件 `hdl_localization` 会以 `/aligned_points` - 此频段流公开广播出这部分的数据：刚好将激光传感器原云转化重合放准至 `map` 原生大参照世界地图框架之内的位准位置点去。这是刚好投其所好ROG-Map胃口的极好馈入品数据食材配置！！

**步骤 3. 建置一个新的配传环境配置文档 YAML。** 将其拟名为: `sim_nav/src/bot_sim/config/rog_map_passive.yaml`:

```yaml
rog_map:
  resolution: 0.1
  inflation_resolution: 0.2
  inflation_step: 2                  # ~大致等于机器人的尺寸截面积半径距离幅度
  map_size: [30.0, 30.0, 3.0]
  map_sliding:
    enable: true
    threshold: 0.5
  virtual_ceil_height: 2.0           # Sentry 型高不足2m; 因此设定了这条削切高顶压线平顶防伪误报上限阈值
  virtual_ground_height: 0.05        # 离地底缘空过大概这五厘米，防雷达打飞地板引起地面雪花反射波高频被拦截扰乱处理效率
  load_pcd_en: false
  point_filt_num: 2
  intensity_thresh: -1
  ros_callback:
    enable: true
    cloud_topic: /aligned_points     # <-- 指接借力来自 hdl_localization 内置产线的流段接口频道端口数据
    odom_topic: /odom                # <-- 指接源源自 hdl_localization 的反馈口总线数据
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
    frame_id: map                    # 同样参照维持住等同和对等于 hdl_localization 发出的那个对应的 TF 母串流世界标的名称！
    pub_unknown_map_en: false
```

**步骤 4. 书写编写编写一份简单超迷你代码包装桥接小驱动模块。**
建立一个这个文件：`sim_nav/src/bot_sim/src/rog_map_node.cpp`，把刚才之前列出来过的 B.5 小节里的那十几行极简的驱动示例模子 `main()` 完整原样的照抄塞入进这个代码体当中就行。紧接着在此项工作区的 `CMakeLists.txt` 构建引导描述脚本文里面把它补充声明好注册进去：

```cmake
find_package(catkin REQUIRED COMPONENTS roscpp rog_map pcl_ros)
add_executable(rog_map_node src/rog_map_node.cpp)
target_link_libraries(rog_map_node ${catkin_LIBRARIES})
```

**步骤 5. 将它并行挂靠拉起到你的 `3DNavUL_Test.launch` 总控环境。** 在你的 `sim_nav/src/bot_sim/launch_real/rog_map.launch` 里边挂上一段:

```xml
<launch>
  <node name="rog_map_node" pkg="bot_sim" type="rog_map_node" output="log">
    <rosparam command="load" file="$(find bot_sim)/config/rog_map_passive.yaml"/>
  </node>
</launch>
```

而后在其母包启动引用端 [3DNavUL_Test.launch](../Old_Nav/3DNavUL_Test.launch) 中加入 `<include file="$(find bot_sim)/launch_real/rog_map.launch"/>` 这行包含挂接申明项。

**最终调试和确认执行运作检验功能。** 后台启执行后跑一次看一看，期间拉个截听探针跑一下指令 `rostopic echo /rog_map_node/rog_map/inf_occ` 如果可以看到正处显示了数据更新且说明它发现了截取点那表示目前在发占据栅格。把 RViz 给点起加载起，选回挂在 `map` 位点的话就能看着跑马灯一样的跑满显示带有大色块粗大灰色包边内里高解析度鲜明黄色核心构成的渲染色分层的精细+膨胀双阶结合地图显像画面呈现给你！。

---

### C.3 模式 2 — 将三维计算的高质量膨胀规避热力网压平注射介入给二位路径老功臣规划器 D*-Lite 获取融合避让躲避特性能力！

**工作思路目标大纲:** 现行服役期的老框架规划系统组件 `dstarlite` 其实一直自带着一套专门订阅聆听着外部传进来并将其称作动态网格（命名规则代号变量就叫做 `dynamic_map_topic_name` ）— 预设下缺省的管道接受流就是取命名指称 `/grid` 这个名称； 所以基于这些条件，我们目前迫切所需的最想要搞的手段玩法与思路规划方向也就是十分简单自然的水到渠成：想办法提取将 ROG-Map 那套针对最新**三维状态所察看到的现实存在着的三维物理碰撞点投影成降为到 2-D 二维切面的 占据网络数据流 (`OccupancyGrid`)** ！

**我们需要开坑构架的新建中转通讯服务类：`rog_map_to_grid`** — 实际上也就是个用来简单倒腾二传倒手中间站转发处理器程序。它只做了这几件活：
1. 朝着 ROG-Map 去执行调取并提取机器当前位置向周边划定了一定二位方圆之内距离下圈出来的一切在案有据在库留存有档案的，同时还是经由自身体碰撞域边界经过膨化安全规避值距离加工渲染以后的那个粗位图层级的信息框内数据——（调取采用的那个 `boxSearchInflate(bmin, bmax, OCCUPIED, cells)` 函数执行此查找操作）；
2. 然后接下来就是只要过滤挑选下在这个高层内存在处于位于 `z ∈ [floor_clip（地线低位）, ceiling_clip（天线帽顶高阶）]` 这个安全合理有效区间的高度区域之中发现存在的这批占区对象数据组块把他们给全部“压平（降切维映射）” 抛掷按倒平放回到同位于原点在投影下的二平面维度中去 (`(x,y)`)；
3. 将倒弄完以后的新产出打包变成 `nav_msgs/OccupancyGrid` 数据封包形态投到通讯管网发布口向频道 `/grid` 里去，那些压出了被霸占的位置的值设置为 100 去代表实打实存在物体不能过去，什么毛线玩意都没有干净可以随笔放心自由通过的地区就空出来发它个 0，而此片被下发传递地图的主轴位置零基点需要设定校准和实际的主体同步重合去锚定在原系之上保证不要乱发生偏移位错即可。

这里有一份简单的基础雏形示范板型模子：

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

配合原工程配置文件之中默认原装早就下好了的定义设置段： `dynamic_map_topic_name: /grid` (这预先老版本内建就直接已经帮你放设置安插排进了这个位于[dstarlite.launch](../Old_Nav/sim_nav/src/bot_sim/launch_real/dstarlite.launch)文件设定好了)，这一连贯的全套动作和代码衔接就形成了如臂使指**完美契合天生一对浑然天成纯即插即用（插拔式热启对接）级的兼容整合体验**：它只要看到传个影子过来告诉规划脑哪里 ROG-Map 被列入了占领占据阻碍区（还是它替你已经顺便把预留机动偏移和膨化躲避规矩的提前计算后结果的量！），接下来这头这个老的只依靠地图跑瞎画盲操作寻迹规划模块也能聪明巧妙自如如虎添翼地做到精准完美无伤躲避开现眼前面新跳出生成的各种那些以前被无视直接盲吃和抓瞎的这些新三维动态热更盲区位里的突然入侵异物与碍事的各种各样奇形怪异非死位障碍障碍挡道体们！！

**调整建议项指南：一些在这个过程里面对于针对“哨兵”机型最最为重中之重核心关键调节控制门锁值项旋钮**

- `virtual_ceil_height: 1.8` — 完全的贴靠对应去和真实本尊物理高度来个门当户对的比对标齐对准校正位；以此帮助他完美排除过略掉诸如头顶之上的悬挂屋架上生下的漫天横飘那些飞灰扰乱和半空中那些不需要考虑完全悬的超高无关宏旨建筑结构管网等。
- `virtual_ground_height: 0.05` — 这个直接对付雷达容易一打下去探得太近就反扫被地板自己的表面纹路搞迷糊产生低水平线上那种低级雪花地面假象波纹误判定影响。
- `inflation_step` — 这个值一般需要拿：大概这台你的主体战车尺寸安全圈估设的半径距离值 /除以你的设定粗位图那参数 `inflation_resolution` 来给近似换标比敲打推算出来。针对目前战车大概大约为 ~0.4 m 外径加上预备定死死值 0.2 m 网距 → 就是刚好能算出得个设置结果：`inflation_step: 2` 。
- `raycasting/local_update_box: [25,25,2.0]` — 这视界大小参数需要拉给足他！让他的视觉可以足够抢先在那古板的寻找预判模块还没有打算盘把主意去考虑打算经过那里和在那之前——先看清探清在那么前方的深水区老远距离里有没潜伏杀机对手在那里了没！
- `raycasting/batch_update_size: 1` 跑出维持 10 赫兹 的扫描工作雷达帧频——以帮助把你的那些反应链和网络反应推迟减慢现象严防被堵滞挤住在 <=100 ms毫秒 的区间范畴。
- `map_sliding/threshold: 0.5` m — 表示至少战车移动在有了足足至少达达到了的：经过物理真机实战大跑大动走挪半米远，它才肯放行和发动调用系统给全部重新更新翻洗并重算洗牌刷新这些周边的数据阵位和重校对排布矩阵数据排列次序和结构关系映射位置等——为了彻底防止当本身没有在移动没有前行的非开行停位发呆休整待命滞留工况下，由于些微抖动就不停一直循环徒劳死命狂叫唤反复重启发力索引地图结构耗费运算。

**关于同时切忌必须保留现有的全套老手原本那一本基底参照大底稿件原版静图。**  `dstarlite` 此设计内部是有功能在把来自 `/map`频道 (固定底盘本图，出自于 `map_server`系统服务) 和  `/grid` (新引入用来传输给其发送时即新发生的动态地形变数据) 这些信息来实施双剑合并混合拼接共治同调运行融合执行机制规划和运作和操作决断的! 所以需要分清一点：ROG-Map 在这出这台双簧的联合协同双主辅联动这台好戏里边，其实仅且只管负责这下面那个新补充给这只新近长出来的可以观看时下周围动向环境改变信息的监控第三只眼；而之前的从 `innowing.pgm`  等预生成的大老基图文件档上吃来的读取那份最初构建生成的赛场的场地物理外壳基础固化大底板及永远都不会被谁修改改变结构地墙壁建筑等等依然仍然还须指望由原有它去兜底提供！

---

### C.4 模式 3 — 基于 ROG-Map 的 `InfMap` 特性体系进行直接纯原生的全 3-D 三维本场立体寻径自主自由行系统寻规划与执行开发工程！

如果且当真项目工程攻坚主团队最后决定了拍死定板去尝试想进行更狂放更大开大合也更加具有远建性的换骨剥肉大型整建开颅更换重设：彻底决定废弃并抛下过去 D*-Lite 然后全方位把规划与路径全转包重写！

- MaRS 实验室提供的 A* 例子 [examples/rog_map_example/Apps/astar_example_node.cpp](../ROG-Map/examples/rog_map_example/Apps/)，使用 `map->isOccupiedInflate(p)` 作为搜索节点的扩展依据——可以直接拷贝作为一个完整的 3-D 体素 A* 算法的原生级替换验证程序来进行应用。
- 另有一个 RRT* 版本的官方用例程序，向我们演示与教授如何实现在被膨化出来的无障碍闲余净空中进行高效的信息知情采样技术演示（通过 `isKnownFreeInflate` 判断）。
- ESDF能够用于直接赋予系统让它开始有本钱拥有更顺滑和拥有被轨迹算法去执行最优化去噪抚顺顺直能力处理方案的底牌: 其可以通过任何如 EGO-Planner 同系同派相同算法路线同门师兄弟一样架构下的基于梯度的参数曲线优化程序经过对提供抛出的公用原生级源参数：通过 `evaluateFirstGrad`接口来汲取调用源自核心底层的拉出其引出梯度值并施加给曲线去发生改变形变以迎合约束！

为求提供暴露使用出其内秉含带被保护包裹封装掉下的隐藏深埋于体量下头深处的 ESDF 特性支持，需采用衍生新拓展外展增加补充上一个新子集封装继承类别派生外漏函数：

```cpp
class SentryMap : public rog_map::ROGMap {
public:
  using rog_map::ROGMap::ROGMap;
  double dist(const Eigen::Vector3d& p) const { return esdf_map_.getDistance(p); }
};
```
(注记：或者或者如果你不嫌这种写代码方式也行，用往其你自装裹封装的封装器类域内部添加一行使用 `friend` 语法关键字修饰加持声明这种更加简单粗暴些手段也能够变通打通经脉解决达成该目的！)

---

### C.5 快速用于应用在咱们自家的“哨兵”机群部署主题、参数表与参照系绑定大抄本速查大全 (cheat-sheet):

| ROG-Map 的此调参值 | 请把它老实果断的改为： | 原因所在： |
|---|---|---|
| `ros_callback/cloud_topic` | `/aligned_points` | hdl_localization 内发布传输出的那个数据串点云流已经是被它在 `map` 这个大座标体系下面提前转换安插对准好的熟货色了 |
| `ros_callback/odom_topic` | `/odom` | hdl_localization 那头跑完自己计算后所产出来那个里程行推姿位数据报告信息 (它属于 `map` 直接通过被重映射定位和参照标地转到为 → `aft_mapped`这一对位对等数据中出炉产地)|
| `visualization/frame_id` | `map` | 强制保证要求它和我们发过来的那批点云及姿位大本源坐标的身份统一共处在同一个绝对统摄和等值系之内同一水平下 |
| `virtual_ceil_height` | 1.8 | 这个差不多就当成咱们当下的“哨兵”这几代大致普遍头顶标准高参数上限了。 |
| `virtual_ground_height` | 0.05 | 地切底盘防尘底隔限（下探止点） |
| `map_size` | [30,30,2.5] | 把视野预留下足够撑广一点要能够稳扎稳打且舒适地容纳包裹囊括完全罩得住超脱超出平时前瞻巡标所需的最基本底限范畴圈套半径大小 |
| `resolution` / `inflation_resolution` | 0.1 / 0.2 | 这个值就是拿去和在我们在目前老地图旧账本里面的 `innowing.yaml` 这上边定义标注的它本身的二维原始固定格像素网细密精度对轨吻合和齐步共振校等保持对应用的！ (数值普遍通常约为：0.05到0.1 m 之间范围打转游荡) |
| `inflation_step` | 2 | ≈ 我们机器人圆柱形假想估量身段粗细（也就是大约两两成对乘过去算出宽：$2 \times 0.2 \text{ m} = 0.4 \text{ m}$） |
| 原供需要发送对接导向输出给负责接手这接力盘子的下面那台下位规引计算仪表的频道管路网端数据管名是啥？ | 指定发往叫 `/grid` (并务必在此加接装一个转接过滤小中介漏斗器来进行搭桥联通去！) | 前边说过了 `dstarlite` 它自个预设好并在那边等这个包裹时要收取收件挂号信签到名字：叫这个 `dynamic_map_topic_name`  （而且早就内置设定写好好了就是直接缺省成这个串符代名词指代叫 /grid ）|

### C.6 落地整合实线投产前的终极排查总备忘事项自省清点表！

- [ ] 操作检查有没正确使用或者放上了拷贝好的软连接了这套 `rog_map`项目进入放置在咱们的开发地这这个 `Old_Nav/sim_nav/src/` 工作主线底下？
- [ ] 确保绝对切断脱开了 `conda deactivate`, 确定已经提前上机按指令按部就班实打实装好了装过 `libdw-dev` + 同时补齐修正了对 `Eigen` 的那个系统路径上必须重搞补充上的重定向跳转库链接库依赖符号补齐,且再重新来一次的无错重打通大编译并能够通过不报系统崩溃未定义连接函数缺漏项告警？
- [ ] 能检查得到并确实有在这个路径下建立和存好了一份专门给它的新的带其属性设置专属参数文档 `config/rog_map_passive.yaml` (参见前文所述段落章节内容指引的 Section C.2 部分去排错纠错)。
- [ ] 完成好独立写写写创建并安放组建装嵌出了一支用于跑他的带了主控运行驱动外包小引擎 `rog_map_node` 源件（前篇提及参照引索： Section B.5 ）。
- [ ] 确实验证无误与现原在服役运作的大主心骨轴线发车令程序文件： `3DNavUL_Test.launch`  相安无事同起协并行没有阻留报警崩掉; 能够在起底发足的同时同台用外部监控 RViz 上清晰捕捉查收捕捉到正在活脱运作的顺着移动跟着跑动态跟车游移式的滑动实时数据画面！
- [ ] (可选：针对上面选用实施的是中策之法采取引入采用注入干预的“模式第二条计划选项 2” ）查验是否确实编写加上添漏堵上放上了并配置无问题这中间用来过手和转发信息转换数据结构网的数据的滤口与代投口通道功能组件插件节点了：即那个对口挂接到跑向针对去向频道 `/grid` 管道的这个叫小名字叫做 `rog_map_to_grid` 此节点组件！
- [ ] 然后在跑场或者全功能系统拟真环境场地做全仿实操仿真中最后走一圈场实验一下做做确认查表核定在那个旧架构规划层 `dstarlite` 会否能对你人为在其路障行进轨迹中或视觉中凭空丢丢包造出来的实时视野中的那种不在这张老旧本图里的临时出现新动态遮挡障碍物和行运动中的盲区视差实体进行发生确实躲避、紧急切绕、规划切线躲让产生变更的新鲜重设修改计算重新生新定新路图出来的这一全新防范应激行路机制起效果！如果生效那表明完美攻关成事。
- [ ] 如果没搞到很好再回头仔细重修捏一捏那些和微调好它的这三主要灵魂参：`virtual_ceil/ground`（遮挡隔断范围）, 再或者微量补调整下重审它那个涉及距离判定的： `inflation_step` , 以及那些这决定看能够侦查监控预警感知得到多大老远框限范畴和多宽感知场视界的这东西参数项控制：`raycasting.local_update_box`！
- [ ] 永远随手千万不要把那些记录收集包的事抛之脑后忘记得一干二净啦——记住最后全套实验走完一定千万拜托记必须留下和挂取下它所有录好抓死这截出来的能够保存存盘封包在那个专属的后缀名带着跑了一场数据里面带了这个节点名： `/rog_map/*` 这条目下面的各种事件回放数据的大包包(`rosbag`)——留底传报回去之后等你们事了拂衣去静下来回放再重头开始后盘审判大会做秋后数据梳理深层纠错解构找漏洞补过提升与精进和技术报告分享。

---

## 附录区 — 关于提供如果要想真正挖掘探索了解透深挖精进其更多底子魔改自定配置内里底层接口深意逻辑及开发架构所必须强硬头皮阅读的全部内部源码与文献资料指引单！

- [rog_map/include/rog_map/rog_map.h](../ROG-Map/rog_map/include/rog_map/rog_map.h)
- [rog_map/include/rog_map/prob_map.h](../ROG-Map/rog_map/include/rog_map/prob_map.h)
- [rog_map/include/rog_map/inf_map.h](../ROG-Map/rog_map/include/rog_map/inf_map.h)
- [rog_map/include/rog_map/esdf_map.h](../ROG-Map/rog_map/include/rog_map/esdf_map.h)
- [rog_map/include/rog_map/free_cnt_map.h](../ROG-Map/rog_map/include/rog_map/free_cnt_map.h)
- [rog_map/include/rog_map/rog_map_core/sliding_map.h](../ROG-Map/rog_map/include/rog_map/rog_map_core/sliding_map.h)
- [rog_map/include/rog_map/rog_map_core/counter_map.h](../ROG-Map/rog_map/include/rog_map/rog_map_core/counter_map.h)
- [rog_map/include/rog_map/rog_map_core/config.hpp](../ROG-Map/rog_map/include/rog_map/rog_map_core/config.hpp)
- [rog_map/include/utils/common_lib.hpp](../ROG-Map/rog_map/include/utils/common_lib.hpp)
- 这些对应的源代码层具体实现在：在路径主干为 [rog_map/src/rog_map/](../ROG-Map/rog_map/src/rog_map/) 这个大目录体系结构管辖之下的各个诸如它叫做的: `rog_map.cpp`, 还有这个 `prob_map.cpp`, 继续比如叫 `inf_map.cpp`, 然后是其叫 `esdf_map.cpp`, 那里面名为 `sliding_map.cpp` 以及这个 `counter_map.cpp` 的所有底子源码实现体库全集之中！