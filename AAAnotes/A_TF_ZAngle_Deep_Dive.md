# TF树 · z_angle · 坡度补偿 深度解析

> 来源：2026-04-28 ~ 2026-05-07 对话记录  
> 涉及文件：`Old_nav/sim_nav/src/bot_sim/src/dstarlite.cpp`  
>           `Old_nav/sim_nav/src/bot_sim/src/ser2msg_decision_givepoint.cpp`  
>           `Old_nav/sim_nav/src/bot_sim/src/real_robot_transform.cpp`

---

## 一、当前 TF 树结构（实测 frames.gv）

```
map
├── virtual_frame          ← /ser2msg_decision_givepoint   25 Hz
└── aft_mapped             ← /velodyne_nodelet_manager      8.5 Hz
    └── gimbal_frame       ← /real_robot_transform          20 Hz
```

`rotbase_frame` 只在 `ser2msg` 内部作矩阵乘法中间量，**不广播到 TF 树**。

---

## 二、每个 Frame 的含义

### `map` — 全局规划坐标系
- 世界固定原点，xy 平面 = 地面（建图时初始化）
- 所有路径规划、栅格地图、导航目标点均在此坐标系下
- 树根节点，不被任何节点广播

### `aft_mapped` — LiDAR SLAM 定位输出
- Point-LIO 跑完一帧点云后输出的机器人**三维位姿**（含真实 pitch/roll/yaw）
- 广播者：`/velodyne_nodelet_manager`，8.5 Hz
- 名字来源：LOAM 系算法惯例"after mapping"缩写
- **重要**：包含机器人爬坡时的真实 pitch，是坡度信息的源头

### `gimbal_frame` — 云台/激光雷达安装坐标系
- Livox 激光雷达的物理安装坐标系
- 广播者：`/real_robot_transform`，20 Hz，挂在 `aft_mapped` 下
- **核心处理**：从 `aft_mapped` 的 RPY 中**抹掉 yaw，只保留 roll + pitch**，再叠安装偏移角

```cpp
// real_robot_transform.cpp 核心逻辑
m.getRPY(roll, pitch, yaw);
qhdl.setRPY(roll, pitch, 0);   // yaw 置零！
qhdl = qhdl.inverse();
// 广播: aft_mapped → gimbal_frame（含pitch/roll，无yaw）
```

- 用途：点云障碍物检测坐标变换（`gimbal_frame` → `map`）

### `rotbase_frame` — 中间过渡帧（仅内部用）
- 仅在 `ser2msg_decision_givepoint` 内部作矩阵乘法中间步，不广播
- 语义：云台偏航解耦后的底盘朝向参考

### `virtual_frame` — **导航命令参考系**（最关键）
- 专门给规划器用的虚拟机器人运动坐标系
- 位置 = 机器人底盘中心（由TF链推算），方向 = 经解耦处理后的朝向
- 广播者：`/ser2msg_decision_givepoint`，25 Hz，**直接挂在 `map` 下**

**计算方式：**
```cpp
// ser2msg_decision_givepoint.cpp
transformMapToGimbal = tfBuffer.lookupTransform("map", "gimbal_frame", ...);

// rotbase_frame 偏航 = -relative_angle（云台相对偏航，当前强制=0）
q2.setRPY(0, 0, -message.relative_angle);  // relative_angle=0

// virtual_frame 偏航 = -imu_angle（底盘IMU角，当前强制=0）
q1.setRPY(0, 0, -message.imu_angle);       // imu_angle=0

location = gimbalframe × rotbaseframe × virtualframe_offset;
// 发布: map → virtual_frame
```

**当前状态**：`relative_angle=0`，`imu_angle=0`，
所以 `virtual_frame` 实际上等于 `gimbal_frame` 在 `map` 下的完整映射，
即 **virtual_frame 继承了 gimbal_frame 的 pitch/roll**。

**设计原意**：云台转向时 `gimbal_frame` 的 yaw 变化，但 `virtual_frame` 保持与底盘一致，规划器不受云台转向影响。

---

## 三、z_angle 前世今生（完整数据流）

```
IMU硬件 (/mcu/chassis_imu → /mcu/yaw_angle)
    │  机器人爬坡时 pitch = θ
    ▼
Point-LIO → aft_mapped (含 pitch=θ)
    ▼
real_robot_transform → gimbal_frame (含 pitch=θ, yaw=0)
    ▼
ser2msg_decision_givepoint
    │  location = gimbalframe × rotbase × virtual_offset
    │  → 广播 map → virtual_frame (含 pitch=θ)
    ▼
dstarlite::publish()
    │  lookupTransform("virtual_frame", "map")
    │  = 变换矩阵 T，其旋转部分 R = Rz(yaw)·Ry(pitch=θ)·Rx(roll)
    │
    │  手动清零平移：translation = (0,0,0)
    │  → tf_transform = 纯旋转算子 R
    ▼
old_cmd_vel = (dx/dis, dy/dis, 0)   ← 2D路径方向，z硬编码=0（map坐标系）
    ▼
new_cmd_vel = tf_transform * old_cmd_vel = R · [x, y, 0]ᵀ
    │
    │  平地（θ=0）：R = Rz(yaw)，结果 z'=0
    │  爬坡（θ≠0）：Ry(θ) 作用于水平向量：
    │    [cosθ·x,  y,  -sinθ·x]ᵀ  →  z' = -sinθ·x ≠ 0
    │
    ▼
xy_lenth = √(new_cmd_vel.x² + new_cmd_vel.y²)   ← 水平投影长度
    ▼
z_angle = atan2(new_cmd_vel.z(), xy_lenth)
        = 向量与水平面夹角 ≈ 地面坡度角 θ
```

**结论**：`z_angle` 不是直接读 IMU，而是通过「把2D水平向量用含坡度的TF旋转矩阵转换后，提取溢出的 z 分量」间接得到的坡度角。

---

## 四、tf_transform 详解

```cpp
// dstarlite.cpp publish() 中
transformStamped = tfBuffer.lookupTransform(
    robot_frame_name,  // target = "virtual_frame"
    map_frame_name,    // source = "map"
    ros::Time(0)
);
transformStamped.transform.translation.{x,y,z} = 0;  // 清零平移
tf2::fromMsg(transformStamped.transform, tf_transform);
```

| 字段 | 类型 | 内容 |
|------|------|------|
| `m_basis` | 3×3矩阵 | 旋转部分 R（由四元数展开） |
| `m_origin` | Vector3 | 平移部分（被强制清零） |

清零后效果：`tf_transform * v` = $R_{map \to virtual} \cdot \vec{v}$

**数学展开（爬坡时）：**

$$R_y(\theta) \cdot \begin{bmatrix}x\\y\\0\end{bmatrix} = \begin{bmatrix}\cos\theta \cdot x \\ y \\ -\sin\theta \cdot x\end{bmatrix}$$

z 分量 = $-\sin\theta \cdot x$，这就是 `new_cmd_vel.z()` 的来源。

---

## 五、坡度补偿逻辑（含 Bug 分析）

```cpp
// dstarlite.cpp line 523-526
double z_angle = atan2(new_cmd_vel.z(), xy_lenth);

// 补偿因子 = 1 - z_angle / (15° in rad = 0.2618)
cmd_vel.linear.x = new_cmd_vel.x()/xy_lenth * new_velocity
                 * (std::min(1 - z_angle/0.2618, 1.7));   // 上坡减速，最大放大1.7倍
cmd_vel.linear.y = new_cmd_vel.y()/xy_lenth * new_velocity
                 * (std::max(1 - z_angle/0.2618, 0.7));   // 有下限 0.7 保护
```

| 坡度 | 补偿因子 | 效果 |
|------|---------|------|
| 0°  | 1.0     | 正常 |
| 7.5° | 0.5    | 减速一半 |
| 15° | 0.0     | 速度归零 |
| **23°** | **-0.53** | **`linear.x` 方向反转！机器人会往回冲** |

**Bug 根因**：`linear.x` 用 `std::min(..., 1.7)` 没有下限保护，坡度超过 15° 时补偿因子变负，速度反向。`linear.y` 因为有 `std::max(..., 0.7)` 保底所以不会反向，但两轴行为不一致。

**其他已知 Bug：**
1. `calculate_velocity()` 末尾 `return L1` 而非 `return velocity`，速度控制完全退化为常数
2. TF lookup 失败后没有 `return`，用 identity transform 静默继续

---

## 六、节点职责一览

| 节点 | 广播的TF | 数据来源 | 频率 |
|------|---------|---------|------|
| `velodyne_nodelet_manager` (Point-LIO) | `map → aft_mapped` | LiDAR点云 + IMU | 8.5 Hz |
| `real_robot_transform` | `aft_mapped → gimbal_frame` | TF查询 + 安装偏移参数 | 20 Hz |
| `ser2msg_decision_givepoint` | `map → virtual_frame` | TF查询 + MCU yaw/IMU话题 | 25 Hz |

---

## 七、重要参数（ser2msg_decision_givepoint）

| 参数 | 含义 | 当前状态 |
|------|------|---------|
| `relative_angle` | 云台相对底盘偏航角 | 强制=0（`onYawAngle` 里写死） |
| `imu_angle` | 底盘IMU横滚角 | 强制=0（`onChassisImu` 里写死） |
| `K` | 坐标系缩放系数 | 2.5（默认） |
| `theta` / `shift_x` / `shift_y` | 坐标系旋转/平移（世界对齐） | 由 launch 参数配置 |

`rotbase→virtual` 固定平移偏移：`(0.028, 0.143, -0.1)` 米（机械安装偏移）

---

## 八、一句话总结

> **z_angle 是：把「2D路径方向向量（z=0）」用「IMU坡度信息通过TF链传播到的旋转矩阵」转换后，旋转出的 z 分量对应的仰角，间接等于地面坡度角。补偿公式在坡度>15°时因子变负，23°时导致速度反向，是当前已知的严重Bug。**
