# PointCloud Converter - Livox CustomMsg to sensor_msgs::PointCloud2

## 概述

`pointcloud_converter` 是一个ROS节点，用于将Livox激光雷达的自定义消息格式 (`livox_ros_driver2::CustomMsg`) 转换为标准的ROS点云消息格式 (`sensor_msgs::PointCloud2`)。

## 功能特性

- **消息格式转换**: 将Livox CustomMsg转换为PointCloud2格式
- **点云过滤**: 支持距离过滤，移除过近的点
- **TF坐标系转换**: 支持设置输出点云的坐标系
- **高效处理**: 50Hz的处理频率

## 编译

该程序已集成到bot_sim包中，使用以下命令编译：

```bash
cd ~/AstarTraining/Old_nav/sim_nav
catkin_make
```

## 使用方法

### 方法1: 使用Launch文件

```bash
roslaunch bot_sim pointcloud_converter.launch
```

### 方法2: 手动运行节点

```bash
rosrun bot_sim pointcloud_converter
```

## ROS参数配置

节点需要以下参数：

| 参数名 | 类型 | 描述 | 默认值 |
|--------|------|------|--------|
| `laser_frame` | string | 输出点云的坐标系 | `aft_mapped` |
| `input_topic` | string | 输入Livox消息的ROS话题 | `/livox/lidar` |
| `output_topic` | string | 输出PointCloud2的ROS话题 | `/pointcloud_converted` |

### 配置示例

可以在launch文件中配置参数，如 `pointcloud_converter.launch`：

```xml
<node pkg="bot_sim" type="pointcloud_converter" name="pointcloud_converter" output="screen">
    <param name="laser_frame" type="string" value="aft_mapped" />
    <param name="input_topic" type="string" value="/livox/lidar" />
    <param name="output_topic" type="string" value="/pointcloud_converted" />
</node>
```

## 输入和输出

### 输入
- **话题**: 由参数 `input_topic` 指定 (默认: `/livox/lidar`)
- **消息类型**: `livox_ros_driver2::CustomMsg`

### 输出
- **话题**: 由参数 `output_topic` 指定 (默认: `/pointcloud_converted`)
- **消息类型**: `sensor_msgs::PointCloud2`
- **点类型**: PCL PointXYZ (x, y, z坐标)

## 自定义过滤规则

程序中的 `filter()` 函数可用于自定义点云过滤规则。修改该函数以实现以下功能：

- **距离过滤**: 移除距离传感器过近或过远的点
- **高度过滤**: 移除高度超出范围的点
- **区域过滤**: 移除在特定区域外的点

示例（修改filter函数）：

```cpp
bool filter(double x, double y, double z)
{
    // 最小距离过滤
    double distance_sq = x*x + y*y + z*z;
    if (distance_sq < 0.1 * 0.1) return false; // 移除距离小于0.1m的点
    
    // 高度过滤
    if (z > 3.0 || z < -1.0) return false; // 移除高度超出范围的点
    
    return true; // 保留该点
}
```

## 多个激光雷达合并

如果需要合并多个激光雷达的点云（如 `threeD_lidar_merge_pointcloud.cpp` 所示），可以：

1. 订阅多个输入话题
2. 对每个点云应用变换（位置偏移、旋转等）
3. 将所有点合并到单一点云中
4. 发布合并后的点云

## 调试

启用调试信息：

```bash
rosrun bot_sim pointcloud_converter __name:=pointcloud_converter __log_level:=debug
```

监听输出话题（可视化）：

```bash
rviz
```

在RViz中添加PointCloud2显示，订阅参数 `output_topic` 中指定的话题。

## 参考

- [Livox ROS Driver 2](https://github.com/Livox-SDK/livox_ros_driver2)
- [PCL Point Cloud Library](https://pointclouds.org/)
- [ROS PointCloud2 Message Format](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html)
