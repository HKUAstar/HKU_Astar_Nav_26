cmake_minimum_required(VERSION 3.5)
project(rog_planner)

add_compile_options(-std=c++14 -O2 -Wall)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  pcl_ros
  rog_map
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs visualization_msgs
                 sensor_msgs tf2_ros tf2_geometry_msgs pcl_ros rog_map
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(rog_planner_node
  src/rog_planner_node.cpp
  src/astar.cpp
  src/bspline_smoother.cpp
  src/static_map_2d.cpp
)

add_dependencies(rog_planner_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(rog_planner_node
  ${catkin_LIBRARIES}
)
