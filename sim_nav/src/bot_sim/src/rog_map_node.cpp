// Stage 1 ROG-Map observability driver.
// Constructs a single ROGMap instance bound to the node's private namespace
// (~rog_map/...). All inputs come via topic callbacks (cloud + odom) and all
// outputs are visualization-only PointCloud2 / MarkerArray under the node's
// private namespace. No planner-side change.
#include <ros/ros.h>
#include <pcl/console/print.h>
#include "rog_map/rog_map.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "rog_map_node");
  ros::NodeHandle nh("~");
  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

  auto map = std::make_shared<rog_map::ROGMap>(nh);
  ROS_INFO("[rog_map_node] ROGMap constructed; entering spin.");

  ros::AsyncSpinner spinner(0);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
