#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Transform.h>
#include "livox_ros_driver2/CustomMsg.h"
// Include for testing
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <math.h>

double first_RADIUS;
double second_RADIUS, max_height, start_height;

double slope_1,slp_first_RADIUS, height_1;
double slope_2, slp_second_RADIUS, height_2;
double slope_3,slp_third_RADIUS, height_3;

bool get_msg = 0;
std::string base_frame;
std::string laser_frame;
std::string scan_left_topic;
std::string scan_right_topic;
std::string new_scan_topic;
ros::Publisher pub;
livox_ros_driver2::CustomMsg scan_copy;
geometry_msgs::TransformStamped transformStamped;
bool filter(double x, double y, double z){
    double dx = x + 0.02843;
    return dx*dx + y*y >= 0.35 * 0.35;
}
void scanCallback(const livox_ros_driver2::CustomMsg &scan)
{
    auto scan_record = scan;
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    double angle_rad = -20 * M_PI / 180.0;
    double cos_a = cos(angle_rad);
    double sin_a = sin(angle_rad);
    for (int i = 0; i < scan_record.points.size(); i++){
        if(!filter(scan_record.points[i].x, scan_record.points[i].y, scan_record.points[i].z))continue;
        double x = scan_record.points[i].x + 0.02843;
        double y_temp = scan_record.points[i].y - 0.14337;
        double z_temp = scan_record.points[i].z;

        // 绕X轴旋转
        double y = y_temp * cos_a - z_temp * sin_a;
        double z = y_temp * sin_a + z_temp * cos_a;
        pcl_cloud.points.push_back(pcl::PointXYZ(x, y, z));
    }
    // for(int i=(int)distances.size()-1;i>=0;i--)printf("%lf ",distances[i]);
    // printf("\nmax_dis: %f\n----------------------\n",max_dis);
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(pcl_cloud, output);
    output.header.frame_id = laser_frame; // replace with your frame id
    output.header.stamp = ros::Time::now();
    pub.publish(output);
    // ---
    // printf("Published new scan\n");
}

double max_dis=0;
std::vector<double> distances;

int main(int argc, char **argv)
{
    std::string node_name = "threeD_lidar_merge_pointcloud_test";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    if (!nh.getParam("/" + node_name + "/laser_frame", laser_frame))
    {
        ROS_ERROR("Failed to retrieve parameter 'laser_frame'");
        return -1;
    }
    if (!nh.getParam("/" + node_name + "/scan_left_topic", scan_left_topic))
    {
        ROS_ERROR("Failed to retrieve parameter 'scan_left_topic'");
        return -1;
    }
    if (!nh.getParam("/" + node_name + "/scan_right_topic", scan_right_topic))
    {
        ROS_ERROR("Failed to retrieve parameter 'scan_right_topic'");
        return -1;
    }
    if (!nh.getParam("/" + node_name + "/new_scan_topic", new_scan_topic))
    {
        ROS_ERROR("Failed to retrieve parameter 'new_scan_topic'");
        return -1;
    }

    ros::Subscriber sub_left = nh.subscribe(scan_left_topic, 1, scanCallback);
    ros::Subscriber sub_right = nh.subscribe(scan_right_topic, 1, scanCallback);
    pub = nh.advertise<sensor_msgs::PointCloud2>(new_scan_topic, 1);
    ros::spin();
    return 0;
}
