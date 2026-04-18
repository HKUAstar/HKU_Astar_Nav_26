#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "livox_ros_driver2/CustomMsg.h"

// Global variables
std::string laser_frame;
std::string input_topic;
std::string output_topic;
ros::Publisher pub;
bool is_custom_msg = true;  // Flag to determine message type

// Callback for CustomMsg format
void customMsgCallback(const livox_ros_driver2::CustomMsg::ConstPtr &msg)
{
    static int callback_count = 0;
    callback_count++;
    
    ROS_INFO("[Callback #%d] Received CustomMsg with %lu points", callback_count, msg->points.size());

    // Filter function to remove points based on criteria
    auto filter = [](double x, double y, double z) {
        double dx = x;
        double dy = y;
        return dx*dx + dy*dy >= 0.1 * 0.1; // Minimum distance filter
    };

    // Create PCL PointCloud
    pcl::PointCloud<pcl::PointXYZI> pcl_cloud;

    // Convert CustomMsg points to PCL PointCloud
    for (size_t i = 0; i < msg->points.size(); i++)
    {
        // Apply filter
        if (!filter(msg->points[i].x, msg->points[i].y, msg->points[i].z))
            continue;

        // Add point to cloud with intensity (reflectivity)
        pcl::PointXYZI point;
        point.x = msg->points[i].x;
        point.y = msg->points[i].y;
        point.z = msg->points[i].z;
        point.intensity = static_cast<float>(msg->points[i].reflectivity) / 255.0f;
        pcl_cloud.points.push_back(point);
    }

    // Set size info for PCL cloud
    pcl_cloud.width = pcl_cloud.points.size();
    pcl_cloud.height = 1;
    pcl_cloud.is_dense = false;

    ROS_INFO("[Callback #%d] After filtering: %lu points", callback_count, pcl_cloud.points.size());

    // Convert PCL PointCloud to ROS PointCloud2 message
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(pcl_cloud, output);
    output.header.frame_id = laser_frame;
    output.header.stamp = msg->header.stamp;

    // Publish the converted point cloud
    pub.publish(output);
    ROS_INFO("[Callback #%d] Published PointCloud2 with frame=%s", callback_count, output.header.frame_id.c_str());
}

// Callback for PointCloud2 format (direct pass-through with potential re-frame)
void pointcloud2Callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    ROS_DEBUG("Received PointCloud2 message");
    
    sensor_msgs::PointCloud2 output = *msg;
    output.header.frame_id = laser_frame;
    
    pub.publish(output);
    ROS_DEBUG("Published PointCloud2 message");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointcloud_converter");
    ros::NodeHandle nh("~");  // Private namespace NodeHandle

    // Get parameters from ROS parameter server
    if (!nh.getParam("laser_frame", laser_frame))
    {
        ROS_ERROR("Failed to retrieve parameter 'laser_frame'");
        return -1;
    }
    if (!nh.getParam("input_topic", input_topic))
    {
        ROS_ERROR("Failed to retrieve parameter 'input_topic'");
        return -1;
    }
    if (!nh.getParam("output_topic", output_topic))
    {
        ROS_ERROR("Failed to retrieve parameter 'output_topic'");
        return -1;
    }
    
    nh.getParam("is_custom_msg", is_custom_msg);  // Optional parameter, default is true

    ROS_INFO("Starting pointcloud_converter");
    ROS_INFO("  Input topic:      %s", input_topic.c_str());
    ROS_INFO("  Output topic:     %s", output_topic.c_str());
    ROS_INFO("  Laser frame:      %s", laser_frame.c_str());
    ROS_INFO("  Message type:     %s", is_custom_msg ? "CustomMsg" : "PointCloud2");

    // Create publisher
    pub = nh.advertise<sensor_msgs::PointCloud2>(output_topic, 10);

    // Create subscriber based on message type
    ros::Subscriber sub;
    if (is_custom_msg)
    {
        ROS_INFO("Subscribing to CustomMsg format...");
        sub = nh.subscribe(input_topic, 10, customMsgCallback);
    }
    else
    {
        ROS_INFO("Subscribing to PointCloud2 format...");
        sub = nh.subscribe<sensor_msgs::PointCloud2>(input_topic, 10, pointcloud2Callback);
    }

    ROS_INFO("Waiting for messages on topic: %s", input_topic.c_str());
    
    ros::spin();  // Simple spin, callback-driven processing

    return 0;
}
