#!/usr/bin/env python3
import rospy
from livox_ros_driver2.msg import CustomMsg
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2

pub = None

def callback(msg):
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = rospy.get_param('~frame_id', 'aft_mapped')
    pts = []
    for p in msg.points:
        try:
            pts.append((p.x, p.y, p.z))
        except Exception:
            # guard: if point fields differ, skip
            continue
    cloud = pc2.create_cloud_xyz32(header, pts)
    pub.publish(cloud)

def main():
    rospy.init_node('livox_to_pointcloud2')
    in_topic = rospy.get_param('~input_topic', '/livox/lidar_192_168_1_105')
    out_topic = rospy.get_param('~output_topic', '/livox_pointcloud2')
    global pub
    pub = rospy.Publisher(out_topic, PointCloud2, queue_size=2)
    rospy.Subscriber(in_topic, CustomMsg, callback, queue_size=2)
    rospy.loginfo('livox_to_pointcloud2: subscribing %s -> publishing %s', in_topic, out_topic)
    rospy.spin()

if __name__ == '__main__':
    main()
