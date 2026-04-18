#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped, PointStamped


def main():
    rospy.init_node("goal_to_clicked_point")
    in_topic = rospy.get_param("~in_topic", "/move_base_simple/goal")
    out_topic = rospy.get_param("~out_topic", "/clicked_point")
    frame_id = rospy.get_param("~frame_id", "map")

    pub = rospy.Publisher(out_topic, PointStamped, queue_size=10)

    def cb(msg: PoseStamped):
        pt = PointStamped()
        pt.header = msg.header
        if not pt.header.frame_id:
            pt.header.frame_id = frame_id
        pt.point = msg.pose.position
        pub.publish(pt)

    rospy.Subscriber(in_topic, PoseStamped, cb, queue_size=10)
    rospy.loginfo("goal_to_clicked_point: %s -> %s", in_topic, out_topic)
    rospy.spin()


if __name__ == "__main__":
    main()
