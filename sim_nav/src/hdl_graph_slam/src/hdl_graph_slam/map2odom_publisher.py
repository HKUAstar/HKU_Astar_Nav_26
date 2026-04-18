#!/usr/bin/python
# SPDX-License-Identifier: BSD-2-Clause
import tf
import rospy
from geometry_msgs.msg import *


class Map2OdomPublisher:
	def __init__(self, odom_frame_id = 'odom', map_frame_id = 'map'):
		self.default_odom_frame_id = odom_frame_id
		self.default_map_frame_id = map_frame_id
		self.last_source_stamp = None
		self.latest_pose = None
		self.latest_map_frame_id = map_frame_id
		self.latest_odom_frame_id = odom_frame_id
		self.broadcaster = tf.TransformBroadcaster()
		self.subscriber = rospy.Subscriber('/hdl_graph_slam/odom2pub', TransformStamped, self.callback)

	def callback(self, odom_msg):
		self.latest_pose = odom_msg.transform
		self.latest_map_frame_id = odom_msg.header.frame_id or self.default_map_frame_id
		self.latest_odom_frame_id = odom_msg.child_frame_id or self.default_odom_frame_id
		self.last_source_stamp = odom_msg.header.stamp

	def spin(self):
		if self.latest_pose is None:
			self.broadcaster.sendTransform((0, 0, 0), (0, 0, 0, 1), rospy.Time.now(), self.default_odom_frame_id, self.default_map_frame_id)
			return

		pose = self.latest_pose
		pos = (pose.translation.x, pose.translation.y, pose.translation.z)
		quat = (pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w)
		self.broadcaster.sendTransform(pos, quat, rospy.Time.now(), self.latest_odom_frame_id, self.latest_map_frame_id)


def main():
	rospy.init_node('map2odom_publisher')

	# get some parameters to define what default frame_id's should be used while we wait for our first odom message
	map_frame_id = rospy.get_param('~map_frame_id', 'map')
	odom_frame_id = rospy.get_param('~odom_frame_id', 'odom')

	node = Map2OdomPublisher(odom_frame_id, map_frame_id)	

	rate = rospy.Rate(30.0)
	while not rospy.is_shutdown():
		node.spin()
		rate.sleep()

if __name__ == '__main__':
	main()
