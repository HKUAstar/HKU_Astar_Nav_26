import os

import rospy
from sensor_msgs.msg import PointCloud2

from hdl_graph_slam.srv import SaveMap, SaveMapRequest


class SaveMapOnUpdate(object):
    def __init__(self):
        self.destination = rospy.get_param('~destination')
        self.resolution = float(rospy.get_param('~resolution', 0.05))
        self.utm = bool(rospy.get_param('~utm', False))
        self.trigger_topic = rospy.get_param('~trigger_topic', '/hdl_graph_slam/map_points')
        self.min_interval = float(rospy.get_param('~min_interval', 0.0))
        self.last_saved_stamp = None
        self.last_save_time = rospy.Time(0)
        self.saving = False
        self.has_received_map = False

        directory = os.path.dirname(self.destination)
        if directory and not os.path.isdir(directory):
            os.makedirs(directory)

        rospy.loginfo('waiting for /hdl_graph_slam/save_map')
        rospy.wait_for_service('/hdl_graph_slam/save_map')
        self.save_map = rospy.ServiceProxy('/hdl_graph_slam/save_map', SaveMap)
        self.subscriber = rospy.Subscriber(self.trigger_topic, PointCloud2, self.map_points_callback, queue_size=1)
        rospy.on_shutdown(self.on_shutdown)

    def map_points_callback(self, msg):
        self.has_received_map = True

        if self.saving:
            return

        if self.last_saved_stamp is not None and msg.header.stamp == self.last_saved_stamp:
            return

        now = rospy.Time.now()
        if self.min_interval > 0.0 and (now - self.last_save_time).to_sec() < self.min_interval:
            return

        self.save_map_to_destination(msg.header.stamp, now)

    def save_map_to_destination(self, stamp=None, now=None):
        request = SaveMapRequest()
        request.utm = self.utm
        request.resolution = self.resolution
        request.destination = self.destination

        self.saving = True
        try:
            response = self.save_map(request)
            if response.success:
                if stamp is not None:
                    self.last_saved_stamp = stamp
                if now is None:
                    now = rospy.Time.now()
                self.last_save_time = now
                rospy.loginfo('saved map to %s', self.destination)
            else:
                rospy.logwarn('save_map returned failure for %s', self.destination)
        except (rospy.ServiceException, rospy.ROSException) as exc:
            rospy.logwarn('save_map call failed: %s', exc)
        finally:
            self.saving = False

    def on_shutdown(self):
        if self.saving or not self.has_received_map:
            return

        try:
            rospy.wait_for_service('/hdl_graph_slam/save_map', timeout=1.0)
        except rospy.ROSException:
            rospy.logwarn('save_map service unavailable during shutdown')
            return

        self.save_map_to_destination()


def main():
    rospy.init_node('save_map_on_update')
    SaveMapOnUpdate()
    rospy.spin()


if __name__ == '__main__':
    main()