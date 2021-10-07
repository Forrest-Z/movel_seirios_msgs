 
#!/usr/bin/python
# SPDX-License-Identifier: BSD-2-Clause
import tf
import rospy
from geometry_msgs.msg import *
from nav_msgs.msg import Odometry


class Map2OdomPublisher:
	def __init__(self):
		self.broadcaster = tf.TransformBroadcaster()
		self.subscriber = rospy.Subscriber('/wheel_odom', Odometry, self.callback)

	def callback(self, odom_msg):
		self.odom_msg = odom_msg

	def spin(self):
		if not hasattr(self, 'odom_msg'):
			self.broadcaster.sendTransform((0, 0, 0), (0, 0, 0, 1), rospy.Time.now(), 'base_link', 'odom')
			return

		pos = (self.odom_msg.pose.pose.position.x, self.odom_msg.pose.pose.position.y, self.odom_msg.pose.pose.position.z)
		quat = (self.odom_msg.pose.pose.orientation.x, self.odom_msg.pose.pose.orientation.y, self.odom_msg.pose.pose.orientation.z, self.odom_msg.pose.pose.orientation.w)

		map_frame_id = self.odom_msg.header.frame_id
		odom_frame_id = self.odom_msg.child_frame_id

		self.broadcaster.sendTransform(pos, quat, rospy.Time.now(), odom_frame_id, map_frame_id)


def main():
	rospy.init_node('map2odom_publisher')
	node = Map2OdomPublisher()

	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
		node.spin()
		rate.sleep()

if __name__ == '__main__':
	main()

