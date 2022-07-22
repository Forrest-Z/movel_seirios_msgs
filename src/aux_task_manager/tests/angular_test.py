#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

if __name__ == "__main__":
    rospy.init_node("ang_test")
    test_ang_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    while not rospy.is_shutdown():
        move_msg = Twist()
        move_msg.linear.x = 0.0
        move_msg.angular.z = 0.1

        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0

        begin_time = rospy.Time.now()
        end_time = begin_time + rospy.Duration(60) # TODO: change duration to time it take for 1 full turn

        while rospy.Time.now() < end_time:
            test_ang_pub.publish(move_msg)
            
        rospy.loginfo("Stopped")
        test_ang_pub.publish(stop_msg)
        rospy.signal_shutdown("test stop")
