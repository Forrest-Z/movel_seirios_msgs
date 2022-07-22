#!/usr/bin/python3

import rospy
from nav_msgs.msg import Odometry

def receive_msg():
    rospy.Subscriber("/odom", Odometry, odomCb)

def odomCb(data):
    rospy.loginfo(data)

if __name__=='__main__':
    # need to use rosrun to run this file
    rospy.init_node('test_sub')
    receive_msg()
    rospy.spin()