#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan

def on_scan_sub(msg):
    scan_range = msg.ranges
    range_size = len(scan_range)
    rospy.loginfo(f"Range is: {range_size}")

if __name__ == "__main__":
    rospy.init_node("check_lidar_range")
    rospy.Subscriber("/scan", LaserScan, on_scan_sub)

    rospy.spin()
