#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
from sensor_msgs import point_cloud2
from movel_seirios_msgs.msg import movelPointcloud2
from movel_seirios_msgs.msg import Point
import std_msgs.msg

class Conversion():
    def __init__(self):
        rospy.init_node("Conversion")
        
       
   
        self.movel_pointclould_pub = rospy.Publisher('/movel_pointclould', movelPointcloud2)

        rospy.Subscriber('velodyne_points', PointCloud2, self.pointcloud_sub)
        
        # Wait for the pointcloud topic to become available
        rospy.wait_for_message('velodyne_points', PointCloud2)
        
    def pointcloud_sub(self, msg):
        # Initialize the centroid coordinates point count
        point_x = point_y = point_z = []
        movel_msg = movelPointcloud2()
        h = std_msgs.msg.Header()
        h.stamp = rospy.Time.now() 
        movel_msg.header = h
        pt = Point()
        # Read in the x, y, z coordinates of all points in the cloud
        for point in point_cloud2.read_points(msg, skip_nans=True):
            pt.x = point[0]
            pt.y = point[1]
            pt.z = point[2]
            movel_msg.points.append(pt)

        self.movel_pointclould_pub.publish(movel_msg)

        
   
                   
if __name__ == '__main__':
    try:
        Conversion()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Conversion node terminated.")
