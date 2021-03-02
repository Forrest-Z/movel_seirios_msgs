#!/usr/bin/env python

import rospy
import sys
import tf
import tf2_ros
import geometry_msgs.msg
from nav_msgs.msg import Odometry
import tf2_geometry_msgs
from std_msgs.msg import String, Bool
import csv

class Static_Transform(object):

    def __init__(self):
        self.node = rospy.init_node('static_tranform_orbmap')
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer) 
        self.x_in_map = 0.0
        self.y_in_map = 0.0
        self.odom_recieved=False
        self.odom_sub_ = rospy.Subscriber('/odom',Odometry, self.cb_odom, queue_size = 1)
        self.list_points=[]
        self.path_value=rospy.get_param('/Orbomator_Transform/path')
        with open(self.path_value, mode='r') as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            for row in csv_reader:
                self.list_points.append(row)



    def cb_odom(self,msg):
            self.odom_recieved=True
            self.latest_odom_ = msg
            transform = self.tf_buffer.lookup_transform('map',msg.header.frame_id, #source frame
                                          rospy.Time(0), #get the tf at first available time
                                         rospy.Duration(1.0))
            pose_transformed = tf2_geometry_msgs.do_transform_pose(msg.pose, transform)
            self.x_in_map = pose_transformed.pose.position.x
            self.y_in_map = pose_transformed.pose.position.y
            #print self.x_in_map,self.y_in_map
    
    def transform(self):
            while not rospy.is_shutdown():
                rate = rospy.Rate(10.0)
                if self.odom_recieved:
                    broadcaster = tf.TransformBroadcaster() 
                    transform=self.check_transform()
                    #print (transform)
                    broadcaster.sendTransform((float(transform[0]),float(transform[1]),float(transform[2])),(float(transform[3]),float(transform[4]),float(transform[5]),float(transform[6])),rospy.Time.now(),"orb_map", "map")
                rate.sleep()

    def check_transform(self):
        count=-1
        for points in self.list_points:
            v1 = (float(points[9])-float(points[7]), float(points[10])-float(points[8]))   # Vector 1
            v2 = (float(points[9])-self.x_in_map, float(points[10])-self.y_in_map)   # Vector 2
            #v2 = (float(points[9])-(0), float(points[10])-(0))
            xp = v1[0]*v2[1] - v1[1]*v2[0]  # Cross product
            count=count+1
            #print xp
            xp = xp*-1
            try:
                if xp > 0:
                    #print ("xp>0",count)
                    above=True
                elif xp < 0:
                    #print("xp<0",count)
                    if above:
                        above=False
                        return self.list_points[count-1]   
                else:
                    return points
            except:
                continue
        #print (count)
        return self.list_points[count]


if __name__ == '__main__':
    static_tranform = Static_Transform()
    static_tranform.transform()
