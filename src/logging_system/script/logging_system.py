#!/usr/bin/env python3

import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from movel_seirios_msgs.msg import RunTaskListActionGoal, RunTaskListActionResult
from time import gmtime, strftime
from tf.transformations import euler_from_quaternion
import re 

class LoggingSystem(object):
    def __init__(self):
        rospy.init_node("logging_system")
        self.nav_mode = False
        self.pose_in = False
        self.currentPose=Pose()
        self.odompose=None
        self.old_odompose=None
        self.acc_distance=0.0
        self.up_time=rospy.get_time()
        self.taskType=''
        self.resut_cb_ = False
        self.setupParams()
        self.setupTopics()
        

    def setupParams(self):
        
        self.log_path_ = rospy.get_param('~log_path','/home/movel/.config/movel/logs')
        self.odom_topic_ = rospy.get_param('~odom_topic','/odom')
        self.pose_topic_ = rospy.get_param('~pose','/pose')


    def setupTopics(self):
        rospy.Subscriber(self.odom_topic_, Odometry, self.odom_cb_)
        rospy.Subscriber(self.pose_topic_, Pose, self.pose_cb_)
        rospy.Subscriber('/task_supervisor/goal', RunTaskListActionGoal, self.task_goal_cb_)
        rospy.Subscriber('/task_supervisor/result', RunTaskListActionResult, self.task_goal_result_cb_)


    def odom_cb_(self,msg):
        if self.nav_mode: 
            self.odompose=msg.pose.pose.position
            if self.odompose and self.old_odompose:
                self.acc_distance+=math.hypot(self.old_odompose.x - self.odompose.x, self.old_odompose.y - self.odompose.y)
            self.old_odompose=self.odompose        

    def pose_cb_(self,msg):
        self.pose_in = True
        if self.nav_mode: 
            self.currentPose=msg


    def task_goal_cb_(self,msg):
        if(msg.goal.task_list.tasks[0].type==3 or msg.goal.task_list.tasks[1].type==6) and (not self.nav_mode):
            if(len(msg.goal.task_list.tasks)==1):
                message = strftime("%Y-%m-%d %H:%M:%S", gmtime()) + " Single point navigation ------> Goal : Coordinates : " + re.sub(r"[{}\"]"," ",str(msg.goal.task_list.tasks[0].payload))
                with open(self.log_path_ + '/log_system.txt', 'a') as infile:
                    infile.write(message+'\n') 
            elif(msg.goal.task_list.tasks[1].type==6):
                message = strftime("%Y-%m-%d %H:%M:%S", gmtime()) + " Path navigation reaching intial point ------> Coordinates : " + re.sub(r"[{}\"]"," ",str(msg.goal.task_list.tasks[0].payload))
                message1 = strftime("%Y-%m-%d %H:%M:%S", gmtime()) + " Path navigation mode    ------> Path : Path Name   : " + str(msg.goal.task_list.name) + re.sub(r"[{}\"]"," ",str(msg.goal.task_list.tasks[1].payload))
                with open(self.log_path_ + '/log_system.txt', 'a') as infile:
                    infile.write(message+'\n')
                    infile.write(message1+'\n')  
            else:

                message = strftime("%Y-%m-%d %H:%M:%S", gmtime()) + " Multi point navigation ------> Goal1 : Coordinates : " + re.sub(r"[{}\"]"," ",str(msg.goal.task_list.tasks[0].payload))
                message1 = strftime("%Y-%m-%d %H:%M:%S", gmtime()) + " Multi point navigation ------> Goal2 : Coordinates : " + re.sub(r"[{}\"]"," ",str(msg.goal.task_list.tasks[1].payload))
                with open(self.log_path_ + '/log_system.txt', 'a') as infile:
                    infile.write(message+'\n')
                    infile.write(message1+'\n') 
                if(len(msg.goal.task_list.tasks)==3):
                    message2 = strftime("%Y-%m-%d %H:%M:%S", gmtime()) + " Multi point navigation ------> Goal3 : Coordinates : " + re.sub(r"[{}\"]"," ",str(msg.goal.task_list.tasks[2].payload))
                    with open(self.log_path_ + '/log_system.txt', 'a') as infile:
                        infile.write(message2+'\n')
        self.up_time=rospy.get_time()
        self.nav_mode=True
        self.resut_cb_ = False
    
    def task_goal_result_cb_(self,msg):
        if not self.resut_cb_:
            self.nav_mode=False 
            self.nav_mode=False
            self.resut_cb_=True
            elapsed = rospy.get_time() - self.up_time
            
            str_acc_distance="%.0f" % (self.acc_distance * 1000.0)
            _, _, theta = euler_from_quaternion((self.currentPose.orientation.x, self.currentPose.orientation.y, self.currentPose.orientation.z, self.currentPose.orientation.w))
            str_robot_yaw="%.2f" % math.degrees(theta)
            str_elapsed="%.0f" % elapsed

            if msg.status.status==2:
                message1= strftime("%Y-%m-%d %H:%M:%S", gmtime()) + " Preempted by client " 
                message= strftime("%Y-%m-%d %H:%M:%S", gmtime()) + " Odometer : " + str(str_acc_distance) + " mm " + str(str_robot_yaw) + " deg " + str(str_elapsed) + " sec"
            if msg.status.status==3:
                message1= strftime("%Y-%m-%d %H:%M:%S", gmtime()) + " Task Succeded " 
                message= strftime("%Y-%m-%d %H:%M:%S", gmtime()) + " Odometer : " + str(str_acc_distance) + " mm " + str(str_robot_yaw) + " deg " + str(str_elapsed) + " sec"
            if msg.status.status==4:
                message1= strftime("%Y-%m-%d %H:%M:%S", gmtime()) + " Task got aborted " 
                message= strftime("%Y-%m-%d %H:%M:%S", gmtime()) + " Odometer : " + str(str_acc_distance) + " mm " + str(str_robot_yaw) + " deg " + str(str_elapsed) + " sec"  
            with open(self.log_path_ + '/log_system.txt', 'a') as infile:
                infile.write(message1+'\n')
                infile.write(message+'\n')        
            self.reset_odom_()

    def reset_odom_(self):
        self.up_time = rospy.get_time()
        self.acc_distance = 0.0
        self.nav_mode = False
        self.pose_in = False
        self.resut_cb_ = False
        

    def log_update_func_(self):
        while not rospy.is_shutdown():  
            if self.nav_mode and self.pose_in:
                _, _, theta = euler_from_quaternion((self.currentPose.orientation.x, self.currentPose.orientation.y, self.currentPose.orientation.z, self.currentPose.orientation.w))
                message= strftime("%Y-%m-%d %H:%M:%S", gmtime()) + " Currently at : " + str("%.2f" % self.currentPose.position.x) + ' ' + str("%.2f" % self.currentPose.position.y) + ' ' + str("%.2f" % math.degrees(theta))
                with open(self.log_path_ + '/log_system.txt', 'a') as infile:
                    infile.write(message+'\n')
            rospy.sleep(1)                

if __name__ =='__main__':
    log_system=LoggingSystem()
    log_system.log_update_func_()
    rospy.spin()

