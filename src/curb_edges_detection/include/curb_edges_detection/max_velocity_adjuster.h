#ifndef VELOCITY_ADJUSTER_H_
#define VELOCITY_ADJUSTER_H_

#include <cmath>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <movel_seirios_msgs/SetSpeed.h>
#include "tf/transform_listener.h"
#include <ros_utils/ros_utils.h>
#include <iostream>
#include <actionlib_msgs/GoalStatusArray.h>


class VelocityAdjuster
{
public:
    VelocityAdjuster();
    ~VelocityAdjuster(){};

    void setupTopics();
    void cmdVelCB(const geometry_msgs::Twist::ConstPtr& twistmsg);
    void odomCB(const nav_msgs::Odometry::ConstPtr& odommsg);
    void statusCB(const actionlib_msgs::GoalStatusArray::ConstPtr& msg);
    void setupParams();
    bool check();

protected:

    ros::NodeHandle nh_;
    ros::Subscriber cmd_vel_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber status_sub_;    
    double roll_;
    double pitch_;
    double yaw_;
    ros::ServiceClient vel_client_;
    double max_vel_x_;
    double max_vel_x_backwards_;
    double acc_lim_x_;
    double vel_x_thresh_;
    double max_vel_theta_;
    std::string cmd_vel_topic_;
    std::string odom_topic_;
    std::string status_topic_;
    geometry_msgs::Twist actual_vel_;
    geometry_msgs::Twist current_vel_;
};

#endif