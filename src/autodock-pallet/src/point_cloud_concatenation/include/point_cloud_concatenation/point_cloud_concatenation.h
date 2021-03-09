#include <assert.h>
#include <sys/time.h>
#include <unistd.h>
#include <ros/ros.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>

#include <list>
#include <string>
#include <boost/algorithm/string.hpp>
#include<geometry_msgs/Twist.h>
#include <fstream>
#include<pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include<pcl/point_types.h>
#include<pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/LaserScan.h>

#include <pcl/common/transforms.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <boost/bind.hpp>
#include <boost/function.hpp>

using namespace std;
using namespace message_filters;

using namespace std;

class FusedPcl {
private:

    ros::Subscriber front_point_cloud;
    ros::Subscriber rear_point_cloud;
    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    ros::Publisher fused_point_cloud_front;
    ros::Publisher fused_point_cloud_rear;
    ros::Publisher fused_point_cloud ;
    ros::Subscriber point_cloud_subscriber_front;
    ros::Subscriber point_cloud_subscriber_rear;
    pcl::PointCloud<pcl::PointXYZI> front_cloud;
    pcl::PointCloud<pcl::PointXYZI> rear_cloud;
    pcl::PointCloud<pcl::PointXYZI> fused_cloud;
    sensor_msgs::LaserScan front_msg;
    sensor_msgs::LaserScan rear_msg;
    float front_lidar_base_link_offset_x;
    float front_lidar_base_link_offset_y;
    float rear_lidar_base_link_offset_x;
    float rear_lidar_base_link_offset_y;


public:

    FusedPcl();
    void callback(const sensor_msgs::PointCloud2::ConstPtr& msg_front, const  sensor_msgs::PointCloud2::ConstPtr& msg_rear);
    void frontCloudCallback(const sensor_msgs::LaserScan& msg);

    void rearCloudCallback(const sensor_msgs::LaserScan& msg);
    void fusedCloud();

};


