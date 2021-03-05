#pragma once
 
#include <ros/ros.h>
 
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
 
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <std_msgs/Float32MultiArray.h> 
#include <sensor_msgs/PointCloud2.h>
 
class PclTestCore
{
 
  private:
    ros::Subscriber sub_point_cloud_;
    ros::Subscriber sub_RPY_;
    std::vector<double> quaternion_{0, 0, 0, 1};
    ros::Publisher pub_filtered_points_;
    ros::Publisher pub_occupancy_grid_;
    float DegreeToRadian(float degrees);
    void rot_array_cb(const std_msgs::Float32MultiArrayConstPtr& in_array);
    void point_cb(const sensor_msgs::PointCloud2ConstPtr& in_cloud);
 
  public:
    PclTestCore(ros::NodeHandle &nh);
    ~PclTestCore();
    void Spin();
};