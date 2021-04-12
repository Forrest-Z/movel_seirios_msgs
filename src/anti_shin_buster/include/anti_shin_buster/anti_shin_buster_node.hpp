#ifndef anti_shin_buster_node_h
#define anti_shin_buster_node_h

#include <ros/ros.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <iostream>

typedef pcl::PointXYZ Pt;
typedef pcl::PointCloud<Pt> Cloud;
typedef pcl::PointCloud<pcl::Normal> CloudNormal;

class AntiShinBusterNode 
{
private:
  ros::NodeHandle nh_;
  ros::Publisher obstacleCloudPub_;
  ros::Publisher surfNormalPub_;

  std::string topic_depth_pointcloud_; // topic for camera depth pointcloud
  float min_z_; // floor level plus margin
  float max_z_; // sensor depth range (minus margin)
  float r_search_; // search radius for normal estimation
  float leaf_size_; // leaf size for voxel grid filter
  int downsample_; // downsample factor;

  tf::TransformListener tfEar_;

public:
  AntiShinBusterNode();
  ~AntiShinBusterNode(){}

  void pointcloudCb(const sensor_msgs::PointCloud2ConstPtr cloud);

  void calcCloudNormal(Cloud::Ptr &in_cloud, CloudNormal::Ptr &cloud_normal);
};

#endif