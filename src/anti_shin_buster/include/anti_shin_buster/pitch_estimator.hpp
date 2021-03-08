#ifndef pitch_estimator_hpp
#define pitch_estimator_hpp

#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

typedef pcl::PointXYZ Pt;
typedef pcl::PointCloud<Pt> Cloud;
typedef pcl::Normal Nm;
typedef pcl::PointCloud<Nm> NormCloud;

class PitchEstimator
{
public:
  PitchEstimator();
  ~PitchEstimator();

  float xyDistance(float x, float y, Pt &pt);
  size_t getMinXYDistance(float x, float y, Cloud::Ptr &cloud);

private:
  ros::NodeHandle nh_;

  // parameters
  float r_support_;
  int N_support_;
  float leaf_size_;

  // publishers
  ros::Publisher support_cloud_pub_;
  ros::Publisher normal_pose_pub_;
  ros::Publisher sparse_cloud_pub_;

  // callbacks
  void cloudCb(const sensor_msgs::PointCloud2ConstPtr &cloud);
};

#endif