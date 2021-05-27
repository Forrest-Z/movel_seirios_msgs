#ifndef POINT_CLOUD_MERGER_H
#define POINT_CLOUD_MERGER_H

#include <sstream>

#include <ros_utils/ros_utils.h>
#include <laser_geometry/laser_geometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/LaserScan.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/PCLPointCloud2.h>

class PointCloudMerger
{
public:
  PointCloudMerger(int mode);
  ~PointCloudMerger();
  
private:
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;

  bool loadParams();
  void setupTopics(int mode);

  void onNewData(const sensor_msgs::LaserScan::ConstPtr& scan, const sensor_msgs::PointCloud2::ConstPtr& cloud);
  void onNewClouds(const sensor_msgs::PointCloud2::ConstPtr& cloud1, const sensor_msgs::PointCloud2::ConstPtr& cloud2);

  /** 
   * Remove point fields besides X, Y, Z each with length 4
   */
  void stripUnimportantFields(const sensor_msgs::PointCloud2& cloud_in, sensor_msgs::PointCloud2& cloud_out);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::PointCloud2> sync_pol;
  message_filters::Synchronizer<sync_pol> *sensors_sync_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> *point_cloud_sub_;
  message_filters::Subscriber<sensor_msgs::LaserScan> *laser_scan_sub_;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> sync_pc2;
  message_filters::Synchronizer<sync_pc2> *pc_sync_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> *point_cloud_1_sub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> *point_cloud_2_sub_;

  ros::Publisher merged_point_cloud_pub_;

  /**
   * Merged point cloud frame
   */
  std::string p_target_frame_;
  /**
   * Merged point cloud
   */
  sensor_msgs::PointCloud2 cloud_merged_;
  /**
   * Projector to project laser scan data
   */
  laser_geometry::LaserProjection projector_;

  tf::TransformListener tf_listener_;
};

#endif
