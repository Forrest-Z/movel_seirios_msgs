#ifndef camera_attitude_monitor_hpp_
#define camera_attitude_monitor_hpp_

#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

using cv::Mat;
using std::string;
using std::vector;

typedef pcl::PointXYZ Pt;
typedef pcl::PointCloud<Pt> Cloud;
typedef pcl::PointXYZI PtI;
typedef pcl::PointCloud<PtI> CloudI;

class CameraAttitudeMonitor
{
public:
  CameraAttitudeMonitor();
  ~CameraAttitudeMonitor(){}

  bool setupTopics();
  bool setupParams();

  bool findFloorNormal(Cloud::Ptr cloud_ptr, tf2::Quaternion &quat);

private:
  // ROS infra
  ros::NodeHandle nh_;
  tf2_ros::TransformBroadcaster tf_mouth_;

  // parameters
  bool use_pointcloud_; // use depth image and generate our own pointcloud if false
  Mat dep_proj_;
  Mat R_opt_to_cam_;
  double min_dep_, max_dep_;
  double seg_distance_;
  int ransac_iter_;
  int max_plane_count_;
  string cam_frame_;
  string level_cam_frame_;

  // bookkeeping
  bool have_dep_info_;

  // subscribers, publishers
  ros::Subscriber dep_sub_;
  ros::Subscriber dep_info_sub_;
  ros::Publisher dep_cloud_pub_;
  ros::Publisher seg_cloud_pub_;

  // callbacks
  void depthCb(const sensor_msgs::ImageConstPtr msg);
  void depthInfoCb(const sensor_msgs::CameraInfoConstPtr msg);
  void cloudCb(const sensor_msgs::PointCloud2ConstPtr msg);
};

#endif