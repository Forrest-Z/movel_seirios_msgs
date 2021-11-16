/*
 *  Pallet detection node takes in 3D lidar point cloud input to detect pallets.
 */

#ifndef PALLET_DETECTION_H
#define PALLET_DETECTION_H

#include <math.h>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <pcl_ros/transforms.h>
#include <tf/tf.h>
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <move_base_msgs/MoveBaseActionResult.h>

// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/statistical_outlier_removal.h>

//#include <dynamic_reconfigure/server.h>
//#include <human_detection/human_detectionConfig.h>

struct Vector3D
{
  float x;
  float y;
  float z;
};

class PalletDetection
{
private:
  //! For logging processing rate with frames per second
  int frames;
  ros::Time start_time;
  bool reset;

  //! For recording xy coordinates of docking goal
  int history_index_;
  std::vector<double> x_history_;
  std::vector<double> y_history_;

  bool goal_published_;
  bool status_received_;
  bool stages_complete_;
  bool next_stage_;
  geometry_msgs::PoseStamped recorded_goal_;

  //! Calculate euclidean distance
  double calcDistance(Eigen::Vector4f c1, Eigen::Vector4f c2);

  //! Calculate angle difference between 2 vectors
  double calcAngleDiff(Vector3D v1, Vector3D v2);

  //! Calculate angle difference between 2 poses
  double calcAngleDiff(geometry_msgs::Pose init_pose, geometry_msgs::Pose target_pose);

  //! Calculate vector cross product
  Vector3D crossProduct(Vector3D vect_A, Vector3D vect_B);

  //! Calculate yaw angle based on 2 points
  double getYaw(std::pair<Eigen::Vector4f,Eigen::Vector4f> centroid_pair);

  //! Filter input point cloud
  void filterCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out);
 
  //! Convert tf to pose
  void tfToPose(geometry_msgs::TransformStamped tf, geometry_msgs::PoseStamped& pose);

  //! Find average dock position from all tracked frames
  void historyAveraging();

public:
  PalletDetection();

  //! Parameters to be loaded (refer to config file for details)
  bool print_fps_;            
  double z_axis_min_;
  double z_axis_max_;
  double x_axis_min_;
  double x_axis_max_;
  int cluster_size_min_;
  int cluster_size_max_;
  double voxel_grid_size_;
  double smoothness_threshold_;
  double curvature_threshold_;
  double verticality_threshold_;
  std::vector<double> pallet_widths_;
  double length_tolerance_;
  double angle_tolerance_;
  double midpoint_tolerance_;
  double max_z_diff_;
  double x_offset_;
  double y_offset_;
  double yaw_offset_;
  double goal_xy_tolerance_;
  double goal_yaw_tolerance_;
  int frames_tracked_;
  std::string target_frame_;
  double inlier_dist_;
  double inlier_yaw_;
  bool use_move_base_;

  //! ROS interfaces
  ros::Publisher cloud_filtered_pub_;
  ros::Publisher goal_pub_;
  ros::Publisher marker_pub_;
  ros::Publisher centroids_pub_;
  ros::Publisher stop_pub_;
  ros::Publisher current_goal_pub_;
  tf2_ros::TransformBroadcaster br_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
  
  //! Get point cloud input to be processed for pallet detection
  void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& ros_pc2_in);

  void statusCallback(std_msgs::Bool success);
  void mbCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr &msg);

  //! dynamic reconfigure
  //void reconfigureCB(human_detection::human_detectionConfig &config, uint32_t level);
  
  bool loadParams();

  //dynamic_reconfigure::Server<human_detection::human_detectionConfig> configServer;
  //dynamic_reconfigure::Server<human_detection::human_detectionConfig>::CallbackType callbackType;

};

#endif
