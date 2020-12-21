/*
 *  Human detection node takes in 3D lidar point cloud input to detect human-like objects.
 */

#ifndef HUMAN_DETECTION_H
#define HUMAN_DETECTION_H

#include <math.h>
#include <algorithm>
#include <functional>
#include <boost/thread/mutex.hpp>

#include <laser_geometry/laser_geometry.h>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include "human_detection/ClusterArray.h"
#include <nav_msgs/OccupancyGrid.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float64.h>

// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
	
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

const int region_max_ = 150; // Change this value to match how far you want to detect.

class HumanDetection
{
private:
  boost::mutex mtx_;                                //!< Mutex lock
  int regions_[100];                                //!< For adaptive clustering
  uint32_t cluster_array_seq_;                      //!< For publishing cluster array
  uint32_t pose_array_seq_;                         //!< For publishing pose array
  geometry_msgs::Point robot_coordinate;            //!< Current robot position
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_pc;       //!< Map represented in point cloud form
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cropped;  //!< Cropped map w.r.t. robot pose
  pcl::PointCloud<pcl::PointXYZ>::Ptr scan_pc;      //!< 2D scan represented in point cloud form
  laser_geometry::LaserProjection projector_;       //!< For converting 2D scan to point cloud

  //! For keeping track of past frames
  int history_index_;
  std::vector<bool> detection_history;

  //! For logging processing rate with frames per second
  int frames;
  clock_t start_time;
  bool reset;

  //! Get map for further processing
  void getMap(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in);

  //! Crop point cloud input for faster processing
  void cropCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out);

  //! Remove floor and ceiling from point cloud input
  void removePlanes(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in, pcl::IndicesPtr pc_indices);

  //! Adaptive clustering for point cloud input
  void adaptiveClustering(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in, pcl::IndicesPtr pc_indices,   std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZI>::Ptr > > &clusters);

  //! Filter for point cloud clusters with human-like dimensions
  void checkDimensions(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZI>::Ptr > > &clusters_in, std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZI>::Ptr > > &clusters_out);

  //! Filter out static objects by comparing point cloud input with map
  void filterStaticObjects(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZI>::Ptr > > &clusters_in, std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZI>::Ptr > > &clusters_out);

  //! Narrow down relevant point cloud by filtering with laser scan converted from 3d point cloud
  void filterLaser(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZI>::Ptr > > &clusters_in, std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZI>::Ptr > > &clusters_out);

public:
  HumanDetection();

  //! Parameters to be loaded (refer to config file for details)
  std::string sensor_model_;
  std::string map_frame_;
  std::string laser_frame_;
  bool print_fps_;            
  double z_axis_min_;
  double z_axis_max_;
  int cluster_size_min_;
  int cluster_size_max_;
  double detection_range_;
  int frames_tracked_;
  double min_height_;
  double max_height_;
  double min_width_;
  double max_width_;
  double cutoff_distance_;
  double voxel_grid_size_;
  double map_inflation_dist_;
  double max_width_length_diff_;
  double close_dist_min_height_;

  //! ROS interfaces
  tf::TransformListener* listener_;
  ros::Publisher cluster_array_pub_;
  ros::Publisher cloud_filtered_pub_;
  ros::Publisher pose_array_pub_;
  ros::Publisher marker_array_pub_;
  ros::Publisher all_clusters_pub_;
  ros::Publisher crop_marker_pub_;
  ros::Publisher map_pub_;
  ros::Publisher detection_pub_;
  ros::Publisher scan_pub_;

  //! Get map to be processed into point cloud format
  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map);

  //! Get current pose of robot for map cropping
  void poseCallback(const geometry_msgs::Pose::ConstPtr &msg);

  //! Get point cloud input to be processed for human detection
  void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& ros_pc2_in);

  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);

  //! Set configurations based on sensor model
  void sensormodelConfig();

  //! Get cropped map based on robot pose
  double getCroppedMap(geometry_msgs::Pose msg);

  //! Find average detection from all tracked frames
  bool historyAveraging(size_t clusters_size, double &average);
};

#endif
