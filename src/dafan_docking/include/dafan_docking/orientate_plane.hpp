#ifndef ORIENTATE_PLANE_H
#define ORIENTATE_PLANE_H

//ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>

//dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <dafan_docking/orientateplaneConfig.h>

//PCL
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
//planar segmentation
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
//PCL filters
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
//Clustering 
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

class OrientatePlane
{
private:
  ros::NodeHandle nh_;

  //////////////////////
  //PUblishers, subscribers and services
  //////////////////////
  ros::Subscriber cloudSub_;
  // ros::Subscriber plannerAdjusterSub_;
  ros::Subscriber goalReceivedSub_;
  ros::Publisher filteredCloudPub_;
  ros::Publisher dockingPlanePub_;
  ros::Publisher dockingPosePub_;
  ros::Publisher debugCloudPub_;
  ros::Publisher XAxisPub_;
  ros::Publisher clusterCloudPub_;

  //Transforms
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_ear_;
  tf::TransformListener tfEar_;

  //dynamic reconfigure server
  dynamic_reconfigure::Server<dafan_docking::orientateplaneConfig> configServer;
  dynamic_reconfigure::Server<dafan_docking::orientateplaneConfig>::CallbackType callbackType;

  //outer params
  bool print_debug_;
  bool pub_docking_goal_;
  long seg_itr;
  
  //goal offset
  float goal_offset_x_;
  float goal_offset_y_;
  //0: Passthrough
  float min_z_; // floor level plus margin
  float max_z_; // sensor depth range (minus margin)
  float min_x_; // minimum depth
  float max_x_; // maximum depth
  //1: Voxel Filter
  float leaf_size_; // leaf size for voxel grid filter
  int min_points_voxel_;
  //2: Statistical Outlier Removal 
  float sor_stddev_;
  int sor_knn_;
  //3: Radius Outlier removal
  float outrem_rad_search_;
  int outrem_min_neigh_;
  //4: Planar segmentation
  int seg_max_itr_;
  float seg_dist_thresh_;
  //5: Euclidean cluster extraction
  float ec_tol_;
  int ec_min_size_;
  int ec_max_size_;

  //Filters
  pcl::PassThrough<pcl::PointXYZ> passthrough_filter_z_;
  pcl::PassThrough<pcl::PointXYZ> passthrough_filter_x_;
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor_filter_;
  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter_;
  pcl::ExtractIndices<pcl::PointXYZ> extract_filter_;

  //planar segmentation
  pcl::SACSegmentation<pcl::PointXYZ> plane_seg_filter_;

  //Radius outlier removal
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;

  //Euclidean extraction
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

public:
  OrientatePlane();
  ~OrientatePlane(){}

  //initialization
  void loadParams();
  void loadPipelineParams();

  //callbacks
  void pointcloudCB(const sensor_msgs::PointCloud2ConstPtr cloud);
  void goalReceivedCB(const std_msgs::BoolConstPtr dist_to_dock);

  //helper functions
  void planeToPose( geometry_msgs::PoseStamped& pose_,  const std::string& frame_id, 
                    const double& t_x, const double& t_y, const double& t_z,
                    const double& ax, const double& ay, const double& az, const double& d);

  void reconfigureCB(dafan_docking::orientateplaneConfig &config, uint32_t level);

};

#endif