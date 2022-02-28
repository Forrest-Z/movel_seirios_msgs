/*
 *  USAGE TOPICS:
 *  1. /human_detection/detection      : Detection value ranging from 0 to 1, where 0 is no detection,
 *                                       and 1 is detection with absolute certainty
 *  2. /human_detection/human_clusters : Visualize bounding boxes of all detected human-like objects
 *
 *  DEBUGGING TOPICS:
 *  1. /human_detection/filtered_cloud : Pre-processed input point cloud
 *  2. /human_detection/crop           : Visualize crop edges for point cloud input and map
 *  3. /human_detection/map_cloud      : Point cloud representation of cropped map
 *  4. /human_detection/scan_cloud     : Point cloud representation of 2d laser data converted from 3D lidar data
 */ 

#include <human_detection/human_detection.h>
#include <ros_utils/ros_utils.h>
#include <movel_hasp_vendor/license.h>

// HumanDetection detect;

// bool loadParams(ros::NodeHandle& nh_params_)
// {
//   ros_utils::ParamLoader loader(nh_params_);

//   loader.get_required("sensor_model", detect.sensor_model_);
//   loader.get_required("map_frame", detect.map_frame_);
//   loader.get_required("laser_frame", detect.laser_frame_);
//   loader.get_required("print_fps", detect.print_fps_);
//   loader.get_required("z_axis_min", detect.z_axis_min_);
//   loader.get_required("z_axis_max", detect.z_axis_max_);
//   loader.get_required("detection_range", detect.detection_range_);
//   loader.get_required("frames_tracked", detect.frames_tracked_);
//   loader.get_required("cluster_size_min", detect.cluster_size_min_);
//   loader.get_required("cluster_size_max", detect.cluster_size_max_);
//   loader.get_required("min_height", detect.min_height_);
//   loader.get_required("max_height", detect.max_height_);
//   loader.get_required("min_width", detect.min_width_);
//   loader.get_required("max_width", detect.max_width_);
//   loader.get_required("cutoff_distance_from_robot", detect.cutoff_distance_);
//   loader.get_required("voxel_grid_leaf_size", detect.voxel_grid_size_);
//   loader.get_required("map_inflation_dist", detect.map_inflation_dist_);
//   loader.get_required("max_width_length_diff", detect.max_width_length_diff_);
//   loader.get_required("close_dist_min_height", detect.close_dist_min_height_);

//   return loader.params_valid();
// }

int main(int argc, char **argv)
{
  #ifdef MOVEL_LICENSE
  MovelLicense ml;                                                                                                   
  if (!ml.login())
    return 1;                                                                                                           
  #endif

  ros::init(argc, argv, "human_detection");

  HumanDetection detect;

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  ros::Time::waitForValid();
  if (!detect.loadParams())
  {
    ROS_FATAL("Error during parameter loading. Shutting down.");
    return 0;
  }
  ROS_INFO("All parameters loaded. Launching.");

  //! TF listener
  tf::TransformListener listener;
  detect.listener_ = &listener;

  //! Subscribers
  ros::Subscriber point_cloud_sub =  nh.subscribe("/velodyne_points", 1, &HumanDetection::pointCloudCallback, &detect);
  ros::Subscriber map_sub =  nh.subscribe("/map", 1, &HumanDetection::mapCallback, &detect);
  ros::Subscriber pose_sub =  nh.subscribe("/pose", 1, &HumanDetection::poseCallback, &detect);
  ros::Subscriber laser_sub =  nh.subscribe("/scan", 1, &HumanDetection::scanCallback, &detect);

  //! Publishers

  detect.cloud_filtered_pub_ =   private_nh.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 100);
  detect.marker_array_pub_ =  private_nh.advertise<visualization_msgs::MarkerArray>("human_clusters", 100);
  detect.crop_marker_pub_ =  private_nh.advertise<visualization_msgs::Marker>("crop", 100);
  detect.map_pub_ =  private_nh.advertise<sensor_msgs::PointCloud2>("map_cloud", 1);
  detect.detection_pub_ =  private_nh.advertise<std_msgs::Float64>("detection", 1);
  detect.scan_pub_ =  private_nh.advertise<sensor_msgs::PointCloud2>("scan_cloud", 1);

  detect.clusters_pub_ = private_nh.advertise<sensor_msgs::PointCloud2>("clusters_cloud", 1);

  //! Publishers that are unneccessary or only for debugging in this version
  //detect.cluster_array_pub_ = private_nh.advertise<human_detection::ClusterArray>("clusters", 100);
  //detect.pose_array_pub_ = private_nh.advertise<geometry_msgs::PoseArray>("poses", 100);
  //detect.all_clusters_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("all_clusters", 100);
  
  //! Set configuration according to sensor model
  detect.sensormodelConfig();

  ros::spin();
  #ifdef MOVEL_LICENSE                                                                                                    
  ml.logout();          
  #endif    

  return 0;
}
