#ifndef LINE_EXTRACTION_ROS_H
#define LINE_EXTRACTION_ROS_H

#include <vector>
#include <string>
#include <mutex>
#include <thread>
#include <cmath>

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/LinearMath/Scalar.h>
#include <tf2_ros/transform_listener.h>
#include <ros_utils/ros_utils.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/transforms.h>

#include "fbsm/LineSegment.h"
#include "fbsm/LineSegmentList.h"
#include "fbsm/line_extraction.h"
#include "fbsm/line.h"
#include "fbsm/feature_description.h"
#include "fbsm/distance_transform.h"
#include "fbsm/utilities.h"

namespace line_extraction
{
class LineExtractionROS
{
public:
  // Constructor / destructor
  LineExtractionROS();
  ~LineExtractionROS();

  const double inf = std::numeric_limits<double>::infinity();

private:
  // ROS
  ros::NodeHandle nh_;
  ros::NodeHandle nh_local_;

  ros::Timer main_timer_;

  ros::Subscriber scan_subscriber_;
  ros::Subscriber map_subscriber_;
  ros::Subscriber goal_subscriber_;
  ros::Subscriber reached_subscriber_;
  ros::Publisher line_publisher_;
  ros::Publisher map_line_publisher_;
  ros::Publisher map_scan_publisher_;
  ros::Publisher laser_scan_publisher_;
  ros::Publisher marker_publisher_;
  ros::Publisher map_marker_publisher_;
  ros::Publisher ini_pos_publisher_;
  ros::Publisher pc_pub, fbsm_pub;
  ros::ServiceServer service;

  tf::StampedTransform transform, transform_odom, transform_amcl, transform_laser;
  tf::StampedTransform transform_laser_base;
  ros::Time scan_stamp;
  tf::TransformListener listener_;

  /** Start Ros Parameters **/
  // Rate of the control loop
  // Unit: hz
  double p_loop_rate_;
  // Frame of the laser scan
  std::string p_laser_frame_;
  // Frame of the map
  std::string p_map_frame_;
  // Frame of the robot base
  std::string p_base_frame_;
  // Frame of the odometry
  std::string p_odom_frame_;
  // Topic of the incoming laser scan
  std::string p_scan_topic_;
  // Topic of the incoming map
  std::string p_map_topic_;
  // Whether to publish markers
  bool p_pub_markers_;
  // Flag to use multi threading
  // Risk hoarding the CPU
  // Unit: bool
  bool p_multithreading_;

  // Line detection package param
  // When debugging matching issues
  // make sure first that the lines in map are well detected
  double p_bearing_std_dev_;
  double p_range_std_dev_;
  double p_least_sq_angle_thresh_;
  double p_least_sq_radius_thresh_;
  double p_max_line_gap_;
  double p_min_line_length_;
  double p_min_range_;
  double p_min_split_dist_;
  double p_outlier_dist_;
  int p_min_line_points_;

  // Scan range cap
  // Unit: m
  double p_max_range_cap_def_;

  // Maximum accepted translation distance
  // a.k.a maximum search radius for translation
  // Unit: m
  double p_dist_upper_threshold_;
  // Maximum accepted rotation
  // a.k.a maximum search angle for rotation
  // Unit: rad
  double p_angle_upper_threshold_;

  // Matching score threshold
  // raycast = predicted scan based on ray-marching for each potential estimate
  // A distance map is pre-processed to improve ray-tracing computation time
  // TODO change score calculation formulas to get more quantifiable score
  double p_raycast_score_threshold_;

  // Matching distance to consider ray as inlier
  // Unit: m
  double p_inlier_dist_;

  // Raycast score is calulated as:
  // (inliers_ratio ^ inliers_exp) * mean_inliers_distance
  // Default to 2
  double p_inliers_exp_;

  // Minmum accepted percentage of inliers
  // Unit: %
  double p_min_inliers_;

  // Minimum accepted change in score from the last run
  // Only used if better check is set to true
  // Helps reduce pose fluctuation due to sensor noise
  double p_precision_;

  // Flag to activate projection check
  // It's a second validation of the transform estimate
  // proj score compares how much of the scan is correctly overlaying the map rather than the minimum overall distance
  // error. If set to true it will reduce the frequency of updates It's usually not needed as long as the raycast score
  // matching is matching correctly Default: false Unit: bool
  bool p_proj_score_;

  // projection matching score threshold
  // Unit: %
  double p_proj_score_threshold_;

  // Projection score change threshold
  // Only used if proj_score is true
  double p_switch_to_proj_threshold_;

  // Histogram bins
  // Used to filter outliers from potential transform estimates
  // Only used if translation or orientation brute force is set to false
  // Filtering is done on rotation estimates first before feeding to translation filtering
  // Higher values will restrict tested hypothesis
  // Default: 6
  int p_filter_bins_;

  // Mask the scan by the detected lines
  // If set to true it will only use the scan points that belongs to lines in the distance error calculation
  // Can be helpful to reduce the impact of random objects in the scene
  // it will disregard the rest of the scan which can cause worse estimates when most of the scan isn't part of lines
  // Default: false
  // Unit: bool
  bool p_scan_line_mask_;

  // Brute force orientation (slow)
  bool p_orientation_bf_;

  // Brute force translation (slow)
  bool p_translation_bf_;

  // Only apply the update if the matching score is better than the current one
  // Default: true
  // Unit: bool
  bool p_better_check_;

  // Matching mode
  // Only the raycast mode is well tested
  // The original paper uses SIFT to match line endpoints which can reduce matching time
  // We started working on that but we didn't get good results using orb detector
  // We can look further into that in the future and check previous commit that include orb_detector file for some
  // insights We can consider using liof detector for that For now just use raycast mode Default: raycast Unit:
  // ["raycast", "FM"]
  std::string p_mode_;

  // Activate matching only near the goal
  bool p_docking_;

  // Linear distance from the goal
  // Only used if docking is set to true
  // Unit: m
  double p_docking_lin_dist_;

  // Angular distance from the goal
  // Only used if docking is set to true
  // Unit: rad
  double p_docking_ang_dist_;

  /** End Ros Parameters **/

  double bestmatch, best_tx, best_ty, best_yaw, best_id, best_ir;
  bool map_ready, map_available, sleeping_;
  std::mutex mutex;

  nav_msgs::OccupancyGrid::ConstPtr map;
  std::vector<std::vector<float>> grid;

  geometry_msgs::PoseStamped goal_;
  // Line extraction
  LineExtraction line_extraction_;
  LineExtraction map_line_extraction_;
  FeatureDescription feature_desc_;
  pcl::PointCloud<pcl::PointXYZ> map_in_laser_pcl;
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> map_pc;
  pcl::KdTree<pcl::PointXYZ>::Ptr map_tree;

  bool data_cached_;  // true after first scan used to cache data

  double max_range_cap, resolution, angle_min, angle_max, angle_increment, range_max, range_min;
  int stop_count = 0;
  int a_clockwise_ = 1;

  // Members
  void run();
  void run(const ros::TimerEvent&);
  void initialize();
  bool loadParams();
  void setupTopics();
  void configureLineExtractors();

  bool correct_pose_srv(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
  void populateLineSegListMsg(const std::vector<Line>&, fbsm::LineSegmentList&);
  void populateMarkerMsg(const std::vector<Line>&, visualization_msgs::Marker&);
  void cacheData(const sensor_msgs::LaserScan::ConstPtr&);
  void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr&);
  void goalCallback(const geometry_msgs::PoseStamped& goal);
  void reachedCallback(const geometry_msgs::PoseStamped& goal);
  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr&);
  void intialize_distance_map();
  void processMap(const ros::Time);
  void scanMask(RangeData& scan, const std::vector<Line>& lines);
  void scanMaskCart(std::vector<double>& xs, std::vector<double>& ys, const std::vector<Line>& lines);
  std::vector<double> getMatchScoreRaycast(const std::vector<double>& scan_x, const std::vector<double>& scan_y,
                                           std::vector<double> tx, std::vector<double> ty, std::vector<double> yaw);
  void rayMarching(const tf::Transform& tf_l_m, std::vector<double>& ranges);
  double getProjectionScore(const std::vector<double>& scan_x, const std::vector<double>& scan_y, double tx, double ty,
                            double yaw);
  void raycastWithRotation(const tf::StampedTransform& rel_transform, std::vector<double>& xs, std::vector<double>& ys,
                           bool pub_map_scan);
  void publishInitialPose(double tx, double ty, double yaw);
  static void getMatchScoreRaycastPerRotation(void* pthis, const std::vector<double>& scan_x,
                                              const std::vector<double>& scan_y, double yaw);
  static void calculateTransformScore(void* pthis, const std::vector<double>& scan_x, const std::vector<double>& scan_y,
                                      double tx, double ty, double yaw);
};

}  // namespace line_extraction

#endif  // LINE_EXTRACTION_ROS_H
