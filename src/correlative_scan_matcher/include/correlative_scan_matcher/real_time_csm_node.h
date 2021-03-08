#ifndef CORRELATIVE_SCAN_MATCHER_REAL_TIME_CSM_NODE_H
#define CORRELATIVE_SCAN_MATCHER_REAL_TIME_CSM_NODE_H

#include <correlative_scan_matcher/probability_grid.h>
#include <correlative_scan_matcher/SetMap.h>

#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Geometry>

#include <mutex>
#include <thread>

/**
 * Classes and functions used by the correlative scan matcher.
 */
namespace correlative_scan_matcher
{
/**
 * Normalizes an angle to [0, 2pi].
 *
 * @param angle the input angle in radians
 * @return The normalized angle in radians
 */
double normalizeAngle(double angle)
{
  /*
  // normalize angle to [-pi, +pi]
  while (angle > M_PI)
    angle -= 2 * M_PI;
  while (angle < -M_PI)
    angle += 2 * M_PI;

  return angle;
  */

  // normalize angle to [0, 2pi]
  return angle - 2 * M_PI * floor(angle / (2 * M_PI));
}

/**
 * Stores 2D poses and their probability score.
 */
class Pose
{
public:
  /**
   * Default constructor that initializes all members to 0.
   */
  Pose() : x(0.), y(0.), theta(0.), score(0.) {}
  /**
   * Linear x position in meters.
   */
  double x;
  /**
   * Linear y position in meters.
   */
  double y;
  /**
   * Angular position in radians.
   */
  double theta;
  /**
   * Probability score.
   */
  double score;
  /**
   * Constructor that accepts a pose and initializes score to 0.
   *
   * @param x linear x position in meters
   * @param y linear y position in meters
   * @param theta angular position in radians
   */
  Pose(double x, double y, double theta) : x(x), y(y), theta(theta), score(0.) {}
  /**
   * Constructor that accepts a pose and score.
   *
   * @param x linear x position in meters
   * @param y linear y position in meters
   * @param theta angular position in radians
   * @param score probability score
   */
  Pose(double x, double y, double theta, double score) : x(x), y(y), theta(theta), score(score) {}

  /**
   * Addition operator.
   */
  Pose& operator+=(const Pose& pose)
  {
    this->x += pose.x;
    this->y += pose.y;
    this->theta += normalizeAngle(pose.theta);
    this->score += pose.score;

    return *this;
  }
};

/**
 * Comparator.
 * Used to order Pose objects by score in a priority queue.
 */
bool operator<(const Pose &a, const Pose &b)
{
  return a.score < b.score;
}

/**
 * Addition operator.
 *
 * @todo sum.theta should be normalized
 */
Pose operator+(const Pose &a, const Pose &b)
{
  Pose sum;
  sum.x = a.x + b.x;
  sum.y = a.y + b.y;
  sum.theta = a.theta + b.theta;
  sum.score = a.score + b.score;

  return sum;
}

/**
 * Division operator.
 */
Pose operator/(const Pose &pose, const int &num)
{
  Pose div;
  div.x = pose.x / num;
  div.y = pose.y / num;
  div.theta = pose.theta / num;
  div.score = pose.score / num;

  return div;
}

/**
 * Returns the Pose representation of a tf::Transform object.
 *
 * @param t input tf::Transform object
 * @return The Pose representation of the input tf::Transform
 */
Pose tf2Pose(const tf::Transform &t)
{
  return Pose(t.getOrigin().getX(), t.getOrigin().getY(), tf::getYaw(t.getRotation()));
}

/**
 * Returns the tf::Transform representation of a Pose object.
 *
 * @param p input Pose object
 * @return The tf::Transform representation of the input Pose
 */
tf::Transform pose2TF(const Pose &p)
{
  tf::Transform t;
  t.setOrigin(tf::Vector3(p.x, p.y, 0.));
  tf::Quaternion q;
  q.setRPY(0, 0, p.theta);
  t.setRotation(q);

  return t;
}

/**
 * Returns the linear distance between two Pose objects.
 *
 * @param a input pose object
 * @param b input pose object
 * @return Linear distance in meters
 */
double getLinearDist(const Pose &a, const Pose &b)
{
  return sqrt(pow(b.y - a.y, 2.0) + pow(b.x - a.x, 2.0));
}

/**
 * Returns the angular distance between two Pose objects.
 *
 * @param a input pose object
 * @param b input pose object
 * @return Angular distance normalized to [0, 2pi] radians
 * @see normalizeAngle
 */
double getAngularDist(const Pose &a, const Pose &b)
{
  // return fabs(normalizeAngle(b.theta - a.theta));
  return fabs(normalizeAngle(b.theta) - normalizeAngle(a.theta));
}

/**
 * Stores laser data
 */
class LaserData
{
public:
  /**
   * Constructor.
   */
  LaserData()
  {
    ranges = NULL;
  };
  /**
   * Destructor.
   */
  ~LaserData()
  {
    delete[] ranges;
  };
  /**
   * Number of rays in the scan.
   */
  int range_count;
  /**
   * Number of valid rays in the scan.
   */
  int valid_range_count;
  double range_max, range_min;
  /**
   * Ray range and angle data.
   * Range is stored at index 0 and angle is stored at index 1.
   */
  double (*ranges)[2];
};

/**
 * Implementation of Real-Time Correlative Scan Matching (Olson, 2009)
 * https://april.eecs.umich.edu/pdfs/olson2009icra.pdf
 */
class RealTimeCSM
{
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  tf::TransformListener tf_listener_;
  tf::TransformBroadcaster tf_broadcaster_;

  ros::Subscriber map_sub_;
  ros::Subscriber scan_sub_;
  ros::Publisher low_res_grid_pub_;
  ros::Publisher high_res_grid_pub_;
  /* ros::Publisher corrected_cloud_pub_; */
  ros::Publisher initial_pose_pub_;
  ros::Publisher raycast_pub_;
  ros::ServiceServer set_map_srv_;
  ros::ServiceServer enable_srv_;
  ros::ServiceServer pause_srv_;

  /**
   * Prevents access to probability grids while map is being processed
   */
  std::timed_mutex grid_mutex_;

  void loadParams();
  void setupTopics();

  void mapCallback(const nav_msgs::OccupancyGridConstPtr &msg);
  void scanCallback(const sensor_msgs::LaserScanConstPtr &msg);
  bool onSetMapSrv(correlative_scan_matcher::SetMap::Request &req, correlative_scan_matcher::SetMap::Response &resp);
  bool onEnable(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp);
  bool onPause(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp);

  /**
   * Whether the matcher is enabled.
   * Disabled by default.
   */
  bool enabled_;
  /**
   * Whether the matcher is paused.
   * Pausing stops matching but continues to publish the latest valid estimate.
   */
  bool paused_;
  /**
   * Whether a map has been received.
   */
  bool got_map_;
  /**
   * Whether we have the laser pose in the base frame.
   */
  bool got_laser_pose_;
  /**
   * Latest laser data
   */
  LaserData *laser_data_;

  /**
   * The laser pose in the base frame.
   */
  Pose laser_pose_;

  /**
   * The map frame.
   * Automatically retrieved from the incoming map.
   */
  std::string map_frame_;
  /**
   * The robot base frame.
   * Used to get the initial pose estimate in map_frame_.
   *
   * Parameter with default value base_link.
   */
  std::string base_frame_;
  /**
   * The correction is published with respect to this frame
   * (reference_frame_ <- matched_frame_).
   * If set to 'odom', the matcher effectively acts as a
   * correction for odometry drift, like AMCL.
   *
   * Parameter with default value map.
   */
  std::string reference_frame_;
  /**
   * The corrected base_frame_.
   * If invert_transform_ is true,
   * this is instead the corrected map_frame_.
   *
   * Parameter with default value csm/base_link.
   */
  std::string matched_frame_;
  /**
   * Standard deviation in meters for the laser Gaussian noise model.
   * Should match the actual standard deviation of the laser
   * as much as possible.
   *
   * Parameter with default value 0.05.
   */
  double sigma_;
  /**
   * Search space in x in meters, with respect to base_frame_.
   * Ignored if do_global_search_ is true.
   *
   * Parameter with default value 0.1.
   */
  double search_x_;
  /**
   * Search space in y in meters, with respect to base_frame_.
   * Ignored if do_global_search_ is true.
   *
   * Parameter with default value 0.1.
   */
  double search_y_;
  /**
   * Search space in theta in radians, with respect to base_frame_.
   * Ignored if do_global_search_ is true.
   *
   * Parameter with default value 0.35.
   */
  double search_a_;
  /**
   * Resolution of the angular search space in radians.
   * Uses the angle increment of the laser.
   */
  double search_a_step_;
  /**
   * Minimum probability score for an estimate to be valid [0.0, 1.0].
   * Increasing the score will prune poor matches early
   * but too high a score will result in no suitable matches.
   *
   * Parameter with default value 0.5.
   */
  double min_score_;
  /**
   * Resolution in meters of the high-resolution probability grid.
   * Only used when raycasting, otherwise the map resolution is used.
   * Whichever is used, the low-resolution grid always has 10x the high resolution.
   *
   * Parameter with default value 0.03.
   */
  double high_grid_resolution_;
  /**
   * Whether to use a raycast from the initial position estimate
   * to generate probability grids.
   * Ignored if do_global_search is true.
   *
   * Raycasting was added because we are using a map rather than
   * a reference scan like in Olsen's paper. This limits the probability
   * grid resolution to the resolution of the map. Raycasting allows
   * higher resolution grids and maximizing probability at the edges of obstacles,
   * but is slow if the map is big (possible inefficient RealTimeCSM::overlapGrid
   * or ProbabilityGrid::updateGrid).
   *
   * Parameter with default value false.
   */
  bool enable_raycast_;
  /**
   * The minimum linear distance in meters
   * from the last raycasted pose to re-trigger a raycast.
   *
   * Parameter with default value 0.5.
   */
  double raycast_lin_thres_;
  /**
   * The minimum angular distance in radians
   * from the last raycasted pose to re-trigger a raycast.
   *
   * Parameter with default value 1.57.
   */
  double raycast_ang_thres_;
  /**
   * Disabled.
   * Due to a bug, the corrected scan is published with
   * the wrong transforms.
   */
  /* bool show_corrected_scan_; */
  /**
   * Whether to publish the corrected map_frame_ <- base_frame_
   * transform to /initialpose to reset AMCL.
   * @todo Should be throttled as constantly resetting AMCL does not
   * allow it to build its filter properly.
   *
   * Parameter with default value false.
   */
  bool publish_initial_pose_;
  /**
   * Whether to do a global search.
   * Sets search_x_ and search_y_ to the map size,
   * and search_a_ to 360 degrees.
   * Used for global localization.
   *
   * @todo Needs benchmarking. If the map is large, global search may be very slow.
   *
   * Parameter with default value false.
   */
  bool do_global_search_;
  /**
   * Whether to get the input map from the topic or the service.
   *
   * Parameter with default value true.
   */
  bool use_map_topic_;
  /**
   * If use_map_topic_ is true, whether to only process
   * the first map received.
   *
   * Parameter with default value true.
   */
  bool first_map_only_;
  /**
   * Whether to publish the last valid transform
   * when no new valid match is found.
   * When used together with odom as reference_frame_,
   * allows localization to continue using odometry
   * when there is no new estimate.
   *
   * Parameter with default value false.
   */
  bool publish_last_valid_tf_;
  /**
   * Whether to publish the generated grids.
   *
   * Parameter with default value false.
   */
  bool publish_tables_;
  /**
   * How long to wait for a transform to be available.
   * If the value needs to be increased significantly despite the
   * required transforms being published regularly,
   * it could be a sign of a slow processor, connection issues,
   * or timing issues between multiple processors.
   *
   * Parameter with default value 0.05.
   */
  double wait_for_transform_;
  /**
   * Whether to publish the correction of the map or base frame.
   * If true, transform published will be
   * reference_frame_ <- corrected map_frame_ (with name from matched_frame_)
   * else,
   * reference_frame_ <- corrected base_frame_ (with name from matched_frame_)
   *
   * Parameter with default value false.
   */
  bool invert_transform_;
  /**
   * The size of the output average window.
   * Set to < 1 to disable average filtering.
   *
   * Parameter with default value 1.
   */
  int filter_window_size_;
  /**
   * If true, uses the last pose estimate rather than the
   * map_frame_ <- base_frame_ transform as the next initial estimate.
   * The last pose estimate is reset whenever the matcher is disabled.
   *
   * Note: may cause estimates to get stuck in local maxima, but is
   * useful for cases where the initial estimate is not being corrected
   * or diverges (such as for rack docking)
   *
   * Parameter with default value false.
   */
  bool use_last_match_;
  /**
   * The minimum number of valid rays in a scan for the
   * scan to be considered for matching.
   *
   * Parameter with default value 100.
   */
  int min_valid_points_;

  /**
   * The last reference_frame_ <- matched_frame_ estimate.
   * Used for publish_last_valid_tf_ and use_last_match_
   */
  tf::Transform last_reference_matched_tf_;
  /**
   * Whether last_reference_matched_tf_ is valid.
   */
  bool last_tf_valid_;
  /**
   * Latest pose where raycast was performed.
   * Used with raycast_lin_thres_ and raycast_ang_thres_
   * to determine whether to re-trigger a raycast.
   */
  Pose last_raycast_pose_;
  /**
   * Whether a raycast has been performed at least once.
   */
  bool have_first_raycast_;

  /**
   * A Map object containing the size, origin, resolution
   * and cells of the received map.
   */
  Map *map_;
  /**
   * Low-resolution probability grid.
   * Has 10x resolution of the high-resolution grid.
   */
  ProbabilityGrid *low_res_table_;
  /**
   * High-resolution probability grid.
   * If enable_raycast_ is true, resolution is high_grid_resolution_,
   * otherwise it matches the resolution of the received map.
   */
  ProbabilityGrid *high_res_table_;

  void processMap(const nav_msgs::OccupancyGrid &msg);
  ProbabilityGrid *createGridFromMap(Map *map, double resolution);
  void resetCells(ProbabilityGrid *grid);
  void overlapGrid(Map *a, int i, int j, Map *b);
  nav_msgs::OccupancyGrid getOccupancyGrid(ProbabilityGrid *grid);

  std::priority_queue<Pose> match(std::priority_queue<Pose> &q, ProbabilityGrid *grid, double search_x, double search_y, double search_a, bool ignore_unknown, bool ignore_obstacles);

  laser_geometry::LaserProjection projector_;

  /**
   * The average filter window.
   */
  std::list<Pose> filter_window_;
  void addToWindow(const Pose& input);
  bool getFilteredResult(Pose& result);
  void clearWindow();

  void reset();

public:
  /**
   * Constructor.
   */
  RealTimeCSM();
  /**
   * Destructor.
   */
  ~RealTimeCSM()
  {
    if (laser_data_ != NULL)
      delete laser_data_;
    if (low_res_table_ != NULL)
      delete low_res_table_;
    if (high_res_table_ != NULL)
      delete high_res_table_;
    if (map_ != NULL)
      delete map_;
  }
};
} // namespace correlative_scan_matcher

#endif
