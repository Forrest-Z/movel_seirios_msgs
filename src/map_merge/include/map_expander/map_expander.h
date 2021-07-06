#ifndef MAP_EXPANDER_H_
#define MAP_EXPANDER_H_

#include <ros_utils/ros_utils.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <atomic>
#include <boost/shared_ptr.hpp>
#include <mutex>

#include <combine_grids/merging_pipeline.h>

namespace map_expander
{

struct MapSource
{
  std::mutex mutex;

  nav_msgs::OccupancyGrid::Ptr writable_map;
  nav_msgs::OccupancyGrid::ConstPtr read_only_map;
  nav_msgs::MapMetaData map_info;

  ros::Subscriber map_sub;
};

struct DynamicMapSource : public MapSource
{
  geometry_msgs::Transform initial_pose;
  ros::Subscriber map_update_sub;
};

struct StaticMapSource : public MapSource
{
  ros::ServiceClient map_client;
};

class MapExpander
{
public:
  MapExpander();
  ~MapExpander();

  void mergeMap(nav_msgs::OccupancyGridPtr& merged_map);
  void setupTopics();

private:
  bool loadParams();

  void loadStaticMap();

  void initialRobotPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

  /* Applies for both dynamic and static maps */
  void fullMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg, MapSource& map);
  /* Only applies for dynamic map */
  void partialMapCallback(const map_msgs::OccupancyGridUpdate::ConstPtr& msg, DynamicMapSource& map);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Subscriber initial_robot_pose_sub_;

  ros::Publisher merged_map_publisher_;

  bool initial_robot_pose_acquired_;
  /**
   * === Some notes and assumptions about initial robot pose ===
   * Unless using ground truth pose in simulation:
   * - initial robot pose is relative to starting point of previous map
   * - initial robot pose always coincides with starting point of current/dynamic map
   * - Relative map origin (not always the starting point necessarily) of previous and current map are the same
   * Therefore, initial robot pose also acts as transform from previous map starting point to current map's.
   * By assuming that the relative origin of each map is the same, initial robot pose is also a transform from
   * the origin of previous map to current map's.
   */
  geometry_msgs::Transform initial_robot_pose_;
  geometry_msgs::Transform dynamic_map_relative_pose_;
  DynamicMapSource current_map_;
  StaticMapSource previous_map_;
  combine_grids::MergingPipeline pipeline_;

  /* Parameters */
  /**
   * Map merging and merged map publishing frequency
   */
  double merging_rate_;
  /**
   * Whether static/previous map is available
   */
  bool static_map_available_;
  /**
   * Whether static map is callable via service
   * (only relevant if static_map_available_ is true)
   */
  bool use_static_map_service_;
  /**
   * Service name to get static map
   * (only relevant if static_map_available_ == true && use_static_map_service_ == true)
   */
  std::string static_map_service_name_;
  /**
   * Frame id (in tf tree) to which merged map is assigned
   */
  std::string map_frame_;
};

}  // namespace map_expander

#endif // MAP_EXPANDER_H_
