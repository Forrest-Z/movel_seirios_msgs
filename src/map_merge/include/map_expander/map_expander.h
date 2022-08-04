#ifndef MAP_EXPANDER_H_
#define MAP_EXPANDER_H_

#include <ros_utils/ros_utils.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
// #include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <atomic>
#include <boost/shared_ptr.hpp>
#include <mutex>
// #include <memory>

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

  bool mergeMap(nav_msgs::OccupancyGridPtr& merged_map);
  void setupTopics();

private:
  bool loadParams();

  void loadStaticMap();

  /* Applies for both dynamic and static maps */
  void fullMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg, MapSource& map);
  /* Only applies for dynamic map */
  void partialMapCallback(const map_msgs::OccupancyGridUpdate::ConstPtr& msg, DynamicMapSource& map);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Publisher merged_map_publisher_;

  DynamicMapSource current_map_;
  StaticMapSource previous_map_;
  combine_grids::MergingPipeline pipeline_;

  // std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  // std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  // uint32_t prev_width_;
  // uint32_t prev_height_;
  // geometry_msgs::Pose prev_origin_;

  /* Parameters */
  /**
   * Map merging and merged map publishing frequency
   */
  double merging_rate_;
  /**
   * Whether there is static/previous map to be merged with the dynamic/current map
   * (false means robot is mapping for the 1st time)
   */
  bool previous_map_available_;
  /**
   * Whether previous map is callable via service
   * (only relevant if previous_map_available_ is true)
   */
  bool previous_map_service_available_;
  /**
   * Service name to get previous map
   * (only relevant if previous_map_available_ == true && previous_map_service_available_ == true)
   */
  std::string previous_map_service_name_;
  /**
   * Frame id (in tf tree) to which merged map is assigned
   */
  std::string map_frame_;
};

}  // namespace map_expander

#endif // MAP_EXPANDER_H_
