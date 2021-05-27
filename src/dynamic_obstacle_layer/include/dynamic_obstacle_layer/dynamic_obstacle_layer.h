#ifndef DYNAMIC_OBSTACLE_LAYER_H_
#define DYNAMIC_OBSTACLE_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <dynamic_obstacle_layer/DynamicPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>

namespace dynamic_obstacle_layer
{

class DynamicLayer : public costmap_2d::Layer, public costmap_2d::Costmap2D
{
public:
  DynamicLayer();

  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                             double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
  bool isDiscretized()
  {
    return true;
  }

  virtual void matchSize();

protected:
  double map_tolerance_;
  double footprint_radius_;
  double range_;

private:
  geometry_msgs::Pose pose_;
  bool has_pose_;
  bool map_received_;

  // ROS subscribers
  ros::Subscriber map_sub_, pose_sub_;
  void getPose(const geometry_msgs::PoseConstPtr& pose);
  void incomingMap(const nav_msgs::OccupancyGridConstPtr& new_map);

  // Interpret values in map  
  unsigned char interpretValue(unsigned char value);
  
  // Inflate costmap obstacle with footprint radius
  void markFootprint(costmap_2d::Costmap2D& master_grid, unsigned int mark_i, unsigned int mark_j);
  
  // Inflate obstacles in map to filter out static obstacles
  void inflateMap(unsigned int mark_i, unsigned int mark_j);
  
  // Get offsets from a point in costmap based on given radius
  void getOffsets(double radius, std::vector< std::vector<int> > &offsets);

  bool track_unknown_space_;
  bool first_map_only_;      ///< @brief Store the first static map and reuse it on reinitializing
  bool trinary_costmap_;
  unsigned char lethal_threshold_, unknown_cost_value_;

  void reconfigureCB(dynamic_obstacle_layer::DynamicPluginConfig &config, uint32_t level);
  dynamic_reconfigure::Server<dynamic_obstacle_layer::DynamicPluginConfig> *dsrv_;
};
}
#endif
