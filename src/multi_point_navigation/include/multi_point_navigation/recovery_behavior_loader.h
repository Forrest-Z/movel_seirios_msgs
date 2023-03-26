#ifndef MULTI_POINT_NAVIGATION_RECOVERY_BEHAVIOR_LOADER_H
#define MULTI_POINT_NAVIGATION_RECOVERY_BEHAVIOR_LOADER_H

#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/recovery_behavior.h>
#include <ros/ros.h>

namespace multi_point_navigation
{
class RecoveryBehaviorLoader
{
private:
  pluginlib::ClassLoader<nav_core::RecoveryBehavior> recovery_loader_;

public:
  RecoveryBehaviorLoader();
  ~RecoveryBehaviorLoader();

  bool loadRecoveryBehaviors(const ros::NodeHandle& nh, tf2_ros::Buffer* tf_buffer,
                             costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap,
                             std::vector<boost::shared_ptr<nav_core::RecoveryBehavior>>& recovery_behaviors);

private:
  bool getRecoveryBehaviorListFromRosParam(const ros::NodeHandle& nh, XmlRpc::XmlRpcValue& behavior_list_out);

  bool validateRecoveryBehaviorList(const XmlRpc::XmlRpcValue& behavior_list);

  bool createRecoveryBehaviors(const XmlRpc::XmlRpcValue& behavior_list, tf2_ros::Buffer* tf_buffer,
                               costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap,
                               std::vector<boost::shared_ptr<nav_core::RecoveryBehavior>>& recovery_behaviors);
};

}  // namespace multi_point_navigation

#endif