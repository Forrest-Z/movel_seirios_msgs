/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *********************************************************************/
#ifndef NAV_MOVE_BASE_ACTION_H_
#define NAV_MOVE_BASE_ACTION_H_

#include <vector>
#include <string>

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <nav_core/base_local_planner.h>
#include <nav_core/base_global_planner.h>
#include <nav_core/recovery_behavior.h>
#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_msgs/GetPlan.h>

#include <pluginlib/class_loader.hpp>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <movel_seirios_msgs/ObstructionStatus.h>

#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include "movel_move_base/MoveBaseConfig.h"
#include "movel_move_base/plan_inspector.h"

#include <sw/redis++/redis++.h>

namespace move_base
{
// typedefs to help us out with the action server so that we don't hace to type so much
typedef actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> MoveBaseActionServer;

enum MoveBaseState
{
  PLANNING,
  CONTROLLING,
  CLEARING,
  FORCING_FAILURE, // to abort and prevent best effort navigation
};

enum RecoveryTrigger
{
  PLANNING_R,
  CONTROLLING_R,
  OSCILLATION_R
};

/**
 * @class MoveBase
 * @brief A class that uses the actionlib::ActionServer interface that moves the robot base to a goal location.
 */
class MoveBase
{
public:
  /**
   * @brief  Constructor for the actions
   * @param name The name of the action
   * @param tf A reference to a TransformListener
   */
  MoveBase(tf2_ros::Buffer& tf);

  /**
   * @brief  Destructor - Cleans up
   */
  virtual ~MoveBase();

  /**
   * @brief  Performs a control cycle
   * @param goal A reference to the goal to pursue
   * @return True if processing of the goal is done, false otherwise
   */
  bool executeCycle(geometry_msgs::PoseStamped& goal);

private:
  /**
   * @brief  A service call that clears the costmaps of obstacles
   * @param req The service request
   * @param resp The service response
   * @return True if the service call succeeds, false otherwise
   */
  bool clearCostmapsService(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp);

  /**
   * @brief  A service call to enable/disable stop at obstacle. 
   *        this function will be called by plan_inspector node
   * @param req The service request
   * @param resp The service response
   * @return True if the service call succeeds, false otherwise
   */

  bool stopObstacleService(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
  
  /**
   * @brief  A service call to enable/disable stop at obstacle. Only used if MOVEL_MOVE_BASE is true, for communicating to other nodes and UI.
   *        The function is exactly same with stopObstacleService.
   * @param req The service request
   * @param resp The service response
   * @return True if the service call succeeds, false otherwise
   */

  bool enablePlanInspector(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

  /**
   * @brief  A service call that can be made when the action is inactive that will return a plan
   * @param  req The goal request
   * @param  resp The plan request
   * @return True if planning succeeded, false otherwise
   */
  bool planService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp);

  /**
   * @brief  Make a new global plan
   * @param  goal The goal to plan to
   * @param  plan Will be filled in with the plan made by the planner
   * @return  True if planning succeeds, false otherwise
   */
  bool makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

  /**
   * @brief  Load the recovery behaviors for the navigation stack from the parameter server
   * @param node The ros::NodeHandle to be used for loading parameters
   * @return True if the recovery behaviors were loaded successfully, false otherwise
   */
  bool loadRecoveryBehaviors(ros::NodeHandle node);

  /**
   * @brief  Loads the default recovery behaviors for the navigation stack
   */
  void loadDefaultRecoveryBehaviors();

  /**
   * @brief  Clears obstacles within a window around the robot
   * @param size_x The x size of the window
   * @param size_y The y size of the window
   */
  void clearCostmapWindows(double size_x, double size_y);

  /**
   * @brief  Publishes a velocity command of zero to the base
   */
  void publishZeroVelocity();

  /**
   * @brief  Reset the state of the move_base action and send a zero velocity command to the base
   */
  void resetState();

  void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal);

  void planThread();

  void executeCb(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal);

  bool isQuaternionValid(const geometry_msgs::Quaternion& q);

  bool getRobotPose(geometry_msgs::PoseStamped& global_pose, costmap_2d::Costmap2DROS* costmap);

  double distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2);

  geometry_msgs::PoseStamped goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg);

  /**
   * @brief This is used to wake the planner at periodic intervals.
   */
  void wakePlanner(const ros::TimerEvent& event);

  /**
   * @brief This is used to initialize the global variables / parameters using redis
   */
  void redisInit();

  /**
   * @brief This is used to reconfigure the parameters or revert it back based on the stop at obstacle state
   */
  void reconfigureParams(bool stop_at_obstacle_state);

  /**
   * @brief This is used to check whether stop_at_obstacle is enabled or not
   */
  bool onStopObstacleCheck(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

  /**
   * @brief This is used to save params for dynamic reconfigure
   */
  void saveParams();

  tf2_ros::Buffer& tf_;

  MoveBaseActionServer* as_;

  boost::shared_ptr<nav_core::BaseLocalPlanner> tc_;
  costmap_2d::Costmap2DROS *planner_costmap_ros_, *controller_costmap_ros_;

  boost::shared_ptr<nav_core::BaseGlobalPlanner> planner_;
  std::string robot_base_frame_, global_frame_;

  std::vector<boost::shared_ptr<nav_core::RecoveryBehavior> > recovery_behaviors_;
  std::vector<std::string> recovery_behavior_names_;
  unsigned int recovery_index_;

  // bookkeeping
  geometry_msgs::PoseStamped global_pose_;
  double planner_frequency_, controller_frequency_, inscribed_radius_, circumscribed_radius_;
  double planner_patience_, controller_patience_;
  int32_t max_planning_retries_;
  uint32_t planning_retries_;
  double conservative_reset_dist_, clearing_radius_;
  ros::Publisher current_goal_pub_, vel_pub_, action_goal_pub_, recovery_status_pub_;
  ros::Publisher obstruction_status_pub_;
  ros::Subscriber goal_sub_;
  ros::ServiceServer make_plan_srv_, clear_costmaps_srv_, stop_obstacle_srv_, plan_inspector_srv_;
  bool received_new_goal_, new_goal_plan_initialized_;
  bool shutdown_costmaps_, clearing_rotation_allowed_, recovery_behavior_enabled_;
  bool make_plan_clear_costmap_, make_plan_add_unreachable_goal_;
  double oscillation_timeout_, oscillation_distance_;
  bool plan_obstructed_, has_valid_control_, stop_caused_by_obstacle_;

  ros::ServiceClient set_teb_params_,set_pebble_params_, set_move_base_param_;
  double weight_obstacle_temp_;
  bool use_teb_;
  bool use_pebble_;
  bool use_obstacle_pebble_;

  MoveBaseState state_;
  RecoveryTrigger recovery_trigger_;

  ros::Time last_valid_plan_, last_valid_control_, last_oscillation_reset_;
  geometry_msgs::Twist last_valid_cmd_vel_;
  geometry_msgs::PoseStamped oscillation_pose_;
  pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader_;
  pluginlib::ClassLoader<nav_core::BaseLocalPlanner> blp_loader_;
  pluginlib::ClassLoader<nav_core::RecoveryBehavior> recovery_loader_;

  // set up plan triple buffer
  std::vector<geometry_msgs::PoseStamped>* planner_plan_;
  std::vector<geometry_msgs::PoseStamped>* latest_plan_;
  std::vector<geometry_msgs::PoseStamped>* controller_plan_;

  // set up the planner's thread
  bool runPlanner_;
  boost::recursive_mutex planner_mutex_;
  boost::condition_variable_any planner_cond_;
  geometry_msgs::PoseStamped planner_goal_;
  boost::thread* planner_thread_;

  boost::recursive_mutex configuration_mutex_;
  dynamic_reconfigure::Server<movel_move_base::MoveBaseConfig>* dsrv_;

  void reconfigureCB(movel_move_base::MoveBaseConfig& config, uint32_t level);

  movel_move_base::MoveBaseConfig last_config_;
  movel_move_base::MoveBaseConfig default_config_;
  bool setup_, p_freq_change_, c_freq_change_;
  bool new_global_plan_;

  // movel_move_base specific params
  bool stop_at_obstacle_;
  double stop_at_obstacle_distance_;
  double clearing_timeout_;
  bool allow_partial_blockage_replan_;
  bool allow_replan_after_timeout_;
  bool allow_recovery_during_timeout_;

  // temp
  double planner_frequency_temp_;
  int max_planning_retries_temp_;
  bool recovery_behavior_enabled_temp_;
  bool clearing_rotation_allowed_temp_;
  double oscillation_timeout_temp_;

  // plan inspector
  PlanInspector* plan_inspector_;
  int obstruction_threshold_;
  double partial_blockage_path_length_threshold_;

  ros::ServiceServer stop_obstacle_checker_;
  // redis
  sw::redis::ConnectionOptions opts_;
  std::string redis_host_;
  std::string redis_port_;
  int socket_timeout_;
  std::ostringstream redis_conn_;
  // redis key
  std::string redis_stop_at_obstacle_lim_key_;
};
};  // namespace move_base

#endif