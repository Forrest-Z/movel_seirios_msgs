#include <movel_move_base/move_base.h>
#include <move_base_msgs/RecoveryStatus.h>
#include <geometry_msgs/Twist.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>
#include <cmath>


namespace move_base {

void MoveBase::reconfigureCB(movel_move_base::MoveBaseConfig &config, uint32_t level)
{
  boost::recursive_mutex::scoped_lock l(configuration_mutex_);

  //The first time we're called, we just want to make sure we have the
  //original configuration
  if(!setup_)
  {
    last_config_ = config;
    default_config_ = config;
    setup_ = true;
    return;
  }

  if(config.restore_defaults) {
    config = default_config_;
    //if someone sets restore defaults on the parameter server, prevent looping
    config.restore_defaults = false;
  }

  if(planner_frequency_ != config.planner_frequency)
  {
    planner_frequency_ = config.planner_frequency;
    p_freq_change_ = true;
  }

  if(controller_frequency_ != config.controller_frequency)
  {
    controller_frequency_ = config.controller_frequency;
    c_freq_change_ = true;
  }

  planner_patience_ = config.planner_patience;
  controller_patience_ = config.controller_patience;
  max_planning_retries_ = config.max_planning_retries;
  conservative_reset_dist_ = config.conservative_reset_dist;

  recovery_behavior_enabled_ = config.recovery_behavior_enabled;
  clearing_rotation_allowed_ = config.clearing_rotation_allowed;
  shutdown_costmaps_ = config.shutdown_costmaps;

  oscillation_timeout_ = config.oscillation_timeout;
  oscillation_distance_ = config.oscillation_distance;
  if(config.base_global_planner != last_config_.base_global_planner) {
    boost::shared_ptr<nav_core::BaseGlobalPlanner> old_planner = planner_;
    //initialize the global planner
    ROS_INFO("Loading global planner %s", config.base_global_planner.c_str());
    try {
      planner_ = bgp_loader_.createInstance(config.base_global_planner);

      // wait for the current planner to finish planning
      boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);

      // Clean up before initializing the new planner
      planner_plan_->clear();
      latest_plan_->clear();
      controller_plan_->clear();
      resetState();
      planner_->initialize(bgp_loader_.getName(config.base_global_planner), planner_costmap_ros_);

      lock.unlock();
    } catch (const pluginlib::PluginlibException& ex) {
      ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the \
                  containing library is built? Exception: %s", config.base_global_planner.c_str(), ex.what());
      planner_ = old_planner;
      config.base_global_planner = last_config_.base_global_planner;
    }
  }

  if(config.base_local_planner != last_config_.base_local_planner){
    boost::shared_ptr<nav_core::BaseLocalPlanner> old_planner = tc_;
    //create a local planner
    try {
      tc_ = blp_loader_.createInstance(config.base_local_planner);
      // Clean up before initializing the new planner
      planner_plan_->clear();
      latest_plan_->clear();
      controller_plan_->clear();
      resetState();
      tc_->initialize(blp_loader_.getName(config.base_local_planner), &tf_, controller_costmap_ros_);
    } catch (const pluginlib::PluginlibException& ex) {
      ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the \
                  containing library is built? Exception: %s", config.base_local_planner.c_str(), ex.what());
      tc_ = old_planner;
      config.base_local_planner = last_config_.base_local_planner;
    }
  }

  make_plan_clear_costmap_ = config.make_plan_clear_costmap;
  make_plan_add_unreachable_goal_ = config.make_plan_add_unreachable_goal;

  last_config_ = config;
}


void MoveBase::publishZeroVelocity()
{
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = 0.0;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = 0.0;
  vel_pub_.publish(cmd_vel);
}


bool MoveBase::isQuaternionValid(const geometry_msgs::Quaternion& q)
{
  //first we need to check if the quaternion has nan's or infs
  if(!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w)){
    ROS_ERROR("Quaternion has nans or infs... discarding as a navigation goal");
    return false;
  }

  tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);

  //next, we need to check if the length of the quaternion is close to zero
  if(tf_q.length2() < 1e-6){
    ROS_ERROR("Quaternion has length close to zero... discarding as navigation goal");
    return false;
  }

  //next, we'll normalize the quaternion and check that it transforms the vertical vector correctly
  tf_q.normalize();

  tf2::Vector3 up(0, 0, 1);

  double dot = up.dot(up.rotate(tf_q.getAxis(), tf_q.getAngle()));

  if(fabs(dot - 1) > 1e-3){
    ROS_ERROR("Quaternion is invalid... for navigation the z-axis of the quaternion must be close to vertical.");
    return false;
  }

  return true;
}

geometry_msgs::PoseStamped MoveBase::goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg)
{
  std::string global_frame = planner_costmap_ros_->getGlobalFrameID();
  geometry_msgs::PoseStamped goal_pose, global_pose;
  goal_pose = goal_pose_msg;

  //just get the latest available transform... for accuracy they should send
  //goals in the frame of the planner
  goal_pose.header.stamp = ros::Time();

  try{
    tf_.transform(goal_pose_msg, global_pose, global_frame);
  }
  catch(tf2::TransformException& ex){
    ROS_WARN("Failed to transform the goal pose from %s into the %s frame: %s",
        goal_pose.header.frame_id.c_str(), global_frame.c_str(), ex.what());
    return goal_pose_msg;
  }

  return global_pose;
}


double MoveBase::distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2)
{
  return hypot(p1.pose.position.x - p2.pose.position.x, p1.pose.position.y - p2.pose.position.y);
}


bool MoveBase::loadRecoveryBehaviors(ros::NodeHandle node)
{
  XmlRpc::XmlRpcValue behavior_list;
  if(node.getParam("recovery_behaviors", behavior_list)){
    if(behavior_list.getType() == XmlRpc::XmlRpcValue::TypeArray){
      for(int i = 0; i < behavior_list.size(); ++i){
        if(behavior_list[i].getType() == XmlRpc::XmlRpcValue::TypeStruct){
          if(behavior_list[i].hasMember("name") && behavior_list[i].hasMember("type")){
            //check for recovery behaviors with the same name
            for(int j = i + 1; j < behavior_list.size(); j++){
              if(behavior_list[j].getType() == XmlRpc::XmlRpcValue::TypeStruct){
                if(behavior_list[j].hasMember("name") && behavior_list[j].hasMember("type")){
                  std::string name_i = behavior_list[i]["name"];
                  std::string name_j = behavior_list[j]["name"];
                  if(name_i == name_j){
                    ROS_ERROR("A recovery behavior with the name %s already exists, this is not allowed. Using the default recovery behaviors instead.",
                        name_i.c_str());
                    return false;
                  }
                }
              }
            }
          }
          else{
            ROS_ERROR("Recovery behaviors must have a name and a type and this does not. Using the default recovery behaviors instead.");
            return false;
          }
        }
        else{
          ROS_ERROR("Recovery behaviors must be specified as maps, but they are XmlRpcType %d. We'll use the default recovery behaviors instead.",
              behavior_list[i].getType());
          return false;
        }
      }

      //if we've made it to this point, we know that the list is legal so we'll create all the recovery behaviors
      for(int i = 0; i < behavior_list.size(); ++i){
        try{
          //check if a non fully qualified name has potentially been passed in
          if(!recovery_loader_.isClassAvailable(behavior_list[i]["type"])){
            std::vector<std::string> classes = recovery_loader_.getDeclaredClasses();
            for(unsigned int i = 0; i < classes.size(); ++i){
              if(behavior_list[i]["type"] == recovery_loader_.getName(classes[i])){
                //if we've found a match... we'll get the fully qualified name and break out of the loop
                ROS_WARN("Recovery behavior specifications should now include the package name. You are using a deprecated API. Please switch from %s to %s in your yaml file.",
                    std::string(behavior_list[i]["type"]).c_str(), classes[i].c_str());
                behavior_list[i]["type"] = classes[i];
                break;
              }
            }
          }

          boost::shared_ptr<nav_core::RecoveryBehavior> behavior(recovery_loader_.createInstance(behavior_list[i]["type"]));

          //shouldn't be possible, but it won't hurt to check
          if(behavior.get() == NULL){
            ROS_ERROR("The ClassLoader returned a null pointer without throwing an exception. This should not happen");
            return false;
          }

          //initialize the recovery behavior with its name
          behavior->initialize(behavior_list[i]["name"], &tf_, planner_costmap_ros_, controller_costmap_ros_);
          recovery_behavior_names_.push_back(behavior_list[i]["name"]);
          recovery_behaviors_.push_back(behavior);
        }
        catch(pluginlib::PluginlibException& ex){
          ROS_ERROR("Failed to load a plugin. Using default recovery behaviors. Error: %s", ex.what());
          return false;
        }
      }
    }
    else{
      ROS_ERROR("The recovery behavior specification must be a list, but is of XmlRpcType %d. We'll use the default recovery behaviors instead.",
          behavior_list.getType());
      return false;
    }
  }
  else{
    //if no recovery_behaviors are specified, we'll just load the defaults
    return false;
  }

  //if we've made it here... we've constructed a recovery behavior list successfully
  return true;
}


//we'll load our default recovery behaviors here
void MoveBase::loadDefaultRecoveryBehaviors()
{
  recovery_behaviors_.clear();
  try{
    //we need to set some parameters based on what's been passed in to us to maintain backwards compatibility
    ros::NodeHandle n("~");
    n.setParam("conservative_reset/reset_distance", conservative_reset_dist_);
    n.setParam("aggressive_reset/reset_distance", circumscribed_radius_ * 4);

    //first, we'll load a recovery behavior to clear the costmap
    boost::shared_ptr<nav_core::RecoveryBehavior> cons_clear(recovery_loader_.createInstance("clear_costmap_recovery/ClearCostmapRecovery"));
    cons_clear->initialize("conservative_reset", &tf_, planner_costmap_ros_, controller_costmap_ros_);
    recovery_behavior_names_.push_back("conservative_reset");
    recovery_behaviors_.push_back(cons_clear);

    //next, we'll load a recovery behavior to rotate in place
    boost::shared_ptr<nav_core::RecoveryBehavior> rotate(recovery_loader_.createInstance("rotate_recovery/RotateRecovery"));
    if(clearing_rotation_allowed_){
      rotate->initialize("rotate_recovery", &tf_, planner_costmap_ros_, controller_costmap_ros_);
      recovery_behavior_names_.push_back("rotate_recovery");
      recovery_behaviors_.push_back(rotate);
    }

    //next, we'll load a recovery behavior that will do an aggressive reset of the costmap
    boost::shared_ptr<nav_core::RecoveryBehavior> ags_clear(recovery_loader_.createInstance("clear_costmap_recovery/ClearCostmapRecovery"));
    ags_clear->initialize("aggressive_reset", &tf_, planner_costmap_ros_, controller_costmap_ros_);
    recovery_behavior_names_.push_back("aggressive_reset");
    recovery_behaviors_.push_back(ags_clear);

    //we'll rotate in-place one more time
    if(clearing_rotation_allowed_){
      recovery_behaviors_.push_back(rotate);
      recovery_behavior_names_.push_back("rotate_recovery");
    }
  }
  catch(pluginlib::PluginlibException& ex){
    ROS_FATAL("Failed to load a plugin. This should not happen on default recovery behaviors. Error: %s", ex.what());
  }

  return;
}


bool MoveBase::getRobotPose(geometry_msgs::PoseStamped& global_pose, costmap_2d::Costmap2DROS* costmap)
{
  tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
  geometry_msgs::PoseStamped robot_pose;
  tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);
  robot_pose.header.frame_id = robot_base_frame_;
  robot_pose.header.stamp = ros::Time(); // latest available
  ros::Time current_time = ros::Time::now();  // save time for checking tf delay later

  // get robot pose on the given costmap frame
  try
  {
    tf_.transform(robot_pose, global_pose, costmap->getGlobalFrameID());
  }
  catch (tf2::LookupException& ex)
  {
    ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
    return false;
  }
  catch (tf2::ConnectivityException& ex)
  {
    ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n", ex.what());
    return false;
  }
  catch (tf2::ExtrapolationException& ex)
  {
    ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n", ex.what());
    return false;
  }

  // check if global_pose time stamp is within costmap transform tolerance
  if (!global_pose.header.stamp.isZero() &&
      current_time.toSec() - global_pose.header.stamp.toSec() > costmap->getTransformTolerance())
  {
    ROS_WARN_THROTTLE(1.0, "Transform timeout for %s. " \
                      "Current time: %.4f, pose stamp: %.4f, tolerance: %.4f", costmap->getName().c_str(),
                      current_time.toSec(), global_pose.header.stamp.toSec(), costmap->getTransformTolerance());
    return false;
  }

  return true;
}

};   // namespace