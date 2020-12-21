/*!
 *  USAGE:
 *  1. Load service (/path_load/load)     : loads path from yaml file with path name as input
 *  2. Cancel service (/path_load/cancel) : cancels and stops path following
 *  3. Pause service (/path_load/pause)   : pauses path following to be resumed later
 *  4. Resume service (/path_load/resume) : resumes paused path following
 *  5. Display topic (/path_load/display) : shows current loaded path
 */

#include <path_recall/path_load_segments.h>
#include <ros/console.h>
#include <ros_utils/ros_utils.h>

PathLoadSegments Loader;

bool loadParams(ros::NodeHandle& nh_private_)
{
  ros_utils::ParamLoader loader(nh_private_);
  loader.get_required("yaml_path", Loader.yaml_path);
  loader.get_required("max_plan_length", Loader.max_plan_length);
  loader.get_required("update_min_dist", Loader.update_min_dist);
  loader.get_required("look_ahead_dist", Loader.look_ahead_dist);
  loader.get_required("look_ahead_angle", Loader.look_ahead_angle);
  loader.get_required("update_time_interval", Loader.update_time_interval);
  return loader.params_valid();
}

int main(int argc, char** argv)
{
  std::string node_name_ = "path_load";
  ros::init(argc, argv, node_name_);
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_("~");

  ros::Time::waitForValid();
  if (!loadParams(nh_private_))
  {
    ROS_FATAL("Error during parameter loading. Shutting down.");
    return 0;
  }
  ROS_INFO("All parameters loaded. Launching.");

  Loader.path_load_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
  Loader.cancel_pub_ = nh_.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);
  Loader.display_pub_ = nh_private_.advertise<nav_msgs::Path>("display", 1);
  Loader.info_pub_ = nh_private_.advertise<path_recall::PathInfo>("path_info", 1);
  Loader.start_pub_ = nh_private_.advertise<std_msgs::Bool>("start", 1);

  Loader.plan_client_ = nh_.serviceClient<nav_msgs::GetPlan>("/move_base/GlobalPlanner/make_plan");

  ros::Subscriber pose_sub_ = nh_.subscribe("/pose", 1, &PathLoadSegments::getPose, &Loader);
  ros::Subscriber feedback_sub_ = nh_.subscribe("/move_base/feedback", 1, &PathLoadSegments::onFeedback, &Loader);
  ros::Subscriber goal_sub_ = nh_.subscribe("/move_base/result", 1, &PathLoadSegments::onGoal, &Loader);

  ros::ServiceServer path_srv_ = nh_private_.advertiseService("path_input", &PathLoadSegments::getPath, &Loader);
  ros::ServiceServer check_srv_ = nh_private_.advertiseService("check", &PathLoadSegments::onCheck, &Loader);
  ros::ServiceServer load_srv_ = nh_private_.advertiseService("load", &PathLoadSegments::onLoad, &Loader);
  ros::ServiceServer pause_srv_ = nh_private_.advertiseService("pause", &PathLoadSegments::onPause, &Loader);
  ros::ServiceServer resume_srv_ = nh_private_.advertiseService("resume", &PathLoadSegments::onResume, &Loader);
  ros::ServiceServer cancel_srv_ = nh_private_.advertiseService("cancel", &PathLoadSegments::onCancel, &Loader);

  ros::spin();
}
