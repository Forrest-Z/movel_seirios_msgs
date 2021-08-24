/*!
 *  USAGE:
 *  1. Load service (/path_load/load)     : loads path from yaml file with path
 * name as input
 *  2. Cancel service (/path_load/cancel) : cancels and stops path following
 *  3. Pause service (/path_load/pause)   : pauses path following to be resumed
 * later
 *  4. Resume service (/path_load/resume) : resumes paused path following
 *  5. Display topic (/path_load/display) : shows current loaded path
 */

#include <path_recall/path_load_segments.h>
#include <ros/console.h>
#include <ros_utils/ros_utils.h>
#include <movel_hasp_vendor/license.h>

PathLoadSegments Loader;

bool loadParams(ros::NodeHandle &nh_private_) {
  ros_utils::ParamLoader loader(nh_private_);
  loader.get_required("yaml_path", Loader.yaml_path_);
  loader.get_required("max_plan_length", Loader.max_plan_length_);
  loader.get_required("update_min_dist", Loader.update_min_dist_);
  loader.get_required("look_ahead_dist", Loader.look_ahead_dist_);
  loader.get_required("look_ahead_angle", Loader.look_ahead_angle_);
  loader.get_required("skip_on_obstruction", Loader.skip_on_obstruction_);
  loader.get_required("update_time_interval", Loader.update_time_interval_);
  loader.get_required("obstruction_threshold", Loader.obstruction_threshold_);
  loader.get_required("clearing_timeout", Loader.clearing_timeout_);
  loader.get_optional("max_ping_count", Loader.max_ping_count_, 60);

  ros::NodeHandle nh("/");
  if (nh.hasParam("move_base/base_local_planner"))
  {
    std::string local_planner;
    nh.getParam("move_base/base_local_planner", local_planner);

    std::string planner_name, xy_tolerance_pname, yaw_tolerance_pname;
    if (local_planner == "teb_local_planner/TebLocalPlannerROS")
      planner_name = "TebLocalPlannerROS";
    else if (local_planner == "dwa_local_planner/DWAPlannerROS")
      planner_name = "DWAPlannerROS";

    if (planner_name.size() > 0)
    {
      xy_tolerance_pname = "move_base/" + planner_name + "/xy_goal_tolerance";
      yaw_tolerance_pname = "move_base/" + planner_name + "/yaw_goal_tolerance";

      ROS_INFO("move_base xy tolerance pname %s", xy_tolerance_pname.c_str());
      ROS_INFO("move_base yaw tolerance pname %s", yaw_tolerance_pname.c_str());

      if (nh.hasParam(xy_tolerance_pname))
        nh.getParam(xy_tolerance_pname, Loader.mb_xy_tolerance_);
      else
      {
        ROS_INFO("xy param name %s not valid", xy_tolerance_pname.c_str());
        return false;
      }

      if (nh.hasParam(yaw_tolerance_pname))
        nh.getParam(yaw_tolerance_pname, Loader.mb_yaw_tolerance_);
      else
      {
        ROS_INFO("yaw param name %s not valid", yaw_tolerance_pname.c_str());
        return false;
      }
    }
    else
    {
      Loader.mb_xy_tolerance_ = Loader.update_min_dist_;
      Loader.mb_yaw_tolerance_ = Loader.look_ahead_angle_;
    }
  }
  else
  {
    Loader.mb_xy_tolerance_ = Loader.update_min_dist_;
    Loader.mb_yaw_tolerance_ = Loader.look_ahead_angle_;
  }

  return loader.params_valid();
}

int main(int argc, char **argv) {
  #ifdef MOVEL_LICENSE                                                                                                    
  MovelLicense ml(7);                                                                                                   
    if (!ml.login())                                                                                                      
      return 1;                                                                                                           
  #endif

  std::string node_name_ = "path_load";
  ros::init(argc, argv, node_name_);
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_("~");

  ros::Time::waitForValid();
  if (!loadParams(nh_private_)) {
    ROS_FATAL("Error during parameter loading. Shutting down.");
    #ifdef MOVEL_LICENSE
    ml.logout();
    #endif
    return 0;
  }
  ROS_INFO("All parameters loaded. Launching.");

  tf2_ros::Buffer tf_buffer;
  Loader.tf_buffer_ = &tf_buffer;
  tf2_ros::TransformListener tf_ear(tf_buffer);

  Loader.path_load_pub_ =
      nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
  Loader.cancel_pub_ =
      nh_.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);
  Loader.display_pub_ = nh_private_.advertise<nav_msgs::Path>("display", 1);
  Loader.info_pub_ =
      nh_private_.advertise<path_recall::PathInfo>("path_info", 1);
  Loader.start_pub_ = nh_private_.advertise<std_msgs::Bool>("start", 1);
  Loader.obstruction_status_pub_ = nh_private_.advertise<movel_seirios_msgs::ObstructionStatus>("/obstruction_status", 1);

  Loader.plan_client_ = nh_.serviceClient<nav_msgs::GetPlan>(
      "/move_base/GlobalPlanner/make_plan");
  Loader.clear_costmaps_client_ = nh_.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
  Loader.reachable_plan_client_ = nh_.serviceClient<nav_msgs::GetPlan>("/make_reachable_plan");

  ros::Subscriber pose_sub_ =
      nh_.subscribe("/pose", 1, &PathLoadSegments::getPose, &Loader);
  ros::Subscriber costmap_sub_ =
      nh_.subscribe("/move_base/local_costmap/costmap", 1, &PathLoadSegments::getCostmap, &Loader);
  ros::Subscriber feedback_sub_ = nh_.subscribe(
      "/move_base/feedback", 1, &PathLoadSegments::onFeedback, &Loader);
  ros::Subscriber goal_sub_ =
      nh_.subscribe("/move_base/result", 1, &PathLoadSegments::onGoal, &Loader);
  ros::Subscriber ts_pause_status_sub_ = 
      nh_.subscribe("/task_supervisor/pause_status", 1, &PathLoadSegments::onPauseStatus, &Loader);

  ros::ServiceServer path_srv_ = nh_private_.advertiseService(
      "path_input", &PathLoadSegments::getPath, &Loader);
  ros::ServiceServer check_srv_ = nh_private_.advertiseService(
      "check", &PathLoadSegments::onCheck, &Loader);
  ros::ServiceServer load_srv_ =
      nh_private_.advertiseService("load", &PathLoadSegments::onLoad, &Loader);
  ros::ServiceServer pause_srv_ = nh_private_.advertiseService(
      "pause", &PathLoadSegments::onPause, &Loader);
  ros::ServiceServer resume_srv_ = nh_private_.advertiseService(
      "resume", &PathLoadSegments::onResume, &Loader);
  ros::ServiceServer cancel_srv_ = nh_private_.advertiseService(
      "cancel", &PathLoadSegments::onCancel, &Loader);

  ros::spin();
  #ifdef MOVEL_LICENSE
  ml.logout();
  #endif
  return 0;
}
