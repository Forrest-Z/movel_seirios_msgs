#ifndef TASK_SUPERVISOR_MULTI_POINT_NAVIGATION_HANDLER_H
#define TASK_SUPERVISOR_MULTI_POINT_NAVIGATION_HANDLER_H

#include <ros/ros.h>

#include <task_supervisor/common.h>
#include <task_supervisor/plugins/task_handler.h>

#include <movel_seirios_msgs/Reports.h>
#include <movel_seirios_msgs/ObstructionStatus.h>

#include <boost/thread/mutex.hpp>
#include <std_srvs/Empty.h>

#include <cmath>
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/ColorRGBA.h"
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include "std_msgs/Bool.h"
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define coord_pair std::pair<float, float>


namespace task_supervisor
{
class MultiPointNavigationHandler : public TaskHandler
{
// private:
public:
  // ROS params
  float p_point_gen_dist_;
  float p_goal_tolerance_x_;
  float p_goal_tolerance_y_;
  float p_angular_vel_;
  float p_linear_vel_;
  bool p_spline_enable_;
  float p_obstruction_timeout_;
  /*
  double p_server_timeout_;
  bool p_static_paths_;
  std::string p_navigation_server_;
  double p_human_detection_min_score_;
  std::string p_human_detection_topic_;
  std::string p_enable_human_detection_msg_;
  std::string p_disable_human_detection_msg_;
  bool p_enable_best_effort_goal_;
  bool p_normal_nav_if_best_effort_unavailable_;
  double p_best_effort_retry_timeout_sec_;
  double p_best_effort_retry_sleep_sec_;
  */
  // variables
  // std::shared_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> nav_ac_ptr_;
  boost::mutex mtx_;
  // bool enable_human_detection_;
  // double human_detection_score_;
  bool task_cancelled_;
  geometry_msgs::Pose robot_pose_;
  bool isHealthy_;
  std::vector<std::vector<float>> coords_for_nav_;
  std::vector<std::vector<float>> coords_for_spline_;
  std::vector<int> points_to_spline_;
  float kp_ = -1.1, ki_ = 0, kd_ = -0.1;
  bool obstructed_;
  int bypass_degree_ = 3;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_ear_;
  std::shared_ptr<costmap_2d::Costmap2DROS> sync_costmap_ptr_;
  float min_obst_timeout_ = 4.0; 
  float obst_check_freq_ = 2.0;

  // topics/services
  /*
  ros::ServiceServer enable_human_detection_srv_;
  ros::ServiceServer enable_best_effort_goal_srv_;
  ros::ServiceClient make_movebase_plan_client_;
  ros::ServiceClient make_reachable_plan_client_;   // planner_utils
  ros::Subscriber human_detection_sub_;
  
  ros::Subscriber loc_report_sub_;
  ros::Publisher movebase_cancel_pub_;
  ros::Publisher obstruction_status_pub_;
  */
  ros::Subscriber robot_pose_sub_;
  ros::Publisher major_marker_pub_;
  ros::Publisher minor_marker_pub_;
  ros::Publisher smooth_marker_pub_;
  ros::Publisher current_marker_pub_;
  ros::Publisher cmd_vel_pub_;
  ros::Subscriber obstacle_sub_;

  template <typename param_type>
  bool load_param_util(std::string param_name, param_type& output);
  bool loadParams();
  // bool start_ActionClient();

  /////////////////////////////
  void robotPoseCB(const geometry_msgs::Pose::ConstPtr& );
  bool navToPoint(int);
  void pointsGen(std::vector<std::vector<float>> );
  void showAllPoints(std::vector<std::vector<float>>);
  void showCurrentGoal(int);
  float pidFn(float, float);
  void obstacleCB(const std_msgs::Bool::ConstPtr& );
  void splinePoints();
  coord_pair midPoint(coord_pair, coord_pair);
  std::vector<float> intersectPoint(coord_pair, coord_pair, coord_pair, coord_pair);
  bool getPointsToSpline(std::vector<std::vector<float>>, std::vector<int>);
  bool obstacleCheck(int );
  /////////////////////////////
  

public:
   
   MultiPointNavigationHandler();
  ~MultiPointNavigationHandler(){};

   /**
     * @brief Method called by task_supervisor when a navigation task is received
     * @param task Relevant task passed to handler by task supervisor
     * @param error_message Error message returned by this handler if execution fails
     * @return ReturnCode which indicates failure, cancellation or success
     */
  virtual ReturnCode runTask(movel_seirios_msgs::Task& task, std::string& error_message);


  bool setupHandler();
  void cancelTask();
  
};

}  // namespace task_supervisor

#endif