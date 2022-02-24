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
  float p_look_ahead_dist_;
  float p_obst_check_freq_;
  float p_goal_tolerance_x_;
  float p_goal_tolerance_y_;
  bool p_spline_enable_;
  float p_obstruction_timeout_;
  float p_kp_, p_ki_, p_kd_;

  // variables
  boost::mutex mtx_;
  bool task_cancelled_;
  geometry_msgs::Pose robot_pose_;
  bool isHealthy_;
  std::vector<std::vector<float>> coords_for_nav_;
  std::vector<std::vector<float>> coords_for_spline_;
  std::vector<int> points_to_spline_;
  // float kp_ = -1.1, ki_ = 0, kd_ = -0.1;
  bool obstructed_;
  int bypass_degree_ = 3;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_ear_;
  std::shared_ptr<costmap_2d::Costmap2DROS> costmap_ptr_;
  float min_obst_timeout_ = 4.0; 
  float obst_check_interval_ = 2.0;

  const float min_angular_vel_ = 0.3, min_linear_vel_ = 0.1;
  const float max_angular_vel_ = 1.0, max_linear_vel_ = 1.0;
  float angular_vel_;
  float linear_vel_;

  int look_ahead_points_ = 2;

  // topics/services
  ros::Subscriber robot_pose_sub_;
  ros::Publisher major_marker_pub_;
  ros::Publisher minor_marker_pub_;
  ros::Publisher smooth_marker_pub_;
  ros::Publisher current_marker_pub_;
  ros::Publisher cmd_vel_pub_;
  ros::ServiceClient clear_costmap_client_;

  template <typename param_type>
  bool load_param_util(std::string param_name, param_type& output);
  bool loadParams();

  /////////////////////////////
  void robotPoseCB(const geometry_msgs::Pose::ConstPtr& );
  bool navToPoint(int);
  bool pointsGen(std::vector<std::vector<float>> );
  void showAllPoints(std::vector<std::vector<float>>);
  void showCurrentGoal(int);
  float pidFn(float, float);
  void splinePoints();
  coord_pair midPoint(coord_pair, coord_pair);
  std::vector<float> intersectPoint(coord_pair, coord_pair, coord_pair, coord_pair);
  bool getPointsToSpline(std::vector<std::vector<float>>, std::vector<int>);
  bool obstacleCheck(int );
  bool clearCostmapFn();
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