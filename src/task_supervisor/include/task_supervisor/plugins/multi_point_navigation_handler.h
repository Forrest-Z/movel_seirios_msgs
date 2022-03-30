#ifndef TASK_SUPERVISOR_MULTI_POINT_NAVIGATION_HANDLER_H
#define TASK_SUPERVISOR_MULTI_POINT_NAVIGATION_HANDLER_H

#include <ros/ros.h>

#include <task_supervisor/common.h>
#include <task_supervisor/plugins/task_handler.h>

#include <movel_seirios_msgs/Reports.h>
#include <movel_seirios_msgs/ObstructionStatus.h>
#include <movel_seirios_msgs/MultipointPath.h>
#include <movel_seirios_msgs/MultipointProgress.h>

#include <boost/thread/mutex.hpp>
#include <std_srvs/Empty.h>

#include <cmath>
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/ColorRGBA.h"
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include "std_msgs/Bool.h"
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <dynamic_reconfigure/server.h>
#include <multi_point/MultipointConfig.h>

#define co_ord_pair std::pair<float, float>


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
  float p_goal_tolerance_ = 0.1;
  float p_angular_tolerance_;
  bool p_spline_enable_;
  float p_obstruction_timeout_;
  float p_kp_, p_ki_, p_kd_;
  bool p_forward_only_ = true;
  float p_angular_acc_, p_linear_acc_;
  int p_bypass_degree_ = 3;
  float p_curve_vel_ = 0.1;

  // variables
  boost::mutex mtx_;
  bool task_cancelled_;
  geometry_msgs::Pose robot_pose_;
  bool isHealthy_;
  std::vector<int> major_indices_;
  std::vector<std::vector<float>> rcvd_multi_coords_;
  std::vector<std::vector<float>> coords_for_nav_;
  bool obstructed_;
  int bypass_degree_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_ear_;
  std::shared_ptr<costmap_2d::Costmap2DROS> costmap_ptr_;
  float min_obst_timeout_ = 4.0; 
  float obst_check_interval_ = 2.0;
  float angular_tolerance_ = 0.1;
  const float min_angular_vel_ = 0.05, min_linear_vel_ = 0.05;
  const float max_angular_vel_ = 1.0, max_linear_vel_ = 1.0;
  float angular_vel_;
  float linear_vel_;
  int look_ahead_points_ = 2;
  bool at_start_point_ = false;
  bool start_at_nearest_point_ = false;

  dynamic_reconfigure::Server<multi_point::MultipointConfig> dynamic_reconf_server_;
  dynamic_reconfigure::Server<multi_point::MultipointConfig>::CallbackType dynamic_reconfigure_callback_;

  // topics/services
  ros::Subscriber robot_pose_sub_;
  ros::Publisher path_visualize_pub_;
  ros::Publisher cmd_vel_pub_;
  ros::Publisher current_goal_pub_;
  ros::ServiceServer path_srv_;

  template <typename param_type>
  bool load_param_util(std::string param_name, param_type& output);
  bool loadParams();

  // Generating Path
  bool pointsGen(std::vector<std::vector<float>>, std::vector<std::vector<float>>& ,bool );
  void splinePoints(std::vector<std::vector<float>>&, std::vector<std::vector<int>>, std::vector<std::vector<float>>& );
  bool getPointsToSpline(std::vector<std::vector<float>>, std::vector<int>, std::vector<std::vector<int>>&);
  co_ord_pair midPoint(co_ord_pair, co_ord_pair);
  std::vector<float> intersectPoint(co_ord_pair, co_ord_pair, co_ord_pair, co_ord_pair);

  // Visualize, topics, service and config
  void visualizePath(int, bool);
  void printGeneratedPath(std::vector<std::vector<float>>);
  void publishCurrentGoal(int );
  bool pathServiceCb(movel_seirios_msgs::MultipointPath::Request&, movel_seirios_msgs::MultipointPath::Response& );
  void robotPoseCB(const geometry_msgs::Pose::ConstPtr& );
  void reconfCB(multi_point::MultipointConfig&, uint32_t );

  // Navigation
  bool navToPoint(int);
  float pidFn(float, float);
  bool obstacleCheck(int );
  float linAccelerationCheck(float );
  float angAccelerationCheck(float );
  void stopRobot();
  

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