#ifndef PURE_PURSUIT_PLANNER_H
#define PURE_PURSUIT_PLANNER_H


#include <ros/ros.h>
#include <Eigen/Core>
#include <mutex>

#include <nav_msgs/Odometry.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <dynamic_reconfigure/server.h>
#include <nav_core/base_local_planner.h>
#include <geometry_msgs/PoseStamped.h>

#include <reg_pure_pursuit_local_planner/RegPurePursuitPlannerConfig.h>
#include <reg_pure_pursuit_local_planner/transform_global_plan.h>

#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

namespace reg_pure_pursuit_local_planner {

class RegPurePursuitPlanner : public nav_core::BaseLocalPlanner 
{
public:
  RegPurePursuitPlanner();
  ~RegPurePursuitPlanner();

  /**
   * @brief  Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base
   * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
   * @return True if a valid velocity command was found, false otherwise
   */
  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

  /**
   * @brief  Check if the goal pose has been achieved by the local planner
   * @return True if achieved, false otherwise
   */
  bool isGoalReached();

  /**
   * @brief  Set the plan that the local planner is following
   * @param plan The plan to pass to the local planner
   * @return True if the plan was updated successfully, false otherwise
   */
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);

  /**
   * @brief Constructs the local planner
   * @param name The name to give this instance of the local planner
   * @param tf A pointer to a transform listener
   * @param costmap_ros The cost map to use for assigning costs to local plans
   */
  void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

private:

  /**
   * @brief Load param from rosparam
   * */
  void loadParams();

  /**
    * @brief Get robot coordinates in local frame
    * @param name The name to give this instance of the local planner
    */
  void getGoalLocalCoordinates( geometry_msgs::PoseStamped &localCoordiantes,
                                geometry_msgs::PoseStamped globalCoordinates,
                                double look_ahead_dist_,
                                geometry_msgs::PoseStamped &goalCoordinates,
                                double & lookahead_euclidean,
                                geometry_msgs::PoseStamped robotPose);

  void applyConstraints(const double & dist_error, const double & lookahead_dist, const double & curvature,
    const double & pose_cost, double & linear_vel, double & sign);

  double costAtPose(const double & x, const double & y);

  void rotateToHeading(double & linear_vel, double & angular_vel,const double & angle_to_path);

  bool shouldRotateToGoalHeading(const geometry_msgs::PoseStamped & carrot_pose);

  bool shouldRotateToPath(const geometry_msgs::PoseStamped & carrot_pose, double & angle_to_path);
  
  bool isCollisionImminent(const geometry_msgs::PoseStamped & robot_pose, const double & linear_vel, const double & angular_vel, const double & carrot_dist);

  bool inCollision(const double & x, const double & y, const double & theta);

  /**
  *@brief Reconfigure config_
  */
  void reconfigureCB(reg_pure_pursuit_local_planner::RegPurePursuitPlannerConfig &config, uint32_t level);

  /**
    * @brief Get euclidean distance
    * @param intial x and y points and end x and y points
    */
  double getEuclidianDistance(const double x_init, const double y_init,
                        const double x_end, const double y_end) const;

  // ROS
  ros::Publisher local_plan_publisher_;
  ros::Publisher carrot_pose_pub_;
  ros::Publisher carrot_arc_pub_;

  //check if plan first at first time
  bool first_setPlan_;
  //true if the goal point is reache and orientation of goal is reached
  bool goal_reached_;

  // Dynamic Reconfigure
  std::shared_ptr< dynamic_reconfigure::Server<RegPurePursuitPlannerConfig> > dsrv_;
  dynamic_reconfigure::Server<RegPurePursuitPlannerConfig>::CallbackType dyn_config_cb;
  reg_pure_pursuit_local_planner::RegPurePursuitPlannerConfig config_;

  //global plan which we run along
  std::vector<geometry_msgs::PoseStamped> global_plan_;

  //last point of the global plan in global frame
  tf::Stamped<tf::Pose> goal_pose_;
  // true if the robot should rotate to gobal plan if new global goal set
  tf::Stamped<tf::Pose> old_goal_pose_;

  // TF thingy
  tf2_ros::Buffer* tf_buffer_;
  double transform_tolerance_;
  std::string name_;

  // Bookeeping last command vel
  geometry_msgs::Twist last_cmd_vel_;

  // Parameters
  double xy_tolerance_;
  double th_tolerance_;
  double look_ahead_dist_;
  double max_linear_vel_;
  double max_angular_vel_;
  double regulated_linear_scaling_min_radius_;
  bool use_regulated_linear_velocity_scaling_;
  bool use_cost_regulated_linear_velocity_scaling_;
  double inflation_cost_scaling_factor_;
  double cost_scaling_dist_;
  double cost_scaling_gain_;
  double regulated_linear_scaling_min_speed_;
  double min_approach_linear_velocity_;
  double lookahead_dist_;

  double rotate_to_heading_min_angle_;
  bool use_rotate_to_heading_;
  double rotate_to_heading_angular_vel_;
  double max_angular_accel_;
  double controler_frequency_;

  double max_allowed_time_to_collision_up_to_carrot_;

  // Utility
  std::shared_ptr<costmap_2d::Costmap2DROS> costmap_ptr_;    
  std::mutex mutex_;
};

};

#endif //CATKIN_WS_PURE_PURSUIT_PLANNER_H
