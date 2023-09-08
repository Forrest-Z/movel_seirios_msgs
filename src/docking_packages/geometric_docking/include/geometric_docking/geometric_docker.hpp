#ifndef geometric_docker_hpp
#define geometric_docker_hpp

#include <geometric_docking/dock_detector.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <actionlib_msgs/GoalID.h>

class GeometricDocker
{
public:
  /**
   * @brief GeomeetricDocker Constructor with separate namespace topic
   * @param nh
   */
  GeometricDocker(ros::NodeHandle& nh);
  ~GeometricDocker(){}

  /**
   * @brief setupParams - Read Parameters from config file
   */
  void setupParams();

  /**
   * @brief setupTopics - Setup subscribers and publishers
   * @param nh
   */
  void setupTopics(ros::NodeHandle& nh_);

  // Advertise Service
  ros::ServiceServer dock_start_sub_;
  ros::ServiceServer dock_cancel_sub_;
  ros::ServiceServer dock_resume_sub_;
  ros::ServiceServer dock_pause_sub_;
  ros::Publisher dock_status_pub_;

  
  /**
   * @brief dockStartCb - Callback function for starting the docking process
   * @param req
   * @param res
   * @return
   */
  bool dockStartCb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

  /**
   * @brief dockCancelCb - Callback function / function for terminating the docking process
   * @param req
   * @param res
   * @return
   */
  bool dockCancelCb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

  /**
   * @brief dockResumeCb - Callback function / function for resuming the docking process
   * @param req
   * @param res
   * @return
   */
  bool dockResumeCb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    /**
   * @brief dockPauseCb - Callback function / function for pausing the docking process
   * @param req
   * @param res
   * @return
   */
  bool dockPauseCb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

private:
  ros::NodeHandle nh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_ear_;

  // Parameters
  std::string docking_frame_;
  double d_lost_th_;
  int max_lost_count_;
  double lpf_wt_;         // low pass filter weight for adjusting dock position

  // Dock detector parameters
  double dock_width_;
  double dock_offset_;
  double update_period_;

  // Control parameters
  bool dock_backwards_;
  // Subscribers and publishers
  ros::Subscriber scan_sub_;
  ros::Subscriber dock_reached_sub_;
  ros::Publisher cmd_vel_pub_;
  ros::Publisher dock_pose_pub_;
  ros::Publisher controller_ref_pub_;
  ros::Publisher controller_stop_pub_;

  // Instances of the classes actually doing the work
  DockDetector dock_detector_;

  // Bookkeeping
  bool active_;
  bool have_dock_pose_;
  geometry_msgs::PoseStamped dock_pose_estimate_;

  ros::Time t_prev_estimate_;
  int lost_count_;
  // Callback functions //

  /**
   * @brief scanCb - Callback function for lidar scan topic
   * @param scan
   * @return
   */
  void scanCb(sensor_msgs::LaserScan scan);
  
  /**
   * @brief dockReachedCb - Callack function when planner-adjuster finished its task
   * @param msg
   * @return
   */
  void dockReachedCb(std_msgs::Bool msg);

  // Auxilliary functions
  void rotatePose(geometry_msgs::Pose &p);
};

#endif
