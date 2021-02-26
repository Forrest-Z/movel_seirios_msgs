#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

class Orbomator
{
public:
  Orbomator();
  ~Orbomator(){}

  void setupParams();
  void setupTopics();

  void reinitPose(geometry_msgs::PoseStamped pose);

private:
  // ros infra
  ros::NodeHandle nh_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_ear_;

  // parameters
  bool allow_reinit_;
  double dist_reinit_; // reinit if distance between amcl and orb is larger than this 
  double dt_reinit_; // reinit if distance qualifies and dt is lower than this
  std::string map_frame_;
  std::string robot_frame_;
  std::string camera_frame_;
  std::string orb_camera_frame_;
  double cov_lin_;
  double cov_ang_;

  // bookkeeping
  bool have_amcl_pose_, have_orb_pose_;
  geometry_msgs::PoseWithCovarianceStamped latest_amcl_pose_;
  geometry_msgs::PoseStamped latest_orb_pose_;

  // subscribers/publishers
  ros::Subscriber amcl_pose_sub_;
  ros::Subscriber orb_pose_sub_;
  ros::Subscriber reinit_sub_;
  ros::Subscriber auto_reinit_sub_;

  ros::Publisher amcl_reinit_pub_;

  // callbacks
  void amclPoseCb(geometry_msgs::PoseWithCovarianceStamped msg);
  void orbPoseCb(geometry_msgs::PoseStamped msg);
  void reinitCb(std_msgs::Empty msg);
  void allowAutoReinitCb(std_msgs::Bool msg);

  // helpers
  double calcPoseDist(geometry_msgs::Pose pose_a, geometry_msgs::Pose pose_b);
};