#include "movel_analytics_utils/distance_tracker.h"

namespace movel_analytics_utils
{
  DistanceTracker::DistanceTracker(ros::NodeHandle& nh, ros::NodeHandle& priv_nh) : nh_(nh), priv_nh_(priv_nh)
  {
    init();
  }

  void DistanceTracker::init()
  {
    ROS_INFO("Initializing distance tracker");
    loadParams();
    publish_distance_routine_ = nh_.createTimer(ros::Duration(publish_distance_rate_), &DistanceTracker::publishDistance, this);

    robot_pose_sub_ = nh_.subscribe("/pose", 1, &DistanceTracker::robotPoseCB, this);
    initialpose_sub_ = nh_.subscribe("/initialpose", 3, &DistanceTracker::initialPoseCB, this);

    distance_pub_ = nh_.advertise<std_msgs::Float32>("/distance_travelled", 1);
  }

  /**
   * @brief Callback for /initialpose topic. Used to reset distance counter when robot is relocalized.
  */
  void DistanceTracker::initialPoseCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
  {
    // Sleep to skip covariance noise and let the robot's position settle
    ros::Duration(1.0).sleep();
    is_relocalized_ = true;
    robot_pose_ = msg->pose.pose;
    last_robot_pose_ = robot_pose_;
  }

  /**
   * @brief Callback for /pose topic. Used to calculate distance travelled.
   * Will not start calculating distance until robot is localized and initialized (first pose message received).
  */
  void DistanceTracker::robotPoseCB(const geometry_msgs::Pose::ConstPtr& msg)
  {
    if (!is_initialized_)
    {
      robot_pose_ = *msg;
      last_robot_pose_ = robot_pose_;
      is_initialized_ = true;
      return;
    }

    if (is_relocalized_)
    {
      // Sleep to skip covariance noise and let the robot's position settle
      ros::Duration(1.0).sleep();
      is_relocalized_ = false;
      robot_pose_ = *msg;
      last_robot_pose_ = robot_pose_;
      return;
    }

    robot_pose_ = *msg;

    distance_increment_ = calculateEuclidianDist(robot_pose_, last_robot_pose_);
    if (distance_increment_ > distance_increment_threshold_) // Skip noisy readings
    {
      distance_ += distance_increment_;
      last_robot_pose_ = robot_pose_;
      // ROS_INFO("Distance travelled: %f", distance_);
    }
  }

  /**
   * @brief Publishes distance travelled to /distance_travelled topic at a rate specified by publish_distance_rate_ param.
  */
  void DistanceTracker::publishDistance(const ros::TimerEvent& event)
  {
    if (!is_initialized_)
    {
      return;
    }
    std_msgs::Float32 distance_msg;
    distance_msg.data = distance_;
    distance_pub_.publish(distance_msg);
    // ROS_INFO("Distance published: %f", distance_);
    distance_ = 0.0;
  }

  float DistanceTracker::calculateEuclidianDist(geometry_msgs::Pose start, geometry_msgs::Pose goal)
  {
    float x_diff = start.position.x - goal.position.x;
    float y_diff = start.position.y - goal.position.y;
    float z_diff = start.position.z - goal.position.z;
    return sqrt(pow(x_diff, 2) + pow(y_diff, 2) + pow(z_diff, 2));
  }

  void DistanceTracker::loadParams()
  {
    ros_utils::ParamLoader param_loader(nh_);
    param_loader.get_optional("publish_distance_rate", publish_distance_rate_, 10);
    param_loader.get_optional("distance_increment_threshold", distance_increment_threshold_, 0.01);
  }
} // namespace movel_analytics_utils