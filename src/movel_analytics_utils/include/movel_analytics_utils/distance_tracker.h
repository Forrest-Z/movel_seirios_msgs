#ifndef MOVEL_ANALYTICS_UTILS__DISTANCE_TRACKER_H_
#define MOVEL_ANALYTICS_UTILS__DISTANCE_TRACKER_H_
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float32.h>
#include <ros_utils/ros_utils.h>

namespace movel_analytics_utils
{    
  class DistanceTracker
  {
  public:
    DistanceTracker(ros::NodeHandle& nh, ros::NodeHandle& priv_nh);
    ~DistanceTracker() = default;

    void init();

  private:
    ros::NodeHandle& nh_;
    ros::NodeHandle& priv_nh_;
    ros::Timer publish_distance_routine_;
    ros::Subscriber robot_pose_sub_;
    ros::Publisher distance_pub_;

    geometry_msgs::Pose robot_pose_;
    geometry_msgs::Pose last_robot_pose_;
    float distance_ = 0;
    float last_distance_ = 0;
    float distance_increment_ = 0;

    // Params
    double distance_increment_threshold_;
    double publish_distance_rate_;
    
    void robotPoseCB(const geometry_msgs::Pose::ConstPtr& msg);

    void publishDistance(const ros::TimerEvent& event);

    float calculateEuclidianDist(geometry_msgs::Pose start, geometry_msgs::Pose goal);

    void loadParams();
  };    

}  // namespace movel_analytics_utils
#endif  // MOVEL_ANALYTICS_UTILS__DISTANCE_TRACKER_H_
