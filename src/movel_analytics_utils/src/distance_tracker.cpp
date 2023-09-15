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
        distance_pub_ = nh_.advertise<std_msgs::Float32>("/distance_travelled", 1);
    }

    void DistanceTracker::robotPoseCB(const geometry_msgs::Pose::ConstPtr& msg)
    {
        robot_pose_ = *msg;
    }

    void DistanceTracker::publishDistance(const ros::TimerEvent& event)
    {
        distance_increment_ = calculateEuclidianDist(robot_pose_, last_robot_pose_);
        distance_ += distance_increment_;
        if (distance_increment_ > distance_increment_threshold_) // Noise prevention
        {
            last_robot_pose_ = robot_pose_;
            std_msgs::Float32 distance_msg;
            distance_msg.data = distance_;
            distance_pub_.publish(distance_msg);
        }
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
        ros_utils::ParamLoader param_loader(priv_nh_);
        param_loader.get_optional("publish_distance_rate", publish_distance_rate_, 1.0);
        param_loader.get_optional("distance_increment_threshold", distance_increment_threshold_, 0.01);
    }
} // namespace movel_analytics_utils