#include <ros_utils/ros_utils.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>


geometry_msgs::TransformStamped transform;

void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  ROS_INFO("[intermediate_tf_pub] received initial pose");
  if (msg->header.frame_id != "superodom")
  {
    ROS_WARN("[intermediate_tf_pub] initial pose header is not superodom; ignoring message.");
    return;
  }

  transform.transform.translation.x = msg->pose.pose.position.x;
  transform.transform.translation.y = msg->pose.pose.position.y;
  transform.transform.translation.z = msg->pose.pose.position.z;
  transform.transform.rotation = msg->pose.pose.orientation;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "intermediate_tf_publisher");
  ros::NodeHandle n;

  ros::Subscriber initial_pose_sub_ = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
    "initial_pose", 1, initialPoseCallback);
  
  transform.header.frame_id = "superodom";
  transform.child_frame_id = "odom";
  // correct initial rotation
  transform.transform.rotation.w = 1.0;

  ros::Rate r(10);
  while (ros::ok())
  {
    static tf2_ros::TransformBroadcaster br;

    transform.header.stamp = ros::Time::now();

    br.sendTransform(transform);

    ros::spinOnce();
    r.sleep();
  }

  return 0;
}