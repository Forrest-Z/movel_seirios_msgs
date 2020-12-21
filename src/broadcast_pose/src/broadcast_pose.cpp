#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "broadcast_pose");
  ros::NodeHandle n;

  ros::Publisher pose_pub = n.advertise<geometry_msgs::Pose>("pose", 10);

  tf::TransformListener listener;
  ros::Time now = ros::Time::now();
  listener.waitForTransform("/map", "/base_link", now, ros::Duration(10.0));
  ros::Duration(0.5).sleep();

  ros::Rate rate(10.0);
  while (n.ok())
  {
    tf::StampedTransform trans;
    try
    {
      listener.lookupTransform("/map", "/base_link", ros::Time(0), trans);
    }
    catch (tf::TransformException& ex)
    {
      ROS_ERROR_THROTTLE(1.0, "%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    tf::Quaternion q = trans.getRotation();
    tf::Vector3 v = trans.getOrigin();

    geometry_msgs::Pose p;
    p.position.x = v.getX();
    p.position.y = v.getY();
    p.position.z = v.getZ();
    p.orientation.x = q.x();
    p.orientation.y = q.y();
    p.orientation.z = q.z();
    p.orientation.w = q.w();

    pose_pub.publish(p);

    rate.sleep();
  }
  return 0;
};
