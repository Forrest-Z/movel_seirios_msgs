#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <movel_hasp_vendor/license.h>

double round_up(double value, int decimal_places) {
    const double multiplier = std::pow(10.0, decimal_places);
    return std::ceil(value * multiplier) / multiplier;
}

int main(int argc, char** argv)
{
#ifdef MOVEL_LICENSE
  MovelLicense ml(1);
  if (!ml.login())
    return 1;
#endif

  ros::init(argc, argv, "broadcast_pose");
  ros::NodeHandle n;
  ros::Publisher pose_pub = n.advertise<geometry_msgs::Pose>("pose", 10);

  tf::TransformListener listener;
  ros::Time now = ros::Time::now();
  listener.waitForTransform("map", "base_link", now, ros::Duration(10.0));
  ros::Duration(0.5).sleep();

  ros::Rate rate(10.0);
  while (n.ok())
  {
    tf::StampedTransform trans;
    try
    {
      listener.lookupTransform("map", "base_link", ros::Time(0), trans);
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
    p.position.x = round_up(v.getX(), 3);
    p.position.y = round_up(v.getY(), 3);
    p.position.z = round_up(v.getZ(), 3);
    p.orientation.x = round_up(q.x(), 3);
    p.orientation.y = round_up(q.y(), 3);
    p.orientation.z = round_up(q.z(), 3);
    p.orientation.w = round_up(q.w(), 3);

    pose_pub.publish(p);

    rate.sleep();
  }
#ifdef MOVEL_LICENSE
  ml.logout();
#endif
  return 0;
};
