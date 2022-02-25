#include <odom_3dof/odom_3dof.h>
#include <movel_hasp_vendor/license.h>

Odom3dof::Odom3dof() : nh_private_("~")
{
  initialize();
}

void Odom3dof::initialize()
{
  if (!loadParams())
  {
    ROS_FATAL("[odom_3dof] Error during parameter loading. Shutting down.");
    return;
  }
  ROS_INFO("[odom_3dof] All parameters loaded. Launching.");

  setupTopics();

  ros::spin();
}

// Load ROS params
bool Odom3dof::loadParams()
{
  ros_utils::ParamLoader loader(nh_private_);
  loader.get_required("input_topic", input_topic_);
  loader.get_required("output_topic", output_topic_);
  loader.get_required("child_frame_id", child_frame_id_);
  return loader.params_valid();
}

// Setup callbacks
void Odom3dof::setupTopics()
{
  odom_sub_ = nh_.subscribe(input_topic_, 1, &Odom3dof::odomCallback, this);  // floam_odom
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>(output_topic_, 1, true);       // rtabmap/odom
}

// Get raw odom and set z, pitch, roll to 0
void Odom3dof::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_in)
{
  nav_msgs::Odometry odom_out;
  odom_out.header.stamp = odom_in->header.stamp; //ros::Time::now();
  odom_out.header.frame_id = odom_in->header.frame_id;
  odom_out.child_frame_id = child_frame_id_;
  odom_out.pose.pose.position.x = odom_in->pose.pose.position.x;
  odom_out.pose.pose.position.y = odom_in->pose.pose.position.y;
  tf::Quaternion q(odom_in->pose.pose.orientation.x, odom_in->pose.pose.orientation.y,
                   odom_in->pose.pose.orientation.z, odom_in->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double r, p, y;
  m.getRPY(r, p, y);
  q.setRPY(0, 0, y);
  geometry_msgs::Quaternion quat;
  tf::quaternionTFToMsg(q, quat);
  odom_out.pose.pose.orientation = quat;
  odom_out.pose.covariance = odom_in->pose.covariance;
  odom_out.twist.twist.linear.x = odom_in->twist.twist.linear.x;
  odom_out.twist.twist.linear.y = odom_in->twist.twist.linear.y;
  odom_out.twist.twist.angular.z = odom_in->twist.twist.angular.z;
  odom_out.twist.covariance = odom_in->twist.covariance;

  geometry_msgs::TransformStamped odom_to_base_link;
  odom_to_base_link.header.stamp = odom_in->header.stamp; //ros::Time::now(); 
  odom_to_base_link.header.frame_id = odom_in->header.frame_id;
  odom_to_base_link.child_frame_id = child_frame_id_;
  odom_to_base_link.transform.translation.x = odom_in->pose.pose.position.x;
  odom_to_base_link.transform.translation.y = odom_in->pose.pose.position.y;
  odom_to_base_link.transform.rotation = quat;

  odom_pub_.publish(odom_out);
  br_.sendTransform(odom_to_base_link);
}

int main(int argc, char** argv)
{
  #ifdef MOVEL_LICENSE
  MovelLicense ml(35);
  if (!ml.login())
    return 1;
  #endif

  ros::init(argc, argv, "odom_3dof");
  Odom3dof m;

  #ifdef MOVEL_LICENSE
  ml.logout();
  #endif
};