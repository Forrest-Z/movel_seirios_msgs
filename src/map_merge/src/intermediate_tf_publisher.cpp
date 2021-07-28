#include <ros_utils/ros_utils.h>
#include <movel_hasp_vendor/license.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

geometry_msgs::Transform superodom2baselink_msg, superodom2odom_msg;
geometry_msgs::TransformStamped superodom2odom_stamped_msg;
bool received_initial_pose = false;

void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  ROS_INFO("[intermediate_tf_pub] received initial pose");
  if (msg->header.frame_id != "superodom")
  {
    ROS_WARN("[intermediate_tf_pub] initial pose header is not superodom; ignoring message.");
    return;
  }

  // initial pose is superodom->base_link transform
  superodom2baselink_msg.translation.x = msg->pose.pose.position.x;
  superodom2baselink_msg.translation.y = msg->pose.pose.position.y;
  superodom2baselink_msg.translation.z = msg->pose.pose.position.z;
  superodom2baselink_msg.rotation = msg->pose.pose.orientation;

  received_initial_pose = true;
}

int main(int argc, char** argv)
{
  #ifdef MOVEL_LICENSE
    MovelLicense ml(36);
    if (!ml.login())
      return 1;
  #endif

  ros::init(argc, argv, "intermediate_tf_publisher");
  ros::NodeHandle n;

  ros::Subscriber initial_pose_sub_ = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
    "initial_pose", 1, initialPoseCallback);  

  // superodom2baselink_msg correct initial rotation
  superodom2baselink_msg.rotation.w = 1.0;

  // superodom2odom_msg correct initial rotation
  superodom2odom_msg.rotation.w = 1.0;

  // initialise superodom2odom_stamped_msg and correct initial rotation
  superodom2odom_stamped_msg.header.frame_id = "superodom";
  superodom2odom_stamped_msg.child_frame_id = "odom";
  superodom2odom_stamped_msg.transform.rotation.w = 1.0;

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);

  ros::Rate r(10);
  while (ros::ok())
  {
    if (received_initial_pose)
    {
      // get odom->base_link transform
      geometry_msgs::Transform odom2baselink_msg;
      try
      {
        geometry_msgs::TransformStamped odom2baselink_stamped_msg = tf_buffer.lookupTransform("odom", "base_link", ros::Time(0));
        odom2baselink_msg = odom2baselink_stamped_msg.transform;
      }
      catch(tf2::TransformException &ex)
      {
        ROS_WARN("[intermediate_tf_publisher] cannot get transform: %s", ex.what());
        ros::Duration(1.0).sleep();
        continue;
      }

      // convert geometry_msgs::Transform types into tf2::Transform types for transform calculation
      tf2::Transform superodom2baselink_tf, odom2baselink_tf;
      tf2::fromMsg(superodom2baselink_msg, superodom2baselink_tf);
      tf2::fromMsg(odom2baselink_msg, odom2baselink_tf);

      // calculate the actual superodom->odom transfrom from superodom->base_link and odom->base_link information
      // superodomTodom = superodomTbase_link * inv(odomTbase_link)
      tf2::Transform superodom2odom_tf = superodom2baselink_tf * odom2baselink_tf.inverse();

      // convert tf2::Transform to geometry_msgs::Transform for publishing tf
      superodom2odom_msg = tf2::toMsg(superodom2odom_tf);

      received_initial_pose = false;
    }

    // populate superodom->odom tf msg header and transform
    superodom2odom_stamped_msg.header.stamp = ros::Time::now();
    superodom2odom_stamped_msg.transform = superodom2odom_msg;

    static tf2_ros::TransformBroadcaster br;

    // publish superodom->odom tf msg
    br.sendTransform(superodom2odom_stamped_msg);

    ros::spinOnce();
    r.sleep();
  }

  #ifdef MOVEL_LICENSE
    ml.logout();
  #endif

  return 0;
}