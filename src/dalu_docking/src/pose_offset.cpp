#include <dalu_docking/pose_offset.h>
#include <movel_hasp_vendor/license.h>

PoseOffset::PoseOffset() : nh_private_("~")
{
  initialize();
}

void PoseOffset::initialize()
{
  if(!loadParams())
  {
    ROS_FATAL("[tag_offset] Failed to load params. Shutting down.");
    return;
  }
  setupTopics();

  ros::spin();
}

bool PoseOffset::loadParams()
{
  if (nh_private_.hasParam("offset"))
    nh_private_.getParam("offset", p_offset_);
  else
    return false;

  if (nh_private_.hasParam("reverse"))
    nh_private_.getParam("reverse", p_reverse_);
  else
    return false;

  if (nh_private_.hasParam("x"))
    nh_private_.getParam("x", p_x_);
  else
    return false;

  if (nh_private_.hasParam("y"))
    nh_private_.getParam("y", p_y_);
  else
    return false;

  if (nh_private_.hasParam("z"))
    nh_private_.getParam("z", p_z_);
  else
    return false;

  if (nh_private_.hasParam("w"))
    nh_private_.getParam("w", p_w_);
  else
    return false;

  return true;
}

void PoseOffset::setupTopics()
{
  pub_timer_ = nh_.createTimer(ros::Duration(1.0/15.0), boost::bind(&PoseOffset::publishTF, this, _1));
}

// Publish tfs based on pose input
void PoseOffset::publishTF(const ros::TimerEvent& event)
{
  geometry_msgs::TransformStamped pose_tf;
  pose_tf.header.stamp = ros::Time::now();
  pose_tf.header.frame_id = "map";
  pose_tf.child_frame_id = "offset_tf2";
  pose_tf.transform.translation.x = p_x_;
  pose_tf.transform.translation.y = p_y_;
  pose_tf.transform.rotation.z = p_z_;
  pose_tf.transform.rotation.w = p_w_;
  br_.sendTransform(pose_tf);

  geometry_msgs::TransformStamped offset_tf;
  offset_tf.header.stamp = ros::Time::now();
  offset_tf.header.frame_id = "offset_tf2";
  offset_tf.child_frame_id = "offset_tf";
  offset_tf.transform.rotation.w = 1;
  if(!p_reverse_)
    offset_tf.transform.translation.x = p_offset_;
  else
    offset_tf.transform.translation.x = -p_offset_;
  br_.sendTransform(offset_tf);
}

int main(int argc, char** argv)
{
  #ifdef MOVEL_LICENSE                                                                                                    
  MovelLicense ml;                                                                                                   
  if (!ml.login())                                                                                                      
    return 1;                                                                                                           
  #endif

  ros::init(argc, argv, "pose_offset");
  PoseOffset po;
  
  #ifdef MOVEL_LICENSE                                                                                                    
  ml.logout();          
  #endif
}
