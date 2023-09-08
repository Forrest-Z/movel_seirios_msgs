#include <dalu_docking/tag_offset.h>
#include <movel_hasp_vendor/license.h>

TagOffset::TagOffset() : nh_private_("~"), tfListener_(tfBuffer_)
{
  initialize();
}

void TagOffset::initialize()
{
  if(!loadParams())
  {
    ROS_FATAL("[tag_offset] Failed to load params. Shutting down.");
    return;
  }
  setupTopics();

  ros::spin();
}

bool TagOffset::loadParams()
{
  if (nh_private_.hasParam("tag_id"))
    nh_private_.getParam("tag_id", p_tag_id_);
  else
    return false;

  if (nh_private_.hasParam("apriltag_x_offset"))
    nh_private_.getParam("apriltag_x_offset", p_apriltag_x_offset_);
  else
    return false;

  if (nh_private_.hasParam("apriltag_y_offset"))
    nh_private_.getParam("apriltag_y_offset", p_apriltag_y_offset_);
  else
    return false;

  return true;
}

void TagOffset::setupTopics()
{
  tag_sub_ = nh_.subscribe("/tag_detections", 10, &TagOffset::tagCallback, this);
}

// Publish transform from apriltag to dock position
void TagOffset::tagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
{
  if(msg->detections.size() > 0)
  {
    for (size_t i = 0; i < msg->detections.size(); i++)
    {
      if (msg->detections[i].id.size() == 1 && msg->detections[i].id[0] == p_tag_id_)
      {
        geometry_msgs::TransformStamped final_goal_tf;
        final_goal_tf.header.stamp = msg->detections[i].pose.header.stamp;
        final_goal_tf.header.frame_id = "tag_0";
        final_goal_tf.child_frame_id = "dock_position";
        final_goal_tf.transform.translation.z = p_apriltag_x_offset_;
        final_goal_tf.transform.translation.x = p_apriltag_y_offset_;
        final_goal_tf.transform.rotation.w = 1;
        br_.sendTransform(final_goal_tf);
      }
    }
  }
}

int main(int argc, char** argv)
{
  #ifdef MOVEL_LICENSE                                                                                                    
  MovelLicense ml;                                                                                                   
  if (!ml.login())                                                                                                      
    return 1;                                                                                                           
  #endif

  ros::init(argc, argv, "tag_offset");
  TagOffset to;
  
  #ifdef MOVEL_LICENSE                                                                                                    
  ml.logout();          
  #endif
}
