#include <dalu_docking/tag_offset.h>
#include <movel_hasp_vendor/license.h>

TagOffset::TagOffset() : nh_private_("~"), tfListener_(tfBuffer_), start_(true)
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
  if (nh_private_.hasParam("apriltag_x_offset"))
    nh_private_.getParam("apriltag_x_offset", p_apriltag_x_offset_);
  else
    return false;

  if (nh_private_.hasParam("apriltag_y_offset"))
    nh_private_.getParam("apriltag_y_offset", p_apriltag_y_offset_);
  else
    return false;

  if (nh_private_.hasParam("reverse"))
    nh_private_.getParam("reverse", p_reverse_);
  else
    return false;

  if (nh_private_.hasParam("apriltag_yaw_tolerance"))
    nh_private_.getParam("apriltag_yaw_tolerance", p_apriltag_yaw_tolerance_);
  else
    return false;

  if (nh_private_.hasParam("frames_tracked"))
    nh_private_.getParam("frames_tracked", p_frames_tracked_);
  else
    return false;
  return true;

  if (nh_private_.hasParam("tag_quantity"))
    nh_private_.getParam("tag_quantity", p_tag_quantity_);
  else
    return false;
  return true;
}

void TagOffset::setupTopics()
{
  tag_sub_ = nh_.subscribe("/tag_detections", 10, &TagOffset::tagCallback, this);
  start_sub_ = nh_.subscribe("/docking_start", 10, &TagOffset::startCallback, this);
}

void TagOffset::startCallback(std_msgs::Empty msg)
{
  start_ = true;
}

// Publish transforms relative to tag position
void TagOffset::tagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
{
  if(msg->detections.size() > 0)
  {
    // Select the nearest tag ID detected to follow for docking (for cases where multiple tags are detected) 
    if(start_)
    {
      double distance = 0;
      size_t index = 0;
      for (size_t i = 0; i < msg->detections.size(); i++)
      {
        if (msg->detections[i].id.size() == p_tag_quantity_) //&& msg->detections[i].id[0] == p_tag_id_)
        {
          geometry_msgs::TransformStamped transform;
          geometry_msgs::PoseWithCovarianceStamped base_link_pose;
          try
          {
            transform = tfBuffer_.lookupTransform("base_link", msg->header.frame_id, ros::Time(0), ros::Duration(0.5));
          }
          catch (tf2::TransformException &ex)
          {
            ROS_WARN("[tag_offset] %s", ex.what());
            return;
          }
          tf2::doTransform(msg->detections[i].pose, base_link_pose, transform);

          if(i == 0)
          {
            distance = fabs(base_link_pose.pose.pose.position.y);
          }
          else
          {
            double temp_distance = fabs(base_link_pose.pose.pose.position.y);
            if(temp_distance < distance)
            {
              distance = temp_distance;
              index = i;
            }
          }
        }
      }
      selected_id_ = msg->detections[index].id[0];
      start_ = false;
    }

    tf::Quaternion q;
    geometry_msgs::Quaternion quat;

    // Match tag orientation to base_link
    geometry_msgs::TransformStamped reorient_tf;
    reorient_tf.header.stamp = ros::Time::now();
    reorient_tf.header.frame_id = "tag_" + std::to_string(selected_id_);
    reorient_tf.child_frame_id = "reorient_tf";
    q.setRPY(-M_PI/2, M_PI/2, 0);
    tf::quaternionTFToMsg(q, quat);
    reorient_tf.transform.rotation = quat;
    br_.sendTransform(reorient_tf);

    geometry_msgs::TransformStamped transform;
    try
    {
      transform = tfBuffer_.lookupTransform("base_link", "reorient_tf", ros::Time(0), ros::Duration(0.5));
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("[tag_offset] %s", ex.what());
      return;
    }
    tf::Quaternion q0(transform.transform.rotation.x, transform.transform.rotation.y,
                      transform.transform.rotation.z, transform.transform.rotation.w);
    tf::Matrix3x3 m0(q0);
    double r0, p0, y0;
    m0.getRPY(r0, p0, y0);

    // Filter out roll & pitch angles
    geometry_msgs::TransformStamped filter_tf;
    filter_tf.header.stamp = ros::Time::now();
    filter_tf.header.frame_id = "reorient_tf";
    filter_tf.child_frame_id = "filter_tf";
    q.setRPY(-r0, -p0, 0);
    tf::quaternionTFToMsg(q, quat);
    filter_tf.transform.rotation = quat;
    br_.sendTransform(filter_tf);    

    // Only process tags that camera faces head on within tolerance
    if (!p_reverse_)
    {
      if(fabs(tf::getYaw(transform.transform.rotation)) > p_apriltag_yaw_tolerance_)
      {
        ROS_WARN("[tag_offset] Tag yaw exceeded tolerance");
        return;
      }
    }
    else
    {
      if((M_PI - fabs(tf::getYaw(transform.transform.rotation))) > p_apriltag_yaw_tolerance_)
      {
        ROS_WARN("[tag_offset] Tag yaw exceeded tolerance");
        return;
      }
    }

    // Publish tf of final docking position
    geometry_msgs::TransformStamped offset_tf;
    offset_tf.header.stamp = ros::Time::now();
    offset_tf.header.frame_id = "filter_tf";
    offset_tf.child_frame_id = "offset_tf";
    offset_tf.transform.translation.x = -p_apriltag_x_offset_;
    offset_tf.transform.translation.y = p_apriltag_y_offset_;
    if (!p_reverse_)
      q.setRPY(0, 0, 0);
    else
      q.setRPY(0, 0, M_PI);
    tf::quaternionTFToMsg(q, quat);
    offset_tf.transform.rotation = quat;
    br_.sendTransform(offset_tf);

    // Publish tf if initial docking position
    offset_tf.child_frame_id = "offset_tf2";
    geometry_msgs::TransformStamped base_link_to_apriltag;
    try
    {
      base_link_to_apriltag = tfBuffer_.lookupTransform("base_link", "reorient_tf", ros::Time(0), ros::Duration(0.5));
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
      return;
    }

    tf::Quaternion q1(base_link_to_apriltag.transform.rotation.x, base_link_to_apriltag.transform.rotation.y,
                      base_link_to_apriltag.transform.rotation.z, base_link_to_apriltag.transform.rotation.w);
    tf::Matrix3x3 m(q1);
    double r, p, yaw;
    m.getRPY(r, p, yaw);

    double dx = base_link_to_apriltag.transform.translation.x;
    double dy = base_link_to_apriltag.transform.translation.y;
    double distance = sqrt(dx*dx + dy*dy);
    double transform_yaw = atan(fabs(dy) / fabs(dx));
    double yaw_relative;
    if((dx > 0 && dy > 0) || (dx < 0 && dy < 0))
      yaw_relative = transform_yaw - yaw;
    else if ((dx > 0 && dy < 0) || (dx < 0 &&  dy > 0))
      yaw_relative = transform_yaw + yaw;

    offset_tf.header.stamp = ros::Time::now();
    if(!p_reverse_)
      offset_tf.transform.translation.x = -distance * cos(fabs(yaw_relative));
    else
      offset_tf.transform.translation.x = distance * cos(fabs(yaw_relative));
    br_.sendTransform(offset_tf);
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
