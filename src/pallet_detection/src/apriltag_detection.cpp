#include <pallet_detection/apriltag_detection.h>

ApriltagDetection::ApriltagDetection() : tfListener_(tfBuffer_),
                                         goal_published_(false),
                                         status_received_(false),
                                         stages_complete_(false),
                                         next_stage_(false),
                                         history_index_(0)
{
  //callbackType = boost::bind(&HumanDetection::reconfigureCB, this, _1, _2);
  //configServer.setCallback(callbackType);
}

bool ApriltagDetection::loadParams()
{
  ros::param::param<int32_t>("~tag_id", tag_id_, 0);
  ros::param::param<double>("~x_offset", x_offset_, 0.5);
  ros::param::param<double>("~y_offset", y_offset_, 0.0);
  ros::param::param<double>("~yaw_offset", yaw_offset_, 0.0);
  ros::param::param<double>("~goal_xy_tolerance", goal_xy_tolerance_, 0.05);
  ros::param::param<double>("~goal_yaw_tolerance", goal_yaw_tolerance_, 1);
  ros::param::param<int>("~frames_tracked", frames_tracked_, 5);
  ros::param::param<double>("~inlier_dist", inlier_dist_, 0.2);
  ros::param::param<double>("~inlier_yaw", inlier_yaw_, 5);
  ros::param::param<bool>("~use_move_base", use_move_base_, false);
  ros::param::param<std::string>("~apriltag_frame", apriltag_frame_, "tag_0");
  ROS_INFO("use move_base: %d", use_move_base_);
  return true;
}

void ApriltagDetection::statusCallback(std_msgs::Bool success)
{
  if(!use_move_base_)
  {
    if(status_received_)
    {
      ROS_INFO("[apriltag_detection] 2nd stage done");
      stages_complete_ = true;
      return;
    }
    ROS_INFO("[apriltag_detection] 1st stage done");
    status_received_ = true;
    next_stage_ = true;
  }
}

void ApriltagDetection::mbCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr &msg)
{
  if(use_move_base_ && (msg->status.status == 3 || msg->status.status == 4))
  {
    if(status_received_)
    {
      ROS_INFO("[apriltag_detection] 2nd stage done");
      stages_complete_ = true;
      return;
    }
    ROS_INFO("[apriltag_detection] 1st stage done");
    status_received_ = true;
    next_stage_ = true;
  }
}

void ApriltagDetection::tagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
{
  //! Publish tf from tag to docking position
  if(msg->detections.size() > 0)
  {
    for (size_t i = 0; i < msg->detections.size(); i++)
    {
      if (msg->detections[i].id.size() == 1 && msg->detections[i].id[0] == tag_id_)
      {
        tf::Quaternion q;
        geometry_msgs::Quaternion quat;

        geometry_msgs::TransformStamped apriltag_tf;
        apriltag_tf.header.stamp = ros::Time::now();
        apriltag_tf.header.frame_id = apriltag_frame_;
        apriltag_tf.child_frame_id = "dock_position";
        q.setRPY(-M_PI/2, M_PI/2, 0);
        tf::quaternionTFToMsg(q, quat);
        apriltag_tf.transform.rotation = quat;
        br_.sendTransform(apriltag_tf);

        geometry_msgs::TransformStamped dock_tf;
        dock_tf.header.stamp = ros::Time::now();
        dock_tf.header.frame_id = "dock_position";
        dock_tf.child_frame_id = "apriltag_dock";
        dock_tf.transform.translation.y = y_offset_;
        q.setRPY(0, 0, yaw_offset_ * M_PI / 180);
        tf::quaternionTFToMsg(q, quat);
        dock_tf.transform.rotation = quat;

        if(!status_received_)
        {
          geometry_msgs::TransformStamped base_link_to_apriltag;
          try
          {
            base_link_to_apriltag = tfBuffer_.lookupTransform("base_link", "dock_position", ros::Time(0), ros::Duration(0.5));
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
          dock_tf.transform.translation.x = 0.9 * -distance * cos(fabs(yaw_relative));
          br_.sendTransform(dock_tf);
        }
        else
        {
          if(next_stage_)
          {
            history_index_ = 0;
            x_history_.clear();
            y_history_.clear();
            geometry_msgs::PoseStamped clear;
            recorded_goal_ = clear;
            next_stage_ = false;
            goal_published_ = false;
          }
          dock_tf.transform.translation.x = -x_offset_;
          br_.sendTransform(dock_tf);
        }
        historyAveraging();
      }
    }
  }
}

double ApriltagDetection::calcDistance(Eigen::Vector4f c1, Eigen::Vector4f c2)
{
  double dx = c1[0] - c2[0];
  double dy = c1[1] - c2[1];
  double dz = c1[2] - c2[2];

  return sqrt(dx*dx + dy*dy + dz*dz);
}

double ApriltagDetection::calcAngleDiff(geometry_msgs::Pose init_pose, geometry_msgs::Pose target_pose)
{
  tf::Quaternion q1(init_pose.orientation.x, init_pose.orientation.y,
                    init_pose.orientation.z, init_pose.orientation.w),
                 q2(target_pose.orientation.x, target_pose.orientation.y,
                    target_pose.orientation.z, target_pose.orientation.w);
  tf::Matrix3x3 m1(q1), m2(q2);
  double r1, p1, y1, r2, p2, y2;
  m1.getRPY(r1, p1, y1);
  m2.getRPY(r2, p2, y2);
  
  double dtheta = fabs(y1 - y2);
  dtheta = std::min(dtheta, 2.0*M_PI - dtheta);
  dtheta = dtheta / M_PI * 180;
  return dtheta;
}

void ApriltagDetection::tfToPose(geometry_msgs::TransformStamped tf, geometry_msgs::PoseStamped& pose)
{
  double norm = sqrt(pow(tf.transform.rotation.z, 2) + pow(tf.transform.rotation.w, 2));
  pose.header.frame_id = "odom";
  pose.pose.position.x = tf.transform.translation.x;
  pose.pose.position.y = tf.transform.translation.y;
  pose.pose.position.z = 0;
  pose.pose.orientation.x = 0;
  pose.pose.orientation.y = 0;
  pose.pose.orientation.z = tf.transform.rotation.z / norm;
  pose.pose.orientation.w = tf.transform.rotation.w / norm;
}

void ApriltagDetection::historyAveraging()
{
  geometry_msgs::TransformStamped dock_goal;
  geometry_msgs::PoseStamped goal_pose;
  
  //! Record a few transforms of docking goal to get the average transform
  if(x_history_.size() < frames_tracked_)
  {
    try
    {
      dock_goal = tfBuffer_.lookupTransform("odom", "apriltag_dock", ros::Time(0), ros::Duration(0.5));
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
      return;
    }
    tfToPose(dock_goal, goal_pose);
    x_history_.push_back(dock_goal.transform.translation.x);
    y_history_.push_back(dock_goal.transform.translation.y);
    return;
  }
  
  double sum_x = 0;
  double sum_y = 0;
  double distance = 0;

  try
  {
    dock_goal = tfBuffer_.lookupTransform("odom", "apriltag_dock", ros::Time(0), ros::Duration(0.5));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s", ex.what());
    return;
  }

  //! Filter out outlier docking position
  if(goal_published_)
  {
    geometry_msgs::PoseStamped current_goal;
    tfToPose(dock_goal, current_goal);
    Eigen::Vector4f p1(recorded_goal_.pose.position.x, recorded_goal_.pose.position.y, 0, 0),
                    p2(current_goal.pose.position.x, current_goal.pose.position.y, 0, 0);
    distance = calcDistance(p1, p2);
    double yaw_diff = calcAngleDiff(recorded_goal_.pose, current_goal.pose);
    if(distance > inlier_dist_ || yaw_diff > inlier_yaw_)
      return;
  }

  //! Overwrite old xy transform of goal with latest transform
  x_history_[history_index_] = dock_goal.transform.translation.x;
  y_history_[history_index_] = dock_goal.transform.translation.y;

  history_index_ = (history_index_ + 1) % frames_tracked_;

  //! Find average xy transform of goal
  for(size_t i = 0; i < frames_tracked_; i++)
  {
    sum_x += x_history_[i];
    sum_y += y_history_[i];
  }
  dock_goal.transform.translation.x = sum_x / frames_tracked_;
  dock_goal.transform.translation.y = sum_y / frames_tracked_;
  tfToPose(dock_goal, goal_pose);

  //! Publish docking goal
  if(!goal_published_)
  {
    goal_pub_.publish(goal_pose);
    recorded_goal_ = goal_pose;
    goal_published_ = true;
  }
  else if(status_received_)
  {
    Eigen::Vector4f g1(recorded_goal_.pose.position.x, recorded_goal_.pose.position.y, 0, 0),
                    g2(goal_pose.pose.position.x, goal_pose.pose.position.y, 0, 0);
    distance = calcDistance(g1, g2);
    double angle = calcAngleDiff(recorded_goal_.pose, goal_pose.pose);
    if(distance > goal_xy_tolerance_ || angle > goal_yaw_tolerance_)
    {
      if(!stages_complete_)
      {
        std_msgs::Bool stop;
        stop.data = true;
        stop_pub_.publish(stop);
        ros::Duration(0.1).sleep();
        goal_pub_.publish(goal_pose);
        recorded_goal_ = goal_pose;
      }
      else
      {
        current_goal_pub_.publish(goal_pose);
        recorded_goal_ = goal_pose;
      }
    }
  }
}