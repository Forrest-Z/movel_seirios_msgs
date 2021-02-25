#include <orbomator/orbomator.hpp>
#include <movel_hasp_vendor/license.h>

Orbomator::Orbomator() : tf_ear_(tf_buffer_), have_amcl_pose_(false), have_orb_pose_(false)
{
  setupParams();
  setupTopics();
}

void Orbomator::setupParams()
{
  ros::NodeHandle nh_local("~");
  allow_reinit_ = false;
  if (nh_local.hasParam("allow_reinit"))
    nh_local.getParam("allow_reinit", allow_reinit_);
  ROS_INFO("allow_reinit %d", allow_reinit_);

  dist_reinit_ = 1.0;
  if (nh_local.hasParam("dist_reinit"))
    nh_local.getParam("dist_reinit", dist_reinit_);
  ROS_INFO("dist_reinit %6.3f", dist_reinit_);

  dt_reinit_ = 1.0;
  if (nh_local.hasParam("dt_reinit"))
    nh_local.getParam("dt_reinit", dt_reinit_);
  ROS_INFO("dt_reinit %6.3f", dt_reinit_);

  map_frame_ = "map";
  if (nh_local.hasParam("map_frame"))
    nh_local.getParam("map_frame", map_frame_);
  ROS_INFO("map_frame %s", map_frame_.c_str());

  robot_frame_ = "base_link";
  if (nh_local.hasParam("robot_frame"))
    nh_local.getParam("robot_frame", robot_frame_);
  ROS_INFO("robot_frame %s", robot_frame_.c_str());

  camera_frame_ = "camera_link";
  if (nh_local.hasParam("camera_frame"))
    nh_local.getParam("camera_frame", camera_frame_);
  ROS_INFO("camera_frame %s", camera_frame_.c_str());

  orb_camera_frame_ = "orb_camera_link";
  if (nh_local.hasParam("orb_camera_frame"))
    nh_local.getParam("orb_camera_frame", orb_camera_frame_);
  ROS_INFO("orb_camera_frame %s", orb_camera_frame_.c_str());

  cov_lin_ = 1.0;
  if (nh_local.hasParam("cov_linear"))
    nh_local.getParam("cov_linear", cov_lin_);
  ROS_INFO("cov linear %6.3f", cov_lin_);
  cov_lin_ *= cov_lin_;

  cov_ang_ = M_PI / 2.0;
  if (nh_local.hasParam("cov_angular"))
    nh_local.getParam("cov_angular", cov_ang_);
  ROS_INFO("cov angular %6.3f", cov_ang_);
  cov_ang_ *= cov_ang_;
}

void Orbomator::setupTopics()
{
  amcl_reinit_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1);

  amcl_pose_sub_ = nh_.subscribe("amcl_pose", 1, &Orbomator::amclPoseCb, this);
  orb_pose_sub_ = nh_.subscribe("orb_slam2_rgbd/pose", 1, &Orbomator::orbPoseCb, this);
  reinit_sub_ = nh_.subscribe("reinit_amcl", 1, &Orbomator::reinitCb, this);
  auto_reinit_sub_ = nh_.subscribe("auto_reinit_to_orb", 1, &Orbomator::allowAutoReinitCb, this);
}

void Orbomator::reinitPose(geometry_msgs::PoseStamped pose)
{
  // transform to map frame
  geometry_msgs::TransformStamped transform;
  try
  {
    transform = tf_buffer_.lookupTransform(map_frame_, pose.header.frame_id, ros::Time(0));
  }
  catch (const tf2::LookupException& e)
  {
    ROS_INFO("at reinit; lookup exception; %s", e.what());
    ROS_INFO("try again later?");
    return;
  }
  catch (const tf2::ExtrapolationException& exc)
  {
    ROS_INFO("at reinit; extrapolation exception: %s", exc.what());
    ROS_INFO("try again later?");
    return;
  }
  catch (const tf2::TransformException& exc)
  {
    ROS_INFO("at reinit; tf exception: %s", exc.what());
    ROS_INFO("try again later?");
    return;
  }
  geometry_msgs::PoseStamped map_pose;
  tf2::doTransform(pose, map_pose, transform);

  geometry_msgs::PoseWithCovarianceStamped reinit_pose;
  reinit_pose.header.frame_id = map_frame_;
  reinit_pose.header.stamp = pose.header.stamp;
  reinit_pose.pose.pose = map_pose.pose;

  // populate linear
  for (int i = 0; i < 3; i++)
  {
    int idx = i * 6 + i;
    reinit_pose.pose.covariance[idx] = cov_lin_;
  }

  // populate angular
  for (int i = 3; i < 6; i++)
  {
    int idx = i * 6 + i;
    reinit_pose.pose.covariance[idx] = cov_ang_;
  }

  // reinit amcl
  amcl_reinit_pub_.publish(reinit_pose);
}

void Orbomator::amclPoseCb(geometry_msgs::PoseWithCovarianceStamped msg)
{
  // ROS_INFO("new amcl pose");
  have_amcl_pose_ = true;
  latest_amcl_pose_ = msg;
}

void Orbomator::orbPoseCb(geometry_msgs::PoseStamped msg)
{
  // ROS_INFO("new orb pose");
  have_orb_pose_ = true;
  latest_orb_pose_ = msg;

  // ROS_INFO("amcl pose %d, reinit %d", have_amcl_pose_, allow_reinit_);

  if (have_amcl_pose_ && allow_reinit_)
  {
    double dx, dt;
    // harmonise frame
    geometry_msgs::Pose map_pose;
    geometry_msgs::TransformStamped transform;
    geometry_msgs::TransformStamped tf_base_to_cam;
    geometry_msgs::TransformStamped tf_orb_cam_to_map;
    try
    {
      transform = tf_buffer_.lookupTransform(latest_amcl_pose_.header.frame_id, msg.header.frame_id, ros::Time(0));

      tf_base_to_cam = tf_buffer_.lookupTransform(camera_frame_, robot_frame_, ros::Time(0));

      tf_orb_cam_to_map =
          tf_buffer_.lookupTransform(latest_amcl_pose_.header.frame_id, orb_camera_frame_, ros::Time(0));
    }
    catch (tf2::LookupException exc)
    {
      ROS_INFO("at pose callback; lookup error: %s", exc.what());
      ROS_INFO("try again later?");

      return;
    }
    catch (const tf2::ExtrapolationException& exc)
    {
      ROS_INFO("at pose callback; extrapolation error: %s", exc.what());
      ROS_INFO("try again later?");
      return;
    }
    catch (const tf2::TransformException& exc)
    {
      ROS_INFO("at pose callback; transform exception: %s", exc.what());
      ROS_INFO("try again later?");
      return;
    }

    geometry_msgs::Pose robot_pose_cam, robot_pose_map;
    robot_pose_cam.position.x = tf_base_to_cam.transform.translation.x;
    robot_pose_cam.position.y = tf_base_to_cam.transform.translation.y;
    robot_pose_cam.position.z = tf_base_to_cam.transform.translation.z;
    robot_pose_cam.orientation = tf_base_to_cam.transform.rotation;

    tf2::doTransform(robot_pose_cam, robot_pose_map, tf_orb_cam_to_map);

    tf2::doTransform(msg.pose, map_pose, transform);

    dx = calcPoseDist(robot_pose_map, latest_amcl_pose_.pose.pose);
    dt = (msg.header.stamp - latest_amcl_pose_.header.stamp).toSec();

    if (dx > dist_reinit_ && fabs(dt) > dt_reinit_)
    {
      ROS_INFO("reiniting because dx %6.3f/%6.3f, dt %6.3f/%6.3f", dx, dist_reinit_, dt, dt_reinit_);
      geometry_msgs::PoseStamped pose_reinit;
      pose_reinit.header.frame_id = latest_amcl_pose_.header.frame_id;
      pose_reinit.header.stamp = msg.header.stamp;
      pose_reinit.pose = robot_pose_map;
      reinitPose(pose_reinit);
    }
  }
}

void Orbomator::reinitCb(std_msgs::Empty msg)
{
  if (!(have_amcl_pose_ && have_orb_pose_))
  {
    ROS_INFO("not ready to reinit, either amcl or orb pose missing");
    return;
  }

  ROS_INFO("force reinit to latest orb pose");
  reinitPose(latest_orb_pose_);
}

void Orbomator::allowAutoReinitCb(std_msgs::Bool msg)
{
  allow_reinit_ = msg.data;
  ROS_INFO("auto reinit is now %d", allow_reinit_);
}

double Orbomator::calcPoseDist(geometry_msgs::Pose pose_a, geometry_msgs::Pose pose_b)
{
  double dx, dy;
  dx = pose_a.position.x - pose_b.position.x;
  dy = pose_a.position.y - pose_b.position.y;

  return sqrt(dx * dx + dy * dy);
}

int main(int argc, char** argv)
{
#ifdef MOVEL_LICENSE
  MovelLicense ml(52);
  if (!ml.login())
    return 1;
#endif

  ros::init(argc, argv, "orbomateur");

  Orbomator orbomator;
  ros::spin();

#ifdef MOVEL_LICENSE
  ml.logout();
#endif

  return 0;
}
