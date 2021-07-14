#include <pointcloud_mux/pointcloud_mux.h>
#include <movel_hasp_vendor/license.h>

PointcloudMux::PointcloudMux()
  : nh_private_("~"), current_view_(View::FRONT)
{
  initialize();
}

void PointcloudMux::initialize()
{
  if (!loadParams())
  {
    ROS_FATAL("[pointcloud_mux] Error during parameter loading. Shutting down.");
    return;
  }
  ROS_INFO("[pointcloud_mux] All parameters loaded. Launching.");

  setupTopics();

  ros::spin();
}

// Load ROS params
bool PointcloudMux::loadParams()
{
  if (nh_private_.hasParam("front_view_pointcloud"))
    nh_private_.getParam("front_view_pointcloud", front_view_pointcloud_);
  else
    return false;

  if (nh_private_.hasParam("rear_view_pointcloud"))
    nh_private_.getParam("rear_view_pointcloud", rear_view_pointcloud_);
  else
    return false;
    
  if (nh_private_.hasParam("left_view_pointcloud"))
    nh_private_.getParam("left_view_pointcloud", left_view_pointcloud_);
  else
    return false;
    
  if (nh_private_.hasParam("right_view_pointcloud"))
    nh_private_.getParam("right_view_pointcloud", right_view_pointcloud_);
  else
    return false;

  if (nh_private_.hasParam("angular_speed_threshold"))
    nh_private_.getParam("angular_speed_threshold", angular_speed_threshold_);
  else
    return false;

  return true;
}

// Setup callbacks
void PointcloudMux::setupTopics()
{
  vel_sub_ = nh_.subscribe("/cmd_vel", 10, &PointcloudMux::velCallback, this);
  pointcloud_pub_ = nh_private_.advertise<sensor_msgs::PointCloud2>("pointcloud", 100);
  if (!front_view_pointcloud_.empty())
    front_sub_ = nh_.subscribe(front_view_pointcloud_.c_str(), 10, &PointcloudMux::frontCallback, this);
    
  if (!rear_view_pointcloud_.empty())
    rear_sub_ = nh_.subscribe(rear_view_pointcloud_.c_str(), 10, &PointcloudMux::rearCallback, this);

  if (!left_view_pointcloud_.empty())
    left_sub_ = nh_.subscribe(left_view_pointcloud_.c_str(), 10, &PointcloudMux::leftCallback, this);

  if (!right_view_pointcloud_.empty())
    right_sub_ = nh_.subscribe(right_view_pointcloud_.c_str(), 10, &PointcloudMux::rightCallback, this);
}

// Get velocity commands
void PointcloudMux::velCallback(const geometry_msgs::Twist::ConstPtr& vel)
{
  if ((vel->linear.x > 0 && fabs(vel->angular.z) < angular_speed_threshold_) && !front_view_pointcloud_.empty())
    current_view_ = View::FRONT;
  else if ((vel->linear.x < 0 && fabs(vel->angular.z) < angular_speed_threshold_) && !rear_view_pointcloud_.empty())
    current_view_ = View::REAR;
  else if ((vel->linear.y > 0 || vel->angular.z >= angular_speed_threshold_) && !left_view_pointcloud_.empty())
    current_view_ = View::LEFT;
  else if ((vel->linear.y < 0 || vel->angular.z <= -angular_speed_threshold_) && !right_view_pointcloud_.empty())
    current_view_ = View::RIGHT;    
}

// Get camera pointclouds
void PointcloudMux::frontCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud)
{
  if (current_view_ == View::FRONT)
    pointcloud_pub_.publish(*cloud);
}

void PointcloudMux::rearCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud)
{
  if (current_view_ == View::REAR)
    pointcloud_pub_.publish(*cloud);
}

void PointcloudMux::leftCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud)
{
  if (current_view_ == View::LEFT)
    pointcloud_pub_.publish(*cloud);
}

void PointcloudMux::rightCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud)
{
  if (current_view_ == View::RIGHT)
    pointcloud_pub_.publish(*cloud);
}

int main(int argc, char** argv)
{
  #ifdef MOVEL_LICENSE
  MovelLicense ml(35);
  if (!ml.login())
    return 1;
  #endif

  ros::init(argc, argv, "pointcloud_mux");
  PointcloudMux mux;

  #ifdef MOVEL_LICENSE
  ml.logout();
  #endif
};
