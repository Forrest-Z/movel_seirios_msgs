#include <pallet_detection/pallet_detection.h>
#include <ros_utils/ros_utils.h>
#include <movel_hasp_vendor/license.h>

int main(int argc, char **argv)
{
  #ifdef MOVEL_LICENSE
  MovelLicense ml;
  if (!ml.login())
    return 1;
  #endif

  ros::init(argc, argv, "pallet_detection");

  PalletDetection detect;

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  ros::Time::waitForValid();
  if (!detect.loadParams())
  {
    ROS_FATAL("Error during parameter loading. Shutting down.");
    return 0;
  }
  ROS_INFO("All parameters loaded. Launching.");

  //! Subscribers
  ros::Subscriber point_cloud_sub = nh.subscribe("/camera/cloud", 1, &PalletDetection::pointCloudCallback, &detect);
  ros::Subscriber status_sub = nh.subscribe("/goal/status", 1, &PalletDetection::statusCallback, &detect);
  ros::Subscriber mb_sub = nh.subscribe("/move_base/result", 1, &PalletDetection::mbCallback, &detect);

  //! Publishers
  detect.goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/pid_goal", 1, true);

  detect.cloud_filtered_pub_ = private_nh.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 100);
  detect.marker_pub_ = private_nh.advertise<visualization_msgs::Marker>("pallets", 100);
  detect.centroids_pub_ = private_nh.advertise<visualization_msgs::Marker>("centroids", 100);
  detect.stop_pub_ = nh.advertise<std_msgs::Bool>("/stop_now", 1);
  detect.current_goal_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("current_goal", 1);

  ros::spin();
  #ifdef MOVEL_LICENSE
  ml.logout();
  #endif

  return 0;
}
