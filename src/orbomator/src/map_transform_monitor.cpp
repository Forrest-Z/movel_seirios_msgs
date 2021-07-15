#include <orbomator/map_transform_monitor.hpp>
#include <movel_hasp_vendor/license.h>


void printTF2Transform(tf2::Transform& transform)
{
  tf2::Vector3 trs = transform.getOrigin();
  tf2::Quaternion rot = transform.getRotation();
}

void writeTF2toCsv(tf2::Transform& transform, string& file_path)
{
  std::fstream csv_write;
  csv_write.open(file_path, std::ios::out | std::ios::app);

  tf2::Vector3 trs = transform.getOrigin();
  tf2::Quaternion rot = transform.getRotation();

  csv_write << "translation, " << trs.getX() << ", " << trs.getY() << ", " << trs.getZ() << ", ";  //<< endl;
  csv_write << "rotation, " << rot.getX() << ", " << rot.getY() << ", " << rot.getZ() << ", " << rot.getW() << "\n";
  csv_write.close();
}

MapTransformMonitor::MapTransformMonitor() : tf_ear_(tf_buffer_), have_map_transform_(false)
{
  setupParams();
  setupTopics();
}

bool MapTransformMonitor::setupParams()
{
  ros::NodeHandle nh_local("~");
  orb_map_frame_ = "orb_map";
  if (nh_local.hasParam("orb_map_frame"))
    nh_local.getParam("orb_map_frame", orb_map_frame_);

  orb_camera_frame_ = "orb_camera_link";
  if (nh_local.hasParam("orb_camera_frame"))
    nh_local.getParam("orb_camera_frame", orb_camera_frame_);

  map_frame_ = "map";
  if (nh_local.hasParam("map_frame"))
    nh_local.getParam("map_frame", map_frame_);

  camera_frame_ = "camera_link";
  if (nh_local.hasParam("camera_frame"))
    nh_local.getParam("camera_frame", camera_frame_);

  log_to_csv_ = false;
  if (nh_local.hasParam("log_to_csv"))
    nh_local.getParam("log_to_csv", log_to_csv_);

  log_file_path_ = "";
  if (nh_local.hasParam("log_file_path"))
  {
    nh_local.getParam("log_file_path", log_file_path_);
  }
  else
  {
    log_to_csv_ = false;
  }

  return true;
}

bool MapTransformMonitor::setupTopics()
{
  orb_pose_sub_ = nh_.subscribe("orb_slam2_rgbd/pose", 1, &MapTransformMonitor::orbPoseCb, this);
  return true;
}

void MapTransformMonitor::orbPoseCb(geometry_msgs::PoseStamped msg)
{
  // ROS_INFO("new orb pose");
  geometry_msgs::TransformStamped tf_cam_to_map;
  geometry_msgs::TransformStamped tf_orbcam_to_orbmap;
  try
  {
    // get camera to map transform
    tf_cam_to_map = tf_buffer_.lookupTransform(map_frame_, camera_frame_, ros::Time(0), ros::Duration(1.0));
    //ROS_INFO("cam to map OK");

    // get orb_camera to orb_map transform
    tf_orbcam_to_orbmap =
        tf_buffer_.lookupTransform(orb_map_frame_, orb_camera_frame_, ros::Time(0), ros::Duration(1.0));
    //ROS_INFO("orb cam to orb map OK");
  }
  catch (...)
  {
    ROS_WARN("Failed to get required transforms ");
    return;
  }
  
  try
  {
  // calculate orb_map to map transform
    tf2::Transform tf2_cam_to_map;
    tf2::Transform tf2_orbcam_to_orbmap;
    tf2::fromMsg(tf_cam_to_map.transform, tf2_cam_to_map);
    tf2::fromMsg(tf_orbcam_to_orbmap.transform, tf2_orbcam_to_orbmap);

    tf2::Transform tf2_orbmap_to_map = tf2_cam_to_map * tf2_orbcam_to_orbmap.inverse();
    //ROS_INFO("unifying transform OK");

    // calculate transform change
    if (!have_map_transform_)
    {
      latest_map_transform_ = tf2_orbmap_to_map;
      have_map_transform_ = true;
    }
    else
    {
      tf2::Transform dtf = latest_map_transform_.inverse() * tf2_orbmap_to_map;
      latest_map_transform_ = tf2_orbmap_to_map;
      // ROS_INFO_STREAM("latest transform " << std::endl << latest_map_transform_);
      // ROS_INFO_STREAM("difference: " << std::endl << dtf);
      // ROS_INFO("delta calca OK");
      // std::cout << "latest transform: " << std::endl;
      //printTF2Transform(latest_map_transform_);

      if (log_to_csv_)
        writeTF2toCsv(latest_map_transform_, log_file_path_);
      //std::cout << "difference: " << std::endl;
      //printTF2Transform(dtf);
    }

    // publish transform
    geometry_msgs::TransformStamped tf_orbmap_to_map;
    tf_orbmap_to_map.header.frame_id = map_frame_;
    tf_orbmap_to_map.header.stamp = msg.header.stamp;
    tf_orbmap_to_map.child_frame_id = orb_map_frame_;
    tf_orbmap_to_map.transform = tf2::toMsg(tf2_orbmap_to_map);
    // ROS_INFO("msg conversion OK");

    tf_mouth_.sendTransform(tf_orbmap_to_map);
  }
  catch (...)
  {
    ROS_WARN("Some weird error");
    return;
  }

}

int main(int argc, char** argv)
{
#ifdef MOVEL_LICENSE
  MovelLicense ml(6);
  if (!ml.login())
    return 1;
#endif

  ros::init(argc, argv, "map_transform_monitor");
  MapTransformMonitor mtm;
  ros::spin();

#ifdef MOVEL_LICENSE
  ml.logout();
#endif

  return 0;
}
